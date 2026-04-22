// --- TOP MODULE: Manual Switch Control ---
module Motor_controller(
    input  wire        CLOCK_50,      // 50 MHz clock
    input  wire [2:0]  SW,            // SW[0]: Enable, SW[1]: Dir, SW[2]: Speed
    output wire        MD_ENA,
    output wire        MD_IN1,
    output wire        MD_IN2,
    output wire        MD_IN3,
    output wire        MD_IN4,
    output wire        MD_ENB
);
    wire [3:0] current_speed;
    wire [1:0] current_dir;

    // --- Switch Logic ---
    // SW[0] (Enable): If 0, speed is 0. If 1, speed is set by SW[2].
    // SW[1] (Direction): 0 = Forward (2'b10), 1 = Reverse (2'b01).
    // SW[2] (Speed Level): 0 = Low (8/15 duty), 1 = High (15/15 duty).

    assign current_speed = (SW[0] == 1'b0) ? 4'd0 :       // Power Off
                           (SW[2] == 1'b1) ? 4'd15 :      // Full Speed
                           4'd8;                          // Medium Speed

    assign current_dir   = (SW[1] == 1'b0) ? 2'b10 :      // Forward
                           2'b01;                         // Reverse

    // Instantiate the motor driver
    motor_controller mc (
        .CLOCK_50(CLOCK_50),
        .speed_L(current_speed), 
        .speed_R(current_speed),
        .dir_L(current_dir), 
        .dir_R(current_dir),
        .ENA(MD_ENA), 
        .IN1(MD_IN1), 
        .IN2(MD_IN2),
        .IN3(MD_IN3), 
        .IN4(MD_IN4), 
        .ENB(MD_ENB)
    );
endmodule

// --- MOTOR CONTROLLER: Routing and Logic ---
module motor_controller(
    input  wire       CLOCK_50,
    input  wire [3:0] speed_L, speed_R,
    input  wire [1:0] dir_L, dir_R,
    output wire       ENA, ENB, IN1, IN2, IN3, IN4
);
    wire clk_safe;

    // Frequency scaling to get the base PWM clock
    frequency_scaling fs (
        .CLOCK_50(CLOCK_50), 
        .clk_out(clk_safe)
    );
    
    // Left and Right PWM Generators
    pwm_generator p_L (
        .clk_in(clk_safe), 
        .duty_cycle(speed_L), 
        .pwm_signal(ENA)
    );
    pwm_generator p_R (
        .clk_in(clk_safe), 
        .duty_cycle(speed_R), 
        .pwm_signal(ENB)
    );

    // Direction Pin Mapping
    assign IN1 = dir_L[1]; 
    assign IN2 = dir_L[0];
    assign IN3 = dir_R[1]; 
    assign IN4 = dir_R[0];
endmodule

// --- FREQUENCY SCALING: Targeted ~1.25 kHz Result ---
module frequency_scaling (
    input  CLOCK_50,
    output reg clk_out
);
    reg [11:0] counter = 12'd0;
    initial clk_out = 0;

    always @(posedge CLOCK_50) begin
        if (counter < 3124) begin 
            counter <= counter + 1;
        end else begin
            counter <= 0;
            clk_out <= ~clk_out; // Toggles at ~8kHz
        end
    end
endmodule

// --- PWM GENERATOR ---
module pwm_generator(
    input  clk_in,
    input  [3:0] duty_cycle,
    output reg pwm_signal
);
    reg [3:0] counter = 4'd0;
    
    always @(posedge clk_in) begin
        counter <= counter + 1;
        // Total 16 steps (0-15). Final Freq = 8kHz / 16 = 500Hz.
        pwm_signal <= (counter < duty_cycle) ? 1'b1 : 1'b0;
    end
endmodule