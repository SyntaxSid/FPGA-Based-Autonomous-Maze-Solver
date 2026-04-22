// =============================================================================
// Outer_Loop_PD.v (Ultrasonic PD Controller)
// =============================================================================
// This module implements the "Outer Loop" of the cascaded control system.
// Instead of directly driving the motors, this module looks at the ultrasonic
// sensors and calculates a DESIRED HEADING ADJUSTMENT.
//
// Inner Loop (Heading_Controller.v) will then do the actual motor driving
// using encoder data to hit this heading perfectly smoothly.
// =============================================================================

module Outer_Loop_PD #(
    parameter signed [15:0] TARGET_LEFT_CM = 6,     // Desired distance from the left wall
    parameter signed [15:0] TARGET_RIGHT_CM = 6,    // Desired distance from the right wall
    parameter signed [15:0] MAX_ADJUST_DEG = 45,     // Maximum degrees we can ask the bot to veer
    parameter signed [15:0] Kp = 5,
    parameter signed [15:0] Kd = 1
)(
    input  wire        clk_50,
    input  wire        rst_n,
    
    // Inputs from Ultrasonic sensors (0-99 cm)
    // These update at ~20 Hz
    input  wire [6:0]  dist_left_cm,
    input  wire [6:0]  dist_right_cm,
    input  wire [6:0]  dist_front_cm,

    // Outputs to the FSM / Inner Loop
    // Positive = steer right (e.g. +3 degrees)
    // Negative = steer left  (e.g. -2 degrees)
    output reg signed [15:0] heading_adjust,
    
    // Let the FSM know if we are getting too close to a wall in front
    output reg              wall_blocking_front
);

    // =========================================================================
    // INTERNAL REGISTERS
    // =========================================================================

    // Detect if sensors are sending valid wall readings (e.g. < 40cm)
    wire left_valid = (dist_left_cm > 0 && dist_left_cm < 40);
    wire right_valid = (dist_right_cm > 0 && dist_right_cm < 40);

    // Detect saturated readings (too close → sonar wraps to max value)
    wire left_saturated  = (dist_left_cm >= 7'd50);
    wire right_saturated = (dist_right_cm >= 7'd50);

    reg signed [15:0] last_err;
    reg signed [15:0] err;
    reg signed [15:0] d_err;
    reg signed [15:0] total_correction;

    // Timer to sample at ~20Hz (50MHz / 2,500,000)
    // We must sample at a fixed slow rate so the derivative term works correctly.
    // If we sample at 50MHz, the error only changes every 50ms, causing d_err to be 
    // a 1-clock-cycle spike and then zero 99.999% of the time!
    localparam SAMPLE_COUNT_MAX = 32'd2_500_000;
    reg [31:0] sample_timer;

    // =========================================================================
    // PD CONTROL LOGIC
    // =========================================================================

    always @(posedge clk_50) begin
        if (!rst_n) begin
            heading_adjust <= 0;
            wall_blocking_front <= 0;
            
            last_err <= 0;
            err <= 0;
            d_err <= 0;
            sample_timer <= 0;
            total_correction <= 0;
        end else begin
            
            // 1. Front Wall Detection
            if (dist_front_cm > 0 && dist_front_cm < 10) 
                wall_blocking_front <= 1;
            else
                wall_blocking_front <= 0;

            // 2. Sampling Timer
            if (sample_timer < SAMPLE_COUNT_MAX) begin
                sample_timer <= sample_timer + 1;
            end else begin
                sample_timer <= 0;
                
                // --- A. Calculate Error ---
                // Saturated sensor = too close to that wall → steer HARD away
                if (left_saturated && !right_saturated) begin
                    // Left sonar maxed out → too close to left wall → steer right
                    err <= MAX_ADJUST_DEG;
                end else if (right_saturated && !left_saturated) begin
                    // Right sonar maxed out → too close to right wall → steer left
                    err <= -MAX_ADJUST_DEG;
                end else if (left_saturated && right_saturated) begin
                    // Both saturated — keep straight, no useful data
                    err <= 0;
                end
                // Normal operation
                else if (left_valid && right_valid) begin
                    // Both walls valid: aim for center between them
                    err <= $signed({1'b0, dist_right_cm}) - $signed({1'b0, dist_left_cm}); 
                end else if (left_valid) begin
                    // Only left wall visible
                    err <= TARGET_LEFT_CM - $signed({1'b0, dist_left_cm});
                end else if (right_valid) begin
                    // Only right wall visible
                    err <= $signed({1'b0, dist_right_cm}) - TARGET_RIGHT_CM;
                end else begin
                    // No walls, keep straight
                    err <= 0;
                end
                
                // --- B. Calculate Derivative ---
                d_err <= err - last_err;
                last_err <= err;
                
                // --- C. PD Equation ---
                total_correction <= (err * Kp) + (d_err * Kd);
            end
            
            // 3. Cap and Assign to heading_adjust
            // We do this every clock cycle but total_correction only updates at 20Hz.
            if (total_correction > MAX_ADJUST_DEG)
                heading_adjust <= MAX_ADJUST_DEG;
            else if (total_correction < -$signed(MAX_ADJUST_DEG))
                heading_adjust <= -$signed(MAX_ADJUST_DEG);
            else
                heading_adjust <= total_correction;

        end
    end

endmodule