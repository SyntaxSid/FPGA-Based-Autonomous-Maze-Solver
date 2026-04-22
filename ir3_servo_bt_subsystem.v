// ============================================================
//  INDEPENDENT IR3 + SERVO + DHT/BT SUBSYSTEM
//  DE10-Lite | 50 MHz Clock
//
//  Sequence (triggered ONLY by ext_start from test_fsm):
//    1) Servo immediately moves to 90 deg (POS_90) for 2 seconds
//    2) After 1 second, DHT11 reading is sent via Bluetooth
//    3) After 2 seconds, servo returns to neutral (POS_NORM)
//
//  Re-trigger is BLOCKED while a cycle is in progress.
//  Exposes ir3_object_detected for external polling.
//  ext_start: pulse HIGH for 1 cycle to trigger sequence externally.
//  ext_done:  pulses HIGH for 1 cycle when sequence completes.
// ============================================================

module ir3_servo_bt_subsystem (
    input  wire        CLOCK_50,   // 50 MHz system clock
    input  wire        rst_n,           // Active-low reset

    // IR Sensor 3
    input  wire        ir_in_3,         // MH Flying Fish OUT pin (active LOW)

    // Servo
    output wire        servo_pwm_3,     // PWM signal to servo

    // DHT11
    inout  wire        dht_sensor_3,    // Bidirectional DHT11 data pin

    // Bluetooth UART
    input  wire        bt_rx_3,         // UART RX from Bluetooth module
    output wire        bt_tx_3,         // UART TX to Bluetooth module

    // Bluetooth START command outputs (to test_fsm)
    output wire        bt_go,           // Pulse when valid START-N-# received
    output wire [3:0]  deadend_count,   // Digit N from START command
    output wire        bt_stop,         // Pulse when E is received
    output wire        bt_reset,        // Pulse when R is received

    // External handshake (from test_fsm)
    input  wire        ext_start,       // Pulse HIGH to trigger sequence
    input  wire        send_end,        // Pulse to trigger END-# message
    input  wire        send_pt_dist,
    input  wire        send_pt_us,
    input  wire [7:0]  soil_data_char,  // Input dynamic soil character from ADC
    input  wire [11:0] adc_val,         // Raw 12-bit ADC value
    output reg         ext_done,        // Pulses HIGH when sequence finishes
    output wire        ir3_object_detected  // Live IR3 detection status
);

// ============================================================
//  Internal wires for debug signals (submodule still needs them)
// ============================================================
wire [3:0]  dbg_state;
wire [5:0]  dbg_bit_cnt;
wire [39:0] dbg_raw_data;
wire        dbg_sensor_in;
wire        dbg_oe;
wire [23:0] dbg_timer;

// ============================================================
//  Timing Constants  (50 MHz clock)
// ============================================================
localparam CLK_FREQ      = 50_000_000;
localparam ONE_SEC       = 32'd50_000_000;    // 1 second
localparam TWO_SEC       = 32'd100_000_000;   // 2 seconds

// Must exceed dht_bluetooth debounce window (DB_MAX = 1_000_000 = 20ms)
// We hold btn LOW for 25ms to guarantee the debounce counter saturates
localparam BTN_HOLD      = 32'd1_250_000;     // 25 ms @ 50 MHz

// Servo PWM parameters
localparam PERIOD        = 32'd1_000_000;     // 20 ms PWM period
localparam POS_NORM      = 32'd75_000;        // 1.5 ms neutral
localparam POS_90        = 32'd25_000;        // 1.0 ms -> 90 deg

// ============================================================
//  IR Sensor 3 Instance
// ============================================================
wire ir3_detection_pulse;

ir_sensor_flying_fish #(
    .CLK_FREQ    (CLK_FREQ),
    .DEBOUNCE_MS (5)
) ir3_inst (
    .clk              (CLOCK_50),
    .rst_n            (rst_n),
    .ir_in            (ir_in_3),
    .object_detected  (ir3_object_detected),
    .detection_pulse  (ir3_detection_pulse)
);

// ============================================================
//  Bluetooth UART RX + Command Parser
// ============================================================
bt_uart_rx bt_rx_inst (
    .clk_50M      (CLOCK_50),
    .rst_n        (rst_n),
    .rx           (bt_rx_3),
    .cmd_go       (bt_go),
    .cmd_deadends (deadend_count),
    .cmd_stop     (bt_stop),
    .cmd_reset    (bt_reset)
);

// ============================================================
//  DHT11 + Bluetooth Instance
// ============================================================
reg  bt_btn_r;       // Registered btn level driven into submodule
wire bt_tx_wire;
wire bt_busy;

dht_bluetooth bt_inst (
    .clk_50M       (CLOCK_50),
    .rst_n         (rst_n),
    .sensor        (dht_sensor_3),
    .btn           (bt_btn_r),    // We drive this directly (active-LOW)
    .deadend_count (deadend_count),
    .soil_data_char(soil_data_char),
    .adc_val       (adc_val),
    .send_end      (send_end),
    .send_pt_dist  (send_pt_dist),
    .send_pt_us    (send_pt_us),
    .tx            (bt_tx_wire),
    .busy          (bt_busy),
    .dbg_state     (dbg_state),
    .dbg_bit_cnt   (dbg_bit_cnt),
    .dbg_raw_data  (dbg_raw_data),
    .dbg_sensor_in (dbg_sensor_in),
    .dbg_oe        (dbg_oe),
    .dbg_timer     (dbg_timer)
);

assign bt_tx_3 = bt_tx_wire;

// ============================================================
//  Servo PWM Generator
// ============================================================
reg [31:0] pwm_counter = 0;
reg [31:0] duty_cycle  = POS_NORM;
reg        servo_out_r = 0;

always @(posedge CLOCK_50) begin
    if (!rst_n) begin
        pwm_counter <= 0;
        servo_out_r <= 0;
    end else begin
        if (pwm_counter < PERIOD - 1)
            pwm_counter <= pwm_counter + 1;
        else
            pwm_counter <= 0;

        servo_out_r <= (pwm_counter < duty_cycle) ? 1'b1 : 1'b0;
    end
end

assign servo_pwm_3 = servo_out_r;

// ============================================================
//  Top-Level Sequencing FSM
//
//  Triggered ONLY by ext_start from test_fsm.
//
//  States:
//    ST_IDLE      -> waiting for ext_start
//    ST_SERVO_ON  -> servo at 90 deg, counting to 1 s
//    ST_BT_TRIG   -> pull btn LOW and hold for 25 ms
//    ST_BT_REL    -> release btn HIGH for 25 ms
//    ST_WAIT_END  -> keep servo bent until 2 s total elapsed
//    ST_RESET     -> return servo to neutral, pulse ext_done
// ============================================================
localparam [2:0]
    ST_IDLE     = 3'd0,
    ST_SERVO_ON = 3'd1,
    ST_BT_TRIG  = 3'd2,
    ST_BT_REL   = 3'd3,
    ST_WAIT_END = 3'd4,
    ST_RESET    = 3'd5;

reg [2:0]  seq_state = ST_IDLE;
reg [31:0] seq_timer = 0;
reg [31:0] btn_timer = 0;

always @(posedge CLOCK_50 or negedge rst_n) begin
    if (!rst_n) begin
        seq_state  <= ST_IDLE;
        seq_timer  <= 0;
        btn_timer  <= 0;
        duty_cycle <= POS_NORM;
        bt_btn_r   <= 1'b1;
        ext_done   <= 1'b0;
    end else begin
        ext_done <= 1'b0;  // Default: deassert each clock

        case (seq_state)

        // -----------------------------------------------
        // IDLE: servo neutral, btn released
        // Only triggered by ext_start from test_fsm
        // -----------------------------------------------
        ST_IDLE: begin
            duty_cycle <= POS_NORM;
            seq_timer  <= 0;
            btn_timer  <= 0;
            bt_btn_r   <= 1'b1;

            if (ext_start) begin
                duty_cycle <= POS_90;
                seq_state  <= ST_SERVO_ON;
            end
        end

        // -----------------------------------------------
        // SERVO_ON: wait 1 second then trigger BT
        // -----------------------------------------------
        ST_SERVO_ON: begin
            seq_timer <= seq_timer + 1;
            if (seq_timer >= ONE_SEC - 1) begin
                btn_timer <= 0;
                seq_state <= ST_BT_TRIG;
            end
        end

        // -----------------------------------------------
        // BT_TRIG: hold btn LOW for 25 ms
        // -----------------------------------------------
        ST_BT_TRIG: begin
            bt_btn_r  <= 1'b0;
            btn_timer <= btn_timer + 1;
            if (btn_timer >= BTN_HOLD - 1) begin
                btn_timer <= 0;
                seq_state <= ST_BT_REL;
            end
        end

        // -----------------------------------------------
        // BT_REL: release btn HIGH for 25 ms
        // -----------------------------------------------
        ST_BT_REL: begin
            bt_btn_r  <= 1'b1;
            btn_timer <= btn_timer + 1;
            if (btn_timer >= BTN_HOLD - 1) begin
                btn_timer <= 0;
                seq_state <= ST_WAIT_END;
            end
        end

        // -----------------------------------------------
        // WAIT_END: keep servo bent; wait until 2 s total
        // -----------------------------------------------
        ST_WAIT_END: begin
            seq_timer <= seq_timer + 1;
            if (seq_timer >= TWO_SEC - 1)
                seq_state <= ST_RESET;
        end

        // -----------------------------------------------
        // RESET: servo back to neutral, pulse ext_done
        // -----------------------------------------------
        ST_RESET: begin
            duty_cycle <= POS_NORM;
            seq_timer  <= 0;
            btn_timer  <= 0;
            bt_btn_r   <= 1'b1;
            ext_done   <= 1'b1;   // Signal completion
            seq_state  <= ST_IDLE;
        end

        default: seq_state <= ST_IDLE;

        endcase
    end
end

endmodule