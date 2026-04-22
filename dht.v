// ============================================================
//  DHT11 Sensor Controller
//  Clock: 50 MHz  |  Timing per datasheet spec
//  SignalTap Logic Analyzer debug ports included
// ============================================================

module dht (
    input  wire        clk_50M,
    input  wire        rst_n,         // Active-low reset

    inout  wire        sensor,        // Bidirectional DHT11 data pin

    // --- Primary Outputs ---
    output reg  [7:0]  T_integral,    // Temperature integer part
    output reg  [7:0]  T_decimal,     // Temperature decimal part
    output reg  [7:0]  H_integral,    // Humidity integer part
    output reg  [7:0]  H_decimal,     // Humidity decimal part
    output reg         data_valid,    // Pulses HIGH 1 clk when new data ready
    output reg         checksum_ok,   // HIGH = last read passed checksum

    // --- SignalTap Debug Ports ---
    output wire [3:0]  dbg_state,     // FSM state number
    output wire [5:0]  dbg_bit_cnt,   // Current bit index (39 down to 0)
    output wire [39:0] dbg_raw_data,  // 40-bit raw shift register
    output wire        dbg_sensor_in, // Live sensor line level
    output wire        dbg_oe,        // 1 = host driving line LOW
    output wire [23:0] dbg_timer      // Microsecond timer value
);

// ============================================================
//  Clock & Timing Parameters
// ============================================================
localparam CLK_FREQ      = 50_000_000;
localparam US_TICK       = CLK_FREQ / 1_000_000; // 50 counts = 1 us

//  ** EXACT values from datasheet **
localparam T_START_LOW   = 18_000;  // Host pulls LOW >= 18 ms  (18000 us)
localparam T_START_HIGH  = 40;      // Host releases HIGH for 40 us
localparam T_RESP_WAIT   = 100;     // Timeout waiting for sensor LOW response
localparam T_TIMEOUT     = 200;     // Per-bit watchdog (us)
localparam T_BIT_THRESH  = 26;      // <= 26 us HIGH = bit '0'; > 26 us = bit '1'
localparam T_INTER_READ  = 2_000_000; // 2 seconds between reads

// ============================================================
//  FSM State Encoding
// ============================================================
localparam [3:0]
    S_IDLE       = 4'd0,
    S_START_LOW  = 4'd1,
    S_START_HIGH = 4'd2,
    S_WAIT_RESP  = 4'd3,
    S_RESP_LOW   = 4'd4,
    S_RESP_HIGH  = 4'd5,
    S_BIT_LOW    = 4'd6,
    S_BIT_HIGH   = 4'd7,
    S_CHECKSUM   = 4'd8,
    S_DONE       = 4'd9,
    S_ERROR      = 4'd10;

// ============================================================
//  Internal Registers
// ============================================================
reg [3:0]  state;
reg [23:0] timer;
reg [9:0]  clk_div;
reg        tick_us;

reg [39:0] shift_reg;
reg [5:0]  bit_cnt;
reg        oe;

// ============================================================
//  Tri-state Sensor Bus
// ============================================================
assign sensor    = oe ? 1'b0 : 1'bz;
wire   sensor_in = sensor;

// ============================================================
//  SignalTap Debug Assignments
// ============================================================
assign dbg_state     = state;
assign dbg_bit_cnt   = bit_cnt;
assign dbg_raw_data  = shift_reg;
assign dbg_sensor_in = sensor_in;
assign dbg_oe        = oe;
assign dbg_timer     = timer;

// ============================================================
//  1 us Tick Generator
// ============================================================
always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        clk_div <= 0;
        tick_us <= 0;
    end else begin
        if (clk_div == US_TICK - 1) begin
            clk_div <= 0;
            tick_us <= 1;
        end else begin
            clk_div <= clk_div + 1;
            tick_us <= 0;
        end
    end
end

// ============================================================
//  Main FSM
// ============================================================
always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        state       <= S_IDLE;
        timer       <= 0;
        oe          <= 0;
        bit_cnt     <= 39;
        shift_reg   <= 0;
        data_valid  <= 0;
        checksum_ok <= 0;
        T_integral  <= 0;
        T_decimal   <= 0;
        H_integral  <= 0;
        H_decimal   <= 0;
    end else begin
        data_valid <= 0;

        case (state)

        S_IDLE: begin
            oe    <= 0;
            timer <= 0;
            state <= S_START_LOW;
        end

        S_START_LOW: begin
            oe <= 1;
            if (tick_us) begin
                if (timer >= T_START_LOW - 1) begin
                    timer <= 0;
                    state <= S_START_HIGH;
                end else
                    timer <= timer + 1;
            end
        end

        S_START_HIGH: begin
            oe <= 0;
            if (tick_us) begin
                if (timer >= T_START_HIGH - 1) begin
                    timer <= 0;
                    state <= S_WAIT_RESP;
                end else
                    timer <= timer + 1;
            end
        end

        S_WAIT_RESP: begin
            if (tick_us) begin
                if (!sensor_in) begin
                    timer <= 0;
                    state <= S_RESP_LOW;
                end else if (timer >= T_RESP_WAIT - 1) begin
                    state <= S_ERROR;
                end else
                    timer <= timer + 1;
            end
        end

        S_RESP_LOW: begin
            if (tick_us) begin
                if (sensor_in) begin
                    timer <= 0;
                    state <= S_RESP_HIGH;
                end else if (timer >= T_TIMEOUT - 1) begin
                    state <= S_ERROR;
                end else
                    timer <= timer + 1;
            end
        end

        S_RESP_HIGH: begin
            if (tick_us) begin
                if (!sensor_in) begin
                    timer   <= 0;
                    bit_cnt <= 39;
                    state   <= S_BIT_LOW;
                end else if (timer >= T_TIMEOUT - 1) begin
                    state <= S_ERROR;
                end else
                    timer <= timer + 1;
            end
        end

        S_BIT_LOW: begin
            if (tick_us) begin
                if (sensor_in) begin
                    timer <= 0;
                    state <= S_BIT_HIGH;
                end else if (timer >= T_TIMEOUT - 1) begin
                    state <= S_ERROR;
                end else
                    timer <= timer + 1;
            end
        end

        S_BIT_HIGH: begin
            if (tick_us) begin
                if (!sensor_in) begin
                    shift_reg <= {shift_reg[38:0],
                                  (timer > T_BIT_THRESH) ? 1'b1 : 1'b0};
                    timer <= 0;

                    if (bit_cnt == 0)
                        state <= S_CHECKSUM;
                    else begin
                        bit_cnt <= bit_cnt - 1;
                        state   <= S_BIT_LOW;
                    end
                end else if (timer >= T_TIMEOUT - 1) begin
                    state <= S_ERROR;
                end else
                    timer <= timer + 1;
            end
        end

        S_CHECKSUM: begin
            if (shift_reg[7:0] == shift_reg[39:32]
                                 + shift_reg[31:24]
                                 + shift_reg[23:16]
                                 + shift_reg[15:8]) begin
                H_integral  <= shift_reg[39:32];
                H_decimal   <= shift_reg[31:24];
                T_integral  <= shift_reg[23:16];
                T_decimal   <= shift_reg[15:8];
                checksum_ok <= 1;
                data_valid  <= 1;
            end else begin
                checksum_ok <= 0;
            end
            timer <= 0;
            state <= S_DONE;
        end

        S_DONE: begin
            if (tick_us) begin
                if (timer >= T_INTER_READ - 1) begin
                    timer <= 0;
                    state <= S_IDLE;
                end else
                    timer <= timer + 1;
            end
        end

        S_ERROR: begin
            oe <= 0;
            if (tick_us) begin
                if (timer >= 24'd1_000_000 - 1) begin
                    timer <= 0;
                    state <= S_IDLE;
                end else
                    timer <= timer + 1;
            end
        end

        default: state <= S_IDLE;

        endcase
    end
end

endmodule
