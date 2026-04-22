// ============================================================
//  DHT11 + Bluetooth UART Transmitter
//  Sends 3 messages per trigger (each ending with \n):
//    1) MPIM-<mpi_id>-#
//    2) MM-<mpi_id>-<SOIL_DATA>-#
//    3) TH-<mpi_id>-<temperature>-<humidity>-#
//  Also supports send_end → transmits END-#\n
//  Clock: 50 MHz | Baud: 115200
// ============================================================

module dht_bluetooth (
    input  wire        clk_50M,
    input  wire        rst_n,
    inout  wire        sensor,
    input  wire        btn,              // Active-LOW trigger for 3-line message
    input  wire [3:0]  deadend_count,    // MPI ID wrap limit
    input  wire [7:0]  soil_data_char,   // Dynamic soil classification char (M or D)
    input  wire [11:0] adc_val,          // Raw 12-bit ADC value (0-4095)
    input  wire        send_end,         // Pulse to trigger END-# message
    input  wire        send_pt_dist,     // Pulse when pre-turn ends by dist
    input  wire        send_pt_us,       // Pulse when pre-turn ends by US
    output reg         tx,
    output reg         busy,

    output wire [3:0]  dbg_state,
    output wire [5:0]  dbg_bit_cnt,
    output wire [39:0] dbg_raw_data,
    output wire        dbg_sensor_in,
    output wire        dbg_oe,
    output wire [23:0] dbg_timer
);

// ============================================================
//  Parameters
// ============================================================
localparam CLK_FREQ  = 50_000_000;
localparam BAUD_RATE = 115_200;
localparam BAUD_DIV  = CLK_FREQ / BAUD_RATE;  // 434
localparam DB_MAX    = 20'd1_000_000;          // 20 ms debounce

// ============================================================
//  DHT11 Instance
// ============================================================
wire [7:0] T_integral, T_decimal, H_integral, H_decimal;
wire       data_valid, checksum_ok;

dht dht_inst (
    .clk_50M     (clk_50M),
    .rst_n       (rst_n),
    .sensor      (sensor),
    .T_integral  (T_integral),
    .T_decimal   (T_decimal),
    .H_integral  (H_integral),
    .H_decimal   (H_decimal),
    .data_valid  (data_valid),
    .checksum_ok (checksum_ok),
    .dbg_state   (dbg_state),
    .dbg_bit_cnt (dbg_bit_cnt),
    .dbg_raw_data(dbg_raw_data),
    .dbg_sensor_in(dbg_sensor_in),
    .dbg_oe      (dbg_oe),
    .dbg_timer   (dbg_timer)
);

// ============================================================
//  Latch latest valid DHT11 reading
// ============================================================
reg [7:0] temp_reg = 0;
reg [7:0] humi_reg = 0;

always @(posedge clk_50M) begin
    if (data_valid && checksum_ok) begin
        temp_reg <= T_integral;
        humi_reg <= H_integral;
    end
end

// ============================================================
//  Button Debounce + Edge Detect (active-low)
// ============================================================
reg [19:0] db_cnt    = 0;
reg        btn_sync0 = 1, btn_sync1 = 1;
reg        btn_prev  = 1;
reg        btn_pulse = 0;

always @(posedge clk_50M) begin
    btn_sync0 <= btn;
    btn_sync1 <= btn_sync0;
    btn_pulse <= 0;

    if (btn_sync1 == btn_prev) begin
        db_cnt <= 0;
    end else begin
        if (db_cnt < DB_MAX)
            db_cnt <= db_cnt + 1;
        else begin
            db_cnt   <= 0;
            btn_prev <= btn_sync1;
            if (btn_sync1 == 1'b0)
                btn_pulse <= 1;
        end
    end
end

// ============================================================
//  MPI ID Counter (1 to deadend_count, wraps)
// ============================================================
reg [3:0] mpi_id = 1;

always @(posedge clk_50M) begin
    if (btn_pulse) begin
        if (mpi_id >= deadend_count)
            mpi_id <= 1;
        else
            mpi_id <= mpi_id + 1;
    end
end

// ============================================================
//  Message Buffer (48 bytes max)
// ============================================================
reg [7:0] msg     [0:47];
reg [5:0] msg_len  = 0;
reg       msg_ready = 0;
reg       sending   = 0;

// ============================================================
//  ASCII digit helpers
// ============================================================
function [7:0] tens_ascii;
    input [7:0] val;
    begin
        tens_ascii = ((val / 10) % 10) + 8'd48;
    end
endfunction

function [7:0] units_ascii;
    input [7:0] val;
    begin
        units_ascii = (val % 10) + 8'd48;
    end
endfunction

// ============================================================
//  Message Builder
//  btn_pulse → 3-line message (MPIM, MM, TH)
//  send_end  → END-#\n
// ============================================================
always @(posedge clk_50M) begin
    msg_ready <= 0;

    if (send_end && !sending) begin
        // ---- END message: "END-#\n" ----
        msg[0] <= "E";
        msg[1] <= "N";
        msg[2] <= "D";
        msg[3] <= "-";
        msg[4] <= "#";
        msg[5] <= 8'h0A;  // \n
        msg_len   <= 6;
        msg_ready <= 1;
    end else if (send_pt_dist && !sending) begin
        msg[0] <= "P"; msg[1] <= "T"; msg[2] <= "-"; msg[3] <= "D"; msg[4] <= "I"; msg[5] <= "S"; msg[6] <= "T"; msg[7] <= 8'h0A;
        msg_len <= 8;
        msg_ready <= 1;

    end else if (send_pt_us && !sending) begin
        msg[0] <= "P"; msg[1] <= "T"; msg[2] <= "-"; msg[3] <= "U"; msg[4] <= "S"; msg[5] <= 8'h0A;
        msg_len <= 6;
        msg_ready <= 1;

    end else if (btn_pulse && !sending) begin
        // ---- Line 1: "MPIM-<id>-#\n" ----
        msg[0]  <= "M";
        msg[1]  <= "P";
        msg[2]  <= "I";
        msg[3]  <= "M";
        msg[4]  <= "-";
        msg[5]  <= mpi_id + 8'd48;
        msg[6]  <= "-";
        msg[7]  <= "#";
        msg[8]  <= 8'h0A;  // \n

        // ---- Line 2: "MM-<id>-<soil>-<adc_val>-#\n" ----
        msg[9]  <= "M";
        msg[10] <= "M";
        msg[11] <= "-";
        msg[12] <= mpi_id + 8'd48;
        msg[13] <= "-";
        msg[14] <= soil_data_char;
        msg[15] <= "-";
        msg[16] <= (adc_val / 1000) % 10 + 8'd48;
        msg[17] <= (adc_val / 100) % 10 + 8'd48;
        msg[18] <= (adc_val / 10) % 10 + 8'd48;
        msg[19] <= (adc_val % 10) + 8'd48;
        msg[20] <= "-";
        msg[21] <= "#";
        msg[22] <= 8'h0A;  // \n

        // ---- Line 3: "TH-<id>-<temp>-<hum>-#\n" ----
        msg[23] <= "T";
        msg[24] <= "H";
        msg[25] <= "-";
        msg[26] <= mpi_id + 8'd48;
        msg[27] <= "-";

        // Temperature digits + dash
        if (temp_reg >= 10) begin
            msg[28] <= tens_ascii(temp_reg);
            msg[29] <= units_ascii(temp_reg);
            msg[30] <= "-";

            // Humidity digits + terminator
            if (humi_reg >= 10) begin
                msg[31] <= tens_ascii(humi_reg);
                msg[32] <= units_ascii(humi_reg);
                msg[33] <= "-";
                msg[34] <= "#";
                msg[35] <= 8'h0A;
                msg_len <= 36;
            end else begin
                msg[31] <= units_ascii(humi_reg);
                msg[32] <= "-";
                msg[33] <= "#";
                msg[34] <= 8'h0A;
                msg_len <= 35;
            end

        end else begin
            msg[28] <= units_ascii(temp_reg);
            msg[29] <= "-";

            if (humi_reg >= 10) begin
                msg[30] <= tens_ascii(humi_reg);
                msg[31] <= units_ascii(humi_reg);
                msg[32] <= "-";
                msg[33] <= "#";
                msg[34] <= 8'h0A;
                msg_len <= 35;
            end else begin
                msg[30] <= units_ascii(humi_reg);
                msg[31] <= "-";
                msg[32] <= "#";
                msg[33] <= 8'h0A;
                msg_len <= 34;
            end
        end

        msg_ready <= 1;
    end
end

// ============================================================
//  UART Transmitter FSM
// ============================================================
localparam U_IDLE  = 3'd0,
           U_START = 3'd1,
           U_DATA  = 3'd2,
           U_STOP  = 3'd3,
           U_NEXT  = 3'd4;

reg [2:0]  u_state  = U_IDLE;
reg [12:0] baud_cnt = 0;
reg [2:0]  bit_idx  = 0;
reg [5:0]  byte_idx = 0;
reg [7:0]  tx_byte  = 0;

always @(posedge clk_50M) begin
    case (u_state)

        U_IDLE: begin
            tx      <= 1'b1;
            busy    <= 1'b0;
            sending <= 1'b0;
            if (msg_ready) begin
                byte_idx <= 0;
                sending  <= 1'b1;
                u_state  <= U_NEXT;
            end
        end

        U_NEXT: begin
            if (byte_idx < msg_len) begin
                tx_byte  <= msg[byte_idx];
                byte_idx <= byte_idx + 1;
                baud_cnt <= 0;
                u_state  <= U_START;
            end else begin
                sending <= 0;
                u_state <= U_IDLE;
            end
        end

        U_START: begin
            tx   <= 1'b0;
            busy <= 1'b1;
            if (baud_cnt < BAUD_DIV - 1)
                baud_cnt <= baud_cnt + 1;
            else begin
                baud_cnt <= 0;
                bit_idx  <= 0;
                u_state  <= U_DATA;
            end
        end

        U_DATA: begin
            tx <= tx_byte[bit_idx];
            if (baud_cnt < BAUD_DIV - 1)
                baud_cnt <= baud_cnt + 1;
            else begin
                baud_cnt <= 0;
                if (bit_idx < 7)
                    bit_idx <= bit_idx + 1;
                else begin
                    bit_idx <= 0;
                    u_state <= U_STOP;
                end
            end
        end

        U_STOP: begin
            tx <= 1'b1;
            if (baud_cnt < BAUD_DIV - 1)
                baud_cnt <= baud_cnt + 1;
            else begin
                baud_cnt <= 0;
                u_state  <= U_NEXT;
            end
        end

        default: u_state <= U_IDLE;
    endcase
end

endmodule