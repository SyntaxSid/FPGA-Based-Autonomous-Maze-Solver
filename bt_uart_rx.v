// ============================================================
//  Bluetooth UART Receiver + Command Parser
//  115200 baud @ 50 MHz
//
//  Receives bytes via UART RX, parses "START-<N>-#" commands.
//  On valid command:
//    cmd_go       — 1-cycle pulse
//    cmd_deadends — digit N (1-9)
// ============================================================

module bt_uart_rx (
    input  wire       clk_50M,
    input  wire       rst_n,
    input  wire       rx,             // UART RX line (idle HIGH)

    output reg        cmd_go,         // Pulse when valid START-N-# received
    output reg [3:0]  cmd_deadends,   // The digit N from the command
    output reg        cmd_stop,       // Pulse when valid E is received
    output reg        cmd_reset       // Pulse when valid R is received
);

// ============================================================
//  UART Parameters
// ============================================================
localparam CLK_FREQ = 50_000_000;
localparam BAUD     = 115_200;
localparam BAUD_DIV = CLK_FREQ / BAUD;       // 434
localparam HALF_DIV = BAUD_DIV / 2;          // 217

// ============================================================
//  RX Synchroniser (2-FF for metastability)
// ============================================================
reg rx_s0 = 1, rx_s1 = 1;
always @(posedge clk_50M) begin
    rx_s0 <= rx;
    rx_s1 <= rx_s0;
end

// ============================================================
//  UART RX State Machine — recovers bytes
// ============================================================
localparam R_IDLE  = 2'd0,
           R_START = 2'd1,
           R_DATA  = 2'd2,
           R_STOP  = 2'd3;

reg [1:0]  r_state  = R_IDLE;
reg [12:0] r_baud   = 0;
reg [2:0]  r_bit    = 0;
reg [7:0]  r_shift  = 0;
reg        rx_valid = 0;
reg [7:0]  rx_byte  = 0;

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        r_state  <= R_IDLE;
        r_baud   <= 0;
        r_bit    <= 0;
        r_shift  <= 0;
        rx_valid <= 0;
        rx_byte  <= 0;
    end else begin
        rx_valid <= 0;

        case (r_state)
        R_IDLE: begin
            if (!rx_s1) begin           // Start bit (falling edge)
                r_baud  <= 0;
                r_state <= R_START;
            end
        end

        R_START: begin                  // Wait half bit period to sample mid-bit
            if (r_baud >= HALF_DIV - 1) begin
                r_baud <= 0;
                r_bit  <= 0;
                r_state <= R_DATA;
            end else
                r_baud <= r_baud + 1;
        end

        R_DATA: begin                   // Sample 8 data bits (LSB first)
            if (r_baud >= BAUD_DIV - 1) begin
                r_baud  <= 0;
                r_shift <= {rx_s1, r_shift[7:1]};
                if (r_bit == 3'd7) begin
                    r_state <= R_STOP;
                end else
                    r_bit <= r_bit + 1;
            end else
                r_baud <= r_baud + 1;
        end

        R_STOP: begin                   // Wait for stop bit
            if (r_baud >= BAUD_DIV - 1) begin
                rx_valid <= 1;
                rx_byte  <= r_shift;
                r_state  <= R_IDLE;
            end else
                r_baud <= r_baud + 1;
        end
        endcase
    end
end

// ============================================================
//  Command Parser FSM
//  Matches: S T A R T - <digit> - #
//  Index:   0 1 2 3 4 5   6     7 8
// ============================================================
localparam [7:0] PAT_S = "S", PAT_T = "T", PAT_A = "A",
                 PAT_R = "R", PAT_DASH = "-", PAT_HASH = "#";

reg [3:0] p_idx = 0;       // Parser position (0-8)
reg [3:0] p_digit = 0;     // Captured digit

always @(posedge clk_50M or negedge rst_n) begin
    if (!rst_n) begin
        p_idx        <= 0;
        p_digit      <= 0;
        cmd_go       <= 0;
        cmd_deadends <= 4'd3;   // Default: 3 dead-ends
        cmd_stop     <= 0;
        cmd_reset    <= 0;
    end else begin
        cmd_go <= 0;
        cmd_stop <= 0;
        cmd_reset <= 0;

        if (rx_valid) begin
            if (rx_byte == "E") cmd_stop <= 1;
            if (rx_byte == "R" && p_idx != 4'd3) cmd_reset <= 1;

            case (p_idx)
            4'd0: p_idx <= (rx_byte == PAT_S)    ? 4'd1 : 4'd0;
            4'd1: p_idx <= (rx_byte == PAT_T)    ? 4'd2 : 4'd0;
            4'd2: p_idx <= (rx_byte == PAT_A)    ? 4'd3 : 4'd0;
            4'd3: p_idx <= (rx_byte == PAT_R)    ? 4'd4 : 4'd0;
            4'd4: p_idx <= (rx_byte == PAT_T)    ? 4'd5 : 4'd0;
            4'd5: p_idx <= (rx_byte == PAT_DASH) ? 4'd6 : 4'd0;
            4'd6: begin
                // Expect ASCII digit '1'-'9'
                if (rx_byte >= "1" && rx_byte <= "9") begin
                    p_digit <= rx_byte - "0";
                    p_idx   <= 4'd7;
                end else
                    p_idx <= 4'd0;
            end
            4'd7: p_idx <= (rx_byte == PAT_DASH) ? 4'd8 : 4'd0;
            4'd8: begin
                if (rx_byte == PAT_HASH) begin
                    // Valid command!
                    cmd_go       <= 1;
                    cmd_deadends <= p_digit;
                end
                p_idx <= 4'd0;  // Reset parser for next command
            end
            default: p_idx <= 4'd0;
            endcase
        end
    end
end

endmodule