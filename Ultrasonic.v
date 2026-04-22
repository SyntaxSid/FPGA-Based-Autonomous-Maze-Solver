// =============================================================================
//  Ultrasonic.v  -  3x HC-SR04 on MAX 10 DE10-Lite (50 MHz)
//
//  Display layout:
//    HEX5 HEX4  =  Left   (00-60 cm)
//    HEX3 HEX2  =  Front  (00-60 cm)
//    HEX1 HEX0  =  Right  (00-60 cm)
//
//  JP1 wiring:
//    PIN_V10 = trig_left   PIN_W9 = echo_left
//    PIN_W10 = trig_front  PIN_V8 = echo_front
//    PIN_V9  = trig_right  PIN_W8 = echo_right
//
//  Round-robin sequencing: Left -> Front -> Right -> gap -> repeat
//  Only ONE sensor fires at a time to prevent acoustic crosstalk.
// =============================================================================

module Ultrasonic(
    input  wire clk,

    input  wire echo_left,
    input  wire echo_front,
    input  wire echo_right,

    output reg  trig_left,
    output reg  trig_front,
    output reg  trig_right,

    // Distance outputs for use by other modules (e.g. FSM)
    output reg [6:0] dist_left_cm,
    output reg [6:0] dist_front_cm,
    output reg [6:0] dist_right_cm
);

// ---------------------------------------------------------------------------
// Timing constants @ 50 MHz
// ---------------------------------------------------------------------------
localparam TRIG_CYCLES   = 32'd500;          // 10 us
localparam ECHO_TIMEOUT  = 32'd200_000;      // 4 ms (~68 cm max wait)
localparam GAP_CYCLES    = 32'd3_000_000;    // 60 ms gap between full cycles

// ---------------------------------------------------------------------------
// Echo synchronisers (double-flop, metastability protection)
// ---------------------------------------------------------------------------
reg [1:0] el_ff, ef_ff, er_ff;
always @(posedge clk) begin
    el_ff <= {el_ff[0], echo_left};
    ef_ff <= {ef_ff[0], echo_front};
    er_ff <= {er_ff[0], echo_right};
end
wire el = el_ff[1];
wire ef = ef_ff[1];
wire er = er_ff[1];

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
localparam S_TRIG_L = 4'd0;
localparam S_RISE_L = 4'd1;
localparam S_FALL_L = 4'd2;
localparam S_TRIG_F = 4'd3;
localparam S_RISE_F = 4'd4;
localparam S_FALL_F = 4'd5;
localparam S_TRIG_R = 4'd6;
localparam S_RISE_R = 4'd7;
localparam S_FALL_R = 4'd8;
localparam S_GAP    = 4'd9;

reg [3:0]  state;
reg [31:0] cnt;
reg [31:0] pw;        // pulse-width counter

// Captured raw pulse widths
reg [31:0] raw_l, raw_f, raw_r;
reg        done_l, done_f, done_r;

always @(posedge clk) begin
    // Default: triggers low, done flags clear each cycle
    trig_left  <= 1'b0;
    trig_front <= 1'b0;
    trig_right <= 1'b0;
    done_l     <= 1'b0;
    done_f     <= 1'b0;
    done_r     <= 1'b0;

    cnt <= cnt + 1;

    case (state)

        // ── LEFT trigger pulse ──────────────────────────────────────────
        S_TRIG_L: begin
            trig_left <= 1'b1;
            if (cnt >= TRIG_CYCLES) begin
                cnt   <= 32'd0;
                pw    <= 32'd0;
                state <= S_RISE_L;
            end
        end

        // Wait for echo HIGH
        S_RISE_L: begin
            if (el) begin
                pw    <= 32'd0;
                cnt   <= 32'd0;
                state <= S_FALL_L;
            end else if (cnt >= ECHO_TIMEOUT) begin
                raw_l  <= 32'd0;      // no echo → show 99 cm
                done_l <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_TRIG_F;
            end
        end

        // Measure echo pulse until LOW
        S_FALL_L: begin
            pw <= pw + 1;
            if (!el) begin
                raw_l  <= pw;
                done_l <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_TRIG_F;
            end else if (pw >= ECHO_TIMEOUT) begin
                raw_l  <= ECHO_TIMEOUT;
                done_l <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_TRIG_F;
            end
        end

        // ── FRONT trigger pulse ─────────────────────────────────────────
        S_TRIG_F: begin
            trig_front <= 1'b1;
            if (cnt >= TRIG_CYCLES) begin
                cnt   <= 32'd0;
                pw    <= 32'd0;
                state <= S_RISE_F;
            end
        end

        S_RISE_F: begin
            if (ef) begin
                pw    <= 32'd0;
                cnt   <= 32'd0;
                state <= S_FALL_F;
            end else if (cnt >= ECHO_TIMEOUT) begin
                raw_f  <= 32'd0;
                done_f <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_TRIG_R;
            end
        end

        S_FALL_F: begin
            pw <= pw + 1;
            if (!ef) begin
                raw_f  <= pw;
                done_f <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_TRIG_R;
            end else if (pw >= ECHO_TIMEOUT) begin
                raw_f  <= ECHO_TIMEOUT;
                done_f <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_TRIG_R;
            end
        end

        // ── RIGHT trigger pulse ─────────────────────────────────────────
        S_TRIG_R: begin
            trig_right <= 1'b1;
            if (cnt >= TRIG_CYCLES) begin
                cnt   <= 32'd0;
                pw    <= 32'd0;
                state <= S_RISE_R;
            end
        end

        S_RISE_R: begin
            if (er) begin
                pw    <= 32'd0;
                cnt   <= 32'd0;
                state <= S_FALL_R;
            end else if (cnt >= ECHO_TIMEOUT) begin
                raw_r  <= 32'd0;
                done_r <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_GAP;
            end
        end

        S_FALL_R: begin
            pw <= pw + 1;
            if (!er) begin
                raw_r  <= pw;
                done_r <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_GAP;
            end else if (pw >= ECHO_TIMEOUT) begin
                raw_r  <= ECHO_TIMEOUT;
                done_r <= 1'b1;
                cnt    <= 32'd0;
                state  <= S_GAP;
            end
        end

        // ── Inter-cycle gap ─────────────────────────────────────────────
        S_GAP: begin
            if (cnt >= GAP_CYCLES) begin
                cnt   <= 32'd0;
                state <= S_TRIG_L;
            end
        end

        default: begin
            cnt   <= 32'd0;
            state <= S_TRIG_L;
        end
    endcase
end

// ---------------------------------------------------------------------------
// Convert raw pulse width to cm
//   cm = cycles / 2941  (50 MHz, 340 m/s)
//   Cap at 60.  raw == 0 (no echo) → 60.
// ---------------------------------------------------------------------------
function [6:0] to_cm;
    input [31:0] raw;
    reg [31:0] d;
    begin
        if (raw == 0)
            to_cm = 7'd60;
        else begin
            d = raw / 32'd2941;
            to_cm = (d > 32'd60) ? 7'd60 : d[6:0];
        end
    end
endfunction

// ---------------------------------------------------------------------------
// Median Filter Function (Size 3)
// Eliminates 1-sample spikes and reduces jitter
// ---------------------------------------------------------------------------
function [6:0] median3;
    input [6:0] a, b, c;
    begin
        if ((a >= b && a <= c) || (a <= b && a >= c))
            median3 = a;
        else if ((b >= a && b <= c) || (b <= a && b >= c))
            median3 = b;
        else
            median3 = c;
    end
endfunction

// ---------------------------------------------------------------------------
// Distance registers — only update when done pulse fires
// ---------------------------------------------------------------------------
reg [6:0] l_hist1, l_hist2;
reg [6:0] f_hist1, f_hist2;
reg [6:0] r_hist1, r_hist2;

always @(posedge clk) begin
    if (done_l) begin
        l_hist2 <= l_hist1;
        l_hist1 <= to_cm(raw_l);
        dist_left_cm  <= median3(to_cm(raw_l), l_hist1, l_hist2);
    end
    if (done_f) begin
        f_hist2 <= f_hist1;
        f_hist1 <= to_cm(raw_f);
        dist_front_cm <= median3(to_cm(raw_f), f_hist1, f_hist2);
    end
    if (done_r) begin
        r_hist2 <= r_hist1;
        r_hist1 <= to_cm(raw_r);
        dist_right_cm <= median3(to_cm(raw_r), r_hist1, r_hist2);
    end
end

endmodule