module Heading_Controller #(
    parameter FIXED_BIAS    = 0,
    parameter signed KP_HEADING = 1,
    parameter signed KP_MOTOR   = 1,
    parameter signed KI_MOTOR   = 1
)(
    input  wire        clk_50,
    input  wire        rst_n,
    input  wire        enable,
    input  wire [8:0]  target_heading,
    input  wire [8:0]  current_heading,
    input  wire [3:0]  base_speed_L,
    input  wire [3:0]  base_speed_R,
    input  wire signed [15:0] actual_speed_R,
    output wire [3:0]  out_steer,
    output reg  [3:0]  out_speed_L,
    output reg  [3:0]  out_speed_R
);

    // =========================================================================
    // Heading error — shortest path, 0-359 wrapped
    // =========================================================================
    wire [8:0] err_raw = (target_heading >= current_heading)
                       ? (target_heading - current_heading)
                       : (9'd360 - current_heading + target_heading);

    wire drifted_right = (err_raw >= 9'd1   && err_raw <= 9'd179);
    wire drifted_left  = (err_raw >= 9'd180 && err_raw <= 9'd359);

    wire [8:0] err_mag = drifted_right ? err_raw
                       : drifted_left  ? (9'd360 - err_raw) : 9'd0;

    // =========================================================================
    // Steering correction — directly in PWM units
    // No integral anywhere. No RPM conversion. No division.
    // The value here is a direct PWM step differential between wheels.
    // At base_speed=4: steer=2 means one wheel at 2, other at 6. Clearly visible.
    // =========================================================================
    wire [3:0] steer = (err_mag >= 9'd25) ? 4'd10 :
                       (err_mag >= 9'd10) ? 4'd7 :
                       (err_mag >= 9'd5)  ? 4'd5 : 
                       (err_mag >= 9'd2)  ? 4'd2 : 
                       (err_mag >= 9'd1)  ? 4'd1 : 4'd0;
    
    assign out_steer = steer;

    // =========================================================================
    // Output — combinatorial, registered one cycle for glitch-free output
    // drifted_right: target is CCW of current → turn left → R faster, L slower
    // drifted_left:  target is CW of current  → turn right → L faster, R slower
    // =========================================================================
    always @(posedge clk_50) begin
        if (!rst_n || !enable) begin
            out_speed_L <= base_speed_L;
            out_speed_R <= base_speed_R;
        end else begin
            if (drifted_right) begin
                out_speed_L <= (base_speed_L > steer)                  ? base_speed_L - steer : 4'd0;
                out_speed_R <= ({1'b0, base_speed_R} + steer <= 5'd15) ? base_speed_R + steer : 4'd15;
            end else if (drifted_left) begin
                out_speed_L <= ({1'b0, base_speed_L} + steer <= 5'd15) ? base_speed_L + steer : 4'd15;
                out_speed_R <= (base_speed_R > steer)                  ? base_speed_R - steer : 4'd0;
            end else begin
                out_speed_L <= base_speed_L;
                out_speed_R <= base_speed_R;
            end
        end
    end

endmodule