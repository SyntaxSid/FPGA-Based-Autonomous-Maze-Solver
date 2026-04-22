// ============================================================
// cos_lut.v — Quarter-wave cosine lookup table
//
// Input:  angle (0-359 degrees, 9-bit unsigned)
// Output: cos_val (signed 11-bit, scale factor 1024)
//         i.e. cos(0) = +1024, cos(90) = 0, cos(180) = -1024
//
// Uses symmetry: only 91 entries stored (0-90 degrees).
//   cos(x)       = +cos_quarter(x)        for 0   <= x <= 90
//   cos(x)       = -cos_quarter(180-x)    for 91  <= x <= 180
//   cos(x)       = -cos_quarter(x-180)    for 181 <= x <= 270
//   cos(x)       = +cos_quarter(360-x)    for 271 <= x <= 359
// ============================================================
module cos_lut(
    input  wire [8:0] angle,       // 0 to 359
    output reg signed [11:0] cos_val // -1024 to +1024
);

    // --- Step 1: Fold angle into 0-90 range & track sign ---
    reg [6:0] quarter_angle;   // 0 to 90
    reg       negate;

    always @(*) begin
        if (angle <= 9'd90) begin
            quarter_angle = angle[6:0];
            negate = 1'b0;
        end else if (angle <= 9'd180) begin
            quarter_angle = 9'd180 - angle;
            negate = 1'b1;
        end else if (angle <= 9'd270) begin
            quarter_angle = angle - 9'd180;
            negate = 1'b1;
        end else begin
            quarter_angle = 9'd360 - angle;
            negate = 1'b0;
        end
    end

    // --- Step 2: Quarter-wave ROM (91 entries) ---
    reg [10:0] cos_quarter;  // unsigned, 0 to 1024

    always @(*) begin
        case (quarter_angle)
            7'd0:  cos_quarter = 11'd1024;
            7'd1:  cos_quarter = 11'd1024;
            7'd2:  cos_quarter = 11'd1023;
            7'd3:  cos_quarter = 11'd1023;
            7'd4:  cos_quarter = 11'd1022;
            7'd5:  cos_quarter = 11'd1020;
            7'd6:  cos_quarter = 11'd1018;
            7'd7:  cos_quarter = 11'd1016;
            7'd8:  cos_quarter = 11'd1014;
            7'd9:  cos_quarter = 11'd1011;
            7'd10: cos_quarter = 11'd1008;
            7'd11: cos_quarter = 11'd1005;
            7'd12: cos_quarter = 11'd1002;
            7'd13: cos_quarter = 11'd998;
            7'd14: cos_quarter = 11'd994;
            7'd15: cos_quarter = 11'd989;
            7'd16: cos_quarter = 11'd984;
            7'd17: cos_quarter = 11'd979;
            7'd18: cos_quarter = 11'd974;
            7'd19: cos_quarter = 11'd968;
            7'd20: cos_quarter = 11'd962;
            7'd21: cos_quarter = 11'd956;
            7'd22: cos_quarter = 11'd949;
            7'd23: cos_quarter = 11'd943;
            7'd24: cos_quarter = 11'd935;
            7'd25: cos_quarter = 11'd928;
            7'd26: cos_quarter = 11'd920;
            7'd27: cos_quarter = 11'd912;
            7'd28: cos_quarter = 11'd904;
            7'd29: cos_quarter = 11'd896;
            7'd30: cos_quarter = 11'd887;
            7'd31: cos_quarter = 11'd878;
            7'd32: cos_quarter = 11'd868;
            7'd33: cos_quarter = 11'd859;
            7'd34: cos_quarter = 11'd849;
            7'd35: cos_quarter = 11'd839;
            7'd36: cos_quarter = 11'd828;
            7'd37: cos_quarter = 11'd818;
            7'd38: cos_quarter = 11'd807;
            7'd39: cos_quarter = 11'd796;
            7'd40: cos_quarter = 11'd784;
            7'd41: cos_quarter = 11'd773;
            7'd42: cos_quarter = 11'd761;
            7'd43: cos_quarter = 11'd749;
            7'd44: cos_quarter = 11'd737;
            7'd45: cos_quarter = 11'd724;
            7'd46: cos_quarter = 11'd711;
            7'd47: cos_quarter = 11'd698;
            7'd48: cos_quarter = 11'd685;
            7'd49: cos_quarter = 11'd672;
            7'd50: cos_quarter = 11'd658;
            7'd51: cos_quarter = 11'd644;
            7'd52: cos_quarter = 11'd630;
            7'd53: cos_quarter = 11'd616;
            7'd54: cos_quarter = 11'd602;
            7'd55: cos_quarter = 11'd587;
            7'd56: cos_quarter = 11'd573;
            7'd57: cos_quarter = 11'd558;
            7'd58: cos_quarter = 11'd543;
            7'd59: cos_quarter = 11'd527;
            7'd60: cos_quarter = 11'd512;
            7'd61: cos_quarter = 11'd496;
            7'd62: cos_quarter = 11'd481;
            7'd63: cos_quarter = 11'd465;
            7'd64: cos_quarter = 11'd449;
            7'd65: cos_quarter = 11'd433;
            7'd66: cos_quarter = 11'd416;
            7'd67: cos_quarter = 11'd400;
            7'd68: cos_quarter = 11'd384;
            7'd69: cos_quarter = 11'd367;
            7'd70: cos_quarter = 11'd350;
            7'd71: cos_quarter = 11'd333;
            7'd72: cos_quarter = 11'd316;
            7'd73: cos_quarter = 11'd299;
            7'd74: cos_quarter = 11'd282;
            7'd75: cos_quarter = 11'd265;
            7'd76: cos_quarter = 11'd248;
            7'd77: cos_quarter = 11'd230;
            7'd78: cos_quarter = 11'd213;
            7'd79: cos_quarter = 11'd195;
            7'd80: cos_quarter = 11'd178;
            7'd81: cos_quarter = 11'd160;
            7'd82: cos_quarter = 11'd143;
            7'd83: cos_quarter = 11'd125;
            7'd84: cos_quarter = 11'd107;
            7'd85: cos_quarter = 11'd89;
            7'd86: cos_quarter = 11'd71;
            7'd87: cos_quarter = 11'd54;
            7'd88: cos_quarter = 11'd36;
            7'd89: cos_quarter = 11'd18;
            7'd90: cos_quarter = 11'd0;
            default: cos_quarter = 11'd0;
        endcase
    end

    // --- Step 3: Apply sign ---
    always @(*) begin
        if (negate)
            cos_val = -$signed({1'b0, cos_quarter});
        else
            cos_val = $signed({1'b0, cos_quarter});
    end

endmodule
