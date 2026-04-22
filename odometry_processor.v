// ============================================================
// odometry_processor.v — Dual-encoder odometry brain
//
// Computes from two wheel encoders:
//   heading      — bot heading 0-359 degrees (differential odometry)
//   x_blocks     — X grid coordinate (signed, starting from 0)
//   y_blocks     — Y grid coordinate (signed, starting from 0)
//   velocity     — bot speed in mm/s
//   drift        — inter-wheel tick delta (L-R per 0.1s window)
//
// Coordinate system:
//   heading = 0   → bot faces +X direction
//   heading = 90  → bot faces +Y direction
//   heading = 180 → bot faces -X direction
//   heading = 270 → bot faces -Y direction
//   Starting position = (0, 0)
//
// Physical constants (PPR = 1400):
//   WHEEL_DIAMETER = 44 mm
//   TICKS_PER_REV  = 1400
//   BLOCK_SIZE     = 250 mm (25 cm)
//   WHEEL_BASE     = 120 mm (MEASURE THIS!)
// ============================================================
module odometry_processor #(
    // ---- ROBOT PARAMETERS (change to match your hardware) ----
    // Wheel base: center-to-center distance between wheels in mm.
    parameter WHEEL_BASE_MM = 94,

    // Derived constants (do not change unless you change motors)
    // PPR = 1400, Wheel diameter = 44mm
    // Differential ticks for one full 360° bot rotation:
    //   = 2 * WHEEL_BASE_MM * PPR / DIAMETER
    //   (factor of 2 because each wheel contributes ±1 to heading_pos)
    parameter TICKS_PER_BOT_ROT = (WHEEL_BASE_MM * 2 * 1400) / 44,
    // K_HEADING = 360 * 4096 / TICKS_PER_BOT_ROT (for mult-shift)
    parameter K_HEADING = (360 * 4096) / TICKS_PER_BOT_ROT,
    // Ticks per 25cm block = 2532
    parameter TICKS_PER_BLOCK = 2532
)(
    input  wire        clk_50,
    input  wire        rst_n,

    // From encoder_processor (Left wheel)
    input  wire        tick_event_L,
    input  wire        tick_dir_L,
    input  wire signed [15:0] tick_delta_L,

    // From encoder_processor (Right wheel)
    input  wire        tick_event_R,
    input  wire        tick_dir_R,
    input  wire signed [15:0] tick_delta_R,

    // Outputs
    output reg [8:0]   heading,          // Bot heading: 0-359 degrees
    output reg signed [7:0] x_blocks,    // X grid coordinate (signed, start=0)
    output reg signed [7:0] y_blocks,    // Y grid coordinate (signed, start=0)
    output reg [15:0]  velocity_mmps,    // Speed in mm/s (unsigned)
    output reg signed [15:0] drift,      // tick_delta_L - tick_delta_R (signed)
    
    // Internal FSM compass overrides
    input  wire        force_heading_snap,
    input  wire [8:0]  snap_target,

    // Block-level position snap (round x_pos/y_pos to nearest block)
    input  wire        snap_position,

    // Minimum-1-block guarantee (asserted after turns/U-turns)
    // When HIGH with snap_position: if the dominant-axis block didn't change,
    // bump it by 1 in the heading direction so the count never stalls.
    input  wire        snap_min_one
);

    // -------------------------------------------------------
    // 1. Power-on reset (0.25s, same as encoder_processor)
    // -------------------------------------------------------
    reg [23:0] por_counter = 24'd0;
    wire       rst = (por_counter < 24'd12_500_000) || !rst_n;

    always @(posedge clk_50) begin
        if (rst) por_counter <= por_counter + 1'd1;
    end

    // -------------------------------------------------------
    // 2. Heading accumulator (PURELY UNSIGNED — no signed mixing)
    //
    //    heading_pos tracks the differential tick count,
    //    wrapping at TICKS_PER_BOT_ROT.
    //
    //    Convention (top-down view):
    //      R-wheel forward alone -> bot turns LEFT  -> heading increases
    //      L-wheel forward alone -> bot turns RIGHT -> heading decreases
    // -------------------------------------------------------
    reg [15:0] heading_pos;  // 0 to TICKS_PER_BOT_ROT - 1

    // Count how many +1 and -1 contributions this cycle (all unsigned)
    wire inc_R = tick_event_R &  tick_dir_R;  // R forward  → +1
    wire dec_R = tick_event_R & ~tick_dir_R;  // R backward → -1
    wire dec_L = tick_event_L &  tick_dir_L;  // L forward  → -1
    wire inc_L = tick_event_L & ~tick_dir_L;  // L backward → +1

    wire [1:0] inc_total = {1'b0, inc_R} + {1'b0, inc_L};  // 0, 1, or 2
    wire [1:0] dec_total = {1'b0, dec_R} + {1'b0, dec_L};  // 0, 1, or 2

    always @(posedge clk_50) begin
        if (rst) begin
            heading_pos <= (16'd90 * TICKS_PER_BOT_ROT) / 16'd360; // Start at 90 degrees
        end else if (force_heading_snap) begin
            // Hardware override! Convert targeted degrees straight into encoder ticks to forcefully purge drift.
            heading_pos <= (snap_target * TICKS_PER_BOT_ROT) / 16'd360;
        end else if (inc_total > dec_total) begin
            // Net positive: heading increases
            // net = 1 or 2 (unsigned, no sign issues)
            if (heading_pos + (inc_total - dec_total) >= TICKS_PER_BOT_ROT)
                heading_pos <= heading_pos + (inc_total - dec_total) - TICKS_PER_BOT_ROT;
            else
                heading_pos <= heading_pos + (inc_total - dec_total);
        end else if (dec_total > inc_total) begin
            // Net negative: heading decreases
            // net = 1 or 2 (unsigned subtraction, always positive here)
            if (heading_pos < (dec_total - inc_total))
                heading_pos <= heading_pos + TICKS_PER_BOT_ROT - (dec_total - inc_total);
            else
                heading_pos <= heading_pos - (dec_total - inc_total);
        end
        // inc_total == dec_total: no change
    end

    // Convert heading_pos to degrees: heading = heading_pos * K_HEADING >> 12
    always @(posedge clk_50) begin
        if (rst)
            heading <= 9'd90;
        else
            heading <= (heading_pos * K_HEADING) >> 12;
    end

    // -------------------------------------------------------
    // 3. Cos & Sin lookups for X/Y projection
    //
    //    cos(heading) -> X component
    //    sin(heading) -> Y component
    //    sin(x) = cos(x - 90), so sin_angle = (heading + 270) % 360
    // -------------------------------------------------------
    wire signed [11:0] cos_heading;  // cos(heading) * 1024
    wire signed [11:0] sin_heading;  // sin(heading) * 1024

    wire [8:0] sin_angle = (heading >= 9'd90)
                         ? (heading - 9'd90)
                         : (heading + 9'd270);

    cos_lut cos_x (
        .angle(heading),
        .cos_val(cos_heading)
    );

    cos_lut cos_y (
        .angle(sin_angle),
        .cos_val(sin_heading)
    );

    // -------------------------------------------------------
    // 4. Gate timer (0.1s) — coordinates, velocity, drift
    // -------------------------------------------------------
    reg [22:0] gate_counter;

    // X and Y position accumulators (in ticks, scaled by cos/sin)
    reg signed [31:0] x_pos;
    reg signed [31:0] y_pos;

    // Block values from the LAST snap event (for snap_min_one comparison)
    // These do NOT update with the gate counter — only when snap_position fires.
    reg signed [7:0] prev_snap_xb;
    reg signed [7:0] prev_snap_yb;

    always @(posedge clk_50) begin
        if (rst) begin
            gate_counter  <= 23'd0;
            x_pos         <= 4 * TICKS_PER_BLOCK;
            y_pos         <= 32'sd0;
            x_blocks      <= 8'sd4;
            y_blocks      <= 8'sd0;
            prev_snap_xb  <= 8'sd4;
            prev_snap_yb  <= 8'sd0;
            velocity_mmps <= 16'd0;
            drift         <= 16'sd0;
        end else begin

            if (gate_counter == 23'd4_999_999) begin
                gate_counter <= 23'd0;

                // --- Drift: L - R tick delta per 0.1s window ---
                drift <= tick_delta_L - tick_delta_R;

                // --- Velocity in mm/s ---
                // velocity = avg_delta * 10 * circumference / PPR
                //          = (L+R)/2 * 10 * 138.23 / 1400
                //          = (L+R) * 506 >> 10
                begin : velocity_calc
                    reg signed [15:0] sum_delta;
                    reg [15:0] abs_sum;
                    reg [31:0] vel_product;

                    sum_delta = tick_delta_L + tick_delta_R;
                    abs_sum = sum_delta[15] ? (-sum_delta) : sum_delta;
                    vel_product = abs_sum * 16'd506;
                    velocity_mmps <= vel_product[25:10]; // >> 10
                end

                // --- X position: project onto cos(heading) ---
                // x_delta = (L+R) * cos(heading) / 2048
                //   /2 for average, /1024 for cos descale
                begin : x_calc
                    reg signed [15:0] sum_delta_x;
                    reg signed [31:0] projected_x;

                    sum_delta_x = tick_delta_L + tick_delta_R;
                    projected_x = sum_delta_x * cos_heading;
                    x_pos <= x_pos + (projected_x >>> 11);
                end

                // --- Y position: project onto sin(heading) ---
                // y_delta = (L+R) * sin(heading) / 2048
                begin : y_calc
                    reg signed [15:0] sum_delta_y;
                    reg signed [31:0] projected_y;

                    sum_delta_y = tick_delta_L + tick_delta_R;
                    projected_y = sum_delta_y * sin_heading;
                    y_pos <= y_pos + (projected_y >>> 11);
                end

                // --- Convert positions to block coordinates ---
                // blocks = pos / TICKS_PER_BLOCK
                //        = pos * 414 >> 20
                begin : block_calc
                    reg [31:0] abs_x, abs_y;
                    reg [7:0]  bx, by;

                    abs_x = x_pos[31] ? (-x_pos) : x_pos;
                    abs_y = y_pos[31] ? (-y_pos) : y_pos;

                    bx = (abs_x * 32'd414 + 32'd524288) >> 20;  // +2^19 rounding bias
                    by = (abs_y * 32'd414 + 32'd524288) >> 20;

                    // Re-apply sign for signed block coordinates
                    x_blocks <= x_pos[31] ? -$signed({24'd0, bx}) : $signed({24'd0, bx});
                    y_blocks <= y_pos[31] ? -$signed({24'd0, by}) : $signed({24'd0, by});
                end

            end else begin
                gate_counter <= gate_counter + 1'd1;
            end

            // Block-level snap: round x_pos and y_pos to nearest whole block
            // Triggered by the FSM at junction detection to purge odometry drift
            if (snap_position) begin
                begin : snap_calc
                    reg [31:0] abs_xp, abs_yp;
                    reg [31:0] rounded_bx, rounded_by;
                    reg signed [7:0] new_xb, new_yb;
                    reg x_is_dominant;

                    // --- Normal rounding ---
                    abs_xp = x_pos[31] ? (-x_pos) : x_pos;
                    abs_yp = y_pos[31] ? (-y_pos) : y_pos;
                    rounded_bx = (abs_xp * 32'd414 + 32'd524288) >> 20;
                    rounded_by = (abs_yp * 32'd414 + 32'd524288) >> 20;

                    // Signed block values after normal rounding
                    new_xb = x_pos[31] ? -rounded_bx[7:0] : rounded_bx[7:0];
                    new_yb = y_pos[31] ? -rounded_by[7:0] : rounded_by[7:0];

                    // --- Minimum-1-block guarantee (after turns/U-turns) ---
                    // Compare against prev_snap (last snapped value), NOT live x_blocks/y_blocks!
                    // The gate counter updates x_blocks/y_blocks every 0.1s, which can poison
                    // the comparison if the bot already moved a full block before this snap fires.
                    x_is_dominant = (heading >= 9'd315 || heading < 9'd45) ||
                                    (heading >= 9'd135 && heading < 9'd225);

                    if (snap_min_one) begin
                        if (x_is_dominant) begin
                            if (heading >= 9'd315 || heading < 9'd45) begin
                                // Heading ~0 (+X): x must increase by at least 1 from last snap
                                if (new_xb <= prev_snap_xb)
                                    new_xb = prev_snap_xb + 8'sd1;
                            end else begin
                                // Heading ~180 (-X): x must decrease by at least 1 from last snap
                                if (new_xb >= prev_snap_xb)
                                    new_xb = prev_snap_xb - 8'sd1;
                            end
                        end else begin
                            if (heading >= 9'd45 && heading < 9'd135) begin
                                // Heading ~90 (+Y): y must increase by at least 1 from last snap
                                if (new_yb <= prev_snap_yb)
                                    new_yb = prev_snap_yb + 8'sd1;
                            end else begin
                                // Heading ~270 (-Y): y must decrease by at least 1 from last snap
                                if (new_yb >= prev_snap_yb)
                                    new_yb = prev_snap_yb - 8'sd1;
                            end
                        end
                    end

                    // Save this snap's result for next time
                    prev_snap_xb <= new_xb;
                    prev_snap_yb <= new_yb;

                    // --- Write back: convert signed blocks → tick positions ---
                    x_pos <= (new_xb >= 0)
                             ? ($signed({24'd0, new_xb}) * TICKS_PER_BLOCK)
                             : -($signed({24'd0, -new_xb}) * TICKS_PER_BLOCK);
                    y_pos <= (new_yb >= 0)
                             ? ($signed({24'd0, new_yb}) * TICKS_PER_BLOCK)
                             : -($signed({24'd0, -new_yb}) * TICKS_PER_BLOCK);
                end
            end

        end
    end

endmodule