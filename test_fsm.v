// ============================================================
// test_fsm.v — Autonomous maze-solving FSM
//
// Triggered by Bluetooth START command (bt_go pulse).
// Direction convention: 2'b01 = forward, 2'b10 = backward
// ============================================================
module test_fsm #(
    parameter TICKS_PER_BLOCK  = 2532,
    parameter MOTOR_SPEED      = 4'd15,
    parameter ARC_ENTRY_TICKS  = 500
)(
    input  wire        clk_50,
    input  wire [8:0]          heading,
    input  wire signed [31:0]  total_ticks_L,
    input  wire        rst_n,
    input  wire [6:0]  dist_left_cm,
    input  wire [6:0]  dist_right_cm,
    input  wire [6:0]  dist_front_cm,
    input  wire        ir_object_detected_right,  // IR sensor: 1 = object detected
    input  wire        ir_object_detected_left,

    // IR3 subsystem handshake
    input  wire        ir3_object_detected,        // IR3 live detection status
    input  wire        ir3_done,                   // IR3 subsystem cycle complete
    output reg         ir3_start,                  // Pulse to trigger IR3 subsystem

    input wire bt_go,
    input wire bt_stop,
    output reg send_end,
    output reg send_pt_dist,
    output reg send_pt_us,


    // Inputs from Outer Loop
    input wire signed [15:0] outer_heading_adjust,
    input wire              outer_wall_front,

    // Hardware Outputs
    output reg         force_heading_snap,
    output reg [8:0]   snap_target,
    output reg         snap_position,     // Pulse to round odometry to nearest block
    output reg         snap_min_one,      // Guarantee at least 1 block change (after turns)
    
    // Fast 100Hz Velocity Data (Ticks per 10ms)
    input wire signed [15:0] actual_speed_L,
    input wire signed [15:0] actual_speed_R,

    output reg [3:0] speed_L,
    output reg [3:0] speed_R,
    output reg [1:0] dir_L,
    output wire [15:0] dbg_fsm_state,
    output wire [3:0] dbg_steer,
    output reg [1:0] dir_R
);

    // === States (5-bit to accommodate IR3 states) ===
    localparam S_IDLE             = 5'd0;
    localparam S_WAIT             = 5'd1;
    localparam S_TURN             = 5'd2;   // Point turn (SW[0] only)
    localparam S_FWD              = 5'd3;   // Forward (SW[1] only)
    localparam S_SEQ_INIT         = 5'd4;
    localparam S_SEQ_FWD          = 5'd5;
    localparam S_SEQ_ARC          = 5'd6;   // Arc/round turn
    localparam S_SEQ_DONE         = 5'd7;
    localparam S_SEQ_SETTLE       = 5'd8;   // 2-cycle wait for turn_target to propagate
    localparam S_SEQ_PRE_TURN     = 5'd9;   // Drive forward ~1200 ticks before turning
    localparam S_SEQ_BUMP_REVERSE = 5'd10;  // Crash Recovery
    localparam S_SEQ_POINT_TURN   = 5'd11;  // 90/180-degree point turn (correction + U-turn)
    localparam S_SEQ_UTURN_STOP   = 5'd12;  // Stop 2s, sample sensors, then U-turn
    // --- Docking alignment states ---
    localparam S_SEQ_UTURN_CENTER     = 5'd13;  // Arc to center between walls + align heading
    // 5'd14 reserved/unused
    localparam S_SEQ_UTURN_POST_ALIGN = 5'd15;  // pause, then U-turn
    // --- IR3 subsystem states ---
    localparam S_SEQ_IR3_ACTIVE       = 5'd16;  // Servo/BT cycle in progress

    assign dbg_fsm_state = {state[4:0], seq_stage[2:0], 4'd0, last_fallback_trigger};

    reg [4:0] state = S_IDLE;
    reg bt_started = 1'b0;     // Latch: HIGH while BT-initiated sequence is active
    reg [4:0] settle_cnt;

    reg [27:0] wait_cnt;
    localparam WAIT_4S = 28'd200_000_000;
    localparam WAIT_2S = 28'd100_000_000;  // 2-second pause for U-turn sensor sampling

    // === Snapshot registers ===
    reg [8:0]          start_heading;
    reg [8:0]          turn_target;
    reg signed [31:0]  start_ticks;
    reg [31:0]         fwd_tick_target;
    reg                turn_is_right;
    reg [8:0]          align_target;   // nearest_grid captured when dead-end detected

    // === Sequence variables ===
    reg [2:0]  seq_stage;
    reg        recently_turned;
    reg        recently_uturned;
    reg        recently_aligned;
    reg [8:0]  saved_junction_target;  // saves turn_target for evasion recovery
    reg [31:0] post_commit_dist;
    reg [31:0] reverse_target;
    reg        ir3_at_stop;            // IR3 was detected during S_SEQ_UTURN_STOP
    reg        long_hunt;              // HIGH when hunting >7000 ticks — disables all fallbacks

    // === Hardware Stall Detector (Encoders) ===
    reg [23:0] stall_timer;
    wire is_enc_stalled = !long_hunt && (stall_timer > 24'd25_000_000); // 0.5 sec threshold
    
    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) stall_timer <= 0;
        else begin
            if (state == S_IDLE || (speed_L == 0 && speed_R == 0)) begin
                stall_timer <= 0;
            end else if (actual_speed_L > 2 || actual_speed_L < -2 || actual_speed_R > 2 || actual_speed_R < -2) begin
                // Robot is physically moving — not stalled
                stall_timer <= 0;
            end else if ((state == S_SEQ_FWD || state == S_SEQ_PRE_TURN) && fwd_base_speed < 5) begin
                // Robot is intentionally decelerating (soft-stop) — don't false-trigger
                stall_timer <= 0;
            end else begin
                stall_timer <= stall_timer + 1;
            end
        end
    end

    // === Sonar Slip Detector (Stuck but wheels slipping on floor) ===
    reg [27:0] sonar_stall_timer;
    reg [6:0]  last_dist_front, last_dist_left, last_dist_right;
    wire is_sonar_stalled = !long_hunt && (sonar_stall_timer > 28'd50_000_000); // 1.0 sec threshold

    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) begin
            sonar_stall_timer <= 0;
            last_dist_front <= 0; last_dist_left <= 0; last_dist_right <= 0;
        end else begin
            if (state == S_IDLE || (speed_L == 0 && speed_R == 0)) begin
                sonar_stall_timer <= 0;
                last_dist_front <= dist_front_cm; last_dist_left <= dist_left_cm; last_dist_right <= dist_right_cm;
            end else if (dist_front_cm > 25 && dist_left_cm > 25 && dist_right_cm > 25) begin
                // Open field, not grinding a wall
                sonar_stall_timer <= 0;
                last_dist_front <= dist_front_cm; last_dist_left <= dist_left_cm; last_dist_right <= dist_right_cm;
            end else if (
                (dist_front_cm > last_dist_front ? dist_front_cm - last_dist_front : last_dist_front - dist_front_cm) > 2 ||
                (dist_left_cm > last_dist_left   ? dist_left_cm - last_dist_left   : last_dist_left - dist_left_cm) > 2 ||
                (dist_right_cm > last_dist_right ? dist_right_cm - last_dist_right : last_dist_right - dist_right_cm) > 2
            ) begin
                // We physically moved at least 3cm relative to walls
                sonar_stall_timer <= 0;
                last_dist_front <= dist_front_cm; last_dist_left <= dist_left_cm; last_dist_right <= dist_right_cm;
            end else begin
                sonar_stall_timer <= sonar_stall_timer + 1;
            end
        end
    end

    // === Global State Timeouts (Universal Failsafe) ===
    reg [31:0] state_timer;
    reg [4:0]  last_state;
    reg [2:0]  last_seq_stage;
    
    always @(posedge clk_50) begin
        if (state != last_state || seq_stage != last_seq_stage) begin
            state_timer <= 0;
            last_state <= state;
            last_seq_stage <= seq_stage;
        end else begin
            state_timer <= state_timer + 1;
        end
    end

    wire is_pre_turn_timeout = !long_hunt && (state == S_SEQ_PRE_TURN) && (state_timer > 32'd150_000_000); // 3 sec blind commit

    reg [3:0] last_fallback_trigger;
    always @(posedge clk_50 or negedge rst_n) begin
        if (!rst_n) last_fallback_trigger <= 4'd0;
        else if (is_enc_stalled) last_fallback_trigger <= 4'd1;
        else if (is_pre_turn_timeout) last_fallback_trigger <= 4'd5;
    end

    // === Safety Clamps for Wall Offset Math ===
    wire [6:0] safe_left_cm  = (dist_left_cm > 7'd15)  ? 7'd6 : dist_left_cm;
    wire [6:0] safe_right_cm = (dist_right_cm > 7'd15) ? 7'd6 : dist_right_cm;

    // === Turn remaining ===
    wire [8:0] turn_rem_left = (turn_target >= heading)
                             ? (turn_target - heading)
                             : (9'd360 + turn_target - heading);

    wire [8:0] turn_rem_right = (heading >= turn_target)
                              ? (heading - turn_target)
                              : (9'd360 + heading - turn_target);

    wire [8:0] active_turn_rem = turn_is_right ? turn_rem_right : turn_rem_left;

    // === Angle-error for alignment (signed, range -180 to +180) ===
    // Used only in S_SEQ_UTURN_ANGLE to correct heading to align_target.
    // Positive err  → heading must increase → turn left  (R-fwd, L-bwd)
    // Negative err  → heading must decrease → turn right (L-fwd, R-bwd)
    // align_target and heading are [8:0] (0-359), zero-extended to 10-bit signed.
    wire signed [9:0] raw_angle_err = $signed({1'b0, align_target}) - $signed({1'b0, heading});
    wire signed [9:0] angle_err =
        (raw_angle_err >  10'sd180) ? (raw_angle_err - 10'sd360) :
        (raw_angle_err < -10'sd180) ? (raw_angle_err + 10'sd360) :
                                       raw_angle_err;
    wire angle_aligned = (angle_err >= -10'sd3) && (angle_err <= 10'sd3);

    // === Lateral centering error (signed, right - left, cm) ===
    // Positive → bot is left of center  → needs to move right
    // Negative → bot is right of center → needs to move left
    wire signed [8:0] lateral_err = $signed({2'b0, dist_right_cm}) - $signed({2'b0, dist_left_cm});
    wire              centered     = (lateral_err >= -9'sd3) && (lateral_err <= 9'sd3);

    // === Alignment turn speed (more aggressive than general turn_speed) ===
    // Uses |angle_err| directly so it's independent of turn_is_right / active_turn_rem.
    wire [9:0] abs_angle_err = angle_err[9] ? (-angle_err) : angle_err;
    reg  [3:0] align_speed;
    always @(*) begin
        if      (abs_angle_err <= 10'd3)  align_speed = 4'd0;   // done (angle_aligned)
        else if (abs_angle_err <= 10'd15) align_speed = 4'd6;   // slow crawl near target
        else if (abs_angle_err <= 10'd50) align_speed = 4'd10;  // medium
        else                              align_speed = 4'd13;  // full blast when far
    end

    // === Turn P-controller ===
    reg [3:0] turn_speed;
    always @(*) begin
        if (active_turn_rem <= 9'd9 || active_turn_rem > 9'd200)
            turn_speed = 4'd0;                // done / badly overshot
        else if (active_turn_rem <= 9'd50)
            turn_speed = 4'd8;                // (5° – 15°) -> bumped to 8 for point turn skid torque
        else if (active_turn_rem <= 9'd60)
            turn_speed = 4'd10;               // (15° – 50°) -> bumped to 10
        else
            turn_speed = 4'd10;               // full speed (> 50°) -> bumped to 10
    end

 // === Arc Turn: Outer wheel speed/direction profile ===
    reg [3:0] arc_outer_speed;
    always @(*) begin
        if (active_turn_rem <= 9'd10) begin
            arc_outer_speed = 4'd0;        // Dead-stop in final 10°
        end else if (active_turn_rem <= 9'd30) begin
            arc_outer_speed = 4'd8;        // Bumped from 5 to 7 to prevent arc turn stiction
        end else begin
            arc_outer_speed = MOTOR_SPEED; // Full speed for the main chunk of the arc
        end
    end

    // === Forward: distance and soft stop ===
    wire signed [31:0] t_delta = total_ticks_L - start_ticks;
    wire [31:0] dist_travelled = (t_delta >= 0) ? t_delta : -t_delta;
    wire [31:0] ticks_remaining = (fwd_tick_target > dist_travelled)
                                ? (fwd_tick_target - dist_travelled) : 32'd0;

    reg [3:0] fwd_base_speed;
    always @(*) begin
        if (ticks_remaining <= 32'd100)
            fwd_base_speed = 4'd6;           // Bumped from 3/4
        else if (ticks_remaining <= 32'd250)
            fwd_base_speed = 4'd7;           // Bumped from 5
        else
            fwd_base_speed = MOTOR_SPEED;
    end

    // === INNER LOOP: Heading Controller ===
    wire [3:0] fwd_spd_L, fwd_spd_R;
    wire fwd_enable = (state == S_SEQ_FWD || state == S_SEQ_PRE_TURN);

    // Outer loop enabled except during final commit block of SW2
    wire use_outer_loop = (state == S_SEQ_FWD && seq_stage == 3'd1);

    wire signed [15:0] adjust = use_outer_loop ? outer_heading_adjust : 5'sd0;
    wire signed [15:0] eff_target_raw = start_heading - adjust;
    
    // Mathematically safe modulo 360 for negative adjustments
    wire [8:0] effective_target = (eff_target_raw < 0) ? (eff_target_raw + 360) :
                                  (eff_target_raw >= 360) ? (eff_target_raw - 360) :
                                  eff_target_raw[8:0];

    // === Motor Hardware Bias ===
    // If the right motor is physically weaker, it naturally drifts right and rides the 5-degree deadband.
    // We subtract 1 PWM from the left motor before the heading controller to equalize hardware torque, 
    // since the robot is already running at max PWM (15).
    wire [3:0] fwd_base_speed_L = (fwd_base_speed == 4'd0) ? 4'd0 :
                                  (fwd_base_speed > 4'd0)  ? fwd_base_speed - 4'd2 : 4'd0;

    Heading_Controller inner_loop (
        .clk_50(clk_50),
        .rst_n(1'b1),
        .enable(fwd_enable),
        .target_heading(effective_target),
        .current_heading(heading),
        .base_speed_L(fwd_base_speed_L),
        .base_speed_R(fwd_base_speed),
        .actual_speed_R(actual_speed_R),
        .out_steer(dbg_steer),
        .out_speed_L(fwd_spd_L),
        .out_speed_R(fwd_spd_R)
    );

    // === Grid-snap helper ===
    wire [8:0] nearest_grid = (heading >= 9'd315 || heading < 9'd45)  ? 9'd0   :
                              (heading < 9'd135)                       ? 9'd90  :
                              (heading < 9'd225)                       ? 9'd180 :
                                                                         9'd270;
    wire [8:0] grid_left  = (nearest_grid == 9'd270) ? 9'd0   : (nearest_grid + 9'd90);
    wire [8:0] grid_right = (nearest_grid == 9'd0)   ? 9'd270 : (nearest_grid - 9'd90);

    wire [8:0] heading_diff_raw = (heading >= start_heading) ? (heading - start_heading) : (start_heading - heading);
    wire [8:0] abs_heading_diff = (heading_diff_raw > 9'd180) ? (9'd360 - heading_diff_raw) : heading_diff_raw;
    wire safe_to_snap = (abs_heading_diff <= 9'd5);

    // =====================================================================
    // Main FSM
    // =====================================================================
    always @(posedge clk_50) begin
        if (!rst_n) begin
            state <= S_IDLE;
            speed_L <= 4'd0; speed_R <= 4'd0;
            force_heading_snap <= 1'b0;
            snap_target <= 9'd0;
            snap_position <= 1'b0;
            snap_min_one <= 1'b0;
            ir3_start <= 1'b0;
            bt_started <= 1'b0;
            send_pt_dist <= 1'b0;
            send_pt_us <= 1'b0;
        end else if (bt_stop) begin
            state <= S_IDLE;
            speed_L <= 4'd0; speed_R <= 4'd0;
            force_heading_snap <= 1'b0;
            snap_target <= 9'd0;
            snap_position <= 1'b0;
            snap_min_one <= 1'b0;
            ir3_start <= 1'b0;
            bt_started <= 1'b0;
            wait_cnt <= 28'd0;
            send_pt_dist <= 1'b0;
            send_pt_us <= 1'b0;
        end else begin
            send_pt_dist <= 1'b0;
            send_pt_us <= 1'b0;
            case (state)

            // ---------- IDLE ----------
            S_IDLE: begin
                speed_L <= 4'd0; speed_R <= 4'd0;
                dir_L <= 2'b01; dir_R <= 2'b01;
                wait_cnt <= 28'd0;
                force_heading_snap <= 1'b0;
                snap_position <= 1'b0;
                snap_min_one <= 1'b0;
                if (bt_go) begin seq_stage <= 3'd0; wait_cnt <= 28'd0; bt_started <= 1'b1; state <= S_SEQ_INIT; end
            end

            // ==============================================================
            // SEQUENCE RUNNER (SW[2]) — Autonomous Junction Navigation
            // ==============================================================

            // ---------- SEQ_INIT: Instant start ----------
            S_SEQ_INIT: begin
                speed_L <= 4'd0; speed_R <= 4'd0;
                force_heading_snap <= 1'b0;
                if (!bt_started) begin
                    state <= S_IDLE;
                end else begin
                    // Initialize straight-line driving safely
                    start_ticks <= total_ticks_L;
                    start_heading <= heading;  // Anchor perfectly
                    fwd_tick_target <= 32'hFFFF_FFFF; // Drive infinitely until interrupt
                    seq_stage <= 3'd1;         // Hunt Mode
                    recently_turned <= 1'b0;
                    recently_uturned <= 1'b0;
                    recently_aligned <= 1'b0;
                    force_heading_snap <= 1'b0;
                    long_hunt <= 1'b0;         // Reset long-hunt flag at start
                    state <= S_SEQ_FWD;
                end
            end

            // ---------- SEQ_FWD: Hunt for junction, or drive final block ----------
            S_SEQ_FWD: begin
                speed_L <= fwd_spd_L; speed_R <= fwd_spd_R;
                dir_L <= 2'b01; dir_R <= 2'b01;
                force_heading_snap <= 1'b0; // Ensure snap drops the moment we start driving
                snap_position      <= 1'b0; // Clear position snap
                snap_min_one       <= 1'b0;

                if (seq_stage == 3'd1) begin
                    // If we drive a substantial distance straight, it's not a consecutive turn
                    if (dist_travelled > 32'd1000) begin
                        recently_turned <= 1'b0;
                        recently_uturned <= 1'b0;
                    end
                    // Latch long_hunt when hunting has gone very far
                    if (dist_travelled > 32'd7000) long_hunt <= 1'b1;

                    if (is_enc_stalled) begin
                        speed_L <= 4'd0; speed_R <= 4'd0;
                        start_ticks <= total_ticks_L;
                        reverse_target <= 32'd600;
                        seq_stage      <= 3'd2; // Straight Evasion
                        state          <= S_SEQ_BUMP_REVERSE;
                    end
                    else if ((!ir_object_detected_right)|(!ir_object_detected_left)) begin
                        // Junction detected! Snap heading to grid ONLY if within 5 degrees
                        if (safe_to_snap) begin
                            force_heading_snap <= 1'b1;
                            snap_target        <= nearest_grid;
                            start_heading      <= nearest_grid;
                        end else begin
                            force_heading_snap <= 1'b0;
                        end
                        start_ticks        <= total_ticks_L;
                        fwd_tick_target    <= recently_uturned ? (32'd1000 - 32'd420) :
                                              recently_turned  ? (32'd1000 - 32'd250) : 32'd1000;
                        seq_stage          <= 3'd2;
                        state              <= S_SEQ_PRE_TURN;
                    end
                    // --- INSTANT DIP & ALIGN + U-TURN: IR detected early ---
                    else if (ir3_object_detected && dist_front_cm < 7'd18) begin
                        speed_L       <= 4'd0; speed_R <= 4'd0;
                        snap_position <= 1'b1;
                        snap_min_one  <= recently_turned ? 1'b1 : 1'b0;
                        
                        align_target  <= nearest_grid;
                        start_ticks   <= total_ticks_L;
                        wait_cnt      <= 28'd0;
                        
                        ir3_start     <= 1'b1;
                        state         <= S_SEQ_IR3_ACTIVE;
                    end
                    // --- NORMAL U-TURN: no junction AND wall close ahead ---
                    else if (dist_front_cm < 7'd12) begin
                        // Stop, sample sensors for 2s, then do angle-align + 180° point turn
                        speed_L  <= 4'd0; speed_R <= 4'd0;
                        wait_cnt <= 28'd0;
                        snap_position <= 1'b1; // Trigger position snap exactly as we detect dead end
                        snap_min_one  <= recently_turned ? 1'b1 : 1'b0;
                        state    <= S_SEQ_UTURN_STOP;
                    end
                end
                else if (seq_stage == 3'd3) begin
                    // Stage 3: Post-commit phase — driving post_commit_dist after the turn
                    // POST-COMMIT CRASH EVASION (10-degree understeer recovery)
                    if (is_enc_stalled) begin
                        speed_L <= 4'd0; speed_R <= 4'd0;
                        start_ticks <= total_ticks_L;
                        // Remaining distance owed + the 600 backup ticks
                        fwd_tick_target <= (fwd_tick_target > dist_travelled)
                                           ? (fwd_tick_target - dist_travelled + 32'd600)
                                           : 32'd600;
                        reverse_target <= 32'd600;
                        seq_stage <= 3'd4;
                        state <= S_SEQ_BUMP_REVERSE;
                    end
                    else if (dist_travelled >= fwd_tick_target) begin
                        speed_L <= 4'd0; speed_R <= 4'd0;
                        recently_turned <= 1'b1;
                        recently_uturned <= 1'b0;
                        state <= S_SEQ_DONE;
                    end
                end

                if (!bt_started) state <= S_IDLE;
            end

            // ---------- SEQ_PRE_TURN: drive forward to center on junction ----------
            S_SEQ_PRE_TURN: begin
                speed_L <= fwd_spd_L; speed_R <= fwd_spd_R;
                dir_L <= 2'b01; dir_R <= 2'b01;

                if (is_enc_stalled || is_pre_turn_timeout) begin
                    speed_L <= 4'd0; speed_R <= 4'd0;
                    start_ticks <= total_ticks_L;
                    fwd_tick_target <= (fwd_tick_target > dist_travelled)
                                       ? (fwd_tick_target - dist_travelled + 32'd600)
                                       : 32'd600;
                    // Set tentative turn direction for evasion based on what's currently open
                    if (!ir_object_detected_left) begin
                        turn_target   <= (start_heading + 9'd90 >= 9'd360) ? (start_heading + 9'd90 - 9'd360) : (start_heading + 9'd90);
                        turn_is_right <= 1'b0;
                    end else begin
                        turn_target   <= (start_heading >= 9'd90) ? (start_heading - 9'd90) : (9'd360 + start_heading - 9'd90);
                        turn_is_right <= 1'b1;
                    end
                    reverse_target <= 32'd600;
                    seq_stage <= 3'd5;  // Reverse → 30-deg evasion
                    state <= S_SEQ_BUMP_REVERSE;
                end
                else if (dist_travelled >= fwd_tick_target ||
                        ((dist_travelled + 32'd500 >= fwd_tick_target) && dist_front_cm > 7'd0 && dist_front_cm < 7'd15)) begin
                    if (dist_travelled >= fwd_tick_target) send_pt_dist <= 1'b1;
                    else send_pt_us <= 1'b1;

                    speed_L <= 4'd0; speed_R <= 4'd0;

                    // === DIRECTION DECISION (Left > Front > Right) ===
                    // Robot is now centered on junction — all sensors are reliable
                    // Uses RELATIVE heading math to start_heading to be drift-immune
                    if (!ir_object_detected_left) begin
                        turn_target      <= (start_heading + 9'd90 >= 9'd360) ? (start_heading + 9'd90 - 9'd360) : (start_heading + 9'd90);
                        turn_is_right    <= 1'b0;
                        post_commit_dist <= 32'd450;
                        snap_position    <= 1'b1;  // Round odometry to nearest block
                        snap_min_one     <= recently_turned ? 1'b1 : 1'b0;
                        settle_cnt       <= 4'd0;
                        state            <= S_SEQ_SETTLE;
                    end
                    else if (dist_front_cm > 7'd25) begin
                        // Front is open — drive past junction to clear IR sensors
                        start_ticks     <= total_ticks_L;
                        // DO NOT snap start_heading here, preserve the corridor's original vector
                        fwd_tick_target <= 32'd450;
                        snap_position   <= 1'b1;  // Round odometry to nearest block
                        snap_min_one    <= recently_turned ? 1'b1 : 1'b0;
                        seq_stage       <= 3'd3;
                        state           <= S_SEQ_FWD;
                    end
                    else if (!ir_object_detected_right) begin
                        turn_target      <= (start_heading >= 9'd90) ? (start_heading - 9'd90) : (9'd360 + start_heading - 9'd90);
                        turn_is_right    <= 1'b1;
                        post_commit_dist <= 32'd450;
                        snap_position    <= 1'b1;  // Round odometry to nearest block
                        snap_min_one     <= recently_turned ? 1'b1 : 1'b0;
                        settle_cnt       <= 4'd0;
                        state            <= S_SEQ_SETTLE;
                    end
                    else begin
                        // Dead end: all sides blocked
                        state <= S_SEQ_DONE;
                    end
                end
                if (!bt_started) state <= S_IDLE;
            end

            // ---------- SEQ_SETTLE: let turn_target propagate before arc ----------
            S_SEQ_SETTLE: begin
                speed_L <= 4'd0; speed_R <= 4'd0;
                snap_position <= 1'b0; // Must clear immediately — cannot stay high during turns
                snap_min_one  <= 1'b0;
                if (settle_cnt >= 4'd3) begin
                    state <= S_SEQ_ARC;
                end else begin
                    settle_cnt <= settle_cnt + 1'd1;
                end
                if (!bt_started) state <= S_IDLE;
            end

            // ---------- SEQ_ARC: Execute the chosen turn into the open path ----------
            S_SEQ_ARC: begin
                force_heading_snap <= 1'b0;
                recently_aligned <= 1'b0; // Reset corridor snap

                // ARC STALL EVASION: crashed mid-turn → reverse, 30° same direction, 700 tick thrust
                if (is_enc_stalled) begin
                    speed_L <= 4'd0; speed_R <= 4'd0;
                    start_ticks    <= total_ticks_L;
                    reverse_target <= 32'd600;
                    post_commit_dist <= 32'd700;  // Forward thrust after 30° evasion
                    seq_stage      <= 3'd5;       // Reuse pre-turn evasion pipeline (30°)
                    state          <= S_SEQ_BUMP_REVERSE;
                end
                else begin
                    if (turn_is_right) begin
                        dir_L   <= 2'b01;          speed_L <= arc_outer_speed;
                        dir_R   <= 2'b01;          speed_R <= 4'd0;
                    end else begin
                        dir_R   <= 2'b01;          speed_R <= arc_outer_speed;
                        dir_L   <= 2'b01;          speed_L <= 4'd0;
                    end

                    if (active_turn_rem <= 9'd17 || active_turn_rem > 9'd180) begin
                        start_ticks     <= total_ticks_L;
                        start_heading   <= turn_target;
                        fwd_tick_target <= post_commit_dist;
                        seq_stage       <= 3'd3;
                        state           <= S_SEQ_FWD;
                    end
                end

                if (!bt_started) state <= S_IDLE;
            end

            // ---------- SEQ_DONE ----------
            S_SEQ_DONE: begin
                speed_L <= 4'd0; speed_R <= 4'd0;
                if (!bt_started) state <= S_IDLE;
                else begin
                    force_heading_snap <= 1'b0;
                    start_ticks        <= total_ticks_L;
                    start_heading      <= heading;
                    fwd_tick_target    <= 32'hFFFF_FFFF;
                    recently_aligned   <= 1'b0;
                    long_hunt          <= 1'b0;  // Clear flag for fresh hunt
                    seq_stage          <= 3'd1;
                    state <= S_SEQ_FWD;
                end
            end

            // ---------- SEQ_BUMP_REVERSE (Crash/Fallback Recovery) ----------
            S_SEQ_BUMP_REVERSE: begin
                force_heading_snap <= 1'b0;
                dir_L <= 2'b10; dir_R <= 2'b10;
                
                if (seq_stage == 3'd4 && state_timer < 32'd25_000_000) begin
                    // Stage 4: Asymmetrical 'Peeling' — first 0.5s curves to yank bumper off wall
                    if (turn_is_right) begin
                        speed_L <= 4'd8; speed_R <= 4'd5;
                    end else begin
                        speed_R <= 4'd8; speed_L <= 4'd5;
                    end
                end else begin
                    speed_L <= 4'd6; speed_R <= 4'd6;
                end
                
                if (dist_travelled >= reverse_target) begin
                    speed_L <= 4'd0; speed_R <= 4'd0;
                    start_ticks <= total_ticks_L;
                    
                    if (seq_stage == 3'd5) begin
                        // PRE-TURN EVASION: 30° toward the junction opening
                        if (turn_is_right) begin
                            // Was turning right → hit left wall → turn 30° RIGHT to evade
                            turn_target <= (heading >= 9'd30) ? (heading - 9'd30) : (9'd360 + heading - 9'd30);
                        end else begin
                            // Was turning left → hit right wall → turn 30° LEFT to evade
                            turn_target <= (heading + 9'd30 >= 9'd360) ? (heading + 9'd30 - 9'd360) : (heading + 9'd30);
                        end
                        seq_stage <= 3'd6;
                        state <= S_SEQ_POINT_TURN;
                    end
                    else if (seq_stage == 3'd4) begin
                        // POST-COMMIT EVASION: 30° to correct understeer
                        if (turn_is_right) begin
                            // Crashed exiting a RIGHT turn → understeer → turn further RIGHT
                            turn_target <= (heading >= 9'd30) ? (heading - 9'd30) : (9'd360 + heading - 9'd30);
                        end else begin
                            // Crashed exiting a LEFT turn → understeer → turn further LEFT
                            turn_target <= (heading + 9'd30 >= 9'd360) ? (heading + 9'd30 - 9'd360) : (heading + 9'd30);
                        end
                        seq_stage <= 3'd7;
                        state <= S_SEQ_POINT_TURN;
                    end
                    else if (seq_stage == 3'd2) begin
                        // STRAIGHT EVASION: 30° opposite of last turn
                        if (turn_is_right) begin
                            // Last was right, so turn LEFT
                            turn_target <= (heading + 9'd30 >= 9'd360) ? (heading + 9'd30 - 9'd360) : (heading + 9'd30);
                            turn_is_right <= 1'b0; // Flag that we are turning LEFT now
                        end else begin
                            // Last was left, so turn RIGHT
                            turn_target <= (heading >= 9'd30) ? (heading - 9'd30) : (9'd360 + heading - 9'd30);
                            turn_is_right <= 1'b1; // Flag that we are turning RIGHT now
                        end
                        seq_stage <= 3'd2; 
                        state <= S_SEQ_POINT_TURN;
                    end
                end
                if (!bt_started) state <= S_IDLE;
            end

            // ---------- SEQ_POINT_TURN (Correction turns + U-turns) ----------
            S_SEQ_POINT_TURN: begin
                if (turn_is_right) begin
                    dir_L <= 2'b01; dir_R <= 2'b10; // CW
                end else begin
                    dir_L <= 2'b10; dir_R <= 2'b01; // CCW
                end

                if (turn_speed == 4'd0) begin
                    speed_L <= 4'd0; speed_R <= 4'd0;
                    start_ticks <= total_ticks_L;

                    if (seq_stage == 3'd6) begin
                        // 30-degree evasion complete → diagonal thrust to clear junction
                        force_heading_snap <= 1'b1;
                        snap_target        <= nearest_grid;
                        start_heading      <= nearest_grid;
                        post_commit_dist   <= 32'd1000;      // 10cm blind thrust
                        seq_stage          <= 3'd3;
                        state              <= S_SEQ_FWD;
                    end
                    else if (seq_stage == 3'd2) begin
                        // Straight evasion complete -> resume hunting
                        force_heading_snap <= 1'b1;
                        snap_target        <= nearest_grid;
                        start_heading      <= nearest_grid;
                        fwd_tick_target    <= 32'hFFFF_FFFF;
                        seq_stage          <= 3'd1;     // Back to hunting
                        state              <= S_SEQ_FWD;
                    end
                    else if (seq_stage == 3'd7) begin
                        // 10-degree post-commit evasion complete → resume remaining thrust
                        force_heading_snap <= 1'b1;
                        snap_target        <= nearest_grid;
                        start_heading      <= nearest_grid;
                        // fwd_tick_target already updated in Stage 3 trap
                        seq_stage          <= 3'd3;
                        state              <= S_SEQ_FWD;
                    end
                    else begin
                        // Generic exit (U-turn, etc.) → back to hunting
                        force_heading_snap <= 1'b1;
                        snap_target        <= nearest_grid;
                        start_heading      <= nearest_grid;
                        fwd_tick_target    <= 32'hFFFF_FFFF;
                        recently_turned    <= 1'b1; // U-turn counts as a turn for min-block guarantee
                        recently_uturned   <= 1'b1; // Flag this specifically as a U-turn
                        seq_stage          <= 3'd1;
                        state              <= S_SEQ_FWD;
                    end
                end else begin
                    speed_L <= turn_speed; speed_R <= turn_speed;
                end
                if (!bt_started) state <= S_IDLE;
            end

            // ---------- SEQ_UTURN_STOP: stop pulse, check IRs ----------
            S_SEQ_UTURN_STOP: begin
                speed_L <= 4'd0; speed_R <= 4'd0;
                snap_position <= 1'b0; // Must clear it immediately so it's a 1-clock pulse
                if (!bt_started) begin
                    state <= S_IDLE;
                end else begin
                    // Instantly re-check IRs before committing to U-turn
                    if ((!ir_object_detected_right) | (!ir_object_detected_left)) begin
                        // Junction detected! Route to PRE_TURN instead of U-turn.
                        start_ticks     <= total_ticks_L;
                        fwd_tick_target <= 32'd400;  // Short centering drive (already near junction)
                        seq_stage       <= 3'd2;
                        state           <= S_SEQ_PRE_TURN;
                    end else begin
                        // Genuine dead end — latch IR3 status, proceed with docking U-turn
                        ir3_at_stop  <= ir3_object_detected;  // Capture IR3 state
                        align_target <= nearest_grid;
                        start_ticks  <= total_ticks_L;
                        wait_cnt     <= 28'd0;
                        state        <= S_SEQ_UTURN_CENTER;
                    end
                end
            end

            // ---------- SEQ_UTURN_CENTER: arc to center + align heading ----------
            // Phase 1 of docking: no front-distance constraint here.
            // Priority 1 — heading badly off (>15°): pure point-turn to straighten first.
            // Priority 2 — laterally off (>3cm): arc backward toward center.
            //   Reversing with speed_L > speed_R arcs the bot to the RIGHT.
            //   Reversing with speed_R > speed_L arcs the bot to the LEFT.
            // Priority 3 — centered but small angle residual: slow point-turn to finish.
            // Exit: centered AND angle_aligned, or 5s timeout.
            S_SEQ_UTURN_CENTER: begin
                if (!bt_started) begin
                    state <= S_IDLE;

                end else if (wait_cnt >= 28'd250_000_000 ||   // 5s timeout
                             (centered && angle_aligned) ||
                             dist_front_cm >= 7'd13) begin  // STOP reversing if front distance >= 13cm
                    // Centered AND straight (or timeout/cap)
                    speed_L  <= 4'd0; speed_R <= 4'd0;
                    wait_cnt <= 28'd0;

                    // Both IR3 triggered and non-triggered paths proceed to U-turn after alignment
                    // (IR3 dip already handled BEFORE this state if triggered)
                    turn_target   <= (align_target >= 9'd180)
                                     ? (align_target - 9'd180)
                                     : (align_target + 9'd180);
                    turn_is_right <= (dist_right_cm >= dist_left_cm);
                    state         <= S_SEQ_UTURN_POST_ALIGN;

                end else begin
                    wait_cnt <= wait_cnt + 1'd1;

                    if (angle_err > 10'sd15 || angle_err < -10'sd15) begin
                        // Priority 1: Heading too far off — straighten first via in-place point turn
                        if (angle_err > 0) begin
                            dir_L <= 2'b10; speed_L <= align_speed;  // L-bwd
                            dir_R <= 2'b01; speed_R <= align_speed;  // R-fwd
                        end else begin
                            dir_L <= 2'b01; speed_L <= align_speed;  // L-fwd
                            dir_R <= 2'b10; speed_R <= align_speed;  // R-bwd
                        end

                    end else if (lateral_err > 9'sd4 && dist_front_cm < 7'd13) begin
                        // Priority 2: Bot LEFT of center → arc backward-right (until front dist reaches 13cm)
                        dir_L   <= 2'b10; dir_R <= 2'b10;
                        speed_L <= 4'd10; speed_R <= 4'd6;

                    end else if (lateral_err < -9'sd4 && dist_front_cm < 7'd13) begin
                        // Priority 2: Bot RIGHT of center → arc backward-left (until front dist reaches 13cm)
                        dir_L   <= 2'b10; dir_R <= 2'b10;
                        speed_L <= 4'd6;  speed_R <= 4'd10;

                    end else begin
                        // Priority 3: Laterally OK (or cap reached) — fine-tune angle in place
                        if (angle_err > 0) begin
                            dir_L <= 2'b10; speed_L <= 4'd6;
                            dir_R <= 2'b01; speed_R <= 4'd6;
                        end else begin
                            dir_L <= 2'b01; speed_L <= 4'd6;
                            dir_R <= 2'b10; speed_R <= 4'd6;
                        end
                    end
                end
            end


            // ---------- SEQ_UTURN_POST_ALIGN: proceed directly to U-turn ----------
            // Motors off. IR3 servo/BT cycle already provided the pause.
            S_SEQ_UTURN_POST_ALIGN: begin
                speed_L <= 4'd0; speed_R <= 4'd0;
                if (!bt_started) begin
                    state <= S_IDLE;
                end else begin
                    state <= S_SEQ_POINT_TURN;
                end
            end

            // ---------- SEQ_IR3_ACTIVE: servo/BT cycle in progress ----------
            // Motors off. Subsystem runs the servo dip + DHT/BT send.
            // When ext_done pulses, proceed to alignment (UTURN_CENTER).
            S_SEQ_IR3_ACTIVE: begin
                speed_L   <= 4'd0; speed_R <= 4'd0;
                ir3_start <= 1'b0;   // Deassert after 1 cycle
                if (!bt_started) begin
                    state <= S_IDLE;
                end else if (ir3_done) begin
                    wait_cnt <= 28'd0;
                    state    <= S_SEQ_UTURN_CENTER;
                end
            end

            default: state <= S_IDLE;
        endcase
        end
    end

endmodule