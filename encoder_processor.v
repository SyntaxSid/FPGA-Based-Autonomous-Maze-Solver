module encoder_processor(
    input  wire        clk_50,
    input  wire        rst_n,
    input  wire        enc_a,
    input  wire        enc_b,
    output reg signed [31:0] total_ticks, // Absolute tick count (signed, forward=positive)
    output reg [8:0]  theta,              // Wheel angle: 0 to 359 degrees
    output reg signed [15:0] tick_delta,  // Ticks counted in last 0.1s window (velocity)
    output reg signed [15:0] tick_delta_fast, // Ticks counted in 10ms window (fast PI loop)
    output wire       tick_event,         // 1-cycle pulse on each quadrature edge
    output wire       tick_dir            // Direction at time of edge: 1=forward, 0=reverse
);

    // -------------------------------------------------------
    // 1. Power-on reset: holds all registers clear for 0.25s
    //    0.25s * 50,000,000 Hz = 12,500,000 cycles
    //    24-bit counter (max 16,777,215) is sufficient.
    // -------------------------------------------------------
    reg [23:0] por_counter = 24'd0;
    wire       rst = (por_counter < 24'd12_500_000) || !rst_n;

    always @(posedge clk_50) begin
        if (rst) por_counter <= por_counter + 1'd1;
    end

    // -------------------------------------------------------
    // 2. Two-stage synchronisers (metastability prevention)
    // -------------------------------------------------------
    reg [2:0] sync_a, sync_b;

    always @(posedge clk_50) begin
        sync_a <= {sync_a[1:0], enc_a};
        sync_b <= {sync_b[1:0], enc_b};
    end

    // -------------------------------------------------------
    // 3. Quadrature edge detect (Phase-A rising + falling)
    // -------------------------------------------------------
    wire count_en = sync_a[2] ^ sync_a[1]; // Fires on every Phase-A edge
    wire dir      = sync_a[1] ^ sync_b[1]; // 1 = forward, 0 = reverse

    assign tick_event = count_en;
    assign tick_dir   = dir;

    // -------------------------------------------------------
    // 4. Internal registers
    // -------------------------------------------------------
    reg [10:0]        rev_pos;      // Per-revolution position: 0 to 1399
    reg [22:0]        gate_counter; // Counts up to 4,999,999 (= 0.1s at 50MHz)
    reg signed [31:0] ticks_prev;   // Tick snapshot from last gate window

    reg [22:0]        fast_gate_counter; // 499,999 (10ms at 50MHz)
    reg signed [31:0] fast_ticks_prev;   // Tick snapshot for fast window

    // -------------------------------------------------------
    // 5. Main logic — single clocked always block
    //    ALL assignments are non-blocking (<=) to avoid
    //    race conditions and latch inference.
    // -------------------------------------------------------
    always @(posedge clk_50) begin

        if (rst) begin
            // --- Hold in reset for first 0.25s after power-on ---
            total_ticks  <= 32'sd0;
            theta        <= 9'd0;
            tick_delta   <= 16'sd0;
            tick_delta_fast <= 16'sd0;
            rev_pos      <= 11'd0;
            gate_counter <= 23'd0;
            ticks_prev   <= 32'sd0;
            fast_gate_counter <= 23'd0;
            fast_ticks_prev <= 32'sd0;

        end else begin

            // =================================================
            // Block A: Encoder tick counting (runs only on edge)
            // =================================================
            if (count_en) begin

                // --- Absolute tick counter (signed) ---
                if (dir)
                    total_ticks <= total_ticks + 1;
                else
                    total_ticks <= total_ticks - 1;

                // --- Per-revolution position (wraps 0 <-> 1399) ---
                if (dir)
                    rev_pos <= (rev_pos == 11'd1399) ? 11'd0    : rev_pos + 1'd1;
                else
                    rev_pos <= (rev_pos == 11'd0)    ? 11'd1399 : rev_pos - 1'd1;

                // --- Theta calculation ---
                // Avoids a hardware divider (which violates 50MHz timing).
                // Approximation: 360/1400 ~= 1053/4096
                //   Max error: < 0.1 degrees across full range.
                // Product: 11-bit * 11-bit = 22-bit max, safe in a 32-bit context.
                // Right-shift by 12 gives a 9-bit result (0-359).
                theta <= (rev_pos * 21'd1053) >> 12;

            end // if (count_en)

            // =================================================
            // Block B: Gate timer - velocity measurement
            //   Latches tick delta every 0.1s.
                 //   Use tick_delta downstream for RPM or P-controller.
            //   RPM = |tick_delta| * 600 / 1400 (compute in display layer)
            // =================================================
            if (gate_counter == 23'd4_999_999) begin
                gate_counter <= 23'd0;
                tick_delta   <= total_ticks[15:0] - ticks_prev[15:0];
                ticks_prev   <= total_ticks;
            end else begin
                gate_counter <= gate_counter + 1'd1;
            end

            // =================================================
            // Block C: Fast Gate timer (10ms)
            // =================================================
            if (fast_gate_counter == 23'd499_999) begin
                fast_gate_counter <= 23'd0;
                tick_delta_fast   <= total_ticks[15:0] - fast_ticks_prev[15:0];
                fast_ticks_prev   <= total_ticks;
            end else begin
                fast_gate_counter <= fast_gate_counter + 1'd1;
            end

        end // else (not reset)

    end // always

endmodule