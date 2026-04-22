module ir_sensor_flying_fish (
    input  wire clk,        // FPGA clock (50MHz assumed)
    input  wire rst_n,      // Active-low reset
    input  wire ir_in,      // Digital OUT pin from MH Flying Fish sensor

    output reg  object_detected,   // 1 = object detected
    output reg  detection_pulse    // 1-cycle pulse on new detection
);

    // Parameters
    parameter CLK_FREQ    = 50_000_000; // 50 MHz clock
    parameter DEBOUNCE_MS = 5;          // 5ms debounce time

    // Debounce counter limit
    parameter DEBOUNCE_LIMIT = (CLK_FREQ / 1000) * DEBOUNCE_MS; // 250,000 cycles

    // Internal signals
    reg ir_sync_0, ir_sync_1;       // Two-stage synchronizer (metastability)
    reg ir_debounced;
    reg ir_prev;

    reg [31:0] debounce_counter;
    reg stable_signal;

    // ──────────────────────────────────────────────
    // Stage 1: Two-stage synchronizer
    // ──────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ir_sync_0 <= 1'b1;
            ir_sync_1 <= 1'b1;
        end else begin
            ir_sync_0 <= ir_in;
            ir_sync_1 <= ir_sync_0;
        end
    end

    // ──────────────────────────────────────────────
    // Stage 2: Debounce logic
    // ──────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debounce_counter <= 0;
            ir_debounced     <= 1'b1;
        end else begin
            if (ir_sync_1 == ir_debounced) begin
                // Signal stable — reset counter
                debounce_counter <= 0;
            end else begin
                // Signal changed — start counting
                debounce_counter <= debounce_counter + 1;

                if (debounce_counter >= DEBOUNCE_LIMIT - 1) begin
                    debounce_counter <= 0;
                    ir_debounced     <= ir_sync_1; // Accept new stable value
                end
            end
        end
    end

    // ──────────────────────────────────────────────
    // Stage 3: Object detection output
    // NOTE: MH Flying Fish OUT = LOW when object detected
    //       OUT = HIGH when no object
    // ──────────────────────────────────────────────
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            object_detected <= 1'b0;
            detection_pulse <= 1'b0;
            ir_prev         <= 1'b1;
        end else begin
            ir_prev <= ir_debounced;

            // Active LOW: invert signal for intuitive output
            object_detected <= ~ir_debounced;

            // Rising edge on detection (LOW->HIGH on raw = object just detected)
            if (ir_prev == 1'b1 && ir_debounced == 1'b0)
                detection_pulse <= 1'b1;
            else
                detection_pulse <= 1'b0;
        end
    end

endmodule
