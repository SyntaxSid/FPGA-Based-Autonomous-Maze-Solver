// ============================================================
//  ADC Controller — MCP3204 / compatible SPI ADC
//  Reads Channel 0, outputs 12-bit value + moisture status
//  adc_sck is driven externally (from ir3_servo_bt_subsystem)
//
//  moisture_status: 2'b01 = WET, 2'b10 = DRY
// ============================================================

module adc_controller(
    input  wire        dout,
    input  wire        adc_sck,
    output wire        adc_cs_n,
    output wire        din,
    output reg  [11:0] d_out_ch0,
    output reg  [7:0]  led_ind,
    output reg  [1:0]  moisture_status  // 2'b01=WET, 2'b10=DRY
);
    parameter MIN  = 1390;
    parameter MAX  = 1391;
    parameter STEP = (MAX - MIN) / 8;

    /*
      One read/write cycle of ADC is 16 bits (0..15).
      Address bits written on falling edges 2, 3, 4.
      Data read on rising edges 4..15.
    */

    reg [3:0] din_counter = 0;
    reg [3:0] sp_counter  = 0;
    reg       adc_cs      = 1;
    reg       din_temp    = 0;   // channel 0 → all address bits 0
    reg [11:0] dout_chx   = 0;

    // ── Write on negedge ────────────────────────────────────
    always @(negedge adc_sck) begin
        din_counter <= din_counter + 1;
        if (din_counter == 0)
            adc_cs <= !adc_cs;
    end

    // ── Read on posedge ─────────────────────────────────────
    always @(posedge adc_sck) begin
        if ((sp_counter >= 4) && (sp_counter <= 15))
            dout_chx[15 - sp_counter] <= dout;
        else
            dout_chx <= 0;
        sp_counter <= sp_counter + 1'b1;
    end

    // ── Latch complete reading ───────────────────────────────
    always @(posedge adc_sck) begin
        if ((sp_counter == 15) && (!adc_cs))
            d_out_ch0 <= dout_chx;
    end

    // ── LED bar (illuminates count LEDs proportional to value)
    always @(*) begin
        integer count;
        if (d_out_ch0 < MIN)
            count = 1;
        else if (d_out_ch0 >= MAX)
            count = 8;
        else
            count = (d_out_ch0 - MIN) / STEP + 1;
        led_ind = (8'hFF >> (8 - count));
    end

    // ── Moisture status ─────────────────────────────────────
    always @(*) begin
        if (d_out_ch0 < MIN)
            moisture_status = 2'b01;  // WET
        else
            moisture_status = 2'b10;  // DRY  (>= MIN  includes >= MAX)
    end

    assign adc_cs_n = adc_cs;
    assign din      = din_temp;

endmodule
