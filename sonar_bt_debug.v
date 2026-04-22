module sonar_bt_debug(
    input        clk_50,
    input  [4:0] fsm_state,
    input  [6:0] dist_l,
    input  [6:0] dist_r,
    input  [6:0] dist_f,
    input  [8:0] heading,
    input signed [15:0] heading_adjust,
    input  [3:0] steer,
    input        ir3,
    input  [3:0] speed_l,
    input  [3:0] speed_r,
    output reg   tx
);
    // 50MHz / 115200 = 434
    localparam BAUD_DIV = 434;
    
    reg [25:0] timer = 0;
    reg trigger = 0;
    
    always @(posedge clk_50) begin
        if (timer >= 12_500_000) begin // 4 times a second
            timer <= 0;
            trigger <= 1;
        end else begin
            timer <= timer + 1;
            trigger <= 0;
        end
    end

    // ASCII conversion helpers
    function [7:0] t100(input [8:0] v); t100 = (v / 100) + 8'd48; endfunction
    function [7:0] t10 (input [8:0] v); t10  = ((v / 10) % 10) + 8'd48; endfunction
    function [7:0] t1  (input [8:0] v); t1   = (v % 10) + 8'd48; endfunction
    function [7:0] tsign(input signed [15:0] v); tsign = (v < 0) ? 8'd45 : 8'd43; endfunction
    function [7:0] tabs10(input signed [15:0] v); tabs10 = (v < 0 ? (-v)/10 : v/10) % 10 + 8'd48; endfunction
    function [7:0] tabs1(input signed [15:0] v);  tabs1  = (v < 0 ? (-v) : v) % 10 + 8'd48; endfunction

    reg [7:0] msg [0:53];
    always @(posedge clk_50) begin
        if (trigger) begin
            msg[0] <= "S"; msg[1] <= "T"; msg[2] <= ":";
            msg[3] <= t10({4'd0, fsm_state}); msg[4] <= t1({4'd0, fsm_state});
            msg[5] <= " "; msg[6] <= "F"; msg[7] <= ":";
            msg[8] <= t100({2'd0, dist_f}); msg[9] <= t10({2'd0, dist_f}); msg[10]<= t1({2'd0, dist_f});
            msg[11]<= " "; msg[12]<= "L"; msg[13]<= ":";
            msg[14]<= t100({2'd0, dist_l}); msg[15]<= t10({2'd0, dist_l}); msg[16]<= t1({2'd0, dist_l});
            msg[17]<= " "; msg[18]<= "R"; msg[19]<= ":";
            msg[20]<= t100({2'd0, dist_r}); msg[21]<= t10({2'd0, dist_r}); msg[22]<= t1({2'd0, dist_r});
            msg[23]<= " "; msg[24]<= "H"; msg[25]<= ":";
            msg[26]<= t100(heading); msg[27]<= t10(heading); msg[28]<= t1(heading);
            msg[29]<= " "; msg[30]<= "A"; msg[31]<= ":";
            msg[32]<= tsign(heading_adjust); msg[33]<= tabs10(heading_adjust); msg[34]<= tabs1(heading_adjust);
            msg[35]<= " "; msg[36]<= "S"; msg[37]<= ":";
            msg[38]<= t1({5'd0, steer});
            msg[39]<= " "; msg[40]<= "I"; msg[41]<= "3"; msg[42]<= ":";
            msg[43]<= ir3 ? "1" : "0";
            msg[44]<= " "; msg[45]<= "M"; msg[46]<= ":";
            msg[47]<= t10({5'd0, speed_l}); msg[48]<= t1({5'd0, speed_l});
            msg[49]<= ",";
            msg[50]<= t10({5'd0, speed_r}); msg[51]<= t1({5'd0, speed_r});
            msg[52]<= 8'h0D; msg[53]<= 8'h0A; // \r \n
        end
    end

    localparam S_IDLE = 0, S_START = 1, S_DATA = 2, S_STOP = 3, S_NEXT = 4;
    reg [2:0] state = S_IDLE;
    reg [5:0] char_idx = 0;
    reg [2:0] bit_idx = 0;
    reg [8:0] baud_cnt = 0;
    reg [7:0] current_char = 0;

    initial tx = 1'b1;

    always @(posedge clk_50) begin
        case (state)
            S_IDLE: begin
                tx <= 1'b1;
                if (trigger) begin
                    char_idx <= 0;
                    state <= S_NEXT;
                end
            end
            S_NEXT: begin
                if (char_idx < 54) begin
                    current_char <= msg[char_idx];
                    char_idx <= char_idx + 1;
                    baud_cnt <= 0;
                    state <= S_START;
                end else begin
                    state <= S_IDLE;
                end
            end
            S_START: begin
                tx <= 1'b0;
                if (baud_cnt < BAUD_DIV - 1) baud_cnt <= baud_cnt + 1;
                else begin baud_cnt <= 0; bit_idx <= 0; state <= S_DATA; end
            end
            S_DATA: begin
                tx <= current_char[bit_idx];
                if (baud_cnt < BAUD_DIV - 1) baud_cnt <= baud_cnt + 1;
                else begin
                    baud_cnt <= 0;
                    if (bit_idx < 7) bit_idx <= bit_idx + 1;
                    else state <= S_STOP;
                end
            end
            S_STOP: begin
                tx <= 1'b1;
                if (baud_cnt < BAUD_DIV - 1) baud_cnt <= baud_cnt + 1;
                else begin baud_cnt <= 0; state <= S_NEXT; end
            end
            default: state <= S_IDLE;
        endcase
    end
endmodule
