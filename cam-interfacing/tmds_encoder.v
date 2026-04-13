`timescale 1ns / 1ps

module tmds_encoder(
    input  wire       clk,
    input  wire [7:0] VD,  
    input  wire [1:0] CD,  
    input  wire       VDE,       
    output reg  [9:0] TMDS = 0
);
    wire [3:0] n1d = VD[0] + VD[1] + VD[2] + VD[3] + VD[4] + VD[5] + VD[6] + VD[7];
    
    wire [8:0] q_m;
    assign q_m[0] = VD[0];
    assign q_m[1] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? ~(q_m[0] ^ VD[1]) : (q_m[0] ^ VD[1]);
    assign q_m[2] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? ~(q_m[1] ^ VD[2]) : (q_m[1] ^ VD[2]);
    assign q_m[3] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? ~(q_m[2] ^ VD[3]) : (q_m[2] ^ VD[3]);
    assign q_m[4] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? ~(q_m[3] ^ VD[4]) : (q_m[3] ^ VD[4]);
    assign q_m[5] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? ~(q_m[4] ^ VD[5]) : (q_m[4] ^ VD[5]);
    assign q_m[6] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? ~(q_m[5] ^ VD[6]) : (q_m[5] ^ VD[6]);
    assign q_m[7] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? ~(q_m[6] ^ VD[7]) : (q_m[6] ^ VD[7]);
    assign q_m[8] = (n1d > 4 || (n1d == 4 && VD[0] == 0)) ? 1'b0 : 1'b1;

    reg [4:0] cnt = 0;
    wire [3:0] n1q_m = q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7];
    wire [3:0] n0q_m = 8 - n1q_m;
    
    always @(posedge clk) begin
        if (VDE) begin
            if (cnt == 0 || n1q_m == n0q_m) begin
                TMDS[9]   <= ~q_m[8];
                TMDS[8]   <= q_m[8];
                TMDS[7:0] <= (q_m[8]) ? q_m[7:0] : ~q_m[7:0];
                cnt <= (q_m[8]) ? (cnt + n1q_m - n0q_m) : (cnt + n0q_m - n1q_m);
            end else begin
                if ((!cnt[4] && n1q_m > n0q_m) || (cnt[4] && n0q_m > n1q_m)) begin
                    TMDS[9]   <= 1'b1;
                    TMDS[8]   <= q_m[8];
                    TMDS[7:0] <= ~q_m[7:0];
                    cnt <= cnt + 2*q_m[8] + n0q_m - n1q_m;
                end else begin
                    TMDS[9]   <= 1'b0;
                    TMDS[8]   <= q_m[8];
                    TMDS[7:0] <= q_m[7:0];
                    cnt <= cnt - 2*(~q_m[8]) + n1q_m - n0q_m;
                end
            end
        end else begin
            cnt <= 0;
            case (CD)
                2'b00: TMDS <= 10'b1101010100;
                2'b01: TMDS <= 10'b0010101011;
                2'b10: TMDS <= 10'b0101010100;
                2'b11: TMDS <= 10'b1010101011;
            endcase
        end
    end
endmodule
