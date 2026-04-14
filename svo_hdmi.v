`timescale 1ns / 1ps
`include "svo_defines.vh"

module svo_hdmi #(
    `SVO_DEFAULT_PARAMS
) (
    input clk,
    input resetn,

    // video clocks
    input clk_pixel,
    input clk_5x_pixel,
    input locked,

    // --- AXI Stream inputs from the camera's PSRAM buffer ---
    input         in_axis_tvalid,
    output        in_axis_tready,
    input  [23:0] in_axis_tdata,
    input  [0:0]  in_axis_tuser,

    // output signals
    output       tmds_clk_n,
    output       tmds_clk_p,
    output [2:0] tmds_d_n,
    output [2:0] tmds_d_p
);

    wire video_enc_tvalid;
    wire video_enc_tready;
    wire [23:0] video_enc_tdata;
    wire [3:0] video_enc_tuser;

    wire clk_pixel_resetn = resetn & locked;

    // The HDMI Encoder translates the AXI Stream into TMDS symbols
    svo_enc #(
        `SVO_PASS_PARAMS
    ) svo_enc_inst (
        .clk(clk_pixel),
        .resetn(clk_pixel_resetn),
        .in_axis_tvalid(in_axis_tvalid),
        .in_axis_tready(in_axis_tready),
        .in_axis_tdata(in_axis_tdata),
        .in_axis_tuser(in_axis_tuser),
        .out_axis_tvalid(video_enc_tvalid),
        .out_axis_tready(video_enc_tready),
        .out_axis_tdata(video_enc_tdata),
        .out_axis_tuser(video_enc_tuser)
    );

    assign video_enc_tready = 1;

    wire [2:0] tmds_d0, tmds_d1, tmds_d2, tmds_d3, tmds_d4;
    wire [2:0] tmds_d5, tmds_d6, tmds_d7, tmds_d8, tmds_d9;
    wire [2:0] tmds_d;

    svo_tmds svo_tmds_0 (
        .clk(clk_pixel),
        .resetn(clk_pixel_resetn),
        .de(!video_enc_tuser[3]),
        .ctrl(video_enc_tuser[2:1]),
        .din(video_enc_tdata[23:16]),
        .dout({tmds_d9[0], tmds_d8[0], tmds_d7[0], tmds_d6[0], tmds_d5[0],
               tmds_d4[0], tmds_d3[0], tmds_d2[0], tmds_d1[0], tmds_d0[0]})
    );

    svo_tmds svo_tmds_1 (
        .clk(clk_pixel),
        .resetn(clk_pixel_resetn),
        .de(!video_enc_tuser[3]),
        .ctrl(2'b0),
        .din(video_enc_tdata[15:8]),
        .dout({tmds_d9[1], tmds_d8[1], tmds_d7[1], tmds_d6[1], tmds_d5[1],
               tmds_d4[1], tmds_d3[1], tmds_d2[1], tmds_d1[1], tmds_d0[1]})
    );

    svo_tmds svo_tmds_2 (
        .clk(clk_pixel),
        .resetn(clk_pixel_resetn),
        .de(!video_enc_tuser[3]),
        .ctrl(2'b0),
        .din(video_enc_tdata[7:0]),
        .dout({tmds_d9[2], tmds_d8[2], tmds_d7[2], tmds_d6[2], tmds_d5[2],
               tmds_d4[2], tmds_d3[2], tmds_d2[2], tmds_d1[2], tmds_d0[2]})
    );
     
    OSER10 tmds_serdes [2:0] (
        .Q(tmds_d),
        .D0(tmds_d0),
        .D1(tmds_d1),
        .D2(tmds_d2),
        .D3(tmds_d3),
        .D4(tmds_d4),
        .D5(tmds_d5),
        .D6(tmds_d6),
        .D7(tmds_d7),
        .D8(tmds_d8),
        .D9(tmds_d9),
        .PCLK(clk_pixel),
        .FCLK(clk_5x_pixel),
        .RESET(~clk_pixel_resetn)
    );
   
    ELVDS_OBUF tmds_bufds [3:0] (
        .I({clk_pixel, tmds_d}),
        .O({tmds_clk_p, tmds_d_p}),
        .OB({tmds_clk_n, tmds_d_n})
    );

endmodule
