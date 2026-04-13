`timescale 1ns / 1ps

module hdmi_interface (
    input  wire       clk_cam,      // 100 MHz (write side)
    input  wire       clk_pixel,    // 25 MHz (pixel clock)
    input  wire       clk_tmds,     // 125 MHz (serial clock)
    input  wire       rst_n,
    input  wire [15:0] rgb_in,      // RGB565 from camera FIFO
    input  wire       wr_en,        // write enable from camera side
    output wire       rd_en,        // read enable for camera FIFO
    output wire       hdmi_hs, hdmi_vs,
    output wire       hdmi_de,
    output wire       hdmi_clk_p, hdmi_clk_n,
    output wire [2:0] hdmi_data_p, hdmi_data_n
);

    // -----------------------------------------------------------------
    // Pixel generator (640x480 @ 60 Hz)
    // -----------------------------------------------------------------
    reg [11:0] hcnt = 0, vcnt = 0;
    reg        hs, vs, de;
    wire       active = (hcnt < 640 && vcnt < 480);

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= 0;
            vcnt <= 0;
            hs   <= 1;
            vs   <= 1;
            de   <= 0;
        end else begin
            if (hcnt == 799) begin
                hcnt <= 0;
                if (vcnt == 524) vcnt <= 0;
                else vcnt <= vcnt + 1;
            end else begin
                hcnt <= hcnt + 1;
            end

            // Horizontal sync (96 clocks)
            hs <= (hcnt >= 656 && hcnt < 752) ? 0 : 1;
            // Vertical sync (2 lines)
            vs <= (vcnt >= 490 && vcnt < 492) ? 0 : 1;
            de <= active;
        end
    end

    // -----------------------------------------------------------------
    // Async FIFO to cross from 100 MHz to 25 MHz
    // -----------------------------------------------------------------
    wire [15:0] fifo_dout;
    wire        fifo_empty, fifo_rd_en;
    reg         rd_en_int;

    asyn_fifo #(.DATA_WIDTH(16), .FIFO_DEPTH_WIDTH(10)) fifo (
        .rst_n(rst_n),
        .clk_write(clk_cam),
        .clk_read(clk_pixel),
        .write(wr_en),
        .read(fifo_rd_en),
        .data_write(rgb_in),
        .data_read(fifo_dout),
        .full(),
        .empty(fifo_empty),
        .data_count_w(),
        .data_count_r()
    );

    assign fifo_rd_en = active & !fifo_empty;
    assign rd_en = wr_en;   // Actually rd_en for camera FIFO is not needed here; we use separate FIFO.

        // -----------------------------------------------------------------
    // REAL TMDS encoder with DC Balancing
    // -----------------------------------------------------------------
    wire [9:0] tmds_r, tmds_g, tmds_b;

    // Blue channel MUST carry HSYNC and VSYNC on the control lines!
    tmds_encoder enc_b (
        .clk(clk_pixel),
        .VD({fifo_dout[4:0], 3'b000}),
        .CD({vs, hs}),
        .VDE(active && !fifo_empty),
        .TMDS(tmds_b)
    );

    // Green channel
    tmds_encoder enc_g (
        .clk(clk_pixel),
        .VD({fifo_dout[10:5], 2'b00}),
        .CD(2'b00),
        .VDE(active && !fifo_empty),
        .TMDS(tmds_g)
    );

    // Red channel
    tmds_encoder enc_r (
        .clk(clk_pixel),
        .VD({fifo_dout[15:11], 3'b000}),
        .CD(2'b00),
        .VDE(active && !fifo_empty),
        .TMDS(tmds_r)
    );

   // -----------------------------------------------------------------
    // Gowin Hardware Serializers (10-to-1)
    // -----------------------------------------------------------------
    wire ser_r, ser_g, ser_b, ser_clk;

    // Red Channel (HDMI Data 2)
    OSER10 serializer_r (
        .Q(ser_r),
        .D0(tmds_r[0]), .D1(tmds_r[1]), .D2(tmds_r[2]), .D3(tmds_r[3]), .D4(tmds_r[4]),
        .D5(tmds_r[5]), .D6(tmds_r[6]), .D7(tmds_r[7]), .D8(tmds_r[8]), .D9(tmds_r[9]),
        .PCLK(clk_pixel), // 27 MHz
        .FCLK(clk_tmds),  // 135 MHz
        .RESET(~rst_n)
    );

    // Green Channel (HDMI Data 1)
    OSER10 serializer_g (
        .Q(ser_g),
        .D0(tmds_g[0]), .D1(tmds_g[1]), .D2(tmds_g[2]), .D3(tmds_g[3]), .D4(tmds_g[4]),
        .D5(tmds_g[5]), .D6(tmds_g[6]), .D7(tmds_g[7]), .D8(tmds_g[8]), .D9(tmds_g[9]),
        .PCLK(clk_pixel), 
        .FCLK(clk_tmds),  
        .RESET(~rst_n)
    );

    // Blue Channel (HDMI Data 0)
    OSER10 serializer_b (
        .Q(ser_b),
        .D0(tmds_b[0]), .D1(tmds_b[1]), .D2(tmds_b[2]), .D3(tmds_b[3]), .D4(tmds_b[4]),
        .D5(tmds_b[5]), .D6(tmds_b[6]), .D7(tmds_b[7]), .D8(tmds_b[8]), .D9(tmds_b[9]),
        .PCLK(clk_pixel), 
        .FCLK(clk_tmds),  
        .RESET(~rst_n)
    );

    // Clock Channel (Sends a constant 10-bit pattern: 5 highs, 5 lows)
    OSER10 serializer_clk (
        .Q(ser_clk),
        .D0(1'b1), .D1(1'b1), .D2(1'b1), .D3(1'b1), .D4(1'b1),
        .D5(1'b0), .D6(1'b0), .D7(1'b0), .D8(1'b0), .D9(1'b0),
        .PCLK(clk_pixel), 
        .FCLK(clk_tmds),  
        .RESET(~rst_n)
    );

    //     // -----------------------------------------------------------------
    // Gowin Emulated LVDS Output Buffers (Differential Pairs)
    // -----------------------------------------------------------------
    
    // HDMI Clock
    ELVDS_OBUF obuf_clk (
        .O(hdmi_clk_p),
        .OB(hdmi_clk_n),
        .I(ser_clk)
    );

    // HDMI Data 2 (Red)
    ELVDS_OBUF obuf_d2 (
        .O(hdmi_data_p[2]),
        .OB(hdmi_data_n[2]),
        .I(ser_r)
    );

    // HDMI Data 1 (Green)
    ELVDS_OBUF obuf_d1 (
        .O(hdmi_data_p[1]),
        .OB(hdmi_data_n[1]),
        .I(ser_g)
    );

    // HDMI Data 0 (Blue)
    ELVDS_OBUF obuf_d0 (
        .O(hdmi_data_p[0]),
        .OB(hdmi_data_n[0]),
        .I(ser_b)
    );

endmodule
