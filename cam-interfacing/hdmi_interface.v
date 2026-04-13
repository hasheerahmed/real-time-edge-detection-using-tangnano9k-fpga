`timescale 1ns / 1ps

// =============================================================================
// hdmi_interface.v  —  640×480 @ 60 Hz HDMI output
//
// Clock domains:
//   clk_wr    = cam_pclk  (~13.5 MHz)  — async FIFO write side
//   clk_pixel = 25 MHz                 — timing generator + TMDS encoder
//   clk_tmds  = 125 MHz                — OSER10 serialiser fast clock
//
// Pixel clock math (640×480 @ 60 Hz):
//   H total = 640 + 16 (front) + 96 (sync) + 48 (back) = 800
//   V total = 480 + 10 (front) +  2 (sync) + 33 (back) = 525
//   Pixel clock = 800 × 525 × 60 = 25,200,000 Hz ≈ 25 MHz  ✓
//   (27 MHz gives 800×525×(27M/800/525) = 64.3 fps — monitor can't lock → stripes)
//
// FIFO:
//   Write side: camera PCLK domain, write when pixel_valid pulses
//   Read side:  pixel clock domain, read only during active video AND when
//               the FIFO has data.  Output black pixels when FIFO is empty
//               so timing is never disturbed.
// =============================================================================

module hdmi_interface (
    input  wire        clk_wr,       // cam_pclk — FIFO write clock
    input  wire        clk_pixel,    // 25 MHz pixel clock
    input  wire        clk_tmds,     // 125 MHz TMDS serial clock
    input  wire        rst_n,

    input  wire [15:0] rgb_in,       // RGB565 from camera (cam_pclk domain)
    input  wire        wr_en,        // pixel valid pulse (cam_pclk domain)

    output wire        hdmi_clk_p, hdmi_clk_n,
    output wire [2:0]  hdmi_data_p, hdmi_data_n
);

    // =========================================================================
    // 640×480 @ 60 Hz timing generator  (clk_pixel = 25 MHz)
    // =========================================================================
    // Horizontal:  640 active + 16 FP + 96 sync + 48 BP  = 800 total
    // Vertical:    480 active + 10 FP +  2 sync + 33 BP  = 525 total
    // H-sync: active LOW during columns 656..751
    // V-sync: active LOW during rows    490..491

    reg [9:0] hcnt = 0;
    reg [9:0] vcnt = 0;
    reg       hs, vs, de;

    wire h_active = (hcnt < 640);
    wire v_active = (vcnt < 480);
    wire active   = h_active & v_active;

    always @(posedge clk_pixel or negedge rst_n) begin
        if (!rst_n) begin
            hcnt <= 0; vcnt <= 0;
            hs <= 1; vs <= 1; de <= 0;
        end else begin
            // Counters
            if (hcnt == 799) begin
                hcnt <= 0;
                vcnt <= (vcnt == 524) ? 0 : vcnt + 1'b1;
            end else begin
                hcnt <= hcnt + 1'b1;
            end
            // Sync pulses (active low, per CEA-861 for 640×480@60)
            hs <= ~(hcnt >= 656 && hcnt < 752);
            vs <= ~(vcnt >= 490 && vcnt < 492);
            de <= active;
        end
    end

    // =========================================================================
    // Async FIFO: camera PCLK → pixel clock domain
    // Depth 1024 (10-bit address) — enough for ~1 line at VGA rates
    // =========================================================================
    wire [15:0] fifo_dout;
    wire        fifo_empty, fifo_full;

    asyn_fifo #(
        .DATA_WIDTH      (16),
        .FIFO_DEPTH_WIDTH(10)
    ) pixel_fifo (
        .rst_n      (rst_n),
        .clk_write  (clk_wr),
        .clk_read   (clk_pixel),
        .write      (wr_en & ~fifo_full),
        .read       (fifo_rd_en),
        .data_write (rgb_in),
        .data_read  (fifo_dout),
        .full       (fifo_full),
        .empty      (fifo_empty),
        .data_count_w(),
        .data_count_r()
    );

    // Read from FIFO only during active video AND when data is available.
    // This is the FIX: do NOT read on every active pixel — only when FIFO has data.
    assign fifo_rd_en = active & ~fifo_empty;

    // =========================================================================
    // Pixel mux: use FIFO data when available; black when FIFO empty
    // The timing (de/hs/vs) is NEVER gated — it always runs free
    // =========================================================================
    wire [7:0] px_r = active ? {fifo_dout[15:11], 3'b000} : 8'h00;
    wire [7:0] px_g = active ? {fifo_dout[10:5],  2'b00}  : 8'h00;
    wire [7:0] px_b = active ? {fifo_dout[4:0],   3'b000} : 8'h00;

    // =========================================================================
    // TMDS encoders  (clk_pixel domain)
    // Blue channel carries sync bits in the blanking period
    // =========================================================================
    wire [9:0] tmds_r, tmds_g, tmds_b;

    tmds_encoder enc_r (
        .clk (clk_pixel), .VD(px_r), .CD(2'b00),    .VDE(de), .TMDS(tmds_r));
    tmds_encoder enc_g (
        .clk (clk_pixel), .VD(px_g), .CD(2'b00),    .VDE(de), .TMDS(tmds_g));
    tmds_encoder enc_b (
        .clk (clk_pixel), .VD(px_b), .CD({vs, hs}), .VDE(de), .TMDS(tmds_b));

    // =========================================================================
    // 10:1 serialisers (Gowin OSER10)
    // PCLK = 25 MHz, FCLK = 125 MHz  (ratio exactly 5:1)
    // =========================================================================
    wire ser_r, ser_g, ser_b, ser_clk;

    OSER10 ser_R (
        .Q(ser_r),
        .D0(tmds_r[0]),.D1(tmds_r[1]),.D2(tmds_r[2]),.D3(tmds_r[3]),.D4(tmds_r[4]),
        .D5(tmds_r[5]),.D6(tmds_r[6]),.D7(tmds_r[7]),.D8(tmds_r[8]),.D9(tmds_r[9]),
        .PCLK(clk_pixel), .FCLK(clk_tmds), .RESET(~rst_n)
    );
    OSER10 ser_G (
        .Q(ser_g),
        .D0(tmds_g[0]),.D1(tmds_g[1]),.D2(tmds_g[2]),.D3(tmds_g[3]),.D4(tmds_g[4]),
        .D5(tmds_g[5]),.D6(tmds_g[6]),.D7(tmds_g[7]),.D8(tmds_g[8]),.D9(tmds_g[9]),
        .PCLK(clk_pixel), .FCLK(clk_tmds), .RESET(~rst_n)
    );
    OSER10 ser_B (
        .Q(ser_b),
        .D0(tmds_b[0]),.D1(tmds_b[1]),.D2(tmds_b[2]),.D3(tmds_b[3]),.D4(tmds_b[4]),
        .D5(tmds_b[5]),.D6(tmds_b[6]),.D7(tmds_b[7]),.D8(tmds_b[8]),.D9(tmds_b[9]),
        .PCLK(clk_pixel), .FCLK(clk_tmds), .RESET(~rst_n)
    );
    // Clock channel: constant 5-high-5-low pattern
    OSER10 ser_CLK (
        .Q(ser_clk),
        .D0(1'b1),.D1(1'b1),.D2(1'b1),.D3(1'b1),.D4(1'b1),
        .D5(1'b0),.D6(1'b0),.D7(1'b0),.D8(1'b0),.D9(1'b0),
        .PCLK(clk_pixel), .FCLK(clk_tmds), .RESET(~rst_n)
    );

    // =========================================================================
    // Emulated LVDS output buffers (Gowin ELVDS_OBUF)
    // =========================================================================
    ELVDS_OBUF obuf_clk (.I(ser_clk), .O(hdmi_clk_p), .OB(hdmi_clk_n));
    ELVDS_OBUF obuf_d0  (.I(ser_b),   .O(hdmi_data_p[0]), .OB(hdmi_data_n[0]));
    ELVDS_OBUF obuf_d1  (.I(ser_g),   .O(hdmi_data_p[1]), .OB(hdmi_data_n[1]));
    ELVDS_OBUF obuf_d2  (.I(ser_r),   .O(hdmi_data_p[2]), .OB(hdmi_data_n[2]));

endmodule
