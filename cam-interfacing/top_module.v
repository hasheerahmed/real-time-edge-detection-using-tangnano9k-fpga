`timescale 1ns / 1ps

// =============================================================================
// top_module.v  —  OV7670 → Tang Nano 9K → HDMI
//
// Clock plan (single PLL):
//   27 MHz crystal → rPLL → 125 MHz (TMDS serial clock)
//   125 MHz → CLKDIV(5) → 25 MHz  (pixel clock, exact for 640×480@60 Hz)
//
// 640×480 @ 60 Hz pixel clock MUST be 25.175 MHz ≈ 25 MHz (close enough).
// Using 27 MHz as pixel clock makes the display run ~8% too fast → stripes.
// =============================================================================

module top_module (
    input  wire       clk,          // 27 MHz onboard crystal
    input  wire       rst_n,        // Active-low reset (S1 button)

    // OV7670 camera
    input  wire       cam_pclk,
    input  wire       cam_href,
    input  wire       cam_vsync,
    input  wire [7:0] cam_data,
    inout  wire       cam_sda,
    inout  wire       cam_scl,
    output wire       cam_xclk,

    // HDMI (TMDS differential pairs)
    output wire       hdmi_clk_p,
    output wire       hdmi_clk_n,
    output wire [2:0] hdmi_data_p,
    output wire [2:0] hdmi_data_n,

    // LEDs (active-low on Tang Nano 9K)
    output wire [5:0] led
);

    // =========================================================================
    // Clock generation
    //   One rPLL:  27 MHz → 125 MHz   (FBDIV=19, IDIV=0, ODIV=4 → 540/4=135...
    //   Actually:  FBDIV+1=? Let Gowin IDE generate this for you.
    //   We ship a wrapper pll_125M.v with the correct defparams.
    //   125 MHz / 5 = 25 MHz via CLKDIV.
    // =========================================================================
    wire clk_125M, locked;
    wire clk_pixel;   // 25 MHz

    pll_125M pll_inst (
        .clkin (clk),
        .clkout(clk_125M),
        .lock  (locked)
    );

    // CLKDIV primitive: divides HCLKIN by DIV_MODE
    // 125 MHz ÷ 5 = 25 MHz
    CLKDIV #(.DIV_MODE("5"), .GSREN("false")) clkdiv_inst (
        .HCLKIN(clk_125M),
        .RESETN(locked),
        .CALIB (1'b1),
        .CLKOUT(clk_pixel)
    );

    wire sys_rst_n = rst_n & locked;

    // =========================================================================
    // Camera interface
    // =========================================================================
    wire [15:0] cam_pixel;
    wire        cam_pixel_valid;   // 1-cycle pulse in cam_pclk domain

    camera_interface cam_inst (
        .clk_27m   (clk),
        .rst_n     (sys_rst_n),
        .cam_pclk  (cam_pclk),
        .cam_href  (cam_href),
        .cam_vsync (cam_vsync),
        .cam_data  (cam_data),
        .cam_sda   (cam_sda),
        .cam_scl   (cam_scl),
        .cam_xclk  (cam_xclk),
        .pixel_data (cam_pixel),
        .pixel_valid(cam_pixel_valid),
        .led       (led[3:0])
    );

    // =========================================================================
    // HDMI interface
    // =========================================================================
    hdmi_interface hdmi_inst (
        .clk_wr   (cam_pclk),       // FIFO write clock = camera PCLK
        .clk_pixel(clk_pixel),      // 25 MHz pixel clock
        .clk_tmds (clk_125M),       // 125 MHz TMDS serial clock
        .rst_n    (sys_rst_n),
        .rgb_in   (cam_pixel),
        .wr_en    (cam_pixel_valid),
        .hdmi_clk_p (hdmi_clk_p),
        .hdmi_clk_n (hdmi_clk_n),
        .hdmi_data_p(hdmi_data_p),
        .hdmi_data_n(hdmi_data_n)
    );

    // =========================================================================
    // Debug LEDs
    // =========================================================================
    assign led[4] = ~locked;       // ON when PLL has no lock
    assign led[5] = ~cam_vsync;    // blinks at frame rate

endmodule
