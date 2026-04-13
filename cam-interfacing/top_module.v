`timescale 1ns / 1ps

module top_module (
    input  wire       clk,          // 27MHz onboard crystal
    input  wire       rst_n,        // S1 button for reset
    input  wire [3:0] key,          
    
    // Camera interface (OV2640)
    input  wire       cam_pclk,
    input  wire       cam_href,
    input  wire       cam_vsync,
    input  wire [7:0] cam_data,
    inout  wire       cam_sda,
    inout  wire       cam_scl,
    output wire       cam_xclk,
    
    // HDMI output
    output wire       hsync,
    output wire       vsync,
    output wire       hdmi_clk_p,
    output wire       hdmi_clk_n,
    output wire [2:0] hdmi_data_p,
    output wire [2:0] hdmi_data_n,
    
    // LEDs
    output wire [5:0] led
);

        // ============================================
    // Clock Generation (PLLs & Dividers)
    // ============================================
    wire clk_108M, clk_135M;
    wire locked_108, locked_135;

    pll_108M pll1 (.clkin(clk), .clkout(clk_108M), .lock(locked_108));
    pll_135M pll2 (.clkin(clk), .clkout(clk_135M), .lock(locked_135));
    
    // GENERATE A PERFECTLY PHASE-ALIGNED 27MHz PIXEL CLOCK
    wire clk_pixel_aligned;
    CLKDIV #(.DIV_MODE("5")) clk_div_inst (
        .RESETN(locked_135),
        .HCLKIN(clk_135M),        // Divide the 135MHz clock by 5
        .CLKOUT(clk_pixel_aligned) // Output perfectly aligned 27MHz
    );

    wire sys_reset_n = rst_n & locked_108 & locked_135;

    // ============================================
    // Camera Interface Instance
    // ============================================
    wire [15:0] pixel_data;
    wire        cam_rd_en;
    wire hdmi_rd_en_unused;
    wire [9:0]  fifo_count;
    wire [3:0]  cam_led_status;

    camera_interface cam_inst (
        .clk(clk),               // 27MHz base input for the 24MHz PLL inside
        .clk_100(clk_108M),      // Now running at 108MHz (handles FIFO writes)
        .rst_n(sys_reset_n),
        .key(key),
        .rd_en(cam_rd_en),
        .data_count_r(fifo_count),
        .dout(pixel_data),
        .cmos_pclk(cam_pclk), 
        .cmos_href(cam_href), 
        .cmos_vsync(cam_vsync),
        .cmos_db(cam_data),
        .cmos_sda(cam_sda), 
        .cmos_scl(cam_scl),
        .cmos_xclk(cam_xclk),
        .led(cam_led_status)
    );

    // ============================================
    // HDMI Interface Instance
    // ============================================
    wire video_active; 
    
        hdmi_interface hdmi_inst (
        .clk_cam(clk_108M),      
        .clk_pixel(clk_pixel_aligned), // USE THE NEW ALIGNED CLOCK HERE!
        .clk_tmds(clk_135M),     
        .rst_n(sys_reset_n),
     
     // 135MHz serial clock (exactly 5x 27MHz)
        .rgb_in(pixel_data),
        .wr_en(cam_rd_en),            // BUG 2 fix
        .rd_en(hdmi_rd_en_unused), 
        .hdmi_hs(hsync), 
        .hdmi_vs(vsync),
        .hdmi_de(video_active),
        .hdmi_clk_p(hdmi_clk_p), 
        .hdmi_clk_n(hdmi_clk_n),
        .hdmi_data_p(hdmi_data_p), 
        .hdmi_data_n(hdmi_data_n)
    );

    // ============================================
    // LED Assignments (Active Low)
    // ============================================
    assign led[3:0] = ~cam_led_status; 
    assign led[4]   = ~sys_reset_n;     // LED5 ON if PLLs lose lock
    assign led[5]   = ~cam_vsync;       // LED6 blinks on new frame

endmodule
