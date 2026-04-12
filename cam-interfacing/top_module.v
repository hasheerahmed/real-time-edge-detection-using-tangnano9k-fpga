`timescale 1ns / 1ps

module top_module (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [3:0] key,
    input  wire       cam_pclk,
    input  wire       cam_href,
    input  wire       cam_vsync,
    input  wire [7:0] cam_data,
    inout  wire       cam_sda,
    inout  wire       cam_scl,
    output wire       cam_xclk,
    output wire       hsync,
    output wire       vsync,
    output wire       hdmi_clk_p,
    output wire       hdmi_clk_n,
    output wire [2:0] hdmi_data_p,
    output wire [2:0] hdmi_data_n,
    output wire [3:0] led
);

    // Simple test - blink LED to verify FPGA works
    reg [24:0] counter = 0;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            counter <= 0;
        else
            counter <= counter + 1;
    end
    
    // ============================================
    // LED Assignments
    // ============================================
    
    // LED0: Blink test (FPGA working indicator)
    assign led[0] = counter[24];
    
    // ============================================
    // Camera Connection Test (ADD THESE LINES)
    // ============================================
    // LED1: Shows camera data bit 0 (should flicker if camera is sending data)
    assign led[1] = cam_data[0];
    
    // LED2: Shows camera pixel clock (should blink at ~24-48MHz - looks constantly on)
    assign led[2] = cam_pclk;
    
    // LED3: Shows camera vertical sync (should blink once per frame - about every 33ms)
    assign led[3] = cam_vsync;
    
    // Simple pass-through for testing
    assign cam_xclk = clk;
    assign hsync = cam_href;
    assign vsync = cam_vsync;
    
    // Simple HDMI output (just for testing)
    assign hdmi_clk_p = clk;
    assign hdmi_clk_n = ~clk;
    assign hdmi_data_p[0] = cam_data[0];
    assign hdmi_data_n[0] = ~cam_data[0];
    assign hdmi_data_p[1] = cam_data[1];
    assign hdmi_data_n[1] = ~cam_data[1];
    assign hdmi_data_p[2] = cam_data[2];
    assign hdmi_data_n[2] = ~cam_data[2];

endmodule
