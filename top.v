`timescale 1ns / 1ps

module top(
    // Board Clocks and Reset
    input  wire       sys_clk,          // 27 MHz crystal
    input  wire       sys_rst_n,        // S1 button

    // OV7670 Camera Pins
    input  wire       video_clk_i,      // PCLK
    input  wire       h_sync_i,         // HREF
    input  wire       v_sync_i,         // VSYNC
    input  wire [7:0] cam_data_i,       // D0-D7
    inout  wire       master_sda,       // I2C
    inout  wire       master_scl,       // I2C
    output wire       cam_clk,          // XCLK (27 MHz out to camera)
    output wire       cam_pwdn,
    output wire       cam_reset,

    // HDMI Pins
    output wire       tmds_clk_p,
    output wire       tmds_clk_n,
    output wire [2:0] tmds_d_p,
    output wire [2:0] tmds_d_n,

    // Internal PSRAM Pins (Routed automatically by Gowin)
    output wire [1:0]  O_psram_ck,
    output wire [1:0]  O_psram_ck_n,
    inout  wire [1:0]  IO_psram_rwds,
    output wire [1:0]  O_psram_reset_n,
    inout  wire [15:0] IO_psram_dq,
    output wire [1:0]  O_psram_cs_n,

    // Debug LEDs (UNCOMMENTED!)
    output wire [2:0] status_leds,
    output wire       debug_led,
    output wire       led_out,
    output wire       led_out1
);

    // =========================================================
    // 1. CLOCK GENERATION
    // =========================================================
    
    // HDMI Clocks: 125 MHz (Serial) and 25 MHz (Pixel)
    wire clk_125M, clk_25M, hdmi_lock;
    Gowin_rPLL pll_hdmi (
        .clkin(sys_clk), 
        .clkout(clk_125M), 
        .lock(hdmi_lock)
    );
    
    Gowin_CLKDIV u_div_5 (
        .clkout(clk_25M), 
        .hclkin(clk_125M), 
        .resetn(hdmi_lock)
    );

    /// PSRAM Clock: 135 MHz (Corrected VCO parameters)
    wire clk_135M, psram_lock;
    rPLL #(
        .FCLKIN("27"), .IDIV_SEL(0), .FBDIV_SEL(19), .ODIV_SEL(4)
    ) pll_psram (
        .CLKIN(sys_clk), .CLKFB(1'b0), .RESET(1'b0), .RESET_P(1'b0),
        .FBDSEL(6'b0), .IDSEL(6'b0), .ODSEL(6'b0), .PSDA(4'b0),
        .DUTYDA(4'b0), .FDLY(4'b0),
        .CLKOUT(clk_135M), .LOCK(psram_lock)
    );
    wire system_ready = hdmi_lock & psram_lock;

    // =========================================================
    // 2. CAMERA INITIALIZATION
    // =========================================================
    wire config_done;
    CameraControl_TOP cam_ctrl (
        .sys_clk(sys_clk), 
        .sys_rst_n(sys_rst_n),
        .master_scl(master_scl), 
        .master_sda(master_sda),
        .led_out(), 
        .cam_reset(cam_reset),
        .cam_clk(cam_clk), 
        .cam_pwdn(cam_pwdn),
        .config_done(config_done)
    );

    // =========================================================
    // 3. PSRAM FRAME BUFFER PIPELINE
    // =========================================================
    
    // A. Capture pixels from camera
    wire load_read_rdy, load_command_valid;
    wire [9:0] load_mem_addr;
    wire [31:0] load_pixel_data;
    wire [1:0] load_command_data;
    wire mem_load_clk;

    CamPixelProcessor #(
        .FRAME_WIDTH(640), .FRAME_HEIGHT(480)
    ) pixel_processor (
        .clk_cam(video_clk_i), 
        .clk_mem(mem_load_clk),
        .reset_n(sys_rst_n), 
        .init(config_done & system_ready),
        .mem_controller_rdy(load_read_rdy), 
        .mem_addr(load_mem_addr),
        .v_sync(v_sync_i), 
        .h_ref(h_sync_i), 
        .cam_data(cam_data_i),
        .pixel_data(load_pixel_data), 
        .command_data(load_command_data),
        .command_data_valid(load_command_valid)
    );

    // B. Physical PSRAM Interface Core
    wire clk_67_5M; // 67.5MHz logic clock from PSRAM
    wire init_done_0, init_done_1;
    wire cmd_0, cmd_1, cmd_en_0, cmd_en_1;
    wire [20:0] addr0, addr1;
    wire [31:0] wr_data0, wr_data1, rd_data0, rd_data1;
    wire rd_data_valid_0, rd_data_valid_1;
    wire [3:0] data_mask_0, data_mask_1;

    assign addr1 = 0; assign wr_data1 = 0; assign cmd_1 = 0;
    assign cmd_en_1 = 0; assign data_mask_1 = 0;

    Video_frame_buffer frame_buffer (
        .clk(sys_clk), 
        .rst_n(sys_rst_n),
        .memory_clk(clk_135M), 
        .pll_lock(psram_lock),
        .O_psram_ck(O_psram_ck), .O_psram_ck_n(O_psram_ck_n),
        .IO_psram_rwds(IO_psram_rwds), .O_psram_reset_n(O_psram_reset_n),
        .IO_psram_dq(IO_psram_dq), .O_psram_cs_n(O_psram_cs_n),
        .init_calib0(init_done_0), .init_calib1(init_done_1),
        .clk_out(clk_67_5M), 
        .cmd0(cmd_0), .cmd1(cmd_1), .cmd_en0(cmd_en_0), .cmd_en1(cmd_en_1),
        .addr0(addr0), .addr1(addr1),
        .wr_data0(wr_data0), .wr_data1(wr_data1),
        .rd_data0(rd_data0), .rd_data1(rd_data1),
        .rd_data_valid0(rd_data_valid_0), .rd_data_valid1(rd_data_valid_1),
        .data_mask0(data_mask_0), .data_mask1(data_mask_1)
    );

    // C. Frame Buffer Arbiter
    wire queue_store_clk, queue_store_wr_en, queue_store_full;
    wire [16:0] video_data_queue_in;

    VideoController #(
        .MEMORY_BURST(32), .ENABLE_OUTPUT_RESIZE(0)
    ) video_controller (
        .clk(clk_67_5M), .rst_n(sys_rst_n), .init_done(init_done_0),
        .cmd(cmd_0), .cmd_en(cmd_en_0), .addr(addr0),
        .wr_data(wr_data0), .rd_data(rd_data0), .rd_data_valid(rd_data_valid_0),
        .error(), .data_mask(data_mask_0),
        .load_clk_o(mem_load_clk), .load_read_rdy(load_read_rdy),
        .load_command_valid(load_command_valid), .load_pixel_data(load_pixel_data),
        .load_mem_addr(load_mem_addr), .load_command_data(load_command_data),
        .store_clk_o(queue_store_clk), .store_wr_en(queue_store_wr_en),
        .store_queue_full(queue_store_full), .store_queue_data(video_data_queue_in)
    );

    // =========================================================
    // 4. HDMI OUTPUT PIPELINE
    // =========================================================

    // A. Cross the clock domain (PSRAM 67.5MHz -> HDMI 25MHz)
    wire [16:0] lcd_queue_data_out;
    wire lcd_queue_empty;
    wire in_axis_tready;
    wire fifo_rd_en = in_axis_tready & !lcd_queue_empty;

    FIFO_cam bridge_fifo (
        .Data(video_data_queue_in),
        .WrReset(~sys_rst_n), .RdReset(~sys_rst_n),
        .WrClk(queue_store_clk), .RdClk(clk_25M),
        .WrEn(queue_store_wr_en), .RdEn(fifo_rd_en),
        .Q(lcd_queue_data_out), .Empty(lcd_queue_empty),
        .Full(queue_store_full)
    );

    // B. Format the data into an AXI Stream for the HDMI Encoder
    wire [4:0] r5 = lcd_queue_data_out[15:11];
    wire [5:0] g6 = lcd_queue_data_out[10:5];
    wire [4:0] b5 = lcd_queue_data_out[4:0];

    reg tvalid_q;
    reg [23:0] rgb888_q; 
    reg start_of_frame_q; // New register for the frame sync flag

    always @(posedge clk_25M) begin
        if (!sys_rst_n) begin
            tvalid_q <= 0;
            rgb888_q <= 24'd0;
            start_of_frame_q <= 0;
        end else begin
            tvalid_q <= fifo_rd_en;
            rgb888_q <= {r5, r5[4:2], g6, g6[5:4], b5, b5[4:2]}; 
            start_of_frame_q <= lcd_queue_data_out[16]; // Grab the secret bit 16 flag!
        end
    end

    // C. Send to HDMI Module
    svo_hdmi hdmi_out (
        .clk(clk_25M), 
        .resetn(sys_rst_n),
        .clk_pixel(clk_25M),
        .clk_5x_pixel(clk_125M),
        .locked(hdmi_lock),

        // AXI Interface
        .in_axis_tvalid(tvalid_q),
        .in_axis_tready(in_axis_tready),
        .in_axis_tdata(rgb888_q),
        .in_axis_tuser(start_of_frame_q), // <-- Safely using the bit-16 flag

        // Physical HDMI Pins
        .tmds_clk_n(tmds_clk_n),
        .tmds_clk_p(tmds_clk_p),
        .tmds_d_n(tmds_d_n),
        .tmds_d_p(tmds_d_p)
    );

    // =========================================================
    // 6-LED DIAGNOSTIC DASHBOARD (Active-Low: 0 = ON)
    // =========================================================
    
    // Pin 10: HDMI PLL Clock is LOCKED and running at 125MHz
    assign led_out = ~hdmi_lock; 
    
    // Pin 11: PSRAM PLL Clock is LOCKED and running at 135MHz
    assign led_out1 = ~psram_lock; 
    
    // Pin 13: Camera I2C Configuration is COMPLETE
    assign debug_led = ~config_done; 
    
    // Pin 14: PSRAM Hardware Initialization is COMPLETE
    assign status_leds[0] = ~init_done_0; 
    
    // Pin 15: Camera is actively sending frames (This will FLICKER rapidly)
    assign status_leds[1] = ~v_sync_i; 

    // Pin 16: Currently Unused. Keep it OFF (1)
    assign status_leds[2] = 1'b1;

endmodule
