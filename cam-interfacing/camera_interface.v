`timescale 1ns / 1ps

// =============================================================================
// camera_interface.v  —  OV7670 RGB565 VGA  (27 MHz XCLK)
//
// Key fixes vs previous version:
//  1. cam_data is latched on cam_pclk's OWN rising edge (not in another domain).
//     The latched value is then what gets assembled into pixels.
//  2. pixel_valid pulses for exactly one cam_pclk cycle when a complete 16-bit
//     pixel is ready.  The async FIFO in hdmi_interface uses cam_pclk as its
//     write clock, so everything is properly synchronised.
//  3. i2c_top ports: rd_tick, rd_data, state are properly connected (left open
//     because we only write to the camera, never read back).
//  4. XCLK = 27 MHz directly from the crystal input.
//  5. OV7670 SCCB address is 0x42 (write).  No 9th-bit ACK expected.
// =============================================================================

module camera_interface (
    input  wire        clk_27m,      // 27 MHz — used for XCLK and SCCB timing
    input  wire        rst_n,

    // Physical OV7670 pins
    input  wire        cam_pclk,
    input  wire        cam_href,
    input  wire        cam_vsync,
    input  wire [7:0]  cam_data,
    inout  wire        cam_sda,
    inout  wire        cam_scl,
    output wire        cam_xclk,

    // Output to HDMI (in cam_pclk clock domain)
    output reg  [15:0] pixel_data,
    output reg         pixel_valid,  // 1-cycle pulse when pixel_data is ready

    // Debug
    output reg  [3:0]  led
);

    // =========================================================================
    // XCLK: give the camera its 27 MHz clock directly
    // =========================================================================
    assign cam_xclk = clk_27m;

    // =========================================================================
    // SCCB (I2C) master — running on clk_27m
    // =========================================================================
    reg        sccb_start = 0;
    reg        sccb_stop  = 0;
    reg  [7:0] sccb_wr_data = 0;
    wire [1:0] sccb_ack;
    // Unused outputs from i2c_top — declare wires so the port is connected
    wire        sccb_rd_tick;
    wire [7:0]  sccb_rd_data;
    wire [3:0]  sccb_state;

    i2c_top #(.freq(100_000)) i2c_inst (
        .clk    (clk_27m),
        .rst_n  (rst_n),
        .start  (sccb_start),
        .stop   (sccb_stop),
        .wr_data(sccb_wr_data),
        .rd_tick(sccb_rd_tick),   // FIX: was unconnected → synthesis error
        .ack    (sccb_ack),
        .rd_data(sccb_rd_data),   // FIX: was unconnected → synthesis error
        .scl    (cam_scl),
        .sda    (cam_sda),
        .state  (sccb_state)      // FIX: was unconnected → synthesis error
    );

    // =========================================================================
    // OV7670 SCCB init ROM  (RGB565, VGA 640×480)
    // Format: {reg_addr[7:0], reg_data[7:0]}
    // =========================================================================
    localparam MSG_INDEX = 38;
    reg [15:0] message [0:MSG_INDEX];

    initial begin
        message[0]  = 16'h12_80; // COM7: software reset
        message[1]  = 16'h12_04; // COM7: RGB output, VGA
        message[2]  = 16'h40_D0; // COM15: RGB565, full output range
        message[3]  = 16'h8C_00; // RGB444: disable
        message[4]  = 16'h3A_04; // TSLB: byte order for RGB565
        message[5]  = 16'h11_01; // CLKRC: prescale /2 → ~13.5 MHz internal
        message[6]  = 16'h6B_0A; // DBLV: bypass PLL
        message[7]  = 16'h17_13; // HSTART
        message[8]  = 16'h18_01; // HSTOP
        message[9]  = 16'h32_92; // HREF edge offset
        message[10] = 16'h19_02; // VSTART
        message[11] = 16'h1A_7A; // VSTOP
        message[12] = 16'h03_0A; // VREF
        message[13] = 16'h0C_00; // COM3: no scaling
        message[14] = 16'h3E_00; // COM14: no PCLK divider
        message[15] = 16'h72_11; // DCWCTR: no downsample
        message[16] = 16'h73_F0; // PCLK: no divide
        message[17] = 16'hA2_02; // PCLK delay
        message[18] = 16'h13_E0; // COM8: disable AEC/AGC/AWB during setup
        message[19] = 16'h00_00; // GAIN: 0
        message[20] = 16'h10_00; // AECH: 0
        message[21] = 16'h0D_40; // COM4: PLL bypass
        message[22] = 16'h14_18; // COM9: 4× max AGC
        message[23] = 16'h24_95; // AEW
        message[24] = 16'h25_33; // AEB
        message[25] = 16'h26_E3; // VPT
        message[26] = 16'h13_E5; // COM8: enable AEC/AGC/AWB
        message[27] = 16'h3D_C0; // COM13: gamma + UV auto
        message[28] = 16'h41_08; // COM16: AWB gain on
        message[29] = 16'h1E_00; // MVFP: no mirror/flip
        message[30] = 16'h4F_80; // MTX1
        message[31] = 16'h50_80; // MTX2
        message[32] = 16'h51_00; // MTX3
        message[33] = 16'h52_22; // MTX4
        message[34] = 16'h53_5E; // MTX5
        message[35] = 16'h54_80; // MTX6
        message[36] = 16'h58_9E; // MTXS
        message[37] = 16'h01_40; // BLUE gain
        message[38] = 16'h02_40; // RED gain
    end

    // =========================================================================
    // SCCB FSM  (runs in clk_27m domain — same domain as i2c_top)
    // =========================================================================
    localparam  S_IDLE       = 0,
                S_START      = 1,
                S_WR_ADDR    = 2,
                S_WR_REG     = 3,
                S_WR_DATA    = 4,
                S_STOP       = 5,
                S_DELAY      = 6,
                S_DONE       = 7;

    reg [3:0]  sccb_st   = S_IDLE;
    reg [8:0]  msg_idx   = 0;
    reg [26:0] dly_cnt   = 0;
    reg        dly_run   = 0;
    reg        sccb_done = 0;   // all registers written

    always @(posedge clk_27m or negedge rst_n) begin
        if (!rst_n) begin
            sccb_st   <= S_IDLE;
            msg_idx   <= 0;
            dly_cnt   <= 0;
            dly_run   <= 0;
            sccb_done <= 0;
            sccb_start<= 0;
            sccb_stop <= 0;
            sccb_wr_data <= 0;
            led       <= 4'b1111;
        end else begin
            // free-running delay counter
            if (dly_run) dly_cnt <= dly_cnt + 1'b1;

            sccb_start <= 0;
            sccb_stop  <= 0;

            case (sccb_st)

                S_IDLE: begin
                    // Wait ~155 ms after reset before first SCCB transaction
                    // bit 22 of dly_cnt @ 27 MHz ≈ 155 ms
                    dly_run <= 1;
                    if (dly_cnt[22]) begin
                        dly_cnt <= 0;
                        dly_run <= 0;
                        sccb_st <= S_START;
                    end
                end

                S_START: begin
                    sccb_start   <= 1;
                    sccb_wr_data <= 8'h42;   // OV7670 write address
                    sccb_st      <= S_WR_ADDR;
                end

                S_WR_ADDR: begin
                    if (sccb_ack[1]) begin   // 9th bit clocked — proceed
                        sccb_wr_data <= message[msg_idx][15:8];
                        sccb_st      <= S_WR_REG;
                    end
                end

                S_WR_REG: begin
                    if (sccb_ack[1]) begin
                        sccb_wr_data <= message[msg_idx][7:0];
                        sccb_st      <= S_WR_DATA;
                    end
                end

                S_WR_DATA: begin
                    if (sccb_ack[1]) begin
                        sccb_stop <= 1;
                        sccb_st   <= S_STOP;
                    end
                end

                S_STOP: begin
                    // Inter-register delay:
                    //   msg 0 (reset): wait bit 22 ≈ 155 ms
                    //   others:        wait bit 16 ≈ 2.4 ms
                    dly_run <= 1;
                    sccb_st <= S_DELAY;
                end

                S_DELAY: begin
                 
                    // Inline logic: done when correct bit sets
                    if ((msg_idx == 0 ? dly_cnt[22] : dly_cnt[16])) begin
                        dly_cnt <= 0;
                        dly_run <= 0;
                        msg_idx <= msg_idx + 1'b1;
                        if (msg_idx == MSG_INDEX) begin
                            sccb_done <= 1;
                            led       <= 4'b0110;  // LEDs 1&2 ON = init done
                            sccb_st   <= S_DONE;
                        end else begin
                            sccb_st <= S_START;
                        end
                    end
                end

                S_DONE: begin
                    // Nothing to do — camera is streaming
                end

                default: sccb_st <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Pixel capture  —  entirely in cam_pclk domain
    //
    // OV7670 RGB565: HREF high during active pixels.
    //   Byte 0 (first PCLK): D[7:0] = {R[4:0], G[5:3]}   → pixel[15:8]
    //   Byte 1 (next  PCLK): D[7:0] = {G[2:0], B[4:0]}   → pixel[7:0]
    //
    // We simply latch cam_data on every rising edge of cam_pclk while HREF=1,
    // alternating between high byte and low byte.
    // =========================================================================
    reg        byte_sel = 0;    // 0 = waiting for high byte, 1 = waiting for low
    reg [7:0]  hi_byte  = 0;

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            byte_sel    <= 0;
            hi_byte     <= 0;
            pixel_data  <= 0;
            pixel_valid <= 0;
        end else begin
            pixel_valid <= 0;   // default: no new pixel

            if (cam_vsync) begin
                // VSYNC high = vertical blank; reset byte alignment
                byte_sel <= 0;
            end else if (cam_href) begin
                if (!byte_sel) begin
                    // First byte
                    hi_byte  <= cam_data;
                    byte_sel <= 1;
                end else begin
                    // Second byte — pixel complete
                    pixel_data  <= {hi_byte, cam_data};
                    pixel_valid <= 1;
                    byte_sel    <= 0;
                end
            end else begin
                // HREF low between lines — reset byte alignment
                byte_sel <= 0;
            end
        end
    end

endmodule
