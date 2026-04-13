`timescale 1ns / 1ps

// =============================================================================
// camera_interface.v — OV7670 RGB565 VGA @ 27MHz XCLK
// =============================================================================
// Key points:
//   • OV7670 SCCB write address = 0x42
//   • SCCB 9th bit is "don't care" — FSM accepts both ACK and NACK
//   • VSYNC HIGH = vertical blank, VSYNC LOW = active frame (falling edge = new frame)
//   • RGB565: first byte = {R[4:0], G[5:3]}, second byte = {G[2:0], B[4:0]}
// =============================================================================

module camera_interface (
    input  wire        clk,         // 27MHz base input
    input  wire        clk_100,     // 108MHz (Fast FSM & SCCB clock)
    input  wire        rst_n,
    input  wire [3:0]  key,         // Unused, kept for top_module compatibility

    // Output to HDMI Interface
    output wire        rd_en,       // Pulses HIGH for one clk_100 cycle when pixel ready
    output wire [9:0]  data_count_r,
    output wire [15:0] dout,

    // Physical Camera Pins
    input  wire        cmos_pclk,
    input  wire        cmos_href,
    input  wire        cmos_vsync,
    input  wire [7:0]  cmos_db,
    inout  wire        cmos_sda,
    inout  wire        cmos_scl,
    output wire        cmos_xclk,

    // Debug LEDs
    output wire [3:0]  led
);

    // =========================================================================
    // 1. CLOCK & INPUT SYNC
    // =========================================================================
    // Feed 27MHz directly to camera as XCLK
    assign cmos_xclk    = clk;
    assign data_count_r = 10'd0;    // Unused port, stubbed to 0

    // Synchronize async camera signals into 108MHz domain (2-stage for metastability)
    reg vsync_1, vsync_2;
    reg pclk_1,  pclk_2;
    reg href_1,  href_2;

    always @(posedge clk_100) begin
        vsync_1 <= cmos_vsync; vsync_2 <= vsync_1;
        pclk_1  <= cmos_pclk;  pclk_2  <= pclk_1;
        href_1  <= cmos_href;  href_2  <= href_1;
    end

    // =========================================================================
    // 2. SCCB / I2C MASTER
    // =========================================================================
    reg        start, stop;
    reg  [7:0] wr_data;
    wire [1:0] ack;

    i2c_top i2c_inst (
        .clk    (clk_100),
        .rst_n  (rst_n),
        .start  (start),
        .stop   (stop),
        .wr_data(wr_data),
        .ack    (ack),
        .sda    (cmos_sda),
        .scl    (cmos_scl)
        
    );

    // =========================================================================
    // 3. FSM STATE DECLARATIONS
    // =========================================================================
    localparam  idle        = 0,
                start_sccb  = 1,
                write_addr  = 2,
                write_data  = 3,
                digest_loop = 4,
                delay_st    = 5,
                vsync_fedge = 6,
                byte1       = 7,
                byte2       = 8,
                fifo_write  = 9;

    reg [3:0]  state_q = idle,  state_d;
    reg [26:0] delay_q = 0,     delay_d;
    reg        sdelay_q = 0,    sdelay_d;
    reg [8:0]  midx_q  = 0,     midx_d;
    reg        delay_finish;

    reg [3:0]  led_q   = 4'b1111, led_d;
    reg [15:0] pixel_q = 0,        pixel_d;
    reg        wr_en_q = 0,        wr_en_d;

    assign led   = led_q;
    assign dout  = pixel_q;
    assign rd_en = wr_en_q;

    // =========================================================================
    // 4. OV7670 SCCB INITIALIZATION ROM — RGB565 VGA
    // =========================================================================
    // Format: {reg_addr[7:0], reg_data[7:0]}
    // Total 39 entries (index 0..38)
    // Entry 0 is the software reset — FSM gives it a long startup delay (~155ms)
    // before sending, and a medium delay (~0.6ms) between subsequent registers.
    // =========================================================================
    localparam MSG_INDEX = 38;
    reg [15:0] message [0:MSG_INDEX];

    initial begin
        // --- Software reset (MUST be first; FSM gives extra-long delay after) ---
        message[0]  = 16'h12_80; // COM7:  software reset (self-clearing)

        // --- Output format ---
        message[1]  = 16'h12_04; // COM7:  RGB output, VGA (no test pattern)
        message[2]  = 16'h40_D0; // COM15: RGB565 format, output range 00–FF
        message[3]  = 16'h8C_00; // RGB444: disable (use RGB565 only)
        message[4]  = 16'h3A_04; // TSLB:  byte-swap order for RGB565

        // --- Clock ---
        message[5]  = 16'h11_01; // CLKRC: prescale /2  → internal = 13.5 MHz
        message[6]  = 16'h6B_0A; // DBLV:  bypass PLL, keep internal clock

        // --- Window / timing (VGA 640x480) ---
        message[7]  = 16'h17_13; // HSTART: horizontal start
        message[8]  = 16'h18_01; // HSTOP:  horizontal stop
        message[9]  = 16'h32_92; // HREF:   edge offset
        message[10] = 16'h19_02; // VSTART: vertical start
        message[11] = 16'h1A_7A; // VSTOP:  vertical stop
        message[12] = 16'h03_0A; // VREF:   vertical frame control

        // --- Scaling (full VGA, no downscale) ---
        message[13] = 16'h0C_00; // COM3:   no scaling
        message[14] = 16'h3E_00; // COM14:  no PCLK divider, no manual scaling
        message[15] = 16'h72_11; // DCWCTR: no down-sample
        message[16] = 16'h73_F0; // PCLK divider: no divide
        message[17] = 16'hA2_02; // PCLK delay

        // --- AEC / AGC / AWB (initial manual values, then enable auto) ---
        message[18] = 16'h13_E0; // COM8:   disable AEC, AGC, AWB during setup
        message[19] = 16'h00_00; // GAIN:   manual gain = 0
        message[20] = 16'h10_00; // AECH:   manual exposure MSB
        message[21] = 16'h0D_40; // COM4:   PLL bypass
        message[22] = 16'h14_18; // COM9:   4× max AGC gain ceiling
        message[23] = 16'h24_95; // AEW:    AGC/AEC upper limit
        message[24] = 16'h25_33; // AEB:    AGC/AEC lower limit
        message[25] = 16'h26_E3; // VPT:    fast/slow AEC mode threshold
        message[26] = 16'h13_E5; // COM8:   enable AEC, AGC, AWB (auto mode on)

        // --- Colour processing ---
        message[27] = 16'h3D_C0; // COM13:  enable gamma, UV auto-adjust
        message[28] = 16'h41_08; // COM16:  AWB gain on, no colour FX
        message[29] = 16'h1E_00; // MVFP:   no mirror, no vertical flip

        // --- Colour matrix (RGB565 tuned) ---
        message[30] = 16'h4F_80; // MTX1
        message[31] = 16'h50_80; // MTX2
        message[32] = 16'h51_00; // MTX3
        message[33] = 16'h52_22; // MTX4
        message[34] = 16'h53_5E; // MTX5
        message[35] = 16'h54_80; // MTX6
        message[36] = 16'h58_9E; // MTXS:  matrix sign + scale

        // --- Manual white-balance seed (auto-overrides after warmup) ---
        message[37] = 16'h01_40; // BLUE:  blue channel gain
        message[38] = 16'h02_40; // RED:   red channel gain
    end

    // =========================================================================
    // 5. SEQUENTIAL LOGIC
    // =========================================================================
    always @(posedge clk_100 or negedge rst_n) begin
        if (!rst_n) begin
            state_q  <= idle;
            delay_q  <= 0;
            sdelay_q <= 0;
            midx_q   <= 0;
            led_q    <= 4'b1111;
            pixel_q  <= 0;
            wr_en_q  <= 0;
        end else begin
            state_q  <= state_d;
            delay_q  <= delay_d;
            sdelay_q <= sdelay_d;
            midx_q   <= midx_d;
            led_q    <= led_d;
            pixel_q  <= pixel_d;
            wr_en_q  <= wr_en_d;
        end
    end

    // =========================================================================
    // 6. FSM NEXT-STATE / OUTPUT LOGIC
    // =========================================================================
    always @* begin
        // Defaults
        state_d  = state_q;
        led_d    = led_q;
        start    = 0;
        stop     = 0;
        wr_data  = 0;
        sdelay_d = sdelay_q;
        delay_d  = delay_q;
        delay_finish = 0;
        midx_d   = midx_q;
        pixel_d  = pixel_q;
        wr_en_d  = 0;

        // Free-running delay counter
        if (sdelay_q) delay_d = delay_q + 1'b1;

        // Delay timeout logic:
        //   • Index 0 (reset cmd): wait for bit 24 ≈ 155ms @108MHz (camera needs time to reset)
        //   • All other indexes:   wait for bit 16 ≈ 0.6ms  @108MHz (inter-register gap)
        //   • Fallback timeout:    bit 26 ≈ 620ms (covers start_sccb and final wait)
        if ((midx_q == 0 ? delay_q[24] : delay_q[16])
            && midx_q != (MSG_INDEX + 1)
            && (state_q != start_sccb)) begin
            delay_finish = 1;
            sdelay_d     = 0;
            delay_d      = 0;
        end else if (delay_q[26] && (midx_q == (MSG_INDEX + 1) || state_q == start_sccb)) begin
            delay_finish = 1;
            sdelay_d     = 0;
            delay_d      = 0;
        end

        case (state_q)

            // -----------------------------------------------------------------
            idle: begin
                if (delay_finish) begin
                    state_d  = start_sccb;
                    sdelay_d = 0;
                end else
                    sdelay_d = 1;
            end

            // -----------------------------------------------------------------
            start_sccb: begin
                start   = 1;
                wr_data = 8'h42;        // OV7670 SCCB write address
                state_d = write_addr;
            end

            // -----------------------------------------------------------------
            // SCCB FIX: OV7670 9th bit is "don't care" — slave never pulls SDA
            // low, so ack[0] is always 0 (NACK in I2C terms).
            // We only check ack[1] (= 1 when the 9th bit clock has occurred),
            // and ignore ack[0] (ACK/NACK value).
            // -----------------------------------------------------------------
            write_addr: begin
                if (ack[1] == 1'b1) begin          // 9th bit received — proceed
                    wr_data = message[midx_q][15:8];
                    state_d = write_data;
                end
                // No retry on NACK — SCCB don't-care; just wait for ack[1]
            end

            // -----------------------------------------------------------------
            write_data: begin
                if (ack[1] == 1'b1) begin          // 9th bit received — proceed
                    wr_data = message[midx_q][7:0];
                    state_d = digest_loop;
                end
            end

            // -----------------------------------------------------------------
            digest_loop: begin
                if (ack[1] == 1'b1) begin          // 9th bit received — done
                    stop     = 1;
                    sdelay_d = 1;
                    midx_d   = midx_q + 1'b1;
                    state_d  = delay_st;
                end
            end

            // -----------------------------------------------------------------
            delay_st: begin
                if (midx_q == (MSG_INDEX + 1) && delay_finish) begin
                    // All registers written — camera ready to stream
                    state_d = vsync_fedge;
                    led_d   = 4'b0110;  // LED[1:2] ON = SCCB init success
                end else if (delay_finish) begin
                    state_d = start_sccb;
                end
            end

            // -----------------------------------------------------------------
            // Wait for VSYNC falling edge (HIGH→LOW) = start of active frame
            // OV7670: VSYNC HIGH during vertical blank, LOW during active video
            // -----------------------------------------------------------------
            vsync_fedge: begin
                if (vsync_1 == 0 && vsync_2 == 1)  // falling edge detected
                    state_d = byte1;
            end

            // -----------------------------------------------------------------
            // Capture first byte of RGB565 pixel on PCLK rising edge (with HREF)
            // pclk_2==0 && pclk_1==1 detects the rising edge of PCLK
            // (delayed 2 clk_100 cycles due to synchronizer — still correct)
            // -----------------------------------------------------------------
            byte1: begin
                if (pclk_1 == 1 && pclk_2 == 0 && href_1 == 1) begin
                    pixel_d[15:8] = cmos_db;    // {R[4:0], G[5:3]}
                    state_d       = byte2;
                end else if (vsync_1 == 1 && vsync_2 == 0) begin
                    // VSYNC rose back HIGH = unexpected end of frame, re-sync
                    state_d = vsync_fedge;
                end
            end

            // -----------------------------------------------------------------
            // Capture second byte of RGB565 pixel
            // -----------------------------------------------------------------
            byte2: begin
                if (pclk_1 == 1 && pclk_2 == 0 && href_1 == 1) begin
                    pixel_d[7:0] = cmos_db;     // {G[2:0], B[4:0]}
                    state_d      = fifo_write;
                end else if (vsync_1 == 1 && vsync_2 == 0) begin
                    state_d = vsync_fedge;
                end
            end

            // -----------------------------------------------------------------
            // Assert rd_en (pixel ready) for exactly one clk_100 cycle,
            // then go back to capture next pixel
            // -----------------------------------------------------------------
            fifo_write: begin
                wr_en_d = 1;
                state_d = byte1;
            end

            default: state_d = idle;

        endcase
    end

endmodule
