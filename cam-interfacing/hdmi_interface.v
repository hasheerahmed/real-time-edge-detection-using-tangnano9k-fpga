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
    // TMDS encoder + serializer
    // -----------------------------------------------------------------
    reg [9:0] tmds_r, tmds_g, tmds_b;

    // Simple TMDS encoding (8b/10b for HDMI)
    function [9:0] tmds_encode;
        input [7:0] data;
        input       ctrl;
        input [1:0] ctrl_mode;
        begin
            if (ctrl) begin
                case (ctrl_mode)
                    2'b00: tmds_encode = 10'b1101010100; // HSync
                    2'b01: tmds_encode = 10'b0010101011; // VSync
                    2'b10: tmds_encode = 10'b0101010100; // Control 2
                    2'b11: tmds_encode = 10'b1010101011; // Control 3
                endcase
            end else begin
                // Simplified encoding – for real projects use full TMDS algorithm
                tmds_encode = {data[0], data[1], data[2], data[3], data[4],
                               data[5], data[6], data[7], 1'b0, 1'b1};
            end
        end
    endfunction

    always @(posedge clk_pixel) begin
        if (active && !fifo_empty) begin
            // RGB565 to 8-bit per channel (simple expansion)
            tmds_r <= tmds_encode({fifo_dout[15:11], 3'b0}, 1'b0, 2'b00);
            tmds_g <= tmds_encode({fifo_dout[10:5], 2'b0}, 1'b0, 2'b00);
            tmds_b <= tmds_encode({fifo_dout[4:0], 3'b0}, 1'b0, 2'b00);
        end else begin
            tmds_r <= tmds_encode(8'h00, 1'b1, {hs, vs});
            tmds_g <= tmds_encode(8'h00, 1'b1, 2'b00);
            tmds_b <= tmds_encode(8'h00, 1'b1, 2'b00);
        end
    end

    // Serializer: 10 bits @ 125 MHz
    wire [9:0] ser_r, ser_g, ser_b;
    ODDR #(.DDR_TYPE("FULL")) oddr_r (
        .Q(ser_r[0]), .D0(tmds_r[0]), .D1(tmds_r[1]), .CLK(clk_tmds), .RST(!rst_n)
    );
    // Repeat for all 10 bits... (for brevity, only shown for 2 bits)
    // In a real implementation you would generate 10 ODDRs per channel.

    assign hdmi_data_p[0] = ser_r[0]; // etc.
    // Clock differential
    ODDR #(.DDR_TYPE("FULL")) oddr_clk (
        .Q(hdmi_clk_p), .D0(1'b1), .D1(1'b0), .CLK(clk_pixel), .RST(!rst_n)
    );
    assign hdmi_clk_n = ~hdmi_clk_p;

    // For a complete design, you must generate all differential pairs.
    // This is a simplified skeleton – refer to Tang Nano 9K HDMI examples.
endmodule
