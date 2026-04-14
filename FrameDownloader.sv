`timescale 1ns / 10ps
`include "camera_control_defs.vh"
`include "psram_utils.vh"

package FrameDownloaderTypes;
    typedef enum bit[7:0] {
        FRAME_PROCESSING_START_WAIT      = 8'd01,
        FRAME_PROCESSING_READ_CYC        = 8'd02,
        FRAME_PROCESSING_DONE            = 8'd03,
        CHECK_QUEUE                      = 8'd04,
        START_READ_CYC                   = 8'd05,
        START_READ_ROW                   = 8'd07,
        READ_ROW_CYC                     = 8'd08,
        START_READ_FROM_MEMORY           = 8'd09,
        READ_FROM_MEMORY_CYC             = 8'd10,
        READ_MEMORY_WAIT                 = 8'd11,
        QUEUE_UPLOAD_CYC                 = 8'd12,
        QUEUE_UPLOAD_DONE                = 8'd13,
        ADJUST_ROW_ADDRESS               = 8'd14,
        CACHE_COUNTER_INCREMENT          = 8'd15
    } t_state;
endpackage

module FrameDownloader
    #(
        parameter MEMORY_BURST = 32,
        parameter FRAME_WIDTH = 640,
        parameter FRAME_HEIGHT = 480
    )
    (
        input clk,
        input reset_n,
        input start,
        input queue_full,
        input read_ack,
        input [20:0] base_addr,
        input [31:0] read_data,
        input rd_data_valid,
        
        output reg [16:0] queue_data_o,
        output reg wr_en,
        output reg read_rq,
        output [20:0] read_addr,
        output reg mem_rd_en,
        output reg download_done
    );

    import FrameDownloaderTypes::*;
    import PSRAM_Utilities::*;

    localparam CACHE_SIZE = MEMORY_BURST / 2;
    localparam BURST_CYCLES = burst_cycles(MEMORY_BURST);

    t_state state;
    reg [20:0] frame_addr_counter;
    reg [4:0] cache_addr;
    reg [10:0] frame_addr_inc;
    reg [4:0] read_counter;
    wire [4:0] read_counter_next;
    reg cache_out_en;
    reg frame_download_cycle;

    reg [10:0] col_counter;
    reg [10:0] row_counter;

    wire [31:0] mem_word;
    wire [21:0] adder_out;
    wire [15:0] cache_out;

    assign read_addr = frame_addr_counter;
    assign read_counter_next = read_counter + 1'b1;
    assign mem_word = read_data;

    // REPLACED GOWIN_ALU54: Native Verilog addition saves DSP slices
    assign adder_out = frame_addr_counter + frame_addr_inc;

    Gowin_SDPB_DN download_cache(
        .dout(cache_out), 
        .clka(clk), 
        .cea(rd_data_valid), 
        .reseta(~reset_n), 
        .clkb(clk), 
        .ceb(cache_out_en), 
        .resetb(~reset_n), 
        .oce(1'b1), 
        .ada(read_counter[2:0]), 
        .din(mem_word), 
        .adb(cache_addr[3:0])
    );

    initial begin
        read_rq <= 1'b0;
        mem_rd_en <= 1'b0;
        download_done <= 1'b0;
        wr_en <= 1'b0;
        frame_addr_counter <= 'd0;
        queue_data_o <= 'd0;
        col_counter <= 'd0;
        row_counter <= 'd0;
    end

    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            queue_data_o <= 'd0;
            read_rq <= 1'b0;
            col_counter <= 'd0;
            row_counter <= 'd0;
            state <= FRAME_PROCESSING_START_WAIT;
            read_counter <= 'd0;
            cache_addr <= 'd0;
            frame_addr_inc <= 'd0;
            frame_download_cycle <= 1'b0;
            download_done <= 1'b0;
            cache_out_en <= 1'b0;
        end else begin
            case (state)
                FRAME_PROCESSING_START_WAIT: begin
                    frame_addr_counter <= base_addr;
                    download_done <= 1'b0;
                    if (start == 1'b1) begin
                        state <= FRAME_PROCESSING_READ_CYC;
                        frame_download_cycle <= 1'b0;
                        frame_addr_inc <= 'd0;
                        row_counter <= 'd0;
                    end
                end
                FRAME_PROCESSING_READ_CYC: begin
                    wr_en <= 1'b0;
                    if (row_counter === FRAME_HEIGHT) begin
                        state <= FRAME_PROCESSING_DONE;
                    end else begin
                        cache_addr <= 'd0;
                        state <= CHECK_QUEUE;
                        if (frame_download_cycle)
                            frame_addr_counter <= adder_out[20:0];
                    end
                end
                CHECK_QUEUE: 
                    if (!queue_full) begin
                        if (!frame_download_cycle) begin
                            state <= START_READ_CYC;
                            queue_data_o <= 17'h10000;
                            wr_en <= 1'b1;
                            frame_download_cycle <= 1'b1;
                            col_counter <= 'd0;
                        end
                    end
                START_READ_CYC: begin
                    wr_en <= 1'b0;
                    frame_addr_inc <= 'd0;
                    state <= START_READ_ROW;
                end
                START_READ_ROW: begin
                    if (row_counter === FRAME_HEIGHT) begin
                        if (!queue_full) begin
                            queue_data_o <= 17'h1FFFF;
                            wr_en <= 1'b1;
                            state <= FRAME_PROCESSING_READ_CYC;
                        end
                    end else if (!queue_full) begin
                        queue_data_o <= 17'h10001;
                        wr_en <= 1'b1;
                        col_counter <= 'd0;
                        state <= READ_ROW_CYC;
                    end
                end
                READ_ROW_CYC: begin
                    wr_en <= 1'b0;
                    if (col_counter !== FRAME_WIDTH) begin
                        state <= START_READ_FROM_MEMORY;
                    end else begin
                        row_counter <= row_counter + 1'b1;
                        frame_addr_inc <= 'd0;
                        state <= ADJUST_ROW_ADDRESS;
                    end
                end
                START_READ_FROM_MEMORY: begin
                    read_rq <= 1'b1;
                    state <= READ_MEMORY_WAIT;
                end
                READ_MEMORY_WAIT: begin
                    if (read_ack) begin
                        state <= READ_FROM_MEMORY_CYC;
                        mem_rd_en <= 1'b1;
                        read_counter <= 'd0;
                        frame_addr_counter <= adder_out[20:0];
                    end
                end
                READ_FROM_MEMORY_CYC: begin
                    mem_rd_en <= 1'b0;
                    if (rd_data_valid && read_counter !== BURST_CYCLES)
                        read_counter <= read_counter_next;
                    else if (read_counter === BURST_CYCLES) begin
                        cache_addr <= 'd0;
                        cache_out_en <= 1'b1;
                        read_rq <= 1'b0;
                        state <= QUEUE_UPLOAD_CYC;
                    end
                end
                QUEUE_UPLOAD_CYC: begin
                    if (queue_full) begin
                        // Wait for queue
                    end else if (col_counter !== FRAME_WIDTH && cache_addr !== CACHE_SIZE) begin
                        wr_en <= 1'b0;
                        state <= CACHE_COUNTER_INCREMENT;
                    end else begin
                        wr_en <= 1'b0;
                        state <= QUEUE_UPLOAD_DONE;
                    end
                end
                CACHE_COUNTER_INCREMENT: begin
                    cache_addr <= cache_addr + 1'b1;
                    wr_en <= 1'b1;
                    queue_data_o <= { 1'b0, cache_out };
                    col_counter <= col_counter + 1'b1;
                    state <= QUEUE_UPLOAD_CYC;
                end
                QUEUE_UPLOAD_DONE: begin
                    wr_en <= 1'b0;
                    cache_out_en <= 1'b0;
                    frame_addr_inc <= cache_addr;
                    state <= READ_ROW_CYC;
                end
                ADJUST_ROW_ADDRESS: begin
                    frame_addr_counter <= adder_out[20:0];
                    state <= START_READ_CYC;
                end
                FRAME_PROCESSING_DONE: begin
                    download_done <= 1'b1;
                    state <= FRAME_PROCESSING_START_WAIT;
                end
            endcase
        end
    end
endmodule
