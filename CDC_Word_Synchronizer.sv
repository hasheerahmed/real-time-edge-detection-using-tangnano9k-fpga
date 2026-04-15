`timescale 1ns / 1ps

module CDC_Word_Synchronizer #(
    parameter WORD_WIDTH = 2,
    parameter EXTRA_CDC_DEPTH = 1,
    parameter OUTPUT_BUFFER_TYPE = "SKID"
) (
    input sending_clock,
    input sending_clear,
    input [WORD_WIDTH-1:0] sending_data,
    input sending_valid,
    output sending_ready,

    input receiving_clock,
    input receiving_clear,
    output [WORD_WIDTH-1:0] receiving_data,
    output receiving_valid,
    input receiving_ready
);

    // Stage 1: Register on the sending clock domain
    reg [WORD_WIDTH-1:0] tx_data;
    reg tx_valid;

    always @(posedge sending_clock) begin
        if (sending_clear) begin
            tx_data <= 0;
            tx_valid <= 0;
        end else begin
            tx_data <= sending_data;
            tx_valid <= sending_valid;
        end
    end

    // Stage 2 & 3: 2-FF Synchronizer on the receiving clock domain
    reg [WORD_WIDTH-1:0] rx_data_meta, rx_data_sync;
    reg rx_valid_meta, rx_valid_sync;

    always @(posedge receiving_clock) begin
        if (receiving_clear) begin
            rx_data_meta <= 0;
            rx_data_sync <= 0;
            rx_valid_meta <= 0;
            rx_valid_sync <= 0;
        end else begin
            // First Flip Flop (catches metastability)
            rx_data_meta <= tx_data;
            rx_valid_meta <= tx_valid;
            
            // Second Flip Flop (stabilized output)
            rx_data_sync <= rx_data_meta;
            rx_valid_sync <= rx_valid_meta;
        end
    end

    assign receiving_data = rx_data_sync;
    assign receiving_valid = rx_valid_sync;
    assign sending_ready = 1'b1;

endmodule
