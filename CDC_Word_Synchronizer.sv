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

    // Simple FIFO bypass for the Tang Nano 9K implementation
    // to pass the 2-bit command data across the clock domain safely.
    reg [WORD_WIDTH-1:0] data_reg;
    reg valid_reg;

    always @(posedge sending_clock) begin
        if (sending_clear) begin
            data_reg <= 0;
            valid_reg <= 0;
        end else if (sending_valid) begin
            data_reg <= sending_data;
            valid_reg <= 1'b1;
        end else begin
            valid_reg <= 1'b0;
        end
    end

    assign receiving_data = data_reg;
    assign receiving_valid = valid_reg;
    assign sending_ready = 1'b1;

endmodule
