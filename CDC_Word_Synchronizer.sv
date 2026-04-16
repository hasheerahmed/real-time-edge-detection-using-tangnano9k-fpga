`timescale 1ns / 1ps
`default_nettype wire

module CDC_Word_Synchronizer #(
    parameter int WORD_WIDTH = 2,
    parameter int EXTRA_CDC_DEPTH = 1,
    parameter string OUTPUT_BUFFER_TYPE = "SKID" // Kept for module port compatibility
)(
    // Producer Domain (Camera)
    input  wire                   sending_clock,
    input  wire                   sending_clear,
    input  wire [WORD_WIDTH-1:0]  sending_data,
    input  wire                   sending_valid,
    output reg                    sending_ready,

    // Consumer Domain (PSRAM)
    input  wire                   receiving_clock,
    input  wire                   receiving_clear,
    output reg  [WORD_WIDTH-1:0]  receiving_data,
    output reg                    receiving_valid,
    input  wire                   receiving_ready
);

    // --- 4-Phase Handshake Wires ---
    reg tx_req;
    reg [WORD_WIDTH-1:0] tx_data;
    reg rx_ack;

    // --- 1-Bit Synchronizers (Safe for clock crossings) ---
    reg [1:0] sync_req;
    always @(posedge receiving_clock or posedge receiving_clear) begin
        if (receiving_clear) sync_req <= 2'b00;
        else sync_req <= {sync_req[0], tx_req};
    end

    reg [1:0] sync_ack;
    always @(posedge sending_clock or posedge sending_clear) begin
        if (sending_clear) sync_ack <= 2'b00;
        else sync_ack <= {sync_ack[0], rx_ack};
    end

    // --- TX Domain State Machine (Producer) ---
    typedef enum logic [1:0] {TX_IDLE, TX_WAIT_ACK_HIGH, TX_WAIT_ACK_LOW} tx_state_t;
    tx_state_t tx_state;

    always @(posedge sending_clock or posedge sending_clear) begin
        if (sending_clear) begin
            tx_state <= TX_IDLE;
            tx_req <= 1'b0;
            tx_data <= '0;
            sending_ready <= 1'b1;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    if (sending_valid && sending_ready) begin
                        tx_data <= sending_data;      // 1. Lock the data
                        tx_req <= 1'b1;               // 2. Raise the flag
                        sending_ready <= 1'b0;        // 3. Block new incoming data
                        tx_state <= TX_WAIT_ACK_HIGH;
                    end
                end
                TX_WAIT_ACK_HIGH: begin
                    if (sync_ack[1] == 1'b1) begin    // Wait for receiver to acknowledge
                        tx_req <= 1'b0;               // Lower the flag
                        tx_state <= TX_WAIT_ACK_LOW;
                    end
                end
                TX_WAIT_ACK_LOW: begin
                    if (sync_ack[1] == 1'b0) begin    // Wait for acknowledgment to drop
                        sending_ready <= 1'b1;        // Ready for the next command!
                        tx_state <= TX_IDLE;
                    end
                end
            endcase
        end
    end

    // --- RX Domain State Machine (Consumer) ---
    typedef enum logic [1:0] {RX_IDLE, RX_WAIT_READY, RX_WAIT_REQ_LOW} rx_state_t;
    rx_state_t rx_state;

    always @(posedge receiving_clock or posedge receiving_clear) begin
        if (receiving_clear) begin
            rx_state <= RX_IDLE;
            rx_ack <= 1'b0;
            receiving_valid <= 1'b0;
            receiving_data <= '0;
        end else begin
            case (rx_state)
                RX_IDLE: begin
                    if (sync_req[1] == 1'b1) begin        // See the TX flag
                        receiving_data <= tx_data;        // Data is 100% stable, safe to read
                        receiving_valid <= 1'b1;          // Tell PSRAM logic we have a command
                        rx_state <= RX_WAIT_READY;
                    end
                end
                RX_WAIT_READY: begin
                    if (receiving_ready) begin            // PSRAM logic accepts the command
                        receiving_valid <= 1'b0;
                        rx_ack <= 1'b1;                   // Send Acknowledge back to TX
                        rx_state <= RX_WAIT_REQ_LOW;
                    end
                end
                RX_WAIT_REQ_LOW: begin
                    if (sync_req[1] == 1'b0) begin        // TX dropped the flag
                        rx_ack <= 1'b0;                   // Drop the Acknowledge
                        rx_state <= RX_IDLE;
                    end
                end
            endcase
        end
    end

endmodule
