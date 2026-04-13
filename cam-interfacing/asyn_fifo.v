`timescale 1ns / 1ps

module asyn_fifo
    #(
        parameter DATA_WIDTH       = 8,
                  FIFO_DEPTH_WIDTH = 11   // total depth = 2**FIFO_DEPTH_WIDTH
    )
    (
        input  wire                        rst_n,
        input  wire                        clk_write, clk_read,
        input  wire                        write, read,
        input  wire [DATA_WIDTH-1:0]       data_write,
        output wire [DATA_WIDTH-1:0]       data_read,
        output reg                         full, empty,
        output reg  [FIFO_DEPTH_WIDTH-1:0] data_count_w, data_count_r
    );

    localparam FIFO_DEPTH = 2**FIFO_DEPTH_WIDTH;

    initial begin
        full  = 0;
        empty = 1;
    end


    // =========================================================
    // WRITE CLOCK DOMAIN
    // =========================================================
    reg  [FIFO_DEPTH_WIDTH:0] w_ptr_q  = 0;
    reg  [FIFO_DEPTH_WIDTH:0] r_ptr_sync;          // read ptr converted to binary, synced to write clk
    wire [FIFO_DEPTH_WIDTH:0] w_grey, w_grey_nxt;
    reg  [FIFO_DEPTH_WIDTH:0] r_grey_sync;         // read grey ptr synchronized into write domain

    wire we;
    integer i_w;                                    // FIX: separate loop var for write domain

    assign w_grey     = w_ptr_q ^ (w_ptr_q >> 1);
    assign w_grey_nxt = (w_ptr_q + 1'b1) ^ ((w_ptr_q + 1'b1) >> 1);
    assign we         = write && !full;

    always @(posedge clk_write or negedge rst_n) begin
        if (!rst_n) begin
            w_ptr_q <= 0;
            full    <= 0;
        end else begin
            if (write && !full) begin
                w_ptr_q <= w_ptr_q + 1'b1;
                full    <= w_grey_nxt == {~r_grey_sync[FIFO_DEPTH_WIDTH:FIFO_DEPTH_WIDTH-1],
                                           r_grey_sync[FIFO_DEPTH_WIDTH-2:0]};
            end else begin
                full    <= w_grey     == {~r_grey_sync[FIFO_DEPTH_WIDTH:FIFO_DEPTH_WIDTH-1],
                                           r_grey_sync[FIFO_DEPTH_WIDTH-2:0]};
            end

            // Grey-to-binary for read pointer (synced into write domain)
            for (i_w = 0; i_w <= FIFO_DEPTH_WIDTH; i_w = i_w + 1)
                r_ptr_sync[i_w] = ^(r_grey_sync >> i_w);

            data_count_w <= (w_ptr_q >= r_ptr_sync) ?
                            (w_ptr_q - r_ptr_sync) :
                            (FIFO_DEPTH - r_ptr_sync + w_ptr_q);
        end
    end


    // =========================================================
    // READ CLOCK DOMAIN
    // =========================================================
    reg  [FIFO_DEPTH_WIDTH:0] r_ptr_q = 0;
    wire [FIFO_DEPTH_WIDTH:0] r_ptr_d;
    reg  [FIFO_DEPTH_WIDTH:0] w_ptr_sync;           // write ptr converted to binary, synced to read clk
    reg  [FIFO_DEPTH_WIDTH:0] w_grey_sync;          // write grey ptr synchronized into read domain
    wire [FIFO_DEPTH_WIDTH:0] r_grey, r_grey_nxt;

    integer i_r;                                    // FIX: separate loop var for read domain

    assign r_grey     = r_ptr_q ^ (r_ptr_q >> 1);
    assign r_grey_nxt = (r_ptr_q + 1'b1) ^ ((r_ptr_q + 1'b1) >> 1);
    assign r_ptr_d    = (read && !empty) ? r_ptr_q + 1'b1 : r_ptr_q;

    always @(posedge clk_read or negedge rst_n) begin
        if (!rst_n) begin
            r_ptr_q <= 0;
            empty   <= 1;
        end else begin
            r_ptr_q <= r_ptr_d;

            if (read && !empty) empty <= r_grey_nxt == w_grey_sync;
            else                empty <= r_grey     == w_grey_sync;

            // Grey-to-binary for write pointer (synced into read domain)
            // FIX: was incorrectly using w_ptr_q/r_ptr_sync (write-domain vars)
            for (i_r = 0; i_r <= FIFO_DEPTH_WIDTH; i_r = i_r + 1)
                w_ptr_sync[i_r] = ^(w_grey_sync >> i_r);

            // FIX: use read-domain vars (w_ptr_sync, r_ptr_q), not write-domain vars
            data_count_r <= (w_ptr_sync >= r_ptr_q) ?
                            (w_ptr_sync - r_ptr_q) :
                            (FIFO_DEPTH - r_ptr_q + w_ptr_sync);
        end
    end


    // =========================================================
    // CLOCK DOMAIN CROSSING — 3-Stage Grey Code Synchronizers
    // FIX: was 2 stages; 3 stages needed for 4:1 clock ratio (108MHz:27MHz)
    // =========================================================

    // Read → Write domain (r_grey into write clock)
    reg [FIFO_DEPTH_WIDTH:0] r_grey_sync_t1, r_grey_sync_t2;

    always @(posedge clk_write or negedge rst_n) begin
        if (!rst_n) begin
            r_grey_sync_t1 <= 0;
            r_grey_sync_t2 <= 0;
            r_grey_sync    <= 0;
        end else begin
            r_grey_sync_t1 <= r_grey;
            r_grey_sync_t2 <= r_grey_sync_t1;
            r_grey_sync    <= r_grey_sync_t2;   // 3rd stage
        end
    end

    // Write → Read domain (w_grey into read clock)
    reg [FIFO_DEPTH_WIDTH:0] w_grey_sync_t1, w_grey_sync_t2;

    always @(posedge clk_read or negedge rst_n) begin
        if (!rst_n) begin
            w_grey_sync_t1 <= 0;
            w_grey_sync_t2 <= 0;
            w_grey_sync    <= 0;
        end else begin
            w_grey_sync_t1 <= w_grey;
            w_grey_sync_t2 <= w_grey_sync_t1;
            w_grey_sync    <= w_grey_sync_t2;   // 3rd stage
        end
    end


    // =========================================================
    // DUAL-PORT BLOCK RAM
    // =========================================================
    dual_port_sync #(
        .ADDR_WIDTH(FIFO_DEPTH_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) m0 (
        .clk_r  (clk_read),
        .clk_w  (clk_write),
        .we     (we),
        .din    (data_write),
        .addr_a (w_ptr_q[FIFO_DEPTH_WIDTH-1:0]),
        .addr_b (r_ptr_d[FIFO_DEPTH_WIDTH-1:0]),
        .dout   (data_read)
    );

endmodule


// =========================================================
// Dual-Port Synchronous Block RAM
// =========================================================
module dual_port_sync
    #(
        parameter ADDR_WIDTH = 11,
                  DATA_WIDTH = 8
    )
    (
        input                    clk_r, clk_w,
        input                    we,
        input  [DATA_WIDTH-1:0]  din,
        input  [ADDR_WIDTH-1:0]  addr_a, addr_b,
        output [DATA_WIDTH-1:0]  dout
    );

    reg [DATA_WIDTH-1:0] ram [2**ADDR_WIDTH-1:0];
    reg [ADDR_WIDTH-1:0] addr_b_q;

    always @(posedge clk_w) begin
        if (we) ram[addr_a] <= din;
    end

    always @(posedge clk_r) begin
        addr_b_q <= addr_b;
    end

    assign dout = ram[addr_b_q];

endmodule
