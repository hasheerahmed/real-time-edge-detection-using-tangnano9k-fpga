`timescale 1ns / 1ps

module Reset_Synchronizer #(
    parameter EXTRA_DEPTH = 1,
    parameter RESET_ACTIVE_STATE = 1
) (
    input receiving_clock,
    input reset_in,
    output reset_out
);
    reg [EXTRA_DEPTH:0] sync_regs;
    
    always @(posedge receiving_clock or posedge reset_in) begin
        if (reset_in)
            sync_regs <= {(EXTRA_DEPTH+1){1'b1}};
        else
            sync_regs <= {sync_regs[EXTRA_DEPTH-1:0], 1'b0};
    end
    
    assign reset_out = sync_regs[EXTRA_DEPTH];
endmodule
