module pll_25M (
    input  wire clk,
    output wire clk_out,
    input  wire RESET,
    output wire LOCKED
);
    rPLL #(
        .FCLKIN(50),
        .IDIV_SEL(2),
        .ODIV_SEL(4),
        .CLKOUT_DIV(25),
        .FBDIV_SEL(1)
    ) pll_inst (
        .CLKIN(clk),
        .CLKFB(clk_out),
        .CLKOUT0(clk_out),
        .LOCK(LOCKED)
    );
endmodule
