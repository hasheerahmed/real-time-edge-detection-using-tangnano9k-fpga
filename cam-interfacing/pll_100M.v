module pll_100M (
    input  wire clk,
    output wire clk_sdram,
    input  wire RESET,
    output wire LOCKED
);
    rPLL #(
        .FCLKIN(50),
        .IDIV_SEL(1),
        .ODIV_SEL(2),
        .CLKOUT_DIV(25),
        .FBDIV_SEL(2)
    ) pll_inst (
        .CLKIN(clk),
        .CLKFB(clk_sdram),
        .CLKOUT0(clk_sdram),
        .LOCK(LOCKED)
    );
endmodule
