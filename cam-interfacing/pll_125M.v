module pll_125M (
    input  wire clk,
    output wire clk_sdram,
    input  wire RESET,
    output wire LOCKED
);
    rPLL #(
        .FCLKIN(50),
        .IDIV_SEL(1),
        .ODIV_SEL(1),
        .CLKOUT_DIV(10),
        .FBDIV_SEL(5)
    ) pll_inst (
        .CLKIN(clk),
        .CLKFB(clk_sdram),
        .CLKOUT0(clk_sdram),
        .LOCK(LOCKED)
    );
endmodule
