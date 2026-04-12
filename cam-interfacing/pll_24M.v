module pll_24M (
    input  wire clkin,
    output wire clkout,
    output wire locked
);
    // Gowin PLL primitive
    rPLL #(
        .FCLKIN(50),
        .IDIV_SEL(2),
        .ODIV_SEL(8),
        .CLKOUT_DIV(50),
        .FBDIV_SEL(1)
    ) pll_inst (
        .CLKIN(clkin),
        .CLKFB(clkout),
        .CLKOUT0(clkout),
        .LOCK(locked)
    );
endmodule

