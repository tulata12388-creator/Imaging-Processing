`timescale 1ns / 1ps
// =============================================================
// sdram_pll_50.v
//
// SAFE rollback PLL
//
// Input:
//   inclk0 = 50 MHz
//
// Outputs:
//   c0 = 50 MHz, 0 phase        -> internal SDRAM/controller clock
//   c1 = 50 MHz, -3 ns phase    -> DRAM_CLK output to SDRAM chip
//   c2 = 25 MHz, 0 phase        -> VGA_CLK + OV7670_XCLK
// =============================================================

module sdram_pll_50(
    input  wire inclk0,
    output wire c0,
    output wire c1,
    output wire c2,
    output wire locked
);

    wire [5:0] clk;
    wire       pll_locked_wire;

    assign c0     = clk[0];
    assign c1     = clk[1];
    assign c2     = clk[2];
    assign locked = pll_locked_wire;

    altpll #(
        .operation_mode("NORMAL"),
        .intended_device_family("Cyclone II"),
        .lpm_type("altpll"),

        // 50 MHz input = 20,000 ps
        .inclk0_input_frequency(20000),

        // c0 = 50 MHz internal SDRAM/controller clock
        .clk0_multiply_by(1),
        .clk0_divide_by(1),
        .clk0_phase_shift("0"),
        .clk0_duty_cycle(50),

        // c1 = 50 MHz SDRAM output clock, phase shifted
        .clk1_multiply_by(1),
        .clk1_divide_by(1),
        .clk1_phase_shift("-3000"),
        .clk1_duty_cycle(50),

        // c2 = 25 MHz VGA + OV7670 XCLK
        .clk2_multiply_by(1),
        .clk2_divide_by(2),
        .clk2_phase_shift("0"),
        .clk2_duty_cycle(50),

        .compensate_clock("CLK0"),

        .port_activeclock("PORT_UNUSED"),
        .port_areset("PORT_UNUSED"),
        .port_clkbad0("PORT_UNUSED"),
        .port_clkbad1("PORT_UNUSED"),
        .port_clkloss("PORT_UNUSED"),
        .port_clkswitch("PORT_UNUSED"),
        .port_configupdate("PORT_UNUSED"),
        .port_fbin("PORT_UNUSED"),
        .port_inclk0("PORT_USED"),
        .port_inclk1("PORT_UNUSED"),
        .port_locked("PORT_USED"),
        .port_pfdena("PORT_UNUSED"),
        .port_phasecounterselect("PORT_UNUSED"),
        .port_phasedone("PORT_UNUSED"),
        .port_phasestep("PORT_UNUSED"),
        .port_phaseupdown("PORT_UNUSED"),
        .port_pllena("PORT_UNUSED"),
        .port_scanaclr("PORT_UNUSED"),
        .port_scanclk("PORT_UNUSED"),
        .port_scanclkena("PORT_UNUSED"),
        .port_scandata("PORT_UNUSED"),
        .port_scandataout("PORT_UNUSED"),
        .port_scandone("PORT_UNUSED"),
        .port_scanread("PORT_UNUSED"),
        .port_scanwrite("PORT_UNUSED"),
        .port_clk0("PORT_USED"),
        .port_clk1("PORT_USED"),
        .port_clk2("PORT_USED"),
        .port_clk3("PORT_UNUSED"),
        .port_clk4("PORT_UNUSED"),
        .port_clk5("PORT_UNUSED")
    ) altpll_component (
        .inclk       ({1'b0, inclk0}),
        .clk         (clk),
        .locked      (pll_locked_wire),

        .activeclock (),
        .areset      (1'b0),
        .clkbad      (),
        .clkena      (6'b111111),
        .clkloss     (),
        .clkswitch   (1'b0),
        .configupdate(1'b0),
        .enable0     (),
        .enable1     (),
        .extclk      (),
        .extclkena   (4'b1111),
        .fbin        (1'b1),
        .fbmimicbidir(),
        .fbout       (),
        .fref        (),
        .icdrclk     (),
        .pfdena      (1'b1),

        .phasecounterselect(4'b0000),
        .phasedone   (),
        .phasestep   (1'b0),
        .phaseupdown (1'b0),

        .pllena      (1'b1),
        .scanaclr    (1'b0),
        .scanclk     (1'b0),
        .scanclkena  (1'b1),
        .scandata    (1'b0),
        .scandataout (),
        .scandone    (),
        .scanread    (1'b0),
        .scanwrite   (1'b0),

        .sclkout0    (),
        .sclkout1    (),
        .vcooverrange(),
        .vcounderrange()
    );

endmodule