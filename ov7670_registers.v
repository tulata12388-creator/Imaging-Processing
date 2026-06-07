`timescale 1ns / 1ps
// =============================================================
// ov7670_registers.v
//
// OV7670 RGB565 VGA configuration.
// Color tuned for less green/yellow cast and more natural tone.
//
// Compatible with ov7670_controller REG_LAST = 8'h4A.
//
// Important:
// - CAMERA_BYTE_SWAP stays 1'b0
// - COM15 stays 40_D0
// - TSLB stays 3A_04
// - Color bar OFF stays 42_00
// =============================================================

module ov7670_registers(
    input      [7:0]  advance,
    output reg [15:0] command
);

always @(*) begin
    case (advance)

        // Reset + delay
        8'h00: command = 16'h12_80; // COM7 reset
        8'h01: command = 16'hFF_FF; // delay marker

        // Basic RGB565 VGA output
        8'h02: command = 16'h12_04; // COM7: RGB output, VGA
        8'h03: command = 16'h11_01; // CLKRC: stable camera clock
        8'h04: command = 16'h0C_00; // COM3: default
        8'h05: command = 16'h3E_00; // COM14: no scaling, normal PCLK
        8'h06: command = 16'h04_00; // COM1: disable CCIR656
        8'h07: command = 16'h40_D0; // COM15: RGB565, full output range
        8'h08: command = 16'h3A_04; // TSLB: RGB565 byte sequence
        8'h09: command = 16'h14_18; // COM9: max AGC value x4

        // -----------------------------------------------------
        // Color matrix tuning
        //
        // This matrix is more natural than the old 80/80/00/22/5E/80 set.
        // It helps skin tone and reduces the flat green/yellow look.
        // -----------------------------------------------------
        8'h0A: command = 16'h4F_B3; // MTX1
        8'h0B: command = 16'h50_B3; // MTX2
        8'h0C: command = 16'h51_00; // MTX3
        8'h0D: command = 16'h52_3D; // MTX4
        8'h0E: command = 16'h53_A7; // MTX5
        8'h0F: command = 16'h54_E4; // MTX6
        8'h10: command = 16'h58_9E; // MTXS
        8'h11: command = 16'h3D_C0; // COM13: gamma + UV/color matrix adjust

        // VGA window
        8'h12: command = 16'h17_14; // HSTART
        8'h13: command = 16'h18_02; // HSTOP
        8'h14: command = 16'h32_80; // HREF
        8'h15: command = 16'h19_03; // VSTART
        8'h16: command = 16'h1A_7B; // VSTOP
        8'h17: command = 16'h03_0A; // VREF

        // Timing / magic values
        8'h18: command = 16'h0F_41; // COM6
        8'h19: command = 16'h1E_00; // MVFP: no mirror/flip
        8'h1A: command = 16'h33_0B; // CHLF
        8'h1B: command = 16'h3C_78; // COM12: no HREF when VSYNC low
        8'h1C: command = 16'h69_00; // GFIX
        8'h1D: command = 16'h74_00; // REG74
        8'h1E: command = 16'hB0_84; // reserved, important for color
        8'h1F: command = 16'hB1_0C; // ABLC1
        8'h20: command = 16'hB2_0E; // reserved
        8'h21: command = 16'hB3_80; // THL_ST

        // Scaling / reference magic values
        8'h22: command = 16'h70_3A;
        8'h23: command = 16'h71_35;
        8'h24: command = 16'h72_11;
        8'h25: command = 16'h73_F0;
        8'h26: command = 16'hA2_02;

        // Gamma curve
        8'h27: command = 16'h7A_20;
        8'h28: command = 16'h7B_10;
        8'h29: command = 16'h7C_1E;
        8'h2A: command = 16'h7D_35;
        8'h2B: command = 16'h7E_5A;
        8'h2C: command = 16'h7F_69;
        8'h2D: command = 16'h80_76;
        8'h2E: command = 16'h81_80;
        8'h2F: command = 16'h82_88;
        8'h30: command = 16'h83_8F;
        8'h31: command = 16'h84_96;
        8'h32: command = 16'h85_A3;
        8'h33: command = 16'h86_AF;
        8'h34: command = 16'h87_C4;
        8'h35: command = 16'h88_D7;
        8'h36: command = 16'h89_E8;

        // AGC / AEC setup
        8'h37: command = 16'h13_E0; // COM8: disable AGC/AEC/AWB first
        8'h38: command = 16'h00_00; // GAIN
        8'h39: command = 16'h10_00; // AECH
        8'h3A: command = 16'h0D_40; // COM4
        8'h3B: command = 16'h14_18; // COM9
        8'h3C: command = 16'hA5_05; // BD50MAX
        8'h3D: command = 16'hAB_07; // DB60MAX
        8'h3E: command = 16'h24_95; // AGC upper limit
        8'h3F: command = 16'h25_33; // AGC lower limit
        8'h40: command = 16'h26_E3; // AGC/AEC fast mode region
        8'h41: command = 16'h9F_78; // HAECC1
        8'h42: command = 16'hA0_68; // HAECC2
        8'h43: command = 16'hA1_03; // HAECC3
        8'h44: command = 16'hA6_D8; // HAECC4
        8'h45: command = 16'hA7_D8; // HAECC5
        8'h46: command = 16'hA8_F0; // HAECC6
        8'h47: command = 16'hA9_90; // HAECC7
        8'h48: command = 16'hAA_94; // HAECC

        // Enable AGC + AEC + AWB.
        // E7 usually gives better auto white balance than E5 here.
        8'h49: command = 16'h13_E7; // COM8: enable AGC/AEC/AWB

        // Color bar OFF
        8'h4A: command = 16'h42_00; // COM17: color bar OFF

        default: command = 16'hFF_FF;
    endcase
end

endmodule
