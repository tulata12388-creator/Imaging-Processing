`timescale 1ns / 1ps
// Module chuy?n d?i mã nh? phân 4-bit sang hi?n th? LED 7 do?n (Common Anode trên kit DE2)

module segment7(
    input  wire [3:0] bin,
    output reg  [6:0] seg
);
    always @(*) begin
        case (bin)
            4'h0: seg = 7'b100_0000; // 0
            4'h1: seg = 7'b111_1001; // 1
            4'h2: seg = 7'b010_0100; // 2
            4'h3: seg = 7'b011_0000; // 3
            4'h4: seg = 7'b001_1001; // 4
            4'h5: seg = 7'b001_0010; // 5
            4'h6: seg = 7'b000_0010; // 6
            4'h7: seg = 7'b111_1000; // 7
            4'h8: seg = 7'b000_0000; // 8
            4'h9: seg = 7'b001_1000; // 9
            4'hA: seg = 7'b000_1000; // A
            4'hB: seg = 7'b000_0011; // b
            4'hC: seg = 7'b100_0110; // C
            4'hD: seg = 7'b010_0001; // d
            4'hE: seg = 7'b000_0110; // E
            4'hF: seg = 7'b000_1110; // F
            default: seg = 7'b111_1111; // Off
        endcase
    end
endmodule