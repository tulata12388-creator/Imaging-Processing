`timescale 1ns / 1ps

module vga_core_de2(
    input  wire       clk_25M,
    input  wire       rst_n,
    output reg        hsync,
    output reg        vsync,
    output wire       video_on,
    output wire [9:0] pixel_x,
    output wire [9:0] pixel_y
);
    localparam [9:0] HD = 10'd640;
    localparam [9:0] HF = 10'd16;
    localparam [9:0] HS = 10'd96;
    localparam [9:0] HB = 10'd48;
    localparam [9:0] HT = 10'd800;

    localparam [9:0] VD = 10'd480;
    localparam [9:0] VF = 10'd10;
    localparam [9:0] VS = 10'd2;
    localparam [9:0] VB = 10'd33;
    localparam [9:0] VT = 10'd525;

    reg [9:0] h_cnt;
    reg [9:0] v_cnt;

    always @(posedge clk_25M or negedge rst_n) begin
        if (!rst_n) begin
            h_cnt <= 10'd0;
            v_cnt <= 10'd0;
        end
        else begin
            if (h_cnt == HT - 1'b1) begin
                h_cnt <= 10'd0;
                if (v_cnt == VT - 1'b1)
                    v_cnt <= 10'd0;
                else
                    v_cnt <= v_cnt + 1'b1;
            end
            else begin
                h_cnt <= h_cnt + 1'b1;
            end
        end
    end

    always @(*) begin
        hsync = ~((h_cnt >= (HD + HF)) && (h_cnt < (HD + HF + HS)));
        vsync = ~((v_cnt >= (VD + VF)) && (v_cnt < (VD + VF + VS)));
    end

    assign video_on = (h_cnt < HD) && (v_cnt < VD);
    assign pixel_x  = h_cnt;
    assign pixel_y  = v_cnt;
endmodule