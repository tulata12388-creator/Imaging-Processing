`timescale 1ns / 1ps
// =============================================================
// vga_interface_de2.v
// Them overlay 4 khung mau:
//   - Do    (Red)    : vien mau do
//   - Xanh la (Green): vien mau xanh la
//   - Vang (Yellow)  : vien mau vang
//   - Xanh duong (Blue): vien mau xanh duong
//
// color_mode[3:0] den tu DE2_Camera_System_SDRAM:
//   4'b0001 -> chi hien khung do
//   4'b0010 -> chi hien khung xanh la
//   4'b0100 -> chi hien khung vang
//   4'b1000 -> chi hien khung xanh duong
//   4'b0111 -> hien ca 3 mau (do+xanh la+vang)  [SW[4] mode]
//   4'b0000 -> normal, khong hien khung mau nao
// =============================================================
module vga_interface_de2(
    input  wire        clk_vga,
    input  wire        rst_n,
    input  wire        empty_fifo,
    input  wire [15:0] din,
    output reg         rd_en,
    output wire        vga_frame_start,

    // Motion detector bbox (box truoc do)
    input  wire [9:0]  box_x_min,
    input  wire [9:0]  box_x_max,
    input  wire [9:0]  box_y_min,
    input  wire [9:0]  box_y_max,
    input  wire        enable_box,   // SW[17] - motion mode

    // Color detector: 4 bbox
    input  wire [9:0]  box_r_xmin,
    input  wire [9:0]  box_r_xmax,
    input  wire [9:0]  box_r_ymin,
    input  wire [9:0]  box_r_ymax,

    input  wire [9:0]  box_g_xmin,
    input  wire [9:0]  box_g_xmax,
    input  wire [9:0]  box_g_ymin,
    input  wire [9:0]  box_g_ymax,

    input  wire [9:0]  box_y_xmin,
    input  wire [9:0]  box_y_xmax,
    input  wire [9:0]  box_y_ymin,
    input  wire [9:0]  box_y_ymax,

    input  wire [9:0]  box_b_xmin,
    input  wire [9:0]  box_b_xmax,
    input  wire [9:0]  box_b_ymin,
    input  wire [9:0]  box_b_ymax,

    // color_mode[3:0]: bit0=red, bit1=green, bit2=yellow, bit3=blue
    // Cho phep hien thi khung mau nao
    input  wire [3:0]  color_mode,

    output wire [9:0]  VGA_R,
    output wire [9:0]  VGA_G,
    output wire [9:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS,
    output wire        VGA_BLANK,
    output wire        VGA_SYNC
);
    localparam DEBUG_EMPTY_COLOR = 1'b0;

    function [9:0] sat10;
        input [11:0] val;
        begin
            if (val > 12'd1023)
                sat10 = 10'd1023;
            else
                sat10 = val[9:0];
        end
    endfunction

    wire [9:0] pixel_x;
    wire [9:0] pixel_y;
    wire       video_on;

    vga_core_de2 u_vga_core (
        .clk_25M  (clk_vga),
        .rst_n    (rst_n),
        .hsync    (VGA_HS),
        .vsync    (VGA_VS),
        .video_on (video_on),
        .pixel_x  (pixel_x),
        .pixel_y  (pixel_y)
    );

    assign vga_frame_start = (pixel_x == 10'd0) && (pixel_y == 10'd480);

    reg [15:0] linebuf [0:319];
    wire [8:0] src_x;
    assign src_x = pixel_x[9:1];

    wire even_vga_line;
    wire even_vga_x;
    assign even_vga_line = ~pixel_y[0];
    assign even_vga_x    = ~pixel_x[0];

    wire fifo_read_slot;
    assign fifo_read_slot = video_on && even_vga_line && even_vga_x;

    reg        rd_en_d;
    reg [8:0]  src_x_d;
    reg [15:0] pix_q;

    always @(posedge clk_vga or negedge rst_n) begin
        if (!rst_n) begin
            rd_en   <= 1'b0;
            rd_en_d <= 1'b0;
            src_x_d <= 9'd0;
            pix_q   <= 16'd0;
        end
        else begin
            rd_en   <= fifo_read_slot && !empty_fifo;
            rd_en_d <= rd_en;
            src_x_d <= src_x;
            if (rd_en_d) begin
                pix_q <= din;
                linebuf[src_x_d] <= din;
            end
            if (!video_on) begin
                pix_q <= 16'd0;
            end
        end
    end

    wire [15:0] pixel565_live;
    wire [15:0] pixel565_replay;
    wire [15:0] pixel565_out;
    assign pixel565_live   = pix_q;
    assign pixel565_replay = linebuf[src_x];
    assign pixel565_out = !video_on ? 16'd0 : (even_vga_line ? pixel565_live : pixel565_replay);

    wire [4:0] r5;
    wire [5:0] g6;
    wire [4:0] b5;
    assign r5 = pixel565_out[15:11];
    assign g6 = pixel565_out[10:5];
    assign b5 = pixel565_out[4:0];

    wire [9:0] r_raw;
    wire [9:0] g_raw;
    wire [9:0] b_raw;
    assign r_raw = {r5, r5};
    assign g_raw = {g6, g6[5:2]};
    assign b_raw = {b5, b5};

    reg [9:0] r_prev;
    reg [9:0] g_prev;
    reg [9:0] b_prev;
    always @(posedge clk_vga or negedge rst_n) begin
        if (!rst_n) begin
            r_prev <= 10'd0;
            g_prev <= 10'd0;
            b_prev <= 10'd0;
        end
        else begin
            r_prev <= r_raw;
            g_prev <= g_raw;
            b_prev <= b_raw;
        end
    end

    wire [11:0] r_smooth_sum;
    wire [11:0] g_smooth_sum;
    wire [11:0] b_smooth_sum;
    assign r_smooth_sum = {2'b00, r_raw} + {2'b00, r_raw} + {2'b00, r_raw} + {2'b00, r_prev};
    assign g_smooth_sum = {2'b00, g_raw} + {2'b00, g_raw} + {2'b00, g_raw} + {2'b00, g_prev};
    assign b_smooth_sum = {2'b00, b_raw} + {2'b00, b_raw} + {2'b00, b_raw} + {2'b00, b_prev};

    wire [9:0] r_filt;
    wire [9:0] g_filt;
    wire [9:0] b_filt;
    assign r_filt = (pixel_x == 10'd0) ? r_raw : r_smooth_sum[11:2];
    assign g_filt = (pixel_x == 10'd0) ? g_raw : g_smooth_sum[11:2];
    assign b_filt = (pixel_x == 10'd0) ? b_raw : b_smooth_sum[11:2];

    wire [11:0] r_base;
    wire [11:0] g_base;
    wire [11:0] b_base;
    assign r_base = {2'b00, r_filt};
    assign g_base = {2'b00, g_filt};
    assign b_base = {2'b00, b_filt};

    wire [11:0] r_tmp;
    wire [11:0] g_tmp;
    wire [11:0] b_tmp;
    assign r_tmp = r_base + (r_base >> 2) + 12'd35;
    assign g_tmp = g_base - (g_base >> 2) - (g_base >> 3) + 12'd20;
    assign b_tmp = b_base + (b_base >> 4) + 12'd28;

    wire [9:0] r_cam;
    wire [9:0] g_cam;
    wire [9:0] b_cam;
    assign r_cam = sat10(r_tmp);
    assign g_cam = sat10(g_tmp);
    assign b_cam = sat10(b_tmp);

    wire empty_debug_pixel;
    assign empty_debug_pixel = DEBUG_EMPTY_COLOR && fifo_read_slot && empty_fifo;

    // =========================================================================
    // LOGIC VE KHUNG CHU NHAT - MOTION DETECTOR (trang)
    // =========================================================================
    wire is_border_x_mot = (pixel_x == box_x_min) || (pixel_x == box_x_min + 10'd1) ||
                           (pixel_x == box_x_max) || (pixel_x == box_x_max + 10'd1);
    wire is_border_y_mot = (pixel_y == box_y_min) || (pixel_y == box_y_min + 10'd1) ||
                           (pixel_y == box_y_max) || (pixel_y == box_y_max + 10'd1);
    wire in_range_x_mot  = (pixel_x >= box_x_min) && (pixel_x <= box_x_max + 10'd1);
    wire in_range_y_mot  = (pixel_y >= box_y_min) && (pixel_y <= box_y_max + 10'd1);
    wire draw_box_mot    = (is_border_x_mot && in_range_y_mot) || (is_border_y_mot && in_range_x_mot);
    wire box_mot_valid   = (box_x_max > box_x_min) && (box_y_max > box_y_min) && enable_box;

    // =========================================================================
    // LOGIC VE KHUNG MAU DO (Red)
    // =========================================================================
    wire is_bx_r = (pixel_x == box_r_xmin) || (pixel_x == box_r_xmin + 10'd1) ||
                   (pixel_x == box_r_xmax) || (pixel_x == box_r_xmax + 10'd1);
    wire is_by_r = (pixel_y == box_r_ymin) || (pixel_y == box_r_ymin + 10'd1) ||
                   (pixel_y == box_r_ymax) || (pixel_y == box_r_ymax + 10'd1);
    wire in_rx_r = (pixel_x >= box_r_xmin) && (pixel_x <= box_r_xmax + 10'd1);
    wire in_ry_r = (pixel_y >= box_r_ymin) && (pixel_y <= box_r_ymax + 10'd1);
    wire draw_r  = (is_bx_r && in_ry_r) || (is_by_r && in_rx_r);
    wire show_r  = draw_r && (box_r_xmax > box_r_xmin) && (box_r_ymax > box_r_ymin) && color_mode[0];

    // =========================================================================
    // LOGIC VE KHUNG MAU XANH LA (Green)
    // =========================================================================
    wire is_bx_g = (pixel_x == box_g_xmin) || (pixel_x == box_g_xmin + 10'd1) ||
                   (pixel_x == box_g_xmax) || (pixel_x == box_g_xmax + 10'd1);
    wire is_by_g = (pixel_y == box_g_ymin) || (pixel_y == box_g_ymin + 10'd1) ||
                   (pixel_y == box_g_ymax) || (pixel_y == box_g_ymax + 10'd1);
    wire in_rx_g = (pixel_x >= box_g_xmin) && (pixel_x <= box_g_xmax + 10'd1);
    wire in_ry_g = (pixel_y >= box_g_ymin) && (pixel_y <= box_g_ymax + 10'd1);
    wire draw_g  = (is_bx_g && in_ry_g) || (is_by_g && in_rx_g);
    wire show_g  = draw_g && (box_g_xmax > box_g_xmin) && (box_g_ymax > box_g_ymin) && color_mode[1];

    // =========================================================================
    // LOGIC VE KHUNG MAU VANG (Yellow)
    // =========================================================================
    wire is_bx_y = (pixel_x == box_y_xmin) || (pixel_x == box_y_xmin + 10'd1) ||
                   (pixel_x == box_y_xmax) || (pixel_x == box_y_xmax + 10'd1);
    wire is_by_y = (pixel_y == box_y_ymin) || (pixel_y == box_y_ymin + 10'd1) ||
                   (pixel_y == box_y_ymax) || (pixel_y == box_y_ymax + 10'd1);
    wire in_rx_y = (pixel_x >= box_y_xmin) && (pixel_x <= box_y_xmax + 10'd1);
    wire in_ry_y = (pixel_y >= box_y_ymin) && (pixel_y <= box_y_ymax + 10'd1);
    wire draw_y  = (is_bx_y && in_ry_y) || (is_by_y && in_rx_y);
    wire show_y  = draw_y && (box_y_xmax > box_y_xmin) && (box_y_ymax > box_y_ymin) && color_mode[2];

    // =========================================================================
    // LOGIC VE KHUNG MAU XANH DUONG (Blue)
    // =========================================================================
    wire is_bx_b = (pixel_x == box_b_xmin) || (pixel_x == box_b_xmin + 10'd1) ||
                   (pixel_x == box_b_xmax) || (pixel_x == box_b_xmax + 10'd1);
    wire is_by_b = (pixel_y == box_b_ymin) || (pixel_y == box_b_ymin + 10'd1) ||
                   (pixel_y == box_b_ymax) || (pixel_y == box_b_ymax + 10'd1);
    wire in_rx_b = (pixel_x >= box_b_xmin) && (pixel_x <= box_b_xmax + 10'd1);
    wire in_ry_b = (pixel_y >= box_b_ymin) && (pixel_y <= box_b_ymax + 10'd1);
    wire draw_b  = (is_bx_b && in_ry_b) || (is_by_b && in_rx_b);
    wire show_b  = draw_b && (box_b_xmax > box_b_xmin) && (box_b_ymax > box_b_ymin) && color_mode[3];

    // =========================================================================
    // MUX pixel cuoi cung:
    // Thu tu uu tien: motion (trang) > red > green > yellow > blue > cam
    // =========================================================================
    wire [9:0] out_r;
    wire [9:0] out_g;
    wire [9:0] out_b;

    assign out_r = (box_mot_valid && draw_box_mot) ? 10'h3FF :
                   show_r                          ? 10'h3FF :
                   show_g                          ? 10'h000 :
                   show_y                          ? 10'h3FF :
                   show_b                          ? 10'h000 :
                   empty_debug_pixel               ? 10'h180 :
                                                     r_cam;

    assign out_g = (box_mot_valid && draw_box_mot) ? 10'h3FF :
                   show_r                          ? 10'h000 :
                   show_g                          ? 10'h3FF :
                   show_y                          ? 10'h3FF :
                   show_b                          ? 10'h000 :
                   empty_debug_pixel               ? 10'h000 :
                                                     g_cam;

    assign out_b = (box_mot_valid && draw_box_mot) ? 10'h3FF :
                   show_r                          ? 10'h000 :
                   show_g                          ? 10'h000 :
                   show_y                          ? 10'h000 :
                   show_b                          ? 10'h3FF :
                   empty_debug_pixel               ? 10'h000 :
                                                     b_cam;

    assign VGA_R     = video_on ? out_r : 10'd0;
    assign VGA_G     = video_on ? out_g : 10'd0;
    assign VGA_B     = video_on ? out_b : 10'd0;
    assign VGA_BLANK = video_on;
    assign VGA_SYNC  = 1'b0;

endmodule