`timescale 1ns / 1ps
// =============================================================
// vga_interface_de2.v  -- v2: them Image Flipping
//
// SW[2] = flip_h: Flip ngang (Horizontal Mirror / Left-Right)
// SW[3] = flip_v: Rotate 180 (Flip ngang + Flip doc)
//
// --- Nguyen tac hoat dong ---
// Camera QVGA 320x240 duoc scale 2x len VGA 640x480.
// Moi pixel camera -> 2x2 pixel VGA.
//
// Luong goc (khong flip):
//   Dong chan VGA (y=0,2,4...): Doc FIFO -> ghi vao linebuf[src_x]
//                               dong thoi ghi vao pix_q de hien thi ngay.
//   Dong le  VGA (y=1,3,5...): Doc lai linebuf[src_x] (replay dong tren).
//
// Flip ngang (flip_h=1 hoac flip_v=1):
//   Dong chan: Doc FIFO -> ghi vao linebuf[src_x] binh thuong (thu tu trai-phai).
//              Hien thi: doc linebuf[319 - src_x] (dao nguoc).
//              LUU Y: phai doi den cuoi dong moi co du lieu day du de doc nguoc.
//              Giai phap: DELAY hien thi 1 dong - dong chan hien thi noi dung
//              linebuf cua DONG TREN (da hoan chinh), dong le replay no.
//              Ket qua: tre 1 dong display nhung anh flip chinh xac.
//
// Flip doc (flip_v=1): xu ly trong sdram_interface_de2:
//   SDRAM doc tu trang cuoi (dong 239) ve dau (dong 0).
//   vga_interface nhan data da theo thu tu nguoc doc -> chi can flip_h them.
//
// =============================================================
module vga_interface_de2(
    input  wire        clk_vga,
    input  wire        rst_n,
    input  wire        empty_fifo,
    input  wire [15:0] din,
    output reg         rd_en,
    output wire        vga_frame_start,

    // Image Flipping mode
    input  wire        flip_h,   // SW[2]: flip ngang (mirror trai-phai)
    input  wire        flip_v,   // SW[3]: flip doc -> rotate 180 khi ket hop flip_h

    // Motion detector bbox
    input  wire [9:0]  box_x_min,
    input  wire [9:0]  box_x_max,
    input  wire [9:0]  box_y_min,
    input  wire [9:0]  box_y_max,
    input  wire        enable_box,

    // Color detector: chi bbox mau do
    input  wire [9:0]  box_r_xmin,
    input  wire [9:0]  box_r_xmax,
    input  wire [9:0]  box_r_ymin,
    input  wire [9:0]  box_r_ymax,

    // color_mode[0] = red detect overlay
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

    // =========================================================================
    // Line buffer: 320 pixels x 16-bit
    // Ghi: moi dong chan, theo thu tu src_x (trai -> phai)
    // Doc: dong le replay, hoac dong chan neu flip (doc nguoc tu dong tren)
    // =========================================================================
    reg [15:0] linebuf [0:319];

    // src_x: index pixel QVGA [0..319] tuong ung pixel_x VGA [0..639]
    wire [8:0] src_x      = pixel_x[9:1];
    wire       even_vga_line = ~pixel_y[0];
    wire       even_vga_x   = ~pixel_x[0];

    // Slot doc FIFO: chi dong chan, pixel chan
    wire fifo_read_slot = video_on && even_vga_line && even_vga_x;

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
                pix_q            <= din;       // giu pixel hien tai de hien thi ngay
                linebuf[src_x_d] <= din;       // luu vao linebuf de replay dong le
            end
            if (!video_on)
                pix_q <= 16'd0;
        end
    end

    // =========================================================================
    // do_flip_h: co hieu luc khi SW[2] HOAC SW[3] duoc bat
    // =========================================================================
    wire do_flip_h = flip_h || flip_v;

    // Dia chi doc nguoc khi flip
    wire [8:0] read_x_flip = 9'd319 - src_x;

    // =========================================================================
    // Chon pixel hien thi:
    //
    // KHONG FLIP (do_flip_h = 0) -- hanh vi goc, giu nguyen:
    //   Dong chan : pix_q  (pixel vua doc tu FIFO, hien thi ngay)
    //   Dong le   : linebuf[src_x]  (replay dong tren)
    //
    // CO FLIP (do_flip_h = 1):
    //   Dong chan : linebuf[319-src_x] -- doc NGUOC linebuf cua DONG TREN.
    //              Dong tren da hoan chinh nen co the doc bat ky vi tri nao.
    //              Noi dung tre 1 dong so voi normal mode nhung anh flip dung.
    //   Dong le   : linebuf[319-src_x] -- replay dong chan vua xong (flip).
    // =========================================================================
    wire [15:0] pixel565_out;

    assign pixel565_out =
        !video_on   ? 16'd0 :
        do_flip_h   ? linebuf[read_x_flip] :   // flip: luon doc nguoc linebuf
        even_vga_line ? pix_q              :   // normal dong chan: tu FIFO
                        linebuf[src_x];        // normal dong le: replay

    // =========================================================================
    // Tach RGB tu RGB565
    // =========================================================================
    wire [4:0] r5 = pixel565_out[15:11];
    wire [5:0] g6 = pixel565_out[10:5];
    wire [4:0] b5 = pixel565_out[4:0];

    wire [9:0] r_raw = {r5, r5};
    wire [9:0] g_raw = {g6, g6[5:2]};
    wire [9:0] b_raw = {b5, b5};

    // Smoothing filter (temporal, 3:1)
    reg [9:0] r_prev, g_prev, b_prev;
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

    wire [11:0] r_smooth_sum = {2'b0,r_raw}+{2'b0,r_raw}+{2'b0,r_raw}+{2'b0,r_prev};
    wire [11:0] g_smooth_sum = {2'b0,g_raw}+{2'b0,g_raw}+{2'b0,g_raw}+{2'b0,g_prev};
    wire [11:0] b_smooth_sum = {2'b0,b_raw}+{2'b0,b_raw}+{2'b0,b_raw}+{2'b0,b_prev};

    wire [9:0] r_filt = (pixel_x == 10'd0) ? r_raw : r_smooth_sum[11:2];
    wire [9:0] g_filt = (pixel_x == 10'd0) ? g_raw : g_smooth_sum[11:2];
    wire [9:0] b_filt = (pixel_x == 10'd0) ? b_raw : b_smooth_sum[11:2];

    // Color correction (giong code goc)
    wire [11:0] r_base = {2'b0, r_filt};
    wire [11:0] g_base = {2'b0, g_filt};
    wire [11:0] b_base = {2'b0, b_filt};

    wire [9:0] r_cam = sat10(r_base + (r_base >> 2) + 12'd35);
    wire [9:0] g_cam = sat10(g_base - (g_base >> 2) - (g_base >> 3) + 12'd20);
    wire [9:0] b_cam = sat10(b_base + (b_base >> 4) + 12'd28);

    wire empty_debug_pixel = DEBUG_EMPTY_COLOR && fifo_read_slot && empty_fifo;

    // =========================================================================
    // Overlay: Motion detector (khung trang)
    // =========================================================================
    wire is_border_x_mot = (pixel_x == box_x_min) || (pixel_x == box_x_min+10'd1) ||
                           (pixel_x == box_x_max) || (pixel_x == box_x_max+10'd1);
    wire is_border_y_mot = (pixel_y == box_y_min) || (pixel_y == box_y_min+10'd1) ||
                           (pixel_y == box_y_max) || (pixel_y == box_y_max+10'd1);
    wire in_range_x_mot  = (pixel_x >= box_x_min) && (pixel_x <= box_x_max+10'd1);
    wire in_range_y_mot  = (pixel_y >= box_y_min) && (pixel_y <= box_y_max+10'd1);
    wire draw_box_mot    = (is_border_x_mot && in_range_y_mot) ||
                           (is_border_y_mot && in_range_x_mot);
    wire box_mot_valid   = (box_x_max > box_x_min) &&
                           (box_y_max > box_y_min) && enable_box;

    // =========================================================================
    // Overlay: Color detector - khung do (Red)
    // =========================================================================
    wire is_bx_r = (pixel_x==box_r_xmin)||(pixel_x==box_r_xmin+10'd1)||
                   (pixel_x==box_r_xmax)||(pixel_x==box_r_xmax+10'd1);
    wire is_by_r = (pixel_y==box_r_ymin)||(pixel_y==box_r_ymin+10'd1)||
                   (pixel_y==box_r_ymax)||(pixel_y==box_r_ymax+10'd1);
    wire in_rx_r = (pixel_x>=box_r_xmin)&&(pixel_x<=box_r_xmax+10'd1);
    wire in_ry_r = (pixel_y>=box_r_ymin)&&(pixel_y<=box_r_ymax+10'd1);
    wire draw_r  = (is_bx_r&&in_ry_r)||(is_by_r&&in_rx_r);
    wire show_r  = draw_r&&(box_r_xmax>box_r_xmin)&&
                   (box_r_ymax>box_r_ymin)&&color_mode[0];

    // =========================================================================
    // MUX pixel cuoi: motion > red > cam
    // =========================================================================
    wire [9:0] out_r =
        (box_mot_valid && draw_box_mot) ? 10'h3FF :
        show_r                          ? 10'h3FF :
        empty_debug_pixel               ? 10'h180 :
                                          r_cam;

    wire [9:0] out_g =
        (box_mot_valid && draw_box_mot) ? 10'h3FF :
        show_r                          ? 10'h000 :
        empty_debug_pixel               ? 10'h000 :
                                          g_cam;

    wire [9:0] out_b =
        (box_mot_valid && draw_box_mot) ? 10'h3FF :
        show_r                          ? 10'h000 :
        empty_debug_pixel               ? 10'h000 :
                                          b_cam;

    assign VGA_R     = video_on ? out_r : 10'd0;
    assign VGA_G     = video_on ? out_g : 10'd0;
    assign VGA_B     = video_on ? out_b : 10'd0;
    assign VGA_BLANK = video_on;
    assign VGA_SYNC  = 1'b0;

endmodule