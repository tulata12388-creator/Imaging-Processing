`timescale 1ns / 1ps
// =============================================================
// vga_interface_de2.v  -- v5: Thermal Camera Mode (SW[10])
//
// SW[2]  = flip_h      : Flip ngang (Horizontal Mirror / Left-Right)
// SW[3]  = flip_v      : Rotate 180 (Flip ngang + Flip doc)
// SW[4]  = brightness  : Tang do sang (+25% moi kenh RGB)
// SW[5]  = grayscale   : Anh xam (BT.601 luma: 0.299R + 0.587G + 0.114B)
// SW[6]  = threshold   : Nhi phan hoa (den/trang theo nguong luma=512)
// SW[7]  = edge_overlay: Lam noi bien (Sobel don gian dung linebuf)
// SW[8]  = grid_overlay: Luoi phan vung (16x16 o tren anh goc)
// SW[10] = thermal_mode: Gia mau nhiet (False Color / Thermal Camera)
//          Chuyen anh xam (luma) -> palette mau nhiet 4 vung:
//          Den(lanh) -> Xanh duong -> Cyan -> Vang -> Do(nong)
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
    input  wire        flip_h,        // SW[2]: flip ngang (mirror trai-phai)
    input  wire        flip_v,        // SW[3]: flip doc -> rotate 180 khi ket hop flip_h

    // Image Processing modes
    input  wire        brightness_mode,  // SW[4]: tang do sang
    input  wire        grayscale_mode,   // SW[5]: anh xam
    input  wire        threshold_mode,   // SW[6]: nhi phan hoa
    input  wire        edge_mode,        // SW[7]: lam noi bien
    input  wire        grid_mode,        // SW[8]: luoi phan vung

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
    // Compare mode: split-screen Inverse vs Natural
    input  wire        compare_mode,  // SW[9]

    // Thermal Camera / False Color mode
    input  wire        thermal_mode,  // SW[10]

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

    assign vga_frame_start = (pixel_x == 10'd0) && (pixel_y == 10'd481);

    // =========================================================================
    // Line buffer: 320 pixels x 16-bit
    // Ghi: moi dong chan, theo thu tu src_x (trai -> phai)
    // Doc: dong le replay, hoac dong chan neu flip (doc nguoc tu dong tren)
    // =========================================================================
    reg [15:0] linebuf      [0:319]; // dong hien tai (ghi khi doc FIFO)
    reg [15:0] linebuf_prev [0:319]; // snapshot dong TREN da hoan chinh (dung cho flip)

    // src_x: index pixel QVGA [0..319] tuong ung pixel_x VGA [0..639]
    wire [8:0] src_x         = pixel_x[9:1];
    wire       even_vga_line = ~pixel_y[0];
    wire       even_vga_x    = ~pixel_x[0];

    // Slot doc FIFO: chi dong chan, pixel chan
    wire fifo_read_slot = video_on && even_vga_line && even_vga_x;

    reg        rd_en_d;
    reg [8:0]  src_x_d;
    reg [15:0] pix_q;

    // =========================================================================
    // linebuf: Luu dong cam dang doc tu FIFO (ghi moi dong chan).
    // linebuf_prev: Snapshot cua dong cam TRUOC DO (da hoan chinh).
    //   - Duoc cap nhat khi dong le dang render: copy linebuf[x] -> linebuf_prev[x]
    //     tai tung vi tri src_x.
    //   - Khi dong chan tiep theo bat dau va flip_h=1, linebuf_prev da day du 320
    //     pixels cua dong truoc -> doc nguoc tu linebuf_prev ma khong conflict.
    // Ket qua: khong co read/write hazard -> anh flip phang, khong bi soc ngang.
    // =========================================================================

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
                pix_q            <= din;
                linebuf[src_x_d] <= din;
            end
            // Khi dong le: copy linebuf -> linebuf_prev tung pixel (1 pixel/clk)
            // Dieu kien: dong le, video_on, pixel chan (moi QVGA pixel ghi 1 lan)
            if (video_on && !even_vga_line && even_vga_x) begin
                linebuf_prev[src_x] <= linebuf[src_x];
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
    // KHONG FLIP (do_flip_h = 0):
    //   Dong chan : pix_q  (pixel vua doc tu FIFO, hien thi ngay)
    //   Dong le   : linebuf[src_x]  (replay dong tren)
    //
    // CO FLIP (do_flip_h = 1):
    //   Dong chan + Dong le: linebuf_prev[319-src_x]
    //     (doc nguoc tu snapshot dong tren, khong conflict voi linebuf dang ghi)
    // =========================================================================
    wire [15:0] pixel565_out;

    assign pixel565_out =
        !video_on     ? 16'd0 :
        do_flip_h     ? linebuf_prev[read_x_flip] : // flip: doc tu snapshot dong tren
        even_vga_line ? pix_q                     : // normal dong chan: tu FIFO
                        linebuf[src_x];              // normal dong le: replay

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

    // =========================================================================
    // Luma (Y) - dung chung cho Grayscale, Threshold, Edge
    // Xap xi BT.601: Y = (77*R + 150*G + 29*B) >> 8
    //   => voi R,G,B la 10-bit (0..1023):
    //      Y = (77*r_cam + 150*g_cam + 29*b_cam) >> 8
    //   Ket qua: 10-bit (0..1023)
    // =========================================================================
    wire [19:0] luma_sum = (20'd77  * {10'b0, r_cam})
                         + (20'd150 * {10'b0, g_cam})
                         + (20'd29  * {10'b0, b_cam});
    wire [9:0] luma = luma_sum[17:8];   // >> 8, lay 10 bit

    // =========================================================================
    // Brightness mode (SW[4])
    // Tang do sang: ket_qua = pixel + pixel/4  (+25% moi kenh)
    // sat10() chong tran so (cap 0..1023)
    // =========================================================================
    wire [11:0] r_bright = {2'b0, r_cam} + {2'b0, r_cam[9:2]};
    wire [11:0] g_bright = {2'b0, g_cam} + {2'b0, g_cam[9:2]};
    wire [11:0] b_bright = {2'b0, b_cam} + {2'b0, b_cam[9:2]};

    wire [9:0] r_after_bright = brightness_mode ? sat10(r_bright) : r_cam;
    wire [9:0] g_after_bright = brightness_mode ? sat10(g_bright) : g_cam;
    wire [9:0] b_after_bright = brightness_mode ? sat10(b_bright) : b_cam;

    // =========================================================================
    // Grayscale mode (SW[5])
    // Thay the R,G,B bang gia tri Luma dong nhat -> anh xam
    // Luma tinh tu r_cam/g_cam/b_cam (truoc brightness de giu chuan mau)
    // =========================================================================
    wire [9:0] r_after_gray = grayscale_mode ? luma : r_after_bright;
    wire [9:0] g_after_gray = grayscale_mode ? luma : g_after_bright;
    wire [9:0] b_after_gray = grayscale_mode ? luma : b_after_bright;

    // =========================================================================
    // Threshold mode (SW[6])
    // Nguong cung dinh = 512 (chinh xac la 50% dai dong)
    // pixel >= 512: trang (1023)  |  pixel < 512: den (0)
    // Ap dung tren luma de dam bao output van la grayscale hop le
    // =========================================================================
    wire       thresh_white  = (luma >= 10'd512);
    wire [9:0] thresh_val    = thresh_white ? 10'd1023 : 10'd0;

    wire [9:0] r_after_thresh = threshold_mode ? thresh_val : r_after_gray;
    wire [9:0] g_after_thresh = threshold_mode ? thresh_val : g_after_gray;
    wire [9:0] b_after_thresh = threshold_mode ? thresh_val : b_after_gray;

    // =========================================================================
    // Edge Overlay mode (SW[7])
    // Phuong phap: Gradient 1D theo chieu ngang (X)
    //
    //   edge_x = |luma(x) - luma(x-1)|
    //
    // luma(x-1) lay tu linebuf_luma[] -- mang rieng luu luma dong hien tai.
    // Neu edge_x > EDGE_THRESH: to mau do len tren anh goc -> overlay.
    // Khong lam bien doc (Y) vi can 2 dong linebuf -> phuc tap hon.
    //
    // Luong:
    //   Dong chan: doc FIFO -> tinh luma -> ghi vao linebuf_luma[src_x_d]
    //   Moi pixel: doc linebuf_luma[src_x_d - 1] lam luma trai
    //   edge_x = |luma_cur - luma_left|
    // =========================================================================
    localparam [9:0] EDGE_THRESH = 10'd80;  // nguong gradient (co the chinh)

    reg [9:0] linebuf_luma [0:319];   // luu luma dong hien tai de tinh gradient

    // Ghi luma vao linebuf_luma cung luc voi linebuf pixel
    // (src_x_d va rd_en_d da duoc tinh o block linebuf phia tren)
    always @(posedge clk_vga) begin
        if (rd_en_d)
            linebuf_luma[src_x_d] <= luma;
    end

    // Doc luma pixel trai: src_x_d - 1 (neu src_x=0 thi lay 0)
    wire [8:0] left_idx       = (src_x_d == 9'd0) ? 9'd0 : (src_x_d - 9'd1);
    wire [9:0] luma_left      = linebuf_luma[left_idx];
    wire [9:0] luma_cur       = luma;

    // Gia tri tuyet doi |luma_cur - luma_left|
    wire [9:0] grad_x = (luma_cur >= luma_left)
                        ? (luma_cur - luma_left)
                        : (luma_left - luma_cur);

    wire is_edge = (grad_x > EDGE_THRESH);

    // Overlay bien mau xanh la cay (Green) len tren anh goc
    wire [9:0] r_after_edge  = (edge_mode && is_edge) ? 10'd0    : r_after_thresh;
    wire [9:0] g_after_edge  = (edge_mode && is_edge) ? 10'd1023 : g_after_thresh;
    wire [9:0] b_after_edge  = (edge_mode && is_edge) ? 10'd0    : b_after_thresh;

    // =========================================================================
    // Grid Overlay mode (SW[8])
    // Ve luoi 16x16 o tren man hinh VGA (640x480).
    // Khoang cach giua cac duong ke = 40 pixel VGA (tuong ung 20 pixel QVGA).
    // Mau luoi: xanh duong (Cyan) de de nhin tren moi nen.
    //
    // Duong doc  : pixel_x % 40 == 0
    // Duong ngang: pixel_y % 40 == 0
    // =========================================================================
    wire is_grid_x = (pixel_x ==10'd0)||(pixel_x ==10'd40)||(pixel_x ==10'd80)||
                     (pixel_x ==10'd120)||(pixel_x==10'd160)||(pixel_x==10'd200)||
                     (pixel_x ==10'd240)||(pixel_x==10'd280)||(pixel_x==10'd320)||
                     (pixel_x ==10'd360)||(pixel_x==10'd400)||(pixel_x==10'd440)||
                     (pixel_x ==10'd480)||(pixel_x==10'd520)||(pixel_x==10'd560)||
                     (pixel_x ==10'd600)||(pixel_x==10'd640);
    wire is_grid_y = (pixel_y ==10'd0)||(pixel_y ==10'd40)||(pixel_y ==10'd80)||
                     (pixel_y ==10'd120)||(pixel_y==10'd160)||(pixel_y==10'd200)||
                     (pixel_y ==10'd240)||(pixel_y==10'd280)||(pixel_y==10'd320)||
                     (pixel_y ==10'd360)||(pixel_y==10'd400)||(pixel_y==10'd440)||
                     (pixel_y ==10'd480);
    wire is_grid    = is_grid_x || is_grid_y;

    // Mau luoi: Cyan (R=0, G=max, B=max)
    wire [9:0] r_after_grid = (grid_mode && is_grid) ? 10'd0    : r_after_edge;
    wire [9:0] g_after_grid = (grid_mode && is_grid) ? 10'd1023 : g_after_edge;
    wire [9:0] b_after_grid = (grid_mode && is_grid) ? 10'd1023 : b_after_edge;

    // Ket noi vao r_proc/g_proc/b_proc de vao MUX cuoi
    wire [9:0] r_proc = r_after_grid;
    wire [9:0] g_proc = g_after_grid;
    wire [9:0] b_proc = b_after_grid;

    // =========================================================================
    // Thermal Camera / False Color mode (SW[10])
    //
    // Palette DA DAO NGUOC: luma thap (toi/nguoi) = NONG, luma cao (sang/nen) = LANH
    // Phu hop voi camera quang hoc: vung toi (da nguoi) cam nhan la "nong" hon nen sang.
    //
    // Buoc 1: Temporal Smoothing (chi dung cho thermal)
    //   - Luu luma cua frame truoc vao linebuf_th_prev[0..319]
    //   - Ghi vao linebuf_th_prev khi doc FIFO (rd_en_d), giong linebuf chinh
    //   - Blend: luma_th = (luma * 3 + luma_prev * 1) >> 2  (75% hien tai + 25% truoc)
    //   - Ket qua: giam flickering/nhieu giua cac frame, anh thermal muot hon
    //   - QUAN TRONG: chi blend khi thermal_mode=1, else dung luma goc (khong lam xao tron)
    //
    // Buoc 2: Dao nguoc luma
    //   luma_inv = 1023 - luma_th
    //
    // Buoc 3: Map luma_inv -> palette 4 vung (moi vung 256 cap):
    //
    //   Vung 1 (luma_inv   0..255):  Den  -> Xanh duong  	 R=0,        G=0,           B=luma_inv*4
    //   Vung 2 (luma_inv 256..511):  Xanh duong -> Cyan  	 R=0,        G=off*4,       B=1023
    //   Vung 3 (luma_inv 512..767):  Cyan -> Vang         	 R=off*4,    G=1023,        B=1023-off*4
    //   Vung 4 (luma_inv 768..1023): Vang -> Do             R=1023,     G=1023-off*4,  B=0
    //
    // Ket qua:
    //   Vung toi  (luma thap)  -> luma_inv cao  -> Do/Vang  (NONG)
    //   Vung sang (luma cao)   -> luma_inv thap -> Xanh/Den (LANH)
    //
    // *4 = dich trai 2 bit (khong can DSP)
    // off = offset trong vung hien tai (0..255), co guard tranh underflow unsigned
    // =========================================================================

    // -------------------------------------------------------------------------
    // Buoc 1: Temporal Smoothing cho Thermal
    //
    // linebuf_th_prev[320]: luu luma cua DONG TRUOC (da duoc blend frame)
    // Ghi cung luc voi linebuf chinh (rd_en_d + src_x_d)
    // -------------------------------------------------------------------------
    reg [9:0] linebuf_th_prev [0:319];  // line buffer luma frame truoc

    always @(posedge clk_vga) begin
        if (rd_en_d)
            linebuf_th_prev[src_x_d] <= luma;  // luu luma dong hien tai
    end

    // Doc luma dong truoc tai cung vi tri src_x_d
    wire [9:0] luma_prev_th = linebuf_th_prev[src_x_d];

    // Blend: 75% luma hien tai + 25% luma dong truoc
    // luma_th = (luma*3 + luma_prev) >> 2
    // Dung 12-bit trung gian tranh tran: max = (1023*3 + 1023) = 4092 < 4096
    wire [11:0] luma_blend_sum = ({2'b0, luma} + {2'b0, luma} + {2'b0, luma}
                                  + {2'b0, luma_prev_th});
    wire [9:0]  luma_th = luma_blend_sum[11:2];  // >> 2

    // Chon: chi dung luma_th khi thermal_mode=1, else dung luma goc de khong anh huong mode khac
    wire [9:0] luma_for_thermal = thermal_mode ? luma_th : luma;

    // Buoc 2: Dao nguoc luma (dung luma_for_thermal)
    wire [9:0] luma_inv = 10'd1023 - luma_for_thermal;

    // Buoc 2: Offset trong moi vung - co guard (tranh underflow khi luma_inv < nguong)
    wire [9:0] th_off2 = (luma_inv >= 10'd256) ? (luma_inv - 10'd256) : 10'd0;
    wire [9:0] th_off3 = (luma_inv >= 10'd512) ? (luma_inv - 10'd512) : 10'd0;
    wire [9:0] th_off4 = (luma_inv >= 10'd768) ? (luma_inv - 10'd768) : 10'd0;

    // *4 = shift trai 2 bit, lay 8 bit thap -> 10 bit (max 255*4=1020 <= 1023, an toan)
    wire [9:0] luma_inv_x4 = {luma_inv[7:0], 2'b00};  // Vung 1
    wire [9:0] off2_x4     = {th_off2[7:0],  2'b00};  // Vung 2
    wire [9:0] off3_x4     = {th_off3[7:0],  2'b00};  // Vung 3
    wire [9:0] off4_x4     = {th_off4[7:0],  2'b00};  // Vung 4

    // Kenh R: 0 -> 0 -> tang dan -> bao hoa 1023
    wire [9:0] th_r =
        (luma_inv < 10'd256) ? 10'd0    :  // Vung 1: R=0 (Den->Xanh duong)
        (luma_inv < 10'd512) ? 10'd0    :  // Vung 2: R=0 (Xanh duong->Cyan)
        (luma_inv < 10'd768) ? off3_x4  :  // Vung 3: R tang dan (Cyan->Vang)
                               10'd1023;   // Vung 4: R bao hoa  (Vang->Do)

    // Kenh G: 0 -> tang dan -> bao hoa -> giam dan
    wire [9:0] th_g =
        (luma_inv < 10'd256) ? 10'd0             :  // Vung 1: G=0
        (luma_inv < 10'd512) ? off2_x4           :  // Vung 2: G tang dan
        (luma_inv < 10'd768) ? 10'd1023          :  // Vung 3: G bao hoa
                               (10'd1023-off4_x4);  // Vung 4: G giam dan

    // Kenh B: tang dan -> bao hoa -> giam dan -> 0
    // Guard them cho phep tru: chi tru khi off3_x4 <= 1023 (luon dung vi max=1020)
    wire [9:0] th_b =
        (luma_inv < 10'd256) ? luma_inv_x4          :  // Vung 1: B tang dan
        (luma_inv < 10'd512) ? 10'd1023             :  // Vung 2: B bao hoa
        (luma_inv < 10'd768) ? (10'd1023 - off3_x4) :  // Vung 3: B giam dan
                               10'd0;                   // Vung 4: B=0

    // Ap dung thermal mode: thay the r_proc/g_proc/b_proc
    wire [9:0] r_final = thermal_mode ? th_r : r_proc;
    wire [9:0] g_final = thermal_mode ? th_g : g_proc;
    wire [9:0] b_final = thermal_mode ? th_b : b_proc;

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
    // Compare mode: duong chia doc giua man hinh (pixel_x == 319 hoac 320)
    // =========================================================================
    wire is_divider = compare_mode && ((pixel_x == 10'd319) || (pixel_x == 10'd320));

    // Nua trai (0..319): Inverse color  -- Nua phai (320..639): Natural color
    wire left_half  = (pixel_x < 10'd320);

    // Mau dao nguoc (Inverse): ~ tren 10 bit = 1023 - gia tri
    wire [9:0] r_inv = 10'd1023 - r_cam;
    wire [9:0] g_inv = 10'd1023 - g_cam;
    wire [9:0] b_inv = 10'd1023 - b_cam;

    // =========================================================================
    // MUX pixel cuoi: motion > red > thermal/proc (brightness/... > cam)
    // =========================================================================
    wire [9:0] out_r =
        is_divider                      ? 10'h3FF :
        (compare_mode && left_half)     ? r_inv   :
        (compare_mode && !left_half)    ? r_cam   :
        (box_mot_valid && draw_box_mot) ? 10'h3FF :
        show_r                          ? 10'h3FF :
        empty_debug_pixel               ? 10'h180 :
                                          r_final;

    wire [9:0] out_g =
        is_divider                      ? 10'h3FF :
        (compare_mode && left_half)     ? g_inv   :
        (compare_mode && !left_half)    ? g_cam   :
        (box_mot_valid && draw_box_mot) ? 10'h3FF :
        show_r                          ? 10'h000 :
        empty_debug_pixel               ? 10'h000 :
                                          g_final;

    wire [9:0] out_b =
		is_divider                      ? 10'h3FF :
        (compare_mode && left_half)     ? b_inv   :
        (compare_mode && !left_half)    ? b_cam   :
        (box_mot_valid && draw_box_mot) ? 10'h3FF :
        show_r                          ? 10'h000 :
        empty_debug_pixel               ? 10'h000 :
                                          b_final;

    assign VGA_R     = video_on ? out_r : 10'd0;
    assign VGA_G     = video_on ? out_g : 10'd0;
    assign VGA_B     = video_on ? out_b : 10'd0;
    assign VGA_BLANK = video_on;
    assign VGA_SYNC  = 1'b0;

endmodule