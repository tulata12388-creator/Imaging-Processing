`timescale 1ns / 1ps
// =============================================================
// motion_detector.v  -  FIXED v2
//
// Do phan giai quet goc : 320x240 (QVGA)
// Do phan giai tinh toan: 80x60  (lay mau o luoi 4x4)
// Thuat toan: True Temporal Frame Differencing
//             + Streak Filter (loc nhieu hang ngang)
//             + Min-Area Filter (loc bbox qua nho)
//             + Hold Timer (giu bbox khi vat dung lai, chong flicker)
//
// ------ DANH SACH SUA SO VOI v1 ------
// FIX 1: curr_gray_q bat dung chu ky pixel_sample (khong phai chu ky sau)
// FIX 2: ram_addr dung shift thay cho phep nhan *80 (tiet kiem LE)
// FIX 3: Min-area filter - chi xuat bbox khi du dien tich
// FIX 4: Hold timer - giu bbox them HOLD_FRAMES frame sau khi mat chuyen dong
// =============================================================

module motion_detector (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,   // Xung 1 chu ky bao bat dau frame moi
    input  wire        pixel_valid,   // Pixel data hop le
    input  wire [15:0] pixel_data,    // RGB565

    // Ket noi voi DE2_Camera_System_SDRAM.v
    output reg  [9:0]  box_x_min,
    output reg  [9:0]  box_x_max,
    output reg  [9:0]  box_y_min,
    output reg  [9:0]  box_y_max
);

    // =========================================================
    // THAM SO TINH CHINH (sua tai day)
    // =========================================================

    // Nguong do lech kenh Green: 15 = nhaycam vua.
    // Giam xuong 10-12 neu muon phat hien chuyen dong nhe hon.
    // Tang len 18-20 neu bi false-positive nhieu.
    localparam [5:0] THRESHOLD  = 6'd15;

    // So macro-pixel lien tiep de cong nhan la chuyen dong that
    // (loc nhieu hat don le). Tang len 3-4 neu con bao nhieu.
    localparam [3:0] MIN_STREAK = 4'd2;

    // Dien tich bbox toi thieu (don vi: macro-pixel 4x4, khong gian 320x240)
    // Gia tri 8 tuong duong vung ~32x32 px tren VGA. Tang len neu can.
    localparam [9:0] MIN_W      = 10'd8;
    localparam [9:0] MIN_H      = 10'd6;

    // So frame giu bbox sau khi mat chuyen dong (chong flicker)
    // 8 frame ~ 267ms @ 30fps. Tang len de giu khung lau hon.
    localparam [3:0] HOLD_FRAMES = 4'd8;

    // =========================================================
    // 1. Bo dem toa do goc (khong gian 320x240)
    // =========================================================
    reg [9:0] x_pos;
    reg [9:0] y_pos;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_pos <= 10'd0;
            y_pos <= 10'd0;
        end else if (frame_start) begin
            x_pos <= 10'd0;
            y_pos <= 10'd0;
        end else if (pixel_valid) begin
            if (x_pos == 10'd319) begin
                x_pos <= 10'd0;
                if (y_pos == 10'd239)
                    y_pos <= 10'd0;
                else
                    y_pos <= y_pos + 1'b1;
            end else begin
                x_pos <= x_pos + 1'b1;
            end
        end
    end

    // Lay mau tai pixel dau tien cua moi o luoi 4x4 -> anh 80x60
    wire pixel_sample = pixel_valid && (x_pos[1:0] == 2'b00) && (y_pos[1:0] == 2'b00);

    // =========================================================
    // 2. RAM noi luu frame truoc (80x60 = 4800 o, moi o 6-bit Green)
    //
    // FIX 2: dung shift thay phep nhan *80
    //   y * 80 = y*64 + y*16 = (y<<6) + (y<<4)
    // =========================================================
    reg  [5:0]  frame_ram [0:4799];
    wire [6:0]  y_idx    = y_pos[9:2];          // 0..59
    wire [6:0]  x_idx    = x_pos[9:2];          // 0..79
    wire [12:0] ram_addr = ({6'd0, y_idx} << 6)
                         + ({6'd0, y_idx} << 4)
                         + {6'd0, x_idx};        // y*80 + x

    reg [5:0] prev_gray;

    always @(posedge clk) begin
        if (pixel_sample) begin
            prev_gray          <= frame_ram[ram_addr];
            frame_ram[ram_addr] <= pixel_data[10:5];   // luu kenh Green frame hien tai
        end
    end

    // =========================================================
    // 3. Pipeline delay 1 chu ky de dong bo du lieu doc RAM
    //
    // FIX 1: curr_gray_q chi chot khi pixel_sample = 1
    //        (khong phai chot vao chu ky SAU pixel_sample)
    // =========================================================
    reg        pixel_sample_q;
    reg [9:0]  x_pos_q;
    reg [9:0]  y_pos_q;
    reg [5:0]  curr_gray_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_sample_q <= 1'b0;
            x_pos_q        <= 10'd0;
            y_pos_q        <= 10'd0;
            curr_gray_q    <= 6'd0;
        end else begin
            pixel_sample_q <= pixel_sample;
            x_pos_q        <= x_pos;
            y_pos_q        <= y_pos;
            // FIX 1: Chot gia tri kenh Green DUNG KHI pixel_sample = 1,
            // de chu ky sau (khi pixel_sample_q = 1 va prev_gray san sang)
            // curr_gray_q va prev_gray la cung 1 pixel, cung 1 vi tri.
            if (pixel_sample)
                curr_gray_q <= pixel_data[10:5];
        end
    end

    // Phep tru tuyet doi: do lech Green giua frame hien tai va frame truoc
    wire [5:0] diff           = (curr_gray_q > prev_gray)
                                ? (curr_gray_q - prev_gray)
                                : (prev_gray   - curr_gray_q);
    wire       motion_detected = (diff > THRESHOLD);

    // =========================================================
    // 4. Streak Filter - loc nhieu hat don le theo hang ngang
    // =========================================================
    reg [3:0] streak_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            streak_cnt <= 4'd0;
        end else if (frame_start || (pixel_sample_q && (x_pos_q == 10'd0))) begin
            streak_cnt <= 4'd0;
        end else if (pixel_sample_q) begin
            if (motion_detected) begin
                if (streak_cnt < 4'd15)
                    streak_cnt <= streak_cnt + 1'b1;
            end else begin
                streak_cnt <= 4'd0;
            end
        end
    end

    // =========================================================
    // 5. Tich luy toa do cuc bien (Min/Max) trong frame hien tai
    // =========================================================
    reg [9:0] x_min_run, x_max_run;
    reg [9:0] y_min_run, y_max_run;

    // FIX 4: Hold timer - dem so frame can giu bbox sau khi mat chuyen dong
    reg [3:0] hold_timer;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_min_run  <= 10'd319; x_max_run  <= 10'd0;
            y_min_run  <= 10'd239; y_max_run  <= 10'd0;
            box_x_min  <= 10'd0;  box_x_max  <= 10'd0;
            box_y_min  <= 10'd0;  box_y_max  <= 10'd0;
            hold_timer <= 4'd0;
        end
        else if (frame_start) begin

            // --- Cuoi frame: danh gia va xuat bbox ---

            // FIX 3: Chi xuat khi co chuyen dong va bbox du lon
            if (   (x_max_run >= x_min_run)
                && (y_max_run >= y_min_run)
                && ((x_max_run - x_min_run) >= MIN_W)
                && ((y_max_run - y_min_run) >= MIN_H)
            ) begin
                // Scale x2: khong gian 320x240 -> VGA 640x480
                box_x_min  <= x_min_run << 1;
                box_x_max  <= x_max_run << 1;
                box_y_min  <= y_min_run << 1;
                box_y_max  <= y_max_run << 1;
                hold_timer <= HOLD_FRAMES;      // reset hold timer
            end
            else begin
                // FIX 4: Neu het chuyen dong, dem nguoc hold_timer
                // truoc khi xoa bbox (tranh flicker khi vat dung ngan)
                if (hold_timer > 4'd0) begin
                    hold_timer <= hold_timer - 4'd1;
                    // Giu nguyen box_x/y_min/max - khong cap nhat
                end else begin
                    // Qua thoi gian giu -> xoa bbox
                    box_x_min <= 10'd0; box_x_max <= 10'd0;
                    box_y_min <= 10'd0; box_y_max <= 10'd0;
                end
            end

            // Khoi tao lai run-registers cho frame tiep theo
            x_min_run <= 10'd319; x_max_run <= 10'd0;
            y_min_run <= 10'd239; y_max_run <= 10'd0;
        end

        // Cap nhat toa do cuc bien khi phat hien chuyen dong hop le
        else if (pixel_sample_q && motion_detected && (streak_cnt >= MIN_STREAK)) begin
            if (x_pos_q < x_min_run) x_min_run <= x_pos_q;
            if (x_pos_q > x_max_run) x_max_run <= x_pos_q;
            if (y_pos_q < y_min_run) y_min_run <= y_pos_q;
            if (y_pos_q > y_max_run) y_max_run <= y_pos_q;
        end
    end

endmodule