`timescale 1ns / 1ps
// =============================================================
// color_detector.v
//
// Phat hien 4 mau dong thoi: Do / Xanh la / Vang / Xanh duong
// Ket qua: 4 bo bbox doc lap, moi bo la 1 khung chu nhat
// bao quanh vung mang mau tuong ung trong frame.
//
// Ket noi y het motion_detector.v:
//   - clk, rst_n, frame_start, pixel_valid, pixel_data[15:0]
//   - pixel_data la RGB565: R[15:11] G[10:5] B[4:0]
//   - Output bbox theo khong gian VGA 640x480 (scale x2 tu 320x240)
//
// Pipeline:
//   OV7670 -> SDRAM write stream -> pixel_valid/pixel_data
//   -> RGB565 extract -> threshold -> streak filter
//   -> bbox accumulator -> hold timer -> output
//
// Thu tu SW mode (xu ly o lop tren DE2_Camera_System_SDRAM.v):
//   SW[1] -> hien bbox_red
//   SW[2] -> hien bbox_green
//   SW[3] -> hien bbox_yellow
//   SW[4] -> hien bbox_blue
//   SW[1]+SW[2]+SW[3] -> hien ca 3 bbox cung luc
// =============================================================

module color_detector (
    input  wire        clk,
    input  wire        rst_n,

    // Giao tiep voi sdram_interface_de2.v (giong motion_detector)
    input  wire        frame_start,   // Xung 1 chu ky: bat dau frame moi
    input  wire        pixel_valid,   // 1 khi pixel_data hop le
    input  wire [15:0] pixel_data,    // RGB565

    // Bbox mau do (Red) -- khong gian VGA 640x480
    output reg  [9:0]  box_r_xmin,
    output reg  [9:0]  box_r_xmax,
    output reg  [9:0]  box_r_ymin,
    output reg  [9:0]  box_r_ymax,

    // Bbox mau xanh la (Green)
    output reg  [9:0]  box_g_xmin,
    output reg  [9:0]  box_g_xmax,
    output reg  [9:0]  box_g_ymin,
    output reg  [9:0]  box_g_ymax,

    // Bbox mau vang (Yellow)
    output reg  [9:0]  box_y_xmin,
    output reg  [9:0]  box_y_xmax,
    output reg  [9:0]  box_y_ymin,
    output reg  [9:0]  box_y_ymax,

    // Bbox mau xanh duong (Blue)
    output reg  [9:0]  box_b_xmin,
    output reg  [9:0]  box_b_xmax,
    output reg  [9:0]  box_b_ymin,
    output reg  [9:0]  box_b_ymax
);

    // =========================================================
    // NGUONG MAU - CHINH TAI DAY KHI CAN
    // =========================================================
    // R kenh: 5-bit (0-31) | G kenh: 6-bit (0-63) | B kenh: 5-bit (0-31)
    //
    // Mau DO: R cao, G thap, B thap
    localparam [4:0] R_R_MIN = 5'd18;   // R >= 18  (~58%)
    localparam [5:0] R_G_MAX = 6'd20;   // G <= 20
    localparam [4:0] R_B_MAX = 5'd12;   // B <= 12
    //
    // Mau XANH LA: G cao, R thap, B thap
    localparam [4:0] G_R_MAX = 5'd12;   // R <= 12
    localparam [5:0] G_G_MIN = 6'd26;   // G >= 26  (~41%)
    localparam [4:0] G_B_MAX = 5'd14;   // B <= 14
    //
    // Mau VANG: R cao, G cao, B thap
    localparam [4:0] Y_R_MIN = 5'd18;   // R >= 18
    localparam [5:0] Y_G_MIN = 6'd28;   // G >= 28  (~44%)
    localparam [4:0] Y_B_MAX = 5'd10;   // B <= 10  (quan trong: loc vang khoi trang)
    //
    // Mau XANH DUONG: B cao, R thap, G thap
    localparam [4:0] B_R_MAX = 5'd10;   // R <= 10
    localparam [5:0] B_G_MAX = 6'd22;   // G <= 22
    localparam [4:0] B_B_MIN = 5'd18;   // B >= 18  (~58%)

    // So macro-pixel lien tiep cung mau de xac nhan la that (loc nhieu)
    localparam [2:0] MIN_STREAK = 3'd2;

    // Dien tich bbox toi thieu (don vi macro-pixel 4x4, khong gian 320x240)
    // MIN_W=6 MIN_H=5 ~ vung 24x20 px trong khong gian 320x240
    localparam [9:0] MIN_W = 10'd6;
    localparam [9:0] MIN_H = 10'd5;

    // So frame giu bbox sau khi mat mau (chong flicker)
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
                y_pos <= (y_pos == 10'd239) ? 10'd0 : y_pos + 1'b1;
            end else begin
                x_pos <= x_pos + 1'b1;
            end
        end
    end

    // Lay mau tai pixel dau cua moi o luoi 4x4 -> anh 80x60
    wire pixel_sample = pixel_valid && (x_pos[1:0] == 2'b00) && (y_pos[1:0] == 2'b00);

    // =========================================================
    // 2. Tach kenh RGB tu RGB565
    //    pixel_data[15:11] = R (5-bit)
    //    pixel_data[10:5]  = G (6-bit)
    //    pixel_data[4:0]   = B (5-bit)
    // =========================================================
    wire [4:0] px_r = pixel_data[15:11];
    wire [5:0] px_g = pixel_data[10:5];
    wire [4:0] px_b = pixel_data[4:0];

    // =========================================================
    // 3. Phat hien mau (to hop nguong RGB)
    //    Tat ca logic to hop, khong co thanh ghi -> delay = 0
    // =========================================================
    wire is_red    = (px_r >= R_R_MIN) && (px_g <= R_G_MAX) && (px_b <= R_B_MAX);
    wire is_green  = (px_r <= G_R_MAX) && (px_g >= G_G_MIN) && (px_b <= G_B_MAX);
    wire is_yellow = (px_r >= Y_R_MIN) && (px_g >= Y_G_MIN) && (px_b <= Y_B_MAX);
    wire is_blue   = (px_r <= B_R_MAX) && (px_g <= B_G_MAX) && (px_b >= B_B_MIN);

    // =========================================================
    // 4. Pipeline delay 1 chu ky
    //    Dong bo voi pixel_sample_q de x_pos_q/y_pos_q la chinh xac
    //    Chot mau TAI THOI DIEM pixel_sample=1 (giong FIX#1 motion_detector)
    // =========================================================
    reg        pixel_sample_q;
    reg [9:0]  x_pos_q;
    reg [9:0]  y_pos_q;
    reg        red_q, green_q, yellow_q, blue_q;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_sample_q <= 1'b0;
            x_pos_q        <= 10'd0;
            y_pos_q        <= 10'd0;
            red_q          <= 1'b0;
            green_q        <= 1'b0;
            yellow_q       <= 1'b0;
            blue_q         <= 1'b0;
        end else begin
            pixel_sample_q <= pixel_sample;
            x_pos_q        <= x_pos;
            y_pos_q        <= y_pos;
            // Chi chot ket qua mau khi dang o tai pixel can lay mau
            if (pixel_sample) begin
                red_q    <= is_red;
                green_q  <= is_green;
                yellow_q <= is_yellow;
                blue_q   <= is_blue;
            end
        end
    end

    // =========================================================
    // 5. Streak Filter - moi mau co bo dem rieng
    //    Reset khi bat dau hang moi (x_pos_q == 0)
    // =========================================================
    reg [2:0] streak_r, streak_g, streak_y, streak_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            streak_r <= 3'd0; streak_g <= 3'd0;
            streak_y <= 3'd0; streak_b <= 3'd0;
        end else if (frame_start || (pixel_sample_q && (x_pos_q == 10'd0))) begin
            streak_r <= 3'd0; streak_g <= 3'd0;
            streak_y <= 3'd0; streak_b <= 3'd0;
        end else if (pixel_sample_q) begin
            // Red streak
            streak_r <= red_q    ? ((&streak_r) ? streak_r : streak_r + 1'b1) : 3'd0;
            // Green streak
            streak_g <= green_q  ? ((&streak_g) ? streak_g : streak_g + 1'b1) : 3'd0;
            // Yellow streak
            streak_y <= yellow_q ? ((&streak_y) ? streak_y : streak_y + 1'b1) : 3'd0;
            // Blue streak
            streak_b <= blue_q   ? ((&streak_b) ? streak_b : streak_b + 1'b1) : 3'd0;
        end
    end

    // Tin hieu xac nhan: mau that + du streak
    wire valid_r = pixel_sample_q && red_q    && (streak_r >= MIN_STREAK);
    wire valid_g = pixel_sample_q && green_q  && (streak_g >= MIN_STREAK);
    wire valid_y = pixel_sample_q && yellow_q && (streak_y >= MIN_STREAK);
    wire valid_b = pixel_sample_q && blue_q   && (streak_b >= MIN_STREAK);

    // =========================================================
    // 6. Bbox Accumulator + Hold Timer - 4 mau doc lap
    //
    // Cau truc moi mau:
    //   x_min_run / x_max_run / y_min_run / y_max_run
    //     -> tich luy trong frame hien tai
    //   hold_timer -> dem nguoc HOLD_FRAMES frame truoc khi xoa
    //   box_*_xmin/xmax/ymin/ymax -> output (khong gian VGA 640x480)
    //
    // Scale: nhan doi (<<1) vi camera 320x240 -> VGA 640x480
    // =========================================================

    // --- Thanh ghi trung gian (run-registers trong frame) ---
    reg [9:0] r_xmin_run, r_xmax_run, r_ymin_run, r_ymax_run;
    reg [9:0] g_xmin_run, g_xmax_run, g_ymin_run, g_ymax_run;
    reg [9:0] y_xmin_run, y_xmax_run, y_ymin_run, y_ymax_run;
    reg [9:0] b_xmin_run, b_xmax_run, b_ymin_run, b_ymax_run;

    // --- Hold timers ---
    reg [3:0] hold_r, hold_g, hold_y, hold_b;

    // ---- Helper task: dung generate de gon hon ----
    // Vi Verilog khong ho tro task viet gon, ta dung 4 always block tuong tu

    // === MAU DO ===
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_xmin_run <= 10'd319; r_xmax_run <= 10'd0;
            r_ymin_run <= 10'd239; r_ymax_run <= 10'd0;
            box_r_xmin <= 10'd0;  box_r_xmax <= 10'd0;
            box_r_ymin <= 10'd0;  box_r_ymax <= 10'd0;
            hold_r     <= 4'd0;
        end else if (frame_start) begin
            // Cuoi frame: kiem tra va xuat bbox neu du lon
            if (   (r_xmax_run >= r_xmin_run)
                && (r_ymax_run >= r_ymin_run)
                && ((r_xmax_run - r_xmin_run) >= MIN_W)
                && ((r_ymax_run - r_ymin_run) >= MIN_H)
            ) begin
                box_r_xmin <= r_xmin_run << 1;
                box_r_xmax <= r_xmax_run << 1;
                box_r_ymin <= r_ymin_run << 1;
                box_r_ymax <= r_ymax_run << 1;
                hold_r     <= HOLD_FRAMES;
            end else begin
                if (hold_r > 4'd0)
                    hold_r <= hold_r - 4'd1;
                else begin
                    box_r_xmin <= 10'd0; box_r_xmax <= 10'd0;
                    box_r_ymin <= 10'd0; box_r_ymax <= 10'd0;
                end
            end
            // Reset run-registers cho frame tiep theo
            r_xmin_run <= 10'd319; r_xmax_run <= 10'd0;
            r_ymin_run <= 10'd239; r_ymax_run <= 10'd0;
        end else if (valid_r) begin
            if (x_pos_q < r_xmin_run) r_xmin_run <= x_pos_q;
            if (x_pos_q > r_xmax_run) r_xmax_run <= x_pos_q;
            if (y_pos_q < r_ymin_run) r_ymin_run <= y_pos_q;
            if (y_pos_q > r_ymax_run) r_ymax_run <= y_pos_q;
        end
    end

    // === MAU XANH LA ===
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            g_xmin_run <= 10'd319; g_xmax_run <= 10'd0;
            g_ymin_run <= 10'd239; g_ymax_run <= 10'd0;
            box_g_xmin <= 10'd0;  box_g_xmax <= 10'd0;
            box_g_ymin <= 10'd0;  box_g_ymax <= 10'd0;
            hold_g     <= 4'd0;
        end else if (frame_start) begin
            if (   (g_xmax_run >= g_xmin_run)
                && (g_ymax_run >= g_ymin_run)
                && ((g_xmax_run - g_xmin_run) >= MIN_W)
                && ((g_ymax_run - g_ymin_run) >= MIN_H)
            ) begin
                box_g_xmin <= g_xmin_run << 1;
                box_g_xmax <= g_xmax_run << 1;
                box_g_ymin <= g_ymin_run << 1;
                box_g_ymax <= g_ymax_run << 1;
                hold_g     <= HOLD_FRAMES;
            end else begin
                if (hold_g > 4'd0)
                    hold_g <= hold_g - 4'd1;
                else begin
                    box_g_xmin <= 10'd0; box_g_xmax <= 10'd0;
                    box_g_ymin <= 10'd0; box_g_ymax <= 10'd0;
                end
            end
            g_xmin_run <= 10'd319; g_xmax_run <= 10'd0;
            g_ymin_run <= 10'd239; g_ymax_run <= 10'd0;
        end else if (valid_g) begin
            if (x_pos_q < g_xmin_run) g_xmin_run <= x_pos_q;
            if (x_pos_q > g_xmax_run) g_xmax_run <= x_pos_q;
            if (y_pos_q < g_ymin_run) g_ymin_run <= y_pos_q;
            if (y_pos_q > g_ymax_run) g_ymax_run <= y_pos_q;
        end
    end

    // === MAU VANG ===
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            y_xmin_run <= 10'd319; y_xmax_run <= 10'd0;
            y_ymin_run <= 10'd239; y_ymax_run <= 10'd0;
            box_y_xmin <= 10'd0;  box_y_xmax <= 10'd0;
            box_y_ymin <= 10'd0;  box_y_ymax <= 10'd0;
            hold_y     <= 4'd0;
        end else if (frame_start) begin
            if (   (y_xmax_run >= y_xmin_run)
                && (y_ymax_run >= y_ymin_run)
                && ((y_xmax_run - y_xmin_run) >= MIN_W)
                && ((y_ymax_run - y_ymin_run) >= MIN_H)
            ) begin
                box_y_xmin <= y_xmin_run << 1;
                box_y_xmax <= y_xmax_run << 1;
                box_y_ymin <= y_ymin_run << 1;
                box_y_ymax <= y_ymax_run << 1;
                hold_y     <= HOLD_FRAMES;
            end else begin
                if (hold_y > 4'd0)
                    hold_y <= hold_y - 4'd1;
                else begin
                    box_y_xmin <= 10'd0; box_y_xmax <= 10'd0;
                    box_y_ymin <= 10'd0; box_y_ymax <= 10'd0;
                end
            end
            y_xmin_run <= 10'd319; y_xmax_run <= 10'd0;
            y_ymin_run <= 10'd239; y_ymax_run <= 10'd0;
        end else if (valid_y) begin
            if (x_pos_q < y_xmin_run) y_xmin_run <= x_pos_q;
            if (x_pos_q > y_xmax_run) y_xmax_run <= x_pos_q;
            if (y_pos_q < y_ymin_run) y_ymin_run <= y_pos_q;
            if (y_pos_q > y_ymax_run) y_ymax_run <= y_pos_q;
        end
    end

    // === MAU XANH DUONG ===
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_xmin_run <= 10'd319; b_xmax_run <= 10'd0;
            b_ymin_run <= 10'd239; b_ymax_run <= 10'd0;
            box_b_xmin <= 10'd0;  box_b_xmax <= 10'd0;
            box_b_ymin <= 10'd0;  box_b_ymax <= 10'd0;
            hold_b     <= 4'd0;
        end else if (frame_start) begin
            if (   (b_xmax_run >= b_xmin_run)
                && (b_ymax_run >= b_ymin_run)
                && ((b_xmax_run - b_xmin_run) >= MIN_W)
                && ((b_ymax_run - b_ymin_run) >= MIN_H)
            ) begin
                box_b_xmin <= b_xmin_run << 1;
                box_b_xmax <= b_xmax_run << 1;
                box_b_ymin <= b_ymin_run << 1;
                box_b_ymax <= b_ymax_run << 1;
                hold_b     <= HOLD_FRAMES;
            end else begin
                if (hold_b > 4'd0)
                    hold_b <= hold_b - 4'd1;
                else begin
                    box_b_xmin <= 10'd0; box_b_xmax <= 10'd0;
                    box_b_ymin <= 10'd0; box_b_ymax <= 10'd0;
                end
            end
            b_xmin_run <= 10'd319; b_xmax_run <= 10'd0;
            b_ymin_run <= 10'd239; b_ymax_run <= 10'd0;
        end else if (valid_b) begin
            if (x_pos_q < b_xmin_run) b_xmin_run <= x_pos_q;
            if (x_pos_q > b_xmax_run) b_xmax_run <= x_pos_q;
            if (y_pos_q < b_ymin_run) b_ymin_run <= y_pos_q;
            if (y_pos_q > b_ymax_run) b_ymax_run <= y_pos_q;
        end
    end

endmodule