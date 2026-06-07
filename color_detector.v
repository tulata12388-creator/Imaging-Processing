`timescale 1ns / 1ps
// =============================================================
// color_detector.v  -- RED ONLY (v5)
//
// Chi giu lai tinh nang phat hien MAU DO (Red).
// Green, Yellow, Blue da duoc loai bo.
//
// Ky thuat phat hien mau Do (Ratio-based, on dinh voi camera OV7670):
//   - r_dom_g : R*8 >= G*7   (R chiem uu the han G)
//   - r_dom_b : R*2 >= B*5   (R/B >= 2.5)
//   - is_red  : px_r >= MIN_BRIGHT  VA  px_r > px_g/2  VA r_dom_g VA r_dom_b
//
// Streak filter: loc nhieu (yeu cau >= MIN_STREAK pixel lien tiep)
// Bbox accumulator: theo doi vung bao quanh (xmin/xmax/ymin/ymax)
// Hold timer: giu bbox hien thi HOLD_FRAMES frame sau khi mat vat the
//
// Interface: giong motion_detector.v, khong thay doi clock/reset/pixel port.
// =============================================================
module color_detector (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        frame_start,
    input  wire        pixel_valid,
    input  wire [15:0] pixel_data,    // RGB565
    // Bbox mau do (Red)
    output reg  [9:0]  box_r_xmin,
    output reg  [9:0]  box_r_xmax,
    output reg  [9:0]  box_r_ymin,
    output reg  [9:0]  box_r_ymax
);
    // =========================================================
    // NGUONG MAU DO -- chinh tai day neu can
    // =========================================================
    localparam [4:0] MIN_BRIGHT_RB = 5'd6;   // R >= 6 (tranh phat hien bong toi)

    // Streak / size / hold
    localparam [2:0] MIN_STREAK  = 3'd2;
    localparam [9:0] MIN_W       = 10'd4;
    localparam [9:0] MIN_H       = 10'd4;
    localparam [3:0] HOLD_FRAMES = 4'd8;

    // =========================================================
    // 1. Bo dem toa do (khong gian 320x240)
    // =========================================================
    reg [9:0] x_pos, y_pos;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_pos <= 10'd0; y_pos <= 10'd0;
        end else if (frame_start) begin
            x_pos <= 10'd0; y_pos <= 10'd0;
        end else if (pixel_valid) begin
            if (x_pos == 10'd319) begin
                x_pos <= 10'd0;
                y_pos <= (y_pos == 10'd239) ? 10'd0 : y_pos + 1'b1;
            end else
                x_pos <= x_pos + 1'b1;
        end
    end

    // Lay mau luoi 4x4
    wire pixel_sample = pixel_valid && (x_pos[1:0] == 2'b00) && (y_pos[1:0] == 2'b00);

    // =========================================================
    // 2. Tach RGB tu RGB565
    // =========================================================
    wire [4:0] px_r = pixel_data[15:11];  // 5-bit (0-31)
    wire [5:0] px_g = pixel_data[10:5];   // 6-bit (0-63)
    wire [4:0] px_b = pixel_data[4:0];    // 5-bit (0-31)

    // Mo rong chieu rong bit cho phep nhan
    wire [9:0]  r10 = {5'b0, px_r};
    wire [10:0] g11 = {5'b0, px_g};
    wire [9:0]  b10 = {5'b0, px_b};

    // =========================================================
    // 3. Phat hien MAU DO (ratio-based)
    // =========================================================
    // r_dom_g: R*8 >= G*7
    wire r_dom_g = ({r10, 3'b0} >= {g11, 2'b0} + {g11, 1'b0} + g11);
    //              R*8       >=   G*4   +  G*2  + G = G*7

    // r_dom_b: R*2 >= B*5
    wire r_dom_b = ({r10, 1'b0} >= {b10, 2'b0} + b10);
    //              R*2       >=   B*4  + B = B*5

    wire is_red = (px_r >= MIN_BRIGHT_RB)
               && (px_r > px_g[5:1])   // R > G/2 (loc do toi)
               && r_dom_g && r_dom_b;

    // =========================================================
    // 4. Pipeline delay 1 chu ky
    // =========================================================
    reg        pixel_sample_q;
    reg [9:0]  x_pos_q, y_pos_q;
    reg        red_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_sample_q <= 1'b0;
            x_pos_q <= 10'd0; y_pos_q <= 10'd0;
            red_q <= 1'b0;
        end else begin
            pixel_sample_q <= pixel_sample;
            x_pos_q <= x_pos;
            y_pos_q <= y_pos;
            if (pixel_sample)
                red_q <= is_red;
        end
    end

    // =========================================================
    // 5. Streak Filter
    // =========================================================
    reg [2:0] streak_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            streak_r <= 3'd0;
        end else if (frame_start || (pixel_sample_q && (x_pos_q == 10'd0))) begin
            streak_r <= 3'd0;
        end else if (pixel_sample_q) begin
            streak_r <= red_q ? ((&streak_r) ? streak_r : streak_r + 1'b1) : 3'd0;
        end
    end
    wire valid_r = pixel_sample_q && red_q && (streak_r >= MIN_STREAK);

    // =========================================================
    // 6. Bbox Accumulator + Hold Timer (scale x2: 320->640)
    // =========================================================
    reg [9:0] r_xmin_run, r_xmax_run, r_ymin_run, r_ymax_run;
    reg [3:0] hold_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_xmin_run <= 10'd319; r_xmax_run <= 10'd0;
            r_ymin_run <= 10'd239; r_ymax_run <= 10'd0;
            box_r_xmin <= 10'd0; box_r_xmax <= 10'd0;
            box_r_ymin <= 10'd0; box_r_ymax <= 10'd0;
            hold_r     <= 4'd0;
        end else if (frame_start) begin
            if ((r_xmax_run >= r_xmin_run) && (r_ymax_run >= r_ymin_run)
               && ((r_xmax_run - r_xmin_run) >= MIN_W)
               && ((r_ymax_run - r_ymin_run) >= MIN_H)) begin
                box_r_xmin <= r_xmin_run << 1;
                box_r_xmax <= r_xmax_run << 1;
                box_r_ymin <= r_ymin_run << 1;
                box_r_ymax <= r_ymax_run << 1;
                hold_r     <= HOLD_FRAMES;
            end else begin
                if (hold_r > 4'd0) hold_r <= hold_r - 4'd1;
                else begin
                    box_r_xmin <= 10'd0; box_r_xmax <= 10'd0;
                    box_r_ymin <= 10'd0; box_r_ymax <= 10'd0;
                end
            end
            r_xmin_run <= 10'd319; r_xmax_run <= 10'd0;
            r_ymin_run <= 10'd239; r_ymax_run <= 10'd0;
        end else if (valid_r) begin
            if (x_pos_q < r_xmin_run) r_xmin_run <= x_pos_q;
            if (x_pos_q > r_xmax_run) r_xmax_run <= x_pos_q;
            if (y_pos_q < r_ymin_run) r_ymin_run <= y_pos_q;
            if (y_pos_q > r_ymax_run) r_ymax_run <= y_pos_q;
        end
    end

endmodule