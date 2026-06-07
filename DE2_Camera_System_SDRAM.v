`timescale 1ns / 1ps
// =============================================================
// DE2 Cyclone II EP2C35 + OV7670 + SDRAM framebuffer
// Top-level co them:
//   - SW[1] : detect mau do -> hien bbox_red
//   - SW[2] : Flip ngang (Horizontal Mirror, Left-Right)
//   - SW[3] : Rotate 180 (Flip ngang + Flip doc)
//   - SW[17]: motion detection mode (giu nguyen)
//   - HEX display: tuy theo SW mode hien thi toa do bbox
// =============================================================
module DE2_Camera_System_SDRAM(
    input  wire        CLOCK_50,
    input  wire [0:0]  KEY,
    input  wire [17:0] SW,
    // OV7670
    input  wire        OV7670_PCLK,
    input  wire        OV7670_VSYNC,
    input  wire        OV7670_HREF,
    input  wire [7:0]  OV7670_D,
    output wire        OV7670_XCLK,
    output wire        OV7670_SIOC,
    inout  wire        OV7670_SIOD,
    output wire        OV7670_RESET,
    output wire        OV7670_PWDN,
    // VGA
    output wire [9:0]  VGA_R,
    output wire [9:0]  VGA_G,
    output wire [9:0]  VGA_B,
    output wire        VGA_HS,
    output wire        VGA_VS,
    output wire        VGA_CLK,
    output wire        VGA_BLANK,
    output wire        VGA_SYNC,
    // SDRAM
    output wire        DRAM_CLK,
    output wire        DRAM_CKE,
    output wire        DRAM_CS_N,
    output wire        DRAM_RAS_N,
    output wire        DRAM_CAS_N,
    output wire        DRAM_WE_N,
    output wire [11:0] DRAM_ADDR,
    output wire [1:0]  DRAM_BA,
    output wire [1:0]  DRAM_DQM,
    inout  wire [15:0] DRAM_DQ,
    // HEX display
    // HEX0-HEX2: toa do X cua bbox dang active
    // HEX4-HEX6: toa do Y cua bbox dang active
    output wire [6:0]  HEX0, HEX1, HEX2, HEX3,
    output wire [6:0]  HEX4, HEX5, HEX6, HEX7,
    // Debug LED
    output wire        LED_TEST
);
    // KEY[0] active-low reset
    wire rst_n = KEY[0];

    // =========================================================
    // Switch decode
    // SW[0]  -> mode normal (khong overlay)
    // SW[1]  -> mode detect mau do
    // SW[2]  -> Flip ngang (Horizontal Mirror)
    // SW[3]  -> Rotate 180 (flip_h + flip_v)
    // SW[17] -> motion detection mode
    // =========================================================
    wire mode_normal  = SW[0];
    wire mode_red     = SW[1];
    wire flip_h       = SW[2];   // Flip ngang: dao trai-phai trong VGA layer
    wire flip_v       = SW[3];   // Flip doc  : SDRAM doc nguoc thu tu dong
                                 // Rotate 180 = SW[3] (flip_h va flip_v deu active)

    wire enable_motion_mode = SW[17];

    // color_mode[0]: bit0=red (cac bit con lai khong dung)
    wire [3:0] color_mode;
    assign color_mode[0] = mode_red;
    assign color_mode[3:1] = 3'b000;

    // =========================================================
    // PLL
    // c0 = 50 MHz internal SDRAM/controller clock
    // c1 = 50 MHz phase-shifted SDRAM clock output
    // c2 = 25 MHz VGA clock + OV7670 XCLK
    // =========================================================
    wire clk_sdram;
    wire clk_sdram_out;
    wire clk_vga;
    wire pll_locked;
    sdram_pll_50 u_pll (
        .inclk0 (CLOCK_50),
        .c0     (clk_sdram),
        .c1     (clk_sdram_out),
        .c2     (clk_vga),
        .locked (pll_locked)
    );
    wire sys_rst_n;
    assign sys_rst_n = rst_n & pll_locked;
    assign VGA_CLK     = clk_vga;
    assign OV7670_XCLK = clk_vga;
    assign DRAM_CLK    = clk_sdram_out;

    wire sdram_clk_unused;

    // =========================================================
    // Camera FIFO -> SDRAM
    // =========================================================
    wire        f2s_data_valid;
    wire [9:0]  data_count_r;
    wire [15:0] cam_fifo_dout;
    // =========================================================
    // SDRAM -> VGA FIFO
    // =========================================================
    wire [15:0] vga_fifo_dout;
    wire        empty_vga_fifo;
    wire        rd_en;
    // =========================================================
    // Frame sync
    // =========================================================
    wire        cam_frame_start;
    wire        vga_frame_start;

    wire [3:0] led_dbg;

    // =========================================================
    // Motion detector bbox
    // =========================================================
    wire [9:0] box_x_min;
    wire [9:0] box_x_max;
    wire [9:0] box_y_min;
    wire [9:0] box_y_max;

    // =========================================================
    // Color detector: bbox mau do
    // =========================================================
    wire [9:0] box_r_xmin, box_r_xmax, box_r_ymin, box_r_ymax;

    // =========================================================
    // Camera interface
    // =========================================================
    camera_interface_de2 u_camera (
        .clk_cfg          (clk_sdram),
        .rst_n            (sys_rst_n),
        .key              (4'b1111),
        .rd_en            (f2s_data_valid),
        .data_count_r     (data_count_r),
        .dout             (cam_fifo_dout),
        .cmos_pclk        (OV7670_PCLK),
        .cmos_href        (OV7670_HREF),
        .cmos_vsync       (OV7670_VSYNC),
        .cmos_db          (OV7670_D),
        .cmos_sda         (OV7670_SIOD),
        .cmos_scl         (OV7670_SIOC),
        .cmos_rst_n       (OV7670_RESET),
        .cmos_pwdn        (OV7670_PWDN),
        .cmos_xclk        (),
        .clk_xclk         (clk_vga),
        .cam_frame_start  (cam_frame_start),
        .led              (led_dbg)
    );

    // =========================================================
    // SDRAM interface (co color_detector ben trong)
    // =========================================================
    sdram_interface_de2 u_sdram_if (
        .clk             (clk_sdram),
        .rst_n           (sys_rst_n),
        .flip_v          (flip_v),
        .clk_vga         (clk_vga),
        .vga_frame_start (vga_frame_start),
        .rd_en           (rd_en),
        .cam_frame_start (cam_frame_start),
        .data_count_r    (data_count_r),
        .f2s_data        (cam_fifo_dout),
        .f2s_data_valid  (f2s_data_valid),
        .empty_fifo      (empty_vga_fifo),
        .dout            (vga_fifo_dout),
        // Motion bbox
        .box_x_min       (box_x_min),
        .box_x_max       (box_x_max),
        .box_y_min       (box_y_min),
        .box_y_max       (box_y_max),
        // Color bbox - Red
        .box_r_xmin      (box_r_xmin),
        .box_r_xmax      (box_r_xmax),
        .box_r_ymin      (box_r_ymin),
        .box_r_ymax      (box_r_ymax),
        // SDRAM pins
        .sdram_clk       (sdram_clk_unused),
        .sdram_cke       (DRAM_CKE),
        .sdram_cs_n      (DRAM_CS_N),
        .sdram_ras_n     (DRAM_RAS_N),
        .sdram_cas_n     (DRAM_CAS_N),
        .sdram_we_n      (DRAM_WE_N),
        .sdram_addr      (DRAM_ADDR),
        .sdram_ba        (DRAM_BA),
        .sdram_dqm       (DRAM_DQM),
        .sdram_dq        (DRAM_DQ)
    );

    // =========================================================
    // VGA interface (overlay khung do)
    // =========================================================
    vga_interface_de2 u_vga (
        .clk_vga         (clk_vga),
        .rst_n           (sys_rst_n),
        .empty_fifo      (empty_vga_fifo),
        .din             (vga_fifo_dout),
        .rd_en           (rd_en),
        .vga_frame_start (vga_frame_start),
        // Image Flipping
        .flip_h          (flip_h),
        .flip_v          (flip_v),
        // Motion bbox
        .box_x_min       (box_x_min),
        .box_x_max       (box_x_max),
        .box_y_min       (box_y_min),
        .box_y_max       (box_y_max),
        .enable_box      (enable_motion_mode),
        // Color bbox - Red
        .box_r_xmin      (box_r_xmin),
        .box_r_xmax      (box_r_xmax),
        .box_r_ymin      (box_r_ymin),
        .box_r_ymax      (box_r_ymax),
        // Color mode mask
        .color_mode      (color_mode),
        // VGA output
        .VGA_R           (VGA_R),
        .VGA_G           (VGA_G),
        .VGA_B           (VGA_B),
        .VGA_HS          (VGA_HS),
        .VGA_VS          (VGA_VS),
        .VGA_BLANK       (VGA_BLANK),
        .VGA_SYNC        (VGA_SYNC)
    );

    assign LED_TEST = led_dbg[0];

    // =========================================================
    // HEX display logic:
    //   SW[1]  -> bbox do (box_r)
    //   SW[17] -> bbox motion
    //   Else   -> hien 0
    // HEX0-HEX2: X min; HEX4-HEX6: Y min; HEX3,HEX7: blank
    // =========================================================
    reg [9:0] hex_xmin;
    reg [9:0] hex_ymin;

    always @(*) begin
        if (enable_motion_mode) begin
            hex_xmin = box_x_min;
            hex_ymin = box_y_min;
        end else if (mode_red) begin
            hex_xmin = box_r_xmin;
            hex_ymin = box_r_ymin;
        end else begin
            hex_xmin = 10'd0;
            hex_ymin = 10'd0;
        end
    end

    segment7 hex0 (.bin(hex_xmin[3:0]),          .seg(HEX0));
    segment7 hex1 (.bin(hex_xmin[7:4]),           .seg(HEX1));
    segment7 hex2 (.bin({2'b00, hex_xmin[9:8]}),  .seg(HEX2));
    assign HEX3 = 7'h7F;  // blank

    segment7 hex4 (.bin(hex_ymin[3:0]),           .seg(HEX4));
    segment7 hex5 (.bin(hex_ymin[7:4]),            .seg(HEX5));
    segment7 hex6 (.bin({2'b00, hex_ymin[9:8]}),  .seg(HEX6));
    assign HEX7 = 7'h7F;  // blank

endmodule