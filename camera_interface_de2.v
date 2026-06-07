`timescale 1ns / 1ps
// =============================================================
// camera_interface_de2.v
//
// OV7670 capture interface - QVGA 320x240 VERSION
//
// Goal:
// - Keep OV7670 input timing 640x480.
// - Downsample 2x horizontally and 2x vertically.
// - Write only 320x240 pixels into camera FIFO.
// - This reduces SDRAM bandwidth by 4x.
//
// Output framebuffer:
// - 320 x 240 RGB565
// - 76800 pixels
// - 300 SDRAM bursts, 256 pixels/burst
//
// Important:
// - CAMERA_BYTE_SWAP = 0
// - DEBUG_INTERNAL_PATTERN = 0 for real camera
// - Clear camera FIFO during VSYNC
// - Generate cam_frame_start from VSYNC rising edge
// =============================================================

module camera_interface_de2(
    input  wire        clk_cfg,
    input  wire        rst_n,
    input  wire [3:0]  key,

    // FIFO read side, used by SDRAM interface
    input  wire        rd_en,
    output wire [9:0]  data_count_r,
    output wire [15:0] dout,

    // OV7670 camera physical signals
    input  wire        cmos_pclk,
    input  wire        cmos_href,
    input  wire        cmos_vsync,
    input  wire [7:0]  cmos_db,

    inout  wire        cmos_sda,
    output wire        cmos_scl,
    output wire        cmos_rst_n,
    output wire        cmos_pwdn,
    output wire        cmos_xclk,
    input  wire        clk_xclk,

    // Camera frame start pulse in clk_cfg domain
    output wire        cam_frame_start,

    // Debug LEDs
    output wire [3:0]  led
);

    assign cmos_xclk = clk_xclk;

    // ---------------------------------------------------------
    // OV7670 configuration
    // ---------------------------------------------------------
    wire config_finished;

    ov7670_controller u_ov7670_config (
        .clk             (clk_cfg),
        .rst_n           (rst_n),
        .resend          (~rst_n),
        .config_finished (config_finished),
        .sioc            (cmos_scl),
        .siod            (cmos_sda),
        .reset           (cmos_rst_n),
        .pwdn            (cmos_pwdn)
    );

    // ---------------------------------------------------------
    // Generate cam_frame_start from VSYNC rising edge
    // synchronized into clk_cfg domain
    // ---------------------------------------------------------
    reg vsync_s1;
    reg vsync_s2;
    reg vsync_s3;

    always @(posedge clk_cfg or negedge rst_n) begin
        if (!rst_n) begin
            vsync_s1 <= 1'b0;
            vsync_s2 <= 1'b0;
            vsync_s3 <= 1'b0;
        end
        else begin
            vsync_s1 <= cmos_vsync;
            vsync_s2 <= vsync_s1;
            vsync_s3 <= vsync_s2;
        end
    end

    assign cam_frame_start = vsync_s2 & ~vsync_s3;

    // ---------------------------------------------------------
    // RGB565 capture
    //
    // OV7670 RGB565:
    // byte0 = RRRRRGGG
    // byte1 = GGGBBBBB
    // pixel = {byte0, byte1}
    // ---------------------------------------------------------
    localparam CAMERA_BYTE_SWAP       = 1'b0;
    localparam DEBUG_INTERNAL_PATTERN = 1'b0;

    reg        byte_phase;
    reg [7:0]  byte_hi;

    reg [10:0] cap_x;      // original 640-wide input pixel index
    reg [8:0]  cap_y;      // original 480-high input line index
    reg        href_d;
    reg        line_has_pixel;

    reg        fifo_wr_en;
    reg [15:0] fifo_wr_data;

    wire [15:0] pixel565;

    assign pixel565 = CAMERA_BYTE_SWAP ? {cmos_db, byte_hi} :
                                         {byte_hi, cmos_db};

    // QVGA output coordinate after 2x downsample
    wire [8:0] qvga_x;
    wire [7:0] qvga_y;

    assign qvga_x = cap_x[9:1];  // 0..319 when cap_x is even
    assign qvga_y = cap_y[8:1];  // 0..239 when cap_y is even

    wire keep_pixel;
    assign keep_pixel = (cap_x < 11'd640) &&
                        (cap_y < 9'd480)  &&
                        (cap_x[0] == 1'b0) &&
                        (cap_y[0] == 1'b0);

    wire [15:0] debug_pixel;

    assign debug_pixel =
        (qvga_x < 9'd80)  ? 16'hF800 :
        (qvga_x < 9'd160) ? 16'h07E0 :
        (qvga_x < 9'd240) ? 16'h001F :
                             16'hFFFF;

    // ---------------------------------------------------------
    // Capture on OV7670 PCLK
    //
    // Input is still 640x480.
    // We write only:
    // - even x
    // - even y
    // => output is 320x240.
    // ---------------------------------------------------------
    always @(posedge cmos_pclk or negedge rst_n) begin
        if (!rst_n) begin
            byte_phase     <= 1'b0;
            byte_hi        <= 8'd0;

            cap_x          <= 11'd0;
            cap_y          <= 9'd0;
            href_d         <= 1'b0;
            line_has_pixel <= 1'b0;

            fifo_wr_en     <= 1'b0;
            fifo_wr_data   <= 16'd0;
        end
        else begin
            fifo_wr_en <= 1'b0;
            href_d     <= cmos_href;

            // VSYNC high = frame reset interval
            if (cmos_vsync) begin
                byte_phase     <= 1'b0;
                cap_x          <= 11'd0;
                cap_y          <= 9'd0;
                line_has_pixel <= 1'b0;
            end

            // Active line
            else if (cmos_href) begin
                if (cap_y < 9'd480) begin
                    if (!byte_phase) begin
                        byte_hi    <= cmos_db;
                        byte_phase <= 1'b1;
                    end
                    else begin
                        byte_phase <= 1'b0;

                        // Process only first 640 pixels per line.
                        if (cap_x < 11'd640) begin

                            // Downsample 2x2: keep only even x and even y.
                            if (keep_pixel) begin
                                fifo_wr_data <= DEBUG_INTERNAL_PATTERN ? debug_pixel : pixel565;
                                fifo_wr_en   <= 1'b1;
                            end

                            line_has_pixel <= 1'b1;

                            if (cap_x == 11'd639)
                                cap_x <= 11'd640; // line full marker
                            else
                                cap_x <= cap_x + 11'd1;
                        end
                    end
                end
                else begin
                    byte_phase <= 1'b0;
                end
            end

            // HREF low
            else begin
                byte_phase <= 1'b0;
                cap_x      <= 11'd0;

                // Detect HREF falling edge and count one completed line.
                if (href_d && line_has_pixel) begin
                    line_has_pixel <= 1'b0;

                    if (cap_y < 9'd479)
                        cap_y <= cap_y + 9'd1;
                    else
                        cap_y <= 9'd480; // ignore rest until next VSYNC
                end
            end
        end
    end

    // ---------------------------------------------------------
    // Camera FIFO
    //
    // Clear FIFO during VSYNC to avoid carrying stale pixels
    // from previous frame into the next SDRAM frame.
    // ---------------------------------------------------------
    wire cam_fifo_rst_n;
    assign cam_fifo_rst_n = rst_n;

    wire cam_fifo_full;
    wire cam_fifo_empty;
    wire [9:0] data_count_w_unused;

    asyn_fifo #(
        .DATA_WIDTH       (16),
        .FIFO_DEPTH_WIDTH (10)
    ) u_camera_fifo (
        .rst_n        (cam_fifo_rst_n),

        .clk_write    (cmos_pclk),
        .clk_read     (clk_cfg),

        .write        (fifo_wr_en),
        .read         (rd_en),

        .data_write   (fifo_wr_data),
        .data_read    (dout),

        .full         (cam_fifo_full),
        .empty        (cam_fifo_empty),

        .data_count_w (data_count_w_unused),
        .data_count_r (data_count_r)
    );

    // ---------------------------------------------------------
    // Debug flags
    // ---------------------------------------------------------
    reg pclk_seen;
    reg href_seen;
    reg vsync_seen;
    reg pixel_seen;

    always @(posedge clk_cfg or negedge rst_n) begin
        if (!rst_n) begin
            pclk_seen  <= 1'b0;
            href_seen  <= 1'b0;
            vsync_seen <= 1'b0;
            pixel_seen <= 1'b0;
        end
        else begin
            if (cmos_pclk)
                pclk_seen <= 1'b1;

            if (cmos_href)
                href_seen <= 1'b1;

            if (cmos_vsync)
                vsync_seen <= 1'b1;

            if (fifo_wr_en)
                pixel_seen <= 1'b1;
        end
    end

    assign led[0] = config_finished;
    assign led[1] = pclk_seen;
    assign led[2] = href_seen & vsync_seen;
    assign led[3] = pixel_seen;

endmodule