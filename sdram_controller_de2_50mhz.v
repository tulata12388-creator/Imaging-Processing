`timescale 1ns / 1ps
// =============================================================
// DE2 SDRAM controller, 50 MHz
//
// DE2 SDRAM assumption:
// - 16-bit data bus
// - 12-bit row address
// - 2 bank bits
// - burst = 256 words
//
// FIXED VERSION:
// - Read burst outputs exactly 256 valid words.
// - Write burst consumes/writes exactly 256 words.
// - Fix write-data alignment with synchronous FIFO.
// - Avoid dropping/shifting 1 word per 256-word burst.
// =============================================================

module sdram_controller_de2_50mhz(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        rw,       // 1 = read, 0 = write
    input  wire        rw_en,
    input  wire [13:0] f_addr,   // {row[11:0], bank[1:0]}

    input  wire [15:0] f2s_data,
    output wire [15:0] s2f_data,
    output wire        s2f_data_valid,
    output reg         f2s_data_valid,
    output reg         ready,

    output wire        s_clk,
    output wire        s_cke,
    output wire        s_cs_n,
    output wire        s_ras_n,
    output wire        s_cas_n,
    output wire        s_we_n,
    output wire [11:0] s_addr,
    output wire [1:0]  s_ba,
    output wire        LDQM,
    output wire        HDQM,
    inout  wire [15:0] s_dq
);

    assign s_clk = clk;

    // ---------------------------------------------------------
    // FSM states
    // ---------------------------------------------------------
    localparam [3:0] start          = 4'd0;
    localparam [3:0] precharge_init = 4'd1;
    localparam [3:0] refresh_1      = 4'd2;
    localparam [3:0] refresh_2      = 4'd3;
    localparam [3:0] load_mode_reg  = 4'd4;
    localparam [3:0] idle           = 4'd5;
    localparam [3:0] read           = 4'd6;
    localparam [3:0] read_data      = 4'd7;
    localparam [3:0] write          = 4'd8;
    localparam [3:0] write_burst    = 4'd9;
    localparam [3:0] refresh        = 4'd10;
    localparam [3:0] delay          = 4'd11;
    localparam [3:0] write_last     = 4'd12;

    // ---------------------------------------------------------
    // Timing at 50 MHz, 20 ns period
    // ---------------------------------------------------------
    localparam [3:0] t_RP  = 4'd2;
    localparam [3:0] t_RC  = 4'd4;
    localparam [3:0] t_MRD = 4'd2;
    localparam [3:0] t_RCD = 4'd2;
    localparam [3:0] t_WR  = 4'd2;
    localparam [3:0] t_CL  = 4'd3;

    // 256-word burst: index 0..255
    localparam [9:0] BURST_LAST = 10'd255;

    // Commands: {CS_N, RAS_N, CAS_N, WE_N}
    localparam [3:0] cmd_precharge = 4'b0010;
    localparam [3:0] cmd_NOP       = 4'b1111;
    localparam [3:0] cmd_activate  = 4'b0011;
    localparam [3:0] cmd_write     = 4'b0100;
    localparam [3:0] cmd_read      = 4'b0101;
    localparam [3:0] cmd_setmode   = 4'b0000;
    localparam [3:0] cmd_refresh   = 4'b0001;

    reg [3:0] state_q;
    reg [3:0] state_d;
    reg [3:0] nxt_q;
    reg [3:0] nxt_d;
    reg [3:0] cmd_q;
    reg [3:0] cmd_d;

    reg [15:0] delay_ctr_q;
    reg [15:0] delay_ctr_d;

    reg [9:0] refresh_ctr_q;
    reg [9:0] refresh_ctr_d;
    reg       refresh_flag_q;
    reg       refresh_flag_d;

    reg [9:0] burst_index_q;
    reg [9:0] burst_index_d;

    reg rw_q;
    reg rw_d;
    reg rw_en_q;
    reg rw_en_d;

    reg [11:0] s_addr_q;
    reg [11:0] s_addr_d;
    reg [1:0]  s_ba_q;
    reg [1:0]  s_ba_d;

    reg [13:0] f_addr_q;
    reg [13:0] f_addr_d;

    reg [15:0] f2s_data_q;
    reg [15:0] f2s_data_d;

    reg [15:0] s2f_data_q;
    reg [15:0] s2f_data_d;
    reg        s2f_data_valid_q;
    reg        s2f_data_valid_d;

    reg tri_q;
    reg tri_d;

    // ---------------------------------------------------------
    // Register update
    // ---------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q          <= start;
            nxt_q            <= start;
            cmd_q            <= cmd_NOP;

            delay_ctr_q      <= 16'd0;
            refresh_ctr_q    <= 10'd0;
            refresh_flag_q   <= 1'b0;
            burst_index_q    <= 10'd0;

            rw_q             <= 1'b0;
            rw_en_q          <= 1'b0;

            s_addr_q         <= 12'd0;
            s_ba_q           <= 2'd0;
            f_addr_q         <= 14'd0;

            f2s_data_q       <= 16'd0;
            s2f_data_q       <= 16'd0;
            s2f_data_valid_q <= 1'b0;

            tri_q            <= 1'b0;
        end
        else begin
            state_q          <= state_d;
            nxt_q            <= nxt_d;
            cmd_q            <= cmd_d;

            delay_ctr_q      <= delay_ctr_d;
            refresh_ctr_q    <= refresh_ctr_d;
            refresh_flag_q   <= refresh_flag_d;
            burst_index_q    <= burst_index_d;

            rw_q             <= rw_d;
            rw_en_q          <= rw_en_d;

            s_addr_q         <= s_addr_d;
            s_ba_q           <= s_ba_d;
            f_addr_q         <= f_addr_d;

            f2s_data_q       <= f2s_data_d;
            s2f_data_q       <= s2f_data_d;
            s2f_data_valid_q <= s2f_data_valid_d;

            tri_q            <= tri_d;
        end
    end

    // ---------------------------------------------------------
    // Next state logic
    // ---------------------------------------------------------
    always @(*) begin
        state_d          = state_q;
        nxt_d            = nxt_q;
        cmd_d            = cmd_NOP;
        delay_ctr_d      = delay_ctr_q;

        ready            = 1'b0;

        s_addr_d         = s_addr_q;
        s_ba_d           = s_ba_q;
        f_addr_d         = f_addr_q;

        rw_d             = rw_q;
        rw_en_d          = rw_en_q;

        f2s_data_d       = f2s_data_q;
        s2f_data_d       = s2f_data_q;
        s2f_data_valid_d = 1'b0;

        f2s_data_valid   = 1'b0;

        tri_d            = 1'b0;
        burst_index_d    = burst_index_q;

        // Refresh every about 7.8 us at 50 MHz
        refresh_flag_d = refresh_flag_q;
        refresh_ctr_d  = refresh_ctr_q + 10'd1;

        if (refresh_ctr_q == 10'd390) begin
            refresh_ctr_d  = 10'd0;
            refresh_flag_d = 1'b1;
        end

        case (state_q)

            // -------------------------------------------------
            // Generic delay state
            // -------------------------------------------------
            delay: begin
                delay_ctr_d = delay_ctr_q - 16'd1;

                // For write, request first FIFO word one cycle before
                // entering write state. Because FIFO RAM is synchronous,
                // f2s_data becomes valid in the next state.
                if (nxt_q == write) begin
                    tri_d          = 1'b1;
                    f2s_data_valid = 1'b1;
                end

                if (delay_ctr_d == 16'd0)
                    state_d = nxt_q;
            end

            // -------------------------------------------------
            // Initialization
            // -------------------------------------------------
            start: begin
                state_d     = delay;
                nxt_d       = precharge_init;
                delay_ctr_d = 16'd10000; // 200 us at 50 MHz
                s_addr_d    = 12'd0;
                s_ba_d      = 2'd0;
            end

            precharge_init: begin
                state_d      = delay;
                nxt_d        = refresh_1;
                delay_ctr_d  = t_RP - 1'b1;
                cmd_d        = cmd_precharge;
                s_addr_d[10] = 1'b1; // precharge all banks
            end

            refresh_1: begin
                state_d     = delay;
                nxt_d       = refresh_2;
                delay_ctr_d = t_RC - 1'b1;
                cmd_d       = cmd_refresh;
            end

            refresh_2: begin
                state_d     = delay;
                nxt_d       = load_mode_reg;
                delay_ctr_d = t_RC - 1'b1;
                cmd_d       = cmd_refresh;
            end

            load_mode_reg: begin
                state_d     = delay;
                nxt_d       = idle;
                delay_ctr_d = t_MRD - 1'b1;
                cmd_d       = cmd_setmode;

                // CL=3, sequential, full-page burst
                s_addr_d = 12'b0000_0011_0111;
                s_ba_d   = 2'b00;
            end

            // -------------------------------------------------
            // Idle / accept command
            // -------------------------------------------------
            idle: begin
                ready = rw_en_q ? 1'b0 : 1'b1;

                if (rw_en_q) begin
                    state_d       = delay;
                    nxt_d         = rw_q ? read : write;
                    delay_ctr_d   = t_RCD - 1'b1;
                    cmd_d         = cmd_activate;

                    burst_index_d = 10'd0;
                    rw_en_d       = 1'b0;

                    // f_addr = {row[11:0], bank[1:0]}
                    s_addr_d = f_addr_q[13:2];
                    s_ba_d   = f_addr_q[1:0];
                end
                else if (refresh_flag_q || rw_en) begin
                    state_d      = delay;
                    nxt_d        = refresh;
                    delay_ctr_d  = t_RP - 1'b1;
                    cmd_d        = cmd_precharge;
                    s_addr_d[10] = 1'b1; // precharge all banks

                    refresh_flag_d = 1'b0;

                    if (rw_en) begin
                        rw_en_d  = 1'b1;
                        f_addr_d = f_addr;
                        rw_d     = rw;
                    end
                end
            end

            refresh: begin
                state_d     = delay;
                nxt_d       = idle;
                delay_ctr_d = t_RC - 1'b1;
                cmd_d       = cmd_refresh;
            end

            // -------------------------------------------------
            // Read burst
            // -------------------------------------------------
            read: begin
                state_d      = delay;
                nxt_d        = read_data;
                delay_ctr_d  = t_CL;
                cmd_d        = cmd_read;

                s_addr_d     = 12'd0;       // column 0
                s_addr_d[10] = 1'b0;        // no auto-precharge
                s_ba_d       = f_addr_q[1:0];
            end

            read_data: begin
                s2f_data_d       = s_dq;
                s2f_data_valid_d = 1'b1;

                if (burst_index_q == BURST_LAST) begin
                    burst_index_d      = 10'd0;

                    // Keep valid HIGH on last word.
                    s2f_data_valid_d   = 1'b1;

                    state_d            = delay;
                    nxt_d              = idle;
                    delay_ctr_d        = t_RP - 1'b1;
                    cmd_d              = cmd_precharge;
                end
                else begin
                    burst_index_d = burst_index_q + 10'd1;
                end
            end

            // -------------------------------------------------
            // Write burst
            //
            // Correct pipeline:
            // - delay state requests word0.
            // - write state captures word0 and issues WRITE.
            // - write_burst captures word1..word255.
            // - write_last holds word255 for one SDRAM data cycle.
            //
            // This avoids shifting/dropping one word per 256-word burst.
            // -------------------------------------------------
            write: begin
                tri_d = 1'b1;

                cmd_d        = cmd_write;
                s_addr_d     = 12'd0;       // column 0
                s_addr_d[10] = 1'b0;        // no auto-precharge
                s_ba_d       = f_addr_q[1:0];

                // Capture word0 from FIFO output.
                f2s_data_d = f2s_data;

                // Request word1 for next cycle.
                f2s_data_valid = 1'b1;

                burst_index_d  = 10'd1;
                state_d        = write_burst;
            end

            write_burst: begin
                tri_d = 1'b1;

                if (burst_index_q == BURST_LAST) begin
                    // f2s_data now corresponds to word255.
                    // Capture it, then hold it for one more data cycle
                    // in write_last before precharge.
                    f2s_data_d     = f2s_data;
                    f2s_data_valid = 1'b0;

                    burst_index_d  = 10'd0;
                    state_d        = write_last;
                end
                else begin
                    // Capture next word and request the following word.
                    f2s_data_d     = f2s_data;
                    f2s_data_valid = 1'b1;

                    burst_index_d  = burst_index_q + 10'd1;
                end
            end

            write_last: begin
                // Hold word255 on bus for its SDRAM write cycle.
                tri_d = 1'b1;

                f2s_data_valid = 1'b0;
                burst_index_d  = 10'd0;

                state_d        = delay;
                nxt_d          = idle;
                delay_ctr_d    = t_WR + t_RP - 1'b1;
                cmd_d          = cmd_precharge;
            end

            default: begin
                state_d = start;
            end
        endcase
    end

    // ---------------------------------------------------------
    // Outputs
    // ---------------------------------------------------------
    assign {s_cs_n, s_ras_n, s_cas_n, s_we_n} = cmd_q;

    assign s_cke  = 1'b1;
    assign LDQM   = 1'b0;
    assign HDQM   = 1'b0;

    assign s_addr = s_addr_q;
    assign s_ba   = s_ba_q;

    assign s_dq = tri_q ? f2s_data_q : 16'hzzzz;

    assign s2f_data       = s2f_data_q;
    assign s2f_data_valid = s2f_data_valid_q;

endmodule