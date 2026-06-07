`timescale 1ns / 1ps

module asyn_fifo
    #(
        parameter DATA_WIDTH = 16,
        parameter FIFO_DEPTH_WIDTH = 10
    )
    (
        input  wire rst_n,
        input  wire clk_write,
        input  wire clk_read,
        input  wire write,
        input  wire read,
        input  wire [DATA_WIDTH-1:0] data_write,
        output wire [DATA_WIDTH-1:0] data_read,
        output reg  full,
        output reg  empty,
        output reg  [FIFO_DEPTH_WIDTH-1:0] data_count_w,
        output reg  [FIFO_DEPTH_WIDTH-1:0] data_count_r
    );

    localparam FIFO_DEPTH = (1 << FIFO_DEPTH_WIDTH);

    reg [FIFO_DEPTH_WIDTH:0] w_ptr_bin;
    reg [FIFO_DEPTH_WIDTH:0] r_ptr_bin;

    wire [FIFO_DEPTH_WIDTH:0] w_ptr_bin_next;
    wire [FIFO_DEPTH_WIDTH:0] r_ptr_bin_next;

    wire [FIFO_DEPTH_WIDTH:0] w_ptr_gray;
    wire [FIFO_DEPTH_WIDTH:0] r_ptr_gray;
    wire [FIFO_DEPTH_WIDTH:0] w_ptr_gray_next;
    wire [FIFO_DEPTH_WIDTH:0] r_ptr_gray_next;

    reg [FIFO_DEPTH_WIDTH:0] r_gray_sync1;
    reg [FIFO_DEPTH_WIDTH:0] r_gray_sync2;
    reg [FIFO_DEPTH_WIDTH:0] w_gray_sync1;
    reg [FIFO_DEPTH_WIDTH:0] w_gray_sync2;

    wire [FIFO_DEPTH_WIDTH:0] r_bin_sync_w;
    wire [FIFO_DEPTH_WIDTH:0] w_bin_sync_r;

    wire do_write;
    wire do_read;

    assign do_write = write && !full;
    assign do_read  = read  && !empty;

    assign w_ptr_bin_next = w_ptr_bin + (do_write ? 1'b1 : 1'b0);
    assign r_ptr_bin_next = r_ptr_bin + (do_read  ? 1'b1 : 1'b0);

    assign w_ptr_gray      = w_ptr_bin ^ (w_ptr_bin >> 1);
    assign r_ptr_gray      = r_ptr_bin ^ (r_ptr_bin >> 1);
    assign w_ptr_gray_next = w_ptr_bin_next ^ (w_ptr_bin_next >> 1);
    assign r_ptr_gray_next = r_ptr_bin_next ^ (r_ptr_bin_next >> 1);

    function [FIFO_DEPTH_WIDTH:0] gray_to_bin;
        input [FIFO_DEPTH_WIDTH:0] g;
        integer i;
        begin
            gray_to_bin[FIFO_DEPTH_WIDTH] = g[FIFO_DEPTH_WIDTH];
            for (i = FIFO_DEPTH_WIDTH - 1; i >= 0; i = i - 1)
                gray_to_bin[i] = gray_to_bin[i+1] ^ g[i];
        end
    endfunction

    assign r_bin_sync_w = gray_to_bin(r_gray_sync2);
    assign w_bin_sync_r = gray_to_bin(w_gray_sync2);

    wire full_next_w;
    assign full_next_w =
        (w_ptr_gray_next == {~r_gray_sync2[FIFO_DEPTH_WIDTH:FIFO_DEPTH_WIDTH-1],
                              r_gray_sync2[FIFO_DEPTH_WIDTH-2:0]});

    wire empty_next_r;
    assign empty_next_r = (r_ptr_gray_next == w_gray_sync2);

    // Count full detection:
    // If lower pointer bits are equal but MSB differs, actual count is FIFO_DEPTH.
    // data_count is only FIFO_DEPTH_WIDTH bits, so saturate to max value.
    wire fifo_full_count_w;
    wire fifo_full_count_r;

    assign fifo_full_count_w =
        (w_ptr_bin_next[FIFO_DEPTH_WIDTH-1:0] == r_bin_sync_w[FIFO_DEPTH_WIDTH-1:0]) &&
        (w_ptr_bin_next[FIFO_DEPTH_WIDTH]     != r_bin_sync_w[FIFO_DEPTH_WIDTH]);

    assign fifo_full_count_r =
        (w_bin_sync_r[FIFO_DEPTH_WIDTH-1:0] == r_ptr_bin_next[FIFO_DEPTH_WIDTH-1:0]) &&
        (w_bin_sync_r[FIFO_DEPTH_WIDTH]     != r_ptr_bin_next[FIFO_DEPTH_WIDTH]);

    wire [FIFO_DEPTH_WIDTH-1:0] count_w_calc;
    wire [FIFO_DEPTH_WIDTH-1:0] count_r_calc;

    assign count_w_calc =
        fifo_full_count_w ? {FIFO_DEPTH_WIDTH{1'b1}} :
        (w_ptr_bin_next[FIFO_DEPTH_WIDTH-1:0] - r_bin_sync_w[FIFO_DEPTH_WIDTH-1:0]);

    assign count_r_calc =
        fifo_full_count_r ? {FIFO_DEPTH_WIDTH{1'b1}} :
        (w_bin_sync_r[FIFO_DEPTH_WIDTH-1:0] - r_ptr_bin_next[FIFO_DEPTH_WIDTH-1:0]);

    // ---------------------------------------------------------
    // Write clock domain
    // ---------------------------------------------------------
    always @(posedge clk_write or negedge rst_n) begin
        if (!rst_n) begin
            w_ptr_bin    <= {FIFO_DEPTH_WIDTH+1{1'b0}};
            full         <= 1'b0;
            r_gray_sync1 <= {FIFO_DEPTH_WIDTH+1{1'b0}};
            r_gray_sync2 <= {FIFO_DEPTH_WIDTH+1{1'b0}};
            data_count_w <= {FIFO_DEPTH_WIDTH{1'b0}};
        end
        else begin
            r_gray_sync1 <= r_ptr_gray;
            r_gray_sync2 <= r_gray_sync1;

            w_ptr_bin <= w_ptr_bin_next;
            full      <= full_next_w;

            data_count_w <= count_w_calc;
        end
    end

    // ---------------------------------------------------------
    // Read clock domain
    // ---------------------------------------------------------
    always @(posedge clk_read or negedge rst_n) begin
        if (!rst_n) begin
            r_ptr_bin    <= {FIFO_DEPTH_WIDTH+1{1'b0}};
            empty        <= 1'b1;
            w_gray_sync1 <= {FIFO_DEPTH_WIDTH+1{1'b0}};
            w_gray_sync2 <= {FIFO_DEPTH_WIDTH+1{1'b0}};
            data_count_r <= {FIFO_DEPTH_WIDTH{1'b0}};
        end
        else begin
            w_gray_sync1 <= w_ptr_gray;
            w_gray_sync2 <= w_gray_sync1;

            r_ptr_bin <= r_ptr_bin_next;
            empty     <= empty_next_r;

            data_count_r <= count_r_calc;
        end
    end

    dual_port_sync #(
        .ADDR_WIDTH(FIFO_DEPTH_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) u_ram (
        .clk_r  (clk_read),
        .clk_w  (clk_write),
        .we     (do_write),
        .din    (data_write),
        .addr_a (w_ptr_bin[FIFO_DEPTH_WIDTH-1:0]),
        .addr_b (r_ptr_bin[FIFO_DEPTH_WIDTH-1:0]),
        .dout   (data_read)
    );

endmodule


module dual_port_sync
    #(
        parameter ADDR_WIDTH = 10,
        parameter DATA_WIDTH = 16
    )
    (
        input  wire clk_r,
        input  wire clk_w,
        input  wire we,
        input  wire [DATA_WIDTH-1:0] din,
        input  wire [ADDR_WIDTH-1:0] addr_a,
        input  wire [ADDR_WIDTH-1:0] addr_b,
        output reg  [DATA_WIDTH-1:0] dout
    );

    reg [DATA_WIDTH-1:0] ram [0:(1<<ADDR_WIDTH)-1];

    always @(posedge clk_w) begin
        if (we)
            ram[addr_a] <= din;
    end

    always @(posedge clk_r) begin
        dout <= ram[addr_b];
    end

endmodule