`timescale 1ns / 1ps

module i2c_top_de2
    #(
        parameter freq = 100_000,
        parameter CLK_HZ = 50_000_000
    )
    (
        input  wire       clk,
        input  wire       rst_n,
        input  wire       start,
        input  wire       stop,
        input  wire [7:0] wr_data,
        output reg        rd_tick,
        output reg  [1:0] ack,
        output reg  [7:0] rd_data,
        inout  wire       scl,
        inout  wire       sda,
        output wire [3:0] state
    );

    localparam integer FULL = CLK_HZ / (2 * freq);
    localparam integer HALF = FULL / 2;
    localparam integer CW   = 16;

    localparam [3:0] idle        = 4'd0;
    localparam [3:0] starting    = 4'd1;
    localparam [3:0] packet      = 4'd2;
    localparam [3:0] ack_servant = 4'd3;
    localparam [3:0] renew_data  = 4'd4;
    localparam [3:0] stop_1      = 4'd5;
    localparam [3:0] stop_2      = 4'd6;

    reg [3:0] state_q, state_d;
    reg [8:0] wr_data_q, wr_data_d;
    reg [3:0] idx_q, idx_d;
    reg       scl_q, scl_d;
    reg       sda_q, sda_d;
    reg [CW-1:0] counter_q, counter_d;

    assign state = state_q;

    wire scl_hi = (scl_q == 1'b1) && (counter_q == HALF[CW-1:0]);
    wire scl_lo = (scl_q == 1'b0) && (counter_q == HALF[CW-1:0]);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q   <= idle;
            wr_data_q <= 9'd0;
            idx_q     <= 4'd0;
            scl_q     <= 1'b1;
            sda_q     <= 1'b1;
            counter_q <= {CW{1'b0}};
        end
        else begin
            state_q   <= state_d;
            wr_data_q <= wr_data_d;
            idx_q     <= idx_d;
            scl_q     <= scl_d;
            sda_q     <= sda_d;
            counter_q <= counter_d;
        end
    end

    always @(*) begin
        counter_d = counter_q + 1'b1;
        scl_d = scl_q;
        if (state_q == idle || state_q == starting || state_q == stop_1 || state_q == stop_2) begin
            scl_d = 1'b1;
        end
        else if (counter_q == FULL[CW-1:0]) begin
            counter_d = {CW{1'b0}};
            scl_d = ~scl_q;
        end
    end

    always @(*) begin
        state_d   = state_q;
        wr_data_d = wr_data_q;
        idx_d     = idx_q;
        sda_d     = sda_q;
        rd_tick   = 1'b0;
        rd_data   = 8'd0;
        ack       = 2'b00;

        case (state_q)
            idle: begin
                sda_d = 1'b1;
                if (start) begin
                    wr_data_d = {wr_data, 1'b1};
                    idx_d = 4'd8;
                    state_d = starting;
                end
            end

            starting: begin
                sda_d = 1'b1;
                if (scl_hi) begin
                    sda_d = 1'b0;
                    state_d = packet;
                end
            end

            packet: begin
                if (scl_lo) begin
                    sda_d = wr_data_q[idx_q];
                    if (idx_q == 4'd0)
                        state_d = ack_servant;
                    else
                        idx_d = idx_q - 1'b1;
                end
            end

            ack_servant: begin
                if (scl_hi) begin
                    ack[1] = 1'b1;
                    ack[0] = !sda;
                    if (stop)
                        state_d = stop_1;
                    else
                        state_d = renew_data;
                end
            end

            renew_data: begin
                wr_data_d = {wr_data, 1'b1};
                idx_d = 4'd8;
                state_d = packet;
            end

            stop_1: begin
                if (scl_lo) begin
                    sda_d = 1'b0;
                    state_d = stop_2;
                end
            end

            stop_2: begin
                if (scl_hi) begin
                    sda_d = 1'b1;
                    state_d = idle;
                end
            end

            default: begin
                state_d = idle;
                sda_d = 1'b1;
            end
        endcase
    end

    // Push-pull style for OV7670 SCCB. SDA is released during ACK.
    assign scl = scl_q ? 1'b1 : 1'b0;
    assign sda = (state_q == ack_servant) ? 1'bz : (sda_q ? 1'b1 : 1'b0);
endmodule
