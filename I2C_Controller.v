`timescale 1ns / 1ps
// ============================================================
// OV7670 SCCB / I2C Controller - push-pull version
//
// Compatible with ov7670_controller.v:
//
// i2c_data = {8'h42, reg_addr, reg_data}
//
// One transaction:
// START
// 0x42      ACK
// reg_addr  ACK
// reg_data  ACK
// STOP
//
// Important fixes:
// - i2c_start can be only 1 clk pulse.
// - This module latches i2c_start and runs until STOP.
// - It does NOT reset when i2c_start goes low.
// - SIOC is driven 0/1 directly.
// - SIOD is driven 0/1 during data bits.
// - SIOD is released only during ACK.
// - ACK is ignored, but ACK timing slot is valid.
//
// clk = 50 MHz
// SIOC ~= 100 kHz
// ============================================================

module I2C_Controller(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [23:0] i2c_data,
    input  wire        i2c_start,
    output wire        i2c_sclk,
    inout  wire        i2c_sdat,
    output reg         i2c_done
);

    // 50 MHz / 125 / 4 = 100 kHz
    localparam [7:0] CLK_DIV = 8'd125;

    reg [7:0] div_cnt;
    wire tick;

    assign tick = (div_cnt == (CLK_DIV - 8'd1));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_cnt <= 8'd0;
        end
        else begin
            if (tick)
                div_cnt <= 8'd0;
            else
                div_cnt <= div_cnt + 8'd1;
        end
    end

    // --------------------------------------------------------
    // Push-pull SCCB output
    // --------------------------------------------------------
    reg scl_reg;
    reg sda_reg;
    reg sda_oe;

    assign i2c_sclk = scl_reg;

    // sda_oe = 1: drive SDA
    // sda_oe = 0: release SDA during ACK
    assign i2c_sdat = sda_oe ? sda_reg : 1'bz;

    // --------------------------------------------------------
    // FSM states
    // --------------------------------------------------------
    localparam S_IDLE       = 5'd0;

    localparam S_START_A    = 5'd1;
    localparam S_START_B    = 5'd2;
    localparam S_START_C    = 5'd3;

    localparam S_BIT_LOW    = 5'd4;
    localparam S_BIT_HIGH_A = 5'd5;
    localparam S_BIT_HIGH_B = 5'd6;
    localparam S_BIT_FALL   = 5'd7;

    localparam S_ACK_LOW    = 5'd8;
    localparam S_ACK_HIGH_A = 5'd9;
    localparam S_ACK_HIGH_B = 5'd10;
    localparam S_ACK_FALL   = 5'd11;

    localparam S_STOP_LOW   = 5'd12;
    localparam S_STOP_HIGH  = 5'd13;
    localparam S_STOP_REL   = 5'd14;

    localparam S_DONE       = 5'd15;

    reg [4:0]  state;
    reg [23:0] shift_reg;
    reg [5:0]  bit_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            shift_reg <= 24'd0;
            bit_cnt   <= 6'd0;

            scl_reg   <= 1'b1;
            sda_reg   <= 1'b1;
            sda_oe    <= 1'b1;

            i2c_done  <= 1'b0;
        end
        else begin
            i2c_done <= 1'b0;

            // Latch one-clock i2c_start pulse immediately.
            // After this point, transaction continues even if i2c_start goes low.
            if ((state == S_IDLE) && i2c_start) begin
                shift_reg <= i2c_data;
                bit_cnt   <= 6'd0;

                scl_reg   <= 1'b1;
                sda_reg   <= 1'b1;
                sda_oe    <= 1'b1;

                state     <= S_START_A;
            end
            else if (tick) begin
                case (state)

                    // ------------------------------------------------
                    // IDLE: bus high
                    // ------------------------------------------------
                    S_IDLE: begin
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b1;
                        sda_oe  <= 1'b1;
                    end

                    // ------------------------------------------------
                    // START condition:
                    // SDA 1 -> 0 while SCL is high
                    // ------------------------------------------------
                    S_START_A: begin
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b1;
                        sda_oe  <= 1'b1;
                        state   <= S_START_B;
                    end

                    S_START_B: begin
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b0;
                        sda_oe  <= 1'b1;
                        state   <= S_START_C;
                    end

                    S_START_C: begin
                        scl_reg <= 1'b0;
                        sda_reg <= 1'b0;
                        sda_oe  <= 1'b1;
                        state   <= S_BIT_LOW;
                    end

                    // ------------------------------------------------
                    // Send data bit, MSB first.
                    // Data is changed while SCL low and held while SCL high.
                    // ------------------------------------------------
                    S_BIT_LOW: begin
                        scl_reg <= 1'b0;
                        sda_reg <= shift_reg[23];
                        sda_oe  <= 1'b1;
                        state   <= S_BIT_HIGH_A;
                    end

                    S_BIT_HIGH_A: begin
                        scl_reg <= 1'b1;
                        sda_reg <= shift_reg[23];
                        sda_oe  <= 1'b1;
                        state   <= S_BIT_HIGH_B;
                    end

                    S_BIT_HIGH_B: begin
                        scl_reg <= 1'b1;
                        sda_reg <= shift_reg[23];
                        sda_oe  <= 1'b1;
                        state   <= S_BIT_FALL;
                    end

                    S_BIT_FALL: begin
                        scl_reg   <= 1'b0;
                        shift_reg <= {shift_reg[22:0], 1'b0};
                        bit_cnt   <= bit_cnt + 6'd1;

                        // ACK after bit 7, bit 15, bit 23
                        if (bit_cnt[2:0] == 3'd7)
                            state <= S_ACK_LOW;
                        else
                            state <= S_BIT_LOW;
                    end

                    // ------------------------------------------------
                    // ACK slot:
                    // Release SDA for one SCL pulse.
                    // ACK value is ignored.
                    // ------------------------------------------------
                    S_ACK_LOW: begin
                        scl_reg <= 1'b0;
                        sda_reg <= 1'b1;
                        sda_oe  <= 1'b0;
                        state   <= S_ACK_HIGH_A;
                    end

                    S_ACK_HIGH_A: begin
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b1;
                        sda_oe  <= 1'b0;
                        state   <= S_ACK_HIGH_B;
                    end

                    S_ACK_HIGH_B: begin
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b1;
                        sda_oe  <= 1'b0;
                        state   <= S_ACK_FALL;
                    end

                    S_ACK_FALL: begin
                        scl_reg <= 1'b0;
                        sda_reg <= 1'b0;
                        sda_oe  <= 1'b1;

                        if (bit_cnt == 6'd24)
                            state <= S_STOP_LOW;
                        else
                            state <= S_BIT_LOW;
                    end

                    // ------------------------------------------------
                    // STOP condition:
                    // SDA 0 -> 1 while SCL is high
                    // ------------------------------------------------
                    S_STOP_LOW: begin
                        scl_reg <= 1'b0;
                        sda_reg <= 1'b0;
                        sda_oe  <= 1'b1;
                        state   <= S_STOP_HIGH;
                    end

                    S_STOP_HIGH: begin
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b0;
                        sda_oe  <= 1'b1;
                        state   <= S_STOP_REL;
                    end

                    S_STOP_REL: begin
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b1;
                        sda_oe  <= 1'b1;
                        state   <= S_DONE;
                    end

                    // ------------------------------------------------
                    // DONE pulse
                    // ------------------------------------------------
                    S_DONE: begin
                        i2c_done <= 1'b1;
                        state    <= S_IDLE;
                    end

                    default: begin
                        state   <= S_IDLE;
                        scl_reg <= 1'b1;
                        sda_reg <= 1'b1;
                        sda_oe  <= 1'b1;
                    end
                endcase
            end
        end
    end

endmodule