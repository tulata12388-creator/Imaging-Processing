`timescale 1ns / 1ps
// =============================================================
// ov7670_controller.v
//
// Robust OV7670 SCCB/I2C register configuration controller.
//
// Port matches camera_interface_de2.v:
//
// ov7670_controller u_ov7670_config (
//     .clk             (clk_cfg),
//     .rst_n           (rst_n),
//     .resend          (~rst_n),
//     .config_finished (config_finished),
//     .sioc            (cmos_scl),
//     .siod            (cmos_sda),
//     .reset           (cmos_rst_n),
//     .pwdn            (cmos_pwdn)
// );
//
// Uses existing I2C_Controller:
// I2C_Controller(
//     .clk,
//     .i2c_data,
//     .i2c_start,
//     .i2c_sclk,
//     .i2c_sdat,
//     .i2c_done
// );
// =============================================================

module ov7670_controller(
    input  wire clk,
    input  wire rst_n,
    input  wire resend,

    output wire config_finished,
    output wire sioc,
    inout  wire siod,
    output wire reset,
    output wire pwdn
);

    // OV7670 control pins
    assign reset = 1'b1;  // active low reset: 1 = normal run
    assign pwdn  = 1'b0;  // active high power down: 0 = camera on

    // Last register index in ov7670_registers.v
    // Make sure ov7670_registers has entries from 8'h00 to 8'h4A.
    localparam [7:0] REG_LAST = 8'h4A;

    // State machine
    localparam [2:0] S_POWER_DELAY = 3'd0;
    localparam [2:0] S_SEND        = 3'd1;
    localparam [2:0] S_WAIT_DONE   = 3'd2;
    localparam [2:0] S_RESET_DELAY = 3'd3;
    localparam [2:0] S_NEXT        = 3'd4;
    localparam [2:0] S_FINISH      = 3'd5;

    reg [2:0] state_q;

    reg [7:0]  cmd_addr;
    reg [25:0] power_cnt;
    reg [23:0] reset_cnt;

    reg i2c_start_q;

    wire [15:0] command;
    wire [23:0] i2c_data;
    wire        i2c_done;

    assign config_finished = (state_q == S_FINISH);

    ov7670_registers u_regs (
        .advance (cmd_addr),
        .command (command)
    );

    // OV7670 SCCB write address = 0x42
    assign i2c_data = {8'h42, command};

    I2C_Controller u_i2c (
        .clk       (clk),
        .rst_n     (rst_n),
        .i2c_data  (i2c_data),
        .i2c_start (i2c_start_q),
        .i2c_sclk  (sioc),
        .i2c_sdat  (siod),
        .i2c_done  (i2c_done)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q     <= S_POWER_DELAY;
            cmd_addr    <= 8'd0;
            power_cnt   <= 26'd0;
            reset_cnt   <= 24'd0;
            i2c_start_q <= 1'b0;
        end
        else if (resend) begin
            state_q     <= S_POWER_DELAY;
            cmd_addr    <= 8'd0;
            power_cnt   <= 26'd0;
            reset_cnt   <= 24'd0;
            i2c_start_q <= 1'b0;
        end
        else begin
            case (state_q)

                // -------------------------------------------------
                // Wait after power-up before SCCB transaction.
                // About 0.67 s at 50 MHz.
                // -------------------------------------------------
                S_POWER_DELAY: begin
                    i2c_start_q <= 1'b0;
                    reset_cnt   <= 24'd0;

                    if (power_cnt == 26'h3FFFFFF) begin
                        power_cnt <= power_cnt;
                        cmd_addr  <= 8'd0;
                        state_q   <= S_SEND;
                    end
                    else begin
                        power_cnt <= power_cnt + 26'd1;
                    end
                end

                // -------------------------------------------------
                // Send one register command.
                // command = 16'hFFFF is a delay marker, not sent.
                // -------------------------------------------------
                S_SEND: begin
                    i2c_start_q <= 1'b0;

                    if (cmd_addr > REG_LAST) begin
                        state_q <= S_FINISH;
                    end
                    else if (command == 16'hFFFF) begin
                        reset_cnt <= 24'd0;
                        state_q   <= S_RESET_DELAY;
                    end
                    else begin
                        i2c_start_q <= 1'b1;
                        state_q     <= S_WAIT_DONE;
                    end
                end

                // -------------------------------------------------
                // Pulse i2c_start for one clock, then wait done.
                // -------------------------------------------------
                S_WAIT_DONE: begin
                    i2c_start_q <= 1'b0;

                    if (i2c_done) begin
                        state_q <= S_NEXT;
                    end
                end

                // -------------------------------------------------
                // Delay marker after COM7 reset.
                // About 168 ms at 50 MHz.
                // -------------------------------------------------
                S_RESET_DELAY: begin
                    i2c_start_q <= 1'b0;

                    if (reset_cnt == 24'hFFFFFF) begin
                        reset_cnt <= reset_cnt;
                        state_q   <= S_NEXT;
                    end
                    else begin
                        reset_cnt <= reset_cnt + 24'd1;
                    end
                end

                // -------------------------------------------------
                // Move to next register.
                // -------------------------------------------------
                S_NEXT: begin
                    i2c_start_q <= 1'b0;

                    if (cmd_addr >= REG_LAST) begin
                        state_q <= S_FINISH;
                    end
                    else begin
                        cmd_addr <= cmd_addr + 8'd1;
                        state_q  <= S_SEND;
                    end
                end

                // -------------------------------------------------
                // Done.
                // -------------------------------------------------
                S_FINISH: begin
                    i2c_start_q <= 1'b0;
                    state_q     <= S_FINISH;
                end

                default: begin
                    state_q     <= S_POWER_DELAY;
                    cmd_addr    <= 8'd0;
                    power_cnt   <= 26'd0;
                    reset_cnt   <= 24'd0;
                    i2c_start_q <= 1'b0;
                end
            endcase
        end
    end

endmodule