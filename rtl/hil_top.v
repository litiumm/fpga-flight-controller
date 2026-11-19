`timescale 1ns/1ps
// hil_top.v - Tang Nano 9K top level for hardware-in-the-loop testing
//
// The PC runs the plant model and sends sensor frames over the on-board
// USB-UART; the FPGA runs pid_core + mixer, replies with the command, and
// drives two real ESC PWM outputs.
//
// Protocol (115200 8N1, little endian, chk = XOR of the 6 payload bytes):
//   PC -> FPGA  A5 | sp(2) angle(2) rate(2) | chk   sensor frame -> one PID update
//   PC -> FPGA  C3 | kp(2) ki(2) kd(2)      | chk   set gains, clear integrator
//   FPGA -> PC  5A | u(2)                   | chk   reply to A5 (chk = XOR of u bytes)
//   FPGA -> PC  3C                                  ack for C3
// A gap of more than ~1 ms in the middle of a frame resets the parser.
//
// LEDs (active low): 0 heartbeat, 1 frame activity, 2 checksum error (sticky),
//                    3 output saturated, 4/5 differential direction.

module hil_top #(
    parameter integer CLK_HZ = 27_000_000,
    parameter integer BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,      // S2 button
    input  wire       uart_rx,
    output wire       uart_tx,
    output wire       pwm1,
    output wire       pwm2,
    output wire [5:0] led
);
    localparam integer CPB = CLK_HZ / BAUD;
    localparam integer OUT_MAX = 400;

    // ---------------------------------------------------------- reset
    reg [7:0] por = 8'd0;
    reg rst = 1'b1;
    always @(posedge clk) begin
        if (!por[7]) por <= por + 1'b1;
        rst <= !por[7] || !rst_n;
    end

    // ---------------------------------------------------------- UART
    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data;
    reg        tx_send;
    wire       tx_busy;

    uart_rx #(.CLKS_PER_BIT(CPB)) u_rx (.clk(clk), .rst(rst), .rx(uart_rx),
                                        .data(rx_data), .valid(rx_valid));
    uart_tx #(.CLKS_PER_BIT(CPB)) u_tx (.clk(clk), .rst(rst), .data(tx_data),
                                        .send(tx_send), .tx(uart_tx), .busy(tx_busy));

    // ---------------------------------------------------------- frame parser
    reg [7:0]  hdr;
    reg [7:0]  buf0, buf1, buf2, buf3, buf4, buf5;
    reg [2:0]  idx;            // 0 = waiting for header, 1..6 payload, 7 checksum
    reg [7:0]  xsum;
    reg [15:0] gap;
    reg        frame_ok_sensor, frame_ok_gains, chk_err;

    always @(posedge clk) begin
        frame_ok_sensor <= 1'b0;
        frame_ok_gains  <= 1'b0;
        if (rst) begin
            idx <= 0; gap <= 0; chk_err <= 1'b0;
        end else begin
            if (idx != 0 && !rx_valid) begin
                gap <= gap + 1'b1;
                if (gap == CLK_HZ / 1000) idx <= 0;   // resync after 1 ms gap
            end
            if (rx_valid) begin
                gap <= 0;
                case (idx)
                    3'd0: if (rx_data == 8'hA5 || rx_data == 8'hC3) begin
                        hdr <= rx_data; xsum <= 8'd0; idx <= 3'd1;
                    end
                    3'd1: begin buf0 <= rx_data; xsum <= xsum ^ rx_data; idx <= 3'd2; end
                    3'd2: begin buf1 <= rx_data; xsum <= xsum ^ rx_data; idx <= 3'd3; end
                    3'd3: begin buf2 <= rx_data; xsum <= xsum ^ rx_data; idx <= 3'd4; end
                    3'd4: begin buf3 <= rx_data; xsum <= xsum ^ rx_data; idx <= 3'd5; end
                    3'd5: begin buf4 <= rx_data; xsum <= xsum ^ rx_data; idx <= 3'd6; end
                    3'd6: begin buf5 <= rx_data; xsum <= xsum ^ rx_data; idx <= 3'd7; end
                    3'd7: begin
                        idx <= 3'd0;
                        if (rx_data == xsum) begin
                            frame_ok_sensor <= (hdr == 8'hA5);
                            frame_ok_gains  <= (hdr == 8'hC3);
                        end else chk_err <= 1'b1;
                    end
                endcase
            end
        end
    end

    // ---------------------------------------------------------- gains + PID
    reg  signed [15:0] kp, kd, sp, ang, rate;
    reg         [15:0] ki;
    reg                pid_start, pid_clear;
    wire signed [15:0] u;
    wire               pid_done, pid_busy;

    always @(posedge clk) begin
        pid_start <= 1'b0;
        pid_clear <= 1'b0;
        if (rst) begin
            kp <= 16'sd5120; ki <= 16'd5243; kd <= 16'sd666;   // tuned defaults
            sp <= 0; ang <= 0; rate <= 0;
        end else if (frame_ok_sensor) begin
            sp   <= {buf1, buf0};
            ang  <= {buf3, buf2};
            rate <= {buf5, buf4};
            pid_start <= 1'b1;
        end else if (frame_ok_gains) begin
            kp <= {buf1, buf0};
            ki <= {buf3, buf2};
            kd <= {buf5, buf4};
            pid_clear <= 1'b1;
        end
    end

    pid_core #(.OUT_MAX(OUT_MAX), .I_MAX(20)) u_pid (
        .clk(clk), .rst(rst || pid_clear), .start(pid_start),
        .setpoint(sp), .angle(ang), .rate(rate),
        .kp(kp), .ki(ki), .kd(kd),
        .u(u), .busy(pid_busy), .done(pid_done)
    );

    // ---------------------------------------------------------- reply
    reg [2:0] tx_left;          // bytes still to send
    reg [7:0] r0, r1, r2, r3;
    reg       tx_wait;

    always @(posedge clk) begin
        tx_send <= 1'b0;
        if (rst) begin
            tx_left <= 0; tx_wait <= 1'b0;
        end else begin
            if (pid_done) begin
                r0 <= 8'h5A; r1 <= u[7:0]; r2 <= u[15:8]; r3 <= u[7:0] ^ u[15:8];
                tx_left <= 3'd4;
            end else if (pid_clear) begin
                r0 <= 8'h3C;
                tx_left <= 3'd1;
            end else if (tx_left != 0 && !tx_busy && !tx_send && !tx_wait) begin
                tx_data <= r0;
                tx_send <= 1'b1;
                tx_wait <= 1'b1;
                {r0, r1, r2} <= {r1, r2, r3};
                tx_left <= tx_left - 1'b1;
            end
            if (tx_busy) tx_wait <= 1'b0;   // transmitter has taken the byte
        end
    end

    // ---------------------------------------------------------- motors
    wire [10:0] m1, m2;
    mixer u_mix (.throttle(11'd1500), .u(u), .m1(m1), .m2(m2));
    pwm_gen #(.CLK_HZ(CLK_HZ)) u_pwm1 (.clk(clk), .rst(rst), .width_us(m1), .pwm(pwm1));
    pwm_gen #(.CLK_HZ(CLK_HZ)) u_pwm2 (.clk(clk), .rst(rst), .width_us(m2), .pwm(pwm2));

    // ---------------------------------------------------------- LEDs
    reg [24:0] hb;
    reg [21:0] act;
    always @(posedge clk) begin
        hb <= hb + 1'b1;
        if (frame_ok_sensor || frame_ok_gains) act <= {22{1'b1}};
        else if (act != 0) act <= act - 1'b1;
    end
    wire sat = (u == OUT_MAX) || (u == -OUT_MAX);
    assign led = ~{u < 0, u > 0, sat, chk_err, act != 0, hb[24]};
endmodule
