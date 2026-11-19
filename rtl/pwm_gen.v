`timescale 1ns/1ps
// pwm_gen.v - standard ESC PWM output (1000-2000 us pulse)
//
// A prescaler makes a 1 us tick from CLK_HZ. The pulse width is latched at
// the start of each frame, so a width change mid-frame never produces a
// runt pulse.

module pwm_gen #(
    parameter integer CLK_HZ    = 27_000_000,
    parameter integer PERIOD_US = 2500           // 400 Hz frame
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [10:0] width_us,
    output reg         pwm
);
    localparam integer DIV = CLK_HZ / 1_000_000;

    reg [$clog2(DIV)-1:0]       pre;
    reg [$clog2(PERIOD_US)-1:0] cnt;
    reg [10:0]                  width_l;

    always @(posedge clk) begin
        if (rst) begin
            pre     <= 0;
            cnt     <= 0;
            width_l <= 11'd1000;
            pwm     <= 1'b0;
        end else begin
            if (pre == DIV-1) begin
                pre <= 0;
                if (cnt == PERIOD_US-1) begin
                    cnt     <= 0;
                    width_l <= width_us;
                end else begin
                    cnt <= cnt + 1'b1;
                end
            end else begin
                pre <= pre + 1'b1;
            end
            pwm <= (cnt < width_l);
        end
    end
endmodule
