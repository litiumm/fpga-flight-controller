`timescale 1ns/1ps
// mixer.v - single-axis (seesaw / one quad axis) motor mixer
//
// m1 = throttle + u, m2 = throttle - u, each clamped to the ESC pulse
// range [MIN_US, MAX_US]. Purely combinational.

module mixer #(
    parameter integer MIN_US = 1000,
    parameter integer MAX_US = 2000
)(
    input  wire        [10:0] throttle,  // us, e.g. 1500 for hover
    input  wire signed [15:0] u,         // us, from pid_core
    output wire        [10:0] m1,
    output wire        [10:0] m2
);
    wire signed [17:0] t  = {7'd0, throttle};
    wire signed [17:0] uu = {{2{u[15]}}, u};
    wire signed [17:0] a  = t + uu;
    wire signed [17:0] b  = t - uu;

    function [10:0] clamp(input signed [17:0] x);
        clamp = (x < MIN_US) ? MIN_US[10:0] :
                (x > MAX_US) ? MAX_US[10:0] : x[10:0];
    endfunction

    assign m1 = clamp(a);
    assign m2 = clamp(b);
endmodule
