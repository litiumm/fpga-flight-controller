`timescale 1ns/1ps
// pid_core.v - fixed-point attitude PID controller, one shared multiplier
//
// One update per `start` pulse (the loop rate is set by whoever pulses it,
// 1 kHz in this design). The three gain products run through a single
// 17x17 signed multiplier over 3 cycles, which trades a few cycles of
// latency for 1 DSP instead of 3. That's fine: at 27 MHz and 1 kHz the
// budget is 27,000 cycles per update and the core uses 5.
//
// Number formats (all two's complement unless marked U):
//   setpoint, angle : Q8.8   degrees            (+/-128 deg, 0.0039 deg LSB)
//   rate (gyro)     : Q12.4  deg/s              (+/-2048 dps, MPU6050-class range)
//   kp              : Q8.8   us per deg
//   ki              : UQ0.16 Ki*dt, us per deg per sample (dt folded in)
//   kd              : Q8.8   us per deg/s
//   u               : integer microseconds of differential motor pulse
//
// Internally all three terms are aligned to Q.24 in a 48-bit accumulator.
// D acts on the measured rate (gyro), not on d(error)/dt, so setpoint
// steps don't cause a derivative kick. The I-term is clamped to +/-I_MAX
// (anti-windup), and the output is rounded and saturated to +/-OUT_MAX.

module pid_core #(
    parameter integer OUT_MAX = 400,
    parameter integer I_MAX   = 100
)(
    input  wire               clk,
    input  wire               rst,       // synchronous, active high; clears integrator
    input  wire               start,     // 1-cycle pulse: sample inputs, run one update
    input  wire signed [15:0] setpoint,
    input  wire signed [15:0] angle,
    input  wire signed [15:0] rate,
    input  wire signed [15:0] kp,
    input  wire        [15:0] ki,
    input  wire signed [15:0] kd,
    output reg  signed [15:0] u,
    output wire               busy,
    output reg                done       // 1-cycle pulse when u is updated
);
    localparam integer QF = 24;

    localparam [2:0] S_IDLE = 3'd0,
                     S_P    = 3'd1,
                     S_I    = 3'd2,
                     S_D    = 3'd3,
                     S_SUM  = 3'd4;

    reg [2:0] state;
    assign busy = (state != S_IDLE);

    // Limits in accumulator format (context-determined 48-bit shift)
    wire signed [47:0] i_lim   = I_MAX   <<< QF;
    wire signed [47:0] out_lim = OUT_MAX;

    // Shared multiplier with registered operands
    reg  signed [16:0] ma, mb;
    wire signed [33:0] prod  = ma * mb;
    wire signed [47:0] prod48 = {{14{prod[33]}}, prod};

    // Latched inputs and partial terms
    wire signed [16:0] err_w = {setpoint[15], setpoint} - {angle[15], angle};
    reg  signed [16:0] err;
    reg  signed [16:0] rate_l;
    reg  signed [47:0] p_term, d_term, i_acc;

    // Integrator update with clamp
    wire signed [47:0] i_next = i_acc + prod48;
    wire signed [47:0] i_clamped = (i_next >  i_lim) ?  i_lim :
                                   (i_next < -i_lim) ? -i_lim : i_next;

    // Sum, round-half-up, convert to integer us, saturate
    wire signed [47:0] sum     = p_term + i_acc + d_term;
    wire signed [47:0] sum_int = (sum + (48'sd1 <<< (QF-1))) >>> QF;
    wire signed [47:0] sum_sat = (sum_int >  out_lim) ?  out_lim :
                                 (sum_int < -out_lim) ? -out_lim : sum_int;

    always @(posedge clk) begin
        done <= 1'b0;
        if (rst) begin
            state  <= S_IDLE;
            i_acc  <= 48'sd0;
            p_term <= 48'sd0;
            d_term <= 48'sd0;
            u      <= 16'sd0;
            ma     <= 17'sd0;
            mb     <= 17'sd0;
            err    <= 17'sd0;
            rate_l <= 17'sd0;
        end else begin
            case (state)
                S_IDLE: if (start) begin
                    err    <= err_w;
                    rate_l <= {rate[15], rate};
                    ma     <= err_w;
                    mb     <= {kp[15], kp};
                    state  <= S_P;
                end
                S_P: begin                       // err(Q8.8) * kp(Q8.8) = Q.16
                    p_term <= prod48 <<< 8;
                    ma     <= err;
                    mb     <= {1'b0, ki};
                    state  <= S_I;
                end
                S_I: begin                       // err(Q8.8) * ki(UQ0.16) = Q.24
                    i_acc  <= i_clamped;
                    ma     <= rate_l;
                    mb     <= {kd[15], kd};
                    state  <= S_D;
                end
                S_D: begin                       // rate(Q12.4) * kd(Q8.8) = Q.12
                    d_term <= -(prod48 <<< 12);
                    state  <= S_SUM;
                end
                S_SUM: begin
                    u     <= sum_sat[15:0];
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
