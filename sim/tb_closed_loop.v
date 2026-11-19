// tb_closed_loop.v - pid_core + mixer closing the loop around a plant model
//
// The plant (single-axis rig with motor lag) is integrated in `real`
// arithmetic in exactly the same operation order as model/fc_model.py, so
// the RTL run can be checked bit-exactly against the Python golden model.
`timescale 1ns/1ps
module tb_closed_loop;
`include "params.vh"

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst = 1'b1, start = 1'b0;
    reg  signed [15:0] sp_q, ang_q, rate_q;
    wire signed [15:0] kp = KP_Q, kd = KD_Q;
    wire        [15:0] ki = KI_Q;
    wire        [10:0] thr = THROTTLE;
    wire signed [15:0] u;
    wire               busy, done;
    wire        [10:0] m1, m2;

    pid_core #(.OUT_MAX(OUT_MAX), .I_MAX(I_MAX)) dut (
        .clk(clk), .rst(rst), .start(start),
        .setpoint(sp_q), .angle(ang_q), .rate(rate_q),
        .kp(kp), .ki(ki), .kd(kd),
        .u(u), .busy(busy), .done(done)
    );

    mixer mix (.throttle(thr), .u(u), .m1(m1), .m2(m2));

    // floor(x*scale + 0.5), saturated to int16 - matches fc_model.q()
    function integer quant(input real x, input real scale);
        real y;
        integer v;
        begin
            y = x * scale + 0.5;
            v = $rtoi(y);
            if (y < 0.0 && $itor(v) != y) v = v - 1;
            if (v >  32767) v =  32767;
            if (v < -32768) v = -32768;
            quant = v;
        end
    endfunction

    integer fd, fo, k, s, sub, r, im1, im2;
    real sp, na, nr, ang, rate;
    real theta, omega, f1, f2, f1c, f2c, tq, alpha;

    initial begin
        fd = $fopen("sim/stimulus.txt", "r");
        fo = $fopen("results/rtl_closed_loop.csv", "w");
        if (fd == 0 || fo == 0) begin
            $display("ERROR: cannot open stimulus/output (run from repo root)");
            $finish;
        end

        theta = THETA0 * PI / 180.0;
        omega = 0.0;
        f1 = TMAX * (THROTTLE - 1000) / 1000.0;
        f2 = f1;
        sub = 0;

        repeat (4) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);

        for (k = 0; k < N_STEPS; k = k + 1) begin
            r = $fscanf(fd, "%f %f %f\n", sp, na, nr);
            ang  = theta * 180.0 / PI + na;
            rate = omega * 180.0 / PI + nr;
            sp_q   = quant(sp,   256.0);
            ang_q  = quant(ang,  256.0);
            rate_q = quant(rate, 16.0);

            @(posedge clk) start <= 1'b1;
            @(posedge clk) start <= 1'b0;
            @(posedge done);
            #1;

            $fwrite(fo, "%0.6f,%0.6f,%0.12f,%0d,%0d,%0d\n",
                    k * DT_CTRL, sp, theta * 180.0 / PI, u, m1, m2);

            im1 = m1;
            im2 = m2;
            for (s = 0; s < SUBSTEPS; s = s + 1) begin
                f1c = TMAX * (im1 - 1000) / 1000.0;
                f2c = TMAX * (im2 - 1000) / 1000.0;
                f1 = f1 + (f1c - f1) * DT_PLANT / TAU;
                f2 = f2 + (f2c - f2) * DT_PLANT / TAU;
                tq = ARM * (f1 - f2) + IMBAL;
                if (sub >= DIST_START && sub < DIST_START + DIST_LEN)
                    tq = tq + DIST;
                alpha = (tq - DAMP * omega) / J;
                omega = omega + alpha * DT_PLANT;
                theta = theta + omega * DT_PLANT;
                sub = sub + 1;
            end
        end

        $fclose(fd);
        $fclose(fo);
        $display("closed loop: %0d samples written to results/rtl_closed_loop.csv", N_STEPS);
        $finish;
    end
endmodule
