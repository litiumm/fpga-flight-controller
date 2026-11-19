// tb_pid_vectors.v - self-checking unit test for pid_core
//
// Replays 20,000 vectors from model/fc_model.py (random gains, full-range
// and edge-case inputs, random resets) and compares every output against
// the bit-exact golden model. Also dumps a short VCD for GTKWave.
`timescale 1ns/1ps
module tb_pid_vectors;
`include "params.vh"
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst = 1'b1, start = 1'b0;
    reg  signed [15:0] kp, kd, sp, ang, rate;
    reg         [15:0] ki;
    wire signed [15:0] u;
    wire               busy, done;

    pid_core #(.OUT_MAX(OUT_MAX), .I_MAX(I_MAX)) dut (
        .clk(clk), .rst(rst), .start(start),
        .setpoint(sp), .angle(ang), .rate(rate),
        .kp(kp), .ki(ki), .kd(kd),
        .u(u), .busy(busy), .done(done)
    );

    integer fd, r, n, errors, vrst;
    reg [15:0] h_kp, h_ki, h_kd, h_sp, h_ang, h_rate, h_exp;

    initial begin
        $dumpfile("results/pid_core.vcd");
        $dumpvars(0, tb_pid_vectors);
        fd = $fopen("sim/vectors.txt", "r");
        if (fd == 0) begin
            $display("ERROR: sim/vectors.txt missing (run `make gen`)");
            $finish;
        end
        kp = 0; ki = 0; kd = 0; sp = 0; ang = 0; rate = 0;
        errors = 0;
        n = 0;
        repeat (3) @(posedge clk);
        rst <= 1'b0;
        @(posedge clk);

        while (!$feof(fd)) begin
            r = $fscanf(fd, "%d %h %h %h %h %h %h %h\n",
                        vrst, h_kp, h_ki, h_kd, h_sp, h_ang, h_rate, h_exp);
            if (r == 8) begin
                if (n == 200) $dumpoff;   // keep the VCD small
                kp = h_kp; ki = h_ki; kd = h_kd;
                sp = h_sp; ang = h_ang; rate = h_rate;
                if (vrst) begin
                    @(posedge clk) rst <= 1'b1;
                    @(posedge clk) rst <= 1'b0;
                    #1;
                end else begin
                    @(posedge clk) start <= 1'b1;
                    @(posedge clk) start <= 1'b0;
                    @(posedge done);
                    #1;
                end
                if (u !== $signed(h_exp)) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("MISMATCH #%0d: kp=%0d ki=%0d kd=%0d sp=%0d ang=%0d rate=%0d  u=%0d expected=%0d",
                                 n, kp, ki, kd, sp, ang, rate, u, $signed(h_exp));
                end
                n = n + 1;
            end
        end
        $fclose(fd);
        if (errors == 0)
            $display("PASS: pid_core matched golden model on all %0d vectors", n);
        else
            $display("FAIL: %0d / %0d vectors mismatched", errors, n);
        $finish;
    end
endmodule
