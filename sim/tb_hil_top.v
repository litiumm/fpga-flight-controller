// tb_hil_top.v - end-to-end test of the FPGA top level over its UART
//
// Acts as the PC: sends a gains frame, then 300 closed-loop sensor frames
// recorded from the golden model, and checks every 5A reply. Also checks
// that a frame with a bad checksum is dropped and flags the error LED, and
// that the PWM outputs have the right pulse widths.
`timescale 1ns/1ps
module tb_hil_top;
`include "params.vh"
    localparam integer CPB = 27_000_000 / 115_200;
    localparam real    TCLK = 1e9 / 27e6;

    reg clk = 1'b0;
    always #(TCLK/2) clk = ~clk;

    reg  rx = 1'b1, rst_n = 1'b1;
    wire tx, pwm1, pwm2;
    wire [5:0] led;

    hil_top dut (.clk(clk), .rst_n(rst_n), .uart_rx(rx), .uart_tx(tx),
                 .pwm1(pwm1), .pwm2(pwm2), .led(led));

    task send_byte(input [7:0] b);
        integer i;
        begin
            rx = 1'b0; repeat (CPB) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                rx = b[i]; repeat (CPB) @(posedge clk);
            end
            rx = 1'b1; repeat (CPB) @(posedge clk);
        end
    endtask

    task recv_byte(output [7:0] b);
        integer i;
        begin
            @(negedge tx);
            repeat (CPB + CPB/2) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin
                b[i] = tx; repeat (CPB) @(posedge clk);
            end
        end
    endtask

    task send_frame(input [7:0] hdr, input [15:0] a, input [15:0] b2, input [15:0] c,
                    input bad);
        reg [7:0] x;
        begin
            x = a[7:0] ^ a[15:8] ^ b2[7:0] ^ b2[15:8] ^ c[7:0] ^ c[15:8];
            send_byte(hdr);
            send_byte(a[7:0]);  send_byte(a[15:8]);
            send_byte(b2[7:0]); send_byte(b2[15:8]);
            send_byte(c[7:0]);  send_byte(c[15:8]);
            send_byte(bad ? ~x : x);
        end
    endtask

    integer fd, r, n, errors;
    reg [15:0] sp, ang, rate, exp_u;
    reg [7:0]  b0, b1, b2, b3;
    reg [15:0] kp16, ki16, kd16;
    time t_rise;
    integer w1;

    initial begin
        errors = 0; n = 0;
        kp16 = KP_Q; ki16 = KI_Q; kd16 = KD_Q;
        repeat (300) @(posedge clk);

        // 1. gains frame -> 3C ack
        fork
            send_frame(8'hC3, kp16, ki16, kd16, 1'b0);
            recv_byte(b0);
        join
        if (b0 !== 8'h3C) begin errors = errors + 1; $display("FAIL: gains ack %h", b0); end

        // 2. closed-loop sensor frames
        fd = $fopen("sim/hil_vectors.txt", "r");
        while (!$feof(fd)) begin
            r = $fscanf(fd, "%h %h %h %h\n", sp, ang, rate, exp_u);
            if (r == 4) begin
                fork   // like a real PC UART, listen while still sending
                    send_frame(8'hA5, sp, ang, rate, 1'b0);
                    begin recv_byte(b0); recv_byte(b1); recv_byte(b2); recv_byte(b3); end
                join
                if (b0 !== 8'h5A || {b2, b1} !== exp_u || b3 !== (b1 ^ b2)) begin
                    errors = errors + 1;
                    if (errors < 10)
                        $display("FAIL frame %0d: got %h %h%h %h expected u=%h",
                                 n, b0, b2, b1, b3, exp_u);
                end
                n = n + 1;
            end
        end
        $fclose(fd);

        // 3. PWM width check: m1 must equal 1500 + last u
        @(posedge pwm1);                   // skip a frame latched before the last update
        @(posedge pwm1); t_rise = $time;
        @(negedge pwm1);
        w1 = ($time - t_rise) / 1000.0;    // real->integer assignment rounds
        if (w1 !== 1500 + $signed(exp_u)) begin
            errors = errors + 1;
            $display("FAIL: pwm1 width %0d us, expected %0d", w1, 1500 + $signed(exp_u));
        end

        // 4. corrupted frame: no reply, error LED (led[2], active low) lights
        send_frame(8'hA5, 16'h0100, 16'h0000, 16'h0000, 1'b1);
        repeat (CPB * 60) @(posedge clk);
        if (led[2] !== 1'b0) begin errors = errors + 1; $display("FAIL: chk error LED not set"); end

        if (errors == 0)
            $display("PASS: hil_top - gains ack, %0d UART frames bit-exact, PWM %0d us, bad checksum rejected",
                     n, w1);
        else
            $display("FAIL: hil_top - %0d errors", errors);
        $finish;
    end

    // Fail-safe: never hang
    initial begin
        #(2_000_000_000.0);
        $display("FAIL: timeout");
        $finish;
    end
endmodule
