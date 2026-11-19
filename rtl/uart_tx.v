`timescale 1ns/1ps
// uart_tx.v - 8N1 UART transmitter

module uart_tx #(
    parameter integer CLKS_PER_BIT = 234
)(
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       send,      // 1-cycle pulse, ignored while busy
    output reg        tx,
    output wire       busy
);
    reg [9:0]  frame;            // {stop, data, start}, shifted out LSB first
    reg [3:0]  bits_left;
    reg [15:0] cnt;

    assign busy = (bits_left != 0);

    always @(posedge clk) begin
        if (rst) begin
            tx        <= 1'b1;
            bits_left <= 0;
            cnt       <= 0;
        end else if (!busy) begin
            tx <= 1'b1;
            if (send) begin
                frame     <= {1'b1, data, 1'b0};
                bits_left <= 4'd10;
                cnt       <= 0;
            end
        end else begin
            tx <= frame[0];
            if (cnt == CLKS_PER_BIT - 1) begin
                cnt       <= 0;
                frame     <= {1'b1, frame[9:1]};
                bits_left <= bits_left - 1'b1;
            end else cnt <= cnt + 1'b1;
        end
    end
endmodule
