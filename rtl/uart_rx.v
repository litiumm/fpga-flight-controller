`timescale 1ns/1ps
// uart_rx.v - 8N1 UART receiver, samples mid-bit, 2-FF synchroniser

module uart_rx #(
    parameter integer CLKS_PER_BIT = 234   // 27 MHz / 115200
)(
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid       // 1-cycle pulse per received byte
);
    reg rx_s1 = 1'b1, rx_s2 = 1'b1;
    always @(posedge clk) begin
        rx_s1 <= rx;
        rx_s2 <= rx_s1;
    end

    localparam [1:0] IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;
    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bitn;
    reg [7:0]  shreg;

    always @(posedge clk) begin
        valid <= 1'b0;
        if (rst) begin
            state <= IDLE;
            cnt   <= 0;
            bitn  <= 0;
            data  <= 0;
        end else case (state)
            IDLE: if (!rx_s2) begin
                cnt   <= 0;
                state <= START;
            end
            START: if (cnt == CLKS_PER_BIT/2 - 1) begin
                cnt   <= 0;
                bitn  <= 0;
                state <= rx_s2 ? IDLE : DATA;   // glitch rejection
            end else cnt <= cnt + 1'b1;
            DATA: if (cnt == CLKS_PER_BIT - 1) begin
                cnt   <= 0;
                shreg <= {rx_s2, shreg[7:1]};
                if (bitn == 3'd7) state <= STOP;
                bitn  <= bitn + 1'b1;
            end else cnt <= cnt + 1'b1;
            STOP: if (cnt == CLKS_PER_BIT - 1) begin
                cnt   <= 0;
                state <= IDLE;
                if (rx_s2) begin               // valid stop bit
                    data  <= shreg;
                    valid <= 1'b1;
                end
            end else cnt <= cnt + 1'b1;
        endcase
    end
endmodule
