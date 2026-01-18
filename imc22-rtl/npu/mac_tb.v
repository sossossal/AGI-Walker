// IMC-22 NPU - MAC 单元测试台
`timescale 1ns/1ps

module mac_tb;
    reg clk, rst_n, acc_clear;
    reg [7:0] a, b;
    wire [31:0] acc;
    
    mac_int8 uut (
        .clk(clk),
        .rst_n(rst_n),
        .a(a),
        .b(b),
        .acc_clear(acc_clear),
        .acc(acc)
    );
    
    initial clk = 0;
    always #5 clk = ~clk;
    
    initial begin
        $dumpfile("mac.vcd");
        $dumpvars(0, mac_tb);
        
        rst_n = 0; acc_clear = 1; a = 0; b = 0;
        #20 rst_n = 1;
        #10 acc_clear = 0;
        
        // 测试: 2*3 + 4*5 = 26
        #10 a = 2; b = 3;
        #10 a = 4; b = 5;
        #10;
        
        $display("Result: %0d (expected 26)", $signed(acc));
        $finish;
    end
endmodule
