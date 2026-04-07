// NPU 点积模块测试台
`timescale 1ns/1ps

module npu_dot_product_tb;
    reg clk, rst_n, enable;
    reg [7:0] weight [0:15];
    reg [7:0] input_data [0:15];
    wire [31:0] result;
    wire valid;
    
    npu_dot_product uut (
        .clk(clk), .rst_n(rst_n), .enable(enable),
        .weight_0(weight[0]), .weight_1(weight[1]), .weight_2(weight[2]), .weight_3(weight[3]),
        .weight_4(weight[4]), .weight_5(weight[5]), .weight_6(weight[6]), .weight_7(weight[7]),
        .weight_8(weight[8]), .weight_9(weight[9]), .weight_10(weight[10]), .weight_11(weight[11]),
        .weight_12(weight[12]), .weight_13(weight[13]), .weight_14(weight[14]), .weight_15(weight[15]),
        .input_0(input_data[0]), .input_1(input_data[1]), .input_2(input_data[2]), .input_3(input_data[3]),
        .input_4(input_data[4]), .input_5(input_data[5]), .input_6(input_data[6]), .input_7(input_data[7]),
        .input_8(input_data[8]), .input_9(input_data[9]), .input_10(input_data[10]), .input_11(input_data[11]),
        .input_12(input_data[12]), .input_13(input_data[13]), .input_14(input_data[14]), .input_15(input_data[15]),
        .result(result), .valid(valid)
    );
    
    initial clk = 0;
    always #5 clk = ~clk;
    
    initial begin
        $dumpfile("dot_product.vcd");
        $dumpvars(0, npu_dot_product_tb);
        
        rst_n = 0; enable = 0;
        #20 rst_n = 1;
        
        // 测试: [1,2,3,...,16] · [1,1,1,...,1] = 136
        weight[0]=1; weight[1]=2; weight[2]=3; weight[3]=4;
        weight[4]=5; weight[5]=6; weight[6]=7; weight[7]=8;
        weight[8]=9; weight[9]=10; weight[10]=11; weight[11]=12;
        weight[12]=13; weight[13]=14; weight[14]=15; weight[15]=16;
        
        input_data[0]=1; input_data[1]=1; input_data[2]=1; input_data[3]=1;
        input_data[4]=1; input_data[5]=1; input_data[6]=1; input_data[7]=1;
        input_data[8]=1; input_data[9]=1; input_data[10]=1; input_data[11]=1;
        input_data[12]=1; input_data[13]=1; input_data[14]=1; input_data[15]=1;
        
        #10 enable = 1;
        #10 enable = 0;
        #20;
        
        $display("NPU Dot Product Test");
        $display("Result: %0d (expected 136)", $signed(result));
        if (result == 136)
            $display("✓ TEST PASSED!");
        else
            $display("✗ TEST FAILED!");
        
        $finish;
    end
endmodule
