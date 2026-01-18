// IMC-22 NPU - 16-MAC 阵列
// 16个并行 MAC 单元用于矩阵乘法加速

module npu_mac_array (
    input wire clk,
    input wire rst_n,
    
    // 16个 INT8 输入 A (权重)
    input wire [7:0] weight_0, weight_1, weight_2, weight_3,
    input wire [7:0] weight_4, weight_5, weight_6, weight_7,
    input wire [7:0] weight_8, weight_9, weight_10, weight_11,
    input wire [7:0] weight_12, weight_13, weight_14, weight_15,
    
    // 16个 INT8 输入 B (激活值)
    input wire [7:0] input_0, input_1, input_2, input_3,
    input wire [7:0] input_4, input_5, input_6, input_7,
    input wire [7:0] input_8, input_9, input_10, input_11,
    input wire [7:0] input_12, input_13, input_14, input_15,
    
    input wire acc_clear,           // 清除所有累加器
    output wire [31:0] result       // 最终累加结果
);

// 16个 MAC 单元的累加器
wire [31:0] acc_0, acc_1, acc_2, acc_3;
wire [31:0] acc_4, acc_5, acc_6, acc_7;
wire [31:0] acc_8, acc_9, acc_10, acc_11;
wire [31:0] acc_12, acc_13, acc_14, acc_15;

// 实例化 16 个 MAC 单元
mac_int8 mac_0 (.clk(clk), .rst_n(rst_n), .a(weight_0), .b(input_0), .acc_clear(acc_clear), .acc(acc_0));
mac_int8 mac_1 (.clk(clk), .rst_n(rst_n), .a(weight_1), .b(input_1), .acc_clear(acc_clear), .acc(acc_1));
mac_int8 mac_2 (.clk(clk), .rst_n(rst_n), .a(weight_2), .b(input_2), .acc_clear(acc_clear), .acc(acc_2));
mac_int8 mac_3 (.clk(clk), .rst_n(rst_n), .a(weight_3), .b(input_3), .acc_clear(acc_clear), .acc(acc_3));
mac_int8 mac_4 (.clk(clk), .rst_n(rst_n), .a(weight_4), .b(input_4), .acc_clear(acc_clear), .acc(acc_4));
mac_int8 mac_5 (.clk(clk), .rst_n(rst_n), .a(weight_5), .b(input_5), .acc_clear(acc_clear), .acc(acc_5));
mac_int8 mac_6 (.clk(clk), .rst_n(rst_n), .a(weight_6), .b(input_6), .acc_clear(acc_clear), .acc(acc_6));
mac_int8 mac_7 (.clk(clk), .rst_n(rst_n), .a(weight_7), .b(input_7), .acc_clear(acc_clear), .acc(acc_7));
mac_int8 mac_8 (.clk(clk), .rst_n(rst_n), .a(weight_8), .b(input_8), .acc_clear(acc_clear), .acc(acc_8));
mac_int8 mac_9 (.clk(clk), .rst_n(rst_n), .a(weight_9), .b(input_9), .acc_clear(acc_clear), .acc(acc_9));
mac_int8 mac_10 (.clk(clk), .rst_n(rst_n), .a(weight_10), .b(input_10), .acc_clear(acc_clear), .acc(acc_10));
mac_int8 mac_11 (.clk(clk), .rst_n(rst_n), .a(weight_11), .b(input_11), .acc_clear(acc_clear), .acc(acc_11));
mac_int8 mac_12 (.clk(clk), .rst_n(rst_n), .a(weight_12), .b(input_12), .acc_clear(acc_clear), .acc(acc_12));
mac_int8 mac_13 (.clk(clk), .rst_n(rst_n), .a(weight_13), .b(input_13), .acc_clear(acc_clear), .acc(acc_13));
mac_int8 mac_14 (.clk(clk), .rst_n(rst_n), .a(weight_14), .b(input_14), .acc_clear(acc_clear), .acc(acc_14));
mac_int8 mac_15 (.clk(clk), .rst_n(rst_n), .a(weight_15), .b(input_15), .acc_clear(acc_clear), .acc(acc_15));

// 累加树 - 4级流水线加法树
// 第1级: 16 -> 8
reg signed [31:0] stage1_0, stage1_1, stage1_2, stage1_3;
reg signed [31:0] stage1_4, stage1_5, stage1_6, stage1_7;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        stage1_0 <= 0; stage1_1 <= 0; stage1_2 <= 0; stage1_3 <= 0;
        stage1_4 <= 0; stage1_5 <= 0; stage1_6 <= 0; stage1_7 <= 0;
    end else begin
        stage1_0 <= $signed(acc_0) + $signed(acc_1);
        stage1_1 <= $signed(acc_2) + $signed(acc_3);
        stage1_2 <= $signed(acc_4) + $signed(acc_5);
        stage1_3 <= $signed(acc_6) + $signed(acc_7);
        stage1_4 <= $signed(acc_8) + $signed(acc_9);
        stage1_5 <= $signed(acc_10) + $signed(acc_11);
        stage1_6 <= $signed(acc_12) + $signed(acc_13);
        stage1_7 <= $signed(acc_14) + $signed(acc_15);
    end
end

// 第2级: 8 -> 4
reg signed [31:0] stage2_0, stage2_1, stage2_2, stage2_3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        stage2_0 <= 0; stage2_1 <= 0; stage2_2 <= 0; stage2_3 <= 0;
    end else begin
        stage2_0 <= stage1_0 + stage1_1;
        stage2_1 <= stage1_2 + stage1_3;
        stage2_2 <= stage1_4 + stage1_5;
        stage2_3 <= stage1_6 + stage1_7;
    end
end

// 第3级: 4 -> 2
reg signed [31:0] stage3_0, stage3_1;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        stage3_0 <= 0; stage3_1 <= 0;
    end else begin
        stage3_0 <= stage2_0 + stage2_1;
        stage3_1 <= stage2_2 + stage2_3;
    end
end

// 第4级: 2 -> 1 (最终结果)
reg signed [31:0] result_reg;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        result_reg <= 0;
    end else begin
        result_reg <= stage3_0 + stage3_1;
    end
end

assign result = result_reg;

endmodule
