// IMC-22 NPU - 简化的 16-MAC 点积计算
// 不使用 MAC 累加器，直接计算点积

module npu_dot_product (
    input wire clk,
    input wire rst_n,
    input wire enable,  // 使能信号
    
    // 16个 INT8 权重
    input wire [7:0] weight_0, weight_1, weight_2, weight_3,
    input wire [7:0] weight_4, weight_5, weight_6, weight_7,
    input wire [7:0] weight_8, weight_9, weight_10, weight_11,
    input wire [7:0] weight_12, weight_13, weight_14, weight_15,
    
    // 16个 INT8 输入
    input wire [7:0] input_0, input_1, input_2, input_3,
    input wire [7:0] input_4, input_5, input_6, input_7,
    input wire [7:0] input_8, input_9, input_10, input_11,
    input wire [7:0] input_12, input_13, input_14, input_15,
    
    output reg [31:0] result,
    output reg valid
);

// 16个乘法结果
wire signed [15:0] prod_0 = $signed(weight_0) * $signed(input_0);
wire signed [15:0] prod_1 = $signed(weight_1) * $signed(input_1);
wire signed [15:0] prod_2 = $signed(weight_2) * $signed(input_2);
wire signed [15:0] prod_3 = $signed(weight_3) * $signed(input_3);
wire signed [15:0] prod_4 = $signed(weight_4) * $signed(input_4);
wire signed [15:0] prod_5 = $signed(weight_5) * $signed(input_5);
wire signed [15:0] prod_6 = $signed(weight_6) * $signed(input_6);
wire signed [15:0] prod_7 = $signed(weight_7) * $signed(input_7);
wire signed [15:0] prod_8 = $signed(weight_8) * $signed(input_8);
wire signed [15:0] prod_9 = $signed(weight_9) * $signed(input_9);
wire signed [15:0] prod_10 = $signed(weight_10) * $signed(input_10);
wire signed [15:0] prod_11 = $signed(weight_11) * $signed(input_11);
wire signed [15:0] prod_12 = $signed(weight_12) * $signed(input_12);
wire signed [15:0] prod_13 = $signed(weight_13) * $signed(input_13);
wire signed [15:0] prod_14 = $signed(weight_14) * $signed(input_14);
wire signed [15:0] prod_15 = $signed(weight_15) * $signed(input_15);

// 累加所有乘积
wire signed [31:0] sum = prod_0 + prod_1 + prod_2 + prod_3 +
                          prod_4 + prod_5 + prod_6 + prod_7 +
                          prod_8 + prod_9 + prod_10 + prod_11 +
                          prod_12 + prod_13 + prod_14 + prod_15;

// 寄存输出
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        result <= 0;
        valid <= 0;
    end else if (enable) begin
        result <= sum;
        valid <= 1;
    end else begin
        valid <= 0;
    end
end

endmodule
