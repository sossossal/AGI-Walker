// IMC-22 NPU - INT8 MAC 单元
// 乘累加单元，用于神经网络推理

module mac_int8 (
    input wire clk,
    input wire rst_n,
    input wire [7:0] a,      // INT8 输入 A
    input wire [7:0] b,      // INT8 输入 B  
    input wire acc_clear,    // 清除累加器
    output reg [31:0] acc    // 32位累加器
);

// 带符号乘法和累加
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        acc <= 32'b0;
    end else if (acc_clear) begin
        acc <= 32'b0;
    end else begin
        // 有符号乘法并累加
        acc <= acc + ($signed(a) * $signed(b));
    end
end

endmodule
