// IMC-22 NPU 顶层模块
// 包含 16-并行计算 + 控制器 + SRAM接口

module imc22_npu (
    input wire clk,
    input wire rst_n,
    
    // AXI-Lite 配置接口（简化）
    input wire [7:0] cfg_addr,
    input wire [31:0] cfg_wdata,
    input wire cfg_wr,
    output reg [31:0] cfg_rdata,
    
    // SRAM 接口（权重和输入数据）
    output reg [9:0] sram_addr,
    input wire [127:0] sram_rdata,  // 16 bytes = 16个INT8
    output reg sram_rd,
    
    // 控制和状态
    input wire npu_start,
    output reg npu_done,
    output reg [31:0] result
);

// 配置寄存器
reg [31:0] num_elements;  // 要处理的元素数量
reg [9:0] weight_base;    // 权重基地址
reg [9:0] input_base;     // 输入基地址

// 状态机
localparam IDLE = 2'b00;
localparam LOAD_WEIGHT = 2'b01;
localparam LOAD_INPUT = 2'b10;
localparam COMPUTE = 2'b11;

reg [1:0] state;
reg [3:0] load_count;

// 16个权重和输入寄存器
reg [7:0] weights [0:15];
reg [7:0] inputs [0:15];

// 点积计算模块
wire [31:0] dot_result;
wire dot_valid;
reg dot_enable;

npu_dot_product u_dot_product (
    .clk(clk),
    .rst_n(rst_n),
    .enable(dot_enable),
    .weight_0(weights[0]), .weight_1(weights[1]), .weight_2(weights[2]), .weight_3(weights[3]),
    .weight_4(weights[4]), .weight_5(weights[5]), .weight_6(weights[6]), .weight_7(weights[7]),
    .weight_8(weights[8]), .weight_9(weights[9]), .weight_10(weights[10]), .weight_11(weights[11]),
    .weight_12(weights[12]), .weight_13(weights[13]), .weight_14(weights[14]), .weight_15(weights[15]),
    .input_0(inputs[0]), .input_1(inputs[1]), .input_2(inputs[2]), .input_3(inputs[3]),
    .input_4(inputs[4]), .input_5(inputs[5]), .input_6(inputs[6]), .input_7(inputs[7]),
    .input_8(inputs[8]), .input_9(inputs[9]), .input_10(inputs[10]), .input_11(inputs[11]),
    .input_12(inputs[12]), .input_13(inputs[13]), .input_14(inputs[14]), .input_15(inputs[15]),
    .result(dot_result),
    .valid(dot_valid)
);

// 配置寄存器读写
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        num_elements <= 16;
        weight_base <= 0;
        input_base <= 256;
    end else if (cfg_wr) begin
        case (cfg_addr)
            8'h00: num_elements <= cfg_wdata;
            8'h04: weight_base <= cfg_wdata[9:0];
            8'h08: input_base <= cfg_wdata[9:0];
        endcase
    end
end

always @(*) begin
    case (cfg_addr)
        8'h00: cfg_rdata = num_elements;
        8'h04: cfg_rdata = {22'b0, weight_base};
        8'h08: cfg_rdata = {22'b0, input_base};
        8'h0C: cfg_rdata = result;
        8'h10: cfg_rdata = {31'b0, npu_done};
        default: cfg_rdata = 32'h0;
    endcase
end

// NPU 控制状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= IDLE;
        npu_done <= 0;
        sram_rd <= 0;
        sram_addr <= 0;
        dot_enable <= 0;
        load_count <= 0;
        result <= 0;
    end else begin
        case (state)
            IDLE: begin
                npu_done <= 0;
                dot_enable <= 0;
                if (npu_start) begin
                    state <= LOAD_WEIGHT;
                    sram_addr <= weight_base;
                    sram_rd <= 1;
                    load_count <= 0;
                end
            end
            
            LOAD_WEIGHT: begin
                if (sram_rd) begin
                    sram_rd <= 0;
                    // 加载16个权重
                    weights[0] <= sram_rdata[7:0];
                    weights[1] <= sram_rdata[15:8];
                    weights[2] <= sram_rdata[23:16];
                    weights[3] <= sram_rdata[31:24];
                    weights[4] <= sram_rdata[39:32];
                    weights[5] <= sram_rdata[47:40];
                    weights[6] <= sram_rdata[55:48];
                    weights[7] <= sram_rdata[63:56];
                    weights[8] <= sram_rdata[71:64];
                    weights[9] <= sram_rdata[79:72];
                    weights[10] <= sram_rdata[87:80];
                    weights[11] <= sram_rdata[95:88];
                    weights[12] <= sram_rdata[103:96];
                    weights[13] <= sram_rdata[111:104];
                    weights[14] <= sram_rdata[119:112];
                    weights[15] <= sram_rdata[127:120];
                    
                    state <= LOAD_INPUT;
                    sram_addr <= input_base;
                    sram_rd <= 1;
                end
            end
            
            LOAD_INPUT: begin
                if (sram_rd) begin
                    sram_rd <= 0;
                    // 加载16个输入
                    inputs[0] <= sram_rdata[7:0];
                    inputs[1] <= sram_rdata[15:8];
                    inputs[2] <= sram_rdata[23:16];
                    inputs[3] <= sram_rdata[31:24];
                    inputs[4] <= sram_rdata[39:32];
                    inputs[5] <= sram_rdata[47:40];
                    inputs[6] <= sram_rdata[55:48];
                    inputs[7] <= sram_rdata[63:56];
                    inputs[8] <= sram_rdata[71:64];
                    inputs[9] <= sram_rdata[79:72];
                    inputs[10] <= sram_rdata[87:80];
                    inputs[11] <= sram_rdata[95:88];
                    inputs[12] <= sram_rdata[103:96];
                    inputs[13] <= sram_rdata[111:104];
                    inputs[14] <= sram_rdata[119:112];
                    inputs[15] <= sram_rdata[127:120];
                    
                    state <= COMPUTE;
                    dot_enable <= 1;
                end
            end
            
            COMPUTE: begin
                dot_enable <= 0;
                if (dot_valid) begin
                    result <= dot_result;
                    npu_done <= 1;
                    state <= IDLE;
                end
            end
        endcase
    end
end

endmodule
