// IMC-22 系统集成顶层
// RISC-V + NPU + SRAM

module imc22_system (
    input wire clk,
    input wire rst_n,
    
    // UART (调试用)
    output wire uart_tx,
    input wire uart_rx
);

// 系统参数
localparam SRAM_SIZE = 1024;  // 1KB

// SRAM
reg [7:0] sram [0:SRAM_SIZE-1];
reg [127:0] sram_rdata;
wire [9:0] sram_addr;
wire sram_rd;

// SRAM 读取逻辑
always @(posedge clk) begin
    if (sram_rd) begin
        sram_rdata <= {sram[sram_addr+15], sram[sram_addr+14], 
                       sram[sram_addr+13], sram[sram_addr+12],
                       sram[sram_addr+11], sram[sram_addr+10],
                       sram[sram_addr+9], sram[sram_addr+8],
                       sram[sram_addr+7], sram[sram_addr+6],
                       sram[sram_addr+5], sram[sram_addr+4],
                       sram[sram_addr+3], sram[sram_addr+2],
                       sram[sram_addr+1], sram[sram_addr]};
    end
end

// NPU 接口
wire [7:0] npu_cfg_addr;
wire [31:0] npu_cfg_wdata;
wire npu_cfg_wr;
wire [31:0] npu_cfg_rdata;
wire npu_start;
wire npu_done;
wire [31:0] npu_result;

// NPU 实例
imc22_npu u_npu (
    .clk(clk),
    .rst_n(rst_n),
    .cfg_addr(npu_cfg_addr),
    .cfg_wdata(npu_cfg_wdata),
    .cfg_wr(npu_cfg_wr),
    .cfg_rdata(npu_cfg_rdata),
    .sram_addr(sram_addr),
    .sram_rdata(sram_rdata),
    .sram_rd(sram_rd),
    .npu_start(npu_start),
    .npu_done(npu_done),
    .result(npu_result)
);

// 简化的控制逻辑（示例）
// 实际应该由 RISC-V CPU 控制
reg [31:0] cycle_count;
reg test_start;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cycle_count <= 0;
        test_start <= 0;
    end else begin
        cycle_count <= cycle_count + 1;
        
        // 在第10个周期启动NPU测试
        if (cycle_count == 10) begin
            test_start <= 1;
        end else begin
            test_start <= 0;
        end
    end
end

assign npu_start = test_start;
assign npu_cfg_addr = 8'h00;
assign npu_cfg_wdata = 32'h10;
assign npu_cfg_wr = 0;

// UART 简化（暂时未实现）
assign uart_tx = 1'b1;

endmodule
