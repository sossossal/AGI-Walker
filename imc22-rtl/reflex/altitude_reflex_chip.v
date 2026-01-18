// Hive-Reflex 高度反馈芯片
// 结合 PID 控制和神经网络反射

module altitude_reflex_chip (
    input wire clk,
    input wire rst_n,
    
    // 传感器输入
    input wire signed [15:0] altitude_current,    // 当前高度 (cm)
    input wire signed [15:0] altitude_target,     // 目标高度 (cm)
    input wire signed [15:0] velocity_z,          // Z轴速度 (cm/s)
    input wire signed [15:0] accel_z,             // Z轴加速度 (cm/s²)
    
    // 配置参数
    input wire [7:0] compliance,      // 柔顺系数 (0-255 -> 0.0-1.0)
    input wire signed [15:0] pid_kp,  // PID 比例系数
    input wire signed [15:0] pid_ki,  // PID 积分系数
    input wire signed [15:0] pid_kd,  // PID 微分系数
    
    // 输出
    output reg signed [15:0] thrust_output,   // 推力输出 (-1000 to 1000)
    output reg control_valid                   // 输出有效标志
);

// ========== PID 控制器 ==========
reg signed [15:0] error;
reg signed [31:0] integral;
reg signed [15:0] derivative;
reg signed [15:0] last_error;

// 计算误差
always @(*) begin
    error = altitude_target - altitude_current;
end

// PID 计算
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        integral <= 0;
        last_error <= 0;
        derivative <= 0;
    end else begin
        // 积分（带限幅）
        integral <= integral + error;
        if (integral > 32767) integral <= 32767;
        if (integral < -32768) integral <= -32768;
        
        // 微分
        derivative <= error - last_error;
        last_error <= error;
    end
end

// PID 输出
wire signed [31:0] pid_p = (error * pid_kp) >>> 8;
wire signed [31:0] pid_i = (integral * pid_ki) >>> 16;
wire signed [31:0] pid_d = (derivative * pid_kd) >>> 8;
wire signed [31:0] pid_output_raw = pid_p + pid_i + pid_d;

// 限幅到 -1000~1000
wire signed [15:0] pid_output = (pid_output_raw > 1000) ? 1000 :
                                 (pid_output_raw < -1000) ? -1000 :
                                 pid_output_raw[15:0];

// ========== 神经网络反射（简化版）==========
// 使用简单的非线性映射模拟 NPU 输出
wire signed [15:0] npu_input_error = error >>> 2;      // 缩放
wire signed [15:0] npu_input_vel = velocity_z >>> 4;   // 缩放
wire signed [15:0] npu_input_acc = accel_z >>> 6;      // 缩放

// 简化的 "神经网络" 反射计算
// 实际应该是 NPU 推理，这里用组合逻辑模拟
wire signed [31:0] npu_layer1 = npu_input_error * 16 + npu_input_vel * 8 + npu_input_acc * 4;
wire signed [15:0] npu_activation = (npu_layer1 > 500) ? 500 :
                                     (npu_layer1 < -500) ? -500 :
                                     npu_layer1[15:0];

// 非线性激活（Tanh 近似）
wire signed [15:0] npu_output = (npu_activation * 80) >>> 7;  // 缩放到 -500~500

// ========== 混合输出（Hive-Reflex 核心）==========
// U_final = U_PID * (1 - γ) + U_NN * γ
wire [15:0] inv_compliance = 255 - compliance;

wire signed [31:0] pid_scaled = (pid_output * inv_compliance);
wire signed [31:0] npu_scaled = (npu_output * compliance);
wire signed [31:0] mixed_output = (pid_scaled + npu_scaled) >>> 8;

// 最终输出限幅
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        thrust_output <= 0;
        control_valid <= 0;
    end else begin
        if (mixed_output > 1000) begin
            thrust_output <= 1000;
        end else if (mixed_output < -1000) begin
            thrust_output <= -1000;
        end else begin
            thrust_output <= mixed_output[15:0];
        end
        control_valid <= 1;
    end
end

endmodule
