// 高度反馈芯片测试台
`timescale 1ns/1ps

module altitude_reflex_chip_tb;
    reg clk, rst_n;
    reg signed [15:0] altitude_current, altitude_target;
    reg signed [15:0] velocity_z, accel_z;
    reg [7:0] compliance;
    reg signed [15:0] pid_kp, pid_ki, pid_kd;
    wire signed [15:0] thrust_output;
    wire control_valid;
    
    // 实例化芯片
    altitude_reflex_chip uut (
        .clk(clk),
        .rst_n(rst_n),
        .altitude_current(altitude_current),
        .altitude_target(altitude_target),
        .velocity_z(velocity_z),
        .accel_z(accel_z),
        .compliance(compliance),
        .pid_kp(pid_kp),
        .pid_ki(pid_ki),
        .pid_kd(pid_kd),
        .thrust_output(thrust_output),
        .control_valid(control_valid)
    );
    
    // 时钟生成
    initial clk = 0;
    always #5 clk = ~clk;  // 100MHz
    
    // 测试场景
    initial begin
        $dumpfile("altitude_reflex.vcd");
        $dumpvars(0, altitude_reflex_chip_tb);
        
        // 初始化
        rst_n = 0;
        altitude_current = 0;
        altitude_target = 100;  // 目标100cm
        velocity_z = 0;
        accel_z = 0;
        compliance = 128;  // 50% 柔顺
        pid_kp = 256;      // Kp = 1.0
        pid_ki = 64;       // Ki = 0.25
        pid_kd = 128;      // Kd = 0.5
        
        #20 rst_n = 1;
        
        $display("\n========================================");
        $display("高度反馈芯片测试");
        $display("========================================");
        
        // 测试1: 纯 PID（compliance=0）
        $display("\n[测试1] 纯 PID 控制 (compliance=0)");
        compliance = 0;
        altitude_current = 0;
        altitude_target = 100;
        velocity_z = 0;
        #10;
        $display("  误差=%d, 推力=%d", altitude_target - altitude_current, thrust_output);
        
        // 测试2: 纯反射（compliance=255）
        $display("\n[测试2] 纯反射控制 (compliance=255)");
        compliance = 255;
        altitude_current = 50;
        velocity_z = 10;
        accel_z = 5;
        #10;
        $display("  高度=%d, 速度=%d, 推力=%d", altitude_current, velocity_z, thrust_output);
        
        // 测试3: 混合控制（compliance=128）
        $display("\n[测试3] 混合控制 (compliance=128 = 50%%)");
        compliance = 128;
        altitude_current = 80;
        velocity_z = 5;
        accel_z = 2;
        #10;
        $display("  高度=%d, 目标=%d, 推力=%d", altitude_current, altitude_target, thrust_output);
        
        // 测试4: 动态场景（着陆）
        $display("\n[测试4] 动态着陆场景");
        altitude_target = 0;  // 着陆
        altitude_current = 200;
        velocity_z = -30;  // 下降
        accel_z = -10;
        compliance = 200;  // 高柔顺
        
        repeat (20) begin
            #10;
            // 模拟下降
            altitude_current = altitude_current + (velocity_z / 10);
            velocity_z = velocity_z + (accel_z / 10);
            
            if (altitude_current % 20 == 0) begin
                $display("  t=%0t: 高度=%d, 速度=%d, 推力=%d", 
                         $time, altitude_current, velocity_z, thrust_output);
            end
            
            // 根据推力调整加速度（简化物理）
            accel_z = thrust_output / 20 - 10;  // 重力-10
        end
        
        $display("\n========================================");
        $display("测试完成");
        $display("========================================\n");
        
        #50 $finish;
    end
    
    // 监控异常
    always @(posedge clk) begin
        if (control_valid && (thrust_output > 1000 || thrust_output < -1000)) begin
            $display("ERROR: 推力超限! %d", thrust_output);
        end
    end
    
endmodule
