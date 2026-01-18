// enhanced_motor_joint.h
// 增强型电机关节 - 实现真实的电机物理特性
// 包括：速度-扭矩曲线、摩擦模型、热模型

#ifndef ENHANCED_MOTOR_JOINT_H
#define ENHANCED_MOTOR_JOINT_H

#include <godot_cpp/classes/hinge_joint3d.hpp>
#include <godot_cpp/classes/rigid_body3d.hpp>

namespace godot {

class EnhancedMotorJoint : public HingeJoint3D {
    GDCLASS(EnhancedMotorJoint, HingeJoint3D)

private:
    // ========== 电机规格参数 ==========
    float stall_torque = 1.0f;              // 堵转扭矩 (N·m)
    float no_load_speed = 5.236f;           // 空载速度 (rad/s)，默认 50 RPM
    float rotor_inertia = 1.0e-6f;          // 转子惯量 (kg·m²)
    float gear_ratio = 1.0f;                // 减速比
    
    // ========== 摩擦力参数 ==========
    float friction_static = 0.01f;          // 静摩擦力矩 (N·m)
    float friction_dynamic = 0.005f;        // 动摩擦力矩 (N·m)
    float viscous_damping = 0.001f;         // 粘性阻尼系数 (N·m·s/rad)
    
    // ========== 热模型参数 ==========
    float temperature = 25.0f;              // 当前温度 (°C)
    float thermal_resistance = 20.0f;       // 热阻 (K/W)
    float thermal_time_constant = 1000.0f;  // 热时间常数 (s)
    float max_winding_temp = 125.0f;        // 最大绕组温度 (°C)
    float ambient_temp = 25.0f;             // 环境温度 (°C)
    
    // ========== 电气参数 ==========
    float max_current = 1.0f;               // 最大电流 (A)
    float voltage = 12.0f;                  // 工作电压 (V)
    float motor_constant = 0.01f;           // 电机扭矩常数 Kt (N·m/A)
    float back_emf_constant = 0.01f;        // 反电动势常数 Ke (V/(rad/s))
    float winding_resistance = 8.0f;        // 绕组电阻 (Ω)
    
    // ========== 运行状态 ==========
    float current_velocity = 0.0f;          // 当前角速度 (rad/s)
    float current_current = 0.0f;           // 当前电流 (A)
    float current_torque = 0.0f;            // 当前输出扭矩 (N·m)
    float target_velocity = 0.0f;           // 目标速度 (rad/s)
    
    // ========== 控制标志 ==========
    bool enable_speed_torque_curve = true;  // 启用速度-扭矩曲线
    bool enable_friction_model = true;      // 启用摩擦模型
    bool enable_thermal_model = true;       // 启用热模型
    bool enable_current_limit = true;       // 启用电流限制
    
    // ========== 性能统计 ==========
    float total_energy_consumed = 0.0f;     // 总能耗 (J)
    float peak_temperature = 25.0f;         // 峰值温度 (°C)

protected:
    static void _bind_methods();

public:
    EnhancedMotorJoint();
    ~EnhancedMotorJoint();
    
    // Godot 生命周期
    void _ready() override;
    void _physics_process(double delta) override;
    
    // ========== 参数设置方法 ==========
    
    // 设置电机规格（从零件库数据）
    void set_motor_specs(float torque, float speed, float inertia);
    void set_stall_torque(float torque) { stall_torque = torque; }
    void set_no_load_speed(float speed) { no_load_speed = speed; }
    void set_rotor_inertia(float inertia) { rotor_inertia = inertia; }
    void set_gear_ratio(float ratio) { gear_ratio = ratio; }
    
    // 设置摩擦参数
    void set_friction_params(float static_f, float dynamic_f, float viscous);
    void set_friction_static(float friction) { friction_static = friction; }
    void set_friction_dynamic(float friction) { friction_dynamic = friction; }
    void set_viscous_damping(float damping) { viscous_damping = damping; }
    
    // 设置热参数
    void set_thermal_params(float resistance, float time_const, float max_temp);
    void set_thermal_resistance(float resistance) { thermal_resistance = resistance; }
    void set_thermal_time_constant(float time_const) { thermal_time_constant = time_const; }
    void set_max_winding_temp(float temp) { max_winding_temp = temp; }
    void set_ambient_temp(float temp) { ambient_temp = temp; }
    
    // 设置电气参数
    void set_electrical_params(float current, float volt, float kt);
    void set_max_current(float current) { max_current = current; }
    void set_voltage(float volt) { voltage = volt; }
    void set_motor_constant(float kt) { motor_constant = kt; }
    
    // 设置目标速度
    void set_target_velocity_rad(float velocity);
    void set_target_velocity_rpm(float rpm);
    
    // ========== 参数获取方法 ==========
    
    float get_stall_torque() const { return stall_torque; }
    float get_no_load_speed() const { return no_load_speed; }
    float get_rotor_inertia() const { return rotor_inertia; }
    float get_temperature() const { return temperature; }
    float get_current_draw() const { return current_current; }
    float get_current_torque() const { return current_torque; }
    float get_current_velocity() const { return current_velocity; }
    float get_total_energy_consumed() const { return total_energy_consumed; }
    
    // ========== 状态查询 ==========
    
    bool is_overloaded() const { return current_current > max_current * 1.2f; }
    bool is_overheating() const { return temperature > max_winding_temp * 0.9f; }
    float get_efficiency() const;
    float get_power_consumption() const { return voltage * current_current; }
    
    // ========== 控制开关 ==========
    
    void set_enable_speed_torque_curve(bool enable) { enable_speed_torque_curve = enable; }
    void set_enable_friction_model(bool enable) { enable_friction_model = enable; }
    void set_enable_thermal_model(bool enable) { enable_thermal_model = enable; }
    void set_enable_current_limit(bool enable) { enable_current_limit = enable; }
    
    bool get_enable_speed_torque_curve() const { return enable_speed_torque_curve; }
    bool get_enable_friction_model() const { return enable_friction_model; }
    bool get_enable_thermal_model() const { return enable_thermal_model; }
    bool get_enable_current_limit() const { return enable_current_limit; }
    
    // ========== 重置和诊断 ==========
    
    void reset_statistics();
    void reset_temperature() { temperature = ambient_temp; }
    String get_diagnostic_info() const;

private:
    // ========== 内部计算方法 ==========
    
    // 计算实际输出扭矩（核心物理模型）
    float calculate_actual_torque(float target_vel, double delta);
    
    // 速度-扭矩曲线
    float calculate_speed_torque_curve(float velocity);
    
    // 摩擦力计算
    float calculate_friction_torque(float velocity, float applied_torque);
    
    // 温度更新
    void update_temperature(float power_loss, double delta);
    
    // 热降额
    float get_thermal_derating_factor() const;
    
    // 电流计算
    float calculate_current(float torque);
    
    // 获取关节当前角速度
    float get_joint_angular_velocity() const;
};

} // namespace godot

#endif // ENHANCED_MOTOR_JOINT_H
