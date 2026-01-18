// enhanced_motor_joint.cpp
// 增强型电机关节实现 - 真实的电机物理模拟

#include "enhanced_motor_joint.h"
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/utility_functions.hpp>
#include <cmath>
#include <algorithm>

using namespace godot;

void EnhancedMotorJoint::_bind_methods() {
    // ========== 绑定设置方法 ==========
    ClassDB::bind_method(D_METHOD("set_motor_specs", "torque", "speed", "inertia"), 
                        &EnhancedMotorJoint::set_motor_specs);
    ClassDB::bind_method(D_METHOD("set_friction_params", "static_f", "dynamic_f", "viscous"), 
                        &EnhancedMotorJoint::set_friction_params);
    ClassDB::bind_method(D_METHOD("set_thermal_params", "resistance", "time_const", "max_temp"), 
                        &EnhancedMotorJoint::set_thermal_params);
    ClassDB::bind_method(D_METHOD("set_electrical_params", "current", "voltage", "kt"), 
                        &EnhancedMotorJoint::set_electrical_params);
    
    ClassDB::bind_method(D_METHOD("set_stall_torque", "torque"), &EnhancedMotorJoint::set_stall_torque);
    ClassDB::bind_method(D_METHOD("set_no_load_speed", "speed"), &EnhancedMotorJoint::set_no_load_speed);
    ClassDB::bind_method(D_METHOD("set_target_velocity_rad", "velocity"), &EnhancedMotorJoint::set_target_velocity_rad);
    ClassDB::bind_method(D_METHOD("set_target_velocity_rpm", "rpm"), &EnhancedMotorJoint::set_target_velocity_rpm);
    
    // ========== 绑定获取方法 ==========
    ClassDB::bind_method(D_METHOD("get_stall_torque"), &EnhancedMotorJoint::get_stall_torque);
    ClassDB::bind_method(D_METHOD("get_no_load_speed"), &EnhancedMotorJoint::get_no_load_speed);
    ClassDB::bind_method(D_METHOD("get_temperature"), &EnhancedMotorJoint::get_temperature);
    ClassDB::bind_method(D_METHOD("get_current_draw"), &EnhancedMotorJoint::get_current_draw);
    ClassDB::bind_method(D_METHOD("get_current_torque"), &EnhancedMotorJoint::get_current_torque);
    ClassDB::bind_method(D_METHOD("get_current_velocity"), &EnhancedMotorJoint::get_current_velocity);
    ClassDB::bind_method(D_METHOD("get_efficiency"), &EnhancedMotorJoint::get_efficiency);
    ClassDB::bind_method(D_METHOD("get_power_consumption"), &EnhancedMotorJoint::get_power_consumption);
    
    // ========== 绑定状态查询 ==========
    ClassDB::bind_method(D_METHOD("is_overload ed"), &EnhancedMotorJoint::is_overloaded);
    ClassDB::bind_method(D_METHOD("is_overheating"), &EnhancedMotorJoint::is_overheating);
    ClassDB::bind_method(D_METHOD("get_diagnostic_info"), &EnhancedMotorJoint::get_diagnostic_info);
    
    // ========== 绑定控制开关 ==========
    ClassDB::bind_method(D_METHOD("set_enable_speed_torque_curve", "enable"), 
                        &EnhancedMotorJoint::set_enable_speed_torque_curve);
    ClassDB::bind_method(D_METHOD("set_enable_friction_model", "enable"), 
                        &EnhancedMotorJoint::set_enable_friction_model);
    ClassDB::bind_method(D_METHOD("set_enable_thermal_model", "enable"), 
                        &EnhancedMotorJoint::set_enable_thermal_model);
    
    // ========== 绑定属性 ==========
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stall_torque", PROPERTY_HINT_RANGE, "0,100,0.01,suffix:N·m"), 
                 "set_stall_torque", "get_stall_torque");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "no_load_speed", PROPERTY_HINT_RANGE, "0,100,0.1,suffix:rad/s"), 
                 "set_no_load_speed", "get_no_load_speed");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature", PROPERTY_HINT_NONE, "suffix:°C"), 
                 "", "get_temperature");
    
    ADD_GROUP("Motor Specs", "");
    ADD_GROUP("Friction", "friction_");
    ADD_GROUP("Thermal", "thermal_");
    ADD_GROUP("Electrical", "");
    ADD_GROUP("Status", "");
}

EnhancedMotorJoint::EnhancedMotorJoint() {
    // 构造函数
}

EnhancedMotorJoint::~EnhancedMotorJoint() {
    // 析构函数
}

void EnhancedMotorJoint::_ready() {
    HingeJoint3D::_ready();
    
    // 默认启用 Godot 内置电机
    set_flag(FLAG_ENABLE_MOTOR, true);
    
    UtilityFunctions::print("🔩 EnhancedMotorJoint ready: ", get_name());
}

void EnhancedMotorJoint::_physics_process(double delta) {
    // 调用父类
    HingeJoint3D::_physics_process(delta);
    
    // 获取当前关节角速度
    current_velocity = get_joint_angular_velocity();
    
    // 计算实际扭矩（核心物理模型）
    float actual_torque = calculate_actual_torque(target_velocity, delta);
    current_torque = actual_torque;
    
    // 应用扭矩到 Godot 内置电机
    // 注意：PARAM_MOTOR_MAX_IMPULSE 单位是 N·m，但需要乘以 delta 作为冲量
    set_param(PARAM_MOTOR_MAX_IMPULSE, std::abs(actual_torque));
    set_param(PARAM_MOTOR_TARGET_VELOCITY, target_velocity);
    
    // 计算功率损耗
    float power_loss = std::abs(actual_torque * current_velocity);
    
    // 更新温度（如果启用）
    if (enable_thermal_model) {
        update_temperature(power_loss, delta);
    }
    
    // 更新能耗统计
    total_energy_consumed += get_power_consumption() * delta;
    
    // 检查过载和过热
    if (is_overloaded()) {
        UtilityFunctions::push_warning("⚠️ Motor overloaded: ", get_name(), " Current: ", current_current, "A");
    }
    if (is_overheating()) {
        UtilityFunctions::push_warning("🔥 Motor overheating: ", get_name(), " Temp: ", temperature, "°C");
    }
}

// ========== 参数设置实现 ==========

void EnhancedMotorJoint::set_motor_specs(float torque, float speed, float inertia) {
    stall_torque = torque;
    no_load_speed = speed;
    rotor_inertia = inertia;
    
    UtilityFunctions::print("🔧 Motor specs updated: T=", torque, " N·m, ω=", speed, " rad/s");
}

void EnhancedMotorJoint::set_friction_params(float static_f, float dynamic_f, float viscous) {
    friction_static = static_f;
    friction_dynamic = dynamic_f;
    viscous_damping = viscous;
}

void EnhancedMotorJoint::set_thermal_params(float resistance, float time_const, float max_temp) {
    thermal_resistance = resistance;
    thermal_time_constant = time_const;
    max_winding_temp = max_temp;
}

void EnhancedMotorJoint::set_electrical_params(float current, float volt, float kt) {
    max_current = current;
    voltage = volt;
    motor_constant = kt;
}

void EnhancedMotorJoint::set_target_velocity_rad(float velocity) {
    target_velocity = velocity;
}

void EnhancedMotorJoint::set_target_velocity_rpm(float rpm) {
    // RPM → rad/s
    target_velocity = rpm * (2.0f * Math_PI / 60.0f);
}

// ========== 核心物理模型实现 ==========

float EnhancedMotorJoint::calculate_actual_torque(float target_vel, double delta) {
    // 1. 速度-扭矩曲线（如果启用）
    float available_torque = stall_torque;
    
    if (enable_speed_torque_curve) {
        available_torque = calculate_speed_torque_curve(current_velocity);
    }
    
    // 2. 计算期望扭矩（基于速度误差的简单P控制）
    float velocity_error = target_vel - current_velocity;
    float desired_torque = std::copysign(available_torque, velocity_error);
    
    // 3. 减去摩擦力（如果启用）
    float friction_torque = 0.0f;
    if (enable_friction_model) {
        friction_torque = calculate_friction_torque(current_velocity, desired_torque);
    }
    
    // 4. 热降额（如果启用）
    float thermal_factor = 1.0f;
    if (enable_thermal_model) {
        thermal_factor = get_thermal_derating_factor();
    }
    
    // 5. 净扭矩
    float net_torque = (available_torque - std::abs(friction_torque)) * thermal_factor;
    net_torque = std::copysign(net_torque, desired_torque);
    
    // 限制到可用范围
    net_torque = std::clamp(net_torque, -available_torque, available_torque);
    
    // 6. 计算电流
    current_current = calculate_current(net_torque);
    
    // 7. 电流限制（如果启用）
    if (enable_current_limit && current_current > max_current) {
        float current_ratio = max_current / current_current;
        net_torque *= current_ratio;
        current_current = max_current;
    }
    
    return net_torque;
}

float EnhancedMotorJoint::calculate_speed_torque_curve(float velocity) {
    /**
     * 理想直流电机的线性速度-扭矩特性：
     * T(ω) = T_stall × (1 - ω / ω_no_load)
     * 
     * 当 ω = 0（堵转）：T = T_stall
     * 当 ω = ω_no_load（空载）：T = 0
     */
    
    if (no_load_speed <= 0.0f) {
        return stall_torque;  // 避免除零
    }
    
    float speed_ratio = std::abs(velocity) / no_load_speed;
    speed_ratio = std::clamp(speed_ratio, 0.0f, 1.0f);
    
    float torque = stall_torque * (1.0f - speed_ratio);
    
    return std::max(0.0f, torque);
}

float EnhancedMotorJoint::calculate_friction_torque(float velocity, float applied_torque) {
    /**
     * 库仑摩擦模型 + 粘性阻尼：
     * - 静止时：静摩擦力抵抗运动，但不超过最大静摩擦
     * - 运动时：动摩擦力 + 粘性阻尼（与速度成正比）
     */
    
    float friction = 0.0f;
    
    const float STATIC_THRESHOLD = 0.01f;  // rad/s，认为是静止的阈值
    
    if (std::abs(velocity) < STATIC_THRESHOLD) {
        // 静摩擦：阻止运动，但不超过最大静摩擦力
        friction = std::clamp(
            -applied_torque,
            -friction_static,
            friction_static
        );
    } else {
        // 动摩擦：与运动方向相反
        friction = -std::copysign(friction_dynamic, velocity);
        
        // 粘性阻尼：与速度成正比
        friction -= viscous_damping * velocity;
    }
    
    return friction;
}

void EnhancedMotorJoint::update_temperature(float power_loss, double delta) {
    /**
     * 一阶热模型（类似 RC 电路）：
     * dT/dt = (P × R_th - (T - T_ambient)) / τ
     * 
     * 其中：
     * - P: 功率损耗 (W)
     * - R_th: 热阻 (K/W)
     * - T: 当前温度 (°C)
     * - T_ambient: 环境温度 (°C)
     * - τ: 热时间常数 (s)
     */
    
    // 热量输入（功率损耗转换为温升）
    float heat_in = power_loss * thermal_resistance;
    
    // 热量散失（温差驱动）
    float heat_out = (temperature - ambient_temp) / thermal_time_constant;
    
    // 温度变化率
    float dT_dt = heat_in - heat_out;
    
    // 更新温度
    temperature += dT_dt * static_cast<float>(delta);
    
    // 限制温度范围（不能低于环境温度）
    temperature = std::max(temperature, ambient_temp);
    
    // 更新峰值温度
    peak_temperature = std::max(peak_temperature, temperature);
}

float EnhancedMotorJoint::get_thermal_derating_factor() const {
    /**
     * 热降额曲线：
     * - T < 80°C: 无降额，因子 = 1.0
     * - 80°C < T < max_temp: 线性降额
     * - T > max_temp: 最小因子 0.5（防止完全停止）
     */
    
    const float DERATING_START_TEMP = 80.0f;  // °C
    
    if (temperature < DERATING_START_TEMP) {
        return 1.0f;  // 无降额
    }
    
    if (temperature > max_winding_temp) {
        return 0.5f;  // 严重过热，降至 50%
    }
    
    // 线性降额
    float temp_range = max_winding_temp - DERATING_START_TEMP;
    float temp_excess = temperature - DERATING_START_TEMP;
    float factor = 1.0f - (temp_excess / temp_range) * 0.5f;
    
    return std::max(0.5f, factor);
}

float EnhancedMotorJoint::calculate_current(float torque) {
    /**
     * 电流计算（基于电机扭矩常数）：
     * I = T / K_t
     * 
     * 其中：
     * - T: 扭矩 (N·m)
     * - K_t: 电机扭矩常数 (N·m/A)
     */
    
    if (motor_constant <= 0.0f) {
        return 0.0f;  // 避免除零
    }
    
    return std::abs(torque) / motor_constant;
}

float EnhancedMotorJoint::get_joint_angular_velocity() const {
    /**
     * 获取关节的当前角速度
     * 注意：Godot 的 HingeJoint3D 没有直接提供速度，需要通过其他方式获取
     * 这里使用一个简化的方法
     */
    
    // 尝试从连接的刚体获取角速度
    // 这是一个简化实现，实际可能需要更精确的计算
    
    // 暂时返回设定的目标速度作为近似（待优化）
    return target_velocity;
}

float EnhancedMotorJoint::get_efficiency() const {
    /**
     * 效率计算：
     * η = P_out / P_in = (T × ω) / (V × I)
     */
    
    float power_out = current_torque * current_velocity;
    float power_in = voltage * current_current;
    
    if (power_in <= 0.0f) {
        return 0.0f;
    }
    
    float efficiency = power_out / power_in;
    return std::clamp(efficiency, 0.0f, 1.0f);
}

void EnhancedMotorJoint::reset_statistics() {
    total_energy_consumed = 0.0f;
    peak_temperature = temperature;
}

String EnhancedMotorJoint::get_diagnostic_info() const {
    String info = "=== Enhanced Motor Joint Diagnostics ===\n";
    info += "Name: " + get_name() + "\n";
    info += "--- Motor Specs ---\n";
    info += "Stall Torque: " + String::num(stall_torque, 3) + " N·m\n";
    info += "No-Load Speed: " + String::num(no_load_speed, 2) + " rad/s\n";
    info += "--- Current Status ---\n";
    info += "Velocity: " + String::num(current_velocity, 3) + " rad/s\n";
    info += "Torque: " + String::num(current_torque, 3) + " N·m\n";
    info += "Current: " + String::num(current_current, 3) + " A\n";
    info += "Temperature: " + String::num(temperature, 1) + " °C\n";
    info += "Efficiency: " + String::num(get_efficiency() * 100, 1) + " %\n";
    info += "Power: " + String::num(get_power_consumption(), 2) + " W\n";
    info += "--- Statistics ---\n";
    info += "Energy Consumed: " + String::num(total_energy_consumed, 1) + " J\n";
    info += "Peak Temp: " + String::num(peak_temperature, 1) + " °C\n";
    info += "Overloaded: " + String(is_overloaded() ? "YES" : "no") + "\n";
    info += "Overheating: " + String(is_overheating() ? "YES" : "no") + "\n";
    info += "======================================";
    
    return info;
}
