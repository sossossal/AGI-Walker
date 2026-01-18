"""
PID控制器 - Python实现
用于参数调优和仿真测试
"""

import time
from typing import Optional, Dict


class PIDController:
    """PID控制器类"""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        """
        初始化PID控制器
        
        Args:
            kp: 比例增益
            ki: 积分增益
            kd: 微分增益
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # 内部状态
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        
        # 限制
        self.output_min = float('-inf')
        self.output_max = float('inf')
        self.integral_min = float('-inf')
        self.integral_max = float('inf')
        
        # 统计
        self.iteration_count = 0
    
    def compute(self, setpoint: float, measured_value: float, 
                current_time: Optional[float] = None) -> float:
        """
        计算PID输出
        
        Args:
            setpoint: 目标值
            measured_value: 测量值
            current_time: 当前时间（可选）
        
        Returns:
            控制输出
        """
        if current_time is None:
            current_time = time.time()
        
        # 计算时间增量
        dt = current_time - self.last_time
        if dt <= 0:
            dt = 0.001
        
        # 计算误差
        error = setpoint - measured_value
        
        # 比例项
        p_term = self.kp * error
        
        # 积分项
        self.integral += error * dt
        self.integral = max(self.integral_min, 
                           min(self.integral, self.integral_max))
        i_term = self.ki * self.integral
        
        # 微分项
        derivative = (error - self.last_error) / dt
        d_term = self.kd * derivative
        
        # 总输出
        output = p_term + i_term + d_term
        output = max(self.output_min, min(output, self.output_max))
        
        # 更新状态
        self.last_error = error
        self.last_time = current_time
        self.iteration_count += 1
        
        return output
    
    def compute_from_error(self, error: float, dt: float) -> float:
        """
        直接从误差计算输出
        
        Args:
            error: 误差值
            dt: 时间增量
        
        Returns:
            控制输出
        """
        # 比例项
        p_term = self.kp * error
        
        # 积分项
        self.integral += error * dt
        self.integral = max(self.integral_min, 
                           min(self.integral, self.integral_max))
        i_term = self.ki * self.integral
        
        # 微分项
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        d_term = self.kd * derivative
        
        # 总输出
        output = p_term + i_term + d_term
        output = max(self.output_min, min(output, self.output_max))
        
        self.last_error = error
        self.iteration_count += 1
        
        return output
    
    def set_limits(self, output_min: float, output_max: float):
        """设置输出限制"""
        self.output_min = output_min
        self.output_max = output_max
    
    def set_integral_limits(self, integral_min: float, integral_max: float):
        """设置积分限制"""
        self.integral_min = integral_min
        self.integral_max = integral_max
    
    def reset(self):
        """重置PID状态"""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = time.time()
        self.iteration_count = 0
    
    def set_tunings(self, kp: float, ki: float, kd: float):
        """动态调整PID参数"""
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def get_terms(self) -> Dict[str, float]:
        """获取PID各项的值（调试用）"""
        return {
            'p_term': self.kp * self.last_error,
            'i_term': self.ki * self.integral,
            'd_term': self.kd * (self.last_error - 0),  # 近似
            'integral': self.integral,
            'error': self.last_error
        }
    
    def print_status(self):
        """打印PID状态"""
        terms = self.get_terms()
        print(f"PID状态 [Kp={self.kp:.2f}, Ki={self.ki:.2f}, Kd={self.kd:.2f}]")
        print(f"  误差: {terms['error']:.3f}")
        print(f"  P项: {terms['p_term']:.3f}")
        print(f"  I项: {terms['i_term']:.3f} (积分={terms['integral']:.3f})")
        print(f"  D项: {terms['d_term']:.3f}")
        print(f"  迭代: {self.iteration_count}")


class BalanceController:
    """平衡控制器 - 使用双PID控制Roll和Pitch"""
    
    def __init__(self, 
                 roll_pid_params=(8.0, 0.5, 3.0),
                 pitch_pid_params=(8.0, 0.5, 3.0)):
        """
        初始化平衡控制器
        
        Args:
            roll_pid_params: (Kp, Ki, Kd) for Roll
            pitch_pid_params: (Kp, Ki, Kd) for Pitch
        """
        self.roll_pid = PIDController(*roll_pid_params)
        self.pitch_pid = PIDController(*pitch_pid_params)
        
        # 设置限制
        self.roll_pid.set_limits(-90, 90)
        self.roll_pid.set_integral_limits(-20, 20)
        
        self.pitch_pid.set_limits(-90, 90)
        self.pitch_pid.set_integral_limits(-20, 20)
        
        # 目标姿态
        self.target_roll = 0.0
        self.target_pitch = 0.0
    
    def compute_balance(self, sensor_data: Dict, dt: float) -> Dict:
        """
        计算平衡控制指令
        
        Args:
            sensor_data: 传感器数据字典
            dt: 时间增量
        
        Returns:
            电机指令字典
        """
        # 获取当前姿态
        orient = sensor_data['sensors']['imu']['orient']
        current_roll = orient[0]
        current_pitch = orient[1]
        
        # 计算误差
        roll_error = self.target_roll - current_roll
        pitch_error = self.target_pitch - current_pitch
        
        # PID计算
        roll_correction = self.roll_pid.compute_from_error(roll_error, dt)
        pitch_correction = self.pitch_pid.compute_from_error(pitch_error, dt)
        
        # 转换为电机指令
        return {
            "motors": {
                "hip_left": pitch_correction + roll_correction,
                "hip_right": pitch_correction - roll_correction
            }
        }
    
    def set_target_posture(self, roll: float, pitch: float):
        """设置目标姿态"""
        self.target_roll = roll
        self.target_pitch = pitch
    
    def reset(self):
        """重置控制器"""
        self.roll_pid.reset()
        self.pitch_pid.reset()
    
    def tune_pid(self, axis: str, kp: float, ki: float, kd: float):
        """动态调整PID参数"""
        if axis.lower() == 'roll':
            self.roll_pid.set_tunings(kp, ki, kd)
        elif axis.lower() == 'pitch':
            self.pitch_pid.set_tunings(kp, ki, kd)


# 使用示例
if __name__ == "__main__":
    # 创建PID控制器
    pid = PIDController(kp=2.0, ki=0.1, kd=0.5)
    pid.set_limits(-100, 100)
    
    print("PID控制器测试\n")
    
    # 模拟控制过程
    setpoint = 10.0  # 目标值
    measured = 0.0   # 初始测量值
    
    for i in range(20):
        # 计算控制输出
        output = pid.compute(setpoint, measured)
        
        # 模拟系统响应（简化的一阶系统）
        measured += output * 0.1
        
        # 打印状态
        if i % 5 == 0:
            print(f"步骤 {i}: 测量值={measured:.2f}, 输出={output:.2f}")
    
    print(f"\n最终测量值: {measured:.2f}")
    print(f"目标值: {setpoint:.2f}")
    print(f"误差: {abs(setpoint - measured):.2f}")
