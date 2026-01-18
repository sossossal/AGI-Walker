"""
物理约束验证系统
检查零件参数是否违反物理规律
"""

import numpy as np
from typing import Dict, List, Tuple


class PhysicsValidator:
    """物理规律验证器"""
    
    def __init__(self):
        self.warnings = []
        self.errors = []
    
    def validate_motor_config(self, motor_params: Dict) -> Tuple[bool, List[str]]:
        """
        验证电机配置是否物理合理
        
        返回: (是否有效, 问题列表)
        """
        issues = []
        
        power = motor_params.get('power', 0)
        voltage = motor_params.get('voltage', 0)
        gear_ratio = motor_params.get('gear_ratio', 1)
        weight = motor_params.get('weight', 0)
        
        # 检查功率密度
        if weight > 0:
            power_density = power / weight  # W/kg
            if power_density > 3000:
                issues.append(f"⚠️ 功率密度过高 ({power_density:.0f} W/kg)，电机会过热")
            elif power_density < 200:
                issues.append(f"ℹ️ 功率密度较低 ({power_density:.0f} W/kg)，可能无法提供足够动力")
        
        # 检查电压
        if voltage > 0:
            current_estimate = power / voltage
            if current_estimate > 100:
                issues.append(f"❌ 电流过大 ({current_estimate:.1f}A)，电机会烧毁！")
            elif current_estimate > 50:
                issues.append(f"⚠️ 电流较大 ({current_estimate:.1f}A)，需要加强散热")
        
        # 检查减速比
        if gear_ratio > 200:
            issues.append(f"⚠️ 减速比过大 ({gear_ratio})，输出速度会很慢")
        elif gear_ratio < 10:
            issues.append(f"⚠️ 减速比过小 ({gear_ratio})，扭矩可能不足")
        
        is_valid = not any('❌' in issue for issue in issues)
        return is_valid, issues
    
    def validate_joint_config(self, joint_params: Dict) -> Tuple[bool, List[str]]:
        """验证关节配置"""
        issues = []
        
        max_torque = joint_params.get('max_torque', 0)
        weight = joint_params.get('weight', 0)
        stiffness = joint_params.get('stiffness', 0)
        
        # 扭矩密度检查
        if weight > 0:
            torque_density = max_torque / weight
            if torque_density > 200:
                issues.append(f"⚠️ 扭矩密度过高 ({torque_density:.0f} Nm/kg)，结构强度可能不足")
        
        # 刚度检查
        if stiffness > 0 and max_torque > 0:
            deflection = max_torque / stiffness  # rad
            deflection_deg = np.degrees(deflection)
            if deflection_deg > 5:
                issues.append(f"⚠️ 在最大扭矩下偏转 {deflection_deg:.2f}°，精度较低")
        
        is_valid = not any('❌' in issue for issue in issues)
        return is_valid, issues
    
    def validate_robot_balance(self, params: Dict) -> Tuple[bool, List[str]]:
        """验证机器人整体平衡性"""
        issues = []
        
        motor_power = params.get('motor_power_multiplier', 1.0)
        mass = params.get('mass_multiplier', 1.0)
        gravity = params.get('gravity', 9.81)
        
        # 功率重量比
        power_to_weight = motor_power / mass
        if power_to_weight < 0.5:
            issues.append(f"❌ 功率重量比过低 ({power_to_weight:.2f})，机器人无法抬起自身重量！")
        elif power_to_weight < 0.8:
            issues.append(f"⚠️ 功率重量比较低 ({power_to_weight:.2f})，动作会很缓慢")
        
        # 重力检查
        if gravity > 15:
            issues.append(f"❌ 重力过大 ({gravity:.1f} m/s²)，机器人会被压垮！")
        elif gravity < 1:
            issues.append(f"ℹ️ 低重力环境 ({gravity:.1f} m/s²)，需要调整控制参数")
        
        is_valid = not any('❌' in issue for issue in issues)
        return is_valid, issues


class PhysicsSimulator:
    """简化的物理模拟器"""
    
    def __init__(self, robot_params: Dict):
        self.params = robot_params
        self.validator = PhysicsValidator()
        
        # 状态
        self.position = 0.0  # 位置 (m)
        self.velocity = 0.0  # 速度 (m/s)
        self.is_fallen = False
        self.time = 0.0
        
    def reset(self):
        """重置状态"""
        self.position = 0.0
        self.velocity = 0.0
        self.is_fallen = False
        self.time = 0.0
    
    def check_stability(self) -> bool:
        """检查稳定性"""
        # 改进的稳定性判断
        motor_power = self.params.get('motor_power_multiplier', 1.0)
        mass = self.params.get('mass_multiplier', 1.0)
        stiffness = self.params.get('joint_stiffness', 1.0)
        damping = self.params.get('joint_damping', 0.5)
        
        # 功率不足 - 更严格
        power_to_weight = motor_power / mass
        if power_to_weight < 0.6:
            return False
        
        # 刚度检查 - 低刚度高概率摔倒
        if stiffness < 0.5:
            # 刚度过低，80%概率摔倒
            return np.random.random() > 0.8
        elif stiffness < 0.8:
            # 刚度较低，50%概率摔倒
            return np.random.random() > 0.5
        elif stiffness > 3.0:
            # 刚度过高，可能震荡
            return np.random.random() > 0.3
        
        # 阻尼过低会不稳定
        if damping < 0.2:
            return np.random.random() > 0.6
        elif damping > 1.2:
            # 阻尼过高也可能有问题
            return np.random.random() > 0.4
        
        return True
    
    def step(self, dt: float = 0.01) -> Dict:
        """
        模拟一步
        
        返回: 状态信息
        """
        if self.is_fallen:
            return {
                'position': self.position,
                'velocity': 0.0,
                'fallen': True,
                'time': self.time
            }
        
        # 检查稳定性
        if not self.check_stability():
            self.is_fallen = True
            return {
                'position': self.position,
                'velocity': 0.0,
                'fallen': True,
                'reason': '参数不当导致失去平衡',
                'time': self.time
            }
        
        # 计算加速度
        motor_power = self.params.get('motor_power_multiplier', 1.0)
        mass = self.params.get('mass_multiplier', 1.0)
        friction = self.params.get('friction', 0.9)
        gravity = self.params.get('gravity', 9.81)
        
        # 修正的运动模型 - 更接近真实机器人
        # 基础推力降低，使速度更合理
        thrust_force = motor_power * 3.0  # N (降低推力系数)
        friction_force = friction * mass * gravity * 1.5  # N (增加摩擦)
        air_resistance = self.velocity * self.velocity * 2.0  # 速度相关阻力
        net_force = thrust_force - friction_force - air_resistance
        
        # 质量对加速度的影响
        acceleration = net_force / (mass * 50.0)  # m/s² (增加质量系数)
        
        # 更新状态
        self.velocity += acceleration * dt
        self.velocity *= 0.95  # 增加阻尼
        self.position += self.velocity * dt
        self.time += dt
        
        return {
            'position': self.position,
            'velocity': self.velocity,
            'fallen': False,
            'time': self.time
        }
    
    def simulate_forward(self, target_distance: float = 1.0, max_time: float = 10.0) -> Dict:
        """
        模拟前进指定距离
        
        参数:
            target_distance: 目标距离 (m)
            max_time: 最大时间 (s)
        
        返回:
            模拟结果
        """
        self.reset()
        
        # 验证参数
        is_valid, issues = self.validator.validate_robot_balance(self.params)
        
        if not is_valid:
            return {
                'success': False,
                'reason': '参数不合理',
                'issues': issues,
                'distance_traveled': 0.0,
                'time_taken': 0.0
            }
        
        # 运行模拟
        trajectory = []
        
        while self.time < max_time:
            state = self.step()
            trajectory.append(state.copy())
            
            # 摔倒
            if state['fallen']:
                return {
                    'success': False,
                    'reason': '机器人摔倒',
                    'issues': [state.get('reason', '未知原因')],
                    'distance_traveled': self.position,
                    'time_taken': self.time,
                    'trajectory': trajectory
                }
            
            # 达到目标
            if self.position >= target_distance:
                return {
                    'success': True,
                    'distance_traveled': self.position,
                    'time_taken': self.time,
                    'avg_speed': self.position / self.time,
                    'trajectory': trajectory,
                    'warnings': issues
                }
        
        # 超时
        return {
            'success': False,
            'reason': '超时未达到目标',
            'distance_traveled': self.position,
            'time_taken': self.time,
            'trajectory': trajectory
        }


if __name__ == "__main__":
    print("物理验证系统加载完成")
