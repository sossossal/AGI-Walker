"""
环境平衡系统
模拟各种环境因素对机器人平衡的影响
"""

import numpy as np
from typing import Dict, List, Tuple
from python_api.physics_validator import PhysicsSimulator


class EnvironmentCondition:
    """环境条件类"""
    
    def __init__(self, name: str):
        self.name = name
        self.ground_slope = 0.0      # 地面倾角 (度)
        self.wind_force = 0.0         # 风力 (N)
        self.wind_direction = 0.0     # 风向 (度, 0=正前方)
        self.friction_coef = 1.0      # 摩擦系数倍数
        self.disturbance = 0.0        # 随机扰动强度
        self.obstacle_density = 0.0   # 障碍物密度
    
    def apply_to_simulator(self, sim: PhysicsSimulator):
        """将环境条件应用到模拟器"""
        # 地面倾斜影响重力
        if self.ground_slope != 0:
            # 倾斜角度转换为重力分量
            slope_rad = np.radians(self.ground_slope)
            sim.params['gravity'] = 9.81 * np.cos(slope_rad)
            # 添加下坡/上坡的额外力
            sim.slope_force = 9.81 * np.sin(slope_rad) * sim.params.get('mass_multiplier', 1.0)
        
        # 摩擦系数调整
        sim.params['friction'] *= self.friction_coef
        
        # 风力和扰动存储
        sim.wind_force = self.wind_force
        sim.disturbance = self.disturbance
    
    def __str__(self):
        conditions = []
        if self.ground_slope != 0:
            conditions.append(f"地面倾斜{self.ground_slope:+.1f}°")
        if self.wind_force != 0:
            conditions.append(f"风力{self.wind_force:.1f}N")
        if self.friction_coef != 1.0:
            conditions.append(f"摩擦×{self.friction_coef:.1f}")
        if self.disturbance != 0:
            conditions.append(f"扰动±{self.disturbance:.1f}")
        
        return f"{self.name}: " + ", ".join(conditions) if conditions else self.name


class BalanceController:
    """平衡控制器"""
    
    def __init__(self, robot_params: Dict):
        self.robot_params = robot_params
        
        # PID控制参数
        self.kp = robot_params.get('joint_stiffness', 1.0) * 5.0
        self.kd = robot_params.get('joint_damping', 0.5) * 3.0
        self.ki = 0.1
        
        # 状态
        self.error_integral = 0.0
        self.last_error = 0.0
    
    def compute_balance_adjustment(self, tilt_angle: float, dt: float = 0.01) -> float:
        """
        计算平衡调整力
        
        参数:
            tilt_angle: 当前倾斜角 (度)
            dt: 时间步长
        
        返回:
            调整力
        """
        # PID控制
        error = -tilt_angle  # 目标是0度
        self.error_integral += error * dt
        error_derivative = (error - self.last_error) / dt
        
        adjustment = (self.kp * error + 
                     self.ki * self.error_integral + 
                     self.kd * error_derivative)
        
        self.last_error = error
        
        return adjustment


class EnvironmentBalanceSimulator(PhysicsSimulator):
    """带环境因素的平衡模拟器"""
    
    def __init__(self, robot_params: Dict, environment: EnvironmentCondition = None):
        super().__init__(robot_params)
        
        self.environment = environment or EnvironmentCondition("标准环境")
        self.environment.apply_to_simulator(self)
        
        # 平衡控制器
        self.balance_controller = BalanceController(robot_params)
        
        # 额外状态
        self.tilt_angle = 0.0  # 机器人倾斜角度
        self.slope_force = 0.0  # 坡度力
        self.wind_force = 0.0   # 风力
        self.disturbance = 0.0  # 扰动
    
    def reset(self):
        """重置状态"""
        super().reset()
        self.tilt_angle = 0.0
        self.balance_controller.error_integral = 0.0
        self.balance_controller.last_error = 0.0
    
    def step(self, dt: float = 0.01) -> Dict:
        """
        模拟一步（带环境和平衡控制）
        """
        if self.is_fallen:
            return {
                'position': self.position,
                'velocity': 0.0,
                'tilt_angle': self.tilt_angle,
                'fallen': True,
                'time': self.time
            }
        
        # 计算环境影响下的倾斜
        motor_power = self.params.get('motor_power_multiplier', 1.0)
        mass = self.params.get('mass_multiplier', 1.0)
        stiffness = self.params.get('joint_stiffness', 1.0)
        
        # 环境扰动
        if self.disturbance > 0:
            random_disturbance = np.random.uniform(-self.disturbance, self.disturbance)
        else:
            random_disturbance = 0
        
        # 计算倾斜力矩
        tilt_torque = (self.slope_force + self.wind_force + random_disturbance) * 0.1
        
        # 平衡力矩（由刚度和控制器提供）
        balance_adjustment = self.balance_controller.compute_balance_adjustment(self.tilt_angle, dt)
        balance_torque = stiffness * balance_adjustment
        
        # 更新倾斜角
        net_torque = tilt_torque - balance_torque
        angular_acceleration = net_torque / (mass * 0.5)  # 简化的转动惯量
        self.tilt_angle += angular_acceleration * dt
        
        # 检查是否倾倒（超过30度）
        if abs(self.tilt_angle) > 30:
            self.is_fallen = True
            return {
                'position': self.position,
                'velocity': 0.0,
                'tilt_angle': self.tilt_angle,
                'fallen': True,
                'reason': f'倾斜角过大 ({self.tilt_angle:.1f}°)',
                'time': self.time
            }
        
        # 检查其他稳定性
        if not self.check_stability():
            self.is_fallen = True
            return {
                'position': self.position,
                'velocity': 0.0,
                'tilt_angle': self.tilt_angle,
                'fallen': True,
                'reason': '参数不当导致失去平衡',
                'time': self.time
            }
        
        # 计算运动（调用父类方法）
        base_result = super().step(dt)
        
        # 添加倾斜角信息
        base_result['tilt_angle'] = self.tilt_angle
        base_result['balance_torque'] = balance_torque
        
        return base_result


# 预定义环境
class Environments:
    """预定义环境库"""
    
    @staticmethod
    def flat_ground():
        """平地"""
        env = EnvironmentCondition("平地")
        return env
    
    @staticmethod
    def uphill_5deg():
        """5度上坡"""
        env = EnvironmentCondition("5度上坡")
        env.ground_slope = 5.0
        return env
    
    @staticmethod
    def downhill_5deg():
        """5度下坡"""
        env = EnvironmentCondition("5度下坡")
        env.ground_slope = -5.0
        return env
    
    @staticmethod
    def uphill_10deg():
        """10度上坡"""
        env = EnvironmentCondition("10度陡坡")
        env.ground_slope = 10.0
        return env
    
    @staticmethod
    def windy():
        """大风"""
        env = EnvironmentCondition("大风环境")
        env.wind_force = 5.0
        return env
    
    @staticmethod
    def icy_ground():
        """冰面（低摩擦）"""
        env = EnvironmentCondition("冰面")
        env.friction_coef = 0.3
        return env
    
    @staticmethod
    def rough_terrain():
        """崎岖地形"""
        env = EnvironmentCondition("崎岖地形")
        env.friction_coef = 1.5
        env.disturbance = 2.0
        return env
    
    @staticmethod
    def extreme_conditions():
        """极端条件"""
        env = EnvironmentCondition("极端环境")
        env.ground_slope = 8.0
        env.wind_force = 3.0
        env.friction_coef = 0.6
        env.disturbance = 1.5
        return env


if __name__ == "__main__":
    print("环境平衡系统加载完成")
    print("\n可用环境:")
    print("  1. 平地")
    print("  2. 上坡 (5°/10°)")
    print("  3. 下坡 (5°)")
    print("  4. 大风环境")
    print("  5. 冰面")
    print("  6. 崎岖地形")
    print("  7. 极端条件")
