"""
四足机器人步态生成器
实现 Trot 和 Gallop 步态
"""

import numpy as np


class GaitGenerator:
    """步态生成器基类"""
    
    def __init__(self, frequency=1.0):
        self.frequency = frequency
        self.phase = 0.0
        
    def update(self, dt):
        """更新相位"""
        self.phase += 2 * np.pi * self.frequency * dt
        self.phase = self.phase % (2 * np.pi)
    
    def get_foot_position(self, leg_id, time):
        """获取指定腿的足端位置"""
        raise NotImplementedError


class TrotGait(GaitGenerator):
    """
    Trot 步态（小跑）
    对角腿同时摆动
    相位: FL-RR = 0°, FR-RL = 180°
    """
    
    def __init__(self, frequency=1.0, step_height=0.05, stride_length=0.15):
        super().__init__(frequency)
        self.step_height = step_height
        self.stride_length = stride_length
        
        # 定义相位偏移
        self.phase_offsets = {
            0: 0.0,         # Front Left
            1: np.pi,       # Front Right
            2: np.pi,       # Rear Left
            3: 0.0,         # Rear Right
        }
    
    def get_foot_trajectory(self, leg_id, phase):
        """
        获取足端轨迹
        
        返回: (x, y, z) 相对于髋关节的位置
        """
        # 计算该腿的相位
        leg_phase = (phase + self.phase_offsets[leg_id]) % (2 * np.pi)
        
        # 前后摆动（x方向）
        if leg_phase < np.pi:
            # 摆动相（空中）
            swing_progress = leg_phase / np.pi
            x = -self.stride_length / 2 + self.stride_length * swing_progress
            z = -0.25 + self.step_height * np.sin(swing_progress * np.pi)
        else:
            # 支撑相（接触地面）
            stance_progress = (leg_phase - np.pi) / np.pi
            x = self.stride_length / 2 - self.stride_length * stance_progress
            z = -0.25
        
        y = 0.0  # 侧向偏移
        
        return np.array([x, y, z])


class GallopGait(GaitGenerator):
    """
    Gallop 步态（疾驰）
    前腿先后着地，后腿先后着地
    相位: FL=0°, FR=90°, RL=180°, RR=270°
    """
    
    def __init__(self, frequency=1.5, step_height=0.08, stride_length=0.25):
        super().__init__(frequency)
        self.step_height = step_height
        self.stride_length = stride_length
        
        # 定义相位偏移
        self.phase_offsets = {
            0: 0.0,              # Front Left
            1: np.pi / 2,        # Front Right
            2: np.pi,            # Rear Left
            3: 3 * np.pi / 2,    # Rear Right
        }
    
    def get_foot_trajectory(self, leg_id, phase):
        """获取 Gallop 步态的足端轨迹"""
        leg_phase = (phase + self.phase_offsets[leg_id]) % (2 * np.pi)
        
        # 更激进的步态
        if leg_phase < np.pi:
            # 快速摆动
            swing_progress = leg_phase / np.pi
            x = -self.stride_length / 2 + self.stride_length * swing_progress
            z = -0.25 + self.step_height * np.sin(swing_progress * np.pi)
        else:
            # 支撑相
            stance_progress = (leg_phase - np.pi) / np.pi
            x = self.stride_length / 2 - self.stride_length * stance_progress
            z = -0.25
        
        y = 0.0
        
        return np.array([x, y, z])


class GaitController:
    """步态控制器 - 管理多种步态的切换"""
    
    def __init__(self):
        self.gaits = {
            'trot': TrotGait(frequency=1.0),
            'gallop': GallopGait(frequency=1.5),
            'walk': TrotGait(frequency=0.5),  # 慢速Trot
        }
        
        self.current_gait = 'trot'
    
    def set_gait(self, gait_name):
        """切换步态"""
        if gait_name in self.gaits:
            self.current_gait = gait_name
        else:
            raise ValueError(f"Unknown gait: {gait_name}")
    
    def get_joint_targets(self, phase):
        """
        根据当前步态生成12个关节的目标角度
        
        返回: (12,) 数组，顺序为 [FL_hip, FL_thigh, FL_shin, FR_..., RL_..., RR_...]
        """
        gait = self.gaits[self.current_gait]
        joint_targets = np.zeros(12)
        
        for leg_id in range(4):
            # 获取足端目标位置
            foot_pos = gait.get_foot_trajectory(leg_id, phase)
            
            # 逆运动学求解关节角度
            joint_angles = self._inverse_kinematics(foot_pos, leg_id)
            
            # 填充关节目标
            joint_targets[leg_id * 3: (leg_id + 1) * 3] = joint_angles
        
        return joint_targets
    
    def _inverse_kinematics(self, foot_pos, leg_id):
        """
        简化的逆运动学求解
        
        假设：
        - 髋关节到大腿: 0.1m
        - 大腿长度: 0.15m
        - 小腿长度: 0.15m
        """
        x, y, z = foot_pos
        
        # Hip 角度（侧向摆动）
        hip_angle = np.arctan2(y, -z)
        
        # 在大腿-小腿平面内求解
        horizontal_dist = np.sqrt(x**2 + (z / np.cos(hip_angle))**2)
        
        # 大腿和小腿长度
        l1 = 0.15
        l2 = 0.15
        
        # Thigh 和 Shin 角度（简化2D IK）
        d = np.clip(horizontal_dist, 0.05, l1 + l2 - 0.01)
        
        cos_shin = (l1**2 + l2**2 - d**2) / (2 * l1 * l2)
        cos_shin = np.clip(cos_shin, -1, 1)
        shin_angle = np.pi - np.arccos(cos_shin)
        
        alpha = np.arctan2(z, x)
        beta = np.arccos(np.clip((d**2 + l1**2 - l2**2) / (2 * d * l1), -1, 1))
        thigh_angle = -(alpha + beta)
        
        return np.array([hip_angle, thigh_angle, shin_angle])


if __name__ == "__main__":
    print("步态生成器测试")
    
    # 创建控制器
    controller = GaitController()
    
    # 测试 Trot 步态
    phase = 0.0
    targets = controller.get_joint_targets(phase)
    print(f"\nTrot 步态关节目标 (phase={phase}):")
    print(f"  关节角度: {targets}")
    
    # 切换到 Gallop
    controller.set_gait('gallop')
    targets = controller.get_joint_targets(phase)
    print(f"\nGallop 步态关节目标 (phase={phase}):")
    print(f"  关节角度: {targets}")
