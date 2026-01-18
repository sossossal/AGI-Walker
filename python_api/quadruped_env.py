"""
Quadruped Robot Gymnasium Environment
四足机器人强化学习环境
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces


class QuadrupedEnv(gym.Env):
    """
    12 DoF 四足机器人环境
    
    观测空间 (34维):
        - 身体姿态 (3): roll, pitch, yaw
        - 身体角速度 (3): angular velocities
        - 身体线速度 (3): linear velocities  
        - 12个关节角度
        - 12个关节角速度
        - 目标方向 (1): target direction
    
    动作空间 (12维):
        - 12个关节的目标位置 (normalized)
    """
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 60}
    
    def __init__(self, render_mode=None, config_path=None):
        super().__init__()
        
        self.render_mode = render_mode
        
        # 观测空间: 34维
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(34,),
            dtype=np.float32
        )
        
        # 动作空间: 12维 (normalized to [-1, 1])
        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),
            dtype=np.float32
        )
        
        # 四足机器人参数
        self.num_legs = 4
        self.joints_per_leg = 3
        self.total_joints = 12
        
        # 关节限制 (弧度)
        self.joint_limits = {
            'hip': (-np.pi/4, np.pi/4),      # ±45°
            'thigh': (-2*np.pi/3, np.pi/3),  # -120° to 60°
            'shin': (0, 5*np.pi/6)           # 0° to 150°
        }
        
        # 奖励权重
        self.reward_weights = {
            'forward_velocity': 1.0,
            'energy_penalty': -0.05,
            'stability': 0.5,
            'orientation': 0.3,
            'height': 0.2
        }
        
        # 状态变量
        self.time_step = 0
        self.max_episode_steps = 1000
        
    def reset(self, seed=None, options=None):
        """重置环境"""
        super().reset(seed=seed)
        
        self.time_step = 0
        
        # 初始化状态
        observation = self._get_observation()
        info = {}
        
        return observation, info
    
    def step(self, action):
        """执行一步"""
        self.time_step += 1
        
        # 执行动作
        self._execute_action(action)
        
        # 获取观测
        observation = self._get_observation()
        
        # 计算奖励
        reward = self._compute_reward(action)
        
        # 检查终止条件
        terminated = self._is_terminated()
        truncated = self.time_step >= self.max_episode_steps
        
        info = {
            'time_step': self.time_step,
            'forward_velocity': self._get_forward_velocity()
        }
        
        return observation, reward, terminated, truncated, info
    
    def _get_observation(self):
        """获取当前观测"""
        # TODO: 从 Godot 获取实际状态
        # 这里返回模拟观测
        obs = np.zeros(34, dtype=np.float32)
        
        # 身体姿态 (roll, pitch, yaw)
        obs[0:3] = [0, 0, 0]
        
        # 身体角速度
        obs[3:6] = [0, 0, 0]
        
        # 身体线速度
        obs[6:9] = [0, 0, 0]
        
        # 12个关节角度
        obs[9:21] = np.zeros(12)
        
        # 12个关节角速度
        obs[21:33] = np.zeros(12)
        
        # 目标方向
        obs[33] = 0
        
        return obs
    
    def _execute_action(self, action):
        """执行动作 - 将归一化动作映射到关节角度"""
        # 将 [-1, 1] 映射到实际关节角度
        joint_targets = np.zeros(12)
        
        for i in range(self.num_legs):
            # Hip
            joint_targets[i*3 + 0] = self._denormalize_joint(
                action[i*3 + 0], *self.joint_limits['hip']
            )
            # Thigh
            joint_targets[i*3 + 1] = self._denormalize_joint(
                action[i*3 + 1], *self.joint_limits['thigh']
            )
            # Shin
            joint_targets[i*3 + 2] = self._denormalize_joint(
                action[i*3 + 2], *self.joint_limits['shin']
            )
        
        # TODO: 发送到 Godot 仿真
        pass
    
    def _denormalize_joint(self, normalized_value, min_val, max_val):
        """将 [-1, 1] 归一化值映射到关节范围"""
        return min_val + (normalized_value + 1) * (max_val - min_val) / 2
    
    def _compute_reward(self, action):
        """计算奖励"""
        reward = 0.0
        
        # 1. 前进速度奖励
        forward_vel = self._get_forward_velocity()
        reward += self.reward_weights['forward_velocity'] * forward_vel
        
        # 2. 能量惩罚
        energy = np.sum(np.square(action))
        reward += self.reward_weights['energy_penalty'] * energy
        
        # 3. 稳定性奖励（低姿态抖动）
        stability = -self._get_body_angular_velocity_magnitude()
        reward += self.reward_weights['stability'] * stability
        
        # 4. 姿态奖励（保持水平）
        orientation = -abs(self._get_body_roll()) - abs(self._get_body_pitch())
        reward += self.reward_weights['orientation'] * orientation
        
        # 5. 高度奖励（保持合适高度）
        height_error = abs(self._get_body_height() - 0.3)
        reward += self.reward_weights['height'] * (-height_error)
        
        return reward
    
    def _is_terminated(self):
        """检查是否终止"""
        # 摔倒检测
        if abs(self._get_body_roll()) > np.pi/3:  # 60度
            return True
        if abs(self._get_body_pitch()) > np.pi/3:
            return True
        
        # 高度检测
        if self._get_body_height() < 0.1:
            return True
        
        return False
    
    # 辅助函数 - TODO: 从 Godot 获取实际值
    def _get_forward_velocity(self):
        return 0.0
    
    def _get_body_angular_velocity_magnitude(self):
        return 0.0
    
    def _get_body_roll(self):
        return 0.0
    
    def _get_body_pitch(self):
        return 0.0
    
    def _get_body_height(self):
        return 0.3
    
    def render(self):
        """渲染环境"""
        if self.render_mode == "rgb_array":
            # TODO: 从 Godot 获取图像
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def close(self):
        """关闭环境"""
        pass


# 注册环境
gym.register(
    id='AGI-Walker/Quadruped-v0',
    entry_point='python_api.quadruped_env:QuadrupedEnv',
    max_episode_steps=1000,
)


if __name__ == "__main__":
    print("Quadruped Environment 创建成功")
    print("使用方法:")
    print("  import gymnasium as gym")
    print("  env = gym.make('AGI-Walker/Quadruped-v0')")
