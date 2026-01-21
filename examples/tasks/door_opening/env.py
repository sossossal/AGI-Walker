"""
任务: 门把手操作 (Door Opening)
目标: 抓住门把手并打开门
难度: ⭐⭐⭐⭐⭐ (非常困难)
机器人: 机械臂
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple


class DoorOpeningEnv(gym.Env):
    """
    门把手操作环境
    
    观测空间:
        - 关节位置/速度 (14D)
        - 末端执行器位置/姿态 (7D)
        - 门把手位置/姿态 (7D)
        - 门角度 (1D)
        - 夹爪状态 (1D)
    
    动作空间:
        - 关节速度 (7D)
        - 夹爪开合 (1D)
    
    奖励函数:
        - 接近把手: -distance
        - 抓住把手: +5.0
        - 转动把手: +10.0
        - 打开门: +20.0
    """
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    
    def __init__(self, render_mode=None):
        super().__init__()
        
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(30,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )
        
        # 环境参数
        self.door_width = 0.8
        self.handle_height = 1.0
        self.handle_radius = 0.03
        
        # 状态
        self.joint_pos = np.zeros(7)
        self.ee_pos = np.zeros(3)
        self.handle_pos = np.array([0.7, 0.0, 1.0])
        self.door_angle = 0.0  # 0=关闭, 1.57=打开90度
        self.handle_grasped = False
        self.handle_turned = False
        
        self.render_mode = render_mode
    
    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)
        
        self.joint_pos = np.array([0, -0.5, 0, -1.5, 0, 1.0, 0])
        self.ee_pos = np.array([0.3, 0.0, 0.8])
        self.door_angle = 0.0
        self.handle_grasped = False
        self.handle_turned = False
        
        obs = self._get_observation()
        info = {"door_angle": 0.0, "opened": False}
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        # 更新机械臂
        self.joint_pos += action[:7] * 0.01
        self.ee_pos += action[:3] * 0.01
        
        # 检测抓取
        dist_to_handle = np.linalg.norm(self.ee_pos - self.handle_pos)
        if dist_to_handle < 0.05 and action[7] > 0.5:
            self.handle_grasped = True
        
        # 转动把手
        if self.handle_grasped:
            self.handle_turned = True
            # 打开门
            self.door_angle = min(self.door_angle + 0.02, 1.57)
        
        # 计算奖励
        reward = self._compute_reward(dist_to_handle)
        
        # 检查完成
        terminated = False
        truncated = self.door_angle > 1.5  # 门打开90度
        
        obs = self._get_observation()
        info = {
            "door_angle": self.door_angle,
            "opened": truncated,
            "handle_grasped": self.handle_grasped
        }
        
        return obs, reward, terminated, truncated, info
    
    def _get_observation(self) -> np.ndarray:
        obs = np.concatenate([
            self.joint_pos,
            np.zeros(7),  # joint_vel
            self.ee_pos,
            np.zeros(4),  # ee_quat
            self.handle_pos,
            np.zeros(4),  # handle_quat
            [self.door_angle],
            [1.0 if self.handle_grasped else 0.0]
        ])
        return obs.astype(np.float32)
    
    def _compute_reward(self, distance: float) -> float:
        approach_reward = -distance
        grasp_reward = 5.0 if self.handle_grasped else 0.0
        turn_reward = 10.0 if self.handle_turned else 0.0
        open_reward = 20.0 * (self.door_angle / 1.57)
        
        return approach_reward + grasp_reward + turn_reward + open_reward


if __name__ == "__main__":
    gym.register(id='DoorOpening-v0', entry_point='__main__:DoorOpeningEnv')
    
    env = DoorOpeningEnv()
    print("🚪 门把手操作任务 Demo")
    
    for episode in range(3):
        obs, info = env.reset()
        total_reward = 0
        
        for step in range(500):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            
            if terminated or truncated:
                break
        
        status = "✅ 成功" if info["opened"] else "❌ 失败"
        print(f"Episode {episode+1}: {status}, 门角度={np.rad2deg(info['door_angle']):.1f}°, Reward={total_reward:.2f}")
