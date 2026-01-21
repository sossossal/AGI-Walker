"""
任务: 协作搬运 (Collaborative Carrying)
目标: 两个机器人协作搬运重物
难度: ⭐⭐⭐⭐⭐ (非常困难)
机器人: 双四足
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple


class CollaborativeCarryingEnv(gym.Env):
    """协作搬运环境"""
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    
    def __init__(self, render_mode=None):
        super().__init__()
        
        # 双机器人动作空间
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(60,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(16,), dtype=np.float32  # 2 robots x 8 joints
        )
        
        # 环境参数
        self.object_mass = 10.0  # 10kg (单个机器人无法搬运)
        self.target_distance = 5.0
        
        # 状态
        self.robot1_pos = np.zeros(3)
        self.robot2_pos = np.zeros(3)
        self.object_pos = np.zeros(3)
        self.object_grasped = [False, False]
        
        self.render_mode = render_mode
    
    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)
        
        # 初始化两个机器人在物体两侧
        self.robot1_pos = np.array([-0.5, 0.0, 0.5])
        self.robot2_pos = np.array([0.5, 0.0, 0.5])
        self.object_pos = np.array([0.0, 0.0, 0.3])
        self.object_grasped = [False, False]
        
        obs = self._get_observation()
        info = {"distance": 0.0, "both_grasped": False}
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        # 分离两个机器人的动作
        action1 = action[:8]
        action2 = action[8:]
        
        # 更新机器人位置
        self.robot1_pos[:2] += action1[:2] * 0.01
        self.robot2_pos[:2] += action2[:2] * 0.01
        
        # 检测抓取
        dist1 = np.linalg.norm(self.robot1_pos - self.object_pos)
        dist2 = np.linalg.norm(self.robot2_pos - self.object_pos)
        
        if dist1 < 0.3:
            self.object_grasped[0] = True
        if dist2 < 0.3:
            self.object_grasped[1] = True
        
        # 只有两个机器人都抓住才能移动物体
        if all(self.object_grasped):
            # 物体位置为两个机器人的中点
            self.object_pos = (self.robot1_pos + self.robot2_pos) / 2
        
        # 计算奖励
        distance_moved = self.object_pos[0]  # x方向移动距离
        reward = self._compute_reward(distance_moved)
        
        # 检查完成
        terminated = False
        truncated = distance_moved >= self.target_distance
        
        obs = self._get_observation()
        info = {
            "distance": distance_moved,
            "both_grasped": all(self.object_grasped)
        }
        
        return obs, reward, terminated, truncated, info
    
    def _get_observation(self) -> np.ndarray:
        obs = np.concatenate([
            self.robot1_pos,
            np.zeros(5),  # robot1 其他状态
            self.robot2_pos,
            np.zeros(5),  # robot2 其他状态
            self.object_pos,
            [1.0 if self.object_grasped[0] else 0.0],
            [1.0 if self.object_grasped[1] else 0.0],
            np.zeros(40)  # 填充
        ])
        return obs[:60].astype(np.float32)
    
    def _compute_reward(self, distance: float) -> float:
        # 移动奖励
        move_reward = distance * 2.0
        
        # 协作奖励
        collab_reward = 5.0 if all(self.object_grasped) else 0.0
        
        # 同步奖励 (两个机器人距离接近)
        sync_penalty = -np.linalg.norm(self.robot1_pos - self.robot2_pos)
        
        return move_reward + collab_reward + sync_penalty * 0.1


if __name__ == "__main__":
    gym.register(id='CollaborativeCarrying-v0', entry_point='__main__:CollaborativeCarryingEnv')
    
    env = CollaborativeCarryingEnv()
    print("🤝 协作搬运任务 Demo")
    
    for episode in range(3):
        obs, info = env.reset()
        total_reward = 0
        
        for step in range(500):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            
            if terminated or truncated:
                break
        
        status = "✅ 完成" if info["distance"] >= 5.0 else f"⏱️ {info['distance']:.2f}m/5.0m"
        print(f"Episode {episode+1}: {status}, 协作={info['both_grasped']}, Reward={total_reward:.2f}")
