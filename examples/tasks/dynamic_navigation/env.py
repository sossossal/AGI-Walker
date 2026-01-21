"""
任务: 动态环境导航 (Dynamic Navigation)
目标: 在快速变化的环境中导航
难度: ⭐⭐⭐⭐ (困难)
机器人: 四足/轮式
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import gymnasium as gym
import numpy as np


class DynamicNavigationEnv(gym.Env):
    """动态环境导航 (基于避障导航扩展)"""
    
    def __init__(self):
        super().__init__()
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(370,), dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        
        self.robot_pos = np.zeros(2)
        self.goal_pos = np.zeros(2)
        self.obstacles = []
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_pos = np.zeros(2)
        self.goal_pos = np.random.uniform(-5, 5, 2)
        # 生成快速移动的障碍物
        self.obstacles = [{"pos": np.random.uniform(-5, 5, 2), "vel": np.random.uniform(-0.5, 0.5, 2)} 
                          for _ in range(10)]
        return self._get_observation(), {}
    
    def step(self, action):
        self.robot_pos += action * 0.1
        # 更新障碍物
        for obs in self.obstacles:
            obs["pos"] += obs["vel"] * 0.1
        
        dist = np.linalg.norm(self.robot_pos - self.goal_pos)
        reward = -dist
        done = dist < 0.5
        
        return self._get_observation(), reward, False, done, {"distance": dist}
    
    def _get_observation(self):
        return np.concatenate([self.robot_pos, self.goal_pos, np.zeros(366)]).astype(np.float32)


if __name__ == "__main__":
    gym.register(id='DynamicNavigation-v0', entry_point='__main__:DynamicNavigationEnv')
    env = DynamicNavigationEnv()
    print("🌪️ 动态环境导航任务 Demo")
    obs, _ = env.reset()
    for _ in range(200):
        action = env.action_space.sample()
        obs, reward, _, done, info = env.step(action)
        if done:
            break
    print(f"✅ Demo 完成, 距离={info['distance']:.2f}m")
