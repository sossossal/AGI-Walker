"""
任务: 崎岖地形 (Rough Terrain)

目标: 在随机生成的崎岖地形上行走 10 米
难度: ⭐⭐⭐⭐ (困难)
机器人: 四足
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple


class RoughTerrainEnv(gym.Env):
    """
    崎岖地形环境
    
    特点:
        - 程序化地形生成 (Perlin Noise)
        - 动态难度调整
        - 多种地形类型 (石头、坑洼、斜坡)
    
    观测空间:
        - 关节位置/速度 (16D)
        - 躯干姿态/速度 (7D)
        - 局部高程图 (25D: 5x5 grid)
        - 目标距离 (1D)
    
    动作空间:
        - 关节目标位置 (8D)
    
    奖励函数:
        - 前进奖励: +1.0 per meter
        - 稳定性: -0.2 * |roll| - 0.2 * |pitch|
        - 能耗: -0.01 * sum(torque^2)
        - 摔倒惩罚: -10.0
        - 地形适应奖励: +0.5 (根据地形复杂度)
    """
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    
    def __init__(self, render_mode=None, difficulty="medium"):
        super().__init__()
        
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(49,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )
        
        # 地形参数
        self.difficulty = difficulty
        self.terrain_length = 10.0  # 10 米
        self.terrain_width = 2.0
        self.terrain_resolution = 0.1  # 10cm 分辨率
        
        # 难度配置
        self.difficulty_config = {
            "easy": {"roughness": 0.05, "obstacle_density": 0.1},
            "medium": {"roughness": 0.10, "obstacle_density": 0.3},
            "hard": {"roughness": 0.20, "obstacle_density": 0.5}
        }
        
        # 状态
        self.robot_pos = np.zeros(3)
        self.robot_vel = np.zeros(3)
        self.joint_pos = np.zeros(8)
        self.joint_vel = np.zeros(8)
        self.terrain_map = None
        
        self.render_mode = render_mode
    
    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)
        
        # 生成地形
        self._generate_terrain()
        
        # 重置机器人
        self.robot_pos = np.array([0.0, 0.0, 0.5])
        self.robot_vel = np.zeros(3)
        self.joint_pos = np.zeros(8)
        self.joint_vel = np.zeros(8)
        
        obs = self._get_observation()
        info = {"distance": 0.0, "terrain_difficulty": self.difficulty}
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        # 应用动作
        self.joint_pos += action * 0.1
        self.robot_pos[0] += 0.02  # 简化: 假设前进
        
        # 获取当前位置的地形高度
        terrain_height = self._get_terrain_height(self.robot_pos[0], self.robot_pos[1])
        self.robot_pos[2] = terrain_height + 0.5  # 保持在地面上方
        
        # 计算奖励
        reward = self._compute_reward(action)
        
        # 检查终止
        terminated = self._check_terminated()
        truncated = self.robot_pos[0] >= self.terrain_length
        
        obs = self._get_observation()
        info = {
            "distance": self.robot_pos[0],
            "terrain_roughness": self._get_local_roughness()
        }
        
        return obs, reward, terminated, truncated, info
    
    def _generate_terrain(self):
        """生成程序化地形"""
        config = self.difficulty_config[self.difficulty]
        
        # 简化版: 使用随机高度图
        grid_size = int(self.terrain_length / self.terrain_resolution)
        self.terrain_map = np.random.randn(grid_size, grid_size) * config["roughness"]
        
        # 添加障碍物
        if np.random.rand() < config["obstacle_density"]:
            obstacle_x = np.random.randint(0, grid_size)
            obstacle_y = np.random.randint(0, grid_size)
            self.terrain_map[obstacle_x, obstacle_y] += 0.3  # 石头
    
    def _get_terrain_height(self, x: float, y: float) -> float:
        """获取指定位置的地形高度"""
        grid_x = int(x / self.terrain_resolution)
        grid_y = int((y + self.terrain_width / 2) / self.terrain_resolution)
        
        if 0 <= grid_x < self.terrain_map.shape[0] and 0 <= grid_y < self.terrain_map.shape[1]:
            return self.terrain_map[grid_x, grid_y]
        return 0.0
    
    def _get_local_heightmap(self) -> np.ndarray:
        """获取局部高程图 (5x5)"""
        heightmap = np.zeros((5, 5))
        for i in range(5):
            for j in range(5):
                x = self.robot_pos[0] + (i - 2) * 0.2
                y = self.robot_pos[1] + (j - 2) * 0.2
                heightmap[i, j] = self._get_terrain_height(x, y)
        return heightmap.flatten()
    
    def _get_local_roughness(self) -> float:
        """计算局部地形粗糙度"""
        heightmap = self._get_local_heightmap().reshape(5, 5)
        return np.std(heightmap)
    
    def _get_observation(self) -> np.ndarray:
        obs = np.concatenate([
            self.joint_pos,      # 8D
            self.joint_vel,      # 8D
            self.robot_pos,      # 3D
            self.robot_vel,      # 3D
            self._get_local_heightmap(),  # 25D
            [self.robot_pos[0] / self.terrain_length],  # 1D (进度)
            [0.0]  # 1D (填充)
        ])
        return obs.astype(np.float32)
    
    def _compute_reward(self, action: np.ndarray) -> float:
        forward_reward = 0.1
        stability_reward = 0.1
        energy_cost = -0.01 * np.sum(action ** 2)
        terrain_bonus = 0.5 * self._get_local_roughness()  # 奖励适应复杂地形
        
        return forward_reward + stability_reward + energy_cost + terrain_bonus
    
    def _check_terminated(self) -> bool:
        if self.robot_pos[2] < 0.2:
            return True
        return False


if __name__ == "__main__":
    gym.register(id='RoughTerrain-v0', entry_point='__main__:RoughTerrainEnv')
    
    env = RoughTerrainEnv(difficulty="medium")
    obs, info = env.reset()
    
    print("🏃 崎岖地形任务 Demo")
    for episode in range(5):
        obs, info = env.reset()
        total_reward = 0
        
        for step in range(200):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            
            if terminated or truncated:
                break
        
        print(f"Episode {episode+1}: Distance={info['distance']:.2f}m, Reward={total_reward:.2f}")
