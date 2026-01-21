"""
任务: 楼梯攀爬 (Stair Climbing)

目标: 训练机器人从平地爬上楼梯
难度: ⭐⭐⭐ (中等)
机器人: 四足/双足
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple


class StairClimbingEnv(gym.Env):
    """
    楼梯攀爬环境
    
    观测空间:
        - 关节位置 (8D)
        - 关节速度 (8D)
        - 躯干姿态 (4D: 四元数)
        - 躯干速度 (3D)
        - 目标距离 (1D)
    
    动作空间:
        - 关节目标位置 (8D)
    
    奖励函数:
        - 前进奖励: +1.0 per meter
        - 高度奖励: +0.5 per step climbed
        - 稳定性: -0.1 * |roll| - 0.1 * |pitch|
        - 能耗惩罚: -0.01 * sum(torque^2)
        - 摔倒惩罚: -10.0
    """
    
    metadata = {"render_modes": ["human", "rgb_array"]}
    
    def __init__(self, render_mode=None):
        super().__init__()
        
        # 观测和动作空间
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(24,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )
        
        # 环境参数
        self.stair_height = 0.15  # 台阶高度 15cm
        self.stair_depth = 0.30   # 台阶深度 30cm
        self.num_stairs = 5       # 台阶数量
        
        # 状态
        self.robot_pos = np.zeros(3)
        self.robot_vel = np.zeros(3)
        self.joint_pos = np.zeros(8)
        self.joint_vel = np.zeros(8)
        self.steps_climbed = 0
        
        self.render_mode = render_mode
    
    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)
        
        # 重置机器人到起点
        self.robot_pos = np.array([0.0, 0.0, 0.5])  # 起始位置
        self.robot_vel = np.zeros(3)
        self.joint_pos = np.zeros(8)
        self.joint_vel = np.zeros(8)
        self.steps_climbed = 0
        
        obs = self._get_observation()
        info = {"steps_climbed": 0}
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        # 应用动作 (简化版,实际应调用物理引擎)
        self.joint_pos += action * 0.1
        self.robot_pos[0] += 0.01  # 简化: 假设机器人前进
        
        # 检查是否爬上台阶
        current_stair = int(self.robot_pos[0] / self.stair_depth)
        if current_stair > self.steps_climbed:
            self.steps_climbed = current_stair
            self.robot_pos[2] += self.stair_height
        
        # 计算奖励
        reward = self._compute_reward(action)
        
        # 检查终止条件
        terminated = self._check_terminated()
        truncated = self.steps_climbed >= self.num_stairs
        
        obs = self._get_observation()
        info = {
            "steps_climbed": self.steps_climbed,
            "distance": self.robot_pos[0]
        }
        
        return obs, reward, terminated, truncated, info
    
    def _get_observation(self) -> np.ndarray:
        """获取观测"""
        # 简化版观测
        obs = np.concatenate([
            self.joint_pos,      # 8D
            self.joint_vel,      # 8D
            self.robot_pos,      # 3D
            self.robot_vel,      # 3D
            [self.steps_climbed / self.num_stairs],  # 1D (归一化进度)
            [0.0]  # 1D (填充到 24D)
        ])
        return obs.astype(np.float32)
    
    def _compute_reward(self, action: np.ndarray) -> float:
        """计算奖励"""
        # 前进奖励
        forward_reward = self.robot_pos[0] * 0.1
        
        # 爬升奖励
        climb_reward = self.steps_climbed * 0.5
        
        # 能耗惩罚
        energy_cost = -0.01 * np.sum(action ** 2)
        
        # 稳定性奖励 (简化: 假设机器人保持水平)
        stability_reward = 0.1
        
        return forward_reward + climb_reward + energy_cost + stability_reward
    
    def _check_terminated(self) -> bool:
        """检查是否摔倒"""
        # 简化: 检查高度
        if self.robot_pos[2] < 0.2:
            return True
        return False
    
    def render(self):
        if self.render_mode == "human":
            print(f"Steps: {self.steps_climbed}/{self.num_stairs}, "
                  f"Pos: {self.robot_pos[0]:.2f}m")


# ==================== 训练脚本 ====================

def train_stair_climbing():
    """训练楼梯攀爬任务"""
    print("\n🏃 开始训练: 楼梯攀爬任务")
    print("="*60)
    
    # 创建环境
    env = StairClimbingEnv()
    
    # 简单的随机策略 (示例)
    print("\n运行随机策略 (10 episodes)...")
    
    for episode in range(10):
        obs, info = env.reset()
        total_reward = 0
        
        for step in range(200):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            
            if terminated or truncated:
                break
        
        print(f"Episode {episode+1}: "
              f"Steps Climbed={info['steps_climbed']}, "
              f"Reward={total_reward:.2f}")
    
    print("\n✅ 训练完成!")
    print("下一步: 使用 PPO 算法进行强化学习训练")


if __name__ == "__main__":
    # 注册环境
    gym.register(
        id='StairClimbing-v0',
        entry_point='__main__:StairClimbingEnv',
    )
    
    # 运行训练
    train_stair_climbing()
