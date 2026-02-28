"""
任务: 物体抓取 (Object Grasping)

目标: 抓取桌面上的随机物体
难度: ⭐⭐⭐⭐ (困难)
机器人: 机械臂
"""

import sys
import os

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple, List


class ObjectGraspingEnv(gym.Env):
    """
    物体抓取环境

    观测空间:
        - 关节位置/速度 (14D: 7 DoF 机械臂)
        - 末端执行器位置/姿态 (7D)
        - 目标物体位置/姿态 (7D)
        - 夹爪状态 (1D)
        - 距离 (1D)

    动作空间:
        - 关节速度 (7D)
        - 夹爪开合 (1D)

    奖励函数:
        - 接近奖励: -distance_to_object
        - 抓取奖励: +10.0 (成功抓取)
        - 举起奖励: +5.0 (举起物体)
        - 能耗: -0.01 * sum(velocity^2)
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, render_mode=None, num_objects=1):
        super().__init__()

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(30,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )

        # 环境参数
        self.num_objects = num_objects
        self.table_height = 0.6
        self.workspace_size = 0.5  # 50cm x 50cm 工作空间

        # 物体类型
        self.object_types = ["cube", "sphere", "cylinder"]

        # 状态
        self.joint_pos = np.zeros(7)
        self.joint_vel = np.zeros(7)
        self.ee_pos = np.zeros(3)  # End-effector position
        self.ee_quat = np.array([0, 0, 0, 1])  # Quaternion
        self.gripper_state = 0.0  # 0=open, 1=closed

        self.object_pos = np.zeros(3)
        self.object_quat = np.array([0, 0, 0, 1])
        self.object_grasped = False

        self.render_mode = render_mode

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)

        # 重置机械臂到初始位置
        self.joint_pos = np.array([0, -0.5, 0, -1.5, 0, 1.0, 0])
        self.joint_vel = np.zeros(7)
        self.ee_pos = np.array([0.3, 0.0, 0.8])
        self.gripper_state = 0.0

        # 随机生成物体
        self._spawn_object()

        obs = self._get_observation()
        info = {"object_type": self.object_types[0], "grasped": False}

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        # 应用动作
        joint_action = action[:7]
        gripper_action = action[7]

        # 更新关节
        self.joint_vel = joint_action * 0.1
        self.joint_pos += self.joint_vel * 0.01

        # 更新末端执行器 (简化版 FK)
        self.ee_pos += joint_action[:3] * 0.01

        # 更新夹爪
        self.gripper_state = np.clip(self.gripper_state + gripper_action * 0.1, 0, 1)

        # 检测抓取
        distance = np.linalg.norm(self.ee_pos - self.object_pos)
        if distance < 0.05 and self.gripper_state > 0.8:
            self.object_grasped = True
            self.object_pos = self.ee_pos.copy()  # 物体跟随末端

        # 计算奖励
        reward = self._compute_reward(distance)

        # 检查终止
        terminated = False
        truncated = self.object_grasped and self.object_pos[2] > self.table_height + 0.2

        obs = self._get_observation()
        info = {
            "distance": distance,
            "grasped": self.object_grasped,
            "lifted": truncated,
        }

        return obs, reward, terminated, truncated, info

    def _spawn_object(self):
        """随机生成物体"""
        # 在桌面上随机位置
        self.object_pos = np.array(
            [
                np.random.uniform(-self.workspace_size / 2, self.workspace_size / 2),
                np.random.uniform(-self.workspace_size / 2, self.workspace_size / 2),
                self.table_height + 0.05,
            ]
        )
        self.object_quat = np.array([0, 0, 0, 1])
        self.object_grasped = False

    def _get_observation(self) -> np.ndarray:
        obs = np.concatenate(
            [
                self.joint_pos,  # 7D
                self.joint_vel,  # 7D
                self.ee_pos,  # 3D
                self.ee_quat,  # 4D
                self.object_pos,  # 3D
                self.object_quat,  # 4D
                [self.gripper_state],  # 1D
                [np.linalg.norm(self.ee_pos - self.object_pos)],  # 1D (距离)
            ]
        )
        return obs.astype(np.float32)

    def _compute_reward(self, distance: float) -> float:
        # 接近奖励
        approach_reward = -distance

        # 抓取奖励
        grasp_reward = 10.0 if self.object_grasped else 0.0

        # 举起奖励
        lift_reward = (
            5.0
            if (self.object_grasped and self.object_pos[2] > self.table_height + 0.1)
            else 0.0
        )

        # 能耗
        energy_cost = -0.01 * np.sum(self.joint_vel**2)

        return approach_reward + grasp_reward + lift_reward + energy_cost


if __name__ == "__main__":
    gym.register(id="ObjectGrasping-v0", entry_point="__main__:ObjectGraspingEnv")

    env = ObjectGraspingEnv()

    print("🤖 物体抓取任务 Demo")
    for episode in range(5):
        obs, info = env.reset()
        total_reward = 0

        for step in range(300):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward

            if terminated or truncated:
                break

        status = "✅ 成功" if info["lifted"] else "❌ 失败"
        print(
            f"Episode {episode+1}: {status}, Distance={info['distance']:.3f}m, Reward={total_reward:.2f}"
        )
