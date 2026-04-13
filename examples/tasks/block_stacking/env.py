"""
:  (Block Stacking)
: 3
:  ()
:
"""

import sys
import os

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

import gymnasium as gym
import numpy as np
from typing import Dict, Tuple, List


class BlockStackingEnv(gym.Env):
    """"""

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, render_mode=None, num_blocks=3):
        super().__init__()

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(50,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )

        self.num_blocks = num_blocks
        self.block_size = 0.05  # 5cm
        self.table_height = 0.6

        #
        self.joint_pos = np.zeros(7)
        self.ee_pos = np.zeros(3)
        self.blocks: List[Dict] = []
        self.stacked_count = 0

        self.render_mode = render_mode

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)

        self.joint_pos = np.array([0, -0.5, 0, -1.5, 0, 1.0, 0])
        self.ee_pos = np.array([0.3, 0.0, 0.8])

        #
        self.blocks = []
        for i in range(self.num_blocks):
            pos = np.array(
                [
                    np.random.uniform(-0.2, 0.2),
                    np.random.uniform(-0.2, 0.2),
                    self.table_height + self.block_size / 2,
                ]
            )
            self.blocks.append({"pos": pos, "grasped": False, "stacked": False})

        self.stacked_count = 0

        obs = self._get_observation()
        info = {"stacked": 0}

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        #
        self.joint_pos += action[:7] * 0.01
        self.ee_pos += action[:3] * 0.01

        #
        for block in self.blocks:
            if not block["stacked"]:
                dist = np.linalg.norm(self.ee_pos - block["pos"])
                if dist < 0.05 and action[7] > 0.5:
                    block["grasped"] = True
                    block["pos"] = self.ee_pos.copy()

        #
        self._check_stacking()

        #
        reward = self._compute_reward()

        #
        terminated = False
        truncated = self.stacked_count >= self.num_blocks

        obs = self._get_observation()
        info = {"stacked": self.stacked_count}

        return obs, reward, terminated, truncated, info

    def _check_stacking(self):
        """"""
        for i, block in enumerate(self.blocks):
            if block["grasped"] and not block["stacked"]:
                #
                for j, other in enumerate(self.blocks):
                    if i != j and other["stacked"]:
                        dist = np.linalg.norm(block["pos"][:2] - other["pos"][:2])
                        if dist < self.block_size and block["pos"][2] > other["pos"][2]:
                            block["stacked"] = True
                            self.stacked_count += 1

    def _get_observation(self) -> np.ndarray:
        obs = [*self.joint_pos, *np.zeros(7), *self.ee_pos]

        #
        for block in self.blocks:
            obs.extend(block["pos"])
            obs.append(1.0 if block["stacked"] else 0.0)

        #
        while len(obs) < 50:
            obs.append(0.0)

        return np.array(obs[:50], dtype=np.float32)

    def _compute_reward(self) -> float:
        stacking_reward = self.stacked_count * 10.0
        return stacking_reward


if __name__ == "__main__":
    gym.register(id="BlockStacking-v0", entry_point="__main__:BlockStackingEnv")

    env = BlockStackingEnv(num_blocks=3)
    print("  Demo")

    for episode in range(3):
        obs, info = env.reset()
        total_reward = 0

        for step in range(300):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward

            if terminated or truncated:
                break

        status = " " if info["stacked"] == 3 else f"⏱ {info['stacked']}/3"
        print(f"Episode {episode + 1}: {status}, Reward={total_reward:.2f}")
