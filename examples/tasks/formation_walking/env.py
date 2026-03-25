"""
:  (Formation Walking)
: 
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
from typing import Dict, Tuple

class FormationWalkingEnv(gym.Env):
    """"""

    def __init__(self, num_robots=3):
        super().__init__()

        self.num_robots = num_robots
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(num_robots * 10,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(num_robots * 8,), dtype=np.float32
        )

        self.robot_positions = np.zeros((num_robots, 3))
        self.target_formation = np.array([[0, 0, 0], [1, 0, 0], [0.5, 1, 0]])  # 

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_positions = np.random.randn(self.num_robots, 3) * 0.5
        return self._get_observation(), {}

    def step(self, action):
        for i in range(self.num_robots):
            self.robot_positions[i, :2] += action[i * 8 : i * 8 + 2] * 0.01

        reward = -np.sum(
            [
                np.linalg.norm(self.robot_positions[i] - self.target_formation[i])
                for i in range(self.num_robots)
            ]
        )

        return self._get_observation(), reward, False, False, {}

    def _get_observation(self):
        return self.robot_positions.flatten()[: self.observation_space.shape[0]].astype(
            np.float32
        )

if __name__ == "__main__":
    gym.register(id="FormationWalking-v0", entry_point="__main__:FormationWalkingEnv")
    env = FormationWalkingEnv()
    print("Formation walking task demo")
    obs, _ = env.reset()
    for _ in range(100):
        action = env.action_space.sample()
        obs, reward, _, _, _ = env.step(action)
    print(f"Demo completed, final reward={reward:.2f}")
