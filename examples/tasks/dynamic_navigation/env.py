"""
:  (Dynamic Navigation)
:
:  ()
: /
"""

import sys
import os

sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
)

import gymnasium as gym
import numpy as np


class DynamicNavigationEnv(gym.Env):
    """()"""

    def __init__(self):
        super().__init__()
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(370,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )

        self.robot_pos = np.zeros(2)
        self.goal_pos = np.zeros(2)
        self.obstacles = []

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_pos = np.zeros(2)
        self.goal_pos = np.random.uniform(-5, 5, 2)
        #
        self.obstacles = [
            {"pos": np.random.uniform(-5, 5, 2), "vel": np.random.uniform(-0.5, 0.5, 2)}
            for _ in range(10)
        ]
        return self._get_observation(), {}

    def step(self, action):
        self.robot_pos += action * 0.1
        #
        for obs in self.obstacles:
            obs["pos"] += obs["vel"] * 0.1

        dist = np.linalg.norm(self.robot_pos - self.goal_pos)
        reward = -dist
        done = dist < 0.5

        return self._get_observation(), reward, False, done, {"distance": dist}

    def _get_observation(self):
        return np.concatenate([self.robot_pos, self.goal_pos, np.zeros(366)]).astype(
            np.float32
        )


if __name__ == "__main__":
    gym.register(id="DynamicNavigation-v0", entry_point="__main__:DynamicNavigationEnv")
    env = DynamicNavigationEnv()
    print("Dynamic navigation task demo")
    obs, _ = env.reset()
    for _ in range(200):
        action = env.action_space.sample()
        obs, reward, _, done, info = env.step(action)
        if done:
            break
    print(f"Demo completed, distance={info['distance']:.2f}m")
