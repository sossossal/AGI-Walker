"""
:  (Slope Walking)
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

class SlopeWalkingEnv(gym.Env):
    """"""

    def __init__(self, slope_angle=15.0):
        super().__init__()
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(25,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )

        self.slope_angle = np.deg2rad(slope_angle)
        self.robot_pos = np.zeros(3)
        self.joint_pos = np.zeros(8)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.robot_pos = np.array([0, 0, 0.5])
        self.joint_pos = np.zeros(8)
        return self._get_observation(), {}

    def step(self, action):
        self.joint_pos += action * 0.1
        self.robot_pos[0] += 0.02  # 
        self.robot_pos[2] = 0.5 + self.robot_pos[0] * np.tan(
            self.slope_angle
        )  # 

        reward = 0.1  # 
        done = self.robot_pos[0] >= 10.0  # 10

        return (
            self._get_observation(),
            reward,
            False,
            done,
            {"distance": self.robot_pos[0]},
        )

    def _get_observation(self):
        return np.concatenate(
            [self.joint_pos, np.zeros(8), self.robot_pos, np.zeros(6)]
        ).astype(np.float32)

if __name__ == "__main__":
    gym.register(id="SlopeWalking-v0", entry_point="__main__:SlopeWalkingEnv")
    env = SlopeWalkingEnv(slope_angle=15)
    print("Slope walking task demo")
    obs, _ = env.reset()
    for _ in range(500):
        action = env.action_space.sample()
        obs, reward, _, done, info = env.step(action)
        if done:
            break
    print(f"Demo completed, distance={info['distance']:.2f}m")
