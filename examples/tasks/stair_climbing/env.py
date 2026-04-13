"""
:  (Stair Climbing)

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
from typing import Dict, Tuple


class StairClimbingEnv(gym.Env):
    """


    :
        -  (8D)
        -  (8D)
        -  (4D: )
        -  (3D)
        -  (1D)

    :
        -  (8D)

    :
        - : +1.0 per meter
        - : +0.5 per step climbed
        - : -0.1 * |roll| - 0.1 * |pitch|
        - : -0.01 * sum(torque^2)
        - : -10.0
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, render_mode=None):
        super().__init__()

        #
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(24,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(8,), dtype=np.float32
        )

        #
        self.stair_height = 0.15  #  15cm
        self.stair_depth = 0.30  #  30cm
        self.num_stairs = 5  #

        #
        self.robot_pos = np.zeros(3)
        self.robot_vel = np.zeros(3)
        self.joint_pos = np.zeros(8)
        self.joint_vel = np.zeros(8)
        self.steps_climbed = 0

        self.render_mode = render_mode

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)

        #
        self.robot_pos = np.array([0.0, 0.0, 0.5])  #
        self.robot_vel = np.zeros(3)
        self.joint_pos = np.zeros(8)
        self.joint_vel = np.zeros(8)
        self.steps_climbed = 0

        obs = self._get_observation()
        info = {"steps_climbed": 0}

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        #  (,)
        self.joint_pos += action * 0.1
        self.robot_pos[0] += 0.01  # :

        #
        current_stair = int(self.robot_pos[0] / self.stair_depth)
        if current_stair > self.steps_climbed:
            self.steps_climbed = current_stair
            self.robot_pos[2] += self.stair_height

        #
        reward = self._compute_reward(action)

        #
        terminated = self._check_terminated()
        truncated = self.steps_climbed >= self.num_stairs

        obs = self._get_observation()
        info = {"steps_climbed": self.steps_climbed, "distance": self.robot_pos[0]}

        return obs, reward, terminated, truncated, info

    def _get_observation(self) -> np.ndarray:
        """"""
        #
        obs = np.concatenate(
            [
                self.joint_pos,  # 8D
                self.joint_vel,  # 8D
                self.robot_pos,  # 3D
                self.robot_vel,  # 3D
                [self.steps_climbed / self.num_stairs],  # 1D ()
                [0.0],  # 1D ( 24D)
            ]
        )
        return obs.astype(np.float32)

    def _compute_reward(self, action: np.ndarray) -> float:
        """"""
        #
        forward_reward = self.robot_pos[0] * 0.1

        #
        climb_reward = self.steps_climbed * 0.5

        #
        energy_cost = -0.01 * np.sum(action**2)

        #  (: )
        stability_reward = 0.1

        return forward_reward + climb_reward + energy_cost + stability_reward

    def _check_terminated(self) -> bool:
        """"""
        # :
        if self.robot_pos[2] < 0.2:
            return True
        return False

    def render(self):
        if self.render_mode == "human":
            print(
                f"Steps: {self.steps_climbed}/{self.num_stairs}, "
                f"Pos: {self.robot_pos[0]:.2f}m"
            )


# ====================  ====================


def train_stair_climbing():
    """"""
    print("\n: ")
    print("=" * 60)

    #
    env = StairClimbingEnv()

    #  ()
    print("\n (10 episodes)...")

    for episode in range(10):
        obs, info = env.reset()
        total_reward = 0

        for step in range(200):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward

            if terminated or truncated:
                break

        print(
            f"Episode {episode + 1}: "
            f"Steps Climbed={info['steps_climbed']}, "
            f"Reward={total_reward:.2f}"
        )

    print("\n!")
    print(":  PPO ")


if __name__ == "__main__":
    #
    gym.register(
        id="StairClimbing-v0",
        entry_point="__main__:StairClimbingEnv",
    )

    #
    train_stair_climbing()
