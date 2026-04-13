"""
:  (Obstacle Avoidance)

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
from typing import Dict, Tuple, List


class ObstacleAvoidanceEnv(gym.Env):
    """


    :
        -  (/)
        -
        -

    :
        - / (6D)
        -  (360D: 360, 1)
        -  (2D)

    :
        -  (1D)
        -  (1D)

    :
        - : +1.0 * (distance_decreased)
        - : +10.0
        - : -5.0
        - : -0.01
    """

    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, render_mode=None, num_obstacles=5):
        super().__init__()

        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(368,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0]), high=np.array([1.0, 1.0]), dtype=np.float32
        )

        #
        self.num_obstacles = num_obstacles
        self.arena_size = 10.0  # 10m x 10m
        self.obstacle_radius = 0.3
        self.robot_radius = 0.2
        self.goal_threshold = 0.5

        #
        self.lidar_range = 5.0
        self.lidar_resolution = 360  # 1

        #
        self.robot_pos = np.zeros(2)
        self.robot_vel = np.zeros(2)
        self.robot_theta = 0.0
        self.goal_pos = np.zeros(2)
        self.obstacles: List[Dict] = []
        self.prev_distance = 0.0

        self.render_mode = render_mode

    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        super().reset(seed=seed)

        #
        self.robot_pos = np.array([0.0, 0.0])
        self.robot_vel = np.zeros(2)
        self.robot_theta = 0.0

        #
        self.goal_pos = np.random.uniform(
            -self.arena_size / 2, self.arena_size / 2, size=2
        )
        self.prev_distance = np.linalg.norm(self.goal_pos - self.robot_pos)

        #
        self._spawn_obstacles()

        obs = self._get_observation()
        info = {"distance_to_goal": self.prev_distance}

        return obs, info

    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        #
        linear_vel = action[0] * 0.5  #  0.5 m/s
        angular_vel = action[1] * 1.0  #  1.0 rad/s

        #
        dt = 0.1
        self.robot_theta += angular_vel * dt
        self.robot_pos[0] += linear_vel * np.cos(self.robot_theta) * dt
        self.robot_pos[1] += linear_vel * np.sin(self.robot_theta) * dt

        #
        self._update_obstacles(dt)

        #
        collision = self._check_collision()

        #
        current_distance = np.linalg.norm(self.goal_pos - self.robot_pos)
        reward = self._compute_reward(current_distance, collision)
        self.prev_distance = current_distance

        #
        terminated = collision
        truncated = current_distance < self.goal_threshold

        obs = self._get_observation()
        info = {
            "distance_to_goal": current_distance,
            "collision": collision,
            "reached_goal": truncated,
        }

        return obs, reward, terminated, truncated, info

    def _spawn_obstacles(self):
        """"""
        self.obstacles = []
        for _ in range(self.num_obstacles):
            pos = np.random.uniform(-self.arena_size / 2, self.arena_size / 2, size=2)
            #
            while (
                np.linalg.norm(pos - self.robot_pos) < 1.0
                or np.linalg.norm(pos - self.goal_pos) < 1.0
            ):
                pos = np.random.uniform(
                    -self.arena_size / 2, self.arena_size / 2, size=2
                )

            # 50%
            is_dynamic = np.random.rand() < 0.5
            velocity = (
                np.random.uniform(-0.2, 0.2, size=2) if is_dynamic else np.zeros(2)
            )

            self.obstacles.append(
                {"pos": pos, "vel": velocity, "radius": self.obstacle_radius}
            )

    def _update_obstacles(self, dt: float):
        """"""
        for obs in self.obstacles:
            obs["pos"] += obs["vel"] * dt

            #
            for i in range(2):
                if abs(obs["pos"][i]) > self.arena_size / 2:
                    obs["vel"][i] *= -1

    def _get_lidar_scan(self) -> np.ndarray:
        """"""
        scan = np.full(self.lidar_resolution, self.lidar_range)

        for i in range(self.lidar_resolution):
            angle = np.deg2rad(i) + self.robot_theta
            ray_dir = np.array([np.cos(angle), np.sin(angle)])

            #
            for obs in self.obstacles:
                vec_to_obs = obs["pos"] - self.robot_pos
                proj = np.dot(vec_to_obs, ray_dir)

                if proj > 0:
                    dist_to_ray = np.linalg.norm(vec_to_obs - proj * ray_dir)
                    if dist_to_ray < obs["radius"]:
                        scan[i] = min(scan[i], proj)

        return scan

    def _check_collision(self) -> bool:
        """"""
        for obs in self.obstacles:
            dist = np.linalg.norm(self.robot_pos - obs["pos"])
            if dist < (self.robot_radius + obs["radius"]):
                return True
        return False

    def _get_observation(self) -> np.ndarray:
        lidar_scan = self._get_lidar_scan()
        goal_relative = self.goal_pos - self.robot_pos

        obs = np.concatenate(
            [
                self.robot_pos,  # 2D
                self.robot_vel,  # 2D
                [self.robot_theta],  # 1D
                [np.linalg.norm(goal_relative)],  # 1D ()
                goal_relative,  # 2D
                lidar_scan,  # 360D
            ]
        )
        return obs.astype(np.float32)

    def _compute_reward(self, current_distance: float, collision: bool) -> float:
        #
        approach_reward = (self.prev_distance - current_distance) * 10.0

        #
        goal_reward = 10.0 if current_distance < self.goal_threshold else 0.0

        #
        collision_penalty = -5.0 if collision else 0.0

        #
        time_penalty = -0.01

        return approach_reward + goal_reward + collision_penalty + time_penalty


if __name__ == "__main__":
    gym.register(id="ObstacleAvoidance-v0", entry_point="__main__:ObstacleAvoidanceEnv")

    env = ObstacleAvoidanceEnv(num_obstacles=5)

    print("  Demo")
    for episode in range(5):
        obs, info = env.reset()
        total_reward = 0

        for step in range(500):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward

            if terminated or truncated:
                break

        status = " " if info["reached_goal"] else (" " if info["collision"] else "⏱ ")
        print(
            f"Episode {episode + 1}: {status}, Distance={info['distance_to_goal']:.2f}m, Reward={total_reward:.2f}"
        )
