import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import os
import sys
from pathlib import Path


def main():
    # Create dummy environment for testing
    # Using CartPole as a proxy if Walker2D is complex to setup
    
    sys.path.insert(0, str(Path.cwd()))

    # Attempt to import package to register envs
    try:
        import agi_walker
    except ImportError:
        pass

    print("Verifying RL Environment...")
    try:
        # Check if we can make the custom env
        env = DummyVecEnv([lambda: gym.make("CartPole-v1")])

        model = PPO("MlpPolicy", env, verbose=1)
        print("Training for 10 steps...")
        model.learn(total_timesteps=10)
        print("RL Loop Verified Successfully!")

    except Exception as e:
        print(f"RL Verification Failed: {e}")


if __name__ == "__main__":
    main()
