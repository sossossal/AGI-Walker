"""
测试改进的策略
"""

import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

def test_policy(model_path="models/improved_ppo_v1", episodes=10):
    """
    测试训练好的策略
    
    参数:
        model_path: 模型路径
        episodes: 测试回合数
    """
    
    # 加载模型和环境
    model = PPO.load(f"{model_path}/final_model")
    
    # 创建测试环境
    env = DummyVecEnv([lambda: gym.make("AGI-Walker/Walker2D-v0")])
    env = VecNormalize.load(f"{model_path}/vec_normalize.pkl", env)
    env.training = False  # 测试模式
    env.norm_reward = False
    
    # 测试
    episode_rewards = []
    episode_lengths = []
    
    print("\n" + "="*50)
    print("测试改进的策略")
    print("="*50 + "\n")
    
    for episode in range(episodes):
        obs = env.reset()
        done = False
        episode_reward = 0
        episode_length = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward[0]
            episode_length += 1
            
            if done[0]:
                break
        
        episode_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
        
        print(f"Episode {episode+1}/{episodes}: "
              f"Reward = {episode_reward:.2f}, "
              f"Length = {episode_length}")
    
    # 统计
    print("\n" + "="*50)
    print("测试结果统计")
    print("="*50)
    print(f"平均奖励: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
    print(f"平均长度: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f}")
    print(f"最佳奖励: {np.max(episode_rewards):.2f}")
    print(f"最差奖励: {np.min(episode_rewards):.2f}")
    print("="*50 + "\n")
    
    return episode_rewards, episode_lengths

if __name__ == "__main__":
    test_policy(model_path="models/improved_ppo_v1", episodes=10)
