"""
训练脚本 - 楼梯攀爬任务
使用 PPO 算法训练预训练模型
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import gymnasium as gym
import numpy as np
from datetime import datetime
import json

# 尝试导入 stable-baselines3
try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import DummyVecEnv
    from stable_baselines3.common.callbacks import BaseCallback
    SB3_AVAILABLE = True
except ImportError:
    SB3_AVAILABLE = False
    print("⚠️ stable-baselines3 未安装")
    print("安装: pip install stable-baselines3")


class TrainingCallback(BaseCallback):
    """训练回调 - 记录进度"""
    
    def __init__(self, save_freq=10000, save_path="./models/"):
        super().__init__()
        self.save_freq = save_freq
        self.save_path = save_path
        self.episode_rewards = []
        self.episode_lengths = []
        
        os.makedirs(save_path, exist_ok=True)
    
    def _on_step(self) -> bool:
        # 记录 episode 信息
        if len(self.model.ep_info_buffer) > 0:
            for info in self.model.ep_info_buffer:
                self.episode_rewards.append(info['r'])
                self.episode_lengths.append(info['l'])
        
        # 定期保存
        if self.n_calls % self.save_freq == 0:
            model_path = os.path.join(
                self.save_path,
                f"checkpoint_{self.n_calls}.zip"
            )
            self.model.save(model_path)
            print(f"💾 模型已保存: {model_path}")
        
        return True


def train_stair_climbing(
    total_timesteps=1000000,
    save_path="./models/stair_climbing/"
):
    """
    训练楼梯攀爬任务
    
    Args:
        total_timesteps: 总训练步数
        save_path: 模型保存路径
    """
    
    if not SB3_AVAILABLE:
        print("❌ 无法训练: stable-baselines3 未安装")
        return None
    
    print("\n🚀 开始训练: 楼梯攀爬任务")
    print("="*60)
    print(f"总步数: {total_timesteps:,}")
    print(f"算法: PPO")
    print(f"保存路径: {save_path}")
    print("="*60)
    
    # 创建环境
    from examples.tasks.stair_climbing.env import StairClimbingEnv
    
    gym.register(
        id='StairClimbing-Train',
        entry_point=StairClimbingEnv,
        max_episode_steps=1000
    )
    
    env = DummyVecEnv([lambda: gym.make('StairClimbing-Train')])
    
    # 创建模型
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        tensorboard_log="./logs/stair_climbing/"
    )
    
    # 创建回调
    callback = TrainingCallback(
        save_freq=50000,
        save_path=save_path
    )
    
    # 开始训练
    start_time = datetime.now()
    
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=callback,
            progress_bar=True
        )
    except KeyboardInterrupt:
        print("\n⚠️ 训练被中断")
    
    # 保存最终模型
    final_path = os.path.join(save_path, "final_model.zip")
    model.save(final_path)
    
    # 保存训练统计
    stats = {
        "task": "stair_climbing",
        "algorithm": "PPO",
        "total_timesteps": total_timesteps,
        "training_time": str(datetime.now() - start_time),
        "avg_episode_reward": np.mean(callback.episode_rewards[-100:]) if callback.episode_rewards else 0,
        "avg_episode_length": np.mean(callback.episode_lengths[-100:]) if callback.episode_lengths else 0,
        "final_model_path": final_path
    }
    
    stats_path = os.path.join(save_path, "training_stats.json")
    with open(stats_path, 'w') as f:
        json.dump(stats, f, indent=2)
    
    print("\n" + "="*60)
    print("✅ 训练完成!")
    print(f"最终模型: {final_path}")
    print(f"平均奖励: {stats['avg_episode_reward']:.2f}")
    print(f"平均步数: {stats['avg_episode_length']:.0f}")
    print("="*60)
    
    return model, stats


def evaluate_model(model_path, num_episodes=10):
    """评估模型"""
    if not SB3_AVAILABLE:
        print("❌ 无法评估: stable-baselines3 未安装")
        return
    
    print(f"\n📊 评估模型: {model_path}")
    
    from examples.tasks.stair_climbing.env import StairClimbingEnv
    
    model = PPO.load(model_path)
    env = StairClimbingEnv()
    
    rewards = []
    success_count = 0
    
    for episode in range(num_episodes):
        obs, info = env.reset()
        total_reward = 0
        steps = 0
        
        while True:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            steps += 1
            
            if terminated or truncated:
                break
        
        rewards.append(total_reward)
        if info.get('steps_climbed', 0) >= 5:
            success_count += 1
        
        print(f"Episode {episode+1}: Reward={total_reward:.2f}, "
              f"Steps Climbed={info.get('steps_climbed', 0)}/5")
    
    print(f"\n评估结果:")
    print(f"  平均奖励: {np.mean(rewards):.2f} ± {np.std(rewards):.2f}")
    print(f"  成功率: {success_count}/{num_episodes} ({success_count/num_episodes*100:.0f}%)")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['train', 'eval'], default='train')
    parser.add_argument('--timesteps', type=int, default=100000)
    parser.add_argument('--model', type=str, default='./models/stair_climbing/final_model.zip')
    args = parser.parse_args()
    
    if args.mode == 'train':
        train_stair_climbing(total_timesteps=args.timesteps)
    else:
        evaluate_model(args.model)
