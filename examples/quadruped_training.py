"""
四足机器人训练示例
使用 PPO 训练 Trot 步态
"""

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

# 导入四足环境
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from python_api.quadruped_env import QuadrupedEnv


def make_env():
    """创建环境"""
    return gym.make('AGI-Walker/Quadruped-v0')


def train_quadruped(total_timesteps=1_000_000, save_freq=50_000):
    """
    训练四足机器人
    
    参数:
        total_timesteps: 总训练步数
        save_freq: 保存频率
    """
    print("="*60)
    print("四足机器人 PPO 训练")
    print("="*60)
    
    # 创建环境
    env = DummyVecEnv([make_env])
    eval_env = DummyVecEnv([make_env])
    
    # 保存路径
    save_dir = "models/quadruped"
    os.makedirs(save_dir, exist_ok=True)
    
    # 回调函数
    checkpoint_callback = CheckpointCallback(
        save_freq=save_freq,
        save_path=save_dir,
        name_prefix='ppo_quadruped'
    )
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=save_dir,
        log_path= save_dir,
        eval_freq=10_000,
        deterministic=True,
        render=False
    )
    
    # 创建 PPO 模型
    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,  # 增加探索
        verbose=1,
        tensorboard_log="./tensorboard_logs/quadruped"
    )
    
    print(f"\n开始训练 (总步数: {total_timesteps:,})")
    print(f"模型将保存至: {save_dir}")
    print(f"TensorBoard: tensorboard --logdir=./tensorboard_logs/quadruped\n")
    
    # 训练
    model.learn(
        total_timesteps=total_timesteps,
        callback=[checkpoint_callback, eval_callback],
        progress_bar=True
    )
    
    # 保存最终模型
    final_path = os.path.join(save_dir, "ppo_quadruped_final.zip")
    model.save(final_path)
    print(f"\n训练完成！模型已保存: {final_path}")
    
    env.close()
    eval_env.close()
    
    return model


def test_trained_model(model_path, n_episodes=5):
    """
    测试训练好的模型
    
    参数:
        model_path: 模型路径
        n_episodes: 测试回合数
    """
    print("="*60)
    print("测试训练好的四足机器人")
    print("="*60)
    
    # 加载模型
    model = PPO.load(model_path)
    print(f"已加载模型: {model_path}\n")
    
    # 创建环境
    env = gym.make('AGI-Walker/Quadruped-v0', render_mode="human")
    
    episode_rewards = []
    
    for episode in range(n_episodes):
        obs, _ = env.reset()
        done = False
        episode_reward = 0
        step = 0
        
        print(f"回合 {episode + 1}/{n_episodes}:")
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            
            episode_reward += reward
            step += 1
        
        episode_rewards.append(episode_reward)
        forward_vel = info.get('forward_velocity', 0)
        
        print(f"  步数: {step}")
        print(f"  总奖励: {episode_reward:.2f}")
        print(f"  前进速度: {forward_vel:.2f} m/s\n")
    
    env.close()
    
    print("="*60)
    print(f"平均奖励: {sum(episode_rewards)/len(episode_rewards):.2f}")
    print(f"最大奖励: {max(episode_rewards):.2f}")
    print(f"最小奖励: {min(episode_rewards):.2f}")
    print("="*60)


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='四足机器人训练示例')
    parser.add_argument('--train', action='store_true', help='训练模型')
    parser.add_argument('--test', type=str, help='测试模型路径')
    parser.add_argument('--timesteps', type=int, default=1_000_000, help='训练步数')
    
    args = parser.parse_args()
    
    if args.train:
        train_quadruped(total_timesteps=args.timesteps)
    elif args.test:
        test_trained_model(args.test)
    else:
        print("使用方法:")
        print("  训练: python quadruped_training.py --train")
        print("  测试: python quadruped_training.py --test models/quadruped/ppo_quadruped_final.zip")
