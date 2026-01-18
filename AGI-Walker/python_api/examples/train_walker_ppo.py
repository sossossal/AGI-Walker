"""
使用 PPO 算法训练步行机器人

需要先启动 Godot 仿真器！
"""
import sys
import os
from pathlib import Path

# 添加父目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from godot_robot_env import GodotRobotEnv, PartsDatabase
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import DummyVecEnv
import argparse


def create_env():
    """创建环境实例"""
    # 从零件库创建机器人配置
    parts_db = PartsDatabase()
    
    robot_config = parts_db.create_robot_config([
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_left"},
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_right"},
        {"part_id": "dynamixel_xl430_w250", "joint": "knee_left"},
        {"part_id": "dynamixel_xl430_w250", "joint": "knee_right"},
    ])
    
    # 物理环境配置
    physics_config = {
        "gravity": 9.81,
        "ground_friction": 0.8,
        "air_density": 1.225
    }
    
    env = GodotRobotEnv(
        robot_config=robot_config,
        physics_config=physics_config,
        host="127.0.0.1",
        port=9999
    )
    
    return env


def train_walker(
    total_timesteps: int = 100000,
    save_path: str = "./models/walker_ppo",
    log_path: str = "./logs/walker_ppo"
):
    """
    训练步行机器人
    
    Args:
        total_timesteps: 总训练步数
        save_path: 模型保存路径
        log_path: 日志路径
    """
    print("=" * 50)
    print("训练步行机器人 - PPO 算法")
    print("=" * 50)
    
    # 创建环境
    print("\n1. 创建环境...")
    env = create_env()
    
    # 测试连接
    print("2. 连接到 Godot 仿真器...")
    if not env.connect():
        print("❌ 无法连接到 Godot！请确保：")
        print("   1. Godot 项目已打开")
        print("   2. 按 F5 运行主场景")
        print("   3. TCP 服务器正在运行（端口 9999）")
        return
    
    # 包装为向量化环境（Stable-Baselines3 需要）
    vec_env = DummyVecEnv([lambda: env])
    
    # 创建 PPO 模型
    print("3. 创建 PPO 模型...")
    model = PPO(
        policy="MultiInputPolicy",  # 使用多输入策略（Dict observation）
        env=vec_env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        verbose=1,
        tensorboard_log=log_path
    )
    
    # 设置回调
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=save_path,
        name_prefix="walker_ppo"
    )
    
    # 开始训练
    print(f"\n4. 开始训练（{total_timesteps} 步）...")
    print("=" * 50)
    print("提示：")
    print("  - 在 Godot 中可以实时观察机器人学习过程")
    print("  - 使用 TensorBoard 查看训练曲线")
    print("  - 按 Ctrl+C 可以安全中断训练")
    print("=" * 50)
    print()
    
    try:
        model.learn(
            total_timesteps=total_timesteps,
            callback=checkpoint_callback,
            progress_bar=True
        )
        
        # 保存最终模型
        final_model_path = os.path.join(save_path, "walker_ppo_final")
        model.save(final_model_path)
        print(f"\n✅ 训练完成！模型已保存到: {final_model_path}")
        
    except KeyboardInterrupt:
        print("\n⚠️  训练被中断")
        final_model_path = os.path.join(save_path, "walker_ppo_interrupted")
        model.save(final_model_path)
        print(f"模型已保存到: {final_model_path}")
    
    finally:
        env.close()


def test_model(model_path: str, episodes: int = 10):
    """
    测试训练好的模型
    
    Args:
        model_path: 模型文件路径
        episodes: 测试回合数
    """
    print("=" * 50)
    print(f"测试模型: {model_path}")
    print("=" * 50)
    
    # 加载模型
    print("\n加载模型...")
    model = PPO.load(model_path)
    
    # 创建环境
    env = create_env()
    env.connect()
    
    print(f"\n开始测试 {episodes} 个回合...")
    
    total_rewards = []
    
    for episode in range(episodes):
        obs, info = env.reset()
        episode_reward = 0
        steps = 0
        done = False
        
        print(f"\n回合 {episode + 1}/{episodes}")
        
        while not done:
            # 使用模型预测动作
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            
            episode_reward += reward
            steps += 1
            done = terminated or truncated
            
            # 打印进度
            if steps % 100 == 0:
                print(f"  步数: {steps}, 累积奖励: {episode_reward:.2f}")
        
        total_rewards.append(episode_reward)
        print(f"  ✓ 回合结束 - 总步数: {steps}, 总奖励: {episode_reward:.2f}")
    
    # 统计
    import numpy as np
    print("\n" + "=" * 50)
    print("测试结果统计:")
    print(f"  平均奖励: {np.mean(total_rewards):.2f} ± {np.std(total_rewards):.2f}")
    print(f"  最佳奖励: {np.max(total_rewards):.2f}")
    print(f"  最差奖励: {np.min(total_rewards):.2f}")
    print("=" * 50)
    
    env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="训练或测试步行机器人")
    parser.add_argument("--mode", choices=["train", "test"], default="train",
                       help="模式：train=训练, test=测试")
    parser.add_argument("--timesteps", type=int, default=100000,
                       help="训练步数（默认：100000）")
    parser.add_argument("--model-path", type=str, default="./models/walker_ppo/walker_ppo_final",
                       help="模型路径（测试模式时使用）")
    parser.add_argument("--episodes", type=int, default=10,
                       help="测试回合数（默认：10）")
    
    args = parser.parse_args()
    
    if args.mode == "train":
        # 训练模式
        train_walker(total_timesteps=args.timesteps)
    else:
        # 测试模式
        test_model(model_path=args.model_path, episodes=args.episodes)
