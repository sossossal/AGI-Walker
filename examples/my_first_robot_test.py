"""
测试训练好的机器人
观察机器人的实际表现
"""

import gymnasium as gym
from stable_baselines3 import PPO
import time

print("="*70)
print("  测试我的机器人")
print("="*70)

# ============ 加载模型 ============
print("\n[1/3] 加载训练好的模型...")
try:
    model = PPO.load("my_robot_models/my_robot_final")
    print("  ✓ 模型加载成功！")
except FileNotFoundError:
    print("  ✗ 错误: 未找到模型文件")
    print("  请先运行 'python my_first_robot_train.py' 训练模型")
    exit(1)

# ============ 创建环境 ============
print("\n[2/3] 创建测试环境...")
print("  模式: 人类可视化 (如果支持)")

try:
    env = gym.make('AGI-Walker/Walker2D-v0', render_mode="human")
except:
    # 如果不支持渲染，使用普通模式
    env = gym.make('AGI-Walker/Walker2D-v0')
    print("  注意: 当前环境不支持可视化渲染")

print("  ✓ 环境创建成功！")

# ============ 测试回合 ============
print("\n[3/3] 开始测试...")
print("-"*70)

n_episodes = 5
episode_rewards = []
episode_lengths = []

for episode in range(n_episodes):
    obs, _ = env.reset()
    done = False
    episode_reward = 0
    step_count = 0
    
    print(f"\n回合 {episode + 1}/{n_episodes}:")
    print("  前进中", end="", flush=True)
    
    while not done and step_count < 1000:
        # 使用训练好的策略预测动作
        action, _states = model.predict(obs, deterministic=True)
        
        # 执行动作
        obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated
        
        episode_reward += reward
        step_count += 1
        
        # 每100步显示一个点
        if step_count % 100 == 0:
            print(".", end="", flush=True)
        
        # 减速以便观察（如果有渲染）
        time.sleep(0.01)
    
    episode_rewards.append(episode_reward)
    episode_lengths.append(step_count)
    
    print()
    print(f"  结果:")
    print(f"    - 步数: {step_count}")
    print(f"    - 奖励: {episode_reward:.2f}")
    
    if step_count >= 1000:
        print(f"    - 状态: ✓ 成功完成1000步！")
    else:
        print(f"    - 状态: 提前终止")

env.close()

# ============ 统计结果 ============
import numpy as np

print("\n"+"="*70)
print("  测试结果统计")
print("="*70)
print(f"  回合数: {n_episodes}")
print(f"  平均奖励: {np.mean(episode_rewards):.2f} ± {np.std(episode_rewards):.2f}")
print(f"  平均步数: {np.mean(episode_lengths):.1f}")
print(f"  最高奖励: {np.max(episode_rewards):.2f}")
print(f"  最低奖励: {np.min(episode_rewards):.2f}")
print("-"*70)

# 性能评估
avg_reward = np.mean(episode_rewards)
if avg_reward > 200:
    rating = "🌟🌟🌟 优秀！"
    comment = "机器人已经学会稳定行走"
elif avg_reward > 100:
    rating = "⭐⭐ 良好"
    comment = "机器人表现不错，可以继续训练提升"
elif avg_reward > 50:
    rating = "⭐ 及格"
    comment = "机器人初步掌握了平衡，建议增加训练时间"
else:
    rating = "需改进"
    comment = "建议重新训练或调整参数"

print(f"\n  性能评级: {rating}")
print(f"  评价: {comment}")
print("="*70)

# 建议
print("\n💡 改进建议:")
if avg_reward < 150:
    print("  1. 增加训练步数 (例如: 200,000 或 500,000)")
    print("  2. 尝试调整学习率")
    print("  3. 使用离线RL或模仿学习加速训练")
else:
    print("  ✓ 当前性能已经很好!")
    print("  可以尝试:")
    print("    - 四足机器人: examples/quadruped_training.py")
    print("    - 更复杂的环境")
    print("    - 部署到真实硬件")

print("\n查看训练曲线:")
print("  tensorboard --logdir=./my_robot_logs/")
print("  访问: http://localhost:6006")
print("="*70)
