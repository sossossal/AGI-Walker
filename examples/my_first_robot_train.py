"""
我的第一个机器人训练项目
这是一个完整的可运行示例，展示从零开始创建和训练机器人的全过程
"""

import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
import os

# 创建保存目录
os.makedirs("my_robot_models", exist_ok=True)
os.makedirs("my_robot_logs", exist_ok=True)

print("=" * 70)
print("  欢迎来到 AGI-Walker！")
print("  我的第一个机器人训练项目")
print("=" * 70)

# ============ 第一步：创建环境 ============
print("\n[步骤 1/5] 创建训练环境...")
print("  环境类型: Walker2D (双足行走)")
print("  观测维度: 17维 (关节角度、速度、身体状态)")
print("  动作维度: 6维 (6个关节的扭矩控制)")

env = DummyVecEnv([lambda: gym.make("AGI-Walker/Walker2D-v0")])
eval_env = DummyVecEnv([lambda: gym.make("AGI-Walker/Walker2D-v0")])

print("  Environment created successfully")

# ============ 第二步：创建模型 ============
print("\n[步骤 2/5] 创建 PPO 强化学习模型...")
print("  算法: PPO (Proximal Policy Optimization)")
print("  策略网络: 多层感知机 (64-64)")
print("  学习率: 3e-4")

model = PPO(
    "MlpPolicy",
    env,
    learning_rate=3e-4,
    n_steps=2048,  # 每次收集2048步经验
    batch_size=64,  # 每批64个样本
    n_epochs=10,  # 每次更新训练10轮
    gamma=0.99,  # 折扣因子
    gae_lambda=0.95,  # GAE参数
    clip_range=0.2,  # PPO剪切范围
    verbose=1,  # 显示详细信息
    tensorboard_log="./my_robot_logs/",
)

print("  Model created successfully")
print(f"  策略参数总数: {sum(p.numel() for p in model.policy.parameters()):,}")

# ============ 第三步：设置回调 ============
print("\n[步骤 3/5] 设置训练回调...")

# 定期保存模型
checkpoint_callback = CheckpointCallback(
    save_freq=10000, save_path="./my_robot_models/", name_prefix="checkpoint"
)

# 定期评估
eval_callback = EvalCallback(
    eval_env,
    best_model_save_path="./my_robot_models/",
    log_path="./my_robot_logs/",
    eval_freq=5000,
    n_eval_episodes=5,
    deterministic=True,
    render=False,
)

print("  Callbacks configured")
print("    - 每 10,000 步保存检查点")
print("    - 每 5,000 步评估一次")

# ============ 第四步：开始训练 ============
print("\n[步骤 4/5] 开始训练...")
print("-" * 70)
print("  训练参数:")
print("    总训练步数: 100,000")
print("    预计时间: 10-15分钟 (取决于硬件)")
print("    日志目录: ./my_robot_logs/")
print("    模型保存: ./my_robot_models/")
print("-" * 70)
print("\n  Tips:")
print("    - 训练过程中可以按 Ctrl+C 暂停")
print("    - 在另一个终端运行 'tensorboard --logdir=./my_robot_logs/' 实时查看训练曲线")
print("    - 然后访问 http://localhost:6006\n")

input("按回车键开始训练...")

try:
    model.learn(
        total_timesteps=100_000,
        callback=[checkpoint_callback, eval_callback],
        progress_bar=True,
    )

    print("\n" + "=" * 70)
    print("  Training completed")
    print("=" * 70)

except KeyboardInterrupt:
    print("\n\n训练被用户中断")
    print("已保存的检查点可以用于继续训练")

# ============ 第五步：保存和测试 ============
print("\n[步骤 5/5] 保存最终模型...")
model.save("my_robot_models/my_robot_final")
print("  Model saved: my_robot_models/my_robot_final.zip")

# 快速测试
print("\n正在进行快速测试...")
test_env = gym.make("AGI-Walker/Walker2D-v0")
obs, _ = test_env.reset()
total_reward = 0
steps = 0

for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, _ = test_env.step(action)
    total_reward += reward
    steps += 1
    if done or truncated:
        break

test_env.close()

print("  测试结果:")
print(f"    - 存活步数: {steps}")
print(f"    - 累计奖励: {total_reward:.2f}")

# ============ 完成总结 ============
print("\n" + "=" * 70)
print("  你已经完成第一个机器人训练示例")
print("=" * 70)

print("\n下一步:")
print("  1. 运行测试脚本:")
print("     python my_first_robot_test.py")
print()
print("  2. 查看训练曲线:")
print("     tensorboard --logdir=./my_robot_logs/")
print("     然后访问: http://localhost:6006")
print()
print("  3. 尝试更多功能:")
print("     - examples/quadruped_training.py (四足机器人)")
print("     - examples/offline_rl_demo.py (离线强化学习)")
print("     - examples/imitation_learning_demo.py (模仿学习)")
print()
print("  📚 学习资源:")
print("     - 完整教程: GETTING_STARTED.md")
print("     - API 文档: docs/")
print("     - GitHub: https://github.com/sossossal/AGI-Walker")
print("=" * 70)
