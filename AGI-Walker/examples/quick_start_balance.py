"""
简易平衡机器人示例
演示如何使用 AGI-Walker 进行从零到一的开发
"""

from godot_robot_env import GodotRobotEnv, PartsDatabase
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
import numpy as np

def main():
    print("=" * 60)
    print("AGI-Walker 快速开始示例：平衡机器人")
    print("=" * 60)
    
    # 步骤 1: 查看可用零件
    print("\n步骤 1: 浏览零件库")
    print("-" * 60)
    
    db = PartsDatabase()
    
    print("\n可用电机:")
    motors = ["dynamixel_xl430_w250", "dynamixel_mx106"]
    for motor_id in motors:
        motor = db.get_part(motor_id)
        print(f"  - {motor['model']}")
        print(f"    扭矩: {motor['specifications']['stall_torque']} N·m")
        print(f"    价格: ${motor['price_usd']}")
    
    # 步骤 2: 设计机器人
    print("\n步骤 2: 设计简单平衡机器人")
    print("-" * 60)
    
    robot_config = {
        "name": "SimpleBalancer",
        "joints": [
            {"name": "hip", "motor": "dynamixel_xl430_w250"},
            {"name": "ankle", "motor": "dynamixel_xl430_w250"}
        ]
    }
    
    total_cost = sum([db.get_part(j["motor"])["price_usd"] 
                      for j in robot_config["joints"]])
    
    print(f"机器人配置: {robot_config['name']}")
    print(f"关节数量: {len(robot_config['joints'])}")
    print(f"总成本: ${total_cost:.2f}")
    
    # 步骤 3: 创建仿真环境
    print("\n步骤 3: 创建仿真环境")
    print("-" * 60)
    
    env = GodotRobotEnv(
        scene_name="balance_bot",
        env_preset="earth",          # 地球重力
        ground_material="concrete"   # 混凝土地面
    )
    
    print(f"观察空间: {env.observation_space}")
    print(f"动作空间: {env.action_space}")
    
    # 步骤 4: 训练策略
    print("\n步骤 4: 训练平衡策略")
    print("-" * 60)
   
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        learning_rate=3e-4
    )
    
    print("开始训练... (这可能需要几分钟)")
    model.learn(total_timesteps=50000)
    
    # 保存模型
    model.save("balance_policy")
    print("✓ 模型已保存: balance_policy.zip")
    
    # 步骤 5: 测试策略
    print("\n步骤 5: 测试训练好的策略")
    print("-" * 60)
    
    obs, info = env.reset()
    episode_reward = 0
    
    for step in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        episode_reward += reward
        
        if terminated or truncated:
            print(f"回合结束 - 总奖励: {episode_reward:.2f}")
            break
    
    # 步骤 6: 导出用于硬件部署
    print("\n步骤 6: (可选) 导出到硬件")
    print("-" * 60)
    
    print("运行以下命令部署到 IMC-22 硬件:")
    print("  python examples/deploy_to_hardware.py")
    
    print("\n" + "=" * 60)
    print("完成！您已经:")
    print("  ✓ 选择了硬件零件")
    print("  ✓ 设计了机器人")
    print("  ✓ 训练了控制策略")
    print("  ✓ 测试了性能")
    print("\n下一步:")
    print("  - 尝试不同的环境设置（月球重力、冰面等）")
    print("  - 使用域随机化提高鲁棒性")
    print("  - 部署到真实硬件")
    print("=" * 60)
    
    env.close()


if __name__ == "__main__":
    main()
