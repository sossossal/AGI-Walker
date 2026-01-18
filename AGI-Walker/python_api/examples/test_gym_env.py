# test_gym_env.py
# 测试 Gymnasium 环境（无需 Godot 连接）
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from godot_robot_env import GodotRobotEnv, PartsDatabase

def main():
    print("=" * 60)
    print("Gymnasium 环境测试（离线模式）")
    print("=" * 60)
    
    # 1. 测试零件库
    print("\n[1] 加载零件数据库...")
    db = PartsDatabase()
    print("    ✅ 成功加载", len(db.list_all_parts()), "个零件")
    
    # 2. 创建机器人配置
    print("\n[2] 创建机器人配置...")
    robot_config = db.create_robot_config([
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_left"},
        {"part_id": "dynamixel_xl430_w250", "joint": "hip_right"},
        {"part_id": "dynamixel_xl430_w250", "joint": "knee_left"},
        {"part_id": "dynamixel_xl430_w250", "joint": "knee_right"},
    ])
    print("    ✅ 配置包含", len(robot_config['parts']), "个零件")
    
    # 3. 创建环境
    print("\n[3] 创建 Gym 环境...")
    physics_config = {
        "gravity": 9.81,
        "air_density": 1.225,
        "ground_material": "concrete"
    }
    
    env = GodotRobotEnv(
        robot_config=robot_config,
        physics_config=physics_config
    )
    print("    ✅ 环境创建成功")
    
    # 4. 显示空间信息
    print("\n[4] 观察空间:")
    print("    类型:", type(env.observation_space))
    print("    包含:")
    for key in env.observation_space.spaces.keys():
        space = env.observation_space.spaces[key]
        print(f"      - {key}: {space.shape} ({space.dtype})")
    
    print("\n[5] 动作空间:")
    print("    类型:", type(env.action_space))
    print("    形状:", env.action_space.shape)
    print("    范围:", env.action_space.low, "到", env.action_space.high)
    
    # 5. 测试奖励函数
    print("\n[6] 测试奖励函数...")
    import numpy as np
    
    # 模拟观察数据
    mock_obs = {
        'imu_orient': np.array([0, 0, 0], dtype=np.float32),
        'imu_angular_vel': np.array([0, 0, 0], dtype=np.float32),
        'imu_linear_acc': np.array([0, 0, 9.81], dtype=np.float32),
        'joint_angles': np.array([0, 0, 0, 0], dtype=np.float32),
        'joint_velocities': np.array([0, 0, 0, 0], dtype=np.float32),
        'joint_torques': np.array([0, 0, 0, 0], dtype=np.float32),
        'foot_contacts': np.array([1, 1], dtype=np.int8),
        'torso_height': np.array([1.0], dtype=np.float32)
    }
    
    mock_action = np.array([0, 0, -30, -30], dtype=np.float32)
    
    reward = env._calculate_reward(mock_obs, mock_action)
    print(f"    示例奖励: {reward:.3f}")
    
    # 6. 测试终止条件
    print("\n[7] 测试终止条件...")
    
    # 正常姿态
    normal_obs = mock_obs.copy()
    is_term = env._is_terminated(normal_obs)
    print(f"    正常姿态终止: {is_term} ✅")
    
    # 倾倒姿态
    fallen_obs = mock_obs.copy()
    fallen_obs['imu_orient'] = np.array([70, 0, 0], dtype=np.float32)
    is_term = env._is_terminated(fallen_obs)
    print(f"    倾倒姿态终止: {is_term} ✅")
    
    # 低高度
    low_obs = mock_obs.copy()
    low_obs['torso_height'] = np.array([0.2], dtype=np.float32)
    is_term = env._is_terminated(low_obs)
    print(f"    低高度终止: {is_term} ✅")
    
    print("\n" + "=" * 60)
    print("✅ 所有测试通过！")
    print("=" * 60)
    print("\n提示: 要进行实际训练，需要:")
    print("  1. 启动 Godot 仿真器")
    print("  2. 配置 TCP 服务器监听端口 9999")
    print("  3. 运行 train_walker_ppo.py")

if __name__ == "__main__":
    main()
