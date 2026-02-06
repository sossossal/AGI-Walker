"""
Parameter Optimizer 功能演示
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from agi_walker.skills.robot_modeling import RobotBuilder
from agi_walker.skills.parameter_optimizer import (
    optimize_mass_distribution,
    tune_pid_controller,
    batch_optimize_pid
)


def demo_mass_optimization():
    """演示质量分布优化"""
    print("=" * 60)
    print("演示 1: 质量分布优化")
    print("=" * 60)
    
    # 创建测试机器人
    robot = (
        RobotBuilder("test_biped")
        .add_torso(height=0.5, mass=5.0)
        .add_leg_pair(thigh_length=0.3, shin_length=0.3)
        .build()
    )
    
    print("\n原始配置:")
    for part in robot.parts:
        print(f"  - {part['id']}: {part['params'].get('mass', 1.0):.2f} kg")
    
    # 优化质量分布
    print("\n开始优化...")
    result = optimize_mass_distribution(
        robot.to_dict(),
        target_com_height=0.22,
        max_iterations=50,
        method="gradient"
    )
    
    print(f"\n优化结果:")
    print(f"  - 成功: {result.success}")
    print(f"  - 迭代次数: {result.iterations}")
    print(f"  - COM误差: {result.com_error:.6f} m")
    print(f"\n优化后质量分布:")
    for part_id, mass in result.mass_distribution.items():
        print(f"  - {part_id}: {mass:.2f} kg")


def demo_pid_tuning():
    """演示PID调优"""
    print("\n" + "=" * 60)
    print("演示 2: PID 参数调优")
    print("=" * 60)
    
    # 创建测试机器人
    robot = (
        RobotBuilder("test_biped")
        .add_torso(height=0.5, mass=5.0)
        .add_leg_pair(thigh_length=0.3, shin_length=0.3)
        .build()
    )
    
    # Ziegler-Nichols 调优
    print("\n方法 1: Ziegler-Nichols")
    gains_zn = tune_pid_controller(
        robot.to_dict(),
        joint_name="hip_flex",
        method="ziegler_nichols"
    )
    
    # 遗传算法调优
    print("\n方法 2: 遗传算法 (简化版)")
    gains_ga = tune_pid_controller(
        robot.to_dict(),
        joint_name="hip_flex",
        method="genetic",
        population_size=20,
        generations=30
    )


def demo_batch_optimization():
    """演示批量优化"""
    print("\n" + "=" * 60)
    print("演示 3: 批量PID调优")
    print("=" * 60)
    
    # 创建测试机器人
    robot = (
        RobotBuilder("test_quadruped")
        .add_torso(height=0.3, mass=6.0)
        .add_leg_pair(thigh_length=0.25, shin_length=0.25)
        .build()
    )
    robot.save("configs/test_quadruped.json")
    
    # 批量调优
    joints = ["hip_left", "hip_right", "knee_left", "knee_right"]
    results = batch_optimize_pid(
        "configs/test_quadruped.json",
        joints,
        method="ziegler_nichols"
    )
    
    print("\n批量优化结果:")
    for joint, gains in results.items():
        print(f"  {joint}: Kp={gains.kp:.2f}, Ki={gains.ki:.2f}, Kd={gains.kd:.2f}")


def main():
    print("\n" + "⚙️" * 30)
    print("Parameter Optimizer Skill 演示")
    print("⚙️" * 30 + "\n")
    
    try:
        demo_mass_optimization()
        demo_pid_tuning()
        demo_batch_optimization()
        
        print("\n" + "=" * 60)
        print("✓ 所有演示完成！")
        print("=" * 60)
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
