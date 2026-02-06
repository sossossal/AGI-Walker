"""
完整工作流演示 - 3个Skills协同工作

演示从建模→优化→转换URDF的完整流程。
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from agi_walker.skills.robot_modeling import RobotBuilder, load_template
from agi_walker.skills.parameter_optimizer import optimize_mass_distribution, tune_pid_controller
from agi_walker.skills.urdf_generator import convert_to_urdf, validate_urdf


def demo_complete_workflow():
    """演示完整工作流程"""
    print("=" * 70)
    print(" " * 20 + "AGI-Walker 完整工作流演示")
    print("=" * 70)
    print()
    
    # ========== 阶段1: 建模 ==========
    print("📐 阶段 1: 机器人建模")
    print("-" * 70)
    
    # 使用RobotBuilder创建机器人
    print("\n方式1: 使用RobotBuilder流式API")
    robot = (
        RobotBuilder("demo_biped")
        .add_torso(height=0.5, mass=5.0)
        .add_leg_pair(
            thigh_length=0.3,
            shin_length=0.3,
            hip_joint="revolute"
        )
        .set_joint_damping(0.5)
        .build()
    )
    
    print(f"✓ 创建机器人: {robot.name}")
    print(f"  - 部件数量: {len(robot.parts)}")
    print(f"  - 连接数量: {len(robot.connections)}")
    
    # 保存初始配置
    robot.save("configs/demo_初始.json")
    print(f"✓ 保存配置: configs/demo_初始.json")
    
    # 使用模板
    print("\n方式2: 加载预设模板")
    robot_template = load_template("biped_basic")
    print(f"✓ 加载模板: {robot_template.name}")
    print(f"  - 总质量: {robot_template.metadata.get('total_mass', 'N/A')} kg")
    print(f"  - 高度: {robot_template.metadata.get('height', 'N/A')} m")
    
    print()
    
    # ========== 阶段2: 优化 ==========
    print("⚙️ 阶段 2: 参数优化")
    print("-" * 70)
    
    # 质量分布优化
    print("\n优化1: 质量分布")
    mass_result = optimize_mass_distribution(
        robot.to_dict(),
        target_com_height=0.22,
        max_iterations=50,
        method="gradient"
    )
    
    print(f"✓ 优化完成")
    print(f"  - 成功: {mass_result.success}")
    print(f"  - 迭代: {mass_result.iterations}")
    print(f"  - COM误差: {mass_result.com_error:.6f} m")
    
    # 应用优化结果
    for part in robot.parts:
        part_id = part["id"]
        if part_id in mass_result.mass_distribution:
            part["params"]["mass"] = mass_result.mass_distribution[part_id]
    
    # PID调优
    print("\n优化2: PID参数调优")
    joints = ["torso_1_to_leg_left_1", "torso_1_to_leg_right_1"]
    
    for joint in joints:
        gains = tune_pid_controller(
            robot.to_dict(),
            joint_name=joint,
            method="ziegler_nichols"
        )
    
    # 保存优化后配置
    robot.save("configs/demo_优化.json")
    print(f"✓ 保存优化配置: configs/demo_优化.json")
    
    print()
    
    # ========== 阶段3: 转换URDF ==========
    print("📄 阶段 3: URDF转换")
    print("-" * 70)
    
    # 转换为URDF
    print("\n转换为URDF格式...")
    convert_to_urdf(
        input_file="configs/demo_优化.json",
        output_file="exports/demo_biped.urdf",
        generate_meshes=False
    )
    
    # 验证URDF
    print("\n验证URDF文件...")
    is_valid = validate_urdf("exports/demo_biped.urdf")
    
    if is_valid:
        print("\n✓ URDF验证通过,可以在以下环境中使用:")
        print("  - ROS 2: ros2 run rviz2 rviz2 -d exports/demo_biped.rviz")
        print("  - Gazebo: gazebo exports/demo_biped.urdf")
        print("  - MuJoCo: 使用 MuJoCo viewer 加载")
    
    print()
    
    # ========== 总结 ==========
    print("=" * 70)
    print("🎉 完整工作流演示完成！")
    print("=" * 70)
    print("\n生成的文件:")
    print("  1. configs/demo_初始.json     - 初始机器人配置")
    print("  2. configs/demo_优化.json     - 优化后配置")
    print("  3. exports/demo_biped.urdf    - URDF格式 (用于ROS 2/Gazebo)")
    print()
    print("下一步:")
    print("  - 在Godot中测试初始配置")
    print("  - 在Gazebo中验证URDF")
    print("  - 使用MuJoCo训练强化学习模型")
    print()


def demo_skills_system():
    """演示Skills系统本身"""
    from agi_walker.skills_loader import get_skills_loader
    
    print("\n" + "=" * 70)
    print(" " * 25 + "Skills 系统状态")
    print("=" * 70)
    
    loader = get_skills_loader()
    
    print(f"\n已加载 {len(loader)} 个 skills:")
    print()
    
    for skill in loader.get_skills_list():
        print(f"{skill.display_name}")
        print(f"  描述: {skill.description[:80]}...")
        print(f"  分类: {skill.category}")
        if skill.requires:
            print(f"  依赖: {skill.requires}")
        print()
    
    # 搜索演示
    print("-" * 70)
    print("搜索功能演示")
    print("-" * 70)
    
    search_terms = ["机器人", "优化", "urdf"]
    for term in search_terms:
        results = loader.search_skills(term)
        print(f"\n搜索 '{term}': 找到 {len(results)} 个结果")
        for skill in results:
            print(f"  - {skill.name}")


def main():
    print("\n" + "🤖" * 35)
    print(" " * 15 + "AGI-Walker × Moltbot Skills")
    print(" " * 20 + "完整演示程序")
    print("🤖" * 35 + "\n")
    
    try:
        # 演示Skills系统
        demo_skills_system()
        
        # 演示完整工作流
        demo_complete_workflow()
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
