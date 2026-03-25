"""
Robot Modeling Skill 完整演示

演示如何使用 Skills 系统快速创建机器人配置。
"""

import sys
import os
import importlib.util
from pathlib import Path

# Add project root to sys.path
project_root = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_root))

from agi_walker.skills_loader import get_skills_loader, search_skills


# Helper to load skill modules with hyphens
def load_skill_module(skill_name):
    skill_path = project_root / "agi_walker" / "skills" / skill_name / "__init__.py"
    if not skill_path.exists():
        raise ImportError(f"Skill module not found: {skill_path}")

    module_name = f"agi_walker.skills.{skill_name.replace('-', '_')}"
    spec = importlib.util.spec_from_file_location(module_name, skill_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


# Load robot-modeling skill dynamically
robot_modeling = load_skill_module("robot-modeling")
RobotBuilder = robot_modeling.RobotBuilder
load_template = robot_modeling.load_template
list_templates = robot_modeling.list_templates


def demo_skills_loader():
    """演示 Skills 加载器功能"""
    print("=" * 60)
    print("演示 1: Skills 系统加载")
    print("=" * 60)

    loader = get_skills_loader()
    print(f"\n✓ 已加载 {len(loader)} 个 skills\n")

    for skill in loader.get_skills_list():
        print(f"  {skill.display_name}")
        print(f"    分类: {skill.category}")
        print(f"    描述: {skill.description[:100]}...")
        print()

    # 搜索功能
    print("\n" + "=" * 60)
    print("演示 2: 搜索 Skills")
    print("=" * 60)

    results = search_skills("机器人")
    print(f"\n搜索 '机器人' 找到 {len(results)} 个结果:\n")
    for skill in results:
        print(f"  - {skill.name}")


def demo_robot_builder():
    """演示 RobotBuilder 创建机器人"""
    print("\n" + "=" * 60)
    print("演示 3: 使用 RobotBuilder 创建机器人")
    print("=" * 60)

    # 创建竞速双足机器人
    robot = (
        RobotBuilder("speed_biped")
        .add_torso(height=0.4, mass=4.0)  # 低重心
        .add_leg_pair(thigh_length=0.35, shin_length=0.35, hip_joint="revolute")  # 长腿
        .set_joint_damping(0.3)  # 低阻尼
        .customize(max_speed=3.0, gait_pattern="trot")
        .build()
    )

    print(f"\n✓ 创建机器人: {robot.name}")
    print(f"  - 部件数量: {len(robot.parts)}")
    print(f"  - 连接数量: {len(robot.connections)}")
    print(f"  - 元数据: {robot.metadata}")

    # 保存
    output_path = "configs/demo_speed_biped.json"
    robot.save(output_path)


def demo_templates():
    """演示加载和定制模板"""
    print("\n" + "=" * 60)
    print("演示 4: 使用预设模板")
    print("=" * 60)

    # 列出所有模板
    templates = list_templates()
    print(f"\n可用模板 ({len(templates)} 个):")
    for tmpl in templates:
        print(f"  - {tmpl}")

    # 加载模板
    print(f"\n加载模板: biped_basic")
    robot = load_template("biped_basic")

    print(f"\n模板信息:")
    print(f"  - 名称: {robot.name}")
    print(f"  - 部件: {len(robot.parts)}")
    print(f"  - 质量: {robot.metadata.get('total_mass', 'N/A')} kg")
    print(f"  - 高度: {robot.metadata.get('height', 'N/A')} m")
    print(f"  - 自由度: {robot.metadata.get('dof', 'N/A')}")

    # 定制模板
    print("\n定制参数...")
    robot.metadata["leg_length"] = 0.35  # 修改腿长
    robot.metadata["custom_name"] = "定制双足"

    # 保存定制版本
    output_path = "configs/demo_custom_biped.json"
    robot.save(output_path)


def demo_batch_generation():
    """演示批量生成机器人变体"""
    print("\n" + "=" * 60)
    print("演示 5: 批量生成机器人变体")
    print("=" * 60)

    leg_lengths = [0.25, 0.30, 0.35, 0.40]
    print(f"\n生成 {len(leg_lengths)} 个不同腿长的变体:\n")

    for leg_length in leg_lengths:
        robot = (
            RobotBuilder(f"biped_leg{int(leg_length*100)}")
            .add_torso(height=0.5, mass=5.0)
            .add_leg_pair(thigh_length=leg_length / 2, shin_length=leg_length / 2)
            .build()
        )

        output_path = f"configs/variants/biped_leg{int(leg_length*100)}.json"
        robot.save(output_path)
        print(f"  {robot.name}: 腿长 {leg_length}m")


def main():
    """运行所有演示"""
    print("\n" + "=" * 30)
    print("AGI-Walker Skills 系统完整演示")
    print("=" * 30 + "\n")

    # 运行所有演示
    demo_skills_loader()
    demo_robot_builder()
    demo_templates()
    demo_batch_generation()

    print("\n" + "=" * 60)
    print("演示完成!")
    print("=" * 60)

    print("\n生成的配置文件:")
    print("  - configs/demo_speed_biped.json")
    print("  - configs/demo_custom_biped.json")
    print("  - configs/variants/biped_leg*.json (4个文件)")

    print("\n下一步:")
    print("  1. 在 Godot 中加载配置进行仿真")
    print("  2. 使用 parameter-optimizer skill 优化参数")
    print("  3. 使用 urdf-generator skill 转换为 URDF 格式")
    print()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
