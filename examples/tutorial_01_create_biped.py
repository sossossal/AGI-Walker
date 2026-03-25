#!/usr/bin/env python
"""
教程1: 从零创建双足机器人

学习目标:
- 使用RobotBuilder流式API
- 添加不同类型的部件
- 设置关节参数
- 保存配置
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))
from agi_walker.skills.robot_modeling import RobotBuilder

print("=" * 70)
print("教程1: 从零创建双足机器人")
print("=" * 70)

# 步骤1: 创建机器人构建器
print("\n步骤1: 初始化机器人构建器")
robot = RobotBuilder("my_first_biped")
print("Created robot: my_first_biped")

# 步骤2: 添加躯干
print("\n步骤2: 添加躯干")
robot.add_torso(
    height=0.6, width=0.3, depth=0.2, mass=8.0  # 60cm高  # 30cm宽  # 20cm厚  # 8kg
)
print("Torso added")

# 步骤3: 添加腿部
print("\n步骤3: 添加左右腿")
robot.add_leg_pair(thigh_length=0.4, shin_length=0.4)  # 大腿40cm  # 小腿40cm
print("Leg pair added")

# 步骤4: 设置关节阻尼
print("\n步骤4: 设置关节阻尼")
robot.set_joint_damping(0.5)
print("Joint damping configured")

# 步骤5: 构建配置
print("\n步骤5: 构建最终配置")
config = robot.build()
print("Configuration built")
print(f"  - 部件数: {len(config.parts)}")
print(f"  - 连接数: {len(config.connections)}")

# 步骤6: 保存
print("\n步骤6: 保存配置")
output_file = Path("configs/tutorial_01_biped.json")
output_file.parent.mkdir(exist_ok=True)
config.save(str(output_file))
print(f"Saved to: {output_file}")

# 显示摘要
print("\n" + "=" * 70)
print("创建完成!")
print("=" * 70)
print(f"机器人名称: {config.name}")
print(f"部件列表:")
for part in config.parts:
    print(f"  - {part['id']} ({part['type']})")
print(f"\n配置文件: {output_file}")
print("\n下一步: 运行 tutorial_02_use_template.py 学习使用模板")
