#!/usr/bin/env python
"""
教程2: 使用模板快速开发

学习目标:
- 列出可用模板
- 加载预设模板
- 修改模板参数
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import importlib.util

# 动态导入robot-modeling skill
skill_path = (
    Path(__file__).parent.parent
    / "agi_walker"
    / "skills"
    / "robot-modeling"
    / "__init__.py"
)
spec = importlib.util.spec_from_file_location("robot_modeling", skill_path)
robot_modeling = importlib.util.module_from_spec(spec)
spec.loader.exec_module(robot_modeling)

print("=" * 70)
print("教程2: 使用模板快速开发")
print("=" * 70)

# 步骤1: 查看可用模板
print("\n步骤1: 列出所有可用模板")
templates = robot_modeling.list_templates()
print(f"找到 {len(templates)} 个模板:")
for name in templates:
    print(f"  - {name}")

# 步骤2: 加载双足模板
print("\n步骤2: 加载双足机器人模板")
biped = robot_modeling.load_template("biped_basic")
print(f"✓ 加载模板: {biped.name}")
print(f"  - 部件数: {len(biped.parts)}")

# 步骤3: 加载四足模板
print("\n步骤3: 加载四足机器人模板")
quadruped = robot_modeling.load_template("quadruped_dog")
print(f"✓ 加载模板: {quadruped.name}")
print(f"  - 部件数: {len(quadruped.parts)}")

# 步骤4: 修改参数
print("\n步骤4: 修改模板参数")
print(f"原始躯干质量: {biped.parts[0]['params']['mass']}")
biped.parts[0]["params"]["mass"] = 10.0  # 修改为10kg
print(f"修改后质量: {biped.parts[0]['params']['mass']}")

# 步骤5: 保存修改
print("\n步骤5: 保存修改后的配置")
biped.save("configs/tutorial_02_custom_biped.json")
print("✓ 保存到: configs/tutorial_02_custom_biped.json")

# 显示对比
print("\n" + "=" * 70)
print("模板对比")
print("=" * 70)
print(f"双足机器人: {len(biped.parts)} 部件")
print(f"四足机器人: {len(quadruped.parts)} 部件")
print("\n下一步: 运行 tutorial_03_optimize.py 学习参���优化")
