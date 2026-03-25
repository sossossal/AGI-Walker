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
from agi_walker.skills.robot_modeling import list_templates, load_template

print("=" * 70)
print("教程2: 使用模板快速开发")
print("=" * 70)

# 步骤1: 查看可用模板
print("\n步骤1: 列出所有可用模板")
templates = list_templates()
print(f"找到 {len(templates)} 个模板:")
for name in templates:
    print(f"  - {name}")

# 步骤2: 加载双足模板
print("\n步骤2: 加载双足机器人模板")
biped = load_template("biped_basic")
print(f"Loaded template: {biped.name}")
print(f"  - 部件数: {len(biped.parts)}")

# 步骤3: 加载四足模板
print("\n步骤3: 加载四足机器人模板")
quadruped = load_template("quadruped_dog")
print(f"Loaded template: {quadruped.name}")
print(f"  - 部件数: {len(quadruped.parts)}")

# 步骤4: 修改参数
print("\n步骤4: 修改模板参数")
print(f"原始躯干质量: {biped.parts[0]['params']['mass']}")
biped.parts[0]["params"]["mass"] = 10.0  # 修改为10kg
print(f"修改后质量: {biped.parts[0]['params']['mass']}")

# 步骤5: 保存修改
print("\n步骤5: 保存修改后的配置")
biped.save("configs/tutorial_02_custom_biped.json")
print("Saved to: configs/tutorial_02_custom_biped.json")

# 显示对比
print("\n" + "=" * 70)
print("模板对比")
print("=" * 70)
print(f"双足机器人: {len(biped.parts)} 部件")
print(f"四足机器人: {len(quadruped.parts)} 部件")
print("\nNext: run tutorial_03_optimize.py to learn parameter optimization")
