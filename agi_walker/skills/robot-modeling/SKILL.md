---
name: robot-modeling
description: "快速创建双足和四足机器人配置，支持模板加载、参数化建模和 JSON 导出。"
category: 建模
emoji: "🤖"
inputs:
  robot_name:
    type: string
    description: 机器人名称
    required: false
  template_name:
    type: string
    description: 模板名称，如 biped_basic 或 quadruped_dog
    required: false
  torso_height:
    type: number
    description: 躯干高度
    required: false
  torso_mass:
    type: number
    description: 躯干质量
    required: false
outputs:
  robot_config:
    type: dict
    description: 机器人配置对象
  output_file:
    type: file_path
    description: 导出的 JSON 配置文件
metadata:
  agi_walker:
    requires:
      python_modules:
        - numpy
        - pydantic
---

# Robot Modeling Skill

用于快速构建机器人结构配置，减少手写 JSON 的成本。

当前 skill 的真实入口位于 `__init__.py`，核心接口包括：

- `RobotBuilder`
- `load_template(...)`
- `list_templates()`

## 适用场景

- 新建基础双足机器人配置
- 基于模板快速派生变体
- 为参数优化、URDF 导出等后续流程准备输入

## 方式一：使用 RobotBuilder 构建

```python
from agi_walker.skills.robot_modeling import RobotBuilder

robot = (
    RobotBuilder("my_biped")
    .add_torso(height=0.5, mass=5.0)
    .add_leg_pair(thigh_length=0.3, shin_length=0.3)
    .set_joint_damping(0.2)
    .build()
)

robot.save("configs/my_biped.json")
```

## 方式二：加载模板

```python
from agi_walker.skills.robot_modeling import load_template

robot = load_template("biped_basic")
robot.save("configs/biped_from_template.json")
```

## 当前可用模板

- `biped_basic`
- `quadruped_dog`

可以通过 `list_templates()` 查看模板清单。

## 已实现能力

- 添加躯干
- 添加双腿
- 添加双臂
- 设置关节阻尼
- 设置关节限位
- 在 `metadata` 中写入自定义参数
- 导出为 JSON 配置

## 说明

- 本 skill 生成的是 AGI-Walker 内部配置格式，不是 URDF
- 如果需要接入 Gazebo、PyBullet 或 ROS 2，请继续使用 `urdf-generator`
- 更详细的 API 说明可见 `references/api.md`
