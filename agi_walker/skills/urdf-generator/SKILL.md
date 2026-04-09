---
name: urdf-generator
description: "将 AGI-Walker 机器人配置转换为 URDF 或 SDF 文件，用于仿真器和 ROS 生态集成。"
category: 转换
emoji: "📄"
inputs:
  input_file:
    type: file_path
    description: 输入机器人配置文件
  output_file:
    type: file_path
    description: 输出 URDF 或 SDF 文件
  generate_meshes:
    type: boolean
    description: 是否生成碰撞网格
    default: false
    required: false
  world_file:
    type: boolean
    description: 导出 SDF 时是否同时生成世界文件
    default: false
    required: false
outputs:
  exported_file:
    type: file_path
    description: 导出的目标文件
  validation_result:
    type: boolean
    description: 导出结果是否通过基础校验
metadata:
  agi_walker:
    requires:
      python_modules:
        - lxml
        - numpy
---

# URDF Generator Skill

用于把 AGI-Walker 内部机器人配置转换为标准描述文件，方便接入外部仿真器和工具链。

当前 skill 的真实入口位于 `__init__.py`，核心接口包括：

- `URDFGenerator`
- `convert_to_urdf(...)`
- `convert_to_sdf(...)`
- `validate_urdf(...)`

## 适用场景

- 导出 URDF 给 ROS / ROS 2 工具链使用
- 导出 SDF 给 Gazebo 类仿真器使用
- 在导出后做基础结构校验

## 导出 URDF

```python
from agi_walker.skills.urdf_generator import convert_to_urdf

convert_to_urdf(
    input_file="configs/my_robot.json",
    output_file="exports/my_robot.urdf",
    generate_meshes=False,
)
```

## 导出 SDF

```python
from agi_walker.skills.urdf_generator import convert_to_sdf

convert_to_sdf(
    input_file="configs/my_robot.json",
    output_file="exports/my_robot.sdf",
    world_file=False,
)
```

## 校验导出结果

```python
from agi_walker.skills.urdf_generator import validate_urdf

ok = validate_urdf("exports/my_robot.urdf")
print(ok)
```

## 当前实现说明

- `URDFGenerator` 会把内部 `parts` 和 `connections` 转换为 `link` / `joint`
- 会为 link 计算基础惯性参数和几何描述
- `convert_to_sdf(...)` 当前采用“先导出 URDF，再包装为简化 SDF”的方式
- `generate_meshes` 参数目前保留，但网格自动生成功能尚未完整实现

## 输出结果

导出成功后，通常会得到：

- 目标格式文件，如 `.urdf` 或 `.sdf`
- 基础日志信息
- 可选的校验结果
