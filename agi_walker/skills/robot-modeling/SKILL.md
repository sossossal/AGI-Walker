---
name: robot-modeling
description: "快速创建双足/四足/轮式机器人模型,支持预设模板和参数化定制。适用于:(1)新建机器人项目 (2)修改现有设计 (3)批量生成变体 (4)导出到Godot仿真"
metadata:
  agi_walker:
    emoji: "🤖"
    category: "建模"
    requires:
      python_modules: ["numpy", "pydantic"]
---

# Robot Modeling Skill

快速创建参数化机器人模型,无需手写代码。

## 快速开始

### 方式 1: 代码创建

```python
from agi_walker import RobotBuilder

# 创建双足机器人
robot = (
    RobotBuilder("my_biped")
    .add_torso(height=0.5, mass=5.0)
    .add_leg_pair(
        thigh_length=0.3,
        shin_length=0.3,
        hip_joint="revolute"
    )
    .build()
)

# 保存配置
robot.save("configs/my_biped.json")
```

### 方式 2: 使用预设模板

```python
from agi_walker.skills import robot_modeling

# 加载模板
robot = robot_modeling.load_template("biped_basic")

# 自定义参数
robot.customize(
    leg_length=0.35,
    torso_mass=6.0
)

# 保存
robot.save("configs/custom_biped.json")
```

### 方式 3: GUI 可视化配置

```bash
python -m agi_walker.skills.robot_modeling.gui
```

## 预设模板

### biped_basic
基础双足机器人,适合步行研究。

- 腿长: 0.3m
- 躯干高度: 0.5m
- 总质量: 5kg
- 自由度: 6 DoF (髋关节x2 + 膝关节x2 + 踝关节x2)

### quadruped_dog
四足犬形机器人,适合跑步/跳跃研究。

- 腿长: 0.25m
- 躯干长度: 0.6m
- 总质量: 8kg
- 自由度: 12 DoF

### wheeled_base
轮式底盘,适合移动机器人研究。

- 轮距: 0.4m
- 轮径: 0.1m
- 总质量: 3kg
- 自由度: 2 DoF (左右轮速)

### humanoid_upper
类人上半身,适合操作研究。

- 臂长: 0.4m
- 肩宽: 0.3m
- 总质量: 4kg
- 自由度: 7 DoF × 2

## API 参考

详见: `references/api.md`

## 常见问题

**Q: 如何调整关节限位?**
A: 使用 `.set_joint_limits(joint_name, min_angle, max_angle)` 方法。

**Q: 如何添加传感器?**
A: 使用 `.add_sensor(sensor_type, mount_link)` 方法。

**Q: 如何导出到 Godot?**
A: 调用 `robot.export_godot("path/to/godot_project")` 自动生成场景文件。

## 示例: 创建竞速机器人

```python
from agi_walker import RobotBuilder

robot = (
    RobotBuilder("racer")
    .add_torso(height=0.3, mass=3.0)  # 低重心
    .add_leg_pair(
        thigh_length=0.4,  # 长腿
        shin_length=0.4,
        hip_width=0.2  # 窄髋
    )
    .set_joint_damping(0.5)  # 降低阻尼提高速度
    .build()
)
```

## 下一步

创建完成后,可以:

1. **仿真验证**: 使用 Godot 查看运动学
2. **参数优化**: 使用 `parameter-optimizer` skill 调优
3. **数据生成**: 使用 `simulation-runner` skill 生成训练数据

---

**相关 Skills**: `parameter-optimizer`, `urdf-generator`, `simulation-runner`
