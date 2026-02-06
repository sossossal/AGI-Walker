---
name: urdf-generator
description: "将AGI-Walker配置转换为URDF/SDF格式供Gazebo/MuJoCo/PyBullet使用。适用于:(1)导出到ROS 2生态 (2)与Gazebo仿真集成 (3)MuJoCo物理引擎 (4)生成碰撞网格 (5)创建可视化模型"
metadata:
  agi_walker:
    emoji: "📄"
    category: "转换"
    requires:
      python_modules: ["lxml", "numpy"]
---

# URDF Generator Skill

将AGI-Walker机器人配置转换为标准URDF/SDF格式,实现与主流仿真器的无缝集成。

## 快速开始

### 转换为URDF

```python
from agi_walker.skills.urdf_generator import convert_to_urdf

convert_to_urdf(
    input_file="configs/my_robot.json",
    output_file="exports/my_robot.urdf",
    generate_meshes=True  # 自动生成碰撞网格
)
```

### 转换为SDF

```python
from agi_walker.skills.urdf_generator import convert_to_sdf

convert_to_sdf(
    input_file="configs/my_robot.json",
    output_file="exports/my_robot.sdf",
    world_file=True  # 生成完整世界文件
)
```

### 批量转换

```bash
python -m agi_walker.skills.urdf_generator.scripts.batch_convert \
    --input configs/ \
    --output exports/ \
    --format urdf,sdf \
    --meshes
```

## 支持的格式

### URDF (Unified Robot Description Format)
**用途**: ROS/ROS 2标准格式

**特性**:
- 完整的运动学树结构
- 物理参数 (质量/惯性)
- 关节类型和限位
- 碰撞和可视化几何

**适用仿真器**:
- Gazebo Classic
- Gazebo Ignition
- PyBullet
- RViz/RViz2

### SDF (Simulation Description Format)
**用途**: Gazebo原生格式

**特性**:
- URDF的超集
- 支持插件系统
- 更强大的物理描述
- 世界环境定义

**适用仿真器**:
- Gazebo Classic
- Gazebo Ignition

### MJCF (MuJoCo XML Format)
**用途**: MuJoCo仿真器 (实验性支持)

**特性**:
- 高性能物理仿真
- 接触模型
- 执行器定义

## 功能特性

### 1. 自动几何生成

```python
from agi_walker.skills.urdf_generator import URDFGenerator

generator = URDFGenerator()
generator.load_config("configs/robot.json")

# 生成简单几何 (盒子/圆柱/球)
generator.generate_collision_geometry(shape="box")

# 或生成三角网格
generator.generate_collision_meshes(resolution="high")

# 导出
generator.export_urdf("exports/robot.urdf")
```

### 2. 材质和纹理

```python
generator.set_visual_material(
    link_name="torso",
    color=[0.8, 0.2, 0.2, 1.0],  # RGBA
    texture="assets/robot_skin.png"
)
```

### 3. 传感器集成

```python
# 添加IMU
generator.add_sensor(
    name="imu_sensor",
    type="imu",
    parent_link="torso",
    update_rate=100.0
)

# 添加相机
generator.add_sensor(
    name="head_camera",
    type="camera",
    parent_link="head",
    image_size=(640, 480),
    fov=1.5708  # 90度
)
```

### 4. Gazebo插件

```python
# 添加差速驱动插件
generator.add_gazebo_plugin(
    plugin_name="differential_drive",
    plugin_type="gazebo_ros_diff_drive",
    parameters={
        "left_joint": "wheel_left_joint",
        "right_joint": "wheel_right_joint",
        "wheel_separation": 0.4,
        "wheel_diameter": 0.1
    }
)
```

## 命令行工具

### 转换单个文件

```bash
python -m agi_walker.skills.urdf_generator.convert \
    --input configs/robot.json \
    --output exports/robot.urdf \
    --format urdf \
    --validate  # 验证URDF有效性
```

### 查看转换预览

```bash
python -m agi_walker.skills.urdf_generator.preview \
    --input configs/robot.json \
    --viewer rviz  # 或 meshlab
```

### 验证URDF

```bash
python -m agi_walker.skills.urdf_generator.validate \
    --file exports/robot.urdf \
    --check-tf  # 检查坐标变换
```

## 高级用法

### 自定义转换规则

创建 `conversion_rules.yaml`:

```yaml
# 部件类型映射
part_mapping:
  torso:
    collision_shape: box
    visual_shape: mesh
    mesh_path: "meshes/torso.stl"
  
  leg:
    collision_shape: cylinder
    mass_distribution: uniform

# 关节配置
joint_config:
  revolute:
    damping: 0.1
    friction: 0.01
    effort_limit: 100.0
    velocity_limit: 10.0

# Gazebo特定配置
gazebo:
  physics:
    max_step_size: 0.001
    real_time_factor: 1.0
```

使用:
```python
generator.load_rules("conversion_rules.yaml")
generator.apply_rules()
```

### 与ROS 2集成

```python
from agi_walker.skills.urdf_generator import generate_ros2_package

generate_ros2_package(
    robot_config="configs/robot.json",
    package_name="my_robot_description",
    output_dir="ros2_ws/src/",
    include_launch_files=True,
    include_rviz_config=True
)
```

生成的包结构:
```
my_robot_description/
├── urdf/
│   └── robot.urdf
├── meshes/
│   └── *.stl
├── launch/
│   ├── display.launch.py
│   └── gazebo.launch.py
├── rviz/
│   └── robot.rviz
├── package.xml
└── CMakeLists.txt
```

## 示例: 完整工作流

```python
from agi_walker.skills.robot_modeling import load_template
from agi_walker.skills.parameter_optimizer import optimize_mass_distribution
from agi_walker.skills.urdf_generator import URDFGenerator

# 1. 加载模板
robot = load_template("biped_basic")

# 2. 优化参数
mass_result = optimize_mass_distribution(
    robot,
    target_com_height=0.25
)

# 应用优化结果
for part_id, mass in mass_result.mass_distribution.items():
    robot.update_part_mass(part_id, mass)

# 3. 转换为URDF
generator = URDFGenerator()
generator.load_config(robot.to_dict())
generator.generate_collision_geometry()
generator.add_sensor("imu", "imu", "torso")
generator.export_urdf("exports/optimized_biped.urdf")

# 4. 在Gazebo中测试
generator.launch_gazebo("exports/optimized_biped.urdf")
```

## 可视化工具

### RViz可视化

```bash
# 生成RViz配置
python -m agi_walker.skills.urdf_generator.generate_rviz \
    --urdf exports/robot.urdf \
    --output exports/robot.rviz

# 启动RViz
ros2 run rviz2 rviz2 -d exports/robot.rviz
```

### Gazebo预览

```bash
# 快速预览
python -m agi_walker.skills.urdf_generator.gazebo_preview \
    --urdf exports/robot.urdf \
    --world empty  # 或 custom.world
```

## 常见问题

**Q: 生成的URDF在RViz中显示不正确?**
A: 
1. 检查坐标变换: `ros2 run tf2_tools view_frames`
2. 验证URDF: `check_urdf robot.urdf`
3. 查看joint状态: 确保所有joint有正确的父子关系

**Q: Gazebo中机器人掉落或抖动?**
A:
1. 检查质量和惯性张量
2. 调整接触参数 (kp/kd)
3. 减小仿真步长

**Q: 如何添加自定义网格?**
A:
```python
generator.set_visual_mesh(
    link_name="torso",
    mesh_path="meshes/custom_torso.stl",
    scale=[1.0, 1.0, 1.0]
)
```

## 最佳实践

1. **命名规范**: 使用描述性名称,避免特殊字符
2. **坐标系**: 遵循ROS标准 (x前 y左 z上)
3. **单位**: 使用SI单位 (米/千克/秒)
4. **惯性**: 使用realistic的惯性张量,避免数值问题
5. **碰撞几何**: 保持简单以提高性能

## 调试技巧

### 可视化坐标系

```python
generator.visualize_frames(
    output="exports/frames.png",
    show_joint_axes=True
)
```

### 导出调试信息

```python
generator.export_debug_info("exports/debug.txt")
# 包含: 质量分布/惯性/关节树/碰撞对
```

## 下一步

转换完成后,可以:

1. **ROS 2集成**: 创建描述包并启动节点
2. **Gazebo仿真**: 测试物理行为和控制
3. **MuJoCo训练**: 用于强化学习训练
4. **硬件部署**: 验证运动学正确性

---

**相关 Skills**: `robot-modeling`, `parameter-optimizer`, `simulation-runner`
**外部工具**: ROS 2, Gazebo, MuJoCo, PyBullet
