# 物理环境增强系统使用指南

## 📋 概述

物理环境增强系统允许您动态调节仿真环境的物理参数，包括重力、空气密度、温度、地面材质等，用于：
- 测试机器人在不同环境下的表现
- 域随机化训练（提高泛化能力）
- 模拟极端环境（月球、火星等�?

---

## 🌍 环境控制�?(EnvironmentController)

### 基本使用

```gdscript
# 添加到场�?
var env_controller = EnvironmentController.new()
add_child(env_controller)

# 设置重力
env_controller.set_gravity(3.71)  # 火星重力

# 设置空气密度
env_controller.set_air_density(0.02)  # kg/m³

# 设置温度
env_controller.set_temperature(-60.0)  # °C

# 设置风力
env_controller.set_wind(Vector3(5, 0, 0))  # 5m/s 东风
```

### 环境预设

快速切换到预定义环境：

```gdscript
# 地球环境（默认）
env_controller.load_preset("earth")

# 月球环境（低重力，无大气�?
env_controller.load_preset("moon")

# 火星环境
env_controller.load_preset("mars")

# 木星环境（高重力�?
env_controller.load_preset("jupiter")
```

**预设参数对比**:

| 环境 | 重力 (m/s²) | 空气密度 (kg/m³) | 温度 (°C) |
|------|-------------|------------------|-----------|
| 地球 | 9.81 | 1.225 | 25 |
| 月球 | 1.62 | 0.0 | -20 |
| 火星 | 3.71 | 0.02 | -60 |
| 木星 | 24.79 | 0.16 | -110 |

### 空气阻力计算

```gdscript
# 在刚体的 _physics_process 中：
func _physics_process(delta):
    var velocity = linear_velocity
    var cross_section = 0.5  # m² (机器人横截面�?
    var drag_coef = 0.47  # 球体阻力系数
    
    var air_drag = env_controller.calculate_air_drag(
        velocity,
        cross_section,
        drag_coef
    )
    
    apply_central_force(air_drag)
```

### 温度影响

```gdscript
# 获取温度影响因子（影响摩擦等�?
var temp_factor = env_controller.get_temperature_factor()
var adjusted_friction = base_friction * temp_factor
```

### 随机扰动（域随机化）

```gdscript
# 每隔一段时间施加随机扰�?
func apply_domain_randomization():
    env_controller.apply_random_disturbance(robot_body, 10.0)
```

---

## 🏗�?地面材质系统

### 使用材质�?

```gdscript
# 添加材质�?
var material_lib = GroundMaterialLibrary.new()
add_child(material_lib)

# 获取地面 StaticBody3D
var ground = $Ground

# 应用材质
material_lib.apply_material(ground, "concrete")  # 混凝�?
material_lib.apply_material(ground, "ice")       # 冰面
material_lib.apply_material(ground, "sand")      # 沙地
```

### 可用材质

| 材质 | 摩擦系数 | 弹�?| 滚动摩擦 | 特点 |
|------|----------|------|----------|------|
| **concrete** | 0.9 | 0.1 | 0.005 | 硬质，高摩擦 |
| **wood** | 0.6 | 0.2 | 0.01 | 中等硬度 |
| **carpet** | 1.0 | 0.05 | 0.03 | 高摩擦，高阻�?|
| **ice** | 0.1 | 0.3 | 0.001 | 极低摩擦 |
| **metal** | 0.4 | 0.4 | 0.005 | 低摩擦，高弹�?|
| **sand** | 0.7 | 0.0 | 0.05 | 可变�?|
| **grass** | 0.75 | 0.1 | 0.02 | 自然地形 |
| **mud** | 0.85 | 0.0 | 0.08 | 可变形，高阻�?|

### 自定义材�?

```gdscript
# 创建自定义材�?
var custom_mat = GroundMaterial.new("Rubber", 1.2, 0.8)
custom_mat.roughness = 0.9
custom_mat.color = Color(0.2, 0.2, 0.2)
custom_mat.rolling_friction = 0.02

# 添加到库
material_lib.add_custom_material(custom_mat)

# 使用
material_lib.apply_material(ground, "Rubber")
```

---

## 🎮 实时控制

### 键盘快捷键示�?

```gdscript
func _input(event):
    if event is InputEventKey and event.pressed:
        match event.keycode:
            # 环境切换
            KEY_1: env_controller.load_preset("earth")
            KEY_2: env_controller.load_preset("moon")
            KEY_3: env_controller.load_preset("mars")
            
            # 材质切换
            KEY_C: material_lib.apply_material(ground, "concrete")
            KEY_I: material_lib.apply_material(ground, "ice")
            KEY_S: material_lib.apply_material(ground, "sand")
            
            # 重力调节
            KEY_UP: env_controller.set_gravity(env_controller.gravity + 1.0)
            KEY_DOWN: env_controller.set_gravity(env_controller.gravity - 1.0)
```

---

## 🔗 �?Python API 集成

### Python 端调�?

```python
from godot_robot_env import GodotRobotEnv

# 创建环境时指定物理参�?
env = GodotRobotEnv(
    physics_config={
        "gravity": 3.71,  # 火星重力
        "air_density": 0.02,
        "temperature": -60.0,
        "ground_material": "sand"
    }
)

# 运行时动态修�?
env.set_physics_params({
    "gravity": 9.81,
    "ground_material": "ice"
})
```

### 域随机化训练

```python
import random

def domain_randomization_callback():
    """每个 episode 随机化环境参�?""
    env.set_physics_params({
        "gravity": random.uniform(5.0, 15.0),
        "air_density": random.uniform(0.5, 2.0),
        "ground_material": random.choice([
            "concrete", "wood", "carpet", "ice", "sand"
        ])
    })

# 在训练循环中
for episode in range(1000):
    domain_randomization_callback()
    obs = env.reset()
    # ... 训练 ...
```

---

## 📊 监控环境状�?

```gdscript
# 获取当前环境信息
var env_info = env_controller.get_environment_info()
print("重力: ", env_info["gravity"])
print("温度: ", env_info["temperature"])
print("风�? ", env_info["wind_velocity"])

# 导出配置
var config = env_controller.to_dict()
# 保存到文件或发送到 Python

# 从配置加�?
env_controller.from_dict(saved_config)
```

---

## 🧪 测试示例场景

创建测试场景 `test_environment.tscn`:

```
根节�?(Node3D)
├── EnvironmentController
├── GroundMaterialLibrary
├── Ground (StaticBody3D)
�?  └── CollisionShape3D (BoxShape3D)
├── Robot (RigidBody3D)
└── Test Script (test_environment.gd)
```

运行场景后：
- �?`1-3` 切换环境预设
- �?`C/I/S` 切换地面材质
- �?`�?↓` 调节重力

---

## 🎯 应用场景

### 1. 鲁棒性测�?
测试机器人在各种环境下的稳定性：
- 月球低重力环�?
- 冰面低摩擦环�?
- 强风干扰环境

### 2. 域随机化训练
提高 Sim-to-Real 迁移能力�?
- 随机化重�?(±20%)
- 随机化摩擦系�?(±30%)
- 随机化扰动力

### 3. 环境适应性研�?
研究最佳环境参数：
- 不同重力下的步态优�?
- 不同地面的能量效�?

---

## 🔧 高级功能

### 连接信号

```gdscript
func _ready():
    env_controller.environment_changed.connect(_on_env_changed)
    env_controller.preset_loaded.connect(_on_preset_loaded)

func _on_env_changed(param_name: String, new_value: float):
    print("参数变化: ", param_name, " = ", new_value)

func _on_preset_loaded(preset_name: String):
    print("加载预设: ", preset_name)
```

### 性能优化

```gdscript
# 批量更新参数（减少信号触发）
env_controller.from_dict({
    "gravity": 3.71,
    "air_density": 0.02,
    "temperature": -60.0
})
```

---

**版本**: 1.0  
**最后更�?*: 2026-01-14
