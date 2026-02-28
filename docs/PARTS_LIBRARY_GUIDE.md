# 机器人零件库使用指南

## 📦 简介

机器人零件库是 AGI-Walker 项目的扩展功能，提供了基于真实硬件规格的数字零件库系统。您可以使用这些零件来构建精确的机器人仿真模型。

## 🎯 功能特性

- ✅ **真实规格数据**：基于实际供应商数据手册的参数
- ✅ **标准化格式**：JSON Schema 验证，确保数据一致性
- ✅ **即插即用**：通过 API 快速创建零件实例
- ✅ **物理精确**：包含质量、惯量、摩擦、热特性等参数
- ✅ **易于扩展**：按照 Schema 添加新零件

## 📂 目录结构

```
parts_library/
├── schema/                    # JSON Schema 定义
│   ├── motor.schema.json      # 电机规格 Schema
│   └── sensor_imu.schema.json # IMU 传感器 Schema
│
├── motors/                    # 电机零件
│   └── dynamixel/
│       ├── xl430_w250.json    # Dynamixel XL430-W250
│       └── mx106.json         # Dynamixel MX-106
│
└── sensors/                   # 传感器零件
    └── imu/
        └── bno055.json        # Bosch BNO055 IMU
```

## 🚀 快速开始

### 1. 启用插件

在 Godot 编辑器中：
1. 打开 `项目` -> `项目设置` -> `插件`
2. 找到 "Robot Simulation Toolkit"
3. 勾选启用

### 2. 在场景中使用零件库

```gdscript
# 在您的脚本中
extends Node3D

var parts_lib: RobotPartsLibrary

func _ready():
    # 创建零件库管理器
    parts_lib = RobotPartsLibrary.new()
    add_child(parts_lib)
    
    # 等待加载完成
    await get_tree().process_frame
    
    # 使用零件库
    create_robot_arm()

func create_robot_arm():
    # 获取零件信息
    var motor_data = parts_lib.get_part("dynamixel_xl430_w250")
    print("使用电机: ", motor_data.get("model"))
    
    # 创建电机实例
    var shoulder_motor = parts_lib.create_motor_instance(
        "dynamixel_xl430_w250",
        self
    )
    shoulder_motor.position = Vector3(0, 1, 0)
```

### 3. 创建电机关节

```gdscript
func create_joint_with_motor():
    # 假设已有两个刚体：upper_arm 和 forearm
    var upper_arm = $UpperArm  # RigidBody3D
    var forearm = $Forearm     # RigidBody3D
    
    # 使用 Dynamixel MX-106 创建肘关节
    var elbow_joint = parts_lib.create_motor_joint(
        "dynamixel_mx106",        # 零件 ID
        upper_arm,                 # 父刚体
        forearm,                   # 子刚体
        Vector3.RIGHT,             # 旋转轴
        Vector3(0, -0.2, 0),      # 父刚体连接点
        Vector3(0, 0.2, 0)        # 子刚体连接点
    )
    
    # 设置关节限位
    elbow_joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, deg_to_rad(-120))
    elbow_joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, deg_to_rad(0))
    
    # 控制电机
    elbow_joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, 2.0)
```

## 🔍 API 参考

### RobotPartsLibrary 类

#### 核心方法

```gdscript
# 获取零件数据
func get_part(part_id: String) -> Dictionary

# 按类别筛选
func get_parts_by_category(category: String) -> Array[Dictionary]

# 按制造商筛选
func get_parts_by_manufacturer(manufacturer: String) -> Array[Dictionary]

# 创建电机实例
func create_motor_instance(part_id: String, parent: Node3D = null) -> Node3D

# 创建电机关节
func create_motor_joint(
    part_id: String,
    body_a: RigidBody3D,
    body_b: RigidBody3D,
    axis: Vector3 = Vector3.RIGHT,
    local_pos_a: Vector3 = Vector3.ZERO,
    local_pos_b: Vector3 = Vector3.ZERO
) -> HingeJoint3D

# 验证零件数据
func validate_part(part_id: String) -> bool

# 列出所有零件
func list_all_parts() -> Array[String]

# 打印统计信息
func print_statistics() -> void
```

## 📊 当前零件库

### 电机/舵机

| Part ID | 型号 | 扭矩 | 速度 | 价格 |
|---------|------|------|------|------|
| `dynamixel_xl430_w250` | XL430-W250-T | 1.4 N·m | 50 RPM | $69.90 |
| `dynamixel_mx106` | MX-106T | 8.4 N·m | 45 RPM | $459.90 |

### 传感器

| Part ID | 型号 | 类型 | 更新率 | 价格 |
|---------|------|------|--------|------|
| `bosch_bno055` | BNO055 | 9轴IMU | 100 Hz | $34.95 |

## ➕ 添加新零件

### 步骤 1: 创建 JSON 文件

在相应类别目录下创建新的 JSON 文件，例如 `parts_library/motors/maxon/ec45.json`:

```json
{
  "part_id": "maxon_ec45_flat",
  "category": "actuator_motor",
  "manufacturer": "Maxon Motor",
  "model": "EC45 flat 30W",
  "specifications": {
    "stall_torque": 0.134,
    "no_load_speed": 7200,
    "weight": 0.136,
    "dimensions": [45, 18.2, 45],
    "gear_ratio": 1,
    "max_current": 2.58,
    "voltage_range": [10, 30],
    "rotor_inertia": 4.7e-5,
    "friction": {
      "static": 0.005,
      "dynamic": 0.002,
      "viscous": 0.0001
    },
    "motor_constant": 0.0355,
    "winding_resistance": 2.52
  },
  "model_path": "res://parts_library/motors/maxon/ec45.glb",
  "price_usd": 245.00,
  "datasheet_url": "https://www.maxongroup.com/maxon/view/product/394159"
}
```

### 步骤 2: 验证数据

使用在线 JSON Schema 验证器或运行：

```gdscript
var is_valid = parts_lib.validate_part("maxon_ec45_flat")
if is_valid:
    print("✓ 零件数据有效")
```

### 步骤 3: 重新加载

重新启动 Godot 项目或手动调用：

```gdscript
parts_lib.load_parts_database()
```

## 🔧 高级用法

### 访问零件元数据

电机实例和关节都包含原始零件数据：

```gdscript
var motor = parts_lib.create_motor_instance("dynamixel_xl430_w250", self)

# 获取零件数据
var part_data = motor.get_meta("part_data")
print("制造商: ", part_data.get("manufacturer"))
print("数据手册: ", part_data.get("datasheet_url"))

# 获取特定参数
var stall_torque = motor.get_meta("stall_torque")
var friction = motor.get_meta("friction_params")
print("静摩擦: ", friction.get("static"), " N·m")
```

### 运行时调整参数

```gdscript
# 修改关节扭矩限制
var joint = parts_lib.create_motor_joint(...)
joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, 2.0)  # 自定义扭矩

# 模拟电机过载降额
var normal_torque = joint.get_meta("part_data")["specifications"]["stall_torque"]
var derated_torque = normal_torque * 0.8  # 降额 20%
joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, derated_torque)
```

## 🧪 测试示例

运行测试场景：

1. 在 Godot 编辑器中创建新场景
2. 添加 `Node3D` 根节点
3. 附加脚本 `res://scripts/test_parts_library.gd`
4. 运行场景 (F5)

预期输出：
```
🔧 开始加载零件库...
  ✓ 加载零件: dynamixel_xl430_w250 (XL430-W250-T)
  ✓ 加载零件: dynamixel_mx106 (MX-106T)
  ✓ 加载零件: bosch_bno055 (BNO055)
✅ 零件库加载完成，共 3 个零件

=== 零件库统计 ===
总零件数: 3
分类统计:
  - actuator_servo: 2
  - sensor_imu: 1
==================
```

## 🐛 故障排除

### 问题 1: 零件加载失败

**症状**: 控制台显示 "目录不存在" 警告

**解决**:
- 检查文件路径是否正确
- 确认 JSON 文件在正确的目录下
- 验证 `PARTS_ROOT` 常量指向 `res://parts_library/`

### 问题 2: JSON 解析错误

**症状**: "JSON 解析失败" 错误

**解决**:
- 使用 JSON 验证工具检查语法
- 确保没有多余的逗号
- 检查引号是否正确闭合

### 问题 3: 零件实例没有物理效果

**症状**: 创建的电机不会掉落或碰撞

**解决**:
- 确认场景中有 `StaticBody3D` 地面
- 检查碰撞层设置
- 验证 RigidBody3D 没有被设置为 `freeze`

## 📚 参考资源

- [Dynamixel 官方文档](https://emanual.robotis.com/)
- [Godot 物理引擎文档](https://docs.godotengine.org/en/stable/tutorials/physics/index.html)
- [JSON Schema 规范](https://json-schema.org/)

## 🔮 未来计划

- [ ] 添加更多品牌的电机（Faulhaber、Maxon、RoboMaster）
- [ ] 支持力/扭矩传感器
- [ ] 支持 LiDAR 和相机传感器
- [ ] 3D 模型库（GLB 格式）
- [ ] 在线零件数据库
- [ ] 可视化零件选择器 UI

---

**版本**: 0.1.0  
**最后更新**: 2026-01-13
