# 📚 完整 API 参考文档

## 目录

- [Python API](#python-api)
- [GDScript API](#gdscript-api)
- [零件数据格式](#零件数据格式)
- [环境配置](#环境配置)

---

## Python API

### PartsDatabase

零件数据库接口。

#### 初始化

```python
from godot_robot_env import PartsDatabase

db = PartsDatabase(parts_library_path=None)
```

**参数**:
- `parts_library_path` (str, optional): 零件库路径，默认为项目中的 `parts_library` 目录

#### 方法

##### get_part(part_id: str) → Dict

获取零件数据。

**参数**:
- `part_id` (str): 零件ID

**返回**: 零件数据字典，如果不存在返回 `None`

**示例**:
```python
motor = db.get_part("dynamixel_xl430_w250")
print(motor['specifications']['stall_torque'])  # 1.4
```

##### get_parts_by_category(category: str) → List[Dict]

按类别获取零件列表。

**参数**:
- `category` (str): 类别名称（如 `"actuator_servo"）

**返回**: 零件列表

**示例**:
```python
servos = db.get_parts_by_category("actuator_servo")
for servo in servos:
    print(servo['model'])
```

##### list_all_parts() → List[str]

列出所有零件ID。

**返回**: 零件ID列表

##### validate_part(part_id: str) → bool

验证零件数据完整性。

**返回**: 是否有效

##### create_robot_config(parts_spec: List[Dict]) → Dict

从零件列表创建机器人配置。

**参数**:
- `parts_spec` (list): 零件规格列表

**示例**:
```python
config = db.create_robot_config([
    {"part_id": "dynamixel_xl430_w250", "joint": "hip_left"},
    {"part_id": "dynixel_xl430_w250", "joint": "hip_right"}
])
```

---

### GodotRobotEnv

Gymnasium 兼容的机器人仿真环境。

#### 初始化

```python
from godot_robot_env import GodotRobotEnv

env = GodotRobotEnv(
    robot_config=None,
    physics_config=None,
    host="127.0.0.1",
    port=9999,
    timeout=10.0
)
```

**参数**:
- `robot_config` (dict, optional): 机器人配置
- `physics_config` (dict, optional): 物理参数配置
- `host` (str): Godot 服务器地址
- `port` (int): 端口号
- `timeout` (float): 连接超时（秒）

#### 属性

##### observation_space

观察空间（Gymnasium Dict）。

**结构**:
```python
{
    'imu_orient': Box(3,),        # 姿态 (roll, pitch, yaw)
    'imu_angular_vel': Box(3,),   # 角速度
    'imu_linear_acc': Box(3,),    # 线性加速度
    'joint_angles': Box(4,),      # 关节角度
    'joint_velocities': Box(4,),  # 关节速度
    'joint_torques': Box(4,),     # 关节扭矩
    'foot_contacts': MultiBinary(2,), # 脚部接触
    'torso_height': Box(1,)       # 躯干高度
}
```

##### action_space

动作空间（Gymnasium Box）。

**形状**: (4,) - 4个关节的目标角度（度）  
**范围**: 见文档说明

#### 方法

##### reset() → Tuple[Dict, Dict]

重置环境。

**返回**: `(observation, info)`

**示例**:
```python
obs, info = env.reset()
```

##### step(action) → Tuple[Dict, float, bool, bool, Dict]

执行一步。

**参数**:
- `action` (np.ndarray): 动作（4个关节角度）

**返回**: `(observation, reward, terminated, truncated, info)`

**示例**:
```python
action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
```

##### set_physics_params(params: Dict)

动态修改物理参数。

**参数**:
- `params` (dict): 参数字典

**示例**:
```python
env.set_physics_params({
    "gravity": 3.71,  # 火星重力
    "ground_material": "sand"
})
```

##### close()

关闭环境。

---

## GDScript API

### EnvironmentController

环境参数控制器。

#### 方法

##### load_preset(preset_name: String)

加载环境预设。

**参数**:
- `preset_name`: 预设名称（`"earth"`, `"moon"`, `"mars"`, `"jupiter"`）

**示例**:
```gdscript
$EnvironmentController.load_preset("moon")
```

##### set_gravity(value: float)

设置重力。

**参数**:
- `value`: 重力值（m/s²）

##### set_air_density(value: float)

设置空气密度。

**参数**:
- `value`: 空气密度（kg/m³）

##### set_temperature(value: float)

设置温度。

**参数**:
- `value`: 温度（°C）

##### calculate_air_drag(velocity: Vector3, cross_section: float, drag_coef: float = 0.47) → Vector3

计算空气阻力。

**返回**: 阻力向量（N）

##### get_environment_info() → Dictionary

获取当前环境信息。

**返回**: 包含所有参数的字典

---

### GroundMaterialLibrary

地面材质库。

#### 方法

##### apply_material(ground: StaticBody3D, material_name: String)

应用材质到地面。

**参数**:
- `ground`: 地面物体
- `material_name`: 材质名称

**可用材质**:
- `"concrete"` - 混凝土
- `"wood"` - 木板
- `"carpet"` - 地毯
- `"ice"` - 冰面
- `"metal"` - 金属
- `"sand"` - 沙地
- `"grass"` - 草地
- `"mud"` - 泥地

**示例**:
```gdscript
$GroundMaterialLibrary.apply_material($Ground, "ice")
```

##### get_material(material_name: String) → GroundMaterial

获取材质对象。

##### list_materials() → Array[String]

列出所有材质名称。

---

## 零件数据格式

### 电机/舵机

```json
{
  "part_id": "unique_id",
  "category": "actuator_servo",
  "manufacturer": "Manufacturer Name",
  "model": "Model Number",
  "specifications": {
    "stall_torque": 1.4,          // N·m
    "no_load_speed": 50,          // RPM
    "weight": 0.057,              // kg
    "voltage_range": [6, 12],     // V
    "max_current": 1.4,           // A
    "friction": {
      "static": 0.01,
      "dynamic": 0.005,
      "viscous": 0.0001
    },
    "thermal": {
      "resistance": 10.0,         // °C/W
      "time_constant": 1500,      // s
      "max_winding_temp": 150     // °C
    }
  },
  "price_usd": 69.90
}
```

### IMU 传感器

```json
{
  "part_id": "unique_id",
  "category": "sensor_imu",
  "manufacturer": "Manufacturer Name",
  "model": "Model Number",
  "specifications": {
    "accelerometer_range": 16,    // g
    "gyroscope_range": 2000,      // dps
    "magnetometer_range": 16,     // gauss
    "update_rate": 100,           // Hz
    "noise_density": {
      "accel": 150,               // μg/√Hz
      "gyro": 0.014               // °/s/√Hz
    }
  },
  "price_usd": 34.95
}
```

---

## 环境配置

### 物理参数

```python
physics_config = {
    "gravity": 9.81,              # m/s²
    "air_density": 1.225,         # kg/m³
    "temperature": 25.0,          # °C
    "ground_material": "concrete",
    "wind_velocity": {            # m/s
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    }
}
```

### 环境预设

| 预设 | 重力 | 空气密度 | 温度 |
|------|------|----------|------|
| earth | 9.81 | 1.225 | 25 |
| moon | 1.62 | 0.0 | -20 |
| mars | 3.71 | 0.02 | -60 |
| jupiter | 24.79 | 0.16 | -110 |

---

## 常量和枚举

### 材质摩擦系数

| 材质 | 摩擦系数 | 弹性 |
|------|----------|------|
| concrete | 0.9 | 0.1 |
| wood | 0.6 | 0.2 |
| carpet | 1.0 | 0.05 |
| ice | 0.1 | 0.3 |
| metal | 0.4 | 0.4 |
| sand | 0.7 | 0.0 |
| grass | 0.75 | 0.1 |
| mud | 0.85 | 0.0 |

---

## 错误处理

### 常见异常

```python
# 连接失败
RuntimeError: "Not connected to Godot simulator"

# 零件未找到
返回 None（不抛异常）

# 无效参数
参数被 clamp 到有效范围
```

---

**文档版本**: 1.0  
**最后更新**: 2026-01-15
