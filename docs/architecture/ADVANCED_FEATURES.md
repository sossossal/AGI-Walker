# AGI-Walker 高级功能扩展指南

本文档说明如何为AGI-Walker项目添加高级功能，包括更多机器人参数、运动路径规划、障碍物识别和平衡控制�?

---

## 📋 目录

1. [扩展机器人参数](#1-扩展机器人参�?
2. [运动路径规划](#2-运动路径规划)
3. [障碍物识别](#3-障碍物识�?
4. [平衡控制算法](#4-平衡控制算法)
5. [集成示例](#5-集成示例)

---

## 1. 扩展机器人参�?

### 1.1 添加更多关节

#### 当前结构�?自由度）
```
Robot
├── Torso
├── LeftLeg (髋关�?
└── RightLeg (髋关�?
```

#### 扩展�?自由度（添加膝关节）
```
Robot
├── Torso
├── LeftThigh (髋关�?
├── LeftCalf (膝关�?
├── RightThigh (髋关�?
└── RightCalf (膝关�?
```

#### Godot场景创建步骤

1. **添加大腿节点**
```
# LeftThigh �?RightThigh (RigidBody3D)
尺寸: 0.2 x 0.4 x 0.2
质量: 2kg
```

2. **添加小腿节点**
```
# LeftCalf �?RightCalf (RigidBody3D)
尺寸: 0.2 x 0.4 x 0.2
质量: 1.5kg
```

3. **配置关节**
```gdscript
# 髋关�?(连接 Torso �?Thigh)
HipLeft/HipRight (HingeJoint3D)
- 限位: -45° �?90°

# 膝关�?(连接 Thigh �?Calf)
KneeLeft/KneeRight (HingeJoint3D)
- 限位: -120° �?0° (只能向后�?
```

#### 修改GDScript

```gdscript
# box_robot.gd 扩展�?
extends Node3D

# 新增关节引用
@onready var left_thigh: RigidBody3D = get_node_or_null("LeftThigh")
@onready var left_calf: RigidBody3D = get_node_or_null("LeftCalf")
@onready var knee_left: HingeJoint3D = get_node_or_null("KneeLeft")
@onready var knee_right: HingeJoint3D = get_node_or_null("KneeRight")

# 扩展关节角度字典
var joint_angles := {
    "hip_left": 0.0,
    "hip_right": 0.0,
    "knee_left": 0.0,    # 新增
    "knee_right": 0.0    # 新增
}

# 扩展目标角度字典
var target_angles := {
    "hip_left": 0.0,
    "hip_right": 0.0,
    "knee_left": 0.0,
    "knee_right": 0.0
}
```

### 1.2 添加脚踝关节�?自由度）

继续扩展可添加：
- **AnkleLeft/AnkleRight**: 脚踝俯仰
- **WaistJoint**: 躯干旋转
- **ArmJoints**: 手臂摆动（辅助平衡）

---

## 2. 运动路径规划

### 2.1 路径表示

创建路径管理器：

```gdscript
# path_manager.gd
extends Node3D

# 路径点列�?
var waypoints: Array[Vector3] = []
var current_waypoint_index := 0

# 路径可视�?
@onready var path_line: MeshInstance3D


func _ready():
    # 示例路径：正方形
    waypoints = [
        Vector3(0, 0, 0),
        Vector3(2, 0, 0),
        Vector3(2, 0, 2),
        Vector3(0, 0, 2),
        Vector3(0, 0, 0)
    ]
    _draw_path()


func get_current_target() -> Vector3:
    """获取当前目标�?""
    if current_waypoint_index < waypoints.size():
        return waypoints[current_waypoint_index]
    return Vector3.ZERO


func advance_waypoint():
    """前进到下一个路径点"""
    current_waypoint_index += 1
    if current_waypoint_index >= waypoints.size():
        print("�?路径完成!")


func is_near_target(robot_pos: Vector3, threshold: float = 0.5) -> bool:
    """检查是否接近目标点"""
    var target = get_current_target()
    return robot_pos.distance_to(target) < threshold


func _draw_path():
    """可视化路�?""
    # 使用ImmediateMesh绘制线条
    var mesh = ImmediateMesh.new()
    mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
    
    for point in waypoints:
        mesh.surface_add_vertex(point)
    
    mesh.surface_end()
    
    if path_line:
        path_line.mesh = mesh
```

### 2.2 导航控制�?

```python
# navigation_controller.py
import math
from tcp_client import GodotClient

class NavigationController:
    """路径跟随控制�?""
    
    def __init__(self, client: GodotClient):
        self.client = client
        self.waypoints = [
            (0, 0),
            (2, 0),
            (2, 2),
            (0, 2)
        ]
        self.current_waypoint = 0
        
    def calculate_heading_angle(self, robot_pos, target_pos):
        """计算朝向目标的转向角"""
        dx = target_pos[0] - robot_pos[0]
        dz = target_pos[1] - robot_pos[1]
        return math.atan2(dz, dx)  # 弧度
    
    def get_motor_commands(self, sensor_data):
        """根据当前位置生成电机指令"""
        # 获取机器人位置（需要添加位置追踪）
        robot_x = sensor_data.get('position_x', 0)
        robot_z = sensor_data.get('position_z', 0)
        
        # 当前目标�?
        target = self.waypoints[self.current_waypoint]
        
        # 计算距离
        dist = math.sqrt((target[0] - robot_x)**2 + (target[1] - robot_z)**2)
        
        # 到达目标点，切换下一�?
        if dist < 0.5:
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
            print(f"�?到达路径�?{self.current_waypoint}")
        
        # 计算转向
        heading = self.calculate_heading_angle((robot_x, robot_z), target)
        
        # 简单的差速驱动（左右腿不同步�?
        turn_gain = 10  # 转向增益
        return {
            "motors": {
                "hip_left": heading * turn_gain,
                "hip_right": -heading * turn_gain
            }
        }
```

---

## 3. 障碍物识�?

### 3.1 距离传感�?

#### Godot�?- 射线检�?

```gdscript
# obstacle_detector.gd
extends Node3D

# 射线传感器数�?
var ray_sensors: Array[RayCast3D] = []
const NUM_RAYS = 5  # 5个方�?
const RAY_LENGTH = 3.0  # 3米检测距�?

func _ready():
    _create_ray_sensors()

func _create_ray_sensors():
    """创建多方向射线传感器"""
    var angles = [-45, -22.5, 0, 22.5, 45]  # �?
    
    for i in range(NUM_RAYS):
        var ray = RayCast3D.new()
        add_child(ray)
        
        # 设置方向
        var angle_rad = deg_to_rad(angles[i])
        ray.target_position = Vector3(
            sin(angle_rad) * RAY_LENGTH,
            0,
            cos(angle_rad) * RAY_LENGTH
        )
        
        ray.enabled = true
        ray_sensors.append(ray)

func get_obstacle_distances() -> Array[float]:
    """获取各方向的障碍物距�?""
    var distances: Array[float] = []
    
    for ray in ray_sensors:
        if ray.is_colliding():
            var collision_point = ray.get_collision_point()
            var dist = global_position.distance_to(collision_point)
            distances.append(dist)
        else:
            distances.append(RAY_LENGTH)  # 无障�?
    
    return distances

func get_sensor_data() -> Dictionary:
    """格式化传感器数据"""
    var distances = get_obstacle_distances()
    return {
        "obstacle_distances": distances,
        "closest_obstacle": distances.min(),
        "has_obstacle": distances.min() < 1.0  # 1米内有障�?
    }
```

#### 在主机器人中集成

```gdscript
# box_robot.gd (添加)
@onready var obstacle_detector = $ObstacleDetector

func get_sensor_data() -> Dictionary:
    # ... 原有代码 ...
    return {
        "timestamp": Time.get_ticks_msec() / 1000.0,
        "sensors": {
            "imu": _get_imu_data(),
            "joints": _get_joint_data(),
            "contacts": _get_contact_data(),
            "obstacles": obstacle_detector.get_sensor_data()  # 新增
        },
        "torso_height": torso.global_position.y
    }
```

### 3.2 视觉传感器（Camera�?

```gdscript
# vision_sensor.gd
extends Camera3D

var viewport: SubViewport

func _ready():
    # 创建离屏渲染
    viewport = SubViewport.new()
    viewport.size = Vector2i(320, 240)  # 低分辨率
    add_child(viewport)

func capture_image() -> Image:
    """捕获相机图像"""
    await RenderingServer.frame_post_draw
    return viewport.get_texture().get_image()

func detect_objects() -> Array:
    """简单的物体检测（颜色识别�?""
    var image = capture_image()
    var objects = []
    
    # 示例：检测红色物�?
    # 实际应用中可以集成计算机视觉算法
    
    return objects
```

---

## 4. 平衡控制算法

### 4.1 PID控制�?

```gdscript
# pid_controller.gd
class_name PIDController

var kp: float  # 比例增益
var ki: float  # 积分增益
var kd: float  # 微分增益

var integral: float = 0.0
var last_error: float = 0.0

func _init(p: float, i: float, d: float):
    kp = p
    ki = i
    kd = d

func compute(error: float, dt: float) -> float:
    """计算PID输出"""
    # 积分�?
    integral += error * dt
    
    # 微分�?
    var derivative = (error - last_error) / dt if dt > 0 else 0.0
    
    # PID公式
    var output = kp * error + ki * integral + kd * derivative
    
    last_error = error
    return output

func reset():
    """重置状�?""
    integral = 0.0
    last_error = 0.0
```

### 4.2 姿态平衡控�?

```gdscript
# balance_controller.gd
extends Node

@onready var robot = get_node("/root/Main/Robot")

# PID控制�?
var roll_pid: PIDController
var pitch_pid: PIDController

func _ready():
    # 调优的PID参数
    roll_pid = PIDController.new(5.0, 0.1, 2.0)
    pitch_pid = PIDController.new(5.0, 0.1, 2.0)

func compute_balance_commands(sensor_data: Dictionary, dt: float) -> Dictionary:
    """计算平衡控制指令"""
    var orient = sensor_data['sensors']['imu']['orient']
    var roll = orient[0]
    var pitch = orient[1]
    
    # 目标姿态：直立�?度）
    var roll_error = 0.0 - roll
    var pitch_error = 0.0 - pitch
    
    # PID计算
    var roll_correction = roll_pid.compute(roll_error, dt)
    var pitch_correction = pitch_pid.compute(pitch_error, dt)
    
    # 转换为电机指令（简化版�?
    return {
        "motors": {
            "hip_left": pitch_correction + roll_correction,
            "hip_right": pitch_correction - roll_correction
        }
    }
```

### 4.3 ZMP（零力矩点）平衡

```python
# zmp_controller.py
import numpy as np

class ZMPController:
    """零力矩点平衡控制�?""
    
    def __init__(self, robot_height=1.0, gravity=9.8):
        self.height = robot_height
        self.g = gravity
        
    def calculate_zmp(self, com_pos, com_acc):
        """
        计算ZMP位置
        com_pos: 重心位置 [x, y, z]
        com_acc: 重心加速度 [ax, ay, az]
        """
        x_com, y_com, z_com = com_pos
        ax, ay, az = com_acc
        
        # ZMP公式
        x_zmp = x_com - (z_com / (az - self.g)) * ax
        y_zmp = y_com - (z_com / (az - self.g)) * ay
        
        return [x_zmp, y_zmp]
    
    def is_stable(self, zmp, support_polygon):
        """
        检查ZMP是否在支撑多边形�?
        zmp: [x, y]
        support_polygon: [(x1,y1), (x2,y2), ...]
        """
        # 使用射线法判断点是否在多边形�?
        # 简化版本：假设支撑多边形是矩形
        x, y = zmp
        min_x = min(p[0] for p in support_polygon)
        max_x = max(p[0] for p in support_polygon)
        min_y = min(p[1] for p in support_polygon)
        max_y = max(p[1] for p in support_polygon)
        
        return min_x <= x <= max_x and min_y <= y <= max_y

    def compute_correction(self, zmp, support_center):
        """计算平衡修正"""
        error_x = support_center[0] - zmp[0]
        error_y = support_center[1] - zmp[1]
        
        # 简单比例控�?
        gain = 2.0
        return [error_x * gain, error_y * gain]
```

### 4.4 重心计算

```gdscript
# center_of_mass.gd
extends Node

func calculate_com(bodies: Array[RigidBody3D]) -> Vector3:
    """计算系统重心"""
    var total_mass = 0.0
    var weighted_pos = Vector3.ZERO
    
    for body in bodies:
        var mass = body.mass
        var pos = body.global_position
        
        weighted_pos += pos * mass
        total_mass += mass
    
    if total_mass > 0:
        return weighted_pos / total_mass
    return Vector3.ZERO

func calculate_support_polygon(contact_points: Array[Vector3]) -> Array:
    """计算支撑多边�?""
    # 返回脚底接触点围成的多边�?
    return contact_points
```

---

## 5. 集成示例

### 5.1 完整的AI控制循环

```python
# advanced_controller.py
import time
from tcp_client import GodotClient
from navigation_controller import NavigationController
from zmp_controller import ZMPController

class AdvancedController:
    """集成所有高级功能的控制�?""
    
    def __init__(self, model_path: str):
        self.client = GodotClient()
        self.navigator = NavigationController(self.client)
        self.zmp = ZMPController()
        
        # AI模型（示例）
        # self.ai_model = load_model(model_path)
        
    def run(self, duration: float = 120.0):
        """运行高级控制循环"""
        self.client.connect()
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # 1. 获取传感器数�?
            sensor_data = self.client.get_latest_sensors()
            if not sensor_data:
                continue
            
            # 2. 障碍物检�?
            if 'obstacles' in sensor_data['sensors']:
                obstacles = sensor_data['sensors']['obstacles']
                if obstacles['has_obstacle']:
                    print(f"⚠️ 检测到障碍物，距离: {obstacles['closest_obstacle']:.2f}m")
                    # 避障逻辑
            
            # 3. 平衡控制
            orient = sensor_data['sensors']['imu']['orient']
            roll, pitch = orient[0], orient[1]
            
            if abs(roll) > 30 or abs(pitch) > 30:
                print("⚠️ 姿态不稳定，执行平衡恢�?)
                # 平衡恢复逻辑
            
            # 4. 路径导航
            nav_commands = self.navigator.get_motor_commands(sensor_data)
            
            # 5. AI推理（如果启用）
            # ai_commands = self.ai_model.predict(sensor_data)
            
            # 6. 融合指令并发�?
            self.client.send_motor_commands(nav_commands)
            
            time.sleep(0.033)  # 30Hz
        
        self.client.close()
```

### 5.2 参数配置文件

```python
# config.py
"""
高级功能配置参数
"""

# 路径规划
PATH_WAYPOINTS = [
    (0, 0),
    (5, 0),
    (5, 5),
    (0, 5)
]

# 障碍物检�?
OBSTACLE_DETECTION_RANGE = 3.0  # �?
OBSTACLE_AVOIDANCE_DISTANCE = 1.0  # �?
NUM_DISTANCE_SENSORS = 5

# 平衡控制
PID_ROLL = {"kp": 5.0, "ki": 0.1, "kd": 2.0}
PID_PITCH = {"kp": 5.0, "ki": 0.1, "kd": 2.0}

# ZMP
ROBOT_HEIGHT = 1.0
GRAVITY = 9.8
STABILITY_MARGIN = 0.05  # �?

# 关节限位
JOINT_LIMITS = {
    "hip": (-45, 90),
    "knee": (-120, 0),
    "ankle": (-30, 30)
}
```

---

## 6. 实施路线�?

### 阶段1: 扩展关节�?周）
- [ ] 添加膝关节和脚踝
- [ ] 更新传感器数据结�?
- [ ] 测试新关节控�?

### 阶段2: 障碍物检测（1周）
- [ ] 实现射线传感�?
- [ ] 集成到传感器数据�?
- [ ] Python端解析障碍物信息

### 阶段3: 路径规划�?周）
- [ ] 创建路径管理�?
- [ ] 实现路径跟随算法
- [ ] 可视化路�?

### 阶段4: 平衡控制�?周）
- [ ] 实现PID控制�?
- [ ] 集成ZMP算法
- [ ] 调优参数

### 阶段5: 集成测试�?周）
- [ ] 联合测试所有功�?
- [ ] 性能优化
- [ ] 文档完善

---

## 7. 调试工具

### 可视化调试器

```gdscript
# debug_overlay.gd
extends Control

@onready var label = $Label

func _process(_delta):
    var robot = get_node("/root/Main/Robot")
    if robot and robot.is_scene_ready:
        var sensor_data = robot.get_sensor_data()
        
        var text = "=== 调试信息 ===\n"
        text += "姿�? Roll=%.1f° Pitch=%.1f°\n" % [
            sensor_data['sensors']['imu']['orient'][0],
            sensor_data['sensors']['imu']['orient'][1]
        ]
        text += "高度: %.2fm\n" % sensor_data['torso_height']
        text += "接地: L=%s R=%s\n" % [
            "�? if sensor_data['sensors']['contacts']['foot_left'] else "�?,
            "�? if sensor_data['sensors']['contacts']['foot_right'] else "�?
        ]
        
        label.text = text
```

---

## 8. 参考资�?

- **机器人学**: 《Modern Robotics�? Kevin Lynch
- **步态规�?*: 《Biped Locomotion�?
- **ZMP理论**: Vukobratović, M. (1972)
- **PID调优**: Ziegler-Nichols方法

---

> 💡 **建议**: 从简单开始，先完�?自由度模型的平衡控制，再逐步增加关节和传感器。每添加一个功能都要充分测试�?
