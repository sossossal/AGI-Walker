# 🎓 进阶使用指南

本指南将帮助您深入使�?Godot 机器人模拟套件，创建自己的机器人、添加新零件、进行高级训练�?

---

## 📋 目录

1. [创建自定义机器人](#1-创建自定义机器人)
2. [添加新零件到零件库](#2-添加新零件到零件�?
3. [高级环境配置](#3-高级环境配置)
4. [域随机化训练](#4-域随机化训练)
5. [性能优化技巧](#5-性能优化技�?

---

## 1. 创建自定义机器人

### 1.1 设计机器人结�?

假设我们要创建一�?6-DOF 双足机器人（每条�?个关节）�?

**步骤**:

#### A. 选择零件

```python
from godot_robot_env import PartsDatabase

db = PartsDatabase()

# 为髋关节选择大扭矩电�?
hip_motor = db.get_part("dynamixel_mx106")  # 8.4 N·m
print(f"髋关节电�? {hip_motor['model']}, {hip_motor['specifications']['stall_torque']} N·m")

# 为膝盖和踝关节选择中等扭矩电机
knee_motor = db.get_part("dynamixel_xl430_w250")  # 1.4 N·m
print(f"膝盖/踝关节电�? {knee_motor['model']}, {knee_motor['specifications']['stall_torque']} N·m")

# 选择 IMU
imu = db.get_part("bosch_bno055")
print(f"IMU: {imu['model']}")
```

#### B. 定义机器人配�?

创建文件 `custom_robots/walker_6dof.json`:

```json
{
  "name": "Walker6DOF",
  "description": "6自由度双足步行机器人",
  "parts": [
    {"part_id": "dynamixel_mx106", "joint": "hip_left", "position": "torso"},
    {"part_id": "dynamixel_mx106", "joint": "hip_right", "position": "torso"},
    {"part_id": "dynamixel_xl430_w250", "joint": "knee_left", "position": "left_thigh"},
    {"part_id": "dynamixel_xl430_w250", "joint": "knee_right", "position": "right_thigh"},
    {"part_id": "dynamixel_xl430_w250", "joint": "ankle_left", "position": "left_calf"},
    {"part_id": "dynamixel_xl430_w250", "joint": "ankle_right", "position": "right_calf"},
    {"part_id": "bosch_bno055", "location": "torso"}
  ],
  "dimensions": {
    "torso_height": 0.4,
    "thigh_length": 0.25,
    "calf_length": 0.25,
    "foot_size": [0.15, 0.08]
  },
  "total_mass": 3.5,
  "estimated_cost": 1150.0
}
```

#### C. �?Python 中加载配�?

```python
import json

# 加载配置
with open("custom_robots/walker_6dof.json") as f:
    robot_config = json.load(f)

# 使用零件库扩展配�?
db = PartsDatabase()
extended_config = db.create_robot_config(robot_config["parts"])

# 计算总成�?
total_cost = sum(
    db.get_part(part["part_id"])["price_usd"]
    for part in robot_config["parts"]
    if db.get_part(part["part_id"])
)
print(f"总成�? ${total_cost:.2f}")

# 计算总质�?
total_mass = sum(
    db.get_part(part["part_id"])["specifications"]["weight"]
    for part in robot_config["parts"]
    if db.get_part(part["part_id"]) and "weight" in db.get_part(part["part_id"])["specifications"]
)
print(f"零件总质�? {total_mass:.3f} kg")
```

### 1.2 �?Godot 中构建机器人

�?Godot 中创建场�?`custom_walker.tscn`:

```
Walker6DOF (Node3D)
├── Torso (RigidBody3D)
�?  ├── CollisionShape3D (BoxShape3D: 0.2x0.4x0.15)
�?  └── MeshInstance3D
�?
├── LeftThigh (RigidBody3D)
�?  ├── CollisionShape3D (CapsuleShape3D)
�?  └── MeshInstance3D
�?
├── LeftCalf (RigidBody3D)
�?  ├── CollisionShape3D (CapsuleShape3D)
�?  └── MeshInstance3D
�?
├── LeftFoot (RigidBody3D)
�?  ├── CollisionShape3D (BoxShape3D)
�?  └── MeshInstance3D
�?
├── (右腿类似...)
�?
├── HipLeft (HingeJoint3D)
├── KneeLeft (HingeJoint3D)
├── AnkleLeft (HingeJoint3D)
└── (右侧关节类似...)
```

#### 应用零件规格脚本

```gdscript
# walker_6dof.gd
extends Node3D

@onready var parts_lib = preload("res://addons/robot_sim_toolkit/scripts/parts_manager.gd").new()

func _ready():
	parts_lib.load_parts_database("res://parts_library")
	apply_part_specs()

func apply_part_specs():
	# 髋关�?- 使用 MX-106
	var mx106 = parts_lib.get_part("dynamixel_mx106")
	if mx106:
		apply_motor_to_joint($HipLeft, mx106)
		apply_motor_to_joint($HipRight, mx106)
	
	# 膝盖和踝关节 - 使用 XL430
	var xl430 = parts_lib.get_part("dynamixel_xl430_w250")
	if xl430:
		apply_motor_to_joint($KneeLeft, xl430)
		apply_motor_to_joint($KneeRight, xl430)
		apply_motor_to_joint($AnkleLeft, xl430)
		apply_motor_to_joint($AnkleRight, xl430)

func apply_motor_to_joint(joint: HingeJoint3D, motor_data: Dictionary):
	var specs = motor_data["specifications"]
	joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, specs["stall_torque"])
	joint.set_meta("part_id", motor_data["part_id"])
	joint.set_meta("stall_torque", specs["stall_torque"])
	print("�?Applied ", motor_data["model"], " to ", joint.name)
```

---

## 2. 添加新零件到零件�?

### 2.1 寻找零件规格

以添�?**Faulhaber 2657 CR** 电机为例�?

#### A. 收集数据

从制造商数据手册收集�?
- 堵转扭矩: 0.134 N·m (at 24V)
- 空载速度: 7200 RPM
- 重量: 126 g
- 尺寸: Ø26 × 57 mm
- 电压范围: 6-24 V
- 最大电�? 4.23 A
- 价格: ~$180 USD

#### B. 创建 JSON 文件

`parts_library/motors/faulhaber/2657cr.json`:

```json
{
  "part_id": "faulhaber_2657cr",
  "category": "actuator_motor",
  "manufacturer": "Faulhaber",
  "model": "2657 CR",
  "description": "精密无刷直流电机，带霍尔传感�?,
  "datasheet_url": "https://www.faulhaber.com/en/products/series/2657cr/",
  
  "specifications": {
    "stall_torque": 0.134,
    "no_load_speed": 7200,
    "weight": 0.126,
    "dimensions": [26, 57, 26],
    "gear_ratio": 1.0,
    
    "voltage_range": [6, 24],
    "rated_voltage": 24,
    "max_current": 4.23,
    "no_load_current": 0.18,
    
    "resolution": null,
    "rotor_inertia": 2.8e-6,
    
    "friction": {
      "static": 0.0013,
      "dynamic": 0.0007,
      "viscous": 0.0001
    },
    
    "thermal": {
      "resistance": 3.6,
      "time_constant": 2800,
      "max_winding_temp": 155
    },
    
    "motor_constant": 0.0317,
    "back_emf_constant": 0.0333,
    "winding_resistance": 0.89
  },
  
  "price_usd": 180.0,
  "availability": "commercial",
  "notes": "高性能无刷电机，适合精密应用"
}
```

#### C. 验证数据

```python
# 验证新零�?
from godot_robot_env import PartsDatabase

db = PartsDatabase()
db.print_statistics()

# 获取新零�?
faulhaber = db.get_part("faulhaber_2657cr")
if faulhaber:
    print("\n�?新零件加载成�?")
    print(f"型号: {faulhaber['model']}")
    print(f"扭矩: {faulhaber['specifications']['stall_torque']} N·m")
    print(f"速度: {faulhaber['specifications']['no_load_speed']} RPM")
else:
    print("�?零件加载失败")
```

### 2.2 添加传感�?

以添�?**Livox Mid-40 激光雷�?*为例�?

#### A. 创建传感�?Schema

`parts_library/schema/sensor_lidar.schema.json`:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "title": "LiDAR Sensor",
  "type": "object",
  "required": ["part_ "category", "manufacturer", "model", "specifications"],
  "properties": {
    "part_id": {"type": "string"},
    "category": {"enum": ["sensor_lidar"]},
    "manufacturer": {"type": "string"},
    "model": {"type": "string"},
    "specifications": {
      "type": "object",
      "required": ["range", "fov", "points_per_second", "accuracy"],
      "properties": {
        "range": {
          "type": "object",
          "properties": {
            "min": {"type": "number", "minimum": 0},
            "max": {"type": "number", "minimum": 0}
          }
        },
        "fov": {
          "type": "object",
          "properties": {
            "horizontal": {"type": "number"},
            "vertical": {"type": "number"}
          }
        },
        "points_per_second": {"type": "integer", "minimum": 0},
        "accuracy": {"type": "number"},
        "wavelength": {"type": "number"},
        "power_consumption": {"type": "number"},
        "weight": {"type": "number"},
        "dimensions": {
          "type": "array",
          "items": {"type": "number"},
          "minItems": 3,
          "maxItems": 3
        }
      }
    }
  }
}
```

#### B. 创建零件数据

`parts_library/sensors/lidar/livox_mid40.json`:

```json
{
  "part_id": "livox_mid40",
  "category": "sensor_lidar",
  "manufacturer": "Livox",
  "model": "Mid-40",
  
  "specifications": {
    "range": {
      "min": 0.5,
      "max": 260
    },
    "fov": {
      "horizontal": 38.4,
      "vertical": 38.4
    },
    "points_per_second": 100000,
    "accuracy": 0.02,
    "wavelength": 905,
    "power_consumption": 8.0,
    "weight": 0.76,
    "dimensions": [127, 88, 73]
  },
  
  "price_usd": 599.0,
  "datasheet_url": "https://www.livoxtech.com/mid-40-and-mid-100"
}
```

---

## 3. 高级环境配置

### 3.1 创建自定义环境预�?

```gdscript
# custom_environments.gd
extends Node

const CUSTOM_PRESETS = {
	"underwater": {
		"gravity": 9.81 * 0.85,  # 水中浮力
		"air_density": 1000.0,  # 水密�?
		"temperature": 10.0,
		"name": "水下环境"
	},
	"high_altitude": {
		"gravity": 9.81,
		"air_density": 0.4,  # 高海拔稀薄空�?
		"temperature": -20.0,
		"name": "高海�?
	},
	"low_gravity_high_friction": {
		"gravity": 3.0,
		"air_density": 1.225,
		"temperature": 25.0,
		"ground_material": "carpet",
		"name": "低重力高摩擦"
	}
}

func apply_custom_preset(env_controller, preset_name: String):
	if CUSTOM_PRESETS.has(preset_name):
		var preset = CUSTOM_PRESETS[preset_name]
		env_controller.from_dict(preset)
		print("�?Applied custom preset: ", preset["name"])
```

### 3.2 动态地形生�?

```gdscript
# terrain_generator.gd
extends Node3D

@export var terrain_size: Vector2 = Vector2(20, 20)
@export var tile_size: float = 1.0

func generate_varied_terrain(material_lib):
	# 创建棋盘格地形，每格不同材质
	var materials = ["concrete", "wood", "sand", "grass"]
	
	for x in range(int(terrain_size.x)):
		for z in range(int(terrain_size.y)):
			var tile = create_ground_tile(
				Vector3(x * tile_size, 0, z * tile_size),
				tile_size
			)
			
			# 随机选择材质
			var mat = materials[randi() % materials.size()]
			material_lib.apply_material(tile, mat)
			
			add_child(tile)

func create_ground_tile(position: Vector3, size: float) -> StaticBody3D:
	var tile = StaticBody3D.new()
	tile.position = position
	
	var shape = CollisionShape3D.new()
	var box = BoxShape3D.new()
	box.size = Vector3(size, 0.1, size)
	shape.shape = box
	tile.add_child(shape)
	
	return tile
```

---

## 4. 域随机化训练

### 4.1 Python 端实�?

创建文件 `python_api/examples/domain_randomization_training.py`:

```python
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from godot_robot_env import GodotRobotEnv

class DomainRandomizationWrapper(gym.Wrapper):
    """域随机化包装�?""
    
    def __init__(self, env, randomize_params=None):
        super().__init__(env)
        self.randomize_params = randomize_params or {
            "gravity": (7.0, 12.0),
            "air_density": (0.5, 2.0),
            "temperature": (-20.0, 40.0),
            "ground_materials": ["concrete", "wood", "ice", "sand"]
        }
    
    def reset(self, **kwargs):
        # 每个 episode 开始时随机化环�?
        self._randomize_environment()
        return self.env.reset(**kwargs)
    
    def _randomize_environment():
        """随机化环境参�?""
        params = {}
        
        # 随机重力
        if "gravity" in self.randomize_params:
            g_min, g_max = self.randomize_params["gravity"]
            params["gravity"] = np.random.uniform(g_min, g_max)
        
        # 随机空气密度
        if "air_density" in self.randomize_params:
            rho_min, rho_max = self.randomize_params["air_density"]
            params["air_density"] = np.random.uniform(rho_min, rho_max)
        
        # 随机温度
        if "temperature" in self.randomize_params:
            t_min, t_max = self.randomize_params["temperature"]
            params["temperature"] = np.random.uniform(t_min, t_max)
        
        # 随机地面材质
        if "ground_materials" in self.randomize_params:
            materials = self.randomize_params["ground_materials"]
            params["ground_material"] = np.random.choice(materials)
        
        # 应用到环�?
        self.env.set_physics_params(params)
        
        print(f"🎲 Randomized: g={params.get('gravity', 9.81):.2f}, "
              f"ρ={params.get('air_density', 1.225):.3f}, "
              f"mat={params.get('ground_material', 'concrete')}")

# 使用示例
env = GodotRobotEnv()
env = DomainRandomizationWrapper(env)

model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
model.save("walker_domain_randomized")
```

### 4.2 结果评估

```python
# evaluate_robustness.py
def evaluate_on_varied_environments(model, env, num_episodes=10):
    """在不同环境中评估策略鲁棒�?""
    
    test_envs = [
        {"name": "Earth", "gravity": 9.81, "ground_material": "concrete"},
        {"name": "Moon", "gravity": 1.62, "ground_material": "sand"},
        {"name": "Ice", "gravity": 9.81, "ground_material": "ice"},
        {"name": "High-G", "gravity": 15.0, "ground_material": "concrete"},
    ]
    
    results = {}
    
    for test_env in test_envs:
        env.set_physics_params(test_env)
        
        episode_rewards = []
        for _ in range(num_episodes):
            obs, _ = env.reset()
            done = False
            total_reward = 0
            
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = env.step(action)
                total_reward += reward
                done = terminated or truncated
            
            episode_rewards.append(total_reward)
        
        results[test_env["name"]] = {
            "mean": np.mean(episode_rewards),
            "std": np.std(episode_rewards)
        }
    
    return results

# 运行评估
results = evaluate_on_varied_environments(model, env)

print("\n=== 鲁棒性评估结�?===")
for env_name, stats in results.items():
    print(f"{env_name:12s}: {stats['mean']:7.2f} ± {stats['std']:.2f}")
```

---

## 5. 性能优化技�?

### 5.1 减少物理计算开销

```gdscript
# 对于静态或远离的机器人部件，降低更新频�?
func optimize_physics_update(body: RigidBody3D, distance_to_camera: float):
	if distance_to_camera > 20.0:
		# 远处物体降低物理更新频率
		body.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_ON
	else:
		body.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
```

### 5.2 批量环境训练

```python
# 使用 SubprocVecEnv 并行训练
from stable_baselines3.common.vec_env import SubprocVecEnv

def make_env(rank, seed=0):
    def _init():
        env = GodotRobotEnv(port=9999 + rank)  # 每个环境不同端口
        env.seed(seed + rank)
        return env
    return _init

# 创建 4 个并行环�?
num_envs = 4
env = SubprocVecEnv([make_env(i) for i in range(num_envs)])

model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=500000)  # 实际训练 4 倍步�?
```

### 5.3 数据记录优化

```python
# 只记录关键数据，避免全量记录
class SelectiveDataLogger:
    def __init__(self, log_interval=100):
        self.log_interval = log_interval
        self.step = 0
        self.data = []
    
    def log(self, obs, reward, done):
        self.step += 1
        if self.step % self.log_interval == 0:
            self.data.append({
                "step": self.step,
                "height": obs["torso_height"][0],
                "orientation": obs["imu_orient"].tolist(),
                "reward": reward
            })
    
    def save(self, filename):
        import json
        with open(filename, 'w') as f:
            json.dump(self.data, f)
```

---

## 📚 实用示例

### 示例 1: 比较不同电机配置

```python
# compare_motor_configs.py
configs = [
    {"name": "Low-cost", "motors": ["xl430"] * 4, "cost": 280},
    {"name": "Mid-range", "motors": ["xl430", "xl430", "mx106", "mx106"], "cost": 1060},
    {"name": "High-end", "motors": ["mx106"] * 4, "cost": 1840}
]

for config in configs:
    env = create_env_with_motors(config["motors"])
    model = PPO("MultiInputPolicy", env)
    model.learn(10000)
    
    success_rate = evaluate(model, env)
    print(f"{config['name']}: {success_rate:.1%} 成功�? ${config['cost']}")
```

### 示例 2: 自适应难度训练

```python
# curriculum_learning.py
class CurriculumEnv(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        self.success_rate = 0.0
        self.difficulty = 0  # 0=easy, 1=medium, 2=hard
    
    def update_difficulty(self, success_rate):
        if success_rate > 0.8 and self.difficulty < 2:
            self.difficulty += 1
            print(f"📈 Increased difficulty to level {self.difficulty}")
        elif success_rate < 0.3 and self.difficulty > 0:
            self.difficulty -= 1
            print(f"📉 Decreased difficulty to level {self.difficulty}")
        
        self._apply_difficulty()
    
    def _apply_difficulty(self):
        if self.difficulty == 0:
            # 简单：标准环境
            self.env.set_physics_params({"gravity": 9.81, "ground_material": "concrete"})
        elif self.difficulty == 1:
            # 中等：轻微随机化
            self.env.set_physics_params({
                "gravity": np.random.uniform(8, 11),
                "ground_material": np.random.choice(["concrete", "wood"])
            })
        else:
            # 困难：完全随机化
            self.env.set_physics_params({
                "gravity": np.random.uniform(5, 15),
                "ground_material": np.random.choice(["concrete", "ice", "sand"])
            })
```

---

## 🎯 下一�?

现在您已经掌握了进阶技巧，可以�?

1. **创建您的机器�?*
   - 设计独特的结�?
   - 选择合适的零件
   - 优化成本和性能

2. **扩展零件�?*
   - 添加您使用的真实硬件
   - 建立自己的零件数据库

3. **高级训练**
   - 域随机化提高鲁棒�?
   - 课程学习加速训�?
   - 多环境并行训�?

4. **Sim-to-Real 迁移**
   - 在仿真中训练
   - 在真实机器人上测�?
   - 迭代优化

---

**祝您开发顺利！** 🚀

---

**版本**: 1.0  
**最后更�?*: 2026-01-14
