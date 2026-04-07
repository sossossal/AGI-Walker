# 棣冨笚 鏉╂盯妯佹担璺ㄦ暏閹稿洤宕?

閺堫剚瀵氶崡妤€鐨㈢敮顔煎И閹劍绻侀崗銉ゅ▏閻?Godot 閺堝搫娅掓禍鐑樐侀幏鐔奉殰娴犺绱濋崚娑樼紦閼奉亜绻侀惃鍕簚閸ｃ劋姹夐妴浣瑰潑閸旂姵鏌婇梿鏈垫閵嗕浇绻樼悰宀勭彯缁狙嗩唲缂佸啨鈧?

---

## 棣冩惖 閻╊喖缍?

1. [閸掓稑缂撻懛顏勭暰娑斿婧€閸ｃ劋姹塢(#1-閸掓稑缂撻懛顏勭暰娑斿婧€閸ｃ劋姹?
2. [濞ｈ濮為弬浼存祩娴犺泛鍩岄梿鏈垫鎼存彯(#2-濞ｈ濮為弬浼存祩娴犺泛鍩岄梿鏈垫鎼?
3. [妤傛楠囬悳顖氼暔闁板秶鐤哴(#3-妤傛楠囬悳顖氼暔闁板秶鐤?
4. [閸╃喖娈㈤張鍝勫鐠侇厾绮宂(#4-閸╃喖娈㈤張鍝勫鐠侇厾绮?
5. [閹嗗厴娴兼ê瀵查幎鈧顪?#5-閹嗗厴娴兼ê瀵查幎鈧?

---

## 1. 閸掓稑缂撻懛顏勭暰娑斿婧€閸ｃ劋姹?

### 1.1 鐠佹崘顓搁張鍝勬珤娴滆櫣绮ㄩ弸?

閸嬪洩顔曢幋鎴滄粦鐟曚礁鍨卞杞扮娑?6-DOF 閸欏矁鍐婚張鍝勬珤娴滅尨绱欏В蹇旀蒋閼?娑擃亜鍙ч懞鍌︾礆閵?

**濮濄儵顎?*:

#### A. 闁瀚ㄩ梿鏈垫

```python
from godot_robot_env import PartsDatabase

db = PartsDatabase()

# 娑撴椽鐝涢崗瀹犲Ν闁瀚ㄦ径褎澹勯惌鈺冩暩閺?
hip_motor = db.get_part("dynamixel_mx106")  # 8.4 N璺痬
print(f"妤傚鍙ч懞鍌滄暩閺? {hip_motor['model']}, {hip_motor['specifications']['stall_torque']} N璺痬")

# 娑撻缚鍟欓惄鏍ф嫲闊繂鍙ч懞鍌炩偓澶嬪娑擃厾鐡戦幍顓犵叐閻㈠灚婧€
knee_motor = db.get_part("dynamixel_xl430_w250")  # 1.4 N璺痬
print(f"閼舵繄娲?闊繂鍙ч懞鍌滄暩閺? {knee_motor['model']}, {knee_motor['specifications']['stall_torque']} N璺痬")

# 闁瀚?IMU
imu = db.get_part("bosch_bno055")
print(f"IMU: {imu['model']}")
```

#### B. 鐎规矮绠熼張鍝勬珤娴滄椽鍘ょ純?

閸掓稑缂撻弬鍥︽ `custom_robots/walker_6dof.json`:

```json
{
  "name": "Walker6DOF",
  "description": "6閼奉亞鏁辨惔锕€寮荤搾铏劄鐞涘本婧€閸ｃ劋姹?,
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

#### C. 閸?Python 娑擃厼濮炴潪浠嬪帳缂?

```python
import json

# 閸旂姾娴囬柊宥囩枂
with open("custom_robots/walker_6dof.json") as f:
    robot_config = json.load(f)

# 娴ｈ法鏁ら梿鏈垫鎼存挻澧跨仦鏇㈠帳缂?
db = PartsDatabase()
extended_config = db.create_robot_config(robot_config["parts"])

# 鐠侊紕鐣婚幀缁樺灇閺?
total_cost = sum(
    db.get_part(part["part_id"])["price_usd"]
    for part in robot_config["parts"]
    if db.get_part(part["part_id"])
)
print(f"閹粯鍨氶張? ${total_cost:.2f}")

# 鐠侊紕鐣婚幀鏄忓窛闁?
total_mass = sum(
    db.get_part(part["part_id"])["specifications"]["weight"]
    for part in robot_config["parts"]
    if db.get_part(part["part_id"]) and "weight" in db.get_part(part["part_id"])["specifications"]
)
print(f"闂嗘湹娆㈤幀鏄忓窛闁? {total_mass:.3f} kg")
```

### 1.2 閸?Godot 娑擃厽鐎鐑樻簚閸ｃ劋姹?

閸?Godot 娑擃厼鍨卞鍝勬簚閺?`custom_walker.tscn`:

```
Walker6DOF (Node3D)
閳规壕鏀㈤埞鈧?Torso (RigidBody3D)
閳?  閳规壕鏀㈤埞鈧?CollisionShape3D (BoxShape3D: 0.2x0.4x0.15)
閳?  閳规柡鏀㈤埞鈧?MeshInstance3D
閳?
閳规壕鏀㈤埞鈧?LeftThigh (RigidBody3D)
閳?  閳规壕鏀㈤埞鈧?CollisionShape3D (CapsuleShape3D)
閳?  閳规柡鏀㈤埞鈧?MeshInstance3D
閳?
閳规壕鏀㈤埞鈧?LeftCalf (RigidBody3D)
閳?  閳规壕鏀㈤埞鈧?CollisionShape3D (CapsuleShape3D)
閳?  閳规柡鏀㈤埞鈧?MeshInstance3D
閳?
閳规壕鏀㈤埞鈧?LeftFoot (RigidBody3D)
閳?  閳规壕鏀㈤埞鈧?CollisionShape3D (BoxShape3D)
閳?  閳规柡鏀㈤埞鈧?MeshInstance3D
閳?
閳规壕鏀㈤埞鈧?(閸欏疇鍚欑猾璁虫妧...)
閳?
閳规壕鏀㈤埞鈧?HipLeft (HingeJoint3D)
閳规壕鏀㈤埞鈧?KneeLeft (HingeJoint3D)
閳规壕鏀㈤埞鈧?AnkleLeft (HingeJoint3D)
閳规柡鏀㈤埞鈧?(閸欏厖鏅堕崗瀹犲Ν缁鎶€...)
```

#### 鎼存梻鏁ら梿鏈垫鐟欏嫭鐗搁懘姘拱

```gdscript
# walker_6dof.gd
extends Node3D

@onready var parts_lib = preload("res://addons/robot_sim_toolkit/scripts/parts_manager.gd").new()

func _ready():
	parts_lib.load_parts_database("res://parts_library")
	apply_part_specs()

func apply_part_specs():
	# 妤傚鍙ч懞?- 娴ｈ法鏁?MX-106
	var mx106 = parts_lib.get_part("dynamixel_mx106")
	if mx106:
		apply_motor_to_joint($HipLeft, mx106)
		apply_motor_to_joint($HipRight, mx106)
	
	# 閼舵繄娲婇崪宀冪閸忓疇濡?- 娴ｈ法鏁?XL430
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
	print("閴?Applied ", motor_data["model"], " to ", joint.name)
```

---

## 2. 濞ｈ濮為弬浼存祩娴犺泛鍩岄梿鏈垫鎼?

### 2.1 鐎电粯澹橀梿鏈垫鐟欏嫭鐗?

娴犮儲鍧婇崝?**Faulhaber 2657 CR** 閻㈠灚婧€娑撹桨绶ラ妴?

#### A. 閺€鍫曟肠閺佺増宓?

娴犲骸鍩楅柅鐘叉櫌閺佺増宓侀幍瀣斀閺€鍫曟肠閿?
- 閸絻娴嗛幍顓犵叐: 0.134 N璺痬 (at 24V)
- 缁岄缚娴囬柅鐔峰: 7200 RPM
- 闁插秹鍣? 126 g
- 鐏忓搫顕? 鑴?6 鑴?57 mm
- 閻㈤潧甯囬懠鍐ㄦ纯: 6-24 V
- 閺堚偓婢堆呮暩濞? 4.23 A
- 娴犻攱鐗? ~$180 USD

#### B. 閸掓稑缂?JSON 閺傚洣娆?

`parts_library/motors/faulhaber/2657cr.json`:

```json
{
  "part_id": "faulhaber_2657cr",
  "category": "actuator_motor",
  "manufacturer": "Faulhaber",
  "model": "2657 CR",
  "description": "缁儳鐦戦弮鐘插煕閻╁瓨绁﹂悽鍨簚閿涘苯鐢棁宥呯毜娴肩姵鍔呴崳?,
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
  "notes": "妤傛ɑ鈧嗗厴閺冪姴鍩涢悽鍨簚閿涘矂鈧倸鎮庣划鎯х槕鎼存梻鏁?
}
```

#### C. 妤犲矁鐦夐弫鐗堝祦

```python
# 妤犲矁鐦夐弬浼存祩娴?
from godot_robot_env import PartsDatabase

db = PartsDatabase()
db.print_statistics()

# 閼惧嘲褰囬弬浼存祩娴?
faulhaber = db.get_part("faulhaber_2657cr")
if faulhaber:
    print("\n閴?閺備即娴傛禒璺哄鏉炶姤鍨氶崝?")
    print(f"閸ㄥ褰? {faulhaber['model']}")
    print(f"閹殿厾鐓? {faulhaber['specifications']['stall_torque']} N璺痬")
    print(f"闁喎瀹? {faulhaber['specifications']['no_load_speed']} RPM")
else:
    print("閴?闂嗘湹娆㈤崝鐘烘祰婢惰精瑙?)
```

### 2.2 濞ｈ濮炴导鐘冲妳閸?

娴犮儲鍧婇崝?**Livox Mid-40 濠碘偓閸忓娴勬潏?*娑撹桨绶ラ妴?

#### A. 閸掓稑缂撴导鐘冲妳閸?Schema

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

#### B. 閸掓稑缂撻梿鏈垫閺佺増宓?

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

## 3. 妤傛楠囬悳顖氼暔闁板秶鐤?

### 3.1 閸掓稑缂撻懛顏勭暰娑斿骞嗘晶鍐暕鐠?

```gdscript
# custom_environments.gd
extends Node

const CUSTOM_PRESETS = {
	"underwater": {
		"gravity": 9.81 * 0.85,  # 濮樼繝鑵戝ù顔煎
		"air_density": 1000.0,  # 濮樻潙鐦戞惔?
		"temperature": 10.0,
		"name": "濮樼繝绗呴悳顖氼暔"
	},
	"high_altitude": {
		"gravity": 9.81,
		"air_density": 0.4,  # 妤傛ɑ鎹ｉ幏鏃傗枅閽栧嫮鈹栧?
		"temperature": -20.0,
		"name": "妤傛ɑ鎹ｉ幏?
	},
	"low_gravity_high_friction": {
		"gravity": 3.0,
		"air_density": 1.225,
		"temperature": 25.0,
		"ground_material": "carpet",
		"name": "娴ｅ酣鍣搁崝娑㈢彯閹解晜鎽?
	}
}

func apply_custom_preset(env_controller, preset_name: String):
	if CUSTOM_PRESETS.has(preset_name):
		var preset = CUSTOM_PRESETS[preset_name]
		env_controller.from_dict(preset)
		print("閴?Applied custom preset: ", preset["name"])
```

### 3.2 閸斻劍鈧礁婀磋ぐ銏㈡晸閹?

```gdscript
# terrain_generator.gd
extends Node3D

@export var terrain_size: Vector2 = Vector2(20, 20)
@export var tile_size: float = 1.0

func generate_varied_terrain(material_lib):
	# 閸掓稑缂撳Λ瀣磸閺嶇厧婀磋ぐ顫礉濮ｅ繑鐗告稉宥呮倱閺夋劘宸?
	var materials = ["concrete", "wood", "sand", "grass"]
	
	for x in range(int(terrain_size.x)):
		for z in range(int(terrain_size.y)):
			var tile = create_ground_tile(
				Vector3(x * tile_size, 0, z * tile_size),
				tile_size
			)
			
			# 闂呭繑婧€闁瀚ㄩ弶鎰窛
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

## 4. 閸╃喖娈㈤張鍝勫鐠侇厾绮?

### 4.1 Python 缁旑垰鐤勯悳?

閸掓稑缂撻弬鍥︽ `python_api/examples/domain_randomization_training.py`:

```python
import gymnasium as gym
import numpy as np
from stable_baselines3 import PPO
from godot_robot_env import GodotRobotEnv

class DomainRandomizationWrapper(gym.Wrapper):
    """閸╃喖娈㈤張鍝勫閸栧懓顥婇崳?""
    
    def __init__(self, env, randomize_params=None):
        super().__init__(env)
        self.randomize_params = randomize_params or {
            "gravity": (7.0, 12.0),
            "air_density": (0.5, 2.0),
            "temperature": (-20.0, 40.0),
            "ground_materials": ["concrete", "wood", "ice", "sand"]
        }
    
    def reset(self, **kwargs):
        # 濮ｅ繋閲?episode 瀵偓婵妞傞梾蹇旀簚閸栨牜骞嗘晶?
        self._randomize_environment()
        return self.env.reset(**kwargs)
    
    def _randomize_environment():
        """闂呭繑婧€閸栨牜骞嗘晶鍐ㄥ棘閺?""
        params = {}
        
        # 闂呭繑婧€闁插秴濮?
        if "gravity" in self.randomize_params:
            g_min, g_max = self.randomize_params["gravity"]
            params["gravity"] = np.random.uniform(g_min, g_max)
        
        # 闂呭繑婧€缁岀儤鐨电€靛棗瀹?
        if "air_density" in self.randomize_params:
            rho_min, rho_max = self.randomize_params["air_density"]
            params["air_density"] = np.random.uniform(rho_min, rho_max)
        
        # 闂呭繑婧€濞撯晛瀹?
        if "temperature" in self.randomize_params:
            t_min, t_max = self.randomize_params["temperature"]
            params["temperature"] = np.random.uniform(t_min, t_max)
        
        # 闂呭繑婧€閸︿即娼伴弶鎰窛
        if "ground_materials" in self.randomize_params:
            materials = self.randomize_params["ground_materials"]
            params["ground_material"] = np.random.choice(materials)
        
        # 鎼存梻鏁ら崚鎵箚婢?
        self.env.set_physics_params(params)
        
        print(f"棣冨箟 Randomized: g={params.get('gravity', 9.81):.2f}, "
              f"锜?{params.get('air_density', 1.225):.3f}, "
              f"mat={params.get('ground_material', 'concrete')}")

# 娴ｈ法鏁ょ粈杞扮伐
env = GodotRobotEnv()
env = DomainRandomizationWrapper(env)

model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
model.save("walker_domain_randomized")
```

### 4.2 缂佹挻鐏夌拠鍕強

```python
# evaluate_robustness.py
def evaluate_on_varied_environments(model, env, num_episodes=10):
    """閸︺劋绗夐崥宀€骞嗘晶鍐ц厬鐠囧嫪鍙婄粵鏍殣妞翠焦顥楅幀?""
    
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

# 鏉╂劘顢戠拠鍕強
results = evaluate_on_varied_environments(model, env)

print("\n=== 妞翠焦顥楅幀褑鐦庢导鎵波閺?===")
for env_name, stats in results.items():
    print(f"{env_name:12s}: {stats['mean']:7.2f} 鍗?{stats['std']:.2f}")
```

---

## 5. 閹嗗厴娴兼ê瀵查幎鈧?

### 5.1 閸戝繐鐨悧鈺冩倞鐠侊紕鐣诲鈧柨鈧?

```gdscript
# 鐎甸€涚艾闂堟瑦鈧焦鍨ㄦ潻婊咁瀲閻ㄥ嫭婧€閸ｃ劋姹夐柈銊ゆ閿涘矂妾锋担搴㈡纯閺備即顣堕悳?
func optimize_physics_update(body: RigidBody3D, distance_to_camera: float):
	if distance_to_camera > 20.0:
		# 鏉╂粌顦╅悧鈺€缍嬮梽宥勭秵閻椻晝鎮婇弴瀛樻煀妫版垹宸?
		body.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_ON
	else:
		body.physics_interpolation_mode = Node.PHYSICS_INTERPOLATION_MODE_OFF
```

### 5.2 閹靛綊鍣洪悳顖氼暔鐠侇厾绮?

```python
# 娴ｈ法鏁?SubprocVecEnv 楠炴儼顢戠拋顓犵矊
from stable_baselines3.common.vec_env import SubprocVecEnv

def make_env(rank, seed=0):
    def _init():
        env = GodotRobotEnv(port=9999 + rank)  # 濮ｅ繋閲滈悳顖氼暔娑撳秴鎮撶粩顖氬經
        env.seed(seed + rank)
        return env
    return _init

# 閸掓稑缂?4 娑擃亜鑻熺悰宀€骞嗘晶?
num_envs = 4
env = SubprocVecEnv([make_env(i) for i in range(num_envs)])

model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=500000)  # 鐎圭偤妾拋顓犵矊 4 閸婂秵顒為弫?
```

### 5.3 閺佺増宓佺拋鏉跨秿娴兼ê瀵?

```python
# 閸欘亣顔囪ぐ鏇炲彠闁款喗鏆熼幑顕嗙礉闁灝鍘ら崗銊╁櫤鐠佹澘缍?
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

## 棣冩憥 鐎圭偟鏁ょ粈杞扮伐

### 缁€杞扮伐 1: 濮ｆ棁绶濇稉宥呮倱閻㈠灚婧€闁板秶鐤?

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
    print(f"{config['name']}: {success_rate:.1%} 閹存劕濮涢悳? ${config['cost']}")
```

### 缁€杞扮伐 2: 閼奉亪鈧倸绨查梾鎯у鐠侇厾绮?

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
            print(f"棣冩惐 Increased difficulty to level {self.difficulty}")
        elif success_rate < 0.3 and self.difficulty > 0:
            self.difficulty -= 1
            print(f"棣冩惒 Decreased difficulty to level {self.difficulty}")
        
        self._apply_difficulty()
    
    def _apply_difficulty(self):
        if self.difficulty == 0:
            # 缁犫偓閸楁洩绱伴弽鍥у櫙閻滎垰顣?
            self.env.set_physics_params({"gravity": 9.81, "ground_material": "concrete"})
        elif self.difficulty == 1:
            # 娑擃厾鐡戦敍姘充氦瀵邦噣娈㈤張鍝勫
            self.env.set_physics_params({
                "gravity": np.random.uniform(8, 11),
                "ground_material": np.random.choice(["concrete", "wood"])
            })
        else:
            # 閸ヤ即姣﹂敍姘暚閸忋劑娈㈤張鍝勫
            self.env.set_physics_params({
                "gravity": np.random.uniform(5, 15),
                "ground_material": np.random.choice(["concrete", "ice", "sand"])
            })
```

---

## 棣冨箚 娑撳绔村?

閻滄澘婀幃銊ュ嚒缂佸繑甯夐幓鈥茬啊鏉╂盯妯侀幎鈧褝绱濋崣顖欎簰閿?

1. **閸掓稑缂撻幃銊ф畱閺堝搫娅掓禍?*
   - 鐠佹崘顓搁悪顒傚閻ㄥ嫮绮ㄩ弸?
   - 闁瀚ㄩ崥鍫モ偓鍌滄畱闂嗘湹娆?
   - 娴兼ê瀵查幋鎰拱閸滃本鈧嗗厴

2. **閹碘晛鐫嶉梿鏈垫鎼?*
   - 濞ｈ濮為幃銊ゅ▏閻劎娈戦惇鐔风杽绾兛娆?
   - 瀵よ櫣鐝涢懛顏勭箒閻ㄥ嫰娴傛禒鑸垫殶閹诡喖绨?

3. **妤傛楠囩拋顓犵矊**
   - 閸╃喖娈㈤張鍝勫閹绘劙鐝ご浣诡棗閹?
   - 鐠囧墽鈻肩€涳缚绡勯崝鐘烩偓鐔活唲缂?
   - 婢舵氨骞嗘晶鍐ㄨ嫙鐞涘矁顔勭紒?

4. **Sim-to-Real 鏉╀胶些**
   - 閸︺劋璞㈤惇鐔惰厬鐠侇厾绮?
   - 閸︺劎婀＄€圭偞婧€閸ｃ劋姹夋稉濠冪ゴ鐠?
   - 鏉╊厺鍞导妯哄

---

**缁佹繃鍋嶅鈧崣鎴︺€庨崚鈺嬬磼** 棣冩畬

---

**閻楀牊婀?*: 1.0  
**閺堚偓閸氬孩娲块弬?*: 2026-01-14
