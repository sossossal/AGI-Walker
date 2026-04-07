# 棣冩憥 鐎瑰本鏆?API 閸欏倽鈧啯鏋冨?

## 閻╊喖缍?

- [Python API](#python-api)
- [GDScript API](#gdscript-api)
- [闂嗘湹娆㈤弫鐗堝祦閺嶇厧绱(#闂嗘湹娆㈤弫鐗堝祦閺嶇厧绱?
- [閻滎垰顣ㄩ柊宥囩枂](#閻滎垰顣ㄩ柊宥囩枂)

---

## Python API

### PartsDatabase

闂嗘湹娆㈤弫鐗堝祦鎼存挻甯撮崣锝冣偓?

#### 閸掓繂顫愰崠?

```python
from godot_robot_env import PartsDatabase

db = PartsDatabase(parts_library_path=None)
```

**閸欏倹鏆?*:
- `parts_library_path` (str, optional): 闂嗘湹娆㈡惔鎾圭熅瀵板嫸绱濇妯款吇娑撴椽銆嶉惄顔昏厬閻?`parts_library` 閻╊喖缍?

#### 閺傝纭?

##### get_part(part_id: str) 閳?Dict

閼惧嘲褰囬梿鏈垫閺佺増宓侀妴?

**閸欏倹鏆?*:
- `part_id` (str): 闂嗘湹娆D

**鏉╂柨娲?*: 闂嗘湹娆㈤弫鐗堝祦鐎涙鍚€閿涘苯顩ч弸婊€绗夌€涙ê婀潻鏂挎礀 `None`

**缁€杞扮伐**:
```python
motor = db.get_part("dynamixel_xl430_w250")
print(motor['specifications']['stall_torque'])  # 1.4
```

##### get_parts_by_category(category: str) 閳?List[Dict]

閹稿琚崚顐ュ箯閸欐牠娴傛禒璺哄灙鐞涖劊鈧?

**閸欏倹鏆?*:
- `category` (str): 缁鍩嗛崥宥囆為敍鍫濐洤 `"actuator_servo"閿?

**鏉╂柨娲?*: 闂嗘湹娆㈤崚妤勩€?

**缁€杞扮伐**:
```python
servos = db.get_parts_by_category("actuator_servo")
for servo in servos:
    print(servo['model'])
```

##### list_all_parts() 閳?List[str]

閸掓鍤幍鈧張澶愭祩娴犵D閵?

**鏉╂柨娲?*: 闂嗘湹娆D閸掓銆?

##### validate_part(part_id: str) 閳?bool

妤犲矁鐦夐梿鏈垫閺佺増宓佺€瑰本鏆ｉ幀褋鈧?

**鏉╂柨娲?*: 閺勵垰鎯侀張澶嬫櫏

##### create_robot_config(parts_spec: List[Dict]) 閳?Dict

娴犲酣娴傛禒璺哄灙鐞涖劌鍨卞鐑樻簚閸ｃ劋姹夐柊宥囩枂閵?

**閸欏倹鏆?*:
- `parts_spec` (list): 闂嗘湹娆㈢憴鍕壐閸掓銆?

**缁€杞扮伐**:
```python
config = db.create_robot_config([
    {"part_id": "dynamixel_xl430_w250", "joint": "hip_left"},
    {"part_id": "dynixel_xl430_w250", "joint": "hip_right"}
])
```

---

### GodotRobotEnv

Gymnasium 閸忕厧顔愰惃鍕簚閸ｃ劋姹夋禒璺ㄦ埂閻滎垰顣ㄩ妴?

#### 閸掓繂顫愰崠?

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

**閸欏倹鏆?*:
- `robot_config` (dict, optional): 閺堝搫娅掓禍娲帳缂?
- `physics_config` (dict, optional): 閻椻晝鎮婇崣鍌涙殶闁板秶鐤?
- `host` (str): Godot 閺堝秴濮熼崳銊ユ勾閸р偓
- `port` (int): 缁旑垰褰涢崣?
- `timeout` (float): 鏉╃偞甯寸搾鍛閿涘牏顫楅敍?

#### 鐏炵偞鈧?

##### observation_space

鐟欏倸鐧傜粚娲？閿涘湙ymnasium Dict閿涘鈧?

**缂佹挻鐎?*:
```python
{
    'imu_orient': Box(3,),        # 婵寧鈧?(roll, pitch, yaw)
    'imu_angular_vel': Box(3,),   # 鐟欐帡鈧喎瀹?
    'imu_linear_acc': Box(3,),    # 缁炬寧鈧冨闁喎瀹?
    'joint_angles': Box(4,),      # 閸忓疇濡憴鎺戝
    'joint_velocities': Box(4,),  # 閸忓疇濡柅鐔峰
    'joint_torques': Box(4,),     # 閸忓疇濡幍顓犵叐
    'foot_contacts': MultiBinary(2,), # 閼存岸鍎撮幒銉ㄐ?
    'torso_height': Box(1,)       # 闊垰鍏辨妯哄
}
```

##### action_space

閸斻劋缍旂粚娲？閿涘湙ymnasium Box閿涘鈧?

**瑜般垻濮?*: (4,) - 4娑擃亜鍙ч懞鍌滄畱閻╊喗鐖ｇ憴鎺戝閿涘牆瀹抽敍? 
**閼煎啫娲?*: 鐟欎焦鏋冨锝堫嚛閺?

#### 閺傝纭?

##### reset() 閳?Tuple[Dict, Dict]

闁插秶鐤嗛悳顖氼暔閵?

**鏉╂柨娲?*: `(observation, info)`

**缁€杞扮伐**:
```python
obs, info = env.reset()
```

##### step(action) 閳?Tuple[Dict, float, bool, bool, Dict]

閹笛嗩攽娑撯偓濮濄儯鈧?

**閸欏倹鏆?*:
- `action` (np.ndarray): 閸斻劋缍旈敍?娑擃亜鍙ч懞鍌濐潡鎼达讣绱?

**鏉╂柨娲?*: `(observation, reward, terminated, truncated, info)`

**缁€杞扮伐**:
```python
action = env.action_space.sample()
obs, reward, terminated, truncated, info = env.step(action)
```

##### set_physics_params(params: Dict)

閸斻劍鈧椒鎱ㄩ弨鍦⒖閻炲棗寮弫鑸偓?

**閸欏倹鏆?*:
- `params` (dict): 閸欏倹鏆熺€涙鍚€

**缁€杞扮伐**:
```python
env.set_physics_params({
    "gravity": 3.71,  # 閻忣偅妲﹂柌宥呭
    "ground_material": "sand"
})
```

##### close()

閸忔娊妫撮悳顖氼暔閵?

---

## GDScript API

### EnvironmentController

閻滎垰顣ㄩ崣鍌涙殶閹貉冨煑閸ｃ劊鈧?

#### 閺傝纭?

##### load_preset(preset_name: String)

閸旂姾娴囬悳顖氼暔妫板嫯顔曢妴?

**閸欏倹鏆?*:
- `preset_name`: 妫板嫯顔曢崥宥囆為敍鍧?earth"`, `"moon"`, `"mars"`, `"jupiter"`閿?

**缁€杞扮伐**:
```gdscript
$EnvironmentController.load_preset("moon")
```

##### set_gravity(value: float)

鐠佸墽鐤嗛柌宥呭閵?

**閸欏倹鏆?*:
- `value`: 闁插秴濮忛崐纭风礄m/s铏忛敍?

##### set_air_density(value: float)

鐠佸墽鐤嗙粚鐑樼毜鐎靛棗瀹抽妴?

**閸欏倹鏆?*:
- `value`: 缁岀儤鐨电€靛棗瀹抽敍鍧榞/m椴侀敍?

##### set_temperature(value: float)

鐠佸墽鐤嗗〒鈺佸閵?

**閸欏倹鏆?*:
- `value`: 濞撯晛瀹抽敍鍩檲閿?

##### calculate_air_drag(velocity: Vector3, cross_section: float, drag_coef: float = 0.47) 閳?Vector3

鐠侊紕鐣荤粚鐑樼毜闂冭濮忛妴?

**鏉╂柨娲?*: 闂冭濮忛崥鎴﹀櫤閿涘湤閿?

##### get_environment_info() 閳?Dictionary

閼惧嘲褰囪ぐ鎾冲閻滎垰顣ㄦ穱鈩冧紖閵?

**鏉╂柨娲?*: 閸栧懎鎯堥幍鈧張澶婂棘閺佹壆娈戠€涙鍚€

---

### GroundMaterialLibrary

閸︿即娼伴弶鎰窛鎼存挶鈧?

#### 閺傝纭?

##### apply_material(ground: StaticBody3D, material_name: String)

鎼存梻鏁ら弶鎰窛閸掓澘婀撮棃顫偓?

**閸欏倹鏆?*:
- `ground`: 閸︿即娼伴悧鈺€缍?
- `material_name`: 閺夋劘宸濋崥宥囆?

**閸欘垳鏁ら弶鎰窛**:
- `"concrete"` - 濞ｅ嘲鍤岄崷?
- `"wood"` - 閺堛劍婢?
- `"carpet"` - 閸︾増顕?
- `"ice"` - 閸愪即娼?
- `"metal"` - 闁叉垵鐫?
- `"sand"` - 濞屾瑥婀?
- `"grass"` - 閼藉婀?
- `"mud"` - 濞夈儱婀?

**缁€杞扮伐**:
```gdscript
$GroundMaterialLibrary.apply_material($Ground, "ice")
```

##### get_material(material_name: String) 閳?GroundMaterial

閼惧嘲褰囬弶鎰窛鐎电钖勯妴?

##### list_materials() 閳?Array[String]

閸掓鍤幍鈧張澶嬫綏鐠愩劌鎮曠粔鑸偓?

---

## 闂嗘湹娆㈤弫鐗堝祦閺嶇厧绱?

### 閻㈠灚婧€/閼稿灚婧€

```json
{
  "part_id": "unique_id",
  "category": "actuator_servo",
  "manufacturer": "Manufacturer Name",
  "model": "Model Number",
  "specifications": {
    "stall_torque": 1.4,          // N璺痬
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
      "resistance": 10.0,         // 鎺矯/W
      "time_constant": 1500,      // s
      "max_winding_temp": 150     // 鎺矯
    }
  },
  "price_usd": 69.90
}
```

### IMU 娴肩姵鍔呴崳?

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
      "accel": 150,               // 娓璯/閳z
      "gyro": 0.014               // 鎺?s/閳z
    }
  },
  "price_usd": 34.95
}
```

---

## 閻滎垰顣ㄩ柊宥囩枂

### 閻椻晝鎮婇崣鍌涙殶

```python
physics_config = {
    "gravity": 9.81,              # m/s铏?
    "air_density": 1.225,         # kg/m椴?
    "temperature": 25.0,          # 鎺矯
    "ground_material": "concrete",
    "wind_velocity": {            # m/s
        "x": 0.0,
        "y": 0.0,
        "z": 0.0
    }
}
```

### 閻滎垰顣ㄦ０鍕啎

| 妫板嫯顔?| 闁插秴濮?| 缁岀儤鐨电€靛棗瀹?| 濞撯晛瀹?|
|------|------|----------|------|
| earth | 9.81 | 1.225 | 25 |
| moon | 1.62 | 0.0 | -20 |
| mars | 3.71 | 0.02 | -60 |
| jupiter | 24.79 | 0.16 | -110 |

---

## 鐢悂鍣洪崪灞剧亣娑?

### 閺夋劘宸濋幗鈺傛憹缁粯鏆?

| 閺夋劘宸?| 閹解晜鎽濈化缁樻殶 | 瀵鈧?|
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

## 闁挎瑨顕ゆ径鍕倞

### 鐢瓕顫嗗鍌氱埗

```python
# 鏉╃偞甯存径杈Е
RuntimeError: "Not connected to Godot simulator"

# 闂嗘湹娆㈤張顏呭閸?
鏉╂柨娲?None閿涘牅绗夐幎娑樼磽鐢潻绱?

# 閺冪姵鏅ラ崣鍌涙殶
閸欏倹鏆熺悮?clamp 閸掔増婀侀弫鍫ｅ瘱閸?
```

---

**閺傚洦銆傞悧鍫熸拱**: 1.0  
**閺堚偓閸氬孩娲块弬?*: 2026-01-15
