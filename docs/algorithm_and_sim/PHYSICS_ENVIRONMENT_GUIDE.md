# 閻椻晝鎮婇悳顖氼暔婢х偛宸辩化鑽ょ埠娴ｈ法鏁ら幐鍥у础

## 棣冩惖 濮掑倽鍫?

閻椻晝鎮婇悳顖氼暔婢х偛宸辩化鑽ょ埠閸忎浇顔忛幃銊ュЗ閹浇鐨熼懞鍌欒雹閻喓骞嗘晶鍐畱閻椻晝鎮婇崣鍌涙殶閿涘苯瀵橀幏顒勫櫢閸旀稏鈧胶鈹栧鏂跨槕鎼达负鈧焦淇惔锔衡偓浣告勾闂堛垺娼楃拹銊х搼閿涘瞼鏁ゆ禍搴窗
- 濞村鐦張鍝勬珤娴滃搫婀稉宥呮倱閻滎垰顣ㄦ稉瀣畱鐞涖劎骞?
- 閸╃喖娈㈤張鍝勫鐠侇厾绮岄敍鍫熷絹妤傛ɑ纭鹃崠鏍厴閸旀冻绱?
- 濡剝瀚欓弸浣侯伂閻滎垰顣ㄩ敍鍫熸箑閻炲啨鈧胶浼€閺勭喓鐡戦敍?

---

## 棣冨 閻滎垰顣ㄩ幒褍鍩楅崳?(EnvironmentController)

### 閸╃儤婀版担璺ㄦ暏

```gdscript
# 濞ｈ濮為崚鏉挎簚閺?
var env_controller = EnvironmentController.new()
add_child(env_controller)

# 鐠佸墽鐤嗛柌宥呭
env_controller.set_gravity(3.71)  # 閻忣偅妲﹂柌宥呭

# 鐠佸墽鐤嗙粚鐑樼毜鐎靛棗瀹?
env_controller.set_air_density(0.02)  # kg/m椴?

# 鐠佸墽鐤嗗〒鈺佸
env_controller.set_temperature(-60.0)  # 鎺矯

# 鐠佸墽鐤嗘搴″
env_controller.set_wind(Vector3(5, 0, 0))  # 5m/s 娑撴粓顥?
```

### 閻滎垰顣ㄦ０鍕啎

韫囶偊鈧喎鍨忛幑銏犲煂妫板嫬鐣炬稊澶屽箚婢у喛绱?

```gdscript
# 閸︽壆鎮嗛悳顖氼暔閿涘牓绮拋銈忕礆
env_controller.load_preset("earth")

# 閺堝牏鎮嗛悳顖氼暔閿涘牅缍嗛柌宥呭閿涘本妫ゆ径褎鐨甸敍?
env_controller.load_preset("moon")

# 閻忣偅妲﹂悳顖氼暔
env_controller.load_preset("mars")

# 閺堛劍妲﹂悳顖氼暔閿涘牓鐝柌宥呭閿?
env_controller.load_preset("jupiter")
```

**妫板嫯顔曢崣鍌涙殶鐎佃鐦?*:

| 閻滎垰顣?| 闁插秴濮?(m/s铏? | 缁岀儤鐨电€靛棗瀹?(kg/m椴? | 濞撯晛瀹?(鎺矯) |
|------|-------------|------------------|-----------|
| 閸︽壆鎮?| 9.81 | 1.225 | 25 |
| 閺堝牏鎮?| 1.62 | 0.0 | -20 |
| 閻忣偅妲?| 3.71 | 0.02 | -60 |
| 閺堛劍妲?| 24.79 | 0.16 | -110 |

### 缁岀儤鐨甸梼璇插鐠侊紕鐣?

```gdscript
# 閸︺劌鍨版担鎾舵畱 _physics_process 娑擃叏绱?
func _physics_process(delta):
    var velocity = linear_velocity
    var cross_section = 0.5  # m铏?(閺堝搫娅掓禍鐑樏幋顏堟桨缁?
    var drag_coef = 0.47  # 閻炲啩缍嬮梼璇插缁粯鏆?
    
    var air_drag = env_controller.calculate_air_drag(
        velocity,
        cross_section,
        drag_coef
    )
    
    apply_central_force(air_drag)
```

### 濞撯晛瀹宠ぐ鍗炴惙

```gdscript
# 閼惧嘲褰囧〒鈺佸瑜板崬鎼烽崶鐘茬摍閿涘牆濂栭崫宥嗘噰閹匡妇鐡戦敍?
var temp_factor = env_controller.get_temperature_factor()
var adjusted_friction = base_friction * temp_factor
```

### 闂呭繑婧€閹垫澘濮╅敍鍫濈厵闂呭繑婧€閸栨牭绱?

```gdscript
# 濮ｅ繘娈ф稉鈧▓鍨闂傚瓨鏌﹂崝鐘绘閺堢儤澹堥崝?
func apply_domain_randomization():
    env_controller.apply_random_disturbance(robot_body, 10.0)
```

---

## 棣冨綀閿?閸︿即娼伴弶鎰窛缁崵绮?

### 娴ｈ法鏁ら弶鎰窛鎼?

```gdscript
# 濞ｈ濮為弶鎰窛鎼?
var material_lib = GroundMaterialLibrary.new()
add_child(material_lib)

# 閼惧嘲褰囬崷浼存桨 StaticBody3D
var ground = $Ground

# 鎼存梻鏁ら弶鎰窛
material_lib.apply_material(ground, "concrete")  # 濞ｅ嘲鍤岄崷?
material_lib.apply_material(ground, "ice")       # 閸愪即娼?
material_lib.apply_material(ground, "sand")      # 濞屾瑥婀?
```

### 閸欘垳鏁ら弶鎰窛

| 閺夋劘宸?| 閹解晜鎽濈化缁樻殶 | 瀵鈧?| 濠婃艾濮╅幗鈺傛憹 | 閻楀湱鍋?|
|------|----------|------|----------|------|
| **concrete** | 0.9 | 0.1 | 0.005 | 绾剝宸濋敍宀勭彯閹解晜鎽?|
| **wood** | 0.6 | 0.2 | 0.01 | 娑擃厾鐡戠涵顒€瀹?|
| **carpet** | 1.0 | 0.05 | 0.03 | 妤傛ɑ鎳囬幙锔肩礉妤傛﹢妯嗙亸?|
| **ice** | 0.1 | 0.3 | 0.001 | 閺嬩椒缍嗛幗鈺傛憹 |
| **metal** | 0.4 | 0.4 | 0.005 | 娴ｅ孩鎳囬幙锔肩礉妤傛ê鑴婇幀?|
| **sand** | 0.7 | 0.0 | 0.05 | 閸欘垰褰夎ぐ?|
| **grass** | 0.75 | 0.1 | 0.02 | 閼奉亞鍔ч崷鏉胯埌 |
| **mud** | 0.85 | 0.0 | 0.08 | 閸欘垰褰夎ぐ顫礉妤傛﹢妯嗛崝?|

### 閼奉亜鐣炬稊澶嬫綏鐠?

```gdscript
# 閸掓稑缂撻懛顏勭暰娑斿娼楃拹?
var custom_mat = GroundMaterial.new("Rubber", 1.2, 0.8)
custom_mat.roughness = 0.9
custom_mat.color = Color(0.2, 0.2, 0.2)
custom_mat.rolling_friction = 0.02

# 濞ｈ濮為崚鏉跨氨
material_lib.add_custom_material(custom_mat)

# 娴ｈ法鏁?
material_lib.apply_material(ground, "Rubber")
```

---

## 棣冨箖 鐎圭偞妞傞幒褍鍩?

### 闁款喚娲忚箛顐ｅ祹闁款喚銇氭笟?

```gdscript
func _input(event):
    if event is InputEventKey and event.pressed:
        match event.keycode:
            # 閻滎垰顣ㄩ崚鍥ㄥ床
            KEY_1: env_controller.load_preset("earth")
            KEY_2: env_controller.load_preset("moon")
            KEY_3: env_controller.load_preset("mars")
            
            # 閺夋劘宸濋崚鍥ㄥ床
            KEY_C: material_lib.apply_material(ground, "concrete")
            KEY_I: material_lib.apply_material(ground, "ice")
            KEY_S: material_lib.apply_material(ground, "sand")
            
            # 闁插秴濮忕拫鍐Ν
            KEY_UP: env_controller.set_gravity(env_controller.gravity + 1.0)
            KEY_DOWN: env_controller.set_gravity(env_controller.gravity - 1.0)
```

---

## 棣冩晢 娑?Python API 闂嗗棙鍨?

### Python 缁旑垵鐨熼悽?

```python
from godot_robot_env import GodotRobotEnv

# 閸掓稑缂撻悳顖氼暔閺冭埖瀵氱€规氨澧块悶鍡楀棘閺?
env = GodotRobotEnv(
    physics_config={
        "gravity": 3.71,  # 閻忣偅妲﹂柌宥呭
        "air_density": 0.02,
        "temperature": -60.0,
        "ground_material": "sand"
    }
)

# 鏉╂劘顢戦弮璺哄З閹椒鎱ㄩ弨?
env.set_physics_params({
    "gravity": 9.81,
    "ground_material": "ice"
})
```

### 閸╃喖娈㈤張鍝勫鐠侇厾绮?

```python
import random

def domain_randomization_callback():
    """濮ｅ繋閲?episode 闂呭繑婧€閸栨牜骞嗘晶鍐ㄥ棘閺?""
    env.set_physics_params({
        "gravity": random.uniform(5.0, 15.0),
        "air_density": random.uniform(0.5, 2.0),
        "ground_material": random.choice([
            "concrete", "wood", "carpet", "ice", "sand"
        ])
    })

# 閸︺劏顔勭紒鍐ㄦ儕閻滎垯鑵?
for episode in range(1000):
    domain_randomization_callback()
    obs = env.reset()
    # ... 鐠侇厾绮?...
```

---

## 棣冩惓 閻╂垶甯堕悳顖氼暔閻樿埖鈧?

```gdscript
# 閼惧嘲褰囪ぐ鎾冲閻滎垰顣ㄦ穱鈩冧紖
var env_info = env_controller.get_environment_info()
print("闁插秴濮? ", env_info["gravity"])
print("濞撯晛瀹? ", env_info["temperature"])
print("妞嬪酣鈧? ", env_info["wind_velocity"])

# 鐎电厧鍤柊宥囩枂
var config = env_controller.to_dict()
# 娣囨繂鐡ㄩ崚鐗堟瀮娴犺埖鍨ㄩ崣鎴︹偓浣稿煂 Python

# 娴犲酣鍘ょ純顔煎鏉?
env_controller.from_dict(saved_config)
```

---

## 棣冃?濞村鐦粈杞扮伐閸︾儤娅?

閸掓稑缂撳ù瀣槸閸︾儤娅?`test_environment.tscn`:

```
閺嶇濡悙?(Node3D)
閳规壕鏀㈤埞鈧?EnvironmentController
閳规壕鏀㈤埞鈧?GroundMaterialLibrary
閳规壕鏀㈤埞鈧?Ground (StaticBody3D)
閳?  閳规柡鏀㈤埞鈧?CollisionShape3D (BoxShape3D)
閳规壕鏀㈤埞鈧?Robot (RigidBody3D)
閳规柡鏀㈤埞鈧?Test Script (test_environment.gd)
```

鏉╂劘顢戦崷鐑樻珯閸氬函绱?
- 閹?`1-3` 閸掑洦宕查悳顖氼暔妫板嫯顔?
- 閹?`C/I/S` 閸掑洦宕查崷浼存桨閺夋劘宸?
- 閹?`閳?閳彵 鐠嬪啳濡柌宥呭

---

## 棣冨箚 鎼存梻鏁ら崷鐑樻珯

### 1. 妞翠焦顥楅幀褎绁寸拠?
濞村鐦張鍝勬珤娴滃搫婀崥鍕潚閻滎垰顣ㄦ稉瀣畱缁嬪啿鐣鹃幀褝绱?
- 閺堝牏鎮嗘担搴ㄥ櫢閸旀稓骞嗘晶?
- 閸愪即娼版担搴㈡噰閹匡妇骞嗘晶?
- 瀵椽顥撻獮鍙夊閻滎垰顣?

### 2. 閸╃喖娈㈤張鍝勫鐠侇厾绮?
閹绘劙鐝?Sim-to-Real 鏉╀胶些閼宠棄濮忛敍?
- 闂呭繑婧€閸栨牠鍣搁崝?(鍗?0%)
- 闂呭繑婧€閸栨牗鎳囬幙锔鹃兇閺?(鍗?0%)
- 闂呭繑婧€閸栨牗澹堥崝銊ュ

### 3. 閻滎垰顣ㄩ柅鍌氱安閹呯埡缁?
閻梻鈹掗張鈧担宕囧箚婢у啫寮弫甯窗
- 娑撳秴鎮撻柌宥呭娑撳娈戝銉︹偓浣风喘閸?
- 娑撳秴鎮撻崷浼存桨閻ㄥ嫯鍏橀柌蹇旀櫏閻?

---

## 棣冩暋 妤傛楠囬崝鐔诲厴

### 鏉╃偞甯存穱鈥冲娇

```gdscript
func _ready():
    env_controller.environment_changed.connect(_on_env_changed)
    env_controller.preset_loaded.connect(_on_preset_loaded)

func _on_env_changed(param_name: String, new_value: float):
    print("閸欏倹鏆熼崣妯哄: ", param_name, " = ", new_value)

func _on_preset_loaded(preset_name: String):
    print("閸旂姾娴囨０鍕啎: ", preset_name)
```

### 閹嗗厴娴兼ê瀵?

```gdscript
# 閹靛綊鍣洪弴瀛樻煀閸欏倹鏆熼敍鍫濆櫤鐏忔垳淇婇崣鐤曢崣鎴礆
env_controller.from_dict({
    "gravity": 3.71,
    "air_density": 0.02,
    "temperature": -60.0
})
```

---

**閻楀牊婀?*: 1.0  
**閺堚偓閸氬孩娲块弬?*: 2026-01-14
