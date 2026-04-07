# 閺堝搫娅掓禍娲祩娴犺泛绨辨担璺ㄦ暏閹稿洤宕?

## 棣冩憹 缁犫偓娴?

閺堝搫娅掓禍娲祩娴犺泛绨遍弰?AGI-Walker 妞ゅ湱娲伴惃鍕⒖鐏炴洖濮涢懗鏂ょ礉閹绘劒绶垫禍鍡楃唨娴滃海婀＄€圭偟鈥栨禒鎯邦潐閺嶈偐娈戦弫鏉跨摟闂嗘湹娆㈡惔鎾堕兇缂佺喆鈧倹鍋嶉崣顖欎簰娴ｈ法鏁ゆ潻娆庣昂闂嗘湹娆㈤弶銉︾€铏圭翱绾喚娈戦張鍝勬珤娴滆桨璞㈤惇鐔改侀崹瀣ㄢ偓?

## 棣冨箚 閸旂喕鍏橀悧瑙勨偓?

- 閴?**閻喎鐤勭憴鍕壐閺佺増宓?*閿涙艾鐔€娴滃骸鐤勯梽鍛返鎼存柨鏅㈤弫鐗堝祦閹靛鍞介惃鍕棘閺?
- 閴?**閺嶅洤鍣崠鏍ㄧ壐瀵?*閿涙SON Schema 妤犲矁鐦夐敍宀€鈥樻穱婵囨殶閹诡喕绔撮懛瀛樷偓?
- 閴?**閸楄櫕褰冮崡宕囨暏**閿涙岸鈧俺绻?API 韫囶偊鈧喎鍨卞娲祩娴犺泛鐤勬笟?
- 閴?**閻椻晝鎮婄划鍓р€?*閿涙艾瀵橀崥顐ュ窛闁插繈鈧焦鍎婚柌蹇嬧偓浣规噰閹匡负鈧胶鍎归悧瑙勨偓褏鐡戦崣鍌涙殶
- 閴?**閺勬挷绨幍鈺佺潔**閿涙碍瀵滈悡?Schema 濞ｈ濮為弬浼存祩娴?

## 棣冩惃 閻╊喖缍嶇紒鎾寸€?

```
parts_library/
閳规壕鏀㈤埞鈧?schema/                    # JSON Schema 鐎规矮绠?
閳?  閳规壕鏀㈤埞鈧?motor.schema.json      # 閻㈠灚婧€鐟欏嫭鐗?Schema
閳?  閳规柡鏀㈤埞鈧?sensor_imu.schema.json # IMU 娴肩姵鍔呴崳?Schema
閳?
閳规壕鏀㈤埞鈧?motors/                    # 閻㈠灚婧€闂嗘湹娆?
閳?  閳规柡鏀㈤埞鈧?dynamixel/
閳?      閳规壕鏀㈤埞鈧?xl430_w250.json    # Dynamixel XL430-W250
閳?      閳规柡鏀㈤埞鈧?mx106.json         # Dynamixel MX-106
閳?
閳规柡鏀㈤埞鈧?sensors/                   # 娴肩姵鍔呴崳銊╂祩娴?
    閳规柡鏀㈤埞鈧?imu/
        閳规柡鏀㈤埞鈧?bno055.json        # Bosch BNO055 IMU
```

## 棣冩畬 韫囶偊鈧喎绱戞慨?

### 1. 閸氼垳鏁ら幓鎺嶆

閸?Godot 缂傛牞绶崳銊よ厬閿?
1. 閹垫挸绱?`妞ゅ湱娲癭 -> `妞ゅ湱娲扮拋鍓х枂` -> `閹绘帊娆
2. 閹垫儳鍩?"Robot Simulation Toolkit"
3. 閸曢箖鈧鎯庨悽?

### 2. 閸︺劌婧€閺咁垯鑵戞担璺ㄦ暏闂嗘湹娆㈡惔?

```gdscript
# 閸︺劍鍋嶉惃鍕壖閺堫兛鑵?
extends Node3D

var parts_lib: RobotPartsLibrary

func _ready():
    # 閸掓稑缂撻梿鏈垫鎼存挾顓搁悶鍡楁珤
    parts_lib = RobotPartsLibrary.new()
    add_child(parts_lib)
    
    # 缁涘绶熼崝鐘烘祰鐎瑰本鍨?
    await get_tree().process_frame
    
    # 娴ｈ法鏁ら梿鏈垫鎼?
    create_robot_arm()

func create_robot_arm():
    # 閼惧嘲褰囬梿鏈垫娣団剝浼?
    var motor_data = parts_lib.get_part("dynamixel_xl430_w250")
    print("娴ｈ法鏁ら悽鍨簚: ", motor_data.get("model"))
    
    # 閸掓稑缂撻悽鍨簚鐎圭偘绶?
    var shoulder_motor = parts_lib.create_motor_instance(
        "dynamixel_xl430_w250",
        self
    )
    shoulder_motor.position = Vector3(0, 1, 0)
```

### 3. 閸掓稑缂撻悽鍨簚閸忓疇濡?

```gdscript
func create_joint_with_motor():
    # 閸嬪洩顔曞鍙夋箒娑撱倓閲滈崚姘秼閿涙pper_arm 閸?forearm
    var upper_arm = $UpperArm  # RigidBody3D
    var forearm = $Forearm     # RigidBody3D
    
    # 娴ｈ法鏁?Dynamixel MX-106 閸掓稑缂撻懖妯哄彠閼?
    var elbow_joint = parts_lib.create_motor_joint(
        "dynamixel_mx106",        # 闂嗘湹娆?ID
        upper_arm,                 # 閻栬泛鍨版担?
        forearm,                   # 鐎涙劕鍨版担?
        Vector3.RIGHT,             # 閺冨娴嗘潪?
        Vector3(0, -0.2, 0),      # 閻栬泛鍨版担鎾圭箾閹恒儳鍋?
        Vector3(0, 0.2, 0)        # 鐎涙劕鍨版担鎾圭箾閹恒儳鍋?
    )
    
    # 鐠佸墽鐤嗛崗瀹犲Ν闂勬劒缍?
    elbow_joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, deg_to_rad(-120))
    elbow_joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, deg_to_rad(0))
    
    # 閹貉冨煑閻㈠灚婧€
    elbow_joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, 2.0)
```

## 棣冩敵 API 閸欏倽鈧?

### RobotPartsLibrary 缁?

#### 閺嶇绺鹃弬瑙勭《

```gdscript
# 閼惧嘲褰囬梿鏈垫閺佺増宓?
func get_part(part_id: String) -> Dictionary

# 閹稿琚崚顐ょ摣闁?
func get_parts_by_category(category: String) -> Array[Dictionary]

# 閹稿鍩楅柅鐘叉櫌缁涙盯鈧?
func get_parts_by_manufacturer(manufacturer: String) -> Array[Dictionary]

# 閸掓稑缂撻悽鍨簚鐎圭偘绶?
func create_motor_instance(part_id: String, parent: Node3D = null) -> Node3D

# 閸掓稑缂撻悽鍨簚閸忓疇濡?
func create_motor_joint(
    part_id: String,
    body_a: RigidBody3D,
    body_b: RigidBody3D,
    axis: Vector3 = Vector3.RIGHT,
    local_pos_a: Vector3 = Vector3.ZERO,
    local_pos_b: Vector3 = Vector3.ZERO
) -> HingeJoint3D

# 妤犲矁鐦夐梿鏈垫閺佺増宓?
func validate_part(part_id: String) -> bool

# 閸掓鍤幍鈧張澶愭祩娴?
func list_all_parts() -> Array[String]

# 閹垫挸宓冪紒鐔活吀娣団剝浼?
func print_statistics() -> void
```

## 棣冩惓 瑜版挸澧犻梿鏈垫鎼?

### 閻㈠灚婧€/閼稿灚婧€

| Part ID | 閸ㄥ褰?| 閹殿厾鐓?| 闁喎瀹?| 娴犻攱鐗?|
|---------|------|------|------|------|
| `dynamixel_xl430_w250` | XL430-W250-T | 1.4 N璺痬 | 50 RPM | $69.90 |
| `dynamixel_mx106` | MX-106T | 8.4 N璺痬 | 45 RPM | $459.90 |

### 娴肩姵鍔呴崳?

| Part ID | 閸ㄥ褰?| 缁鐎?| 閺囧瓨鏌婇悳?| 娴犻攱鐗?|
|---------|------|------|--------|------|
| `bosch_bno055` | BNO055 | 9鏉炵MU | 100 Hz | $34.95 |

## 閴?濞ｈ濮為弬浼存祩娴?

### 濮濄儵顎?1: 閸掓稑缂?JSON 閺傚洣娆?

閸︺劎娴夋惔鏃傝閸掝偆娲拌ぐ鏇氱瑓閸掓稑缂撻弬鎵畱 JSON 閺傚洣娆㈤敍灞肩伐婵?`parts_library/motors/maxon/ec45.json`:

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

### 濮濄儵顎?2: 妤犲矁鐦夐弫鐗堝祦

娴ｈ法鏁ら崷銊у殠 JSON Schema 妤犲矁鐦夐崳銊﹀灗鏉╂劘顢戦敍?

```gdscript
var is_valid = parts_lib.validate_part("maxon_ec45_flat")
if is_valid:
    print("閴?闂嗘湹娆㈤弫鐗堝祦閺堝鏅?)
```

### 濮濄儵顎?3: 闁插秵鏌婇崝鐘烘祰

闁插秵鏌婇崥顖氬З Godot 妞ゅ湱娲伴幋鏍ㄥ閸斻劏鐨熼悽顭掔窗

```gdscript
parts_lib.load_parts_database()
```

## 棣冩暋 妤傛楠囬悽銊︾《

### 鐠佸潡妫堕梿鏈垫閸忓啯鏆熼幑?

閻㈠灚婧€鐎圭偘绶ラ崪灞藉彠閼哄倿鍏橀崠鍛儓閸樼喎顫愰梿鏈垫閺佺増宓侀敍?

```gdscript
var motor = parts_lib.create_motor_instance("dynamixel_xl430_w250", self)

# 閼惧嘲褰囬梿鏈垫閺佺増宓?
var part_data = motor.get_meta("part_data")
print("閸掑爼鈧姴鏅? ", part_data.get("manufacturer"))
print("閺佺増宓侀幍瀣斀: ", part_data.get("datasheet_url"))

# 閼惧嘲褰囬悧鐟扮暰閸欏倹鏆?
var stall_torque = motor.get_meta("stall_torque")
var friction = motor.get_meta("friction_params")
print("闂堟瑦鎳囬幙? ", friction.get("static"), " N璺痬")
```

### 鏉╂劘顢戦弮鎯扮殶閺佹潙寮弫?

```gdscript
# 娣囶喗鏁奸崗瀹犲Ν閹殿厾鐓╅梽鎰煑
var joint = parts_lib.create_motor_joint(...)
joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, 2.0)  # 閼奉亜鐣炬稊澶嬪閻?

# 濡剝瀚欓悽鍨簚鏉╁洩娴囬梽宥夘杺
var normal_torque = joint.get_meta("part_data")["specifications"]["stall_torque"]
var derated_torque = normal_torque * 0.8  # 闂勫秹顤?20%
joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, derated_torque)
```

## 棣冃?濞村鐦粈杞扮伐

鏉╂劘顢戝ù瀣槸閸︾儤娅欓敍?

1. 閸?Godot 缂傛牞绶崳銊よ厬閸掓稑缂撻弬鏉挎簚閺?
2. 濞ｈ濮?`Node3D` 閺嶇濡悙?
3. 闂勫嫬濮為懘姘拱 `res://scripts/test_parts_library.gd`
4. 鏉╂劘顢戦崷鐑樻珯 (F5)

妫板嫭婀℃潏鎾冲毉閿?
```
棣冩暋 瀵偓婵濮炴潪浠嬫祩娴犺泛绨?..
  閴?閸旂姾娴囬梿鏈垫: dynamixel_xl430_w250 (XL430-W250-T)
  閴?閸旂姾娴囬梿鏈垫: dynamixel_mx106 (MX-106T)
  閴?閸旂姾娴囬梿鏈垫: bosch_bno055 (BNO055)
閴?闂嗘湹娆㈡惔鎾冲鏉炶棄鐣幋鎰剁礉閸?3 娑擃亪娴傛禒?

=== 闂嗘湹娆㈡惔鎾剁埠鐠?===
閹娴傛禒鑸垫殶: 3
閸掑棛琚紒鐔活吀:
  - actuator_servo: 2
  - sensor_imu: 1
==================
```

## 棣冩偘 閺佸懘娈伴幒鎺楁珟

### 闂傤噣顣?1: 闂嗘湹娆㈤崝鐘烘祰婢惰精瑙?

**閻ュ洨濮?*: 閹貉冨煑閸欑増妯夌粈?"閻╊喖缍嶆稉宥呯摠閸? 鐠€锕€鎲?

**鐟欙絽鍠?*:
- 濡偓閺屻儲鏋冩禒鎯扮熅瀵板嫭妲搁崥锔筋劀绾?
- 绾喛顓?JSON 閺傚洣娆㈤崷銊︻劀绾喚娈戦惄顔肩秿娑?
- 妤犲矁鐦?`PARTS_ROOT` 鐢悂鍣洪幐鍥ф倻 `res://parts_library/`

### 闂傤噣顣?2: JSON 鐟欙絾鐎介柨娆掝嚖

**閻ュ洨濮?*: "JSON 鐟欙絾鐎芥径杈Е" 闁挎瑨顕?

**鐟欙絽鍠?*:
- 娴ｈ法鏁?JSON 妤犲矁鐦夊銉ュ徔濡偓閺屻儴顕㈠▔?
- 绾喕绻氬▽鈩冩箒婢舵矮缍戦惃鍕偓妤€褰?
- 濡偓閺屻儱绱╅崣閿嬫Ц閸氾附顒滅涵顕€妫撮崥?

### 闂傤噣顣?3: 闂嗘湹娆㈢€圭偘绶ュ▽鈩冩箒閻椻晝鎮婇弫鍫熺亯

**閻ュ洨濮?*: 閸掓稑缂撻惃鍕暩閺堣桨绗夋导姘竴閽€鑺ュ灗绾扮増鎸?

**鐟欙絽鍠?*:
- 绾喛顓婚崷鐑樻珯娑擃厽婀?`StaticBody3D` 閸︿即娼?
- 濡偓閺屻儳顫幘鐐茬湴鐠佸墽鐤?
- 妤犲矁鐦?RigidBody3D 濞屸剝婀佺悮顐ヮ啎缂冾喕璐?`freeze`

## 棣冩憥 閸欏倽鈧啳绁┃?

- [Dynamixel 鐎规ɑ鏌熼弬鍥ㄣ€俔(https://emanual.robotis.com/)
- [Godot 閻椻晝鎮婂鏇熸惛閺傚洦銆俔(https://docs.godotengine.org/en/stable/tutorials/physics/index.html)
- [JSON Schema 鐟欏嫯瀵朷(https://json-schema.org/)

## 棣冩暛 閺堫亝娼电拋鈥冲灊

- [ ] 濞ｈ濮為弴鏉戭樋閸濅胶澧濋惃鍕暩閺堢尨绱橣aulhaber閵嗕府axon閵嗕阜oboMaster閿?
- [ ] 閺€顖涘瘮閸?閹殿厾鐓╂导鐘冲妳閸?
- [ ] 閺€顖涘瘮 LiDAR 閸滃瞼娴夐張杞扮炊閹扮喎娅?
- [ ] 3D 濡€崇€锋惔鎿勭礄GLB 閺嶇厧绱￠敍?
- [ ] 閸︺劎鍤庨梿鏈垫閺佺増宓佹惔?
- [ ] 閸欘垵顫嬮崠鏍祩娴犲爼鈧瀚ㄩ崳?UI

---

**閻楀牊婀?*: 0.1.0  
**閺堚偓閸氬孩娲块弬?*: 2026-01-13
