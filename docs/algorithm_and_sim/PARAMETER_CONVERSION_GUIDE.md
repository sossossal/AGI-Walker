# 閻喎鐤勯崣鍌涙殶閸掓澘绱╅幙搴″棘閺佹壆娈戞潪顒佸床閹稿洤宕?

## 棣冩惢 濮掑倽鍫?

鐏忓棛骞囩€圭偘绗橀悾宀€娈戦張鍝勬珤娴滄椽娴傛禒璺哄棘閺佹媽娴嗛幑顫礋濞撳憡鍨欏鏇熸惛閻椻晝鎮婇崣鍌涙殶閺勵垯绔存稉?*閻椻晝鎮婂鐑樐?*閻ㄥ嫯绻冪粙瀣ㄢ偓鍌涚壋韫囧啯瀵幋妯烘躬娴滃函绱?

1. **閸欏倹鏆熼弰鐘茬殸**閿涙碍鏆熼幑顔藉閸愬瞼娈戦崣鍌涙殶 閳?瀵洘鎼搁惄瀛樺复娴ｈ法鏁ら惃鍕棘閺?
2. **閸楁洑缍呴幑銏㈢暬**閿涙艾鍩楅柅鐘叉櫌娴ｈ法鏁ゆ稉宥呮倱閻ㄥ嫬宕熸担宥囬兇缂?
3. **濡€崇€风粻鈧崠?*閿涙氨婀＄€圭偟澧块悶鍡欏箛鐠烇繝娓剁憰浣烘暏瀵洘鎼搁弨顖涘瘮閻ㄥ嫯绻庢导鍏寄侀崹瀣€冪粈?
4. **閹嗗厴楠炲疇銆€**閿涙氨绨跨涵顔煎 vs 鐠侊紕鐣婚弫鍫㈠芳

---

## 棣冩暋 閸忚渹缍嬫潪顒佸床鏉╁洨鈻?

### 缁€杞扮伐閿涙ynamixel XL430-W250 閻㈠灚婧€

#### 濮濄儵顎?1: 娴犲孩鏆熼幑顔藉閸愬本褰侀崣鏍у斧婵寮弫?

**閺夈儲绨?*: [ROBOTIS 鐎规ɑ鏌熼幍瀣斀](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)

| 閺佺増宓侀幍瀣斀閸欏倹鏆?| 閸?| 閸楁洑缍?|
|--------------|-----|------|
| Stall Torque (12V) | 1.4 | N璺痬 |
| No Load Speed (12V) | 50 | RPM |
| Weight | 57.2 | g |
| Dimensions (WxHxD) | 28.5 鑴?34.0 鑴?46.5 | mm |
| Voltage Range | 6.5 ~ 12.0 | V |
| Stall Current | 1.4 | A |
| Standby Current | 40 | mA |
| Protocol Type | TTL Half Duplex | - |
| Resolution | 4096 | pulses/rev |
| Gear Reduction Ratio | 257.4 | - |
| Operating Temp | -5 ~ +72 | 鎺矯 |

---

#### 濮濄儵顎?2: 閸楁洑缍呮潪顒佸床閸?SI 閺嶅洤鍣崡鏇氱秴

```python
# 鏉烆剚宕查崙鑺ユ殶
def convert_to_si(raw_values):
    return {
        'mass': 57.2 / 1000,                    # g 閳?kg = 0.0572 kg
        'dimensions': [
            28.5 / 1000,                        # mm 閳?m = 0.0285 m
            46.5 / 1000,                        # mm 閳?m = 0.0465 m
            34.0 / 1000                         # mm 閳?m = 0.034 m
        ],
        'stall_torque': 1.4,                    # N璺痬 (瀹歌尙绮￠弰?SI)
        'no_load_speed': 50 * (2*pi/60),        # RPM 閳?rad/s = 5.236 rad/s
        'voltage_range': [6.5, 12.0],           # V (瀹歌尙绮￠弰?SI)
        'max_current': 1.4                      # A (瀹歌尙绮￠弰?SI)
    }
```

**缂佹挻鐏?*:
```json
{
  "mass": 0.0572,              // kg
  "dimensions": [0.0285, 0.0465, 0.034],  // m
  "stall_torque": 1.4,         // N璺痬
  "no_load_speed": 5.236,      // rad/s
  "max_current": 1.4           // A
}
```

---

#### 濮濄儵顎?3: 閺勭姴鐨犻崚?Godot 閻椻晝鎮婇崣鍌涙殶

##### 3.1 閸掓矮缍嬬拹銊╁櫤閿涘湩igidBody3D.mass閿?

**閻╁瓨甯撮弰鐘茬殸**:
```gdscript
motor_body.mass = 0.0572  # kg
```

閴?**缁犫偓閸?*閿涙俺宸濋柌蹇曟纯閹恒儰濞囬悽顭掔礉閺冪娀娓舵潪顒佸床

---

##### 3.2 绾扮増鎸掕ぐ銏㈠Ц鐏忓搫顕?

**鏉烆剚宕查柅鏄忕帆**:
```gdscript
var dimensions = [0.0285, 0.0465, 0.034]  # [W, D, H] 閸楁洑缍呴敍姘辫儗

# Godot 閻?BoxShape3D.size 閺勵垰鍙忕亸鍝勵嚟閿涘牅绗夐弰顖氬磹鐏忓搫顕敍?
# 閸ф劖鐖ｇ化浼欑窗X=鐎? Y=妤? Z=濞?
var collision_shape = BoxShape3D.new()
collision_shape.size = Vector3(
    dimensions[0],  # 鐎硅棄瀹?閳?X
    dimensions[2],  # 妤傛ê瀹?閳?Y閿涘湙odot Y鏉炴潙鎮滄稉濠忕礆
    dimensions[1]   # 濞ｅ崬瀹?閳?Z
)
```

閳跨媴绗?**濞夈劍鍓?*閿?
- Godot 娴ｈ法鏁?Y 鏉炴潙鎮滄稉濠勬畱閸ф劖鐖ｇ化?
- 閺佺増宓侀幍瀣斀闁艾鐖堕弰?Z 鏉炴潙鎮滄稉?
- 闂団偓鐟曚線鍣搁弬鐗堟Ё鐏忓嫬娼楅弽鍥叡

---

##### 3.3 閻㈠灚婧€閹殿厾鐓?閳?閸忓疇濡崣鍌涙殶

鏉╂瑦妲?*閺堚偓婢跺秵娼?*閻ㄥ嫯娴嗛幑顫礉閸ョ姳璐?Godot 閻?`HingeJoint3D` 娑撳秶娲块幒銉δ侀幏鐔烘暩閺堣櫣澹掗幀褋鈧?

**闂傤噣顣?*閿涙碍鏆熼幑顔藉閸愬瞼绮伴惃鍕Ц"閸絻娴嗛幍顓犵叐"閸?缁岄缚娴囬柅鐔峰"閿涘奔绲?Godot 閻ㄥ嫬鍙ч懞鍌氬棘閺佺増妲搁敍?
- `PARAM_MOTOR_TARGET_VELOCITY`閿涘牏娲伴弽鍥潡闁喎瀹抽敍瀹篴d/s閿?
- `PARAM_MOTOR_MAX_IMPULSE`閿涘牊娓舵径褍鍟块柌蹇ョ礉N璺痬璺痵閿?

**鏉烆剚宕查弬瑙勭《 1閿涙氨鐣濋崠鏍侀崹瀣剁礄瑜版挸澧犵€圭偟骞囬敍?*

```gdscript
# 閻╁瓨甯存担璺ㄦ暏閸絻娴嗛幍顓犵叐娴ｆ粈璐熼張鈧径褍鍟块柌蹇ョ礄濮ｅ繐鎶氶敍?
joint.set_param(
    HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, 
    stall_torque  # 1.4 N璺痬
)

# 鐠佸墽鐤嗛惄顔界垼闁喎瀹抽敍鍧產d/s閿?
var target_speed = no_load_speed  # 5.236 rad/s
joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, target_speed)
```

閳跨媴绗?**鐏炩偓闂勬劖鈧?*閿?
- 鏉╂瑤閲滃Ο鈥崇€烽崑鍥啎閻㈠灚婧€閹粯妲告潏鎾冲毉閹帒鐣鹃幍顓犵叐
- 韫囩晫鏆愭禍鍡涒偓鐔峰-閹殿厾鐓╅弴鑼殠閿涘牏鏁搁張楦挎祮闁喕绉鸿箛顐礉閹殿厾鐓╃搾濠傜毈閿?

---

**鏉烆剚宕查弬瑙勭《 2閿涙岸鈧喎瀹?閹殿厾鐓╅弴鑼殠濡€崇€烽敍鍦橠Extension 娑擃厼鐤勯悳甯礆**

閻喎鐤勯惃鍕纯濞翠胶鏁搁張娲紥瀵邦亞鍤庨幀褔鈧喎瀹?閹殿厾鐓╅悧瑙勨偓褝绱?

```
T(锠? = T_stall 鑴?(1 - 锠?/ 锠卂no_load)

閸忔湹鑵戦敍?
- T_stall = 閸絻娴嗛幍顓犵叐閿?.4 N璺痬閿?
- 锠卂no_load = 缁岄缚娴囬柅鐔峰閿?.236 rad/s閿?
- 锠?= 瑜版挸澧犵憴鎺椻偓鐔峰
```

**GDScript 鐎圭偟骞?*閿涘牆鐤勯弮鎯邦吀缁犳绱?
```gdscript
func calculate_motor_torque(current_velocity: float) -> float:
    var stall_torque = 1.4      # N璺痬
    var no_load_speed = 5.236   # rad/s
    
    # 闁喎瀹?閹殿厾鐓╅弴鑼殠
    var speed_factor = 1.0 - (abs(current_velocity) / no_load_speed)
    speed_factor = clamp(speed_factor, 0.0, 1.0)
    
    var available_torque = stall_torque * speed_factor
    return available_torque

func _physics_process(delta):
    # 閼惧嘲褰囬崗瀹犲Ν瑜版挸澧犵憴鎺椻偓鐔峰
    var current_velocity = joint.get_param(HingeJoint3D.PARAM_ANGULAR_VELOCITY)
    
    # 鐠侊紕鐣荤€圭偤妾幍顓犵叐
    var actual_torque = calculate_motor_torque(current_velocity)
    
    # 鎼存梻鏁ら崚鏉垮彠閼?
    joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, actual_torque * delta)
```

**閺佸牊鐏夌€佃鐦?*:

| 闁喎瀹?| 缁犫偓閸栨牗膩閸ㄥ澹勯惌?| 閻喎鐤勫Ο鈥崇€烽幍顓犵叐 |
|------|--------------|--------------|
| 0 rad/s (閸絻娴? | 1.4 N璺痬 | 1.4 N璺痬 閴?|
| 2.6 rad/s (50%) | 1.4 N璺痬 閴?| 0.7 N璺痬 閴?|
| 5.2 rad/s (缁岄缚娴? | 1.4 N璺痬 閴?| 0 N璺痬 閴?|

---

##### 3.4 鏉烆剙鐡欓幆顖炲櫤

**闂傤噣顣?*閿涙碍鏆熼幑顔藉閸愬矂鈧艾鐖舵稉宥囨纯閹恒儲褰佹笟娑滄祮鐎涙劖鍎婚柌蹇ョ礉闂団偓鐟曚浇顓哥粻妤佸灗娴兼壆鐣婚妴?

**娴兼壆鐣婚崗顒€绱?*閿涘牆娓鹃弻鍙樼秼鏉╂垳鎶€閿?
```python
# 閸嬪洩顔曟潪顒€鐡欓弰顖氱杽韫囧啫娓鹃弻鍙樼秼
def estimate_rotor_inertia(diameter_mm, length_mm, mass_kg):
    """
    I = (1/2) 鑴?m 鑴?r铏?
    """
    radius = (diameter_mm / 1000) / 2  # 鏉烆剚宕叉稉铏硅儗楠炶泛褰囬崡濠傜窞
    inertia = 0.5 * mass_kg * (radius ** 2)
    return inertia

# XL430 娴兼壆鐣婚敍鍫濅海鐠佹崘娴嗙€涙劗娲垮?20mm閿涘矁宸濋柌蹇撳窗濮?30%閿?
rotor_mass = 0.0572 * 0.3  # 缁?17g
rotor_inertia = estimate_rotor_inertia(20, 30, rotor_mass)
# 缂佹挻鐏夐敍姘卞 3.5e-6 kg璺痬铏?
```

**鎼存梻鏁ら崚?Godot**:
```gdscript
# 閺傝纭?閿涙俺顔曠純顔煎灠娴ｆ挾娈戦幆顖涒偓褍绱堕柌蹇ョ礄婵″倹鐏夐梿鏈垫閺堫剝闊╁鐑樐佹稉鍝勫灠娴ｆ搫绱?
motor_body.inertia = Vector3(
    rotor_inertia,
    rotor_inertia,
    rotor_inertia * 2  # 濞屾寧妫嗘潪顒冮叡閺傜懓鎮滈幆顖炲櫤閺囨潙銇?
)

# 閺傝纭?閿涙艾婀?GDExtension 娑擃厾娈戦悽鍨簚濡剝瀚欓崳銊よ厬娴ｈ法鏁?
# 瑜板崬鎼烽崝鐘烩偓鐔峰閿涙?= T / I閿涘牐顫楅崝鐘烩偓鐔峰 = 閹殿厾鐓?/ 閹垶鍣洪敍?
```

---

##### 3.5 閹解晜鎽濋崝娑樺棘閺?

**闂傤噣顣?*閿涙碍鏆熼幑顔藉閸愬苯鍤戞稊?*娴犲簼绗?*閻╁瓨甯寸紒娆忓毉閹解晜鎽濋崝娑樺棘閺佽埇鈧?

**娴兼壆鐣婚弬瑙勭《**閿?

**A. 閸╄桨绨弫鍫㈠芳閸欏秵甯?*

```python
# 閺佺増宓侀幍瀣斀娑擃厾娈戦弫鍫㈠芳閿涘牆顩ч弸婊勬箒閿?
efficiency = 0.70  # 70% 閺佸牏宸?

# 閹圭喎銇戦崝鐔哄芳娑撴槒顩﹂弶銉ㄥ殰閹解晜鎽?
def estimate_friction(efficiency, stall_torque):
    # 缁犫偓閸栨牕浜ｇ拋鎾呯窗閹圭喎銇戦崝鐔哄芳 = 閹解晜鎽濋崝娑氱叐 鑴?楠炲啿娼庨柅鐔峰
    # 闂堟瑦鎳囬幙锔惧娑撳搫鐗潪顒佸閻晝娈?1-3%
    static_friction = stall_torque * (1 - efficiency) * 0.5
    dynamic_friction = static_friction * 0.6  # 閸斻劍鎳囬幙锔惧娑撴椽娼ら幗鈺傛憹閻?60%
    
    return {
        'static': static_friction,
        'dynamic': dynamic_friction
    }

friction = estimate_friction(0.70, 1.4)
# 缂佹挻鐏夐敍?
# {
#   'static': 0.021 N璺痬,
#   'dynamic': 0.0126 N璺痬
# }
```

**B. 缂佸繘鐛欓崐?*

閺嶈宓侀悽鍨簚缁鐎锋担璺ㄦ暏閸忕鐎烽崐纭风窗

| 閻㈠灚婧€缁鐎?| 闂堟瑦鎳囬幙?閸絻娴嗛幍顓犵叐 | 閸斻劍鎳囬幙?闂堟瑦鎳囬幙?|
|----------|-----------------|---------------|
| 妤傛绨挎惔锕佸煐閺堢尨绱欓柌鎴濈潣姒昏儻鐤嗛敍?| 1-2% | 50-60% |
| 閺咁噣鈧俺鍩栭張鐚寸礄婵夋垶鏋℃鑳枂閿?| 3-5% | 60-70% |
| 閺冪姴鍩涢悽鍨簚閿涘牏娲挎す鎲嬬礆 | 0.5-1% | 40-50% |

**Godot 鎼存梻鏁?*:
```gdscript
# 閺傝纭?閿涙矮濞囬悽?Godot 閸愬懐鐤嗛惃鍕▎鐏忕厧寮弫甯礄缁鏆愰敍?
joint.set_param(HingeJoint3D.PARAM_ANGULAR_DAMPING, 0.5)

# 閺傝纭?閿涙艾婀?GDExtension 娑擃厼鐤勯悳鎵翱绾喚娈戦幗鈺傛憹濡€崇€?
func apply_friction(velocity: float, applied_torque: float) -> float:
    var friction_torque = 0.0
    
    # 鎼存挷绮﹂幗鈺傛憹濡€崇€?
    if abs(velocity) < 0.01:  # 鏉╂垳鎶€闂堟瑦顒?
        # 闂堟瑦鎳囬幙锔肩窗閹跺灚濮夋潻鎰З閿涘奔绲炬稉宥堢Т鏉╁洭娼ら幗鈺傛憹閸?
        friction_torque = clamp(
            -applied_torque,
            -STATIC_FRICTION,
            STATIC_FRICTION
        )
    else:
        # 閸斻劍鎳囬幙锔肩窗娑撳氦绻嶉崝銊︽煙閸氭垹娴夐崣?
        friction_torque = -sign(velocity) * DYNAMIC_FRICTION
        # 缁ɑ鈧勬噰閹匡讣绱欐稉搴ㄢ偓鐔峰閹存劖顒滃В鏃撶礆
        friction_torque -= VISCOUS_DAMPING * velocity
    
    return friction_torque
```

---

##### 3.6 閻戭厾澹掗幀褍寮弫?

**閺佺増宓侀幍瀣斀閸欏倹鏆?*:
- Continuous Current: 0.69 A閿涘牐绻涚紒顓犳暩濞翠緤绱?
- Operating Temperature: -5 ~ 72鎺矯

**鏉烆剚宕叉稉铏瑰劰濡€崇€烽崣鍌涙殶**:

```python
def calculate_thermal_params(continuous_current, stall_current, voltage):
    """
    閻戭參妯?R_th = 铻朤 / P
    閸忔湹鑵?铻朤 = 濞撯晛宕岄敍瀛?= 閸旂喓宸奸幑鐔烩偓?
    """
    # 閸嬪洩顔曟潻鐐电敾閻㈠灚绁︽稉瀣帒鐠佸憡淇崡?50鎺矯閿涘牏骞嗘晶?25鎺矯 閳?75鎺矯閿?
    max_temp_rise = 50  # 鎺矯
    
    # 鏉╃偟鐢婚崝鐔哄芳
    continuous_power = continuous_current * voltage / 2  # 娴兼壆鐣?
    # P = 0.69 鑴?12 / 2 閳?4.14 W
    
    # 閻戭參妯?
    thermal_resistance = max_temp_rise / continuous_power
    # R_th = 50 / 4.14 閳?12 K/W閿涘牅绲炬潻娆忋亰娴ｅ函绱濈€圭偤妾痪?20-30 K/W閿?
    
    # 娴ｈ法鏁ょ紒蹇涚崣閸婇棿鎱ㄥ?
    thermal_resistance = 25  # K/W閿涘牆鐨崹瀣暩閺堝搫鍚€閸ㄥ鈧》绱?
    
    # 閻戭厽妞傞梻鏉戠埗閺佸府绱欑紒蹇涚崣閸婄》绱?
    # 鐏忓繐鐎烽悽鍨簚閿?0-30 閸掑棝鎸撴潏鎯у煂閻戭厼閽╃悰?
    thermal_time_constant = 20 * 60  # 1200 缁?
    
    return {
        'resistance': thermal_resistance,
        'time_constant': thermal_time_constant,
        'max_winding_temp': 125  # 鎺矯閿涘牊鐖ｉ崙鍡欑卜缂傛鐡戠痪?E閿?
    }
```

**Godot 鎼存梻鏁ら敍鍦橠Extension閿?*:
```cpp
// 缁犫偓閸栨牜娈戦悜顓熌侀崹瀣剁礄娑撯偓闂?RC 閻絻鐭鹃敍?
void update_temperature(float power_loss, float delta) {
    const float AMBIENT_TEMP = 25.0f;  // 鎺矯
    const float THERMAL_R = 25.0f;      // K/W
    const float THERMAL_TAU = 1200.0f;  // s
    
    // 濞撯晛宕岄弬鍦柤閿涙瓰T/dt = (P鑴砇 - T) / 锜?
    float heat_in = power_loss * THERMAL_R;
    float heat_out = (temperature - AMBIENT_TEMP) / THERMAL_TAU;
    
    temperature += (heat_in - heat_out) * delta;
    
    // 濞撯晛瀹虫穱婵囧Б
    if (temperature > MAX_WINDING_TEMP) {
        apply_thermal_derating();
    }
}
```

---

## 棣冩惓 鐎瑰本鏆ｆ潪顒佸床濞翠胶鈻奸崶?

```mermaid
graph TD
    A[閺佺増宓侀幍瀣斀] --> B{閸欏倹鏆熺猾璇茬€穧
    
    B -->|鐠愩劑鍣?鐏忓搫顕瓅 C[閻╁瓨甯存潪顒佸床閸楁洑缍匽
    C --> D[RigidBody3D.mass<br>CollisionShape3D.size]
    
    B -->|閻㈠灚婧€閹嗗厴| E[瀵よ櫣鐝涢悧鈺冩倞濡€崇€穄
    E --> F1[闁喎瀹?閹殿厾鐓╅弴鑼殠]
    E --> F2[閹解晜鎽濆Ο鈥崇€穄
    E --> F3[閻戭厽膩閸ㄥ獓
    
    F1 --> G[HingeJoint3D<br>閹?GDExtension]
    F2 --> G
    F3 --> G
    
    B -->|閺堫亞鐓￠崣鍌涙殶| H[娴兼壆鐣婚幋鏍ㄧ叀鐞涒暀
    H --> I{閸欘垶娼幀顫?
    I -->|妤傛 G
    I -->|娴ｅ藩 J[閺嶅洩顔囨稉杞板強缁犳鈧?br>瀵板懎鐤勫ù瀣崣鐠囦箽
```

---

## 棣冨箚 鐎圭偤妾鍫滅伐閿涙艾鐣弫瀵告畱 XL430 閸欏倹鏆熼弰鐘茬殸

### JSON 閺佺増宓侀敍鍫ｇ翻閸忋儻绱?

```json
{
  "part_id": "dynamixel_xl430_w250",
  "specifications": {
    "stall_torque": 1.4,           // 閺佺増宓侀幍瀣斀閿涙氨娲块幒銉х舶閸?
    "no_load_speed": 50,            // 閺佺増宓侀幍瀣斀閿涙氨娲块幒銉х舶閸戠尨绱橰PM閿?
    "weight": 0.057,                // 閺佺増宓侀幍瀣斀閿涙氨娲块幒銉х舶閸戠尨绱欐潪顒佸床娑?kg閿?
    "dimensions": [28.5, 46.5, 34], // 閺佺増宓侀幍瀣斀閿涙氨娲块幒銉х舶閸戠尨绱欐潪顒佸床娑?mm閿?
    "rotor_inertia": 3.5e-6,        // 娴兼壆鐣婚敍姘唨娴滃骸鏄傜€电鎷扮拹銊╁櫤
    "friction": {
      "static": 0.015,              // 娴兼壆鐣婚敍姘壄鏉烆剚澹勯惌鈺冩畱 1%
      "dynamic": 0.008,             // 娴兼壆鐣婚敍姘舵饯閹解晜鎽濋惃?53%
      "viscous": 0.001              // 缂佸繘鐛欓崐纭风窗鐏忓繐鐎烽懜鍨簚閸忕鐎烽崐?
    },
    "thermal": {
      "resistance": 25.0,           // 娴兼壆鐣婚敍姘唨娴滃氦绻涚紒顓犳暩濞?
      "time_constant": 1200,        // 缂佸繘鐛欓崐纭风窗20閸掑棝鎸?
      "max_winding_temp": 125       // 閺嶅洤鍣敍姘辩卜缂傛鐡戠痪?E
    },
    "motor_constant": 0.0108,       // 鐠侊紕鐣婚敍娆縯 = T_stall / I_stall
    "winding_resistance": 8.4       // 閺佺増宓侀幍瀣斀閿涙碍婀侀弮鍓佺舶閸?
  }
}
```

### Godot 娴狅絿鐖滈敍鍫ｇ翻閸戠尨绱?

```gdscript
# 1. 閸掓稑缂撻崚姘秼
var motor = RigidBody3D.new()
motor.mass = 0.057  # 閳?閻╁瓨甯存担璺ㄦ暏

# 2. 閸掓稑缂撶喊鐗堟寬瑜般垻濮?
var collision = CollisionShape3D.new()
var shape = BoxShape3D.new()
shape.size = Vector3(0.0285, 0.034, 0.0465)  # 閳?閸楁洑缍呮潪顒佸床 + 閸ф劖鐖ｉ弰鐘茬殸
collision.shape = shape
motor.add_child(collision)

# 3. 閸掓稑缂撻崗瀹犲Ν
var joint = HingeJoint3D.new()
joint.node_a = parent_body.get_path()
joint.node_b = motor.get_path()

# 4. 鎼存梻鏁ら悽鍨簚閸欏倹鏆熼敍鍫㈢暆閸栨牗膩閸ㄥ绱?
joint.set_param(
    HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE,
    1.4  # 閳?閻╁瓨甯存担璺ㄦ暏閸絻娴嗛幍顓犵叐
)

# 5. 鐎涙ê鍋嶉崗鍐╂殶閹诡喕绶垫妯奸獓濡€崇€锋担璺ㄦ暏
motor.set_meta("stall_torque", 1.4)
motor.set_meta("no_load_speed", 5.236)  # 閳?RPM 閳?rad/s
motor.set_meta("friction_static", 0.015)  # 閳?娴兼壆鐣婚崐?
motor.set_meta("friction_dynamic", 0.008)
motor.set_meta("thermal_resistance", 25.0)
```

### GDExtension C++ 娴狅絿鐖滈敍鍫ョ彯缁狙勀侀崹瀣剁礆

```cpp
// 妤傛楠囬悽鍨簚濡剝瀚欓崳顭掔礄濮ｅ繋閲滈悧鈺冩倞鐢嗙殶閻㈩煉绱?
float EnhancedMotorJoint::calculate_actual_torque(float target_velocity, float delta) {
    // 1. 閼惧嘲褰囪ぐ鎾冲闁喎瀹?
    float current_velocity = get_current_angular_velocity();
    
    // 2. 闁喎瀹?閹殿厾鐓╅弴鑼殠
    float speed_ratio = abs(current_velocity) / no_load_speed;
    float torque_available = stall_torque * (1.0f - speed_ratio);
    torque_available = std::max(0.0f, torque_available);
    
    // 3. 閸戝繐骞撻幗鈺傛憹閸?
    float friction_torque = 0.0f;
    if (abs(current_velocity) < 0.01f) {
        // 闂堟瑦鎳囬幙?
        friction_torque = std::copysign(friction_static, target_velocity);
    } else {
        // 閸斻劍鎳囬幙?+ 缁ɑ鈧囨▎鐏?
        friction_torque = std::copysign(friction_dynamic, current_velocity);
        friction_torque += viscous_damping * current_velocity;
    }
    
    // 4. 閻戭參妾锋０?
    float temp_factor = 1.0f;
    if (temperature > 80.0f) {
        temp_factor = std::max(0.5f, 1.0f - (temperature - 80.0f) / 100.0f);
    }
    
    // 5. 閸戔偓閹殿厾鐓?
    float net_torque = (torque_available - abs(friction_torque)) * temp_factor;
    
    // 6. 閺囧瓨鏌婂〒鈺佸
    float power_loss = abs(net_torque * current_velocity);
    update_temperature(power_loss, delta);
    
    return net_torque;
}
```

---

## 閳跨媴绗?鐢瓕顫嗛梽鐑芥Ш閸滃本鏁為幇蹇庣皑妞?

### 1. 閸楁洑缍呮潪顒佸床闁挎瑨顕?

```gdscript
# 閴?闁挎瑨顕ら敍姘箷鐠佹媽娴嗛幑銏犲礋娴?
motor.mass = 57.2  # 鏉╂瑤绱伴崚娑樼紦娑撯偓娑?57kg 閻ㄥ嫮鏁搁張鐚寸磼

# 閴?濮濓絿鈥?
motor.mass = 57.2 / 1000  # g 閳?kg
```

### 2. 閸ф劖鐖ｇ化缁樿穿濞?

```gdscript
# 閴?闁挎瑨顕ら敍姘辨纯閹恒儰濞囬悽銊︽殶閹诡喗澧滈崘宀€娈戦崸鎰垼
# 閺佺増宓侀幍瀣斀閿涙瓙(28.5) 鑴?H(34.0) 鑴?D(46.5) mm閿涘鏉炴潙鎮滄稉?
shape.size = Vector3(28.5, 34.0, 46.5) / 1000

# 閴?濮濓絿鈥橀敍姘跺櫢閺傜増妲х亸鍕煂 Godot 閸ф劖鐖ｇ化浼欑礄Y鏉炴潙鎮滄稉濠忕礆
shape.size = Vector3(
    28.5 / 1000,  # X = 鐎硅棄瀹?
    34.0 / 1000,  # Y = 妤傛ê瀹抽敍鍦檕dot閿?
    46.5 / 1000   # Z = 濞ｅ崬瀹?
)
```

### 3. 鏉╁洤瀹崇粻鈧崠鏍⒖閻炲棙膩閸?

```gdscript
# 閴?闁挎瑨顕ら敍姘海鐠佸墽鏁搁張鐑樷偓缁樻Ц鏉堟挸鍤張鈧径褎澹勯惌?
joint.set_param(PARAM_MOTOR_MAX_IMPULSE, 1.4)
# 缂佹挻鐏夐敍姘簚閸ｃ劋姹夐崷銊╃彯闁喕绻嶉崝銊︽閹殿厾鐓╂潻鍥с亣閿涘奔绗夐惇鐔风杽

# 閴?濮濓絿鈥橀敍姘杽閻滀即鈧喎瀹?閹殿厾鐓╅弴鑼殠
func _physics_process(delta):
    var velocity = get_joint_velocity()
    var torque = calculate_motor_torque(velocity)
    joint.set_param(PARAM_MOTOR_MAX_IMPULSE, torque * delta)
```

### 4. 韫囩晫鏆愭导鎵暬閸欏倹鏆熼惃鍕瑝绾喖鐣鹃幀?

```json
// 閴?娑撳秴銈介敍姘梾閺堝鐖ｅ▔銊ゅ強缁犳鈧?
{
  "friction": {
    "static": 0.015,
    "dynamic": 0.008
  }
}

// 閴?婵傛枻绱伴弰搴ｂ€橀弽鍥ㄦ暈楠炶埖褰佹笟娑欐降濠?
{
  "friction": {
    "static": 0.015,     // 娴兼壆鐣婚崐纭风礉閸╄桨绨崼浣冩祮閹殿厾鐓?1%
    "dynamic": 0.008,    // 娴兼壆鐣婚崐纭风礉闂堟瑦鎳囬幙锔炬畱 53%
    "source": "estimated",
    "confidence": "medium"
  },
  "notes": "閹解晜鎽濋崣鍌涙殶娑撹桨鍙婄粻妤€鈧》绱濆楦款唴闁俺绻冪€圭偞绁存宀冪槈"
}
```

---

## 棣冩暕 妤犲矁鐦夐弬瑙勭《

### 1. 閻炲棜顔戞宀冪槈

**閼充粙鍣虹€瑰牊浜藉Λ鈧弻?*:
```python
def verify_power_balance(motor_data):
    """妤犲矁鐦夐崝鐔哄芳楠炲疇銆€"""
    voltage = 12.0  # V
    stall_current = 1.4  # A
    stall_torque = 1.4  # N璺痬
    
    # 鏉堟挸鍙嗛悽闈涘閻?
    electrical_power = voltage * stall_current  # 16.8 W
    
    # 閺堢儤顫崝鐔哄芳閿涘牆鐗潪顒佹娑?0閿?
    mechanical_power = stall_torque * 0  # 0 W
    
    # 閹圭喕鈧濮涢悳鍥风礄鎼存棁顕氶幒銉ㄧ箮鏉堟挸鍙嗛崝鐔哄芳閿?
    loss_power = electrical_power - mechanical_power  # 16.8 W
    
    # 濡偓閺屻儲宕懓妤佹Ц閸氾箑鎮庨悶鍡礄鏉烆剚宕叉稉铏瑰劰闁插骏绱?
    temp_rise = loss_power * thermal_resistance  # 16.8 鑴?25 = 420鎺矯
    # 閳跨媴绗?鏉╂瑥銇婃妯圭啊閿涗浇顕╅弰搴＄壄鏉烆剚妞傞悽鍨簚娴兼俺绻冮悜顓ㄧ礉闂団偓鐟曚椒绻氶幎?
    
    return temp_rise < max_winding_temp
```

### 2. 鐎佃鐦禒璺ㄦ埂

閸?Godot 娑擃參鍣搁悳鐗堟殶閹诡喗澧滈崘灞艰厬閻ㄥ嫭鐖ｉ崙鍡樼ゴ鐠囨洩绱?

```gdscript
# 濞村鐦?閿涙艾鐗潪顒佸閻晜绁寸拠?
func test_stall_torque():
    # 閸ュ搫鐣鹃崗瀹犲Ν閿涘本绁撮柌蹇旀付婢堆勫閻?
    joint.set_param(PARAM_MOTOR_TARGET_VELOCITY, 0)
    
    var measured_torque = measure_joint_torque()
    var spec_torque = 1.4
    
    var error = abs(measured_torque - spec_torque) / spec_torque
    assert(error < 0.1, "閹殿厾鐓╃拠顖氭▕鎼?< 10%")

# 濞村鐦?閿涙氨鈹栨潪浠嬧偓鐔峰濞村鐦?
func test_no_load_speed():
    # 閺冪姾绀嬫潪鏂ょ礉濞村鍣洪張鈧径褔鈧喎瀹?
    joint.set_param(PARAM_MOTOR_TARGET_VELOCITY, 999)
    
    await get_tree().create_timer(2.0).timeout
    var measured_speed = get_joint_velocity()
    var spec_speed = 5.236  # rad/s
    
    var error = abs(measured_speed - spec_speed) / spec_speed
    assert(error < 0.1, "闁喎瀹崇拠顖氭▕鎼?< 10%")
```

### 3. 鐎圭偞绁撮弫鐗堝祦鐎佃鐦敍鍦玦m-to-Real閿?

婵″倹鐏夐張澶屾埂鐎圭偟娈戦悽鍨簚閿涘苯褰叉禒銉ㄧ箻鐞涘苯顕В鏃€绁寸拠鏇窗

| 濞村鐦い?| 娴犺法婀￠崐?| 鐎圭偞绁撮崐?| 鐠囶垰妯?| 閸欘垱甯撮崣妤嬬吹 |
|--------|--------|--------|------|----------|
| 閸絻娴嗛幍顓犵叐 | 1.40 N璺痬 | 1.38 N璺痬 | 1.4% | 閴?|
| 缁岄缚娴囬柅鐔峰 | 5.24 rad/s | 5.18 rad/s | 1.1% | 閴?|
| 閸旂娀鈧喐妞傞梻?(0閳?0 RPM) | 0.45 s | 0.52 s | 15.5% | 閳跨媴绗?闂団偓鐟曚浇鐨熼弫瀛樺劵闁?|
| 鏉╃偟鐢绘潻鎰攽濞撯晛宕?| 42鎺矯 | 48鎺矯 | 14.3% | 閳跨媴绗?闂団偓鐟曚浇鐨熼弫瀵稿劰闂?|

---

## 棣冩惐 閸欏倹鏆熺划鍓р€樻惔锕€鍨庣痪?

閺嶈宓侀弶銉︾爱閸滃苯褰查棃鐘斥偓褝绱濈€电懓寮弫鎷岀箻鐞涘苯鍨庣痪褝绱?

| 缁涘楠?| 閺夈儲绨?| 閸忕鐎风拠顖氭▕ | 缁€杞扮伐 |
|------|------|----------|------|
| **A - 閻╁瓨甯村ù瀣櫤** | 閺佺増宓侀幍瀣斀閺勫海鈥樼紒娆忓毉 | < 5% | 鐠愩劑鍣洪妴浣告槀鐎垫悶鈧礁鐗潪顒佸閻?|
| **B - 鐠侊紕鐣婚幒銊ヮ嚤** | 閸╄桨绨崗鏈电铂閸欏倹鏆熺拋锛勭暬 | 5-15% | 閻㈠灚婧€鐢憡鏆熼妴浣瑰劵闁?|
| **C - 缂佸繘鐛欐导鎵暬** | 閸╄桨绨崥宀€琚禍褍鎼х紒蹇涚崣 | 15-30% | 閹解晜鎽濈化缁樻殶閵嗕胶鍎归梼?|
| **D - 缁鏆愰悮婊勭ゴ** | 缂傚搫鐨弫鐗堝祦閿涘奔濞囬悽銊╃帛鐠併倕鈧?| > 30% | 缁ɑ鈧囨▎鐏忕鈧焦娼楅弬娆忚剨閹?|

**瀵ら缚顔?*閿?
- A/B 缁狙冨棘閺佸府绱伴惄瀛樺复娴ｈ法鏁?
- C 缁狙冨棘閺佸府绱伴弽鍥ㄦ暈娑撹桨鍙婄粻妤嬬礉閸氬海鐢婚崣顖炩偓姘崇箖鐎圭偞绁存导妯哄
- D 缁狙冨棘閺佸府绱伴弽鍥ㄦ暈娑撳搫绶熸宀冪槈閿涘奔绱崗鍫㈤獓娴ｅ海娈戞い鍦窗閸欘垯浜掗崗鍫㈡暏姒涙顓婚崐?

---

## 棣冩畬 閹崵绮?

鐏忓棛婀＄€圭偛寮弫鎷屾祮閹诡澀璐熷鏇熸惛閸欏倹鏆熼惃鍕彠闁款喗顒炴銈忕窗

1. **閺€鍫曟肠閺佺増宓侀幍瀣斀閸欏倹鏆?* 閳?閸樼喎顫愰弫鐗堝祦
2. **閸楁洑缍呮潪顒佸床** 閳?SI 閺嶅洤鍣崡鏇氱秴
3. **閻椻晝鎮婂鐑樐?* 閳?闁瀚ㄩ崥鍫モ偓鍌滄畱鏉╂垳鎶€濡€崇€?
4. **閸欏倹鏆熼弰鐘茬殸** 閳?閺勭姴鐨犻崚鏉跨穿閹?API
5. **妤犲矁鐦夐崪宀冪殶娴?* 閳?鐎佃鐦€圭偞绁撮弫鐗堝祦

鏉╂瑤閲滄潻鍥┾柤闂団偓鐟曚緤绱?
- 棣冩惢 **閻椻晝鎮婇惌銉ㄧ槕**閿涙氨鎮婄憴锝呭棘閺佹壆娈戦悧鈺冩倞閹板繋绠?
- 棣冩暋 **瀹搞儳鈻奸崚銈嗘焽**閿涙艾婀划鍓р€樻惔锕€鎷伴弫鍫㈠芳闂傚瓨娼堢悰?
- 棣冃?**鐎圭偤鐛欐宀冪槈**閿涙岸鈧俺绻冨ù瀣槸绾喕绻氬锝団€橀幀?

---

**閻楀牊婀?*: 1.0  
**娴ｆ粏鈧?*: AGI-Walker Team  
**閺堚偓閸氬孩娲块弬?*: 2026-01-13
