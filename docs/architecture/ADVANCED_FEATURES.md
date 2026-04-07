# AGI-Walker 妤傛楠囬崝鐔诲厴閹碘晛鐫嶉幐鍥у础

閺堫剚鏋冨锝堫嚛閺勫骸顩ф担鏇氳礋AGI-Walker妞ゅ湱娲板ǎ璇插妤傛楠囬崝鐔诲厴閿涘苯瀵橀幏顒佹纯婢舵碍婧€閸ｃ劋姹夐崣鍌涙殶閵嗕浇绻嶉崝銊ㄧ熅瀵板嫯顫夐崚鎺嬧偓渚€娈扮喊宥囧⒖鐠囧棗鍩嗛崪灞介挬鐞涒剝甯堕崚韬测偓?

---

## 棣冩惖 閻╊喖缍?

1. [閹碘晛鐫嶉張鍝勬珤娴滃搫寮弫鐧?#1-閹碘晛鐫嶉張鍝勬珤娴滃搫寮弫?
2. [鏉╂劕濮╃捄顖氱窞鐟欏嫬鍨漖(#2-鏉╂劕濮╃捄顖氱窞鐟欏嫬鍨?
3. [闂呮粎顣查悧鈺勭槕閸掔帡(#3-闂呮粎顣查悧鈺勭槕閸?
4. [楠炲疇銆€閹貉冨煑缁犳纭禲(#4-楠炲疇銆€閹貉冨煑缁犳纭?
5. [闂嗗棙鍨氱粈杞扮伐](#5-闂嗗棙鍨氱粈杞扮伐)

---

## 1. 閹碘晛鐫嶉張鍝勬珤娴滃搫寮弫?

### 1.1 濞ｈ濮為弴鏉戭樋閸忓疇濡?

#### 瑜版挸澧犵紒鎾寸€敍?閼奉亞鏁辨惔锔肩礆
```
Robot
閳规壕鏀㈤埞鈧?Torso
閳规壕鏀㈤埞鈧?LeftLeg (妤傚鍙ч懞?
閳规柡鏀㈤埞鈧?RightLeg (妤傚鍙ч懞?
```

#### 閹碘晛鐫嶆稉?閼奉亞鏁辨惔锔肩礄濞ｈ濮為懚婵嗗彠閼哄偊绱?
```
Robot
閳规壕鏀㈤埞鈧?Torso
閳规壕鏀㈤埞鈧?LeftThigh (妤傚鍙ч懞?
閳规壕鏀㈤埞鈧?LeftCalf (閼舵繂鍙ч懞?
閳规壕鏀㈤埞鈧?RightThigh (妤傚鍙ч懞?
閳规柡鏀㈤埞鈧?RightCalf (閼舵繂鍙ч懞?
```

#### Godot閸︾儤娅欓崚娑樼紦濮濄儵顎?

1. **濞ｈ濮炴径褑鍚欓懞鍌滃仯**
```
# LeftThigh 閸?RightThigh (RigidBody3D)
鐏忓搫顕? 0.2 x 0.4 x 0.2
鐠愩劑鍣? 2kg
```

2. **濞ｈ濮炵亸蹇氬悪閼哄倻鍋?*
```
# LeftCalf 閸?RightCalf (RigidBody3D)
鐏忓搫顕? 0.2 x 0.4 x 0.2
鐠愩劑鍣? 1.5kg
```

3. **闁板秶鐤嗛崗瀹犲Ν**
```gdscript
# 妤傚鍙ч懞?(鏉╃偞甯?Torso 閸?Thigh)
HipLeft/HipRight (HingeJoint3D)
- 闂勬劒缍? -45鎺?閸?90鎺?

# 閼舵繂鍙ч懞?(鏉╃偞甯?Thigh 閸?Calf)
KneeLeft/KneeRight (HingeJoint3D)
- 闂勬劒缍? -120鎺?閸?0鎺?(閸欘亣鍏橀崥鎴濇倵瀵?
```

#### 娣囶喗鏁糋DScript

```gdscript
# box_robot.gd 閹碘晛鐫嶉悧?
extends Node3D

# 閺傛澘顤冮崗瀹犲Ν瀵洜鏁?
@onready var left_thigh: RigidBody3D = get_node_or_null("LeftThigh")
@onready var left_calf: RigidBody3D = get_node_or_null("LeftCalf")
@onready var knee_left: HingeJoint3D = get_node_or_null("KneeLeft")
@onready var knee_right: HingeJoint3D = get_node_or_null("KneeRight")

# 閹碘晛鐫嶉崗瀹犲Ν鐟欐帒瀹崇€涙鍚€
var joint_angles := {
    "hip_left": 0.0,
    "hip_right": 0.0,
    "knee_left": 0.0,    # 閺傛澘顤?
    "knee_right": 0.0    # 閺傛澘顤?
}

# 閹碘晛鐫嶉惄顔界垼鐟欐帒瀹崇€涙鍚€
var target_angles := {
    "hip_left": 0.0,
    "hip_right": 0.0,
    "knee_left": 0.0,
    "knee_right": 0.0
}
```

### 1.2 濞ｈ濮為懘姘崇閸忓疇濡敍?閼奉亞鏁辨惔锔肩礆

缂佈呯敾閹碘晛鐫嶉崣顖涘潑閸旂媴绱?
- **AnkleLeft/AnkleRight**: 閼存俺绗ｆ穱顖欒瘽
- **WaistJoint**: 闊垰鍏遍弮瀣祮
- **ArmJoints**: 閹靛鍣﹂幗鍡楀З閿涘牐绶熼崝鈺侀挬鐞涒槄绱?

---

## 2. 鏉╂劕濮╃捄顖氱窞鐟欏嫬鍨?

### 2.1 鐠侯垰绶炵悰銊с仛

閸掓稑缂撶捄顖氱窞缁狅紕鎮婇崳顭掔窗

```gdscript
# path_manager.gd
extends Node3D

# 鐠侯垰绶為悙鐟板灙鐞?
var waypoints: Array[Vector3] = []
var current_waypoint_index := 0

# 鐠侯垰绶為崣顖濐潒閸?
@onready var path_line: MeshInstance3D


func _ready():
    # 缁€杞扮伐鐠侯垰绶為敍姘劀閺傜懓鑸?
    waypoints = [
        Vector3(0, 0, 0),
        Vector3(2, 0, 0),
        Vector3(2, 0, 2),
        Vector3(0, 0, 2),
        Vector3(0, 0, 0)
    ]
    _draw_path()


func get_current_target() -> Vector3:
    """閼惧嘲褰囪ぐ鎾冲閻╊喗鐖ｉ悙?""
    if current_waypoint_index < waypoints.size():
        return waypoints[current_waypoint_index]
    return Vector3.ZERO


func advance_waypoint():
    """閸撳秷绻橀崚棰佺瑓娑撯偓娑擃亣鐭惧鍕仯"""
    current_waypoint_index += 1
    if current_waypoint_index >= waypoints.size():
        print("閴?鐠侯垰绶炵€瑰本鍨?")


func is_near_target(robot_pos: Vector3, threshold: float = 0.5) -> bool:
    """濡偓閺屻儲妲搁崥锔藉复鏉╂垹娲伴弽鍥╁仯"""
    var target = get_current_target()
    return robot_pos.distance_to(target) < threshold


func _draw_path():
    """閸欘垵顫嬮崠鏍熅瀵?""
    # 娴ｈ法鏁mmediateMesh缂佹ê鍩楃痪鎸庢蒋
    var mesh = ImmediateMesh.new()
    mesh.surface_begin(Mesh.PRIMITIVE_LINE_STRIP)
    
    for point in waypoints:
        mesh.surface_add_vertex(point)
    
    mesh.surface_end()
    
    if path_line:
        path_line.mesh = mesh
```

### 2.2 鐎佃壈鍩呴幒褍鍩楅崳?

```python
# navigation_controller.py
import math
from tcp_client import GodotClient

class NavigationController:
    """鐠侯垰绶炵捄鐔兼閹貉冨煑閸?""
    
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
        """鐠侊紕鐣婚張婵嗘倻閻╊喗鐖ｉ惃鍕祮閸氭垼顫?""
        dx = target_pos[0] - robot_pos[0]
        dz = target_pos[1] - robot_pos[1]
        return math.atan2(dz, dx)  # 瀵冨
    
    def get_motor_commands(self, sensor_data):
        """閺嶈宓佽ぐ鎾冲娴ｅ秶鐤嗛悽鐔稿灇閻㈠灚婧€閹稿洣鎶?""
        # 閼惧嘲褰囬張鍝勬珤娴滆桨缍呯純顕嗙礄闂団偓鐟曚焦鍧婇崝鐘辩秴缂冾喛鎷烽煪顏庣礆
        robot_x = sensor_data.get('position_x', 0)
        robot_z = sensor_data.get('position_z', 0)
        
        # 瑜版挸澧犻惄顔界垼閻?
        target = self.waypoints[self.current_waypoint]
        
        # 鐠侊紕鐣荤捄婵堫瀲
        dist = math.sqrt((target[0] - robot_x)**2 + (target[1] - robot_z)**2)
        
        # 閸掓媽鎻惄顔界垼閻愮櫢绱濋崚鍥ㄥ床娑撳绔存稉?
        if dist < 0.5:
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
            print(f"閴?閸掓媽鎻捄顖氱窞閻?{self.current_waypoint}")
        
        # 鐠侊紕鐣绘潪顒€鎮?
        heading = self.calculate_heading_angle((robot_x, robot_z), target)
        
        # 缁犫偓閸楁洜娈戝顕€鈧喖鈹嶉崝顭掔礄瀹革箑褰搁懙澶哥瑝閸氬本顒為敍?
        turn_gain = 10  # 鏉烆剙鎮滄晶鐐垫抄
        return {
            "motors": {
                "hip_left": heading * turn_gain,
                "hip_right": -heading * turn_gain
            }
        }
```

---

## 3. 闂呮粎顣查悧鈺勭槕閸?

### 3.1 鐠烘繄顬囨导鐘冲妳閸?

#### Godot缁?- 鐏忓嫮鍤庡Λ鈧ù?

```gdscript
# obstacle_detector.gd
extends Node3D

# 鐏忓嫮鍤庢导鐘冲妳閸ｃ劍鏆熺紒?
var ray_sensors: Array[RayCast3D] = []
const NUM_RAYS = 5  # 5娑擃亝鏌熼崥?
const RAY_LENGTH = 3.0  # 3缁櫕顥呭ù瀣獩缁?

func _ready():
    _create_ray_sensors()

func _create_ray_sensors():
    """閸掓稑缂撴径姘煙閸氭垵鐨犵痪澶哥炊閹扮喎娅?""
    var angles = [-45, -22.5, 0, 22.5, 45]  # 鎼?
    
    for i in range(NUM_RAYS):
        var ray = RayCast3D.new()
        add_child(ray)
        
        # 鐠佸墽鐤嗛弬鐟版倻
        var angle_rad = deg_to_rad(angles[i])
        ray.target_position = Vector3(
            sin(angle_rad) * RAY_LENGTH,
            0,
            cos(angle_rad) * RAY_LENGTH
        )
        
        ray.enabled = true
        ray_sensors.append(ray)

func get_obstacle_distances() -> Array[float]:
    """閼惧嘲褰囬崥鍕煙閸氭垹娈戦梾婊咁暡閻椻晞绐涚粋?""
    var distances: Array[float] = []
    
    for ray in ray_sensors:
        if ray.is_colliding():
            var collision_point = ray.get_collision_point()
            var dist = global_position.distance_to(collision_point)
            distances.append(dist)
        else:
            distances.append(RAY_LENGTH)  # 閺冪娀娈扮喊?
    
    return distances

func get_sensor_data() -> Dictionary:
    """閺嶇厧绱￠崠鏍︾炊閹扮喎娅掗弫鐗堝祦"""
    var distances = get_obstacle_distances()
    return {
        "obstacle_distances": distances,
        "closest_obstacle": distances.min(),
        "has_obstacle": distances.min() < 1.0  # 1缁啿鍞撮張澶愭绾?
    }
```

#### 閸︺劋瀵岄張鍝勬珤娴滆桨鑵戦梿鍡樺灇

```gdscript
# box_robot.gd (濞ｈ濮?
@onready var obstacle_detector = $ObstacleDetector

func get_sensor_data() -> Dictionary:
    # ... 閸樼喐婀佹禒锝囩垳 ...
    return {
        "timestamp": Time.get_ticks_msec() / 1000.0,
        "sensors": {
            "imu": _get_imu_data(),
            "joints": _get_joint_data(),
            "contacts": _get_contact_data(),
            "obstacles": obstacle_detector.get_sensor_data()  # 閺傛澘顤?
        },
        "torso_height": torso.global_position.y
    }
```

### 3.2 鐟欏棜顫庢导鐘冲妳閸ｎ煉绱機amera閿?

```gdscript
# vision_sensor.gd
extends Camera3D

var viewport: SubViewport

func _ready():
    # 閸掓稑缂撶粋璇茬潌濞撳弶鐓?
    viewport = SubViewport.new()
    viewport.size = Vector2i(320, 240)  # 娴ｅ骸鍨庢潏銊у芳
    add_child(viewport)

func capture_image() -> Image:
    """閹规洝骞忛惄鍛婃簚閸ユ儳鍎?""
    await RenderingServer.frame_post_draw
    return viewport.get_texture().get_image()

func detect_objects() -> Array:
    """缁犫偓閸楁洜娈戦悧鈺€缍嬪Λ鈧ù瀣剁礄妫版粏澹婄拠鍡楀焼閿?""
    var image = capture_image()
    var objects = []
    
    # 缁€杞扮伐閿涙碍顥呭ù瀣閼硅尙澧挎担?
    # 鐎圭偤妾惔鏃傛暏娑擃厼褰叉禒銉╂肠閹存劘顓哥粻妤佹簚鐟欏棜顫庣粻妤佺《
    
    return objects
```

---

## 4. 楠炲疇銆€閹貉冨煑缁犳纭?

### 4.1 PID閹貉冨煑閸?

```gdscript
# pid_controller.gd
class_name PIDController

var kp: float  # 濮ｆ柧绶ユ晶鐐垫抄
var ki: float  # 缁夘垰鍨庢晶鐐垫抄
var kd: float  # 瀵邦喖鍨庢晶鐐垫抄

var integral: float = 0.0
var last_error: float = 0.0

func _init(p: float, i: float, d: float):
    kp = p
    ki = i
    kd = d

func compute(error: float, dt: float) -> float:
    """鐠侊紕鐣籔ID鏉堟挸鍤?""
    # 缁夘垰鍨庢い?
    integral += error * dt
    
    # 瀵邦喖鍨庢い?
    var derivative = (error - last_error) / dt if dt > 0 else 0.0
    
    # PID閸忣剙绱?
    var output = kp * error + ki * integral + kd * derivative
    
    last_error = error
    return output

func reset():
    """闁插秶鐤嗛悩鑸碘偓?""
    integral = 0.0
    last_error = 0.0
```

### 4.2 婵寧鈧礁閽╃悰鈩冨付閸?

```gdscript
# balance_controller.gd
extends Node

@onready var robot = get_node("/root/Main/Robot")

# PID閹貉冨煑閸?
var roll_pid: PIDController
var pitch_pid: PIDController

func _ready():
    # 鐠嬪啩绱惃鍑盜D閸欏倹鏆?
    roll_pid = PIDController.new(5.0, 0.1, 2.0)
    pitch_pid = PIDController.new(5.0, 0.1, 2.0)

func compute_balance_commands(sensor_data: Dictionary, dt: float) -> Dictionary:
    """鐠侊紕鐣婚獮瀹犮€€閹貉冨煑閹稿洣鎶?""
    var orient = sensor_data['sensors']['imu']['orient']
    var roll = orient[0]
    var pitch = orient[1]
    
    # 閻╊喗鐖ｆ慨鎸庘偓渚婄窗閻╁鐝涢敍?鎼达讣绱?
    var roll_error = 0.0 - roll
    var pitch_error = 0.0 - pitch
    
    # PID鐠侊紕鐣?
    var roll_correction = roll_pid.compute(roll_error, dt)
    var pitch_correction = pitch_pid.compute(pitch_error, dt)
    
    # 鏉烆剚宕叉稉铏规暩閺堢儤瀵氭禒銈忕礄缁犫偓閸栨牜澧楅敍?
    return {
        "motors": {
            "hip_left": pitch_correction + roll_correction,
            "hip_right": pitch_correction - roll_correction
        }
    }
```

### 4.3 ZMP閿涘牓娴傞崝娑氱叐閻愮櫢绱氶獮瀹犮€€

```python
# zmp_controller.py
import numpy as np

class ZMPController:
    """闂嗚泛濮忛惌鈺冨仯楠炲疇銆€閹貉冨煑閸?""
    
    def __init__(self, robot_height=1.0, gravity=9.8):
        self.height = robot_height
        self.g = gravity
        
    def calculate_zmp(self, com_pos, com_acc):
        """
        鐠侊紕鐣籞MP娴ｅ秶鐤?
        com_pos: 闁插秴绺炬担宥囩枂 [x, y, z]
        com_acc: 闁插秴绺鹃崝鐘烩偓鐔峰 [ax, ay, az]
        """
        x_com, y_com, z_com = com_pos
        ax, ay, az = com_acc
        
        # ZMP閸忣剙绱?
        x_zmp = x_com - (z_com / (az - self.g)) * ax
        y_zmp = y_com - (z_com / (az - self.g)) * ay
        
        return [x_zmp, y_zmp]
    
    def is_stable(self, zmp, support_polygon):
        """
        濡偓閺岊櫊MP閺勵垰鎯侀崷銊︽暜閹炬垵顦挎潏鐟拌埌閸?
        zmp: [x, y]
        support_polygon: [(x1,y1), (x2,y2), ...]
        """
        # 娴ｈ法鏁ょ亸鍕殠濞夋洖鍨介弬顓犲仯閺勵垰鎯侀崷銊ヮ樋鏉堢懓鑸伴崘?
        # 缁犫偓閸栨牜澧楅張顒婄窗閸嬪洩顔曢弨顖涙嫼婢舵俺绔熻ぐ銏℃Ц閻晛鑸?
        x, y = zmp
        min_x = min(p[0] for p in support_polygon)
        max_x = max(p[0] for p in support_polygon)
        min_y = min(p[1] for p in support_polygon)
        max_y = max(p[1] for p in support_polygon)
        
        return min_x <= x <= max_x and min_y <= y <= max_y

    def compute_correction(self, zmp, support_center):
        """鐠侊紕鐣婚獮瀹犮€€娣囶喗顒?""
        error_x = support_center[0] - zmp[0]
        error_y = support_center[1] - zmp[1]
        
        # 缁犫偓閸楁洘鐦笟瀣付閸?
        gain = 2.0
        return [error_x * gain, error_y * gain]
```

### 4.4 闁插秴绺剧拋锛勭暬

```gdscript
# center_of_mass.gd
extends Node

func calculate_com(bodies: Array[RigidBody3D]) -> Vector3:
    """鐠侊紕鐣荤化鑽ょ埠闁插秴绺?""
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
    """鐠侊紕鐣婚弨顖涙嫼婢舵俺绔熻ぐ?""
    # 鏉╂柨娲栭懘姘俺閹恒儴袝閻愮懓娲块幋鎰畱婢舵俺绔熻ぐ?
    return contact_points
```

---

## 5. 闂嗗棙鍨氱粈杞扮伐

### 5.1 鐎瑰本鏆ｉ惃鍑橧閹貉冨煑瀵邦亞骞?

```python
# advanced_controller.py
import time
from tcp_client import GodotClient
from navigation_controller import NavigationController
from zmp_controller import ZMPController

class AdvancedController:
    """闂嗗棙鍨氶幍鈧張澶愮彯缁狙冨閼崇晫娈戦幒褍鍩楅崳?""
    
    def __init__(self, model_path: str):
        self.client = GodotClient()
        self.navigator = NavigationController(self.client)
        self.zmp = ZMPController()
        
        # AI濡€崇€烽敍鍫仛娓氬绱?
        # self.ai_model = load_model(model_path)
        
    def run(self, duration: float = 120.0):
        """鏉╂劘顢戞妯奸獓閹貉冨煑瀵邦亞骞?""
        self.client.connect()
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # 1. 閼惧嘲褰囨导鐘冲妳閸ｃ劍鏆熼幑?
            sensor_data = self.client.get_latest_sensors()
            if not sensor_data:
                continue
            
            # 2. 闂呮粎顣查悧鈺傤梾濞?
            if 'obstacles' in sensor_data['sensors']:
                obstacles = sensor_data['sensors']['obstacles']
                if obstacles['has_obstacle']:
                    print(f"閳跨媴绗?濡偓濞村鍩岄梾婊咁暡閻椻晪绱濈捄婵堫瀲: {obstacles['closest_obstacle']:.2f}m")
                    # 闁潡娈伴柅鏄忕帆
            
            # 3. 楠炲疇銆€閹貉冨煑
            orient = sensor_data['sensors']['imu']['orient']
            roll, pitch = orient[0], orient[1]
            
            if abs(roll) > 30 or abs(pitch) > 30:
                print("閳跨媴绗?婵寧鈧椒绗夌粙鍐茬暰閿涘本澧界悰灞介挬鐞涒剝浠径?)
                # 楠炲疇銆€閹垹顦查柅鏄忕帆
            
            # 4. 鐠侯垰绶炵€佃壈鍩?
            nav_commands = self.navigator.get_motor_commands(sensor_data)
            
            # 5. AI閹恒劎鎮婇敍鍫濐洤閺嬫粌鎯庨悽顭掔礆
            # ai_commands = self.ai_model.predict(sensor_data)
            
            # 6. 閾诲秴鎮庨幐鍥︽姢楠炶泛褰傞柅?
            self.client.send_motor_commands(nav_commands)
            
            time.sleep(0.033)  # 30Hz
        
        self.client.close()
```

### 5.2 閸欏倹鏆熼柊宥囩枂閺傚洣娆?

```python
# config.py
"""
妤傛楠囬崝鐔诲厴闁板秶鐤嗛崣鍌涙殶
"""

# 鐠侯垰绶炵憴鍕灊
PATH_WAYPOINTS = [
    (0, 0),
    (5, 0),
    (5, 5),
    (0, 5)
]

# 闂呮粎顣查悧鈺傤梾濞?
OBSTACLE_DETECTION_RANGE = 3.0  # 缁?
OBSTACLE_AVOIDANCE_DISTANCE = 1.0  # 缁?
NUM_DISTANCE_SENSORS = 5

# 楠炲疇銆€閹貉冨煑
PID_ROLL = {"kp": 5.0, "ki": 0.1, "kd": 2.0}
PID_PITCH = {"kp": 5.0, "ki": 0.1, "kd": 2.0}

# ZMP
ROBOT_HEIGHT = 1.0
GRAVITY = 9.8
STABILITY_MARGIN = 0.05  # 缁?

# 閸忓疇濡梽鎰秴
JOINT_LIMITS = {
    "hip": (-45, 90),
    "knee": (-120, 0),
    "ankle": (-30, 30)
}
```

---

## 6. 鐎圭偞鏌︾捄顖滃殠閸?

### 闂冭埖顔?: 閹碘晛鐫嶉崗瀹犲Ν閿?閸涱煉绱?
- [ ] 濞ｈ濮為懚婵嗗彠閼哄倸鎷伴懘姘崇
- [ ] 閺囧瓨鏌婃导鐘冲妳閸ｃ劍鏆熼幑顔剧波閺?
- [ ] 濞村鐦弬鏉垮彠閼哄倹甯堕崚?

### 闂冭埖顔?: 闂呮粎顣查悧鈺傤梾濞村绱?閸涱煉绱?
- [ ] 鐎圭偟骞囩亸鍕殠娴肩姵鍔呴崳?
- [ ] 闂嗗棙鍨氶崚棰佺炊閹扮喎娅掗弫鐗堝祦濞?
- [ ] Python缁旑垵袙閺嬫劙娈扮喊宥囧⒖娣団剝浼?

### 闂冭埖顔?: 鐠侯垰绶炵憴鍕灊閿?閸涱煉绱?
- [ ] 閸掓稑缂撶捄顖氱窞缁狅紕鎮婇崳?
- [ ] 鐎圭偟骞囩捄顖氱窞鐠虹喖娈㈢粻妤佺《
- [ ] 閸欘垵顫嬮崠鏍熅瀵?

### 闂冭埖顔?: 楠炲疇銆€閹貉冨煑閿?閸涱煉绱?
- [ ] 鐎圭偟骞嘝ID閹貉冨煑閸?
- [ ] 闂嗗棙鍨歓MP缁犳纭?
- [ ] 鐠嬪啩绱崣鍌涙殶

### 闂冭埖顔?: 闂嗗棙鍨氬ù瀣槸閿?閸涱煉绱?
- [ ] 閼辨柨鎮庡ù瀣槸閹碘偓閺堝濮涢懗?
- [ ] 閹嗗厴娴兼ê瀵?
- [ ] 閺傚洦銆傜€瑰苯鏉?

---

## 7. 鐠嬪啳鐦銉ュ徔

### 閸欘垵顫嬮崠鏍殶鐠囨洖娅?

```gdscript
# debug_overlay.gd
extends Control

@onready var label = $Label

func _process(_delta):
    var robot = get_node("/root/Main/Robot")
    if robot and robot.is_scene_ready:
        var sensor_data = robot.get_sensor_data()
        
        var text = "=== 鐠嬪啳鐦穱鈩冧紖 ===\n"
        text += "婵寧鈧? Roll=%.1f鎺?Pitch=%.1f鎺砛n" % [
            sensor_data['sensors']['imu']['orient'][0],
            sensor_data['sensors']['imu']['orient'][1]
        ]
        text += "妤傛ê瀹? %.2fm\n" % sensor_data['torso_height']
        text += "閹恒儱婀? L=%s R=%s\n" % [
            "閴? if sensor_data['sensors']['contacts']['foot_left'] else "閴?,
            "閴? if sensor_data['sensors']['contacts']['foot_right'] else "閴?
        ]
        
        label.text = text
```

---

## 8. 閸欏倽鈧啳绁┃?

- **閺堝搫娅掓禍鍝勵劅**: 閵嗗odern Robotics閵? Kevin Lynch
- **濮濄儲鈧浇顫夐崚?*: 閵嗗たiped Locomotion閵?
- **ZMP閻炲棜顔?*: Vukobratovi鑶? M. (1972)
- **PID鐠嬪啩绱?*: Ziegler-Nichols閺傝纭?

---

> 棣冩寱 **瀵ら缚顔?*: 娴犲海鐣濋崡鏇炵磻婵绱濋崗鍫濈暚閸?閼奉亞鏁辨惔锔侥侀崹瀣畱楠炲疇銆€閹貉冨煑閿涘苯鍟€闁劖顒炴晶鐐插閸忓疇濡崪灞肩炊閹扮喎娅掗妴鍌涚槨濞ｈ濮炴稉鈧稉顏勫閼充粙鍏樼憰浣稿帠閸掑棙绁寸拠鏇樷偓?
