extends RigidBody3D

## Quadruped Robot Controller
## 四足机器人控制器

# 关节引用
var joints = {
	"FL": {"hip": null, "thigh": null, "shin": null},
	"FR": {"hip": null, "thigh": null, "shin": null},
	"RL": {"hip": null, "thigh": null, "shin": null},
	"RR": {"hip": null, "thigh": null, "shin": null}
}

# 控制参数
var joint_targets = []
var joint_velocities = []

func _ready():
	# 初始化关节引用
	_initialize_joints()
	
	# 设置物理参数
	mass = 12.0
	gravity_scale = 1.0
	
	print("Quadruped Robot initialized")
	print("Total joints: ", joints.size() * 3)

func _initialize_joints():
	"""初始化所有关节的引用"""
	var leg_names = ["FL", "FR", "RL", "RR"]
	
	for leg in leg_names:
		# Hip joint
		var hip_path = leg + "_Hip"
		joints[leg]["hip"] = get_node(hip_path)
		
		# Thigh joint
		var thigh_path = leg + "_Hip/" + leg + "_Thigh/" + leg + "_Thigh_Joint"
		joints[leg]["thigh"] = get_node(thigh_path)
		
		# Shin would be at the end, but for now we track the thigh joint
	
	# 初始化目标数组
	joint_targets.resize(12)
	joint_velocities.resize(12)
	for i in range(12):
		joint_targets[i] = 0.0
		joint_velocities[i] = 0.0

func _physics_process(delta):
	"""物理更新 - 应用关节控制"""
	_apply_joint_control(delta)
	_update_sensors()

func _apply_joint_control(delta):
	"""应用关节控制"""
	var joint_index = 0
	
	for leg in ["FL", "FR", "RL", "RR"]:
		# Hip control
		if joints[leg]["hip"]:
			var hip_joint = joints[leg]["hip"] as HingeJoint3D
			var target_angle = joint_targets[joint_index]
			_control_joint(hip_joint, target_angle, delta)
		joint_index += 1
		
		# Thigh control
		if joints[leg]["thigh"]:
			var thigh_joint = joints[leg]["thigh"] as HingeJoint3D
			var target_angle = joint_targets[joint_index]
			_control_joint(thigh_joint, target_angle, delta)
		joint_index += 1
		
		# Shin control
		joint_index += 1 # Placeholder for shin

func _control_joint(joint: HingeJoint3D, target_angle: float, delta: float):
	"""PD 控制单个关节"""
	if not joint or not joint.motor_enabled:
		return
	
	# 获取当前角度 (简化 - 实际需要从物理引擎读取)
	var current_angle = 0.0 # TODO: 从 joint 获取实际角度
	
	# PD 控制
	var kp = 50.0 # 比例增益
	var kd = 5.0 # 微分增益
	
	var error = target_angle - current_angle
	var torque = kp * error - kd * 0.0 # TODO: 添加速度反馈
	
	# 应用扭矩（通过motor）
	joint.set("motor/target_velocity", torque)

func _update_sensors():
	"""更新传感器读数"""
	# IMU 数据
	var angular_velocity = get_angular_velocity()
	var linear_velocity = get_linear_velocity()
	
	# TODO: 发送到 Python API

# ============ Python API 接口 ============

func set_joint_targets(targets: Array):
	"""设置所有关节的目标位置"""
	if targets.size() != 12:
		push_error("Invalid joint targets size: " + str(targets.size()))
		return
	
	joint_targets = targets

func get_state() -> Dictionary:
	"""获取机器人状态用于 RL"""
	var state = {
		"position": global_position,
		"rotation": rotation,
		"linear_velocity": linear_velocity,
		"angular_velocity": angular_velocity,
		"joint_angles": _get_joint_angles(),
		"joint_velocities": joint_velocities
	}
	return state

func _get_joint_angles() -> Array:
	"""获取所有关节角度"""
	var angles = []
	angles.resize(12)
	
	# TODO: 从物理引擎读取实际关节角度
	for i in range(12):
		angles[i] = 0.0
	
	return angles

func apply_action(action: Array):
	"""应用 RL 动作"""
	set_joint_targets(action)

func reset_robot():
	"""重置机器人到初始状态"""
	global_position = Vector3(0, 0.5, 0)
	rotation = Vector3.ZERO
	linear_velocity = Vector3.ZERO
	angular_velocity = Vector3.ZERO
	
	# 重置关节
	for i in range(12):
		joint_targets[i] = 0.0
		joint_velocities[i] = 0.0
