extends Node3D
## 简单盒子机器人
## 包含躯干和两条腿，使用HingeJoint连接

# 机器人组件引用（使用get_node_or_null避免崩溃）
@onready var torso: RigidBody3D = get_node_or_null("Torso")
@onready var left_leg: RigidBody3D = get_node_or_null("LeftLeg")
@onready var right_leg: RigidBody3D = get_node_or_null("RightLeg")
@onready var hip_left: HingeJoint3D = get_node_or_null("HipLeft")
@onready var hip_right: HingeJoint3D = get_node_or_null("HipRight")

# 关节角度存储（用于传感器）
var joint_angles := {
	"hip_left": 0.0,
	"hip_right": 0.0
}

# 关节目标角度（来自AI控制）
var target_angles := {
	"hip_left": 0.0,
	"hip_right": 0.0
}

# 电机参数
const MOTOR_FORCE = 500.0 # 扭矩大小
const MOTOR_SPEED = 5.0 # 响应速度

# 场景是否完整
var is_scene_ready := false


func _ready():
	print("🤖 盒子机器人初始化中...")
	
	# 检查场景是否完整
	if not _validate_scene():
		push_error("❌ 机器人场景不完整！请按照 SCENE_SETUP_GUIDE.md 创建场景")
		print("\n" + "=" * 60)
		print("⚠️  场景搭建提示")
		print("=" * 60)
		print("请在Godot编辑器中为Robot节点添加以下子节点:")
		print("  - Torso (RigidBody3D)")
		print("  - LeftLeg (RigidBody3D)")
		print("  - RightLeg (RigidBody3D)")
		print("  - HipLeft (HingeJoint3D)")
		print("  - HipRight (HingeJoint3D)")
		print("\n详细步骤请参考: godot_project/SCENE_SETUP_GUIDE.md")
		print("=" * 60 + "\n")
		return
	
	is_scene_ready = true
	print("✅ 机器人场景验证通过")
	_setup_motors()


func _validate_scene() -> bool:
	"""验证场景节点是否完整"""
	var missing_nodes = []
	
	if not torso:
		missing_nodes.append("Torso")
	if not left_leg:
		missing_nodes.append("LeftLeg")
	if not right_leg:
		missing_nodes.append("RightLeg")
	if not hip_left:
		missing_nodes.append("HipLeft")
	if not hip_right:
		missing_nodes.append("HipRight")
	
	if missing_nodes.size() > 0:
		print("❌ 缺少节点: " + ", ".join(missing_nodes))
		return false
	
	return true


func _setup_motors():
	"""配置关节电机"""
	# 左髋关节
	hip_left.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
	hip_left.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, MOTOR_FORCE)
	
	# 右髋关节
	hip_right.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
	hip_right.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, MOTOR_FORCE)


func _physics_process(delta):
	"""物理步进 - 更新电机控制"""
	if not is_scene_ready:
		return
	
	_update_motors(delta)
	_update_joint_angles()


func _update_motors(_delta):
	"""根据目标角度更新电机"""
	# 左髋
	var left_error = target_angles["hip_left"] - joint_angles["hip_left"]
	var left_velocity = left_error * MOTOR_SPEED
	hip_left.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, left_velocity)
	
	# 右髋
	var right_error = target_angles["hip_right"] - joint_angles["hip_right"]
	var right_velocity = right_error * MOTOR_SPEED
	hip_right.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, right_velocity)


func _update_joint_angles():
	"""更新关节角度（从物理引擎读取）"""
	# 注意: Godot的HingeJoint没有直接获取角度的API
	# 这里使用近似方法：通过刚体的相对旋转计算
	
	# 左腿相对躯干的旋转
	var left_relative = torso.global_transform.basis.inverse() * left_leg.global_transform.basis
	joint_angles["hip_left"] = rad_to_deg(left_relative.get_euler().z)
	
	# 右腿相对躯干的旋转
	var right_relative = torso.global_transform.basis.inverse() * right_leg.global_transform.basis
	joint_angles["hip_right"] = rad_to_deg(right_relative.get_euler().z)


func get_sensor_data() -> Dictionary:
	"""收集所有传感器数据"""
	if not is_scene_ready:
		# 返回空数据避免崩溃
		return {
			"timestamp": Time.get_ticks_msec() / 1000.0,
			"sensors": {
				"imu": {"accel": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0], "orient": [0.0, 0.0, 0.0]},
				"joints": {"hip_left": {"angle": 0.0, "velocity": 0.0}, "hip_right": {"angle": 0.0, "velocity": 0.0}},
				"contacts": {"foot_left": false, "foot_right": false}
			},
			"torso_height": 0.0
		}
	
	return {
		"timestamp": Time.get_ticks_msec() / 1000.0,
		"sensors": {
			"imu": _get_imu_data(),
			"joints": _get_joint_data(),
			"contacts": _get_contact_data()
		},
		"torso_height": torso.global_position.y
	}


func _get_imu_data() -> Dictionary:
	"""IMU传感器数据（加速度、陀螺仪、姿态）"""
	var euler = torso.global_transform.basis.get_euler()
	
	return {
		"accel": [
			torso.linear_velocity.x,
			torso.linear_velocity.y - 9.8, # 减去重力
			torso.linear_velocity.z
		],
		"gyro": [
			torso.angular_velocity.x,
			torso.angular_velocity.y,
			torso.angular_velocity.z
		],
		"orient": [
			rad_to_deg(euler.x), # roll
			rad_to_deg(euler.y), # pitch
			rad_to_deg(euler.z) # yaw
		]
	}


func _get_joint_data() -> Dictionary:
	"""关节位置和速度数据"""
	return {
		"hip_left": {
			"angle": joint_angles["hip_left"],
			"velocity": 0.0 # TODO: 通过差分计算
		},
		"hip_right": {
			"angle": joint_angles["hip_right"],
			"velocity": 0.0
		}
	}


func _get_contact_data() -> Dictionary:
	"""接触传感器数据"""
	# 简化版本：通过检测脚的高度判断是否接地
	var left_foot_pos = left_leg.global_position
	var right_foot_pos = right_leg.global_position
	
	return {
		"foot_left": left_foot_pos.y < 0.3, # 接近地面
		"foot_right": right_foot_pos.y < 0.3
	}


func apply_motor_commands(command: Dictionary):
	"""应用来自AI的电机指令"""
	if command.has("motors"):
		var motors = command["motors"]
		
		if motors.has("hip_left"):
			target_angles["hip_left"] = clamp(motors["hip_left"], -45, 90)
		
		if motors.has("hip_right"):
			target_angles["hip_right"] = clamp(motors["hip_right"], -45, 90)


func reset_pose():
	"""重置机器人到初始姿态"""
	target_angles["hip_left"] = 0.0
	target_angles["hip_right"] = 0.0
	
	# 重置物理状态
	torso.linear_velocity = Vector3.ZERO
	torso.angular_velocity = Vector3.ZERO
	left_leg.linear_velocity = Vector3.ZERO
	right_leg.linear_velocity = Vector3.ZERO

func apply_sim_params(params: Dictionary):
	"""应用域随机化参数 (Sim2Real)"""
	if not is_scene_ready:
		return
		
	# 1. 质量缩放
	if params.has("mass_scale"):
		var scale = float(params["mass_scale"])
		torso.mass = 5.0 * scale
		left_leg.mass = 1.0 * scale
		right_leg.mass = 1.0 * scale
		
	# 2. 摩擦力缩放
	if params.has("friction_scale"):
		var f_scale = float(params["friction_scale"])
		for body in [torso, left_leg, right_leg]:
			if body.physics_material_override:
				body.physics_material_override.friction = 1.0 * f_scale
			else:
				var mat = PhysicsMaterial.new()
				mat.friction = 1.0 * f_scale
				body.physics_material_override = mat

	# 3. 电机强度缩放
	if params.has("motor_strength"):
		var m_scale = float(params["motor_strength"])
		var force = MOTOR_FORCE * m_scale
		hip_left.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, force)
		hip_right.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, force)
