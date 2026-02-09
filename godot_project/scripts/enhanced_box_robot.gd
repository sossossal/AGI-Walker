# enhanced_box_robot.gd  
# 增强的机器人控制器 - 集成零件库和环境系统
extends Node3D

## 零件和环境系统
@onready var parts_library = preload("res://addons/robot_sim_toolkit/scripts/parts_manager.gd").new()
@onready var env_controller = get_node_or_null("/root/EnvironmentController")

## 机器人部件引用
@onready var torso: RigidBody3D = $Torso
@onready var left_thigh: RigidBody3D = $LeftThigh
@onready var right_thigh: RigidBody3D = $RightThigh

@onready var hip_left: HingeJoint3D = $HipLeft
@onready var hip_right: HingeJoint3D = $HipRight

## 传感器数据
var sensor_data: Dictionary = {}

## 零件配置
var robot_parts_config: Array = []

func _ready():
	print("🤖 Enhanced Robot initialized")
	_initialize_parts_library()
	_initialize_parts_library()
	apply_physics_config() # 应用初始物理配置
	_setup_sensors()
	_setup_sensors()
	
	if env_controller:
		env_controller.environment_changed.connect(_on_environment_changed)

## 初始化零件库
func _initialize_parts_library():
	# 加载零件数据
	parts_library.load_parts_database("res://parts_library")
	
	# 配置机器人使用的零件
	robot_parts_config = [
		{"part_id": "dynamixel_xl430_w250", "joint": "hip_left"},
		{"part_id": "dynamixel_xl430_w250", "joint": "hip_right"},
		{"part_id": "bosch_bno055", "location": "torso"}
	]
	
	# 应用零件规格到关节
	_apply_part_specs()

## 应用零件规格
func _apply_part_specs():
	for part_config in robot_parts_config:
		var part_id = part_config["part_id"]
		var part_data = parts_library.get_part(part_id)
		
		if part_data:
			var category = part_data.get("category", "")
			
			if category == "actuator_servo":
				_apply_motor_specs(part_config["joint"], part_data)
			elif category == "sensor_imu":
				_apply_sensor_specs(part_config["location"], part_data)

## 应用电机规格到关节
func _apply_motor_specs(joint_name: String, part_data: Dictionary):
	var joint: HingeJoint3D = get_node_or_null(joint_name.capitalize())
	if not joint:
		return
	
	var specs = part_data.get("specifications", {})
	
	# 应用扭矩限制
	var stall_torque = specs.get("stall_torque", 1.0)
	joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, stall_torque)
	
	# 存储零件数据为元数据
	joint.set_meta("part_id", part_data["part_id"])
	joint.set_meta("stall_torque", stall_torque)
	joint.set_meta("no_load_speed", specs.get("no_load_speed", 60.0))
	
	print("  ✅ Applied motor specs to ", joint_name, ": ", stall_torque, " N·m")

## 应用传感器规格
func _apply_sensor_specs(location: String, part_data: Dictionary):
	# 存储传感器配置
	sensor_data[location] = {
		"part_id": part_data["part_id"],
		"specifications": part_data.get("specifications", {})
	}
	print("  ✅ Configured sensor at ", location)

## 设置传感器
func _setup_sensors():
	# IMU 传感器模拟
	sensor_data["imu"] = {
		"orientation": Vector3.ZERO,
		"angular_velocity": Vector3.ZERO,
		"linear_acceleration": Vector3.ZERO
	}

func _physics_process(delta):
	_update_sensors(delta)
	_apply_environmental_effects(delta)

## 更新传感器数据
func _update_sensors(delta):
	if not torso:
		return
	
	# IMU 数据
	var basis = torso.global_transform.basis
	var euler = basis.get_euler()
	
	sensor_data["imu"]["orientation"] = Vector3(
		rad_to_deg(euler.x), # roll
		rad_to_deg(euler.y), # pitch
		rad_to_deg(euler.z) # yaw
	)
	
	sensor_data["imu"]["angular_velocity"] = torso.angular_velocity
	sensor_data["imu"]["linear_acceleration"] = torso.linear_velocity / delta

## 应用环境效应
func _apply_environmental_effects(delta):
	if not env_controller or not torso:
		return
	
	# 1. 空气阻力
	if env_controller.air_density > 0.01:
		var cross_section = 0.3 # m² (估算)
		var air_drag = env_controller.calculate_air_drag(
			torso.linear_velocity,
			cross_section,
			0.47 # 阻力系数
		)
		torso.apply_central_force(air_drag)
	
	# 2. 温度影响 (影响关节摩擦)
	var temp_factor = env_controller.get_temperature_factor()
	# 可以应用到关节阻尼等

## 环境变化回调
func _on_environment_changed(param_name: String, new_value: float):
	print("🌍 Environment changed: ", param_name, " = ", new_value)
	
	# 根据环境变化调整机器人行为
	if param_name == "gravity":
		# 重力变化可能需要调整步态
		pass

## 获取传感器数据 (用于TCP通信)
func get_sensor_data() -> Dictionary:
	return sensor_data.duplicate(true)

## 设置电机目标 (从TCP接收)
func set_motor_targets(targets: Dictionary):
	for joint_name in targets.keys():
		var joint = get_node_or_null(joint_name.capitalize())
		if joint and joint is HingeJoint3D:
			var target_angle_deg = targets[joint_name]
			var target_angle_rad = deg_to_rad(target_angle_deg)
			# 使用PID或其他控制方法设置目标
			joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, target_angle_rad)

## 获取机器人状态
func get_robot_state() -> Dictionary:
	return {
		"sensors": get_sensor_data(),
		"parts_config": robot_parts_config,
		"environment": env_controller.get_environment_info() if env_controller else {},
		"torso_height": torso.global_position.y if torso else 0.0
	}

func apply_physics_config():
	"""应用物理配置"""
	if not is_inside_tree(): return
	print("🔧 Applying Physics Config...")
	
	# 应用刚体参数
	if torso: PhysicsConfig.apply_to_torso(torso)
	if left_thigh: PhysicsConfig.apply_to_leg(left_thigh)
	if right_thigh: PhysicsConfig.apply_to_leg(right_thigh)
	
	# 应用关节参数
	if hip_left: PhysicsConfig.apply_to_hip_joint(hip_left)
	if hip_right: PhysicsConfig.apply_to_hip_joint(hip_right)
	
	# 如果有零件库的特定参数，覆盖PhysicsConfig的通用参数
	# (例如特定电机扭矩)
	_apply_part_specs()
