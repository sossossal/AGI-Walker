# test_enhanced_motor.gd
# 测试 EnhancedMotorJoint 的示例脚本

extends Node3D

# 注意: 如果 EnhancedMotorJoint 未正确注册,先使用 HingeJoint3D
@onready var motor_joint: HingeJoint3D = $EnhancedMotorJoint

func _ready():
	print("=== EnhancedMotorJoint Test ===")
	
	# 检查节点是否存在
	if motor_joint == null:
		push_error("Motor joint not found!")
		return
	
	print("Node type: ", motor_joint.get_class())
	
	# 检查是否是 EnhancedMotorJoint 类型
	if motor_joint.has_method("set_motor_specs"):
		print("✅ EnhancedMotorJoint detected!")
		
		# 设置电机参数
		motor_joint.set_motor_specs(
			2.0, # stall_torque (N·m)
			10.0, # no_load_speed (rad/s)
			0.001 # rotor_inertia (kg·m²)
		)
		
		# 设置摩擦参数
		motor_joint.set_friction_params(
			0.1, # static_friction
			0.05, # dynamic_friction
			0.01 # viscous_damping
		)
		
		# 设置热参数
		motor_joint.set_thermal_params(
			5.0, # thermal_resistance (K/W)
			30.0, # thermal_time_constant (s)
			120.0 # max_winding_temp (°C)
		)
		
		# 启用各种模型
		motor_joint.set_enable_speed_torque_curve(true)
		motor_joint.set_enable_friction_model(true)
		motor_joint.set_enable_thermal_model(true)
		
		print("✅ Motor specs configured")
		print("Stall Torque: ", motor_joint.get_stall_torque(), " N·m")
		print("No-Load Speed: ", motor_joint.get_no_load_speed(), " rad/s")
	else:
		push_warning("⚠️ EnhancedMotorJoint methods not available. Using standard HingeJoint3D.")
		push_warning("This means the GDExtension did not load correctly.")
		push_warning("Please check:")
		push_warning("  1. Plugin is enabled in Project Settings -> Plugins")
		push_warning("  2. DLL file exists: addons/robot_sim_toolkit/bin/robotparts.windows.x86_64.dll")
		push_warning("  3. Console shows: '✅ Robot Simulation Toolkit GDExtension loaded'")

func _process(_delta):
	if motor_joint == null:
		return
	
	# 检查是否有 EnhancedMotorJoint 方法
	if not motor_joint.has_method("set_target_velocity_rad"):
		return
	
	# 设置目标速度(正弦波)
	var target_vel = sin(Time.get_ticks_msec() / 1000.0) * 5.0
	motor_joint.set_target_velocity_rad(target_vel)
	
	# 每秒打印一次状态
	if int(Time.get_ticks_msec()) % 1000 < 16:
		print("--- Motor Status ---")
		print("Temperature: ", motor_joint.get_temperature(), " °C")
		print("Current: ", motor_joint.get_current_draw(), " A")
		print("Torque: ", motor_joint.get_current_torque(), " N·m")
		print("Velocity: ", motor_joint.get_current_velocity(), " rad/s")
		print("Efficiency: ", motor_joint.get_efficiency() * 100, " %")
		print("Power: ", motor_joint.get_power_consumption(), " W")
		
		if motor_joint.is_overloaded():
			push_warning("⚠️ Motor overloaded!")
		if motor_joint.is_overheating():
			push_warning("🔥 Motor overheating!")
