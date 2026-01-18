extends Node
## 平衡控制器
## 使用PID控制器维持机器人姿态平衡

@export var enabled := true # 是否启用平衡控制
@export var debug_mode := false # 调试模式

# PID控制器
var roll_pid: PIDController # Roll轴（左右倾斜）
var pitch_pid: PIDController # Pitch轴（前后倾斜）

# 目标姿态（度）
var target_roll := 0.0
var target_pitch := 0.0

# 引用
@onready var robot = get_node("/root/Main/Robot")

# 统计
var balance_time := 0.0 # 平衡持续时间


func _ready():
	_initialize_pids()
	print("✅ 平衡控制器已初始化")


func _initialize_pids():
	"""初始化PID控制器"""
	# Roll控制器（左右平衡）
	roll_pid = PIDController.new(
		8.0, # Kp - 比例增益（快速响应）
		0.5, # Ki - 积分增益（消除稳态误差）
		3.0 # Kd - 微分增益（减少震荡）
	)
	roll_pid.set_limits(-90, 90) # 输出限制（角度）
	roll_pid.set_integral_limits(-20, 20) # 积分限制
	
	# Pitch控制器（前后平衡）
	pitch_pid = PIDController.new(
		8.0, # Kp
		0.5, # Ki
		3.0 # Kd
	)
	pitch_pid.set_limits(-90, 90)
	pitch_pid.set_integral_limits(-20, 20)


func _physics_process(delta):
	"""每个物理帧更新平衡控制"""
	if not enabled or not robot or not robot.is_scene_ready:
		return
	
	# 获取传感器数据
	var sensor_data = robot.get_sensor_data()
	
	# 计算平衡指令
	var balance_commands = compute_balance(sensor_data, delta)
	
	# 应用到机器人（融合到现有目标角度）
	apply_balance_commands(balance_commands)
	
	# 更新统计
	_update_statistics(sensor_data)


func compute_balance(sensor_data: Dictionary, dt: float) -> Dictionary:
	"""
	计算平衡所需的关节角度修正
	返回: {"hip_left": angle, "hip_right": angle}
	"""
	# 获取当前姿态
	var orient = sensor_data['sensors']['imu']['orient']
	var current_roll = orient[0] # Roll（左右倾斜）
	var current_pitch = orient[1] # Pitch（前后倾斜）
	
	# 计算误差
	var roll_error = target_roll - current_roll
	var pitch_error = target_pitch - current_pitch
	
	# PID计算修正量
	var roll_correction = roll_pid.compute_error(roll_error, dt)
	var pitch_correction = pitch_pid.compute_error(pitch_error, dt)
	
	# 调试输出
	if debug_mode and Engine.get_physics_frames() % 30 == 0: # 每秒一次
		print("平衡控制 | Roll: %.1f°→%.1f° | Pitch: %.1f°→%.1f°" %
			[current_roll, roll_correction, current_pitch, pitch_correction])
	
	# 转换为关节指令
	# Roll修正：左右腿反向调整
	# Pitch修正：左右腿同向调整
	return {
		"hip_left": pitch_correction + roll_correction,
		"hip_right": pitch_correction - roll_correction
	}


func apply_balance_commands(commands: Dictionary):
	"""应用平衡指令到机器人"""
	if not robot:
		return
	
	# 融合到机器人的目标角度（叠加而不是覆盖）
	for joint_name in commands:
		if joint_name in robot.target_angles:
			robot.target_angles[joint_name] += commands[joint_name]


func set_target_posture(roll: float, pitch: float):
	"""设置目标姿态"""
	target_roll = roll
	target_pitch = pitch
	print("目标姿态更新: Roll=%.1f°, Pitch=%.1f°" % [roll, pitch])


func tune_pid(axis: String, kp: float, ki: float, kd: float):
	"""动态调整PID参数"""
	match axis.to_lower():
		"roll":
			roll_pid.set_tunings(kp, ki, kd)
			print("Roll PID已调整: Kp=%.2f, Ki=%.2f, Kd=%.2f" % [kp, ki, kd])
		"pitch":
			pitch_pid.set_tunings(kp, ki, kd)
			print("Pitch PID已调整: Kp=%.2f, Ki=%.2f, Kd=%.2f" % [kp, ki, kd])


func reset():
	"""重置PID状态"""
	roll_pid.reset()
	pitch_pid.reset()
	balance_time = 0.0
	print("平衡控制器已重置")


func _update_statistics(sensor_data: Dictionary):
	"""更新统计信息"""
	var orient = sensor_data['sensors']['imu']['orient']
	var roll = abs(orient[0])
	var pitch = abs(orient[1])
	
	# 如果姿态接近直立，增加平衡时间
	if roll < 5.0 and pitch < 5.0:
		balance_time += get_physics_process_delta_time()


func get_balance_quality() -> float:
	"""
	获取平衡质量评分 (0-100)
	基于姿态偏差和平衡持续时间
	"""
	if not robot or not robot.is_scene_ready:
		return 0.0
	
	var sensor_data = robot.get_sensor_data()
	var orient = sensor_data['sensors']['imu']['orient']
	
	var roll_deviation = abs(orient[0])
	var pitch_deviation = abs(orient[1])
	var total_deviation = roll_deviation + pitch_deviation
	
	# 偏差越小，分数越高
	var deviation_score = max(0, 100 - total_deviation * 2)
	
	return deviation_score


func print_debug_info():
	"""打印调试信息"""
	print("\n" + "=" * 50)
	print("平衡控制器状态")
	print("=" * 50)
	print("平衡时间: %.1fs" % balance_time)
	print("平衡质量: %.1f/100" % get_balance_quality())
	print("\nRoll PID:")
	roll_pid.print_status()
	print("\nPitch PID:")
	pitch_pid.print_status()
	print("=" * 50 + "\n")
