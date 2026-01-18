extends RefCounted
## PID控制器
## 比例-积分-微分控制器，用于系统闭环控制

class_name PIDController

# PID参数
var kp: float # 比例增益 (Proportional)
var ki: float # 积分增益 (Integral)
var kd: float # 微分增益 (Derivative)

# 内部状态
var integral: float = 0.0 # 积分累积
var last_error: float = 0.0 # 上一次误差
var last_time: float = 0.0 # 上一次更新时间

# 限制参数
var output_min: float = - INF # 输出下限
var output_max: float = INF # 输出上限
var integral_min: float = - INF # 积分下限（防止积分饱和）
var integral_max: float = INF # 积分上限

# 统计信息
var iteration_count: int = 0


func _init(p: float = 1.0, i: float = 0.0, d: float = 0.0):
	"""
	初始化PID控制器
	p: 比例增益
	i: 积分增益
	d: 微分增益
	"""
	kp = p
	ki = i
	kd = d
	last_time = Time.get_ticks_msec() / 1000.0


func compute(setpoint: float, measured_value: float, current_time: float = -1.0) -> float:
	"""
	计算PID输出
	setpoint: 目标值
	measured_value: 当前测量值
	current_time: 当前时间（秒），如果为-1则自动获取
	"""
	# 自动获取时间
	if current_time < 0:
		current_time = Time.get_ticks_msec() / 1000.0
	
	# 计算时间增量
	var dt = current_time - last_time
	if dt <= 0:
		dt = 0.001 # 防止除零
	
	# 计算误差
	var error = setpoint - measured_value
	
	# 比例项
	var p_term = kp * error
	
	# 积分项（梯形积分）
	integral += error * dt
	integral = clamp(integral, integral_min, integral_max) # 防止积分饱和
	var i_term = ki * integral
	
	# 微分项（使用误差的变化率）
	var derivative = (error - last_error) / dt
	var d_term = kd * derivative
	
	# 总输出
	var output = p_term + i_term + d_term
	
	# 限制输出范围
	output = clamp(output, output_min, output_max)
	
	# 更新状态
	last_error = error
	last_time = current_time
	iteration_count += 1
	
	return output


func compute_error(error: float, dt: float) -> float:
	"""
	直接使用误差计算（用于已知误差的情况）
	error: 误差值 (setpoint - measured_value)
	dt: 时间增量
	"""
	# 比例项
	var p_term = kp * error
	
	# 积分项
	integral += error * dt
	integral = clamp(integral, integral_min, integral_max)
	var i_term = ki * integral
	
	# 微分项
	var derivative = (error - last_error) / dt if dt > 0 else 0.0
	var d_term = kd * derivative
	
	# 总输出
	var output = p_term + i_term + d_term
	output = clamp(output, output_min, output_max)
	
	last_error = error
	iteration_count += 1
	
	return output


func set_limits(out_min: float, out_max: float):
	"""设置输出限制"""
	output_min = out_min
	output_max = out_max


func set_integral_limits(int_min: float, int_max: float):
	"""设置积分限制（防止积分饱和）"""
	integral_min = int_min
	integral_max = int_max


func reset():
	"""重置PID状态"""
	integral = 0.0
	last_error = 0.0
	last_time = Time.get_ticks_msec() / 1000.0
	iteration_count = 0


func set_tunings(p: float, i: float, d: float):
	"""动态调整PID参数"""
	kp = p
	ki = i
	kd = d


func get_terms() -> Dictionary:
	"""获取PID各项的当前值（用于调试）"""
	var error = last_error
	return {
		"p_term": kp * error,
		"i_term": ki * integral,
		"d_term": kd * (error - last_error) if last_time > 0 else 0.0,
		"integral": integral,
		"error": error
	}


func print_status():
	"""打印PID状态（调试用）"""
	var terms = get_terms()
	print("PID状态 [Kp=%.2f, Ki=%.2f, Kd=%.2f]" % [kp, ki, kd])
	print("  误差: %.3f" % terms["error"])
	print("  P项: %.3f" % terms["p_term"])
	print("  I项: %.3f (积分=%.3f)" % [terms["i_term"], terms["integral"]])
	print("  D项: %.3f" % terms["d_term"])
	print("  迭代: %d" % iteration_count)
