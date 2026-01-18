# ground_tilt_controller.gd
# 地面倾斜控制器 - 动态调节地面角度
extends Node3D

class_name GroundTiltController

@export var ground: StaticBody3D
@export var tilt_speed: float = 1.0 # 倾斜速度 (度/秒)
@export var max_tilt_angle: float = 30.0 # 最大倾斜角度

var current_tilt: Vector2 = Vector2.ZERO # (pitch, roll)
var target_tilt: Vector2 = Vector2.ZERO

signal tilt_changed(pitch: float, roll: float)

func _ready():
	if not ground:
		push_warning("Ground body not assigned!")
	print("🏔️ GroundTiltController initialized")

func _process(delta):
	if current_tilt != target_tilt:
		# 平滑过渡到目标倾斜角度
		current_tilt = current_tilt.lerp(target_tilt, tilt_speed * delta)
		_apply_tilt()

## 设置倾斜角度 (度)
func set_tilt(pitch: float, roll: float):
	target_tilt = Vector2(
		clamp(pitch, -max_tilt_angle, max_tilt_angle),
		clamp(roll, -max_tilt_angle, max_tilt_angle)
	)

## 设置 pitch (前后倾斜)
func set_pitch(degrees: float):
	set_tilt(degrees, current_tilt.y)

## 设置 roll (左右倾斜)  
func set_roll(degrees: float):
	set_tilt(current_tilt.x, degrees)

## 应用倾斜到地面
func _apply_tilt():
	if not ground:
		return
	
	var pitch_rad = deg_to_rad(current_tilt.x)
	var roll_rad = deg_to_rad(current_tilt.y)
	
	# 创建旋转变换
	var rotation = Basis()
	rotation = rotation.rotated(Vector3.RIGHT, pitch_rad)
	rotation = rotation.rotated(Vector3.FORWARD, roll_rad)
	
	ground.transform.basis = rotation
	
	tilt_changed.emit(current_tilt.x, current_tilt.y)

## 重置到水平
func reset_tilt():
	set_tilt(0.0, 0.0)

## 随机倾斜 (域随机化)
func randomize_tilt(max_angle: float = 15.0):
	var random_pitch = randf_range(-max_angle, max_angle)
	var random_roll = randf_range(-max_angle, max_angle)
	set_tilt(random_pitch, random_roll)

## 震动效果 (短暂的随机倾斜)
func apply_shake(intensity: float = 5.0, duration: float = 0.5):
	var original_tilt = target_tilt
	randomize_tilt(intensity)
	await get_tree().create_timer(duration).timeout
	target_tilt = original_tilt

## 获取当前倾斜信息
func get_tilt_info() -> Dictionary:
	return {
		"pitch": current_tilt.x,
		"roll": current_tilt.y,
		"tilt_magnitude": current_tilt.length()
	}

## 模拟地震
func simulate_earthquake(duration: float = 5.0, intensity: float = 10.0):
	var elapsed = 0.0
	while elapsed < duration:
		randomize_tilt(intensity * randf_range(0.5, 1.0))
		await get_tree().create_timer(randf_range(0.1, 0.3)).timeout
		elapsed + 0.2
	reset_tilt()
