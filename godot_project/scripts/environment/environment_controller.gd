# environment_controller.gd
# 物理环境控制器 - 动态调节重力、空气密度、温度等参数
extends Node

class_name EnvironmentController

## 环境参数
var gravity: float = 9.81  # m/s² (地球标准重力)
var air_density: float = 1.225  # kg/m³ (海平面标准)
var temperature: float = 25.0  # °C
var wind_velocity: Vector3 = Vector3.ZERO  # m/s
var ground_friction: float = 0.8  # 地面摩擦系数

## 环境预设
const PRESETS = {
	"earth": {
		"gravity": 9.81,
		"air_density": 1.225,
		"temperature": 25.0,
		"name": "地球"
	},
	"moon": {
		"gravity": 1.62,
		"air_density": 0.0,
		"temperature": -20.0,
		"name": "月球"
	},
	"mars": {
		"gravity": 3.71,
		"air_density": 0.02,
		"temperature": -60.0,
		"name": "火星"
	},
	"jupiter": {
		"gravity": 24.79,
		"air_density": 0.16,  # 云层顶部近似
		"temperature": -110.0,
		"name": "木星"
	},
	"custom": {
		"gravity": 9.81,
		"air_density": 1.225,
		"temperature": 25.0,
		"name": "自定义"
	}
}

## 信号
signal environment_changed(param_name: String, new_value: float)
signal preset_loaded(preset_name: String)

func _ready():
	print("🌍 EnvironmentController initialized")
	apply_environment_parameters()

## 设置重力
func set_gravity(value: float) -> void:
	gravity = value
	apply_gravity()
	environment_changed.emit("gravity", value)

## 设置空气密度
func set_air_density(value: float) -> void:
	air_density = value
	environment_changed.emit("air_density", value)

## 设置温度
func set_temperature(value: float) -> void:
	temperature = value
	environment_changed.emit("temperature", value)

## 设置风力
func set_wind(velocity: Vector3) -> void:
	wind_velocity = velocity
	environment_changed.emit("wind", velocity.length())

## 设置地面摩擦
func set_ground_friction(value: float) -> void:
	ground_friction = value
	environment_changed.emit("ground_friction", value)

## 应用重力到物理世界
func apply_gravity() -> void:
	var space_state = get_viewport().world_3d.direct_space_state
	if space_state:
		PhysicsServer3D.area_set_param(
			get_viewport().world_3d.space,
			PhysicsServer3D.AREA_PARAM_GRAVITY,
			gravity
		)
		print("✅ Gravity set to: ", gravity, " m/s²")

## 应用所有环境参数
func apply_environment_parameters() -> void:
	apply_gravity()
	print("🌡️ Temperature: ", temperature, "°C")
	print("💨 Air density: ", air_density, " kg/m³")
	print("🌪️ Wind: ", wind_velocity)

## 加载环境预设
func load_preset(preset_name: String) -> void:
	if not PRESETS.has(preset_name):
		push_warning("Unknown preset: " + preset_name)
		return
	
	var preset = PRESETS[preset_name]
	set_gravity(preset["gravity"])
	set_air_density(preset["air_density"])
	set_temperature(preset["temperature"])
	
	preset_loaded.emit(preset_name)
	print("🌍 Loaded environment preset: ", preset["name"])

## 获取当前环境信息
func get_environment_info() -> Dictionary:
	return {
		"gravity": gravity,
		"air_density": air_density,
		"temperature": temperature,
		"wind_velocity": wind_velocity,
		"ground_friction": ground_friction
	}

## 应用空气阻力（可被刚体调用）
func calculate_air_drag(velocity: Vector3, cross_section_area: float, drag_coefficient: float = 0.47) -> Vector3:
	"""
	计算空气阻力
	F_drag = 0.5 * ρ * v² * A * C_d
	
	参数:
	- velocity: 物体速度 (m/s)
	- cross_section_area: 横截面积 (m²)
	- drag_coefficient: 阻力系数 (球体约0.47)
	"""
	if air_density <= 0.0:
		return Vector3.ZERO
	
	var relative_velocity = velocity - wind_velocity
	var speed_squared = relative_velocity.length_squared()
	
	if speed_squared < 0.01:
		return Vector3.ZERO
	
	var drag_magnitude = 0.5 * air_density * speed_squared * cross_section_area * drag_coefficient
	var drag_direction = -relative_velocity.normalized()
	
	return drag_direction * drag_magnitude

## 应用温度影响（温度影响材料属性）
func get_temperature_factor() -> float:
	"""
	温度因子：影响摩擦系数等
	低温 → 摩擦增大
	高温 → 摩擦减小
	"""
	# 基准温度 20°C
	var temp_diff = temperature - 20.0
	# 每10°C变化约1%
	var factor = 1.0 - (temp_diff / 1000.0)
	return clamp(factor, 0.8, 1.2)

## 随机扰动（模拟环境不确定性）
func apply_random_disturbance(body: RigidBody3D, magnitude: float = 1.0) -> void:
	"""
	对刚体施加随机扰动力
	用于域随机化训练
	"""
	var random_force = Vector3(
		randf_range(-1.0, 1.0),
		randf_range(-0.5, 0.5),
		randf_range(-1.0, 1.0)
	).normalized() * magnitude
	
	body.apply_central_force(random_force)

## 导出配置到字典（用于保存/网络传输）
func to_dict() -> Dictionary:
	return {
		"gravity": gravity,
		"air_density": air_density,
		"temperature": temperature,
		"wind_velocity": {
			"x": wind_velocity.x,
			"y": wind_velocity.y,
			"z": wind_velocity.z
		},
		"ground_friction": ground_friction
	}

## 从字典加载配置
func from_dict(data: Dictionary) -> void:
	if data.has("gravity"):
		set_gravity(data["gravity"])
	if data.has("air_density"):
		set_air_density(data["air_density"])
	if data.has("temperature"):
		set_temperature(data["temperature"])
	if data.has("wind_velocity"):
		var wind_data = data["wind_velocity"]
		set_wind(Vector3(wind_data["x"], wind_data["y"], wind_data["z"]))
	if data.has("ground_friction"):
		set_ground_friction(data["ground_friction"])
