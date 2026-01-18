## parts_manager.gd
## 机器人零件库管理器
## 负责加载、验证和实例化各种机器人零件
class_name RobotPartsLibrary
extends Node

## 零件数据库（part_id -> part_data 字典）
var parts_database: Dictionary = {}

## 已加载的零件实例缓存
var loaded_instances: Dictionary = {}

## 零件库根目录
const PARTS_ROOT = "res://parts_library/"

## 支持的零件类别
enum PartCategory {
	ACTUATOR_SERVO,
	ACTUATOR_MOTOR,
	SENSOR_IMU,
	SENSOR_FORCE,
	STRUCTURE
}


func _ready():
	load_parts_database()


## 从 JSON 文件加载所有零件数据
func load_parts_database() -> void:
	print("🔧 开始加载零件库...")
	
	var categories = {
		"motors": ["dynamixel"],
		"sensors": ["imu"]
	}
	
	var total_loaded = 0
	
	for category in categories:
		for subcategory in categories[category]:
			var dir_path = PARTS_ROOT + category + "/" + subcategory
			var parts = _scan_json_files(dir_path)
			
			for part_file in parts:
				var part_data = _load_json_file(part_file)
				if part_data:
					var part_id = part_data.get("part_id", "")
					if part_id != "":
						parts_database[part_id] = part_data
						total_loaded += 1
						print("  ✓ 加载零件: ", part_id, " (", part_data.get("model", ""), ")")
	
	print("✅ 零件库加载完成，共 ", total_loaded, " 个零件")


## 扫描目录下的所有 JSON 文件
func _scan_json_files(dir_path: String) -> Array[String]:
	var result: Array[String] = []
	var dir = DirAccess.open(dir_path)
	
	if dir == null:
		push_warning("目录不存在: " + dir_path)
		return result
	
	dir.list_dir_begin()
	var file_name = dir.get_next()
	
	while file_name != "":
		if not dir.current_is_dir() and file_name.ends_with(".json"):
			result.append(dir_path + "/" + file_name)
		file_name = dir.get_next()
	
	dir.list_dir_end()
	return result


## 加载 JSON 文件
func _load_json_file(file_path: String) -> Dictionary:
	var file = FileAccess.open(file_path, FileAccess.READ)
	if file == null:
		push_error("无法打开文件: " + file_path)
		return {}
	
	var json_string = file.get_as_text()
	file.close()
	
	var json = JSON.new()
	var parse_result = json.parse(json_string)
	
	if parse_result != OK:
		push_error("JSON 解析失败: " + file_path)
		return {}
	
	return json.data


## 根据 part_id 获取零件数据
func get_part(part_id: String) -> Dictionary:
	return parts_database.get(part_id, {})


## 获取某个类别的所有零件
func get_parts_by_category(category: String) -> Array[Dictionary]:
	var result: Array[Dictionary] = []
	for part_id in parts_database:
		var part = parts_database[part_id]
		if part.get("category", "") == category:
			result.append(part)
	return result


## 获取某个制造商的所有零件
func get_parts_by_manufacturer(manufacturer: String) -> Array[Dictionary]:
	var result: Array[Dictionary] = []
	for part_id in parts_database:
		var part = parts_database[part_id]
		if part.get("manufacturer", "") == manufacturer:
			result.append(part)
	return result


## 创建电机实例
## 返回一个配置好物理参数的 RigidBody3D 节点
func create_motor_instance(part_id: String, parent: Node3D = null) -> Node3D:
	var part_data = get_part(part_id)
	
	if part_data.is_empty():
		push_error("零件未找到: " + part_id)
		return null
	
	# 创建刚体节点
	var motor_body = RigidBody3D.new()
	motor_body.name = part_data.get("model", part_id)
	
	# 设置质量
	var specs = part_data.get("specifications", {})
	motor_body.mass = specs.get("weight", 0.1)
	
	# 创建碰撞形状（简化为盒子）
	var collision = CollisionShape3D.new()
	var shape = BoxShape3D.new()
	
	var dimensions = specs.get("dimensions", [30, 50, 35]) # mm
	# 转换为米
	shape.size = Vector3(
		dimensions[0] / 1000.0,
		dimensions[2] / 1000.0, # 高度作为 Y
		dimensions[1] / 1000.0
	)
	
	collision.shape = shape
	motor_body.add_child(collision)
	
	# 添加视觉网格（立方体占位符）
	var mesh_instance = MeshInstance3D.new()
	var box_mesh = BoxMesh.new()
	box_mesh.size = shape.size
	mesh_instance.mesh = box_mesh
	motor_body.add_child(mesh_instance)
	
	# 存储零件数据为元数据
	motor_body.set_meta("part_id", part_id)
	motor_body.set_meta("part_data", part_data)
	motor_body.set_meta("stall_torque", specs.get("stall_torque", 1.0))
	motor_body.set_meta("no_load_speed", specs.get("no_load_speed", 60.0))
	motor_body.set_meta("friction_params", specs.get("friction", {}))
	motor_body.set_meta("thermal_params", specs.get("thermal", {}))
	
	# 添加到父节点
	if parent:
		parent.add_child(motor_body)
	
	# 缓存实例
	if not loaded_instances.has(part_id):
		loaded_instances[part_id] = []
	loaded_instances[part_id].append(motor_body)
	
	print("🔩 创建电机实例: ", part_id, " (质量: ", motor_body.mass, "kg)")
	
	return motor_body


## 创建关节连接（带电机）
## 将两个刚体用铰链关节连接，并应用电机参数
func create_motor_joint(
	part_id: String,
	body_a: RigidBody3D,
	body_b: RigidBody3D,
	axis: Vector3 = Vector3.RIGHT,
	local_pos_a: Vector3 = Vector3.ZERO,
	local_pos_b: Vector3 = Vector3.ZERO
) -> HingeJoint3D:
	var part_data = get_part(part_id)
	if part_data.is_empty():
		push_error("电机零件未找到: " + part_id)
		return null
	
	var joint = HingeJoint3D.new()
	joint.name = "MotorJoint_" + part_data.get("model", part_id)
	
	# 设置关节节点路径
	joint.node_a = body_a.get_path()
	joint.node_b = body_b.get_path()
	
	# 设置旋转轴
	# 注意：HingeJoint3D 的轴是沿 X 轴
	var transform_a = Transform3D(Basis(), local_pos_a)
	var transform_b = Transform3D(Basis(), local_pos_b)
	
	# 如果轴不是 X 轴，需要旋转
	if not axis.is_equal_approx(Vector3.RIGHT):
		var rotation = Basis().looking_at(axis)
		transform_a.basis = rotation
		transform_b.basis = rotation
	
	# 在 Godot 4.x 中，关节的位置由节点的全局位置决定
	body_a.add_child(joint)
	joint.global_position = body_a.global_position + local_pos_a
	
	# 应用电机参数
	var specs = part_data.get("specifications", {})
	var stall_torque = specs.get("stall_torque", 1.0)
	
	# 启用电机
	joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, 0.0)
	joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, stall_torque)
	joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, true)
	
	# 设置限位（默认 ±90度）
	joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, deg_to_rad(-90))
	joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, deg_to_rad(90))
	joint.set_flag(HingeJoint3D.FLAG_USE_LIMIT, true)
	
	# 存储元数据
	joint.set_meta("part_id", part_id)
	joint.set_meta("part_data", part_data)
	
	print("🔗 创建电机关节: ", part_id, " (扭矩: ", stall_torque, " N·m)")
	
	return joint


## 验证零件数据的完整性
func validate_part(part_id: String) -> bool:
	var part_data = get_part(part_id)
	if part_data.is_empty():
		return false
	
	# 检查必需字段
	var required_fields = ["part_id", "category", "manufacturer", "model", "specifications"]
	for field in required_fields:
		if not part_data.has(field):
			push_warning("零件 " + part_id + " 缺少字段: " + field)
			return false
	
	return true


## 列出所有已加载的零件
func list_all_parts() -> Array[String]:
	var result: Array[String] = []
	for part_id in parts_database:
		result.append(part_id)
	return result


## 打印零件库统计信息
func print_statistics() -> void:
	print("\n=== 零件库统计 ===")
	print("总零件数: ", parts_database.size())
	
	var categories = {}
	for part_id in parts_database:
		var category = parts_database[part_id].get("category", "unknown")
		if not categories.has(category):
			categories[category] = 0
		categories[category] += 1
	
	print("分类统计:")
	for category in categories:
		print("  - ", category, ": ", categories[category])
	
	print("==================\n")
