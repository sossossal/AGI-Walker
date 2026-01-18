## test_parts_library.gd
## 测试零件库功能的脚本
extends Node3D

@onready var parts_lib: RobotPartsLibrary

func _ready():
	# 创建零件库管理器
	parts_lib = RobotPartsLibrary.new()
	add_child(parts_lib)
	
	# 等待零件库加载完成
	await get_tree().process_frame
	
	# 打印统计信息
	parts_lib.print_statistics()
	
	# 测试获取零件
	test_get_parts()
	
	# 测试创建电机实例
	test_create_motor()
	
	# 测试创建关节
	test_create_joint()


func test_get_parts():
	print("\n=== 测试获取零件 ===")
	
	# 获取单个零件
	var xl430 = parts_lib.get_part("dynamixel_xl430_w250")
	if not xl430.is_empty():
		print("✓ 成功获取 XL430:")
		print("  型号: ", xl430.get("model"))
		print("  扭矩: ", xl430.get("specifications", {}).get("stall_torque"), " N·m")
		print("  价格: $", xl430.get("price_usd"))
	
	# 获取categorty
	var servos = parts_lib.get_parts_by_category("actuator_servo")
	print("✓ 舵机类零件数量: ", servos.size())
	
	# 获取制造商
	var robotis_parts = parts_lib.get_parts_by_manufacturer("ROBOTIS")
	print("✓ ROBOTIS 零件数量: ", robotis_parts.size())


func test_create_motor():
	print("\n=== 测试创建电机实例 ===")
	
	# 创建一个 XL430 电机
	var motor = parts_lib.create_motor_instance("dynamixel_xl430_w250", self)
	
	if motor:
		motor.position = Vector3(0, 2, 0)
		print("✓ 电机已创建，位置: ", motor.position)
		print("  质量: ", motor.mass, " kg")
		print("  扭矩: ", motor.get_meta("stall_torque"), " N·m")


func test_create_joint():
	print("\n=== 测试创建电机关节 ===")
	
	# 创建两个刚体
	var body_a = RigidBody3D.new()
	body_a.name = "BodyA"
	body_a.mass = 5.0
	add_child(body_a)
	body_a.position = Vector3(-2, 2, 0)
	
	var collision_a = CollisionShape3D.new()
	var shape_a = BoxShape3D.new()
	shape_a.size = Vector3(0.3, 0.5, 0.3)
	collision_a.shape = shape_a
	body_a.add_child(collision_a)
	
	var mesh_a = MeshInstance3D.new()
	mesh_a.mesh = BoxMesh.new()
	mesh_a.mesh.size = shape_a.size
	body_a.add_child(mesh_a)
	
	var body_b = RigidBody3D.new()
	body_b.name = "BodyB"
	body_b.mass = 2.0
	add_child(body_b)
	body_b.position = Vector3(-2, 1.2, 0)
	
	var collision_b = CollisionShape3D.new()
	var shape_b = BoxShape3D.new()
	shape_b.size = Vector3(0.2, 0.4, 0.2)
	collision_b.shape = shape_b
	body_b.add_child(collision_b)
	
	var mesh_b = MeshInstance3D.new()
	mesh_b.mesh = BoxMesh.new()
	mesh_b.mesh.size = shape_b.size
	body_b.add_child(mesh_b)
	
	# 创建电机关节连接两个刚体
	var joint = parts_lib.create_motor_joint(
		"dynamixel_xl430_w250",
		body_a,
		body_b,
		Vector3.RIGHT,
		Vector3(0, -0.25, 0),
		Vector3(0, 0.2, 0)
	)
	
	if joint:
		print("✓ 电机关节已创建")
		print("  最大扭矩: ", joint.get_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE), " N·m")
		
		# 设置电机目标速度
		joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, 1.0)
		print("  目标速度: 1.0 rad/s")


func _input(event):
	if event is InputEventKey and event.pressed:
		if event.keycode == KEY_SPACE:
			print("\n按下空格键 - 列出所有零件:")
			var all_parts = parts_lib.list_all_parts()
			for part_id in all_parts:
				var part = parts_lib.get_part(part_id)
				print("  - ", part_id, ": ", part.get("model"), " ($", part.get("price_usd"), ")")
