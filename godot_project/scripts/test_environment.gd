# test_environment.gd
# 环境系统测试脚本
extends Node3D

@onready var env_controller = $EnvironmentController
@onready var material_library = $GroundMaterialLibrary
@onready var ground = $Ground

func _ready():
	print("\n=== 环境系统测试 ===\n")
	
	# 测试1：环境预设
	test_environment_presets()
	
	# 测试2：地面材质
	test_ground_materials()
	
	# 测试3：动态参数调节
	test_dynamic_parameters()
	
	print("\n=== 测试完成 ===\n")

func test_environment_presets():
	print("[1] 测试环境预设...")
	
	# 地球环境
	env_controller.load_preset("earth")
	await get_tree().create_timer(0.5).timeout
	
	# 月球环境
	env_controller.load_preset("moon")
	await get_tree().create_timer(0.5).timeout
	
	# 火星环境
	env_controller.load_preset("mars")
	await get_tree().create_timer(0.5).timeout
	
	print("  ✅ 环境预设测试完成\n")

func test_ground_materials():
	print("[2] 测试地面材质...")
	
	var materials = material_library.list_materials()
	print("  可用材质: ", materials)
	
	# 测试每种材质
	for mat_name in materials:
		material_library.apply_material(ground, mat_name)
		var mat = material_library.get_material(mat_name)
		print("  - ", mat.material_name, ": 摩擦=", mat.friction, " 弹性=", mat.bounce)
		await get_tree().create_timer(0.2).timeout
	
	print("  ✅ 地面材质测试完成\n")

func test_dynamic_parameters():
	print("[3] 测试动态参数...")
	
	# 测试重力调节
	for g in [9.81, 5.0, 15.0]:
		env_controller.set_gravity(g)
		print("  重力: ", g, " m/s²")
		await get_tree().create_timer(0.3).timeout
	
	# 测试温度
	for temp in [25.0, -50.0, 100.0]:
		env_controller.set_temperature(temp)
		print("  温度: ", temp, " °C")
		await get_tree().create_timer(0.2).timeout
	
	# 测试风力
	env_controller.set_wind(Vector3(5, 0, 2))
	print("  风力: 5m/s 东风")
	
	print("  ✅ 动态参数测试完成\n")

## 键盘控制测试
func _input(event):
	if event is InputEventKey and event.pressed:
		match event.keycode:
			KEY_1:
				env_controller.load_preset("earth")
				print("🌍 切换到地球环境")
			KEY_2:
				env_controller.load_preset("moon")
				print("🌑 切换到月球环境")
			KEY_3:
				env_controller.load_preset("mars")
				print("🔴 切换到火星环境")
			KEY_C:
				material_library.apply_material(ground, "concrete")
				print("切换到混凝土地面")
			KEY_I:
				material_library.apply_material(ground, "ice")
				print("切换到冰面")
			KEY_S:
				material_library.apply_material(ground, "sand")
				print("切换到沙地")
