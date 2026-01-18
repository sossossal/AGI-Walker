# ground_material_library.gd
# 地面材质库 - 预定义各种地面材质
extends Node

class_name GroundMaterialLibrary

## 材质库
var materials: Dictionary = {}

func _ready():
	_initialize_materials()
	print("📚 Ground Material Library initialized with ", materials.size(), " materials")

## 初始化预定义材质
func _initialize_materials() -> void:
	# 1. 混凝土 - 高摩擦，硬质
	var concrete = GroundMaterial.new("Concrete", 0.9, 0.1)
	concrete.roughness = 0.8
	concrete.color = Color(0.6, 0.6, 0.6)
	concrete.rolling_friction = 0.005
	concrete.sound_type = "concrete"
	materials["concrete"] = concrete
	
	# 2. 木板 - 中等摩擦
	var wood = GroundMaterial.new("Wood", 0.6, 0.2)
	wood.roughness = 0.6
	wood.color = Color(0.6, 0.4, 0.2)
	wood.rolling_friction = 0.01
	wood.sound_type = "wood"
	materials["wood"] = wood
	
	# 3. 地毯/橡胶 - 高摩擦，高阻尼
	var carpet = GroundMaterial.new("Carpet", 1.0, 0.05)
	carpet.roughness = 0.9
	carpet.color = Color(0.4, 0.3, 0.3)
	carpet.rolling_friction = 0.03
	carpet.sound_type = "soft"
	materials["carpet"] = carpet
	
	# 4. 冰面 - 极低摩擦
	var ice = GroundMaterial.new("Ice", 0.1, 0.3)
	ice.roughness = 0.1
	ice.color = Color(0.8, 0.9, 1.0)
	ice.rolling_friction = 0.001
	ice.sound_type = "ice"
	materials["ice"] = ice
	
	# 5. 金属 - 低摩擦，高弹性
	var metal = GroundMaterial.new("Metal", 0.4, 0.4)
	metal.roughness = 0.3
	metal.color = Color(0.7, 0.7, 0.8)
	metal.rolling_friction = 0.005
	metal.sound_type = "metal"
	materials["metal"] = metal
	
	# 6. 沙地 - 中等摩擦，可变形
	var sand = GroundMaterial.new("Sand", 0.7, 0.0)
	sand.roughness = 1.0
	sand.color = Color(0.9, 0.8, 0.6)
	sand.rolling_friction = 0.05 # 沙地滚动摩擦大
	sand.is_deformable = true
	sand.sound_type = "sand"
	materials["sand"] = sand
	
	# 7. 草地 - 中高摩擦
	var grass = GroundMaterial.new("Grass", 0.75, 0.1)
	grass.roughness = 0.8
	grass.color = Color(0.2, 0.6, 0.2)
	grass.rolling_friction = 0.02
	grass.sound_type = "grass"
	materials["grass"] = grass
	
	# 8. 泥地 - 高摩擦，可变形
	var mud = GroundMaterial.new("Mud", 0.85, 0.0)
	mud.roughness = 1.0
	mud.color = Color(0.4, 0.3, 0.2)
	mud.rolling_friction = 0.08 # 泥地阻力大
	mud.is_deformable = true
	mud.sound_type = "mud"
	materials["mud"] = mud

## 获取材质
func get_material(material_name: String) -> GroundMaterial:
	if materials.has(material_name):
		return materials[material_name]
	else:
		push_warning("Material not found: " + material_name + ", using concrete")
		return materials["concrete"]

## 应用材质到地面
func apply_material(ground: StaticBody3D, material_name: String) -> void:
	var mat = get_material(material_name)
	mat.apply_to_static_body(ground)
	print("✅ Applied material '", mat.material_name, "' to ground")

## 列出所有材质
func list_materials() -> Array[String]:
	var names: Array[String] = []
	for key in materials.keys():
		names.append(key)
	return names

## 获取材质属性对比
func get_material_comparison() -> Dictionary:
	var comparison = {}
	for key in materials.keys():
		var mat = materials[key]
		comparison[key] = {
			"friction": mat.friction,
			"bounce": mat.bounce,
			"rolling_friction": mat.rolling_friction,
			"deformable": mat.is_deformable
		}
	return comparison

## 添加自定义材质
func add_custom_material(mat: GroundMaterial) -> void:
	materials[mat.material_name.to_lower()] = mat
	print("➕ Added custom material: ", mat.material_name)
