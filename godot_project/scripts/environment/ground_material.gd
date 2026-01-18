# ground_material.gd
# 地面材质系统 - 定义不同地面的物理属性
extends Resource

class_name GroundMaterial

## 材质属性
@export var material_name: String = "Default"
@export var friction: float = 0.8
@export var bounce: float = 0.0
@export var roughness: float = 0.5
@export var color: Color = Color(0.5, 0.5, 0.5)

## 高级属性
@export var rolling_friction: float = 0.01 # 滚动摩擦
@export var is_deformable: bool = false # 是否可变形（沙地、泥地）
@export var sound_type: String = "concrete" # 接触声音类型

func _init(
	p_name: String = "Default",
	p_friction: float = 0.8,
	p_bounce: float = 0.0
):
	material_name = p_name
	friction = p_friction
	bounce = p_bounce

## 获取物理材质
func get_physics_material() -> PhysicsMaterial:
	var mat = PhysicsMaterial.new()
	mat.friction = friction
	mat.bounce = bounce
	return mat

## 应用到静态体
func apply_to_static_body(body: StaticBody3D) -> void:
	var mat = get_physics_material()
	body.physics_material_override = mat
	
	# 设置视觉材质（如果有 MeshInstance3D）
	for child in body.get_children():
		if child is MeshInstance3D:
			var mesh_mat = StandardMaterial3D.new()
			mesh_mat.albedo_color = color
			mesh_mat.roughness = roughness
			child.set_surface_override_material(0, mesh_mat)

## 获取材质信息
func to_dict() -> Dictionary:
	return {
		"name": material_name,
		"friction": friction,
		"bounce": bounce,
		"roughness": roughness,
		"color": color.to_html(),
		"rolling_friction": rolling_friction,
		"is_deformable": is_deformable
	}
