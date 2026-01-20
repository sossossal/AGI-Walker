extends Node3D

## 程序化地形生成器
## 将本脚本附加到场景中的 TerrainGenerator 节点 (Node3D)
## 需要有一个子节点 Ground (StaticBody3D)

@export var map_size: int = 128
@export var height_scale: float = 2.0
@export var frequency: float = 0.05

var noise: FastNoiseLite
var terrain_collider: CollisionShape3D
var terrain_mesh: MeshInstance3D

func _ready():
	noise = FastNoiseLite.new()
	noise.noise_type = FastNoiseLite.TYPE_PERLIN
	noise.seed = randi()
	
	# 查找或创建地面节点
	var ground = get_node_or_null("Ground")
	if not ground:
		ground = StaticBody3D.new()
		ground.name = "Ground"
		add_child(ground)
		
	# 查找或创建碰撞体
	terrain_collider = ground.get_node_or_null("CollisionShape3D")
	if not terrain_collider:
		terrain_collider = CollisionShape3D.new()
		terrain_collider.name = "CollisionShape3D"
		ground.add_child(terrain_collider)
		
	# 查找或创建网格 (Visual)
	terrain_mesh = ground.get_node_or_null("MeshInstance3D")
	if not terrain_mesh:
		terrain_mesh = MeshInstance3D.new()
		terrain_mesh.name = "MeshInstance3D"
		ground.add_child(terrain_mesh)

func generate(seed_val: int = 0, roughness: float = 1.0):
	"""根据种子和粗糙度生成地形"""
	print("🌍 Generating terrain with seed: %d" % seed_val)
	
	noise.seed = seed_val
	noise.frequency = frequency * roughness
	
	# ==============================
	# 1. 更新物理碰撞 (HeightMapShape3D)
	# ==============================
	var shape = HeightMapShape3D.new()
	shape.map_width = map_size
	shape.map_depth = map_size
	
	var data = PackedFloat32Array()
	# HeightMapShape3D expects a flat array of floats
	# Size is width * depth
	
	# 为了居中，我们从 -size/2 到 size/2
	for z in range(map_size):
		for x in range(map_size):
			var h = noise.get_noise_2d(x, z) * height_scale * roughness
			# 让他平滑点，边缘设为0? (可选)
			data.append(h)
			
	shape.map_data = data
	terrain_collider.shape = shape
	
	# ==============================
	# 2. 更新视觉网格 (ArrayMesh)
	# ==============================
	# 这里简单使用 PlaneMesh 加上 Shader 或者直接修改顶点
	# 为了简单起见，我们生成一个新的 ArrayMesh
	
	var st = SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	
	var uv_scale = 1.0 / map_size
	
	for z in range(map_size):
		for x in range(map_size):
			# HeightMapShape 也是基于 grid 的
			# Grid coordinate to local coordinate
			# HeightMapShape centers the map. Width/Depth are number of vertices? No, number of quads usually +1
			# Godot HeightMapShape3D is tricky. Let's align roughly.
			# 采样高度
			var h = data[z * map_size + x]
			
			# Create vertices
			# Center is (0,0,0)
			var vx = (x - map_size * 0.5)
			var vz = (z - map_size * 0.5)
			
			st.set_uv(Vector2(x * uv_scale, z * uv_scale))
			st.add_vertex(Vector3(vx, h, vz))

	# Indices generation (Quad -> 2 Triangles)
	for z in range(map_size - 1):
		for x in range(map_size - 1):
			var tl = z * map_size + x
			var tr = tl + 1
			var bl = (z + 1) * map_size + x
			var br = bl + 1
			
			st.add_index(tl)
			st.add_index(tr)
			st.add_index(bl)
			
			st.add_index(tr)
			st.add_index(br)
			st.add_index(bl)
			
	st.generate_normals()
	terrain_mesh.mesh = st.commit()
	
	# 创建材质
	var mat = StandardMaterial3D.new()
	mat.albedo_color = Color(0.3, 0.5, 0.3) # Greenish
	terrain_mesh.material_override = mat
