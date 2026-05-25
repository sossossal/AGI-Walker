extends Node

# TCP Server for RL Interface
# Listens on port 9000
# Exchanges JSON data with Length Prefix (4 bytes, Little Endian)

const RobotAssemblerScript = preload("res://scripts/robot_assembler.gd")

var server = TCPServer.new()
var PORT = 9000
var connection: StreamPeerTCP = null
var robot_node: Node3D = null
var robot_assembler = null
var generated_controller = null
var last_loaded_robot_config: Dictionary = {}
var last_instruction_set: Dictionary = {}
var simulated_circuit_config: Dictionary = {}
var last_instruction_command_batch: Array = []
var last_compatibility_params: Dictionary = {}
var terrain_config: Dictionary = {}
var terrain_summary: Dictionary = {}
var current_control_state := {
	"linear_x": 0.0,
	"linear_y": 0.0,
	"angular_z": 0.0,
	"gait": "",
	"pose": "",
	"step_count": 0
}

func _ready():
	var cli_args = OS.get_cmdline_args()
	var user_args = OS.get_cmdline_user_args()
	for arg in cli_args + user_args:
		if arg.begins_with("--tcp-port="):
			PORT = arg.trim_prefix("--tcp-port=").to_int()

	var err = server.listen(PORT)
	if err == OK:
		print("✅ [TCP] RL Server listening on port %d" % PORT)
	else:
		print("❌ [TCP] Failed to listen on port %d, error: %d" % [PORT, err])
		
	# Try to find a robot node in the scene
	# In headless verification, we might need to spawn a dummy one
	robot_node = get_tree().root.find_child("*Robot*", true, false) as Node3D
	if not robot_node and get_tree().current_scene:
		robot_node = get_tree().current_scene.find_child("*Robot*", true, false) as Node3D
	robot_assembler = RobotAssemblerScript.new()
	robot_assembler.name = "RobotAssembler"
	add_child(robot_assembler)

func _process(delta):
	# 1. Accept New Connections
	if server.is_connection_available():
		if connection != null:
			print("⚠️ [TCP] Dropping existing connection for new one.")
			connection.disconnect_from_host()
			
		connection = server.take_connection()
		connection.set_no_delay(true)
		print("🔗 [TCP] Client connected: %s (BigEndian=%s)" % [connection.get_connected_host(), connection.is_big_endian_enabled()])
		
	# 2. Process Data
	if connection != null:
		connection.poll()
		var status = connection.get_status()
		if status == StreamPeerTCP.STATUS_CONNECTED:
			_handle_messages()
		elif status == StreamPeerTCP.STATUS_NONE or status == StreamPeerTCP.STATUS_ERROR:
			print("❌ [TCP] Connection lost.")
			connection = null

func _handle_messages():
	# Protocol: Length (4 bytes LE) + JSON payload
	if connection.get_available_bytes() < 4:
		return
		
	# Peek length? StreamPeer doesn't support peek easily, we just read.
	# But we need to be careful not to read partial packets if we can't buffer.
	# For simplicity in this demo, we assume blocking read for the length is fine or we manage buffer.
	# However, get_available_bytes checks internal buffer.
	
	# Let's read length
	var length = connection.get_32() # Reads 4 bytes
	
	# Wait for payload
	# In a real rigorous impl, we'd buffer. Here we block-wait or check again?
	# StreamPeerTCP.get_data is blocking? No, it returns error if not enough.
	# But we already checked available bytes >= 4. Now we need available >= length.
	
	# Danger: if payload hasn't arrived, we are stuck? 
	# Actually get_32 removed 4 bytes. 
	# A better approach for async:
	# buffer += received_bytes
	# while buffer.size() >= 4:
	#    len = ...
	#    if buffer.size() >= 4 + len: extract...
	
	# For this verification script, let's just do a blocking wait loop with timeout for payload
	# because Gym sends command and waits for reply, it's synchronized.
	
	var timeout = 0.5
	var time_waited = 0.0
	while connection.get_available_bytes() < length:
		OS.delay_msec(1)
		connection.poll()
		time_waited += 0.001
		if time_waited > timeout:
			print("❌ [TCP] Timeout waiting for payload.")
			connection.disconnect_from_host() # Force disconnect
			connection = null
			return

	var json_str = connection.get_utf8_string(length)
	var cmd = JSON.parse_string(json_str)
	
	if cmd:
		_process_command(cmd)
	else:
		print("❌ [TCP] Failed to parse JSON")

func _process_command(cmd):
	var response = {}
	
	if cmd.type == "reset":
		print("🔄 [TCP] Resetting Simulation")
		if generated_controller != null:
			generated_controller.reset_pose()
		elif robot_node != null:
			if robot_node.has_method("reset_pose"):
				robot_node.reset_pose()
			if cmd.has("sim_params") and robot_node.has_method("apply_sim_params"):
				robot_node.apply_sim_params(cmd.get("sim_params"))
		response = _get_observation()
		
	elif cmd.type == "step":
		var action = cmd.get("action", [])
		if generated_controller != null:
			generated_controller.apply_action(action)
		elif robot_node != null and robot_node.has_method("apply_action"):
			robot_node.apply_action(action)
		response = _get_observation()
		response["reward"] = 0.0 # Placeholder
		response["done"] = false
		
	elif cmd.type == "load_robot":
		print("📦 [TCP] Loading Robot Config")
		var robot_config = cmd.get("robot_config", {})
		if robot_node == null:
			robot_node = get_tree().root.find_child("*Robot*", true, false) as Node3D
			if not robot_node and get_tree().current_scene:
				robot_node = get_tree().current_scene.find_child("*Robot*", true, false) as Node3D

		if robot_assembler != null and not robot_config.get("parts", []).is_empty():
			var parent = get_tree().current_scene as Node3D
			if parent == null:
				parent = Node3D.new()
				parent.name = "GeneratedRobotRoot"
				add_child(parent)
			robot_node = robot_assembler.build_from_config(robot_config, parent)
			generated_controller = robot_node.get_node_or_null("GeneratedRobotController")
			last_loaded_robot_config = robot_config.duplicate(true)
			response = robot_assembler.last_summary
		elif robot_node != null and robot_node.has_method("load_from_dict"):
			robot_node.load_from_dict(robot_config)
			last_loaded_robot_config = robot_config.duplicate(true)
			response = {"status": "success"}
		else:
			last_loaded_robot_config = robot_config.duplicate(true)
			print("ℹ️ [TCP] No robot node available; storing config in fallback mode")
			response = {"status": "success", "mode": "fallback"}
	elif cmd.type == "configure_terrain":
		response = _configure_terrain(cmd.get("terrain", {}))

	elif cmd.type == "get_schema":
		if generated_controller != null:
			response = _generated_robot_schema()
		elif robot_node != null and robot_node.has_method("get_schema"):
			response = robot_node.get_schema()
		else:
			# Fallback Dummy Schema
			response = {
				"sensors": {
					"battery": {"type": "float32", "shape": [1]},
					"vector": {"type": "float32", "shape": [24]},
					"instruction_runtime": {"type": "dict"},
					"simulated_circuit": {"type": "dict"},
					"terrain": {"type": "dict"}
				},
				"actuators": {
					"action": {"type": "float32", "shape": [2], "range": [-10.0, 10.0]},
					"instruction_set": {"type": "dict"},
					"configure_simulated_circuit": {"type": "dict"}
				},
				"meta": {
					"last_loaded_parts": last_loaded_robot_config.get("parts", []).size(),
					"last_loaded_connections": last_loaded_robot_config.get("connections", []).size(),
					"last_instruction_step_count": last_instruction_set.get("steps", []).size(),
					"last_instruction_sequence": last_instruction_set.get("sequence_name", ""),
					"simulated_circuit_transport": simulated_circuit_config.get("transport", ""),
					"terrain": terrain_summary
				}
			}
	elif cmd.type == "configure_simulated_circuit":
		simulated_circuit_config = cmd.get("simulated_circuit", {}).duplicate(true)
		response = {
			"status": "success",
			"mode": "fallback",
			"simulated_circuit": simulated_circuit_config,
			"message": "simulated circuit configured"
		}
	elif cmd.type == "instruction_set":
		var instruction_payload = cmd.get("instruction_set", {})
		last_instruction_set = instruction_payload.duplicate(true)
		last_instruction_command_batch = cmd.get("simulated_circuit_command_batch", []).duplicate(true)
		last_compatibility_params = cmd.get("compatibility_params", {}).duplicate(true)
		_apply_instruction_steps(last_instruction_set.get("steps", []))
		response = {
			"status": "success",
			"mode": "fallback",
			"sequence_name": last_instruction_set.get("sequence_name", ""),
			"instruction_step_count": last_instruction_set.get("steps", []).size(),
			"simulated_circuit_configured": not simulated_circuit_config.is_empty(),
			"compatibility_params": last_compatibility_params,
			"simulated_circuit_command_batch": last_instruction_command_batch
		}
			
	_send_response(response)

func _apply_instruction_steps(steps: Array) -> void:
	if generated_controller != null:
		generated_controller.apply_instruction_steps(steps)
	current_control_state["step_count"] = steps.size()
	for step in steps:
		var kind = step.get("kind", "")
		match kind:
			"set_velocity":
				current_control_state["linear_x"] = float(step.get("linear_x", 0.0))
				current_control_state["linear_y"] = float(step.get("linear_y", 0.0))
				current_control_state["angular_z"] = float(step.get("angular_z", 0.0))
			"set_gait":
				current_control_state["gait"] = str(step.get("gait", ""))
			"set_pose":
				current_control_state["pose"] = str(step.get("pose", ""))
			"emergency_stop":
				current_control_state["linear_x"] = 0.0
				current_control_state["linear_y"] = 0.0
				current_control_state["angular_z"] = 0.0

func _configure_terrain(config: Dictionary) -> Dictionary:
	terrain_config = config.duplicate(true)
	var parent = get_tree().current_scene as Node3D
	if parent == null:
		parent = get_parent() as Node3D
	if parent == null:
		return {
			"status": "error",
			"error": "terrain parent Node3D was not available"
		}
	var terrain = parent.get_node_or_null("MountainTerrain")
	if terrain == null:
		terrain = StaticBody3D.new()
		terrain.name = "MountainTerrain"
		parent.add_child(terrain)
	var collider = terrain.get_node_or_null("CollisionShape3D")
	if collider == null:
		collider = CollisionShape3D.new()
		collider.name = "CollisionShape3D"
		terrain.add_child(collider)
	var mesh_instance = terrain.get_node_or_null("MeshInstance3D")
	if mesh_instance == null:
		mesh_instance = MeshInstance3D.new()
		mesh_instance.name = "MeshInstance3D"
		terrain.add_child(mesh_instance)
	var map_size = max(4, int(config.get("map_size", 48)))
	var cell_size = max(0.05, float(config.get("cell_size_m", 0.35)))
	var data = _mountain_height_data(config, map_size)
	var shape = HeightMapShape3D.new()
	shape.map_width = map_size
	shape.map_depth = map_size
	shape.map_data = data
	collider.shape = shape
	mesh_instance.mesh = _mountain_mesh(data, map_size, cell_size)
	terrain.physics_material_override = _terrain_physics_material(config)
	terrain_summary = _terrain_summary(config, data, map_size, cell_size)
	print("⛰️ [TCP] Mountain terrain configured: %s" % [terrain_summary])
	return terrain_summary.duplicate(true)

func _mountain_height_data(config: Dictionary, map_size: int) -> PackedFloat32Array:
	var seed = float(config.get("seed", 42))
	var ridge_height = float(config.get("ridge_height_m", 0.55))
	var roughness = float(config.get("roughness_m", 0.08))
	var frequency = float(config.get("frequency", 0.16))
	var data = PackedFloat32Array()
	for z in range(map_size):
		for x in range(map_size):
			var nx = float(x) / max(1.0, float(map_size - 1))
			var centered_z = float(z) - float(map_size) * 0.5
			var ridge = ridge_height * nx
			var wave = sin(float(x) * frequency + seed * 0.01) * roughness
			var cross = cos(centered_z * frequency * 1.7 + float(x) * 0.11) * roughness * 0.65
			var rocks = sin(float(x) * 0.41 + float(z) * 0.29 + seed) * roughness * 0.35
			data.append(ridge + wave + cross + rocks)
	return data

func _mountain_mesh(data: PackedFloat32Array, map_size: int, cell_size: float) -> ArrayMesh:
	var st = SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	for z in range(map_size):
		for x in range(map_size):
			var index = z * map_size + x
			var vx = (float(x) - float(map_size) * 0.5) * cell_size
			var vz = (float(z) - float(map_size) * 0.5) * cell_size
			st.set_uv(Vector2(float(x) / map_size, float(z) / map_size))
			st.add_vertex(Vector3(vx, data[index], vz))
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
	return st.commit()

func _terrain_physics_material(config: Dictionary) -> PhysicsMaterial:
	var material = PhysicsMaterial.new()
	material.friction = float(config.get("friction", 1.15))
	material.rough = true
	return material

func _terrain_summary(config: Dictionary, data: PackedFloat32Array, map_size: int, cell_size: float) -> Dictionary:
	var min_height = INF
	var max_height = -INF
	var sum_height = 0.0
	for height in data:
		min_height = min(min_height, height)
		max_height = max(max_height, height)
		sum_height += height
	return {
		"status": "success",
		"terrain_version": str(config.get("terrain_version", "dynamic_godot_mountain_terrain.v1")),
		"type": str(config.get("type", "mountain")),
		"seed": int(config.get("seed", 42)),
		"map_size": map_size,
		"cell_size_m": cell_size,
		"height_samples": data.size(),
		"min_height_m": min_height,
		"max_height_m": max_height,
		"height_range_m": max_height - min_height,
		"average_height_m": sum_height / max(1, data.size()),
		"friction": float(config.get("friction", 1.15)),
		"collision_shape": "HeightMapShape3D",
		"mesh_type": "ArrayMesh"
	}

func _get_observation():
	if generated_controller != null:
		var sensor_data = generated_controller.get_sensor_data()
		sensor_data["simulated_circuit"] = {
			"configured": not simulated_circuit_config.is_empty(),
			"transport": simulated_circuit_config.get("transport", ""),
			"command_batch_size": last_instruction_command_batch.size()
		}
		sensor_data["terrain"] = terrain_summary.duplicate(true)
		return sensor_data

	if robot_node != null and robot_node.has_method("get_sensor_data"):
		var data = robot_node.get_sensor_data()
		data["terrain"] = terrain_summary.duplicate(true)
		return data
		
	var vec = []
	vec.resize(24)
	vec.fill(0.1)
	
	# Construct Mock Observation
	return {
		"image": [], # optimized out for now
		"vector": vec, # Mock 24-dim vector
		"battery": 100.0,
		"instruction_runtime": {
			"sequence_name": last_instruction_set.get("sequence_name", ""),
			"step_count": current_control_state["step_count"],
			"linear_x": current_control_state["linear_x"],
			"linear_y": current_control_state["linear_y"],
			"angular_z": current_control_state["angular_z"],
			"gait": current_control_state["gait"],
			"pose": current_control_state["pose"]
		},
		"simulated_circuit": {
			"configured": not simulated_circuit_config.is_empty(),
			"transport": simulated_circuit_config.get("transport", ""),
			"command_batch_size": last_instruction_command_batch.size()
		},
		"terrain": terrain_summary.duplicate(true)
	}

func _generated_robot_schema() -> Dictionary:
	return {
		"sensors": {
			"body_count": {"type": "int32", "shape": [1]},
			"joint_count": {"type": "int32", "shape": [1]},
			"position": {"type": "float32", "shape": [3]},
			"body_states": {"type": "dict"},
			"joint_states": {"type": "dict"},
			"instruction_runtime": {"type": "dict"},
			"simulated_circuit": {"type": "dict"},
			"terrain": {"type": "dict"}
		},
		"actuators": {
			"action": {"type": "float32", "shape": [generated_controller.joints.size()]},
			"instruction_set": {"type": "dict"},
			"configure_simulated_circuit": {"type": "dict"}
		},
		"meta": {
			"dynamic_robot_generation": true,
			"last_loaded_parts": last_loaded_robot_config.get("parts", []).size(),
			"last_loaded_connections": last_loaded_robot_config.get("connections", []).size(),
			"assembly_summary": robot_assembler.last_summary if robot_assembler else {},
			"terrain": terrain_summary
		}
	}

func _send_response(resp_dict):
	var json_str = JSON.stringify(resp_dict)
	var data = json_str.to_utf8_buffer()
	var length = data.size()
	
	connection.put_32(length)
	connection.put_data(data)
