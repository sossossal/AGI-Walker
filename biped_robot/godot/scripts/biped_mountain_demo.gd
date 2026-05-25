extends Node3D

const TERRAIN_SIZE := 42.0
const TERRAIN_GRID := 48
const WALKWAY_WIDTH := 3.6
const ROBOT_SPEED := 0.55
const GAIT_CYCLE_SECONDS := 1.15
const GODOT_IO_INPUT := "res://../config/godot_io_input.json"

var _time: float = 0.0
var _robot_speed: float = ROBOT_SPEED
var _gait_cycle_seconds: float = GAIT_CYCLE_SECONDS
var _telemetry_interval_seconds: float = 0.05
var _next_telemetry_time: float = 0.0
var _output_telemetry_path: String = "res://../test_env/godot_io_telemetry.jsonl"
var _output_report_path: String = "res://../test_env/godot_io_report.json"
var _commands: Array = []
var _next_command_index: int = 0
var _commands_applied: Array = []
var _telemetry_rows: int = 0
var _min_robot_z: float = INF
var _max_robot_z: float = -INF
var _max_abs_roll: float = 0.0
var _max_abs_pitch: float = 0.0
var _telemetry_file: FileAccess
var _robot_root: Node3D
var _parts: Dictionary = {}

func _ready() -> void:
	_load_io_input()
	_open_io_outputs()
	_build_world()
	_build_robot()

func _process(delta: float) -> void:
	_time += delta
	_apply_due_commands()
	_animate_robot(_time)
	_record_godot_io_telemetry()

func _exit_tree() -> void:
	_write_godot_io_report()

func _build_world() -> void:
	var sun := DirectionalLight3D.new()
	sun.name = "Sun"
	sun.rotation_degrees = Vector3(-52.0, 38.0, 0.0)
	sun.light_energy = 2.4
	add_child(sun)

	var camera := Camera3D.new()
	camera.name = "FollowCamera"
	camera.position = Vector3(4.2, 3.2, 7.0)
	camera.rotation_degrees = Vector3(-20.0, 34.0, 0.0)
	camera.current = true
	add_child(camera)

	var terrain := MeshInstance3D.new()
	terrain.name = "ProceduralMountainTerrain"
	terrain.mesh = _build_terrain_mesh()
	var terrain_material := StandardMaterial3D.new()
	terrain_material.albedo_color = Color(0.34, 0.46, 0.30)
	terrain_material.roughness = 0.92
	terrain_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	terrain.material_override = terrain_material
	add_child(terrain)

	var path := MeshInstance3D.new()
	path.name = "VisibleWalkway"
	var path_mesh := BoxMesh.new()
	path_mesh.size = Vector3(WALKWAY_WIDTH, 0.025, TERRAIN_SIZE)
	path.mesh = path_mesh
	path.position = Vector3(0.0, 0.03, 0.0)
	var path_material := StandardMaterial3D.new()
	path_material.albedo_color = Color(0.42, 0.38, 0.29)
	path_material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	path.material_override = path_material
	add_child(path)

func _build_robot() -> void:
	_robot_root = Node3D.new()
	_robot_root.name = "HumanoidBipedRobot"
	add_child(_robot_root)

	_parts["torso"] = _box("Torso", Vector3(0.42, 0.64, 0.24), Color(0.16, 0.35, 0.78), _robot_root)
	_parts["head"] = _box("Head", Vector3(0.24, 0.24, 0.24), Color(0.70, 0.76, 0.82), _robot_root)
	_parts["left_upper_leg"] = _box("LeftUpperLeg", Vector3(0.14, 0.46, 0.14), Color(0.12, 0.20, 0.28), _robot_root)
	_parts["left_lower_leg"] = _box("LeftLowerLeg", Vector3(0.12, 0.46, 0.12), Color(0.12, 0.20, 0.28), _robot_root)
	_parts["left_foot"] = _box("LeftFoot", Vector3(0.18, 0.08, 0.32), Color(0.05, 0.07, 0.09), _robot_root)
	_parts["right_upper_leg"] = _box("RightUpperLeg", Vector3(0.14, 0.46, 0.14), Color(0.12, 0.20, 0.28), _robot_root)
	_parts["right_lower_leg"] = _box("RightLowerLeg", Vector3(0.12, 0.46, 0.12), Color(0.12, 0.20, 0.28), _robot_root)
	_parts["right_foot"] = _box("RightFoot", Vector3(0.18, 0.08, 0.32), Color(0.05, 0.07, 0.09), _robot_root)
	_parts["left_upper_arm"] = _box("LeftUpperArm", Vector3(0.11, 0.36, 0.11), Color(0.18, 0.44, 0.86), _robot_root)
	_parts["left_lower_arm"] = _box("LeftLowerArm", Vector3(0.10, 0.34, 0.10), Color(0.18, 0.44, 0.86), _robot_root)
	_parts["right_upper_arm"] = _box("RightUpperArm", Vector3(0.11, 0.36, 0.11), Color(0.18, 0.44, 0.86), _robot_root)
	_parts["right_lower_arm"] = _box("RightLowerArm", Vector3(0.10, 0.34, 0.10), Color(0.18, 0.44, 0.86), _robot_root)

func _animate_robot(seconds: float) -> void:
	var z: float = fposmod(seconds * _robot_speed + TERRAIN_SIZE * 0.5, TERRAIN_SIZE) - TERRAIN_SIZE * 0.5
	var ground: float = _terrain_height(0.0, z)
	var phase: float = TAU * seconds / _gait_cycle_seconds
	var left_stride: float = sin(phase)
	var right_stride: float = sin(phase + PI)
	var vertical_bob: float = 0.045 * abs(sin(phase * 2.0))

	_robot_root.position = Vector3(0.0, ground + 0.98 + vertical_bob, z)
	_robot_root.rotation_degrees.z = 3.0 * sin(phase * 0.5)
	_robot_root.rotation_degrees.x = 2.0 * sin(phase * 0.35)

	(_parts["torso"] as Node3D).position = Vector3(0.0, 0.16, 0.0)
	(_parts["head"] as Node3D).position = Vector3(0.0, 0.64, 0.0)

	_pose_leg("left", -0.13, left_stride)
	_pose_leg("right", 0.13, right_stride)
	_pose_arm("left", -0.34, -left_stride)
	_pose_arm("right", 0.34, -right_stride)

func _pose_leg(side: String, x_offset: float, stride: float) -> void:
	var lift: float = max(0.0, stride) * 0.12
	var swing_z: float = stride * 0.18
	var upper := _parts[side + "_upper_leg"] as Node3D
	var lower := _parts[side + "_lower_leg"] as Node3D
	var foot := _parts[side + "_foot"] as Node3D
	upper.position = Vector3(x_offset, -0.24, swing_z * 0.45)
	lower.position = Vector3(x_offset, -0.68 + lift * 0.35, swing_z * 0.85)
	foot.position = Vector3(x_offset, -0.94 + lift, swing_z + 0.05)
	upper.rotation_degrees.x = -18.0 * stride
	lower.rotation_degrees.x = 22.0 * max(0.0, -stride)
	foot.rotation_degrees.x = 8.0 * stride

func _pose_arm(side: String, x_offset: float, stride: float) -> void:
	var upper := _parts[side + "_upper_arm"] as Node3D
	var lower := _parts[side + "_lower_arm"] as Node3D
	upper.position = Vector3(x_offset, 0.08, -0.04 - stride * 0.12)
	lower.position = Vector3(x_offset, -0.26, -0.04 - stride * 0.18)
	upper.rotation_degrees.x = 22.0 * stride
	lower.rotation_degrees.x = 14.0 * stride

func _box(part_name: String, size: Vector3, color: Color, parent: Node) -> MeshInstance3D:
	var mesh_instance := MeshInstance3D.new()
	mesh_instance.name = part_name
	var mesh := BoxMesh.new()
	mesh.size = size
	mesh_instance.mesh = mesh
	var material := StandardMaterial3D.new()
	material.albedo_color = color
	material.roughness = 0.68
	mesh_instance.material_override = material
	parent.add_child(mesh_instance)
	return mesh_instance

func _build_terrain_mesh() -> Mesh:
	var surface := SurfaceTool.new()
	surface.begin(Mesh.PRIMITIVE_TRIANGLES)
	var half: float = TERRAIN_SIZE * 0.5
	var step: float = TERRAIN_SIZE / float(TERRAIN_GRID)
	for z_index in range(TERRAIN_GRID):
		for x_index in range(TERRAIN_GRID):
			var x0: float = -half + x_index * step
			var x1: float = x0 + step
			var z0: float = -half + z_index * step
			var z1: float = z0 + step
			var p00: Vector3 = Vector3(x0, _terrain_height(x0, z0), z0)
			var p10: Vector3 = Vector3(x1, _terrain_height(x1, z0), z0)
			var p01: Vector3 = Vector3(x0, _terrain_height(x0, z1), z1)
			var p11: Vector3 = Vector3(x1, _terrain_height(x1, z1), z1)
			surface.add_vertex(p00)
			surface.add_vertex(p10)
			surface.add_vertex(p11)
			surface.add_vertex(p00)
			surface.add_vertex(p11)
			surface.add_vertex(p01)
	surface.generate_normals()
	return surface.commit()

func _terrain_height(x: float, z: float) -> float:
	var ridge: float = sin(z * 0.34) * 0.85 + cos((x + z) * 0.18) * 0.28
	var rough: float = sin(x * 1.15) * cos(z * 0.72) * 0.18
	var walkway_blend: float = clamp(abs(x) / WALKWAY_WIDTH, 0.0, 1.0)
	return lerp(ridge * 0.18, ridge + rough, walkway_blend)

func _load_io_input() -> void:
	if not FileAccess.file_exists(GODOT_IO_INPUT):
		return
	var file := FileAccess.open(GODOT_IO_INPUT, FileAccess.READ)
	if file == null:
		push_error("Unable to open Godot IO input: " + GODOT_IO_INPUT)
		return
	var parsed: Variant = JSON.parse_string(file.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY:
		push_error("Godot IO input must be a JSON object")
		return
	var input := parsed as Dictionary
	var simulation := input.get("simulation", {}) as Dictionary
	_robot_speed = ROBOT_SPEED * float(simulation.get("speed_scale", 1.0))
	_gait_cycle_seconds = float(simulation.get("gait_cycle_seconds", GAIT_CYCLE_SECONDS))
	_telemetry_interval_seconds = float(simulation.get("telemetry_interval_seconds", 0.05))
	_commands = input.get("commands", []) as Array
	_commands.sort_custom(func(left: Dictionary, right: Dictionary) -> bool:
		return float(left.get("time_s", 0.0)) < float(right.get("time_s", 0.0))
	)
	var outputs := input.get("outputs", {}) as Dictionary
	_output_telemetry_path = str(outputs.get("telemetry_path", _output_telemetry_path))
	_output_report_path = str(outputs.get("report_path", _output_report_path))

func _open_io_outputs() -> void:
	_ensure_parent_dir(_output_telemetry_path)
	_ensure_parent_dir(_output_report_path)
	_telemetry_file = FileAccess.open(_output_telemetry_path, FileAccess.WRITE)
	if _telemetry_file == null:
		push_error("Unable to open Godot telemetry output: " + _output_telemetry_path)

func _apply_due_commands() -> void:
	while _next_command_index < _commands.size():
		var command := _commands[_next_command_index] as Dictionary
		if _time < float(command.get("time_s", 0.0)):
			return
		if str(command.get("command", "")) == "set_gait":
			_robot_speed = ROBOT_SPEED * float(command.get("speed_scale", 1.0))
			_gait_cycle_seconds = float(command.get("gait_cycle_seconds", _gait_cycle_seconds))
		var applied := command.duplicate(true)
		applied["applied_time_s"] = snapped(_time, 0.0001)
		_commands_applied.append(applied)
		_next_command_index += 1

func _record_godot_io_telemetry() -> void:
	if _telemetry_file == null or _time < _next_telemetry_time or _robot_root == null:
		return
	_next_telemetry_time += _telemetry_interval_seconds
	var z: float = _robot_root.position.z
	var ground: float = _terrain_height(_robot_root.position.x, z)
	var phase: float = TAU * _time / _gait_cycle_seconds
	var left_clearance: float = 0.035 + max(0.0, sin(phase)) * 0.12
	var right_clearance: float = 0.035 + max(0.0, sin(phase + PI)) * 0.12
	_min_robot_z = min(_min_robot_z, z)
	_max_robot_z = max(_max_robot_z, z)
	_max_abs_roll = max(_max_abs_roll, abs(_robot_root.rotation_degrees.z))
	_max_abs_pitch = max(_max_abs_pitch, abs(_robot_root.rotation_degrees.x))
	var row := {
		"time_s": snapped(_time, 0.0001),
		"robot_x_m": snapped(_robot_root.position.x, 0.0001),
		"robot_y_m": snapped(_robot_root.position.y, 0.0001),
		"robot_z_m": snapped(z, 0.0001),
		"terrain_height_m": snapped(ground, 0.0001),
		"speed_scale": snapped(_robot_speed / ROBOT_SPEED, 0.0001),
		"gait_cycle_seconds": snapped(_gait_cycle_seconds, 0.0001),
		"roll_degrees": snapped(_robot_root.rotation_degrees.z, 0.0001),
		"pitch_degrees": snapped(_robot_root.rotation_degrees.x, 0.0001),
		"left_foot_clearance_m": snapped(left_clearance, 0.0001),
		"right_foot_clearance_m": snapped(right_clearance, 0.0001),
		"commands_applied_count": _commands_applied.size()
	}
	_telemetry_file.store_line(JSON.stringify(row, "", false, true))
	_telemetry_rows += 1

func _write_godot_io_report() -> void:
	if _telemetry_file != null:
		_telemetry_file.flush()
		_telemetry_file.close()
	var checks := {
		"telemetry_rows_present": _telemetry_rows >= 20,
		"commands_applied": _commands_applied.size() >= _commands.size(),
		"roll_within_acceptance": _max_abs_roll <= 6.0,
		"pitch_within_acceptance": _max_abs_pitch <= 5.0
	}
	var passed: bool = true
	for value in checks.values():
		passed = passed and bool(value)
	var report := {
		"schema_version": "biped-godot-io-report.v1",
		"status": "passed" if passed else "failed",
		"input_path": GODOT_IO_INPUT,
		"telemetry_path": _output_telemetry_path,
		"telemetry_rows": _telemetry_rows,
		"commands_expected": _commands.size(),
		"commands_applied": _commands_applied,
		"metrics": {
			"min_robot_z_m": snapped(_min_robot_z, 0.0001),
			"max_robot_z_m": snapped(_max_robot_z, 0.0001),
			"max_abs_roll_degrees": snapped(_max_abs_roll, 0.0001),
			"max_abs_pitch_degrees": snapped(_max_abs_pitch, 0.0001)
		},
		"checks": checks
	}
	var report_file := FileAccess.open(_output_report_path, FileAccess.WRITE)
	if report_file == null:
		push_error("Unable to open Godot IO report output: " + _output_report_path)
		return
	report_file.store_string(JSON.stringify(report, "\t", false, true))
	report_file.store_string("\n")
	report_file.close()

func _ensure_parent_dir(path: String) -> void:
	var absolute := ProjectSettings.globalize_path(path)
	var parent := absolute.get_base_dir()
	DirAccess.make_dir_recursive_absolute(parent)
