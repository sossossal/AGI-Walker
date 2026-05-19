extends Node
class_name RobotAssembler

const GeneratedRobotControllerScript = preload("res://scripts/generated_robot_controller.gd")

var last_summary: Dictionary = {}


func build_from_config(config: Dictionary, parent: Node3D) -> Node3D:
	var robot = Node3D.new()
	robot.name = config.get("name", "GeneratedRobot")
	_remove_previous_robot(parent, robot.name)
	parent.add_child(robot)

	var body_summary = _create_bodies(config.get("parts", []), robot)
	var bodies = body_summary["bodies"]
	var joint_summary = _create_joints(config.get("connections", []), robot, bodies)
	var expected_parts = _valid_part_count(config.get("parts", []))
	var expected_joints = _valid_connection_count(config.get("connections", []))
	var failed_joint_count = joint_summary["failed"].size()
	var parameterized_joints = _parameterized_joint_count(joint_summary["joint_nodes"])
	var parts_complete = bodies.size() == expected_parts
	var joints_complete = joint_summary["created"] == expected_joints and failed_joint_count == 0
	var parameters_complete = parameterized_joints == joint_summary["created"]
	var assembly_complete = parts_complete and joints_complete and parameters_complete
	var controller = GeneratedRobotControllerScript.new()
	controller.name = "GeneratedRobotController"
	robot.add_child(controller)
	controller.configure(robot, config.get("control", {}))

	last_summary = {
		"status": "success",
		"robot_name": robot.name,
		"expected_parts": expected_parts,
		"parts_created": bodies.size(),
		"parts_complete": parts_complete,
		"expected_joints": expected_joints,
		"joints_created": joint_summary["created"],
		"failed_joints": failed_joint_count,
		"joints_complete": joints_complete,
		"parameterized_joints": parameterized_joints,
		"parameters_complete": parameters_complete,
		"complete": assembly_complete,
		"part_nodes": body_summary["part_nodes"],
		"joint_nodes": joint_summary["joint_nodes"],
		"warnings": joint_summary["warnings"],
		"failed_connections": joint_summary["failed"]
	}
	robot.set_meta("assembly_summary", last_summary)
	robot.set_meta("robot_config", config.duplicate(true))
	return robot


func _create_bodies(parts: Array, robot: Node3D) -> Dictionary:
	var bodies := {}
	var part_nodes := []
	for part in parts:
		if not (part is Dictionary) or not part.has("id"):
			continue
		var body = _create_body(part)
		robot.add_child(body)
		bodies[part["id"]] = body
		part_nodes.append(_body_mapping(part, body))
	return {"bodies": bodies, "part_nodes": part_nodes}


func _valid_part_count(parts: Array) -> int:
	var count := 0
	for part in parts:
		if part is Dictionary and part.has("id"):
			count += 1
	return count


func _valid_connection_count(connections: Array) -> int:
	var count := 0
	for connection in connections:
		if connection is Dictionary:
			count += 1
	return count


func _parameterized_joint_count(joint_nodes: Array) -> int:
	var count := 0
	for joint_node in joint_nodes:
		if joint_node is Dictionary and not joint_node.get("applied_parameters", {}).is_empty():
			count += 1
	return count


func _create_body(part: Dictionary) -> RigidBody3D:
	var params = part.get("params", {})
	var body = RigidBody3D.new()
	body.name = str(part["id"])
	body.mass = float(params.get("mass", 1.0))
	body.position = _vector3(params.get("position", [0.0, 0.0, 0.0]))
	body.rotation = _vector3(params.get("rotation", [0.0, 0.0, 0.0]))
	var collision = _collision_for_part(part)
	collision.name = "Collision"
	body.add_child(collision)
	var mesh = _mesh_for_part(part)
	mesh.name = "Mesh"
	body.add_child(mesh)
	body.set_meta("part_config", part.duplicate(true))
	return body


func _collision_for_part(part: Dictionary) -> CollisionShape3D:
	var collision = CollisionShape3D.new()
	collision.shape = _shape_for_part(part)
	return collision


func _mesh_for_part(part: Dictionary) -> MeshInstance3D:
	var mesh_instance = MeshInstance3D.new()
	mesh_instance.mesh = _mesh_for_part_shape(part)
	return mesh_instance


func _shape_for_part(part: Dictionary) -> Shape3D:
	var params = part.get("params", {})
	match part.get("shape", "box"):
		"capsule":
			var shape = CapsuleShape3D.new()
			shape.radius = float(params.get("radius", 0.04))
			shape.height = float(params.get("length", 0.3))
			return shape
		"cylinder":
			var shape = CylinderShape3D.new()
			shape.radius = float(params.get("radius", 0.04))
			shape.height = float(params.get("length", 0.3))
			return shape
		"sphere":
			var shape = SphereShape3D.new()
			shape.radius = float(params.get("radius", 0.05))
			return shape
		_:
			var shape = BoxShape3D.new()
			shape.size = _vector3(params.get("size", [0.2, 0.2, 0.2]))
			return shape


func _mesh_for_part_shape(part: Dictionary) -> Mesh:
	var params = part.get("params", {})
	match part.get("shape", "box"):
		"capsule":
			var mesh = CapsuleMesh.new()
			mesh.radius = float(params.get("radius", 0.04))
			mesh.height = float(params.get("length", 0.3))
			return mesh
		"cylinder":
			var mesh = CylinderMesh.new()
			mesh.top_radius = float(params.get("radius", 0.04))
			mesh.bottom_radius = float(params.get("radius", 0.04))
			mesh.height = float(params.get("length", 0.3))
			return mesh
		"sphere":
			var mesh = SphereMesh.new()
			mesh.radius = float(params.get("radius", 0.05))
			mesh.height = float(params.get("radius", 0.05)) * 2.0
			return mesh
		_:
			var mesh = BoxMesh.new()
			mesh.size = _vector3(params.get("size", [0.2, 0.2, 0.2]))
			return mesh


func _create_joints(connections: Array, robot: Node3D, bodies: Dictionary) -> Dictionary:
	var created = 0
	var failed := []
	var warnings := []
	var joint_nodes := []
	for connection in connections:
		if not (connection is Dictionary):
			continue
		if not bodies.has(connection.get("from")) or not bodies.has(connection.get("to")):
			failed.append(connection.get("name", "unnamed"))
			continue
		var joint = _create_joint(connection, bodies)
		robot.add_child(joint)
		created += 1
		joint_nodes.append(_joint_mapping(connection, joint))
		if connection.get("joint_type", "fixed") == "fixed":
			warnings.append("%s uses Generic6DOFJoint3D with locked linear and angular axes" % joint.name)
	return {
		"created": created,
		"failed": failed,
		"warnings": warnings,
		"joint_nodes": joint_nodes
	}


func _create_joint(connection: Dictionary, bodies: Dictionary) -> Joint3D:
	var joint = _new_joint(connection.get("joint_type", "fixed"))
	joint.name = str(connection.get("name", "%s_to_%s" % [connection.get("from"), connection.get("to")]))
	joint.node_a = bodies[connection["from"]].get_path()
	joint.node_b = bodies[connection["to"]].get_path()
	joint.position = _vector3(connection.get("origin", [0.0, 0.0, 0.0]))
	_orient_joint(joint, _vector3(connection.get("axis", [0.0, 1.0, 0.0])))
	_apply_joint_params(joint, connection)
	joint.set_meta("connection_config", connection.duplicate(true))
	joint.set_meta("applied_parameters", _joint_applied_params(connection, joint))
	return joint


func _body_mapping(part: Dictionary, body: RigidBody3D) -> Dictionary:
	var collision = body.get_node_or_null("Collision")
	var mesh = body.get_node_or_null("Mesh")
	return {
		"part_id": str(part["id"]),
		"part_type": str(part.get("type", "")),
		"shape": str(part.get("shape", "box")),
		"body_node": str(body.get_path()),
		"body_class": body.get_class(),
		"collision_node": str(collision.get_path()) if collision else "",
		"collision_shape": collision.shape.get_class() if collision and collision.shape else "",
		"collision_parameters": _shape_parameters(collision.shape) if collision and collision.shape else {},
		"mesh_node": str(mesh.get_path()) if mesh else "",
		"mesh_type": mesh.mesh.get_class() if mesh and mesh.mesh else "",
		"mesh_parameters": _mesh_parameters(mesh.mesh) if mesh and mesh.mesh else {},
		"mass": body.mass,
		"position": [body.position.x, body.position.y, body.position.z],
		"rotation": [body.rotation.x, body.rotation.y, body.rotation.z]
	}


func _shape_parameters(shape: Shape3D) -> Dictionary:
	if shape is BoxShape3D:
		return {"size": [shape.size.x, shape.size.y, shape.size.z]}
	if shape is CapsuleShape3D:
		return {"radius": shape.radius, "height": shape.height}
	if shape is CylinderShape3D:
		return {"radius": shape.radius, "height": shape.height}
	if shape is SphereShape3D:
		return {"radius": shape.radius}
	return {}


func _mesh_parameters(mesh: Mesh) -> Dictionary:
	if mesh is BoxMesh:
		return {"size": [mesh.size.x, mesh.size.y, mesh.size.z]}
	if mesh is CapsuleMesh:
		return {"radius": mesh.radius, "height": mesh.height}
	if mesh is CylinderMesh:
		return {
			"top_radius": mesh.top_radius,
			"bottom_radius": mesh.bottom_radius,
			"height": mesh.height
		}
	if mesh is SphereMesh:
		return {"radius": mesh.radius, "height": mesh.height}
	return {}


func _joint_mapping(connection: Dictionary, joint: Joint3D) -> Dictionary:
	return {
		"connection_name": str(connection.get("name", joint.name)),
		"joint_node": str(joint.get_path()),
		"joint_class": joint.get_class(),
		"joint_type": str(connection.get("joint_type", "fixed")),
		"from": str(connection.get("from", "")),
		"to": str(connection.get("to", "")),
		"node_a": str(joint.node_a),
		"node_b": str(joint.node_b),
		"origin": [joint.position.x, joint.position.y, joint.position.z],
		"axis": connection.get("axis", [0.0, 1.0, 0.0]),
		"applied_parameters": joint.get_meta("applied_parameters", {})
	}


func _new_joint(joint_type: String) -> Joint3D:
	match joint_type:
		"hinge", "revolute":
			return HingeJoint3D.new()
		"slider", "prismatic":
			return SliderJoint3D.new()
		_:
			return Generic6DOFJoint3D.new()


func _apply_joint_params(joint: Joint3D, connection: Dictionary) -> void:
	if joint is HingeJoint3D:
		_apply_hinge_params(joint, connection)
	elif joint is SliderJoint3D:
		_apply_slider_params(joint, connection)
	elif joint is Generic6DOFJoint3D:
		_apply_fixed_params(joint)


func _apply_hinge_params(joint: HingeJoint3D, connection: Dictionary) -> void:
	var limits = connection.get("limits", {})
	joint.set_flag(HingeJoint3D.FLAG_USE_LIMIT, true)
	joint.set_param(HingeJoint3D.PARAM_LIMIT_LOWER, float(limits.get("lower", -1.57)))
	joint.set_param(HingeJoint3D.PARAM_LIMIT_UPPER, float(limits.get("upper", 1.57)))
	var motor = connection.get("motor", {})
	joint.set_flag(HingeJoint3D.FLAG_ENABLE_MOTOR, bool(motor.get("enabled", false)))
	joint.set_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY, float(motor.get("target_velocity", 0.0)))
	joint.set_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE, float(motor.get("max_impulse", 100.0)))


func _apply_slider_params(joint: SliderJoint3D, connection: Dictionary) -> void:
	var limits = connection.get("limits", {})
	joint.set_param(SliderJoint3D.PARAM_LINEAR_LIMIT_LOWER, float(limits.get("lower", -0.5)))
	joint.set_param(SliderJoint3D.PARAM_LINEAR_LIMIT_UPPER, float(limits.get("upper", 0.5)))


func _apply_fixed_params(joint: Generic6DOFJoint3D) -> void:
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT, true)
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT, 0.0)
	joint.set_param_x(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT, 0.0)
	joint.set_flag_x(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	joint.set_flag_y(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	joint.set_flag_z(Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT, true)
	joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT, 0.0)
	joint.set_param_x(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
	joint.set_param_y(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)
	joint.set_param_z(Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT, 0.0)


func _joint_applied_params(connection: Dictionary, joint: Joint3D) -> Dictionary:
	var limits = connection.get("limits", {})
	var motor = connection.get("motor", {})
	var dynamics = connection.get("dynamics", {})
	var applied := {
		"source": {
			"limits": limits.duplicate(true) if limits is Dictionary else {},
			"motor": motor.duplicate(true) if motor is Dictionary else {},
			"dynamics": dynamics.duplicate(true) if dynamics is Dictionary else {}
		},
		"runtime": {}
	}
	if joint is HingeJoint3D:
		applied["runtime"] = {
			"limit_enabled": joint.get_flag(HingeJoint3D.FLAG_USE_LIMIT),
			"limit_lower": joint.get_param(HingeJoint3D.PARAM_LIMIT_LOWER),
			"limit_upper": joint.get_param(HingeJoint3D.PARAM_LIMIT_UPPER),
			"motor_supported": true,
			"motor_enabled": joint.get_flag(HingeJoint3D.FLAG_ENABLE_MOTOR),
			"motor_target_velocity": joint.get_param(HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY),
			"motor_max_impulse": joint.get_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE),
			"dynamics_configured": dynamics is Dictionary and not dynamics.is_empty(),
			"damping_requested": float(dynamics.get("damping", 0.0)) if dynamics is Dictionary else 0.0,
			"damping_applied": false
		}
	elif joint is SliderJoint3D:
		applied["runtime"] = {
			"limit_enabled": true,
			"limit_lower": joint.get_param(SliderJoint3D.PARAM_LINEAR_LIMIT_LOWER),
			"limit_upper": joint.get_param(SliderJoint3D.PARAM_LINEAR_LIMIT_UPPER),
			"motor_supported": false,
			"motor_enabled": false,
			"dynamics_configured": dynamics is Dictionary and not dynamics.is_empty(),
			"damping_applied": false
		}
	else:
		applied["runtime"] = {
			"limit_enabled": true,
			"linear_limit_enabled": _generic_axis_flags(joint, Generic6DOFJoint3D.FLAG_ENABLE_LINEAR_LIMIT),
			"linear_lower": _generic_axis_params(joint, Generic6DOFJoint3D.PARAM_LINEAR_LOWER_LIMIT),
			"linear_upper": _generic_axis_params(joint, Generic6DOFJoint3D.PARAM_LINEAR_UPPER_LIMIT),
			"angular_limit_enabled": _generic_axis_flags(joint, Generic6DOFJoint3D.FLAG_ENABLE_ANGULAR_LIMIT),
			"angular_lower": _generic_axis_params(joint, Generic6DOFJoint3D.PARAM_ANGULAR_LOWER_LIMIT),
			"angular_upper": _generic_axis_params(joint, Generic6DOFJoint3D.PARAM_ANGULAR_UPPER_LIMIT),
			"motor_supported": false,
			"motor_enabled": false,
			"dynamics_configured": dynamics is Dictionary and not dynamics.is_empty(),
			"damping_applied": false,
			"fixed_approximation": true,
			"fixed_lock_applied": true
		}
	return applied


func _generic_axis_flags(joint: Generic6DOFJoint3D, flag: int) -> Dictionary:
	return {
		"x": joint.get_flag_x(flag),
		"y": joint.get_flag_y(flag),
		"z": joint.get_flag_z(flag)
	}


func _generic_axis_params(joint: Generic6DOFJoint3D, param: int) -> Dictionary:
	return {
		"x": joint.get_param_x(param),
		"y": joint.get_param_y(param),
		"z": joint.get_param_z(param)
	}


func _orient_joint(joint: Joint3D, axis: Vector3) -> void:
	if axis.length() <= 0.0001:
		return
	var target = axis.normalized()
	if target.is_equal_approx(Vector3.RIGHT):
		return
	var rotation_axis = Vector3.RIGHT.cross(target)
	if rotation_axis.length() <= 0.0001:
		rotation_axis = Vector3.UP
	joint.basis = Basis(rotation_axis.normalized(), Vector3.RIGHT.angle_to(target))


func _remove_previous_robot(parent: Node3D, robot_name: String) -> void:
	var old_robot = parent.get_node_or_null(robot_name)
	if old_robot:
		parent.remove_child(old_robot)
		old_robot.free()


func _vector3(raw_value) -> Vector3:
	if raw_value is Array and raw_value.size() >= 3:
		return Vector3(float(raw_value[0]), float(raw_value[1]), float(raw_value[2]))
	return Vector3.ZERO
