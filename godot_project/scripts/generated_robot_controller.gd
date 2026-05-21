extends Node
class_name GeneratedRobotController

var robot_root: Node3D = null
var joints: Dictionary = {}
var joint_targets: Dictionary = {}
var current_control_state := {
	"linear_x": 0.0,
	"linear_y": 0.0,
	"angular_z": 0.0,
	"gait": "",
	"pose": "",
	"step_count": 0
}


func configure(root: Node3D, control_config: Dictionary = {}) -> void:
	robot_root = root
	_collect_joints(root)
	_apply_control_defaults(control_config)


func apply_action(action) -> void:
	if action is Dictionary:
		for joint_name in action.keys():
			if joints.has(joint_name):
				joint_targets[joint_name] = float(action[joint_name])
	elif action is Array:
		var names = joints.keys()
		for index in range(min(action.size(), names.size())):
			joint_targets[names[index]] = float(action[index])


func apply_instruction_steps(steps: Array) -> void:
	current_control_state["step_count"] = steps.size()
	for step in steps:
		_apply_instruction_step(step)


func reset_pose() -> void:
	if robot_root == null:
		return
	for body in _find_bodies(robot_root):
		body.linear_velocity = Vector3.ZERO
		body.angular_velocity = Vector3.ZERO
	for joint_name in joints.keys():
		joint_targets[joint_name] = 0.0


func get_sensor_data() -> Dictionary:
	var bodies = _find_bodies(robot_root) if robot_root else []
	var root_position = robot_root.global_position if robot_root else Vector3.ZERO
	return {
		"timestamp": Time.get_ticks_msec() / 1000.0,
		"body_count": bodies.size(),
		"joint_count": joints.size(),
		"position": [root_position.x, root_position.y, root_position.z],
		"body_states": _body_states(bodies),
		"joint_states": _joint_states(),
		"instruction_runtime": current_control_state,
		"joints": _joint_snapshot()
	}


func _physics_process(_delta: float) -> void:
	for joint_name in joints.keys():
		var joint = joints[joint_name]
		if joint is HingeJoint3D:
			joint.set_param(
				HingeJoint3D.PARAM_MOTOR_TARGET_VELOCITY,
				float(joint_targets.get(joint_name, 0.0))
			)


func _collect_joints(root: Node) -> void:
	joints.clear()
	for node in _walk_nodes(root):
		if node is Joint3D:
			joints[node.name] = node
			joint_targets[node.name] = _initial_joint_target(node)


func _apply_control_defaults(control_config: Dictionary) -> void:
	var configured_joints = control_config.get("joints", {})
	if not (configured_joints is Dictionary):
		return
	for joint_name in configured_joints.keys():
		if joints.has(joint_name):
			joints[joint_name].set_meta("control", configured_joints[joint_name])


func _apply_instruction_step(step: Dictionary) -> void:
	match step.get("kind", ""):
		"set_velocity":
			current_control_state["linear_x"] = float(step.get("linear_x", 0.0))
			current_control_state["linear_y"] = float(step.get("linear_y", 0.0))
			current_control_state["angular_z"] = float(step.get("angular_z", 0.0))
		"set_gait":
			current_control_state["gait"] = str(step.get("gait", ""))
		"set_pose":
			current_control_state["pose"] = str(step.get("pose", ""))
		"joint_targets":
			apply_action(step.get("targets", {}))
		"emergency_stop":
			for joint_name in joint_targets.keys():
				joint_targets[joint_name] = 0.0
			current_control_state["linear_x"] = 0.0
			current_control_state["linear_y"] = 0.0
			current_control_state["angular_z"] = 0.0


func _joint_snapshot() -> Dictionary:
	var snapshot := {}
	for joint_name in joints.keys():
		snapshot[joint_name] = {"target_velocity": joint_targets.get(joint_name, 0.0)}
	return snapshot


func _body_states(bodies: Array) -> Dictionary:
	var states := {}
	for body in bodies:
		states[body.name] = {
			"position": _vector3_array(body.global_position),
			"rotation": _vector3_array(body.global_rotation),
			"linear_velocity": _vector3_array(body.linear_velocity),
			"angular_velocity": _vector3_array(body.angular_velocity),
			"mass": body.mass,
			"contact_count": body.get_contact_count(),
			"contacts": _contact_names(body)
		}
	return states


func _contact_names(body: RigidBody3D) -> Array:
	var names := []
	for collider in body.get_colliding_bodies():
		if collider is Node:
			names.append(collider.name)
	return names


func _joint_states() -> Dictionary:
	var states := {}
	for joint_name in joints.keys():
		var joint = joints[joint_name]
		var relative_angle = _joint_relative_angle(joint)
		var connection_config = joint.get_meta("connection_config", {})
		states[joint_name] = {
			"joint_class": joint.get_class(),
			"target_velocity": joint_targets.get(joint_name, 0.0),
			"node_a": str(joint.node_a),
			"node_b": str(joint.node_b),
			"origin": _vector3_array(joint.global_position),
			"axis": _vector3_array(joint.global_basis.x.normalized()),
			"relative_angle": relative_angle,
			"limits": _joint_limit_state(relative_angle, connection_config),
			"applied_parameters": _applied_parameters_with_runtime(joint),
			"control_parameters": joint.get_meta("control", {}).duplicate(true),
			"endpoint_distance": _joint_endpoint_distance(joint),
			"body_a": _joint_endpoint_state(joint.node_a),
			"body_b": _joint_endpoint_state(joint.node_b)
		}
	return states


func _initial_joint_target(joint: Joint3D) -> float:
	var connection_config = joint.get_meta("connection_config", {})
	if not (connection_config is Dictionary):
		return 0.0
	var motor = connection_config.get("motor", {})
	if motor is Dictionary:
		return float(motor.get("target_velocity", 0.0))
	return 0.0


func _applied_parameters_with_runtime(joint: Joint3D) -> Dictionary:
	var applied = joint.get_meta("applied_parameters", {}).duplicate(true)
	var runtime = applied.get("runtime", {})
	if not (runtime is Dictionary):
		runtime = {}
	if joint is HingeJoint3D:
		runtime["limit_enabled"] = joint.get_flag(HingeJoint3D.FLAG_USE_LIMIT)
		runtime["limit_lower"] = joint.get_param(HingeJoint3D.PARAM_LIMIT_LOWER)
		runtime["limit_upper"] = joint.get_param(HingeJoint3D.PARAM_LIMIT_UPPER)
		runtime["motor_enabled"] = joint.get_flag(HingeJoint3D.FLAG_ENABLE_MOTOR)
		runtime["motor_target_velocity"] = joint_targets.get(joint.name, 0.0)
		runtime["motor_max_impulse"] = joint.get_param(HingeJoint3D.PARAM_MOTOR_MAX_IMPULSE)
	elif joint is SliderJoint3D:
		runtime["limit_enabled"] = true
		runtime["limit_lower"] = joint.get_param(SliderJoint3D.PARAM_LINEAR_LIMIT_LOWER)
		runtime["limit_upper"] = joint.get_param(SliderJoint3D.PARAM_LINEAR_LIMIT_UPPER)
	applied["runtime"] = runtime
	return applied


func _joint_limit_state(relative_angle: float, connection_config: Dictionary) -> Dictionary:
	if str(connection_config.get("joint_type", "")) == "fixed":
		return {"configured": false, "not_applicable": true, "reason": "fixed_joint"}
	var limits = connection_config.get("limits", {})
	if not (limits is Dictionary) or not limits.has("lower") or not limits.has("upper"):
		return {"configured": false}
	var lower = float(limits.get("lower", 0.0))
	var upper = float(limits.get("upper", 0.0))
	var margin_lower = relative_angle - lower
	var margin_upper = upper - relative_angle
	return {
		"configured": true,
		"lower": lower,
		"upper": upper,
		"margin_lower": margin_lower,
		"margin_upper": margin_upper,
		"min_margin": min(margin_lower, margin_upper),
		"violation": relative_angle < lower or relative_angle > upper
	}


func _joint_endpoint_state(endpoint_path: NodePath) -> Dictionary:
	var endpoint = _resolve_node_path(endpoint_path)
	if endpoint is Node3D:
		var state = {
			"path": str(endpoint_path),
			"name": endpoint.name,
			"node_class": endpoint.get_class(),
			"position": _vector3_array(endpoint.global_position),
			"rotation": _vector3_array(endpoint.global_rotation)
		}
		if endpoint is RigidBody3D:
			state["linear_velocity"] = _vector3_array(endpoint.linear_velocity)
			state["angular_velocity"] = _vector3_array(endpoint.angular_velocity)
			state["mass"] = endpoint.mass
		return state
	return {
		"path": str(endpoint_path),
		"missing": true
	}


func _joint_endpoint_distance(joint: Joint3D) -> float:
	var node_a = _resolve_node_path(joint.node_a)
	var node_b = _resolve_node_path(joint.node_b)
	if node_a is Node3D and node_b is Node3D:
		return node_a.global_position.distance_to(node_b.global_position)
	return -1.0


func _joint_relative_angle(joint: Joint3D) -> float:
	var node_a = _resolve_node_path(joint.node_a)
	var node_b = _resolve_node_path(joint.node_b)
	if not (node_a is Node3D and node_b is Node3D):
		return 0.0
	var axis = joint.global_basis.x.normalized()
	var reference_a = _project_on_plane(node_a.global_basis.y.normalized(), axis)
	var reference_b = _project_on_plane(node_b.global_basis.y.normalized(), axis)
	if reference_a.length() <= 0.0001 or reference_b.length() <= 0.0001:
		reference_a = _project_on_plane(node_a.global_basis.z.normalized(), axis)
		reference_b = _project_on_plane(node_b.global_basis.z.normalized(), axis)
	if reference_a.length() <= 0.0001 or reference_b.length() <= 0.0001:
		return 0.0
	reference_a = reference_a.normalized()
	reference_b = reference_b.normalized()
	var unsigned_angle = reference_a.angle_to(reference_b)
	var sign_value = sign(axis.dot(reference_a.cross(reference_b)))
	return unsigned_angle * sign_value


func _project_on_plane(value: Vector3, normal: Vector3) -> Vector3:
	return value - normal * value.dot(normal)


func _resolve_node_path(path: NodePath) -> Node:
	var node = get_node_or_null(path)
	if node == null and robot_root != null:
		node = robot_root.get_node_or_null(path)
	return node


func _find_bodies(root: Node) -> Array:
	return _walk_nodes(root).filter(func(node): return node is RigidBody3D)


func _vector3_array(value: Vector3) -> Array:
	return [value.x, value.y, value.z]


func _walk_nodes(root: Node) -> Array:
	var nodes := []
	var stack := [root]
	while not stack.is_empty():
		var node = stack.pop_back()
		nodes.append(node)
		for child in node.get_children():
			stack.append(child)
	return nodes
