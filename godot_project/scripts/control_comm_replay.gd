extends SceneTree

const LOG_VERSION := "godot_control_comm_simulation_log.v1"
const ENVELOPE_VERSION := "control_message_envelope.v1"


func _init() -> void:
	var args := OS.get_cmdline_user_args()
	var scenario_path := _arg_value(args, "--control-comm-scenario", "")
	var output_path := _arg_value(args, "--control-comm-log", "user://godot_control_comm_simulation_log.json")
	var scenario: Dictionary = _default_scenario()
	if scenario_path != "":
		var loaded: Variant = _read_json_file(scenario_path)
		if typeof(loaded) == TYPE_DICTIONARY:
			scenario = loaded
	var log_payload := build_log_payload(scenario)
	_write_json_file(output_path, log_payload)
	quit(0)


func build_log_payload(scenario: Dictionary) -> Dictionary:
	var cycle_count := int(scenario.get("cycle_count", 0))
	var cycle_period_ns := int(scenario.get("cycle_period_ns", 0))
	var events: Array = []
	for cycle_index in range(cycle_count):
		var timestamp_ns := cycle_index * cycle_period_ns
		var envelope := {
			"schema_version": ENVELOPE_VERSION,
			"topic": str(scenario.get("topic", "")),
			"sequence": cycle_index,
			"timestamp_ns": timestamp_ns,
			"source": str(scenario.get("source", "")),
			"target": str(scenario.get("target", "")),
			"payload_type": str(scenario.get("payload_type", "")),
			"payload": scenario.get("command", {}),
			"metadata": {
				"scenario_id": str(scenario.get("scenario_id", "")),
				"cycle_index": cycle_index,
			},
		}
		events.append({
			"cycle_index": cycle_index,
			"event_type": "message_delivered",
			"evidence_source": "godot_script",
			"envelope": envelope,
			"godot_node_path": "/root",
			"script_source": "res://scripts/control_comm_replay.gd",
		})
	return {
		"log_version": LOG_VERSION,
		"status": "success",
		"evidence_source": "godot_script",
		"script_name": "control_comm_replay.gd",
		"godot_profile": {
			"mode": "script",
			"executable": OS.get_executable_path(),
			"project": ProjectSettings.globalize_path("res://"),
		},
		"scenario_id": str(scenario.get("scenario_id", "")),
		"cycle_count": cycle_count,
		"message_event_count": events.size(),
		"events": events,
		"artifact_paths": {
			"log": "provided_by_runner"
		},
		"residual_risks": [
			"real_hardware_transport_not_run"
		],
		"errors": [],
	}


func _default_scenario() -> Dictionary:
	return {
		"schema_version": "control_comm_simulation_scenario.v1",
		"scenario_id": "default_joint_command_stream",
		"cycle_count": 4,
		"cycle_period_ns": 10000000,
		"jitter_budget_ns": 0,
		"source": "controller",
		"target": "joint_endpoint.left_hip",
		"topic": "agi/control/joint/left_hip/command",
		"payload_type": "joint_velocity_command",
		"command": {
			"joint_name": "left_hip",
			"target_velocity_rad_s": 0.25,
			"target_torque_nm": 1.5,
		},
	}


func _arg_value(args: Array, name: String, default_value: String) -> String:
	var index := args.find(name)
	if index == -1 or index + 1 >= args.size():
		return default_value
	return str(args[index + 1])


func _read_json_file(path: String) -> Variant:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		return {}
	var text := file.get_as_text()
	var parsed: Variant = JSON.parse_string(text)
	if parsed == null:
		return {}
	return parsed


func _write_json_file(path: String, payload: Dictionary) -> void:
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file == null:
		push_error("Unable to write control communication log: " + path)
		return
	file.store_string(JSON.stringify(payload, "\t"))
