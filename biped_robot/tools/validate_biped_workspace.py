from __future__ import annotations

import json
import py_compile
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_FILES = [
    "LOCAL_PLAN.md",
    "README.md",
    "config/robot.json",
    "config/actuators.json",
    "config/communication.json",
    "config/godot_io_input.json",
    "config/mountain_terrain.json",
    "godot/project.godot",
    "godot/scenes/biped_mountain_demo.tscn",
    "godot/scripts/biped_mountain_demo.gd",
    "contracts/hardware_interface.json",
    "tools/check_contact_stability.py",
    "tools/build_hardware_gap_report.py",
    "tools/simulate_actuator_physics.py",
    "tools/build_component_parameter_log.py",
    "tools/simulate_communication.py",
    "tools/validate_godot_io.py",
    "tools/build_retention_manifest.py",
    "tools/run_local_acceptance.py",
    "tools/simulate_biped.py",
    "tools/validate_biped_workspace.py",
]


def read_json(relative_path: str) -> dict[str, Any]:
    with (ROOT / relative_path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def fail(message: str) -> None:
    raise SystemExit(f"validation failed: {message}")


def validate_required_files() -> None:
    missing = [path for path in REQUIRED_FILES if not (ROOT / path).is_file()]
    if missing:
        fail(f"missing required files: {missing}")


def validate_robot_contract() -> None:
    robot = read_json("config/robot.json")
    if robot.get("schema_version") != "biped-robot.v1":
        fail("robot schema_version must be biped-robot.v1")
    if robot.get("name") != "mountain_humanoid_biped":
        fail("robot name must be mountain_humanoid_biped")
    if len(robot.get("segments", [])) < 10:
        fail("robot must define at least 10 humanoid segments")
    joint_names = {joint.get("name") for joint in robot.get("joints", [])}
    required_joints = {"left_hip_pitch", "left_knee_pitch", "right_hip_pitch", "right_knee_pitch"}
    if not required_joints.issubset(joint_names):
        fail(f"robot missing required joints: {sorted(required_joints - joint_names)}")
    gait = robot.get("gait", {})
    if gait.get("stability_margin_min", 0.0) < 0.1:
        fail("gait stability_margin_min must be at least 0.1")


def validate_terrain_contract() -> None:
    terrain = read_json("config/mountain_terrain.json")
    if terrain.get("schema_version") != "mountain-terrain.v1":
        fail("terrain schema_version must be mountain-terrain.v1")
    if terrain.get("grid_resolution", 0) < 24:
        fail("terrain grid_resolution must be at least 24")
    acceptance = terrain.get("acceptance", {})
    if acceptance.get("max_allowed_slope_degrees", 0.0) <= 0.0:
        fail("terrain acceptance must define max_allowed_slope_degrees")
    if acceptance.get("min_stability_margin", 0.0) < 0.1:
        fail("terrain min_stability_margin must be at least 0.1")


def validate_actuator_contract() -> None:
    actuators = read_json("config/actuators.json")
    if actuators.get("schema_version") != "biped-actuators.v1":
        fail("actuator schema_version must be biped-actuators.v1")
    if int(actuators.get("control_hz", 0)) < 60:
        fail("actuator control_hz must be at least 60")
    defaults = actuators.get("defaults", {})
    for required in ["kp", "kd", "torque_limit_nm", "motor_constant_nm_per_amp", "max_temperature_c"]:
        if float(defaults.get(required, 0.0)) <= 0.0:
            fail(f"actuator defaults missing positive value: {required}")
    acceptance = actuators.get("acceptance", {})
    for required in ["max_tracking_error_degrees", "max_temperature_c", "max_current_amp", "max_saturation_ratio"]:
        if required not in acceptance:
            fail(f"actuator acceptance missing: {required}")
    actuator_names = {joint.get("name") for joint in actuators.get("joints", [])}
    robot_names = {joint.get("name") for joint in read_json("config/robot.json").get("joints", [])}
    if actuator_names != robot_names:
        fail(f"actuator joint set must match robot joints: {sorted(robot_names - actuator_names)}")
    retention = actuators.get("retention", {})
    if retention.get("telemetry_format") != "jsonl":
        fail("actuator telemetry retention format must be jsonl")
    if "component_parameter_log.json" not in retention.get("required_files", []):
        fail("actuator retention must include component_parameter_log.json")


def validate_godot_io_contract() -> None:
    contract = read_json("config/godot_io_input.json")
    if contract.get("schema_version") != "biped-godot-io-input.v1":
        fail("Godot IO schema_version must be biped-godot-io-input.v1")
    simulation = contract.get("simulation", {})
    if float(simulation.get("duration_seconds", 0.0)) <= 0.0:
        fail("Godot IO simulation duration must be positive")
    if float(simulation.get("telemetry_interval_seconds", 0.0)) <= 0.0:
        fail("Godot IO telemetry interval must be positive")
    outputs = contract.get("outputs", {})
    for required in ["telemetry_path", "report_path"]:
        if not str(outputs.get(required, "")).startswith("res://../test_env/"):
            fail(f"Godot IO output must stay inside biped_robot/test_env: {required}")
    if not contract.get("commands"):
        fail("Godot IO input must include at least one command")


def validate_communication_contract() -> None:
    contract = read_json("config/communication.json")
    if contract.get("schema_version") != "biped-communication.v1":
        fail("communication schema_version must be biped-communication.v1")
    transport = contract.get("transport", {})
    for required in ["command_channel", "telemetry_channel", "ack_channel"]:
        if not transport.get(required):
            fail(f"communication transport missing: {required}")
    for required in ["base_latency_ms", "jitter_ms", "ack_latency_ms"]:
        if float(transport.get(required, -1.0)) < 0.0:
            fail(f"communication transport must define non-negative {required}")
    acceptance = contract.get("acceptance", {})
    for required in [
        "min_command_delivery_ratio",
        "min_ack_ratio",
        "min_telemetry_delivery_ratio",
        "max_command_latency_ms",
        "max_ack_latency_ms",
        "max_telemetry_latency_ms",
        "max_jitter_ms",
    ]:
        if required not in acceptance:
            fail(f"communication acceptance missing: {required}")
    outputs = contract.get("outputs", {})
    for required in ["events_path", "report_path"]:
        if not str(outputs.get(required, "")).startswith("test_env/"):
            fail(f"communication output must stay inside biped_robot/test_env: {required}")


def validate_godot_contract() -> None:
    project_text = (ROOT / "godot" / "project.godot").read_text(encoding="utf-8")
    scene_text = (ROOT / "godot" / "scenes" / "biped_mountain_demo.tscn").read_text(encoding="utf-8")
    script_text = (ROOT / "godot" / "scripts" / "biped_mountain_demo.gd").read_text(encoding="utf-8")
    if 'run/main_scene="res://scenes/biped_mountain_demo.tscn"' not in project_text:
        fail("Godot project must launch biped_mountain_demo.tscn")
    if "res://scripts/biped_mountain_demo.gd" not in scene_text:
        fail("Godot scene must attach biped_mountain_demo.gd")
    for expected in ["HumanoidBipedRobot", "ProceduralMountainTerrain", "_animate_robot", "_record_godot_io_telemetry"]:
        if expected not in script_text:
            fail(f"Godot script missing expected marker: {expected}")


def validate_hardware_boundary_contract() -> None:
    contract = read_json("contracts/hardware_interface.json")
    if contract.get("schema_version") != "biped-hardware-interface.v1":
        fail("hardware interface schema_version must be biped-hardware-interface.v1")
    boundary = contract.get("ownership_boundary", {})
    if boundary.get("current_folder") != "biped_robot":
        fail("hardware boundary must keep current_folder as biped_robot")
    if boundary.get("outside_changes_require_user_approval") is not True:
        fail("hardware boundary must require user approval for outside changes")
    safe_defaults = contract.get("safe_defaults", {})
    if safe_defaults.get("hardware_commands_enabled") is not False:
        fail("hardware commands must stay disabled until real hardware integration is approved")
    capability_names = {capability.get("name") for capability in contract.get("required_capabilities", [])}
    for required in {"motor_transport", "joint_state_feedback", "imu_feedback", "ros2_bridge"}:
        if required not in capability_names:
            fail(f"hardware boundary missing capability: {required}")


def validate_python_tools() -> None:
    for relative_path in [
        "tools/simulate_biped.py",
        "tools/validate_biped_workspace.py",
        "tools/check_contact_stability.py",
        "tools/build_hardware_gap_report.py",
        "tools/simulate_actuator_physics.py",
        "tools/build_component_parameter_log.py",
        "tools/simulate_communication.py",
        "tools/validate_godot_io.py",
        "tools/build_retention_manifest.py",
        "tools/run_local_acceptance.py",
    ]:
        py_compile.compile(str(ROOT / relative_path), doraise=True)


def main() -> int:
    validate_required_files()
    validate_robot_contract()
    validate_terrain_contract()
    validate_actuator_contract()
    validate_godot_io_contract()
    validate_communication_contract()
    validate_godot_contract()
    validate_hardware_boundary_contract()
    validate_python_tools()
    print("validation passed: biped_robot workspace is complete")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
