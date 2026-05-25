from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
ROBOT_CONFIG = ROOT / "config" / "robot.json"
ACTUATOR_CONFIG = ROOT / "config" / "actuators.json"
ACTUATOR_REPORT = ROOT / "test_env" / "actuator_physics_report.json"
SIM_REPORT = ROOT / "test_env" / "biped_sim_report.json"
CONTACT_REPORT = ROOT / "test_env" / "contact_stability_report.json"
DEFAULT_OUTPUT = ROOT / "test_env" / "component_parameter_log.json"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def segment_log(robot: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for segment in robot["segments"]:
        width, height, depth = [float(value) for value in segment["size"]]
        mass = float(segment["mass"])
        volume = width * height * depth
        rows.append(
            {
                "component_type": "body_segment",
                "name": segment["name"],
                "size_meters": {"x": width, "y": height, "z": depth},
                "mass_kg": mass,
                "approx_volume_m3": round(volume, 6),
                "approx_density_kg_per_m3": round(mass / volume, 3) if volume else None,
                "simulation_role": "visual_mass_distribution",
            }
        )
    return rows


def joint_log(
    robot: dict[str, Any],
    actuator_config: dict[str, Any],
    actuator_report: dict[str, Any],
) -> list[dict[str, Any]]:
    actuator_by_name = {joint["name"]: joint for joint in actuator_config["joints"]}
    observed_by_name = actuator_report.get("joint_summaries", {})
    defaults = actuator_config["defaults"]
    rows = []
    for joint in robot["joints"]:
        name = joint["name"]
        actuator = actuator_by_name[name]
        observed = observed_by_name.get(name, {})
        rows.append(
            {
                "component_type": "actuated_joint",
                "name": name,
                "axis": joint["axis"],
                "limit_degrees": {"min": joint["limit"][0], "max": joint["limit"][1]},
                "command_profile": {
                    "target_amplitude_degrees": actuator["target_amplitude_degrees"],
                    "phase_offset_degrees": actuator["phase_offset_degrees"],
                    "positive_only": bool(actuator.get("positive_only", False)),
                },
                "actuator_model": {
                    "kp": defaults["kp"],
                    "kd": defaults["kd"],
                    "inertia_kg_m2": defaults["inertia_kg_m2"],
                    "damping_nms_per_rad": defaults["damping_nms_per_rad"],
                    "gravity_load_nm": actuator["gravity_load_nm"],
                    "torque_limit_nm": defaults["torque_limit_nm"],
                    "velocity_limit_degrees_per_second": defaults["velocity_limit_degrees_per_second"],
                    "motor_constant_nm_per_amp": defaults["motor_constant_nm_per_amp"],
                    "winding_resistance_ohm": defaults["winding_resistance_ohm"],
                    "thermal_capacity_j_per_c": defaults["thermal_capacity_j_per_c"],
                    "thermal_resistance_c_per_w": defaults["thermal_resistance_c_per_w"],
                },
                "latest_simulation_metrics": {
                    "max_tracking_error_degrees": observed.get("max_tracking_error_degrees"),
                    "max_current_amp": observed.get("max_current_amp"),
                    "max_torque_nm": observed.get("max_torque_nm"),
                    "max_temperature_c": observed.get("max_temperature_c"),
                    "energy_wh": observed.get("energy_wh"),
                    "saturation_count": observed.get("saturation_count"),
                },
            }
        )
    return rows


def build_log() -> dict[str, Any]:
    robot = load_json(ROBOT_CONFIG)
    actuator_config = load_json(ACTUATOR_CONFIG)
    actuator_report = load_json(ACTUATOR_REPORT)
    sim_report = load_json(SIM_REPORT)
    contact_report = load_json(CONTACT_REPORT)
    return {
        "schema_version": "biped-component-parameter-log.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "scope": "biped_robot software simulation",
        "robot": robot["name"],
        "body": robot["body"],
        "simulation_summary": {
            "biped_simulation_status": sim_report["status"],
            "actuator_physics_status": actuator_report["status"],
            "contact_stability_status": contact_report["status"],
            "duration_s": sim_report["duration_s"],
            "control_hz": actuator_config["control_hz"],
            "actuator_samples_per_joint": actuator_report["samples_per_joint"],
        },
        "body_segments": segment_log(robot),
        "actuated_joints": joint_log(robot, actuator_config, actuator_report),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build per-component parameter logs from the latest local simulation.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    payload = build_log()
    write_json(args.output, payload)
    print(json.dumps({"status": "passed", "output": str(args.output), "segments": len(payload["body_segments"]), "joints": len(payload["actuated_joints"])}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
