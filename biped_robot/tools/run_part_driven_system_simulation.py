from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "part_driven_system_simulation_report.json"
DEFAULT_TRACE = ROOT / "test_env" / "part_driven_system_simulation_trace.jsonl"


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


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True))
            handle.write("\n")


def safe_status(payload: dict[str, Any]) -> str | None:
    return payload.get("status") or payload.get("overall_status") or payload.get("software_coverage_status") or ("present" if payload.get("schema_version") else None)


def segment_volume(segment: dict[str, Any]) -> float:
    x, y, z = [float(value) for value in segment["size"]]
    return x * y * z


def component_inventory(robot: dict[str, Any], actuators: dict[str, Any]) -> dict[str, Any]:
    segment_mass = sum(float(segment["mass"]) for segment in robot["segments"])
    body_mass = float(robot["body"]["mass"])
    actuator_names = {joint["name"] for joint in actuators["joints"]}
    robot_joint_names = {joint["name"] for joint in robot["joints"]}
    return {
        "body_segment_count": len(robot["segments"]),
        "joint_count": len(robot["joints"]),
        "actuated_joint_count": len(actuators["joints"]),
        "configured_body_mass_kg": body_mass,
        "segment_mass_sum_kg": round(segment_mass, 5),
        "segment_mass_gap_kg": round(body_mass - segment_mass, 5),
        "actuator_joint_set_matches_robot": actuator_names == robot_joint_names,
    }


def mass_and_geometry_summary(robot: dict[str, Any]) -> dict[str, Any]:
    volumes = [segment_volume(segment) for segment in robot["segments"]]
    densities = [
        float(segment["mass"]) / volume
        for segment, volume in zip(robot["segments"], volumes, strict=True)
        if volume > 0.0
    ]
    return {
        "body_height_m": robot["body"]["height"],
        "body_mass_kg": robot["body"]["mass"],
        "center_of_mass_height_m": robot["body"]["center_of_mass_height"],
        "total_segment_volume_m3": round(sum(volumes), 6),
        "min_segment_density_kg_per_m3": round(min(densities), 3),
        "max_segment_density_kg_per_m3": round(max(densities), 3),
        "avg_segment_density_kg_per_m3": round(sum(densities) / len(densities), 3),
    }


def actuator_system_summary(actuators: dict[str, Any], report: dict[str, Any]) -> dict[str, Any]:
    summaries = report.get("joint_summaries", {})
    max_torque = max(float(row.get("max_torque_nm", 0.0)) for row in summaries.values())
    max_temp = max(float(row.get("max_temperature_c", 0.0)) for row in summaries.values())
    total_energy = sum(float(row.get("energy_wh", 0.0)) for row in summaries.values())
    return {
        "status": report.get("status"),
        "control_hz": actuators["control_hz"],
        "samples_per_joint": report.get("samples_per_joint"),
        "joint_count": report.get("joint_count"),
        "max_tracking_error_degrees": report.get("metrics", {}).get("max_tracking_error_degrees"),
        "max_current_amp": report.get("metrics", {}).get("max_current_amp"),
        "max_torque_nm": round(max_torque, 5),
        "max_temperature_c": round(max_temp, 5),
        "total_energy_wh": round(total_energy, 6),
        "checks": report.get("checks", {}),
    }


def terrain_and_contact_summary(sim_report: dict[str, Any], contact_report: dict[str, Any]) -> dict[str, Any]:
    return {
        "simulation_status": sim_report.get("status"),
        "contact_status": contact_report.get("status"),
        "duration_s": sim_report.get("duration_s"),
        "motion_metrics": sim_report.get("metrics", {}),
        "contact_metrics": contact_report.get("metrics", {}),
        "checks": {
            "simulation": sim_report.get("checks", {}),
            "contact": contact_report.get("checks", {}),
        },
    }


def hardware_options(hardware: dict[str, Any], software_passed: bool) -> dict[str, Any]:
    return {
        "available_after_software_pass": software_passed,
        "current_status": "blocked_by_missing_external_runtime",
        "hardware_commands_enabled": hardware.get("safe_defaults", {}).get("hardware_commands_enabled"),
        "required_capabilities": [
            {
                "name": capability.get("name"),
                "current_status": capability.get("current_status"),
                "required_for_live_validation": True,
            }
            for capability in hardware.get("required_capabilities", [])
        ],
        "simulation_substitute_allowed_for_hardware_claims": False,
    }


def build_trace(report: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for name, status in report["source_artifact_status"].items():
        rows.append(
            {
                "event": "source_artifact_checked",
                "source": name,
                "status": status,
                "evidence_source": report["evidence_source"],
            }
        )
    for name, value in report["checks"].items():
        rows.append(
            {
                "event": "system_check_evaluated",
                "check": name,
                "passed": value,
                "evidence_source": report["evidence_source"],
            }
        )
    return rows


def build_report() -> dict[str, Any]:
    robot = load_json(ROOT / "config" / "robot.json")
    actuators = load_json(ROOT / "config" / "actuators.json")
    hardware = load_json(ROOT / "contracts" / "hardware_interface.json")
    reports = {
        "biped_simulation": load_json(ROOT / "test_env" / "biped_sim_report.json"),
        "actuator_physics": load_json(ROOT / "test_env" / "actuator_physics_report.json"),
        "contact_stability": load_json(ROOT / "test_env" / "contact_stability_report.json"),
        "godot_io": load_json(ROOT / "test_env" / "godot_io_report.json"),
        "communication": load_json(ROOT / "test_env" / "communication_report.json"),
        "public_data_replay": load_json(ROOT / "test_env" / "public_real_data_replay_report.json"),
        "component_parameters": load_json(ROOT / "test_env" / "component_parameter_log.json"),
        "hardware_gap": load_json(ROOT / "test_env" / "hardware_gap_report.json"),
    }
    inventory = component_inventory(robot, actuators)
    source_status = {name: safe_status(payload) for name, payload in reports.items()}
    checks = {
        "component_inventory_complete": inventory["body_segment_count"] >= 10 and inventory["joint_count"] == inventory["actuated_joint_count"],
        "actuator_joint_set_matches_robot": inventory["actuator_joint_set_matches_robot"],
        "body_mass_matches_segments_or_payload": abs(float(inventory["configured_body_mass_kg"]) - float(inventory["segment_mass_sum_kg"])) <= 12.0,
        "actuator_physics_passed": reports["actuator_physics"].get("status") == "passed",
        "terrain_and_contact_passed": reports["biped_simulation"].get("status") == "passed" and reports["contact_stability"].get("status") == "passed",
        "godot_io_passed": reports["godot_io"].get("status") == "passed",
        "communication_passed": reports["communication"].get("status") == "passed",
        "public_data_replay_passed": reports["public_data_replay"].get("status") == "passed",
        "component_parameter_log_present": reports["component_parameters"].get("schema_version") == "biped-component-parameter-log.v1",
        "hardware_gate_remains_blocked": str(reports["hardware_gap"].get("status", "")).startswith("blocked"),
        "hardware_commands_disabled": hardware.get("safe_defaults", {}).get("hardware_commands_enabled") is False,
    }
    software_passed = all(checks.values())
    return {
        "schema_version": "part_driven_system_simulation_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if software_passed else "failed",
        "evidence_source": "part_parameter_software_simulation",
        "robot": robot["name"],
        "component_inventory": inventory,
        "mass_and_geometry_summary": mass_and_geometry_summary(robot),
        "actuator_system_summary": actuator_system_summary(actuators, reports["actuator_physics"]),
        "terrain_and_contact_summary": terrain_and_contact_summary(reports["biped_simulation"], reports["contact_stability"]),
        "godot_io_summary": {
            "status": reports["godot_io"].get("status"),
            "telemetry_rows": reports["godot_io"].get("telemetry_rows"),
            "commands_expected": reports["godot_io"].get("commands_expected"),
            "metrics": reports["godot_io"].get("metrics", {}),
        },
        "communication_summary": {
            "status": reports["communication"].get("status"),
            "transport": reports["communication"].get("transport", {}),
            "metrics": reports["communication"].get("metrics", {}),
        },
        "public_data_replay_summary": {
            "status": reports["public_data_replay"].get("status"),
            "evidence_source": reports["public_data_replay"].get("evidence_source"),
            "sample_summary": reports["public_data_replay"].get("sample_summary", {}),
        },
        "hardware_validation_options": hardware_options(hardware, software_passed),
        "source_artifact_status": source_status,
        "checks": checks,
        "residual_risks": [
            "This is a system-level software aggregation over deterministic and Godot-generated artifacts, not hardware-in-the-loop evidence.",
            "Rigid-body contact and actuator behavior remain approximations until a higher-fidelity physics or hardware validation task is approved.",
            "Real hardware, serial/CAN and ROS2 validation remain blocked by missing external runtime and must be run as separate live validation gates.",
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a part-parameter-driven system software simulation report.")
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--trace-output", type=Path, default=DEFAULT_TRACE)
    args = parser.parse_args()

    report = build_report()
    trace = build_trace(report)
    write_json(args.report_output, report)
    write_jsonl(args.trace_output, trace)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "trace": str(args.trace_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
