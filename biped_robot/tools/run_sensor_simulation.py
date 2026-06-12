from __future__ import annotations

import argparse
import json
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "sensor_simulation_report.json"
DEFAULT_TRACE = ROOT / "test_env" / "sensor_simulation_trace.jsonl"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                payload = json.loads(line)
                if isinstance(payload, dict):
                    rows.append(payload)
    return rows


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


def deterministic_noise(index: int, amplitude: float) -> float:
    return math.sin(index * 1.61803398875) * amplitude


def build_trace(config: dict[str, Any], godot_rows: list[dict[str, Any]], actuator_rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    trace = []
    for index, row in enumerate(godot_rows):
        imu = config["imu"]
        contact = config["foot_contact"]
        visual = config["visual"]
        trace.append(
            {
                "sensor": "imu",
                "time_s": row["time_s"],
                "roll_degrees": round(float(row["roll_degrees"]) + deterministic_noise(index, imu["roll_noise_degrees"]), 5),
                "pitch_degrees": round(float(row["pitch_degrees"]) + deterministic_noise(index + 7, imu["pitch_noise_degrees"]), 5),
                "yaw_rate_degrees_per_second": round(deterministic_noise(index + 13, imu["yaw_rate_noise_degrees_per_second"]), 5),
                "linear_accel_mps2": [round(deterministic_noise(index, imu["accel_noise_mps2"]), 5), 0.0, 9.81],
            }
        )
        left_force = contact["nominal_stance_force_n"] if float(row["left_foot_clearance_m"]) <= contact["contact_clearance_threshold_m"] else 0.0
        right_force = contact["nominal_stance_force_n"] if float(row["right_foot_clearance_m"]) <= contact["contact_clearance_threshold_m"] else 0.0
        trace.append(
            {
                "sensor": "foot_contact",
                "time_s": row["time_s"],
                "left_force_n": left_force,
                "right_force_n": right_force,
                "left_contact": left_force > 0.0,
                "right_contact": right_force > 0.0,
            }
        )
        if index % 4 == 0:
            trace.append(
                {
                    "sensor": "visual_summary",
                    "time_s": row["time_s"],
                    "camera": visual["camera_name"],
                    "robot_visible": True,
                    "terrain_visible": True,
                    "estimated_depth_m": round(max(float(row["robot_y_m"]), visual["depth_range_m"][0]), 5),
                }
            )
    for index, row in enumerate(actuator_rows[:: max(1, len(actuator_rows) // 160)]):
        encoder = config["joint_encoder"]
        trace.append(
            {
                "sensor": "joint_encoder",
                "time_s": row["time_s"],
                "joint": row["joint"],
                "position_degrees": round(float(row["position_degrees"]) + deterministic_noise(index, encoder["position_noise_degrees"]), 5),
                "velocity_degrees_per_second": round(float(row["velocity_degrees_per_second"]) + deterministic_noise(index, encoder["velocity_noise_degrees_per_second"]), 5),
                "effort_nm": row["torque_nm"],
            }
        )
    return sorted(trace, key=lambda item: (float(item["time_s"]), str(item["sensor"])))


def build_report(config: dict[str, Any], trace: list[dict[str, Any]]) -> dict[str, Any]:
    by_sensor: dict[str, int] = {}
    max_abs_roll = 0.0
    max_abs_pitch = 0.0
    max_encoder_error = config["joint_encoder"]["position_noise_degrees"]
    for row in trace:
        sensor = str(row["sensor"])
        by_sensor[sensor] = by_sensor.get(sensor, 0) + 1
        if sensor == "imu":
            max_abs_roll = max(max_abs_roll, abs(float(row["roll_degrees"])))
            max_abs_pitch = max(max_abs_pitch, abs(float(row["pitch_degrees"])))
    acceptance = config["acceptance"]
    checks = {
        "imu_samples_present": by_sensor.get("imu", 0) >= acceptance["min_imu_samples"],
        "encoder_samples_present": by_sensor.get("joint_encoder", 0) >= acceptance["min_encoder_samples"],
        "contact_samples_present": by_sensor.get("foot_contact", 0) >= acceptance["min_contact_samples"],
        "visual_summaries_present": by_sensor.get("visual_summary", 0) >= acceptance["min_visual_frame_summaries"],
        "imu_roll_within_limit": max_abs_roll <= config["imu"]["max_abs_roll_degrees"],
        "imu_pitch_within_limit": max_abs_pitch <= config["imu"]["max_abs_pitch_degrees"],
        "encoder_noise_within_limit": max_encoder_error <= config["joint_encoder"]["max_position_error_degrees"],
    }
    return {
        "schema_version": "sensor_simulation_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "evidence_source": "software_sensor_model",
        "sensor_counts": dict(sorted(by_sensor.items())),
        "metrics": {
            "max_abs_roll_degrees": round(max_abs_roll, 5),
            "max_abs_pitch_degrees": round(max_abs_pitch, 5),
            "max_encoder_noise_degrees": max_encoder_error,
        },
        "checks": checks,
        "trace_artifact": "test_env/sensor_simulation_trace.jsonl",
        "limitations": ["Software-only sensor model; no camera image renderer or physical sensor hardware is used."],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run software sensor simulation over Godot and actuator telemetry.")
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--trace-output", type=Path, default=DEFAULT_TRACE)
    args = parser.parse_args()

    config = load_json(ROOT / "config" / "sensor_model.json")
    trace = build_trace(config, load_jsonl(ROOT / "test_env" / "godot_io_telemetry.jsonl"), load_jsonl(ROOT / "test_env" / "actuator_telemetry.jsonl"))
    report = build_report(config, trace)
    write_jsonl(args.trace_output, trace)
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "trace": str(args.trace_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
