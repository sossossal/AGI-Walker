from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "fault_injection_report.json"
DEFAULT_TRACE = ROOT / "test_env" / "fault_injection_trace.jsonl"


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


def classify_fault(fault: dict[str, Any], system: dict[str, Any]) -> dict[str, Any]:
    actuator = system["actuator_system_summary"]
    terrain = system["terrain_and_contact_summary"]["motion_metrics"]
    comm = system["communication_summary"]["metrics"]
    fault_type = fault["type"]
    safe_state = "degraded_continue"
    metrics: dict[str, Any] = {}
    if fault_type == "actuator_derate":
        metrics["estimated_tracking_error_degrees"] = round(float(actuator["max_tracking_error_degrees"]) / float(fault["torque_scale"]), 5)
    elif fault_type == "joint_stiction":
        metrics["estimated_tracking_error_degrees"] = round(float(actuator["max_tracking_error_degrees"]) + float(fault["tracking_error_add_degrees"]), 5)
    elif fault_type == "sensor_drift":
        metrics["estimated_pitch_degrees"] = round(float(system["godot_io_summary"]["metrics"]["max_abs_pitch_degrees"]) + float(fault["pitch_drift_degrees"]), 5)
    elif fault_type == "communication_jitter":
        metrics["estimated_latency_ms"] = round(float(comm["max_command_latency_ms"]) * float(fault["latency_multiplier"]), 5)
    elif fault_type == "foot_slip":
        metrics["estimated_stability_margin"] = round(float(terrain["min_stability_margin"]) * float(fault["friction_scale"]), 5)
    elif fault_type == "emergency_stop":
        safe_state = "stop_commands_and_hold_pose"
        metrics["trigger_time_s"] = fault["trigger_time_s"]
    return {"fault": fault["name"], "type": fault_type, "safe_state": safe_state, "metrics": metrics, "classified": True}


def main() -> int:
    parser = argparse.ArgumentParser(description="Run software-only fault injection classification.")
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--trace-output", type=Path, default=DEFAULT_TRACE)
    args = parser.parse_args()

    config = load_json(ROOT / "config" / "fault_injection.json")
    system = load_json(ROOT / "test_env" / "part_driven_system_simulation_report.json")
    rows = [classify_fault(fault, system) for fault in config["faults"]]
    acceptance = config["acceptance"]
    checks = {
        "faults_classified": sum(1 for row in rows if row["classified"]) >= acceptance["min_faults_classified"],
        "estop_safe_state_present": any(row["safe_state"] == "stop_commands_and_hold_pose" for row in rows) if acceptance["require_estop_safe_state"] else True,
        "derated_tracking_within_limit": all(row["metrics"].get("estimated_tracking_error_degrees", 0.0) <= acceptance["max_derated_tracking_error_degrees"] for row in rows),
        "jitter_latency_within_limit": all(row["metrics"].get("estimated_latency_ms", 0.0) <= acceptance["max_jitter_latency_ms"] for row in rows),
        "slip_stability_within_limit": all(row["metrics"].get("estimated_stability_margin", acceptance["min_slip_stability_margin"]) >= acceptance["min_slip_stability_margin"] for row in rows),
    }
    report = {
        "schema_version": "fault_injection_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "fault_count": len(rows),
        "checks": checks,
        "trace_artifact": "test_env/fault_injection_trace.jsonl",
        "limitations": ["Faults are classified against software simulation metrics, not injected into physical hardware."],
    }
    write_jsonl(args.trace_output, rows)
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "trace": str(args.trace_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
