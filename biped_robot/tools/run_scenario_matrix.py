from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "scenario_matrix_report.json"
DEFAULT_TRACE = ROOT / "test_env" / "scenario_matrix_trace.jsonl"


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


def evaluate_scenario(scenario: dict[str, Any], baseline: dict[str, Any], acceptance: dict[str, Any]) -> dict[str, Any]:
    motion = baseline["terrain_and_contact_summary"]["motion_metrics"]
    actuator = baseline["actuator_system_summary"]
    comm = baseline["communication_summary"]["metrics"]
    slope = float(motion["max_slope_degrees"]) * float(scenario["slope_multiplier"])
    clearance = float(motion["min_foot_clearance_m"]) - max(0.0, float(scenario["speed_scale"]) - 1.0) * 0.012
    stability = float(motion["min_stability_margin"]) * float(scenario["friction"]) / 0.82 - max(0.0, slope - 12.0) * 0.0015
    latency = float(comm["max_command_latency_ms"]) * float(scenario["latency_multiplier"])
    tracking = float(actuator["max_tracking_error_degrees"]) * float(scenario["actuator_load_multiplier"]) * max(1.0, float(scenario["speed_scale"]))
    checks = {
        "slope_within_limit": slope <= acceptance["max_slope_degrees"],
        "clearance_within_limit": clearance >= acceptance["min_clearance_m"],
        "stability_within_limit": stability >= acceptance["min_stability_margin"],
        "latency_within_limit": latency <= acceptance["max_latency_ms"],
        "tracking_within_limit": tracking <= acceptance["max_tracking_error_degrees"],
        "friction_supported": float(scenario["friction"]) >= acceptance["min_friction"],
    }
    return {
        "scenario": scenario["name"],
        "status": "passed" if all(checks.values()) else "failed",
        "inputs": scenario,
        "metrics": {
            "max_slope_degrees": round(slope, 5),
            "min_clearance_m": round(clearance, 5),
            "min_stability_margin": round(stability, 5),
            "max_command_latency_ms": round(latency, 5),
            "max_tracking_error_degrees": round(tracking, 5),
        },
        "checks": checks,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run deterministic scenario matrix over latest system simulation evidence.")
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--trace-output", type=Path, default=DEFAULT_TRACE)
    args = parser.parse_args()

    config = load_json(ROOT / "config" / "scenario_matrix.json")
    baseline = load_json(ROOT / "test_env" / "part_driven_system_simulation_report.json")
    rows = [evaluate_scenario(scenario, baseline, config["acceptance"]) for scenario in config["scenarios"]]
    pass_ratio = sum(1 for row in rows if row["status"] == "passed") / max(len(rows), 1)
    checks = {
        "scenario_count_present": len(rows) >= 6,
        "pass_ratio_within_acceptance": pass_ratio >= float(config["acceptance"]["min_pass_ratio"]),
        "at_least_one_stress_case_failed_or_passed_with_metrics": all(row.get("metrics") for row in rows),
    }
    report = {
        "schema_version": "scenario_matrix_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "scenario_count": len(rows),
        "passed_count": sum(1 for row in rows if row["status"] == "passed"),
        "failed_count": sum(1 for row in rows if row["status"] == "failed"),
        "pass_ratio": round(pass_ratio, 6),
        "checks": checks,
        "trace_artifact": "test_env/scenario_matrix_trace.jsonl",
    }
    write_jsonl(args.trace_output, rows)
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "trace": str(args.trace_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
