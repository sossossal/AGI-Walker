from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "part_driven_system_simulation_report.json"
DEFAULT_TRACE = ROOT / "test_env" / "part_driven_system_simulation_trace.jsonl"
REQUIRED_SECTIONS = [
    "component_inventory",
    "mass_and_geometry_summary",
    "actuator_system_summary",
    "terrain_and_contact_summary",
    "godot_io_summary",
    "communication_summary",
    "public_data_replay_summary",
    "hardware_validation_options",
    "source_artifact_status",
    "checks",
    "residual_risks",
]


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
                if not isinstance(payload, dict):
                    raise ValueError(f"{path} contains a non-object row")
                rows.append(payload)
    return rows


def validate(report: dict[str, Any], trace: list[dict[str, Any]]) -> dict[str, Any]:
    hardware = report.get("hardware_validation_options", {})
    checks = {
        "schema_valid": report.get("schema_version") == "part_driven_system_simulation_report.v1",
        "status_passed": report.get("status") == "passed",
        "evidence_source_valid": report.get("evidence_source") == "part_parameter_software_simulation",
        "required_sections_present": all(section in report for section in REQUIRED_SECTIONS),
        "component_inventory_complete": report.get("component_inventory", {}).get("body_segment_count", 0) >= 10,
        "all_report_checks_passed": all(report.get("checks", {}).values()),
        "hardware_blocked": hardware.get("current_status") == "blocked_by_missing_external_runtime",
        "hardware_commands_disabled": hardware.get("hardware_commands_enabled") is False,
        "simulation_not_hardware_substitute": hardware.get("simulation_substitute_allowed_for_hardware_claims") is False,
        "trace_rows_present": len(trace) >= len(report.get("checks", {})),
        "trace_has_source_checks": any(row.get("event") == "source_artifact_checked" for row in trace),
        "trace_has_system_checks": any(row.get("event") == "system_check_evaluated" for row in trace),
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "checks": checks,
        "trace_rows": len(trace),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate part-driven system simulation report and trace.")
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--trace", type=Path, default=DEFAULT_TRACE)
    args = parser.parse_args()

    result = validate(load_json(args.report), load_jsonl(args.trace))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
