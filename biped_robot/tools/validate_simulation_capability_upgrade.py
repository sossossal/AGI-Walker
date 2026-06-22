from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "simulation_capability_upgrade_report.json"
REQUIRED_CAPABILITIES = {
    "sensor_simulation",
    "scenario_matrix",
    "robot_description_mapping",
    "fault_injection",
    "ros2_topic_contract_simulation",
    "godot_visual_acceptance",
    "part_driven_system_simulation",
}


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def validate(report: dict[str, Any]) -> dict[str, Any]:
    statuses = report.get("capability_status", {})
    checks = {
        "schema_valid": report.get("schema_version") == "simulation_capability_upgrade_report.v1",
        "status_passed": report.get("status") == "passed",
        "required_capabilities_present": REQUIRED_CAPABILITIES.issubset(statuses),
        "all_new_capabilities_passed": all(statuses.get(name) == "passed" for name in REQUIRED_CAPABILITIES),
        "hardware_status_blocked": str(statuses.get("hardware_gap", "")).startswith("blocked"),
        "hardware_not_substituted": report.get("hardware_boundary", {}).get("simulation_substitute_allowed_for_hardware_claims") is False,
        "all_report_checks_passed": all(report.get("capability_checks", {}).values()),
    }
    return {"status": "passed" if all(checks.values()) else "failed", "checks": checks}


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate simulation capability upgrade aggregate report.")
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()

    result = validate(load_json(args.report))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
