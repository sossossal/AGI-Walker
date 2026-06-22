from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "test_env" / "simulation_capability_upgrade_report.json"
REPORTS = {
    "part_driven_system_simulation": "part_driven_system_simulation_report.json",
    "sensor_simulation": "sensor_simulation_report.json",
    "scenario_matrix": "scenario_matrix_report.json",
    "robot_description_mapping": "robot_description_mapping_report.json",
    "fault_injection": "fault_injection_report.json",
    "ros2_topic_contract_simulation": "ros2_topic_contract_simulation_report.json",
    "godot_visual_acceptance": "godot_visual_acceptance_report.json",
    "hardware_gap": "hardware_gap_report.json",
}


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


def main() -> int:
    parser = argparse.ArgumentParser(description="Build aggregate simulation capability upgrade report.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    payloads = {name: load_json(ROOT / "test_env" / file_name) for name, file_name in REPORTS.items()}
    statuses = {name: payload.get("status") for name, payload in payloads.items()}
    capability_checks = {
        "sensor_simulation_passed": statuses["sensor_simulation"] == "passed",
        "scenario_matrix_passed": statuses["scenario_matrix"] == "passed",
        "robot_description_mapping_passed": statuses["robot_description_mapping"] == "passed",
        "fault_injection_passed": statuses["fault_injection"] == "passed",
        "ros2_topic_contract_simulation_passed": statuses["ros2_topic_contract_simulation"] == "passed",
        "godot_visual_acceptance_passed": statuses["godot_visual_acceptance"] == "passed",
        "system_simulation_still_passed": statuses["part_driven_system_simulation"] == "passed",
        "hardware_still_blocked": str(statuses["hardware_gap"]).startswith("blocked"),
    }
    report = {
        "schema_version": "simulation_capability_upgrade_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(capability_checks.values()) else "failed",
        "evidence_source": "software_simulation_capability_upgrade",
        "capability_status": statuses,
        "capability_checks": capability_checks,
        "capability_summary": {
            "sensor_models": payloads["sensor_simulation"].get("sensor_counts", {}),
            "scenario_matrix": {
                "scenario_count": payloads["scenario_matrix"].get("scenario_count"),
                "pass_ratio": payloads["scenario_matrix"].get("pass_ratio"),
            },
            "description_mapping_formats": list(payloads["robot_description_mapping"].get("target_formats", {})),
            "fault_count": payloads["fault_injection"].get("fault_count"),
            "ros2_topics": payloads["ros2_topic_contract_simulation"].get("topics_seen", []),
            "visual_frame_summary_count": payloads["godot_visual_acceptance"].get("frame_summary_count"),
        },
        "hardware_boundary": {
            "status": statuses["hardware_gap"],
            "simulation_substitute_allowed_for_hardware_claims": False,
        },
        "residual_risks": [
            "Default capability upgrade uses software models and telemetry-derived visual summaries.",
            "URDF/SDF/MJCF output is a mapping preview, not a simulator-native loaded model.",
            "ROS2 topic contract simulation does not start a ROS2 runtime.",
        ],
    }
    write_json(args.output, report)
    print(json.dumps({"status": report["status"], "output": str(args.output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
