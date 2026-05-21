from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
CONTRACT = ROOT / "contracts" / "hardware_interface.json"
DEFAULT_OUTPUT = ROOT / "test_env" / "hardware_gap_report.json"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def build_report(contract: dict[str, Any]) -> dict[str, Any]:
    blocked = [
        capability
        for capability in contract["required_capabilities"]
        if capability["current_status"] in {"missing_hardware", "missing_ros2_runtime"}
    ]
    return {
        "status": "blocked_by_missing_external_runtime" if blocked else "ready",
        "contract": contract["name"],
        "hardware_commands_enabled": contract["safe_defaults"]["hardware_commands_enabled"],
        "outside_changes_require_user_approval": contract["ownership_boundary"]["outside_changes_require_user_approval"],
        "blocked_checks": [
            {
                "name": capability["name"],
                "reason": capability["current_status"],
            }
            for capability in blocked
        ],
        "required_before_live_hardware_acceptance": [
            "Provide physical biped hardware or a verified hardware-in-the-loop rig.",
            "Provide serial or CAN transport details and emergency-stop procedure.",
            "Provide ROS2 runtime and topic bridge requirements if ROS2 acceptance is required.",
            "Approve changes outside biped_robot/ before integrating shared hardware or Godot modules."
        ],
        "local_mitigation": [
            "Hardware commands are disabled by contract.",
            "Godot-side behavior is covered by local visual and deterministic simulation gates.",
            "Missing live checks are recorded as blocked rather than silently accepted."
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build a machine-readable report for unavailable live hardware checks.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    report = build_report(load_json(CONTRACT))
    write_json(args.output, report)
    print(json.dumps({"status": report["status"], "output": str(args.output)}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
