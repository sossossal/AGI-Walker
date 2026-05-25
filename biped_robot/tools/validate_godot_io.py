from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
INPUT = ROOT / "config" / "godot_io_input.json"
DEFAULT_REPORT = ROOT / "test_env" / "godot_io_report.json"
DEFAULT_TELEMETRY = ROOT / "test_env" / "godot_io_telemetry.jsonl"


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


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate Godot input/output simulation artifacts.")
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--telemetry", type=Path, default=DEFAULT_TELEMETRY)
    args = parser.parse_args()

    contract = load_json(INPUT)
    report = load_json(args.report)
    rows = load_jsonl(args.telemetry)
    acceptance = contract["acceptance"]
    required_fields = {
        "time_s",
        "robot_z_m",
        "terrain_height_m",
        "speed_scale",
        "gait_cycle_seconds",
        "roll_degrees",
        "pitch_degrees",
        "left_foot_clearance_m",
        "right_foot_clearance_m",
    }
    missing = [field for field in required_fields if any(field not in row for row in rows)]
    checks = {
        "report_status_passed": report.get("status") == "passed",
        "telemetry_rows_present": len(rows) >= int(acceptance["min_telemetry_rows"]),
        "commands_applied": len(report.get("commands_applied", [])) >= len(contract["commands"]),
        "required_fields_present": not missing,
        "roll_within_acceptance": max(abs(float(row["roll_degrees"])) for row in rows) <= float(acceptance["max_allowed_abs_roll_degrees"]),
        "pitch_within_acceptance": max(abs(float(row["pitch_degrees"])) for row in rows) <= float(acceptance["max_allowed_abs_pitch_degrees"]),
    }
    result = {
        "status": "passed" if all(checks.values()) else "failed",
        "report": str(args.report),
        "telemetry": str(args.telemetry),
        "rows": len(rows),
        "checks": checks,
        "missing_fields": sorted(set(missing)),
    }
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
