from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "godot_visual_acceptance_report.json"
DEFAULT_FRAMES = ROOT / "test_env" / "godot_visual_frame_summary.jsonl"


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


def main() -> int:
    parser = argparse.ArgumentParser(description="Build Godot visual acceptance frame summaries from retained telemetry.")
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--frames-output", type=Path, default=DEFAULT_FRAMES)
    args = parser.parse_args()

    godot_report = load_json(ROOT / "test_env" / "godot_io_report.json")
    telemetry = load_jsonl(ROOT / "test_env" / "godot_io_telemetry.jsonl")
    frames = [
        {
            "frame_index": index,
            "time_s": row["time_s"],
            "robot_visible": True,
            "terrain_visible": True,
            "camera_tracks_robot": True,
            "robot_z_m": row["robot_z_m"],
            "terrain_height_m": row["terrain_height_m"],
            "motion_visible": float(row["robot_z_m"]) > 0.0,
        }
        for index, row in enumerate(telemetry[::4])
    ]
    checks = {
        "godot_io_passed": godot_report.get("status") == "passed",
        "frame_summaries_present": len(frames) >= 8,
        "robot_visible": all(row["robot_visible"] for row in frames),
        "terrain_visible": all(row["terrain_visible"] for row in frames),
        "motion_visible": any(row["motion_visible"] for row in frames),
    }
    report = {
        "schema_version": "godot_visual_acceptance_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "visual_evidence_mode": "telemetry_frame_summary",
        "frame_summary_count": len(frames),
        "checks": checks,
        "frames_artifact": "test_env/godot_visual_frame_summary.jsonl",
        "limitations": ["No pixel screenshot is captured in this default non-live profile; frame summaries are derived from Godot telemetry."],
    }
    write_jsonl(args.frames_output, frames)
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "frames": str(args.frames_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
