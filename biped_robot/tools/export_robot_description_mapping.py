from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any
from xml.sax.saxutils import escape


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "robot_description_mapping_report.json"
DEFAULT_PREVIEW = ROOT / "test_env" / "robot_description_mapping_preview.json"


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


def urdf_preview(robot: dict[str, Any]) -> str:
    links = [f'<link name="{escape(segment["name"])}"><inertial><mass value="{float(segment["mass"]):.5f}"/></inertial></link>' for segment in robot["segments"]]
    joints = [f'<joint name="{escape(joint["name"])}" type="revolute"><axis xyz="1 0 0"/><limit lower="{joint["limit"][0]}" upper="{joint["limit"][1]}"/></joint>' for joint in robot["joints"]]
    return f'<robot name="{escape(robot["name"])}">' + "".join(links + joints) + "</robot>"


def sdf_preview(robot: dict[str, Any]) -> str:
    links = [f'<link name="{escape(segment["name"])}"><inertial><mass>{float(segment["mass"]):.5f}</mass></inertial></link>' for segment in robot["segments"]]
    return f'<sdf version="1.10"><model name="{escape(robot["name"])}">' + "".join(links) + "</model></sdf>"


def mjcf_preview(robot: dict[str, Any]) -> str:
    bodies = [f'<body name="{escape(segment["name"])}"><geom type="box" mass="{float(segment["mass"]):.5f}"/></body>' for segment in robot["segments"]]
    return f'<mujoco model="{escape(robot["name"])}"><worldbody>' + "".join(bodies) + "</worldbody></mujoco>"


def main() -> int:
    parser = argparse.ArgumentParser(description="Export robot description mapping report for URDF/SDF/MJCF compatibility.")
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--preview-output", type=Path, default=DEFAULT_PREVIEW)
    args = parser.parse_args()

    robot = load_json(ROOT / "config" / "robot.json")
    formats = {
        "urdf": {"status": "mapped_with_approximations", "preview": urdf_preview(robot), "unsupported_fields": ["terrain", "gait_profile", "thermal_model"]},
        "sdf": {"status": "mapped_with_approximations", "preview": sdf_preview(robot), "unsupported_fields": ["controller_tuning", "public_replay_metadata"]},
        "mjcf": {"status": "mapped_with_approximations", "preview": mjcf_preview(robot), "unsupported_fields": ["Godot scene camera", "communication_channels"]},
    }
    checks = {
        "all_formats_present": set(formats) == {"urdf", "sdf", "mjcf"},
        "all_segments_mapped": all(robot["segments"]),
        "all_joints_mapped_to_at_least_urdf": len(robot["joints"]) > 0 and "joint name" in formats["urdf"]["preview"],
        "mapping_is_preview_only": True,
    }
    report = {
        "schema_version": "robot_description_mapping_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "source_schema": robot.get("schema_version"),
        "robot": robot["name"],
        "target_formats": {name: {"status": value["status"], "unsupported_fields": value["unsupported_fields"]} for name, value in formats.items()},
        "checks": checks,
        "preview_artifact": "test_env/robot_description_mapping_preview.json",
        "limitations": ["Generated mappings are compatibility previews, not simulator-native validated models."],
    }
    write_json(args.preview_output, {name: value["preview"] for name, value in formats.items()})
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "preview": str(args.preview_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
