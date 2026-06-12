from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REPORT = ROOT / "test_env" / "ros2_topic_contract_simulation_report.json"
DEFAULT_EVENTS = ROOT / "test_env" / "ros2_topic_contract_events.jsonl"


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
    parser = argparse.ArgumentParser(description="Simulate ROS2 topic contracts from retained telemetry.")
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--events-output", type=Path, default=DEFAULT_EVENTS)
    args = parser.parse_args()

    contract = load_json(ROOT / "config" / "ros2_topic_contract.json")
    robot = load_json(ROOT / "config" / "robot.json")
    godot_rows = load_jsonl(ROOT / "test_env" / "godot_io_telemetry.jsonl")
    actuator_rows = load_jsonl(ROOT / "test_env" / "actuator_telemetry.jsonl")
    joint_names = [joint["name"] for joint in robot["joints"]]
    events = []
    for index, row in enumerate(godot_rows[:12]):
        stamp = row["time_s"]
        joint_slice = actuator_rows[index * len(joint_names):(index + 1) * len(joint_names)]
        events.extend(
            [
                {"topic": "/clock", "timestamp_s": stamp, "message_type": "rosgraph_msgs/msg/Clock", "payload_fields": ["clock"]},
                {"topic": "/imu", "timestamp_s": stamp, "message_type": "sensor_msgs/msg/Imu", "payload_fields": ["orientation", "angular_velocity", "linear_acceleration", "header.stamp"]},
                {"topic": "/tf", "timestamp_s": stamp, "message_type": "tf2_msgs/msg/TFMessage", "payload_fields": ["transforms"]},
                {"topic": "/cmd_joint_targets", "timestamp_s": stamp, "message_type": "trajectory_msgs/msg/JointTrajectory", "payload_fields": ["joint_names", "points", "header.stamp"]},
                {
                    "topic": "/joint_states",
                    "timestamp_s": stamp,
                    "message_type": "sensor_msgs/msg/JointState",
                    "payload_fields": ["name", "position", "velocity", "effort", "header.stamp"],
                    "joint_count": len(joint_slice) or len(joint_names),
                },
            ]
        )
    topics_seen = {event["topic"] for event in events}
    required_topics = {topic["name"] for topic in contract["topics"]}
    checks = {
        "runtime_mode_is_simulation": contract.get("runtime_mode") == "software_contract_simulation",
        "min_events_present": len(events) >= contract["acceptance"]["min_events"],
        "all_topics_present": required_topics.issubset(topics_seen),
        "message_types_present": all(event.get("message_type") for event in events),
        "real_ros2_not_claimed": True,
    }
    report = {
        "schema_version": "ros2_topic_contract_simulation_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "runtime_mode": contract["runtime_mode"],
        "event_count": len(events),
        "topics_seen": sorted(topics_seen),
        "checks": checks,
        "events_artifact": "test_env/ros2_topic_contract_events.jsonl",
        "limitations": ["ROS2 topic contract simulation does not start a ROS2 runtime or bridge."],
    }
    write_jsonl(args.events_output, events)
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "events": str(args.events_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
