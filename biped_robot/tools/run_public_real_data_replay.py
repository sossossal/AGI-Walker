from __future__ import annotations

import argparse
import json
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCES = ROOT / "config" / "public_real_data_sources.json"
DEFAULT_FIXTURE = ROOT / "fixtures" / "public_real_data_replay_fixture.json"
DEFAULT_TRACE = ROOT / "test_env" / "public_real_data_replay_trace.jsonl"
DEFAULT_REPORT = ROOT / "test_env" / "public_real_data_replay_report.json"
REQUIRED_PAYLOADS = {"joint_state", "foot_contact", "imu", "odom", "twist", "can_frame"}
REQUIRED_DOMAINS = {"legged_robot", "ros2", "can"}


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


def source_ids(sources: dict[str, Any]) -> set[str]:
    return {str(source["source_id"]) for source in sources.get("sources", [])}


def count_by(rows: list[dict[str, Any]], key: str) -> dict[str, int]:
    counts: dict[str, int] = {}
    for row in rows:
        value = str(row.get(key, ""))
        counts[value] = counts.get(value, 0) + 1
    return dict(sorted(counts.items()))


def timestamps_monotonic(rows: list[dict[str, Any]]) -> bool:
    last_by_dataset: dict[str, float] = {}
    for row in rows:
        dataset = str(row.get("source_dataset"))
        timestamp = float(row.get("timestamp_s", -1.0))
        if timestamp < last_by_dataset.get(dataset, -1.0):
            return False
        last_by_dataset[dataset] = timestamp
    return True


def map_sample(row: dict[str, Any], known_joints: set[str]) -> dict[str, Any]:
    payload_type = str(row["payload_type"])
    payload = row["payload"]
    if payload_type == "joint_state":
        joint = str(payload.get("joint"))
        return {
            "domain": "actuator",
            "valid": joint in known_joints,
            "fields": ["joint", "position_degrees", "velocity_degrees_per_second", "torque_nm"],
        }
    if payload_type in {"foot_contact", "imu"} and row["source_domain"] == "legged_robot":
        return {"domain": "contact", "valid": True, "fields": sorted(payload)}
    if payload_type in {"odom", "twist", "imu"} and row["source_domain"] == "ros2":
        return {"domain": "godot_io", "valid": True, "fields": sorted(payload)}
    if payload_type == "can_frame":
        return {"domain": "communication", "valid": int(payload.get("dlc", 0)) <= 8, "fields": sorted(payload)}
    return {"domain": "unmapped", "valid": False, "fields": sorted(payload) if isinstance(payload, dict) else []}


def build_trace(samples: list[dict[str, Any]], known_joints: set[str]) -> list[dict[str, Any]]:
    trace: list[dict[str, Any]] = []
    for sequence, row in enumerate(samples, start=1):
        mapping = map_sample(row, known_joints)
        trace.append(
            {
                "sequence": sequence,
                "timestamp_s": row["timestamp_s"],
                "source_dataset": row["source_dataset"],
                "source_domain": row["source_domain"],
                "payload_type": row["payload_type"],
                "evidence_source": "public_dataset_replay",
                "transport_claim": "simulated_transport",
                "hardware_validation": "real_hardware_not_run",
                "mapped_domain": mapping["domain"],
                "mapping_valid": mapping["valid"],
                "mapped_fields": mapping["fields"],
            }
        )
    return trace


def build_report(sources: dict[str, Any], fixture: dict[str, Any], robot: dict[str, Any], trace: list[dict[str, Any]]) -> dict[str, Any]:
    samples = fixture.get("samples", [])
    datasets = {str(row.get("source_dataset")) for row in samples}
    payload_types = set(count_by(samples, "payload_type"))
    domains = set(count_by(samples, "source_domain"))
    mapped_domains = set(count_by(trace, "mapped_domain"))
    known_sources = source_ids(sources)
    source_references_valid = datasets.issubset(known_sources)
    checks = {
        "sources_schema_valid": sources.get("schema_version") == "public_real_data_sources.v1",
        "fixture_schema_valid": fixture.get("schema_version") == "public_real_data_replay_fixture.v1",
        "offline_mode_enforced": sources.get("network_download_default") is False,
        "source_references_valid": source_references_valid,
        "required_domains_present": REQUIRED_DOMAINS.issubset(domains),
        "required_payloads_present": REQUIRED_PAYLOADS.issubset(payload_types),
        "timestamps_monotonic_per_dataset": timestamps_monotonic(samples),
        "actuator_mapping_present": "actuator" in mapped_domains,
        "contact_mapping_present": "contact" in mapped_domains,
        "godot_io_mapping_present": "godot_io" in mapped_domains,
        "communication_mapping_present": "communication" in mapped_domains,
        "all_mappings_valid": all(bool(row["mapping_valid"]) for row in trace),
        "real_hardware_claim_blocked": True,
    }
    return {
        "schema_version": "public_real_data_replay_report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "evidence_source": "public_dataset_replay",
        "transport_claim": "simulated_transport",
        "hardware_validation": "real_hardware_not_run",
        "robot_name": robot.get("name"),
        "source_summary": {
            "source_count": len(sources.get("sources", [])),
            "datasets_replayed": sorted(datasets),
            "network_download_default": sources.get("network_download_default"),
        },
        "sample_summary": {
            "sample_count": len(samples),
            "source_domains": count_by(samples, "source_domain"),
            "payload_types": count_by(samples, "payload_type"),
            "mapped_domains": count_by(trace, "mapped_domain"),
        },
        "checks": checks,
        "trace_artifact": str(DEFAULT_TRACE.relative_to(ROOT)).replace("\\", "/"),
        "residual_risks": [
            "Offline public dataset-shaped samples are not measurements from this biped robot.",
            "Replay evidence does not validate physical motor transport, serial/CAN hardware, or ROS2 runtime.",
            "Future network download mode needs explicit size, license, hash and failure-policy controls before use."
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay offline public dataset-shaped samples into biped simulation evidence.")
    parser.add_argument("--sources", type=Path, default=DEFAULT_SOURCES)
    parser.add_argument("--fixture", type=Path, default=DEFAULT_FIXTURE)
    parser.add_argument("--trace-output", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()

    sources = load_json(args.sources)
    fixture = load_json(args.fixture)
    robot = load_json(ROOT / "config" / "robot.json")
    known_joints = {str(joint.get("name")) for joint in robot.get("joints", [])}
    trace = build_trace(fixture.get("samples", []), known_joints)
    report = build_report(sources, fixture, robot, trace)
    report["trace_artifact"] = str(args.trace_output.relative_to(ROOT)).replace("\\", "/")
    write_jsonl(args.trace_output, trace)
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "report": str(args.report_output), "trace": str(args.trace_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
