from __future__ import annotations

import argparse
import json
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
COMM_CONFIG = ROOT / "config" / "communication.json"
GODOT_INPUT = ROOT / "config" / "godot_io_input.json"
GODOT_REPORT = ROOT / "test_env" / "godot_io_report.json"
GODOT_TELEMETRY = ROOT / "test_env" / "godot_io_telemetry.jsonl"
DEFAULT_EVENTS = ROOT / "test_env" / "communication_events.jsonl"
DEFAULT_REPORT = ROOT / "test_env" / "communication_report.json"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return payload


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if line.strip():
                payload = json.loads(line)
                if not isinstance(payload, dict):
                    raise ValueError(f"{path} contains a non-object row")
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


def latency_ms(index: int, base_ms: float, jitter_ms: float) -> float:
    return base_ms + math.sin(index * 1.618) * jitter_ms


def should_drop(index: int, every_n: int) -> bool:
    return every_n > 0 and index > 0 and index % every_n == 0


def build_command_events(config: dict[str, Any], godot_input: dict[str, Any]) -> tuple[list[dict[str, Any]], dict[str, float]]:
    transport = config["transport"]
    events: list[dict[str, Any]] = []
    command_latencies: list[float] = []
    ack_latencies: list[float] = []
    command_loss_every = int(transport["deterministic_loss_every_n_command_packets"])
    for index, command in enumerate(godot_input["commands"], start=1):
        packet_id = f"cmd-{index:04d}"
        send_time = float(command["time_s"])
        command_latency = latency_ms(index, float(transport["base_latency_ms"]), float(transport["jitter_ms"]))
        dropped = should_drop(index, command_loss_every)
        events.append(
            {
                "event": "command_sent",
                "packet_id": packet_id,
                "channel": transport["command_channel"],
                "time_s": round(send_time, 5),
                "payload": command,
            }
        )
        if dropped:
            events.append(
                {
                    "event": "packet_dropped",
                    "packet_id": packet_id,
                    "channel": transport["command_channel"],
                    "time_s": round(send_time + command_latency / 1000.0, 5),
                    "reason": "deterministic_command_loss",
                }
            )
            continue
        delivered_time = send_time + command_latency / 1000.0
        ack_latency = latency_ms(index, float(transport["ack_latency_ms"]), float(transport["jitter_ms"]) * 0.35)
        ack_time = delivered_time + ack_latency / 1000.0
        command_latencies.append(command_latency)
        ack_latencies.append(ack_latency)
        events.append(
            {
                "event": "command_delivered",
                "packet_id": packet_id,
                "channel": transport["command_channel"],
                "time_s": round(delivered_time, 5),
                "latency_ms": round(command_latency, 5),
            }
        )
        events.append(
            {
                "event": "ack_received",
                "packet_id": f"ack-{index:04d}",
                "ack_for_packet_id": packet_id,
                "channel": transport["ack_channel"],
                "delivered_time_s": round(delivered_time, 5),
                "ack_time_s": round(ack_time, 5),
                "latency_ms": round(ack_latency, 5),
            }
        )
    metrics = {
        "commands_sent": float(len(godot_input["commands"])),
        "commands_delivered": float(len(command_latencies)),
        "acks_received": float(len(ack_latencies)),
        "max_command_latency_ms": max(command_latencies) if command_latencies else 0.0,
        "max_ack_latency_ms": max(ack_latencies) if ack_latencies else 0.0,
    }
    return events, metrics


def build_telemetry_events(config: dict[str, Any], telemetry: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, float]]:
    transport = config["transport"]
    downsample = int(transport["telemetry_downsample"])
    loss_every = int(transport["deterministic_loss_every_n_telemetry_packets"])
    selected = telemetry[::downsample]
    events: list[dict[str, Any]] = []
    latencies: list[float] = []
    previous_latency: float | None = None
    jitter_values: list[float] = []
    for index, row in enumerate(selected, start=1):
        packet_id = f"tel-{index:04d}"
        send_time = float(row["time_s"])
        telemetry_latency = latency_ms(index, float(transport["base_latency_ms"]) * 0.75, float(transport["jitter_ms"]))
        if should_drop(index, loss_every):
            events.append(
                {
                    "event": "packet_dropped",
                    "packet_id": packet_id,
                    "channel": transport["telemetry_channel"],
                    "time_s": round(send_time + telemetry_latency / 1000.0, 5),
                    "reason": "deterministic_telemetry_loss",
                }
            )
            continue
        receive_time = send_time + telemetry_latency / 1000.0
        latencies.append(telemetry_latency)
        if previous_latency is not None:
            jitter_values.append(abs(telemetry_latency - previous_latency))
        previous_latency = telemetry_latency
        events.append(
            {
                "event": "telemetry_received",
                "packet_id": packet_id,
                "channel": transport["telemetry_channel"],
                "send_time_s": round(send_time, 5),
                "receive_time_s": round(receive_time, 5),
                "latency_ms": round(telemetry_latency, 5),
                "payload": {
                    "time_s": row["time_s"],
                    "robot_z_m": row["robot_z_m"],
                    "speed_scale": row["speed_scale"],
                    "roll_degrees": row["roll_degrees"],
                    "pitch_degrees": row["pitch_degrees"],
                },
            }
        )
    metrics = {
        "telemetry_selected": float(len(selected)),
        "telemetry_delivered": float(len(latencies)),
        "max_telemetry_latency_ms": max(latencies) if latencies else 0.0,
        "max_jitter_ms": max(jitter_values) if jitter_values else 0.0,
    }
    return events, metrics


def build_report(config: dict[str, Any], godot_report: dict[str, Any], command_metrics: dict[str, float], telemetry_metrics: dict[str, float]) -> dict[str, Any]:
    acceptance = config["acceptance"]
    command_delivery_ratio = command_metrics["commands_delivered"] / max(command_metrics["commands_sent"], 1.0)
    ack_ratio = command_metrics["acks_received"] / max(command_metrics["commands_sent"], 1.0)
    telemetry_delivery_ratio = telemetry_metrics["telemetry_delivered"] / max(telemetry_metrics["telemetry_selected"], 1.0)
    checks = {
        "godot_io_passed": godot_report.get("status") == "passed",
        "command_delivery_ratio": command_delivery_ratio >= float(acceptance["min_command_delivery_ratio"]),
        "ack_ratio": ack_ratio >= float(acceptance["min_ack_ratio"]),
        "telemetry_delivery_ratio": telemetry_delivery_ratio >= float(acceptance["min_telemetry_delivery_ratio"]),
        "command_latency_within_acceptance": command_metrics["max_command_latency_ms"] <= float(acceptance["max_command_latency_ms"]),
        "ack_latency_within_acceptance": command_metrics["max_ack_latency_ms"] <= float(acceptance["max_ack_latency_ms"]),
        "telemetry_latency_within_acceptance": telemetry_metrics["max_telemetry_latency_ms"] <= float(acceptance["max_telemetry_latency_ms"]),
        "jitter_within_acceptance": telemetry_metrics["max_jitter_ms"] <= float(acceptance["max_jitter_ms"]),
    }
    return {
        "schema_version": "biped-communication-report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "transport": config["transport"],
        "metrics": {
            **command_metrics,
            **telemetry_metrics,
            "command_delivery_ratio": round(command_delivery_ratio, 6),
            "ack_ratio": round(ack_ratio, 6),
            "telemetry_delivery_ratio": round(telemetry_delivery_ratio, 6),
        },
        "checks": checks,
        "source_artifacts": {
            "godot_io_report": str(GODOT_REPORT.relative_to(ROOT)).replace("\\", "/"),
            "godot_io_telemetry": str(GODOT_TELEMETRY.relative_to(ROOT)).replace("\\", "/"),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Run simulated communication over Godot input/output artifacts.")
    parser.add_argument("--events-output", type=Path, default=DEFAULT_EVENTS)
    parser.add_argument("--report-output", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()

    config = load_json(COMM_CONFIG)
    godot_input = load_json(GODOT_INPUT)
    godot_report = load_json(GODOT_REPORT)
    godot_telemetry = load_jsonl(GODOT_TELEMETRY)
    command_events, command_metrics = build_command_events(config, godot_input)
    telemetry_events, telemetry_metrics = build_telemetry_events(config, godot_telemetry)
    events = sorted(command_events + telemetry_events, key=lambda row: float(row.get("time_s", row.get("receive_time_s", 0.0))))
    report = build_report(config, godot_report, command_metrics, telemetry_metrics)
    write_jsonl(args.events_output, events)
    write_json(args.report_output, report)
    print(json.dumps({"status": report["status"], "events": str(args.events_output), "report": str(args.report_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
