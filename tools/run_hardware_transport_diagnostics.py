from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.godot_robot_env.hardware_controller import (
    default_imc22_transport_profile,
    run_imc22_transport_diagnostics,
)


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run IMC-22 transport profile diagnostics."
    )
    parser.add_argument(
        "--transport",
        choices=["socketcan", "pcan", "replay", "serial_bridge"],
        default="socketcan",
    )
    parser.add_argument("--profile-file", help="Optional JSON profile override file.")
    parser.add_argument(
        "--fault-table-file",
        help="Optional vendor fault table JSON file.",
    )
    parser.add_argument("--replay-source", help="Replay fixture path override.")
    parser.add_argument("--channel", help="Transport channel override.")
    parser.add_argument("--serial-port", help="Serial bridge port override.")
    parser.add_argument("--baudrate", type=int, help="Serial bridge baudrate override.")
    parser.add_argument("--bitrate", type=int, help="CAN bitrate override.")
    parser.add_argument(
        "--attempt-connect",
        action="store_true",
        help="Attempt to open the transport and collect live/replay node diagnostics.",
    )
    parser.add_argument(
        "--output",
        default="test_env/hardware_transport_diagnostics_report.json",
        help="Output report path.",
    )
    parser.add_argument(
        "--telemetry-output",
        help="Optional JSON path for exporting fault telemetry mapping.",
    )
    return parser.parse_args(argv)


def _load_profile(args: argparse.Namespace) -> dict[str, Any]:
    profile = default_imc22_transport_profile(args.transport)
    if args.profile_file:
        profile.update(json.loads(Path(args.profile_file).read_text(encoding="utf-8")))
    for key, value in {
        "replay_source": args.replay_source,
        "channel": args.channel,
        "serial_port": args.serial_port,
        "baudrate": args.baudrate,
        "bitrate": args.bitrate,
        "fault_table_source": args.fault_table_file,
    }.items():
        if value is not None:
            profile[key] = value
    return profile


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    profile = _load_profile(args)
    report = run_imc22_transport_diagnostics(
        profile,
        attempt_connect=args.attempt_connect,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    if args.telemetry_output and "fault_telemetry_report" in report:
        telemetry_path = Path(args.telemetry_output)
        telemetry_path.parent.mkdir(parents=True, exist_ok=True)
        telemetry_path.write_text(
            json.dumps(report["fault_telemetry_report"], ensure_ascii=False, indent=2)
            + "\n",
            encoding="utf-8",
        )
    return 0 if report["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
