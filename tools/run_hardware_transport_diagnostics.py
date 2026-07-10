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
    IMC22_TRANSPORT_DIAGNOSTICS_SCHEMA_VERSION,
    default_imc22_transport_profile,
    run_imc22_transport_diagnostics,
)

SOURCE_PATH_FIELDS = (
    "profile_file",
    "replay_source",
    "fault_table_source",
    "recovery_policy_source",
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
    parser.add_argument(
        "--recovery-policy-file",
        help="Optional vendor recovery policy JSON file.",
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


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _resolve_relative_path(
    value: str | None,
    *,
    base_dir: Path,
    allow_empty: bool = False,
) -> tuple[bool, Path | None, str | None]:
    text = _text(value)
    if not text:
        return (True, None, None) if allow_empty else (False, None, "empty")
    path = Path(text)
    if path.is_absolute():
        return False, None, "absolute"
    if ".." in path.parts:
        return False, None, "parent_directory"
    base_relative = base_dir / path
    if base_relative.exists():
        return True, base_relative, None
    return True, PROJECT_ROOT / path, None


def _path_status(
    value: str | None,
    *,
    base_dir: Path,
    allow_empty: bool = True,
) -> dict[str, Any]:
    valid, resolved_path, error = _resolve_relative_path(
        value,
        base_dir=base_dir,
        allow_empty=allow_empty,
    )
    return {
        "path": _text(value),
        "path_valid": valid,
        "path_error": error,
        "resolved_path": str(resolved_path) if resolved_path is not None else None,
        "exists": bool(resolved_path and resolved_path.exists()),
    }


def _load_profile(args: argparse.Namespace, profile_status: dict[str, Any]) -> dict[str, Any]:
    profile = default_imc22_transport_profile(args.transport)
    if profile_status["path"] and profile_status["path_valid"] and profile_status["exists"]:
        profile.update(
            json.loads(Path(profile_status["resolved_path"]).read_text(encoding="utf-8"))
        )
    for key, value in {
        "replay_source": args.replay_source,
        "channel": args.channel,
        "serial_port": args.serial_port,
        "baudrate": args.baudrate,
        "bitrate": args.bitrate,
        "fault_table_source": args.fault_table_file,
        "recovery_policy_source": args.recovery_policy_file,
    }.items():
        if value is not None:
            profile[key] = value
    return profile


def _source_path_statuses(
    args: argparse.Namespace,
    profile: dict[str, Any],
) -> dict[str, dict[str, Any]]:
    output_dir = Path(args.output).resolve().parent
    return {
        "profile_file": _path_status(
            args.profile_file,
            base_dir=output_dir,
            allow_empty=True,
        ),
        "replay_source": _path_status(
            _text(profile.get("replay_source")),
            base_dir=output_dir,
            allow_empty=True,
        ),
        "fault_table_source": _path_status(
            _text(profile.get("fault_table_source")),
            base_dir=output_dir,
            allow_empty=True,
        ),
        "recovery_policy_source": _path_status(
            _text(profile.get("recovery_policy_source")),
            base_dir=output_dir,
            allow_empty=True,
        ),
    }


def _source_path_blockers(
    statuses: dict[str, dict[str, Any]]
) -> list[dict[str, str]]:
    blockers: list[dict[str, str]] = []
    for field in SOURCE_PATH_FIELDS:
        status = statuses[field]
        if not status["path"]:
            continue
        if not status["path_valid"]:
            blockers.append(
                {
                    "field": field,
                    "reason": str(status["path_error"]),
                }
            )
        elif not status["exists"]:
            blockers.append(
                {
                    "field": field,
                    "reason": "missing",
                }
            )
    return blockers


def _apply_resolved_sources(
    profile: dict[str, Any],
    statuses: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    resolved = dict(profile)
    for field in ("replay_source", "fault_table_source", "recovery_policy_source"):
        status = statuses[field]
        if status["path"] and status["path_valid"] and status["resolved_path"]:
            resolved[field] = status["resolved_path"]
    return resolved


def _blocked_source_path_report(
    *,
    args: argparse.Namespace,
    profile: dict[str, Any],
    statuses: dict[str, dict[str, Any]],
    blockers: list[dict[str, str]],
) -> dict[str, Any]:
    return {
        "schema_version": IMC22_TRANSPORT_DIAGNOSTICS_SCHEMA_VERSION,
        "status": "blocked",
        "transport_profile": profile,
        "source_path_statuses": statuses,
        "source_path_blockers": blockers,
        "checks": [
            {
                "name": "source_path_validation",
                "status": "blocked",
                "message": "Source paths must be relative, stay within allowed roots, and exist before diagnostics run.",
                "details": {
                    "blockers": blockers,
                    "output": args.output,
                },
            }
        ],
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    profile_status = _path_status(
        args.profile_file,
        base_dir=Path(args.output).resolve().parent,
        allow_empty=True,
    )
    profile = _load_profile(args, profile_status)
    source_path_statuses = _source_path_statuses(args, profile)
    source_path_blockers = _source_path_blockers(source_path_statuses)
    if source_path_blockers:
        report = _blocked_source_path_report(
            args=args,
            profile=profile,
            statuses=source_path_statuses,
            blockers=source_path_blockers,
        )
    else:
        report = run_imc22_transport_diagnostics(
            _apply_resolved_sources(profile, source_path_statuses),
            attempt_connect=args.attempt_connect,
        )
        report["source_path_statuses"] = source_path_statuses
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
