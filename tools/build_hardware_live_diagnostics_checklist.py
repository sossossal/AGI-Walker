from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence


DEFAULT_FAULT_TABLE = "deployment/hardware/imc22_reflex_fault_table.json"
DEFAULT_RECOVERY_POLICY = "deployment/hardware/imc22_reflex_recovery_policy.json"
DEFAULT_REPORT = "test_env/hardware_live/hardware_transport_diagnostics_report.json"
DEFAULT_TELEMETRY = "test_env/hardware_live/hardware_fault_telemetry_report.json"
PROJECT_ROOT = Path(__file__).resolve().parents[1]


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a hardware live diagnostics readiness checklist."
    )
    parser.add_argument(
        "--transport",
        choices=["socketcan", "pcan", "serial_bridge"],
        required=True,
        help="Live transport to validate.",
    )
    parser.add_argument("--channel", help="CAN channel override, e.g. can0.")
    parser.add_argument("--profile-file", help="Optional JSON profile override file.")
    parser.add_argument("--serial-port", help="Serial bridge port, e.g. COM5.")
    parser.add_argument("--baudrate", type=int, help="Serial bridge baudrate.")
    parser.add_argument("--bitrate", type=int)
    parser.add_argument("--fault-table-file", default=DEFAULT_FAULT_TABLE)
    parser.add_argument("--recovery-policy-file", default=DEFAULT_RECOVERY_POLICY)
    parser.add_argument(
        "--output",
        default="test_env/hardware_live/live_diagnostics_checklist.json",
    )
    parser.add_argument("--diagnostics-output", default=DEFAULT_REPORT)
    parser.add_argument("--telemetry-output", default=DEFAULT_TELEMETRY)
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


def _profile_file_status(args: argparse.Namespace) -> dict[str, Any]:
    valid, resolved_path, error = _resolve_relative_path(
        args.profile_file,
        base_dir=Path(args.output).resolve().parent,
        allow_empty=True,
    )
    return {
        "path": _text(args.profile_file),
        "path_valid": valid,
        "path_error": error,
        "resolved_path": str(resolved_path) if resolved_path is not None else None,
        "exists": bool(resolved_path and resolved_path.exists()),
    }


def _load_profile_overrides(profile_status: dict[str, Any]) -> dict[str, Any]:
    if not profile_status["path"] or not profile_status["path_valid"]:
        return {}
    resolved_path = profile_status["resolved_path"]
    if not resolved_path or not Path(resolved_path).exists():
        return {}
    return json.loads(Path(resolved_path).read_text(encoding="utf-8"))


def _input_value(args: argparse.Namespace, profile: dict[str, Any], key: str) -> Any:
    value = getattr(args, key)
    return value if value is not None else profile.get(key)


def _effective_inputs(
    args: argparse.Namespace,
    *,
    profile_status: dict[str, Any] | None = None,
) -> dict[str, Any]:
    profile_status = profile_status or _profile_file_status(args)
    profile = _load_profile_overrides(profile_status)
    return {
        "profile_file": args.profile_file,
        "channel": _input_value(args, profile, "channel"),
        "serial_port": _input_value(args, profile, "serial_port"),
        "baudrate": _input_value(args, profile, "baudrate"),
        "bitrate": _input_value(args, profile, "bitrate") or 1_000_000,
        "fault_table_file": (
            _input_value(args, profile, "fault_table_file") or args.fault_table_file
        ),
        "recovery_policy_file": (
            _input_value(args, profile, "recovery_policy_file")
            or args.recovery_policy_file
        ),
    }


def _diagnostics_command(args: argparse.Namespace, inputs: dict[str, Any]) -> list[str]:
    command = [
        "python",
        "tools/run_hardware_transport_diagnostics.py",
        "--transport",
        args.transport,
        "--bitrate",
        str(inputs["bitrate"]),
        "--fault-table-file",
        inputs["fault_table_file"],
        "--recovery-policy-file",
        inputs["recovery_policy_file"],
        "--attempt-connect",
        "--output",
        args.diagnostics_output,
        "--telemetry-output",
        args.telemetry_output,
    ]
    if inputs["channel"]:
        command.extend(["--channel", str(inputs["channel"])])
    if inputs["serial_port"]:
        command.extend(["--serial-port", str(inputs["serial_port"])])
    if inputs["baudrate"]:
        command.extend(["--baudrate", str(inputs["baudrate"])])
    return command


def build_hardware_live_diagnostics_checklist(args: argparse.Namespace) -> dict[str, Any]:
    profile_status = _profile_file_status(args)
    inputs = _effective_inputs(args, profile_status=profile_status)
    missing_inputs: list[str] = []
    blockers: list[str] = []
    if not profile_status["path_valid"]:
        blockers.append("profile_file_path_invalid")
    if args.transport in {"socketcan", "pcan"} and not inputs["channel"]:
        missing_inputs.append("channel")
    if args.transport == "serial_bridge":
        if not inputs["serial_port"]:
            missing_inputs.append("serial_port")
        if not inputs["baudrate"]:
            missing_inputs.append("baudrate")

    if missing_inputs:
        blockers.append("transport_inputs_missing")
    diagnostics_command = _diagnostics_command(args, inputs)
    return {
        "schema_version": "1.0",
        "status": "blocked" if blockers else "ready_to_run",
        "transport": args.transport,
        "blockers": blockers,
        "missing_inputs": missing_inputs,
        "inputs": inputs,
        "profile_file_status": profile_status,
        "diagnostics_command": diagnostics_command,
        "evidence": {
            "diagnostics_report": args.diagnostics_output,
            "fault_telemetry_report": args.telemetry_output,
        },
        "checklist": [
            {
                "id": "physical_safety",
                "owner": "hardware_operator",
                "status": "manual_required",
                "description": "Confirm power, emergency stop, mechanical clearance, and safe recovery posture.",
            },
            {
                "id": "transport_inputs",
                "owner": "delivery_engineer",
                "status": "blocked" if missing_inputs else "ready",
                "description": "Confirm live transport channel or serial bridge parameters.",
            },
            {
                "id": "vendor_tables",
                "owner": "delivery_engineer",
                "status": "ready",
                "description": "Use external fault table and recovery policy for diagnostics.",
            },
            {
                "id": "run_diagnostics",
                "owner": "delivery_engineer",
                "status": "ready_to_run" if not missing_inputs else "blocked",
                "command": " ".join(diagnostics_command),
            },
            {
                "id": "archive_evidence",
                "owner": "release_engineer",
                "status": "pending",
                "description": "Archive diagnostics and fault telemetry reports into live evidence.",
            },
        ],
        "completion_criteria": [
            "hardware_transport_diagnostics_report.status == ready",
            "hardware_fault_telemetry_report.entries is not empty",
            "fault classes match the selected vendor fault table",
            "recovery policy source is recorded in the diagnostics report",
        ],
    }


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    checklist = build_hardware_live_diagnostics_checklist(args)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(checklist, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if checklist["status"] == "ready_to_run" else 1


if __name__ == "__main__":
    raise SystemExit(main())
