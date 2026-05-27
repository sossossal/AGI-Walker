"""Run or discover Godot control/communication replay evidence."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from agi_walker.core.simulation.control_comm_simulation import (  # noqa: E402
    build_default_control_comm_scenario,
    validate_control_comm_scenario,
    validate_godot_control_comm_simulation_log,
)

GODOT_CONTROL_COMM_REPLAY_REPORT_VERSION = "godot_control_comm_replay_report.v1"
DEFAULT_SCRIPT = "res://scripts/control_comm_replay.gd"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a Godot headless replay for control communication logs."
    )
    parser.add_argument("--scenario", type=Path, help="Scenario JSON path.")
    parser.add_argument(
        "--scenario-id",
        default="default_joint_command_stream",
        help="Scenario id used when --scenario is omitted.",
    )
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--cycle-period-ns", type=int, default=10_000_000)
    parser.add_argument("--godot-exe", type=Path)
    parser.add_argument("--project", type=Path, default=ROOT / "godot_project")
    parser.add_argument("--script", default=DEFAULT_SCRIPT)
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "test_env" / "godot_control_comm_replay",
    )
    parser.add_argument("--timeout-seconds", type=float, default=20.0)
    parser.add_argument(
        "--dry-run-discovery",
        action="store_true",
        help="Only write discovery metadata; do not launch Godot.",
    )
    args = parser.parse_args()

    args.output_root.mkdir(parents=True, exist_ok=True)
    scenario_path = args.scenario or args.output_root / "godot_replay_scenario.json"
    if args.scenario:
        scenario = json.loads(args.scenario.read_text(encoding="utf-8"))
    else:
        scenario = build_default_control_comm_scenario(
            scenario_id=args.scenario_id,
            cycle_count=args.cycles,
            cycle_period_ns=args.cycle_period_ns,
        )
        _write_json(scenario_path, scenario)

    scenario_errors = validate_control_comm_scenario(scenario)
    if scenario_errors:
        report = _base_report(args, scenario_path=scenario_path)
        report.update(
            {
                "status": "error",
                "failure_category": "invalid_scenario",
                "errors": scenario_errors,
            }
        )
        _write_report(args.output_root, report)
        return 1

    executable = _resolve_godot_executable(args.godot_exe)
    scenario_path = scenario_path.resolve()
    report = _base_report(args, scenario_path=scenario_path, executable=executable)
    if args.dry_run_discovery:
        if not executable["available"]:
            report["status"] = "blocked"
            report["failure_category"] = "missing_godot_executable"
            report["residual_risks"].append("godot_executable_missing")
        _write_report(args.output_root, report)
        return 0

    if not executable["available"]:
        report["status"] = "blocked"
        report["failure_category"] = "missing_godot_executable"
        report["errors"].append("Godot executable was not found")
        report["residual_risks"].append("godot_executable_missing")
        _write_report(args.output_root, report)
        return 1

    log_path = (args.output_root / "godot_control_comm_simulation_log.json").resolve()
    command = [
        executable["resolved_path"],
        "--headless",
        "--path",
        str(args.project),
        "--script",
        args.script,
        "--",
        "--control-comm-scenario",
        str(scenario_path),
        "--control-comm-log",
        str(log_path),
    ]
    report["command"] = command
    try:
        completed = subprocess.run(
            command,
            cwd=ROOT,
            text=True,
            capture_output=True,
            timeout=args.timeout_seconds,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        report["status"] = "error"
        report["failure_category"] = "godot_replay_timeout"
        report["errors"].append(str(exc))
        _write_report(args.output_root, report)
        return 1

    report["exit_code"] = completed.returncode
    report["stdout_tail"] = _tail_lines(completed.stdout)
    report["stderr_tail"] = _tail_lines(completed.stderr)
    if completed.returncode != 0:
        report["status"] = "error"
        report["failure_category"] = "godot_replay_failed"
        report["errors"].append(f"Godot replay exited with {completed.returncode}")
        _write_report(args.output_root, report)
        return 1
    if not log_path.exists():
        report["status"] = "error"
        report["failure_category"] = "godot_log_missing"
        report["errors"].append("Godot replay did not write the expected log artifact")
        _write_report(args.output_root, report)
        return 1

    log_payload = json.loads(log_path.read_text(encoding="utf-8"))
    log_errors = validate_godot_control_comm_simulation_log(log_payload)
    report["log_validation_errors"] = log_errors
    report["artifact_paths"]["godot_control_comm_simulation_log"] = str(log_path)
    report["godot_log_summary"] = {
        "log_version": log_payload.get("log_version"),
        "status": log_payload.get("status"),
        "scenario_id": log_payload.get("scenario_id"),
        "message_event_count": log_payload.get("message_event_count"),
    }
    if log_errors:
        report["status"] = "error"
        report["failure_category"] = "invalid_godot_log"
        report["errors"].extend(log_errors)
        _write_report(args.output_root, report)
        return 1

    report["status"] = "success"
    report["failure_category"] = None
    report["residual_risks"] = ["real_hardware_transport_not_run"]
    _write_report(args.output_root, report)
    return 0


def _base_report(
    args: argparse.Namespace,
    *,
    scenario_path: Path,
    executable: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "report_version": GODOT_CONTROL_COMM_REPLAY_REPORT_VERSION,
        "status": "success",
        "failure_category": None,
        "dry_run": bool(args.dry_run_discovery),
        "project": str(args.project),
        "script": args.script,
        "scenario_path": str(scenario_path),
        "godot_executable": executable or _resolve_godot_executable(args.godot_exe),
        "command": [],
        "exit_code": None,
        "artifact_paths": {
            "scenario": str(scenario_path),
            "report": str(args.output_root / "godot_control_comm_replay_report.json"),
        },
        "godot_log_summary": {},
        "log_validation_errors": [],
        "stdout_tail": [],
        "stderr_tail": [],
        "residual_risks": [],
        "errors": [],
    }


def _resolve_godot_executable(requested: Path | None) -> dict[str, Any]:
    candidates: list[tuple[str, str]] = []
    if requested is not None:
        candidates.append(("argument", str(requested)))
    for env_name in ["GODOT_EXECUTABLE", "GODOT", "GODOT_EXE", "GODOT_PATH"]:
        value = os.environ.get(env_name)
        if value:
            candidates.append((env_name, value))
    for binary in ["godot", "godot4", "Godot"]:
        value = shutil.which(binary)
        if value:
            candidates.append(("PATH", value))
    source, path_text = candidates[0] if candidates else ("not_found", "")
    path = Path(path_text) if path_text else None
    available = bool(path and path.exists() and path.is_file())
    return {
        "source": source,
        "requested": path_text,
        "resolved_path": str(path) if path else None,
        "available": available,
    }


def _write_report(output_root: Path, report: dict[str, Any]) -> None:
    path = output_root / "godot_control_comm_replay_report.json"
    _write_json(path, report)
    print(f"godot_control_comm_replay_report={path}")
    print(f"godot_control_comm_replay_status={report['status']}")
    if report["failure_category"]:
        print(f"godot_control_comm_replay_failure_category={report['failure_category']}")


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _tail_lines(value: str, limit: int = 40) -> list[str]:
    if not value:
        return []
    return value.splitlines()[-limit:]


if __name__ == "__main__":
    raise SystemExit(main())
