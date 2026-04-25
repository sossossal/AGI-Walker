#!/usr/bin/env python
"""Run instruction-control validation smoke and write a structured summary."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


_configure_stdio()
PROJECT_ROOT = _find_repo_root()

SPECIALIZED_REPORTS = {
    "godot_instruction_smoke": Path(
        "godot_instruction_smoke/godot_instruction_smoke_report.json"
    ),
    "ros2_instruction_smoke": Path("ros2_instruction_smoke/ros2_instruction_smoke_report.json"),
    "simulated_circuit_replay_smoke": Path(
        "simulated_circuit_smoke/simulated_circuit_replay_smoke_report.json"
    ),
}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run AGI-Walker instruction-control validation smoke."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to locate tests/run_smoke_tests.py.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "instruction_control_validation"),
        help="Output root used for smoke artifacts.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional explicit summary report path. Defaults to <output-root>/instruction_control_validation_report.json.",
    )
    parser.add_argument(
        "--python-executable",
        default=sys.executable,
        help="Python executable used to run tests/run_smoke_tests.py.",
    )
    parser.add_argument(
        "--skip-smoke-run",
        action="store_true",
        help="Reuse existing artifacts under output-root instead of rerunning smoke.",
    )
    return parser


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _infer_report_status(payload: dict[str, Any]) -> str:
    explicit_status = payload.get("status")
    if isinstance(explicit_status, str) and explicit_status:
        return explicit_status

    latest_result = payload.get("latest_result")
    if isinstance(latest_result, dict):
        latest_status = latest_result.get("status")
        if latest_status == "applied":
            return "passed"
        if isinstance(latest_status, str) and latest_status:
            return latest_status

    event_name = payload.get("event")
    if isinstance(event_name, str) and event_name.endswith("_applied"):
        return "passed"

    return "unknown"


def _run_smoke(
    project_root: Path,
    output_root: Path,
    python_executable: str,
) -> subprocess.CompletedProcess[str]:
    command = [
        python_executable,
        "tests/run_smoke_tests.py",
        "--output-root",
        str(output_root),
    ]
    return subprocess.run(
        command,
        cwd=str(project_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )


def _collect_specialized_reports(output_root: Path) -> tuple[dict[str, Any], list[str]]:
    summary: dict[str, Any] = {}
    missing: list[str] = []
    for name, relative_path in SPECIALIZED_REPORTS.items():
        report_path = output_root / relative_path
        if not report_path.exists():
            missing.append(str(relative_path).replace("\\", "/"))
            summary[name] = {"status": "missing", "path": str(report_path)}
            continue
        payload = _load_json(report_path)
        summary[name] = {
            "status": _infer_report_status(payload),
            "path": str(report_path),
        }
    return summary, missing


def build_validation_report(
    *,
    project_root: Path,
    output_root: Path,
    smoke_result: subprocess.CompletedProcess[str] | None,
) -> dict[str, Any]:
    smoke_report_path = output_root / "smoke_report.json"
    smoke_report = _load_json(smoke_report_path) if smoke_report_path.exists() else None
    specialized_reports, missing_reports = _collect_specialized_reports(output_root)

    failure_reasons: list[str] = []
    status = "passed"

    if smoke_result is not None and smoke_result.returncode != 0:
        status = "blocked"
        failure_reasons.append(f"smoke_runner_exit_code={smoke_result.returncode}")

    if smoke_report is None:
        status = "blocked"
        failure_reasons.append("smoke_report_missing")
    elif smoke_report.get("status") != "passed":
        status = "blocked"
        failure_reasons.append(f"smoke_report_status={smoke_report.get('status')}")

    if missing_reports:
        status = "blocked"
        failure_reasons.append(f"specialized_reports_missing={len(missing_reports)}")

    for name, payload in specialized_reports.items():
        if payload["status"] != "passed":
            status = "blocked"
            failure_reasons.append(f"{name}={payload['status']}")

    return {
        "schema_version": "1.0",
        "artifact_type": "instruction_control_validation_report",
        "status": status,
        "generated_at": _now_iso(),
        "project_root": str(project_root),
        "output_root": str(output_root),
        "smoke_report": {
            "path": str(smoke_report_path),
            "exists": smoke_report is not None,
            "status": None if smoke_report is None else smoke_report.get("status"),
        },
        "specialized_reports": specialized_reports,
        "missing_reports": missing_reports,
        "failure_reasons": failure_reasons,
        "smoke_runner": None
        if smoke_result is None
        else {
            "returncode": smoke_result.returncode,
            "stdout_tail": smoke_result.stdout.splitlines()[-10:],
            "stderr_tail": smoke_result.stderr.splitlines()[-10:],
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    project_root = Path(args.project_root)
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_file = (
        Path(args.report_file)
        if args.report_file is not None
        else output_root / "instruction_control_validation_report.json"
    )

    smoke_result: subprocess.CompletedProcess[str] | None = None
    if not args.skip_smoke_run:
        smoke_result = _run_smoke(project_root, output_root, args.python_executable)

    payload = build_validation_report(
        project_root=project_root,
        output_root=output_root,
        smoke_result=smoke_result,
    )
    report_file.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"instruction_control_validation_report_written={report_file}")
    print(f"instruction_control_validation_status={payload['status']}")
    print(f"instruction_control_validation_missing_reports={payload['missing_reports']}")
    return 0 if payload["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
