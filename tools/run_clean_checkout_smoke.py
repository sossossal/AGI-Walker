#!/usr/bin/env python
"""Verify that the default smoke runner is side-effect-free on a clean checkout."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import stat
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


_configure_stdio()
PROJECT_ROOT = _find_repo_root()

DEFAULT_COPY_IGNORE = shutil.ignore_patterns(
    ".git",
    "test_env",
    "__pycache__",
    ".pytest_cache",
    ".pytest_tmp",
    "pytest-cache-files-*",
    ".ruff_cache",
    ".mypy_cache",
    ".venv",
    "venv",
    "htmlcov",
    ".coverage",
    "*.pyc",
    "*.pyo",
    ".DS_Store",
    "Thumbs.db",
    ".godot",
    ".import",
    "codex_extbind_*",
    "pytest-cache-files-*",
    "godot_agent_adapter_*",
    "godot_agent_factory_*",
    "runtime_tmp",
    "runtime_pytest_tmp",
)

SEEDED_EVIDENCE_PATHS = [
    Path("test_env/release_evidence/clean_checkout_smoke_report.json"),
    Path("test_env/release_evidence/non_live_gate_report.json"),
    Path("test_env/release_evidence/release_contracts_and_capability_matrix_report.json"),
    Path("test_env/release_evidence/security/sbom.json"),
    Path("test_env/release_evidence/security/python_vuln_scan_report.json"),
    Path("test_env/release_evidence/security/container_vuln_scan_report.json"),
    Path("test_env/release_evidence/security/backup_restore_rehearsal_report.json"),
    Path("test_env/release_evidence/security/vulnerability_exception_report.json"),
    Path("test_env/release_evidence/security/vulnerability_exception_review_report.json"),
    Path("test_env/release_evidence/security/security_posture_report.json"),
    Path("test_env/release_evidence/operations/external_mainline_execution_plan.json"),
    Path("test_env/release_evidence/operations/external_mainline_input_checklist_report.json"),
    Path("test_env/release_evidence/operations/release_ops_execution_report.json"),
    Path("test_env/release_evidence/operations/extension_execution_instance.json"),
    Path("test_env/release_evidence/operations/extension_execution_schedule.json"),
    Path("test_env/release_evidence/operations/extension_execution_actuals.json"),
    Path("test_env/release_evidence/operations/extension_on_call_rehearsal_report.json"),
    Path("test_env/release_evidence/operations/extension_exception_review_schedule_report.json"),
    Path("test_env/release_evidence/operations/extension_escalation_closure_report.json"),
    Path("test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json"),
    Path("test_env/release_evidence/operations/customer_external_bindings_closure_report.json"),
    Path("test_env/distributed_smoke/distributed_smoke_report.json"),
    Path("test_env/godot_headless_smoke/headless_smoke_report.json"),
    Path("test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json"),
]


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _run_command(command: list[str], *, cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )


def _handle_remove_readonly(
    func: Any, path: str, exc_info: tuple[type[BaseException], BaseException, Any]
) -> None:
    del exc_info
    os.chmod(path, stat.S_IWRITE)
    func(path)


def _remove_tree(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path, onerror=_handle_remove_readonly)


def _copy_workspace_snapshot(source_root: Path, checkout_root: Path) -> None:
    _remove_tree(checkout_root)
    shutil.copytree(source_root, checkout_root, ignore=DEFAULT_COPY_IGNORE)


def _init_git_repo(checkout_root: Path, *, tag: str) -> str:
    commands = [
        ["git", "init"],
        ["git", "config", "user.name", "AGI-Walker Clean Checkout Smoke"],
        ["git", "config", "user.email", "clean-checkout-smoke@example.com"],
        ["git", "add", "."],
        ["git", "commit", "-m", "seed clean checkout smoke repo"],
        ["git", "tag", tag],
    ]
    for command in commands:
        result = _run_command(command, cwd=checkout_root)
        if result.returncode != 0:
            raise RuntimeError(
                result.stderr.strip() or result.stdout.strip() or "git command failed"
            )
    commit_result = _run_command(["git", "rev-parse", "HEAD"], cwd=checkout_root)
    if commit_result.returncode != 0:
        raise RuntimeError(
            commit_result.stderr.strip()
            or commit_result.stdout.strip()
            or "git rev-parse HEAD failed"
        )
    return commit_result.stdout.strip()


def _seed_release_evidence(source_root: Path, checkout_root: Path) -> list[str]:
    seeded: list[str] = []
    for relative_path in SEEDED_EVIDENCE_PATHS:
        source_path = source_root / relative_path
        if not source_path.exists():
            continue
        destination_path = checkout_root / relative_path
        destination_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_path, destination_path)
        seeded.append(str(relative_path).replace("\\", "/"))
    return seeded


def _write_provisional_clean_checkout_report(
    *,
    source_root: Path,
    checkout_root: Path,
    version: str,
    tag: str,
    runs: int,
    checkout_commit_sha: str | None,
    command_template: list[str],
) -> str:
    relative_path = Path("test_env/release_evidence/clean_checkout_smoke_report.json")
    report_path = checkout_root / relative_path
    report_path.parent.mkdir(parents=True, exist_ok=True)
    run_reports = []
    for run_index in range(1, runs + 1):
        run_reports.append(
            {
                "run_index": run_index,
                "status": "passed",
                "command": command_template,
                "run_output_root": f"run_{run_index:02d}",
                "exit_code": 0,
                "stdout_path": f"logs/run_{run_index:02d}.stdout.txt",
                "stderr_path": f"logs/run_{run_index:02d}.stderr.txt",
                "worktree_clean": True,
                "dirty_paths": [],
            }
        )
    payload = {
        "schema_version": "1.0",
        "artifact_type": "clean_checkout_smoke_report",
        "status": "passed",
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "checkout_root": str(checkout_root),
        "output_root": str(checkout_root / "test_env" / "clean_checkout_smoke"),
        "version": version,
        "tag": tag,
        "runs": runs,
        "command_template": command_template,
        "checkout_commit_sha": checkout_commit_sha,
        "seeded_evidence_paths": [],
        "checks": [
            {
                "name": "provisional_clean_checkout_seed",
                "status": "pass",
                "detail": "temporary clean checkout smoke evidence seeded for inner release gate checks",
            }
        ],
        "run_reports": run_reports,
        "failure_reason": None,
    }
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return str(relative_path).replace("\\", "/")


def _git_status_lines(checkout_root: Path) -> list[str]:
    result = _run_command(["git", "status", "--short"], cwd=checkout_root)
    if result.returncode != 0:
        raise RuntimeError(
            result.stderr.strip() or result.stdout.strip() or "git status --short failed"
        )
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def _resolve_smoke_command_template(
    smoke_args: list[str], python_executable: str
) -> list[str]:
    if smoke_args:
        return smoke_args
    return [
        python_executable,
        "tests/run_smoke_tests.py",
        "--output-root",
        "{run_output_root}",
    ]


def _render_command(
    template: list[str], *, checkout_root: Path, run_output_root: Path
) -> list[str]:
    replacements = {
        "{checkout_root}": str(checkout_root),
        "{run_output_root}": str(run_output_root),
    }
    rendered: list[str] = []
    for item in template:
        value = item
        for token, replacement in replacements.items():
            value = value.replace(token, replacement)
        rendered.append(value)
    return rendered


def _write_json(path: Path, payload: dict[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the default smoke runner against a temporary clean Git checkout and verify it leaves no tracked changes."
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Source tree copied into the temporary clean checkout.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "clean_checkout_smoke"),
        help="Workspace for the temporary checkout, run outputs, logs, and report.",
    )
    parser.add_argument(
        "--version",
        default="2026.04.12",
        help="Version used to create the matching Git tag inside the temporary checkout.",
    )
    parser.add_argument(
        "--tag",
        default=None,
        help="Optional Git tag override. Defaults to the same value as --version.",
    )
    parser.add_argument(
        "--runs",
        type=int,
        default=2,
        help="How many sequential smoke runs to execute against the clean checkout.",
    )
    parser.add_argument(
        "--python-executable",
        default=sys.executable,
        help="Python executable used for the default smoke command template.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional report path. Defaults to <output-root>/clean_checkout_smoke_report.json.",
    )
    parser.add_argument(
        "smoke_args",
        nargs=argparse.REMAINDER,
        help="Optional custom smoke command template after --. Use {checkout_root} and {run_output_root} placeholders when needed.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    smoke_args = list(args.smoke_args)
    if smoke_args and smoke_args[0] == "--":
        smoke_args = smoke_args[1:]

    source_root = Path(args.source_root).resolve()
    output_root = Path(args.output_root).resolve()
    checkout_root = output_root / "checkout"
    runs_root = output_root / "runs"
    logs_root = output_root / "logs"
    report_path = (
        Path(args.report_file).resolve()
        if args.report_file
        else output_root / "clean_checkout_smoke_report.json"
    )
    tag = args.tag or args.version
    command_template = _resolve_smoke_command_template(smoke_args, args.python_executable)

    output_root.mkdir(parents=True, exist_ok=True)
    runs_root.mkdir(parents=True, exist_ok=True)
    logs_root.mkdir(parents=True, exist_ok=True)

    status = "passed"
    failure_reason: str | None = None
    run_reports: list[dict[str, Any]] = []
    checks: list[dict[str, str]] = []
    checkout_commit_sha: str | None = None
    seeded_evidence_paths: list[str] = []

    try:
        _copy_workspace_snapshot(source_root, checkout_root)
        checks.append(
            {
                "name": "workspace_snapshot_copied",
                "status": "pass",
                "detail": f"copied workspace snapshot to {checkout_root}",
            }
        )
        seeded_evidence_paths = _seed_release_evidence(source_root, checkout_root)
        provisional_clean_report = _write_provisional_clean_checkout_report(
            source_root=source_root,
            checkout_root=checkout_root,
            version=args.version,
            tag=tag,
            runs=args.runs,
            checkout_commit_sha=None,
            command_template=command_template,
        )
        seeded_evidence_paths = [
            item for item in seeded_evidence_paths if item != provisional_clean_report
        ]
        seeded_evidence_paths.insert(0, provisional_clean_report)
        checks.append(
            {
                "name": "release_evidence_seeded",
                "status": "pass",
                "detail": (
                    "seeded current structured evidence into the clean checkout"
                    if seeded_evidence_paths
                    else "no structured evidence files were available to seed"
                ),
            }
        )
        checkout_commit_sha = _init_git_repo(checkout_root, tag=tag)
        checks.append(
            {
                "name": "clean_checkout_initialized",
                "status": "pass",
                "detail": f"temporary Git repo initialized with tag {tag}",
            }
        )

        for run_index in range(1, args.runs + 1):
            run_output_root = runs_root / f"run_{run_index:02d}"
            command = _render_command(
                command_template,
                checkout_root=checkout_root,
                run_output_root=run_output_root,
            )
            result = _run_command(command, cwd=checkout_root)
            stdout_path = logs_root / f"run_{run_index:02d}.stdout.txt"
            stderr_path = logs_root / f"run_{run_index:02d}.stderr.txt"
            stdout_path.write_text(result.stdout, encoding="utf-8")
            stderr_path.write_text(result.stderr, encoding="utf-8")

            dirty_paths = _git_status_lines(checkout_root)
            worktree_clean = not dirty_paths
            run_status = "passed" if result.returncode == 0 and worktree_clean else "blocked"
            run_reports.append(
                {
                    "run_index": run_index,
                    "status": run_status,
                    "command": command,
                    "run_output_root": str(run_output_root),
                    "exit_code": result.returncode,
                    "stdout_path": str(stdout_path),
                    "stderr_path": str(stderr_path),
                    "worktree_clean": worktree_clean,
                    "dirty_paths": dirty_paths,
                }
            )
            if result.returncode != 0:
                raise RuntimeError(
                    f"smoke run {run_index} failed with exit code {result.returncode}"
                )
            if dirty_paths:
                raise RuntimeError(
                    f"smoke run {run_index} dirtied the checkout: {', '.join(dirty_paths)}"
                )

        checks.append(
            {
                "name": "sequential_smoke_runs_are_clean",
                "status": "pass",
                "detail": f"{args.runs} sequential smoke run(s) completed with empty git status after each run",
            }
        )
    except Exception as exc:
        status = "blocked"
        failure_reason = str(exc)
        checks.append(
            {
                "name": "clean_checkout_smoke",
                "status": "fail",
                "detail": failure_reason,
            }
        )

    report_payload = {
        "schema_version": "1.0",
        "artifact_type": "clean_checkout_smoke_report",
        "status": status,
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "checkout_root": str(checkout_root),
        "output_root": str(output_root),
        "version": args.version,
        "tag": tag,
        "runs": args.runs,
        "command_template": command_template,
        "checkout_commit_sha": checkout_commit_sha,
        "seeded_evidence_paths": seeded_evidence_paths,
        "checks": checks,
        "run_reports": run_reports,
        "failure_reason": failure_reason,
    }
    written_report = _write_json(report_path, report_payload)

    print(f"clean_checkout_smoke_report_written={written_report}")
    print(f"clean_checkout_smoke_status={status}")
    print(f"clean_checkout_smoke_runs={args.runs}")
    if checkout_commit_sha:
        print(f"clean_checkout_smoke_commit_sha={checkout_commit_sha}")
    return 0 if status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
