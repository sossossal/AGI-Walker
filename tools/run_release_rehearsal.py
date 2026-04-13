#!/usr/bin/env python
"""Run an end-to-end stable release gate rehearsal in a temporary Git repo."""

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


_configure_stdio()
PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_contracts import validate_release_manifest_artifact  # noqa: E402


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


def _init_git_repo(source_root: Path, *, version: str, tag: str) -> dict[str, Any]:
    source_root.mkdir(parents=True, exist_ok=True)
    (source_root / "README.md").write_text(
        f"# Release rehearsal for {version}\n",
        encoding="utf-8",
    )
    commands = [
        ["git", "init"],
        ["git", "config", "user.name", "AGI-Walker Release Rehearsal"],
        ["git", "config", "user.email", "release-rehearsal@example.com"],
        ["git", "add", "README.md"],
        ["git", "commit", "-m", "seed rehearsal repo"],
        ["git", "tag", tag],
    ]
    for command in commands:
        result = _run_command(command, cwd=source_root)
        if result.returncode != 0:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip() or "git command failed")

    commit_sha = _run_command(["git", "rev-parse", "HEAD"], cwd=source_root).stdout.strip()
    short_commit_sha = _run_command(
        ["git", "rev-parse", "--short=12", "HEAD"],
        cwd=source_root,
    ).stdout.strip()
    return {
        "source_root": str(source_root),
        "commit_sha": commit_sha,
        "short_commit_sha": short_commit_sha,
        "tag": tag,
    }


def _seed_live_evidence(project_root: Path) -> list[dict[str, str]]:
    reports = {
        project_root / "test_env" / "distributed_smoke" / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "actor_id": "release-rehearsal-actor",
            "checks": [
                {"name": "compose_build", "status": "pass"},
                {"name": "compose_up", "status": "pass"},
                {"name": "web_panel_status", "status": "pass"},
            ],
        },
        project_root / "test_env" / "godot_headless_smoke" / "headless_smoke_report.json": {
            "status": "passed",
            "message": "Seeded Godot headless live smoke evidence for release rehearsal.",
        },
        project_root / "test_env" / "ros2_bridge_smoke" / "ros2_bridge_smoke_report.json": {
            "status": "passed",
            "message": "Seeded ROS2 bridge live smoke evidence for release rehearsal.",
        },
    }
    seeded_paths: list[dict[str, str]] = []
    for path, payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        seeded_paths.append({"name": path.name, "path": str(path)})
    return seeded_paths


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a stable release rehearsal with a temporary Git source repo."
    )
    parser.add_argument(
        "--version",
        default="2026.04.12-rehearsal",
        help="Version used for the rehearsal manifest and matching tag.",
    )
    parser.add_argument(
        "--tag",
        default=None,
        help="Optional tag override. Defaults to the same value as --version.",
    )
    parser.add_argument(
        "--build-id",
        default="release-rehearsal",
        help="Stable build identifier written into the rehearsal manifest.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "release_rehearsal"),
        help="Workspace used for the temporary Git repo, reports, and manifest.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional override path for the structured rehearsal report.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = (
        Path(args.report_file)
        if args.report_file
        else output_root / "release_rehearsal_report.json"
    )
    source_root = output_root / "git_source"
    manifest_path = output_root / "release_manifest.json"
    tag = args.tag or args.version

    checks: list[dict[str, str]] = []
    status = "failed"
    builder_stdout = ""
    builder_stderr = ""
    git_source: dict[str, Any] | None = None
    evidence_paths: list[dict[str, str]] = []

    try:
        git_source = _init_git_repo(source_root, version=args.version, tag=tag)
        checks.append(
            {
                "name": "git_repo_init",
                "status": "pass",
                "detail": f"temporary Git repo initialized at {source_root}",
            }
        )
        checks.append(
            {
                "name": "version_tag_created",
                "status": "pass",
                "detail": f"created matching tag {tag}",
            }
        )

        evidence_paths = _seed_live_evidence(output_root)
        checks.append(
            {
                "name": "live_evidence_seeded",
                "status": "pass",
                "detail": "seeded distributed, Godot, and ROS2 live evidence reports",
            }
        )

        builder = _run_command(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "build_release_artifact.py"),
                "--version",
                args.version,
                "--channel",
                "stable",
                "--build-id",
                args.build_id,
                "--release-summary",
                "Stable release rehearsal validation.",
                "--project-root",
                str(output_root),
                "--source-root",
                str(source_root),
                "--approval-status",
                "approved",
                "--approved-by",
                "release-manager",
                "--approved-at",
                _now_iso(),
                "--approval-notes",
                "stable rehearsal signoff",
                "--output",
                str(manifest_path),
            ],
            cwd=PROJECT_ROOT,
        )
        builder_stdout = builder.stdout.strip()
        builder_stderr = builder.stderr.strip()
        if builder.returncode != 0:
            raise RuntimeError(builder_stderr or builder_stdout or "release builder failed")
        checks.append(
            {
                "name": "release_manifest_built",
                "status": "pass",
                "detail": f"built stable manifest at {manifest_path}",
            }
        )

        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        validation_errors = validate_release_manifest_artifact(payload)
        if validation_errors:
            raise RuntimeError("; ".join(validation_errors))
        checks.append(
            {
                "name": "release_manifest_validated",
                "status": "pass",
                "detail": "manifest matches release contract",
            }
        )

        if payload.get("release_gate_status") != "ready":
            raise RuntimeError(
                f"expected release_gate_status='ready', got {payload.get('release_gate_status')!r}"
            )
        checks.append(
            {
                "name": "stable_gate_ready",
                "status": "pass",
                "detail": "stable rehearsal reached release_gate_status=ready",
            }
        )
        status = "passed"
    except Exception as exc:
        checks.append(
            {
                "name": "release_rehearsal",
                "status": "fail",
                "detail": str(exc),
            }
        )
        status = "failed"

    report = {
        "schema_version": "1.0",
        "artifact_type": "release_rehearsal_report",
        "status": status,
        "version": args.version,
        "tag": tag,
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "manifest_path": str(manifest_path),
        "report_path": str(report_path),
        "git_source": git_source,
        "evidence_paths": evidence_paths,
        "checks": checks,
        "builder_stdout": builder_stdout,
        "builder_stderr": builder_stderr,
    }
    written_report = _write_report(report_path, report)

    print(f"release_rehearsal_written={written_report}")
    print(f"release_rehearsal_gate={'ready' if status == 'passed' else 'blocked'}")
    print(f"release_rehearsal_tag={tag}")
    return 0 if status == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
