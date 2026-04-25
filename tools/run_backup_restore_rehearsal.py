#!/usr/bin/env python
"""Run a non-destructive backup/restore rehearsal and write a structured report."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path
from time import perf_counter


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


def _remove_tree(path: Path) -> None:
    if not path.exists():
        return

    def _handle_remove_readonly(func, current_path, exc_info):
        del exc_info
        Path(current_path).chmod(0o666)
        func(current_path)

    shutil.rmtree(path, onerror=_handle_remove_readonly)


_configure_stdio()
PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.security_posture_contracts import (  # noqa: E402
    build_backup_restore_rehearsal_report,
    write_backup_restore_rehearsal_report,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a non-destructive backup and restore rehearsal."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve deployment examples and optional release manifest.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "security" / "backup_restore_rehearsal"),
        help="Directory used for the rehearsal sandbox.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional explicit output path for the structured rehearsal report.",
    )
    parser.add_argument(
        "--actor",
        default="backup-restore-rehearsal",
        help="Actor string recorded in the rehearsal report.",
    )
    parser.add_argument(
        "--release-manifest",
        default=None,
        help="Optional related release manifest path to record in the report.",
    )
    return parser


def _copy_tree(source: Path, destination: Path) -> None:
    if destination.exists():
        _remove_tree(destination)
    shutil.copytree(source, destination)


def _copy_file(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, destination)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    project_root = Path(args.project_root)
    output_root = Path(args.output_root)
    report_file = (
        Path(args.report_file)
        if args.report_file is not None
        else output_root / "backup_restore_rehearsal_report.json"
    )

    source_runtime_root = output_root / "source_runtime"
    source_config_root = output_root / "source_config"
    backup_snapshot_root = output_root / "backup_snapshot"
    restored_runtime_root = output_root / "restored_runtime"
    restored_config_root = output_root / "restored_config"

    for path in [
        source_runtime_root,
        source_config_root,
        backup_snapshot_root,
        restored_runtime_root,
        restored_config_root,
    ]:
        if path.exists():
            _remove_tree(path)
        path.mkdir(parents=True, exist_ok=True)

    compose_env_example = project_root / "deployment" / "compose.env.example"
    web_panel_env_example = project_root / "deployment" / "web_panel.env.example"
    if not compose_env_example.is_file():
        parser.error(f"missing compose env example: {compose_env_example}")
    if not web_panel_env_example.is_file():
        parser.error(f"missing web panel env example: {web_panel_env_example}")

    (source_runtime_root / "db").mkdir(parents=True, exist_ok=True)
    (source_runtime_root / "workflow_runs" / "run-001").mkdir(parents=True, exist_ok=True)
    (source_runtime_root / "workflow_archive" / "archive-001").mkdir(
        parents=True, exist_ok=True
    )
    (source_runtime_root / "backups").mkdir(parents=True, exist_ok=True)

    (source_runtime_root / "db" / "app.sqlite3").write_text(
        "backup rehearsal db seed\n", encoding="utf-8"
    )
    (source_runtime_root / "workflow_runs" / "run-001" / "result.json").write_text(
        '{"status":"passed"}\n', encoding="utf-8"
    )
    (
        source_runtime_root / "workflow_archive" / "archive-001" / "manifest.json"
    ).write_text('{"archived":true}\n', encoding="utf-8")
    (source_runtime_root / "backups" / "README.txt").write_text(
        "backup rehearsal seed\n", encoding="utf-8"
    )

    _copy_file(compose_env_example, source_config_root / "compose.env")
    _copy_file(web_panel_env_example, source_config_root / "web_panel.env")

    started = perf_counter()

    for directory_name in ["db", "workflow_runs", "workflow_archive", "backups"]:
        _copy_tree(
            source_runtime_root / directory_name,
            backup_snapshot_root / directory_name,
        )
    _copy_file(source_config_root / "compose.env", backup_snapshot_root / "compose.env")
    _copy_file(
        source_config_root / "web_panel.env",
        backup_snapshot_root / "web_panel.env",
    )

    for directory_name in ["db", "workflow_runs", "workflow_archive", "backups"]:
        _copy_tree(
            backup_snapshot_root / directory_name,
            restored_runtime_root / directory_name,
        )
    _copy_file(backup_snapshot_root / "compose.env", restored_config_root / "compose.env")
    _copy_file(
        backup_snapshot_root / "web_panel.env",
        restored_config_root / "web_panel.env",
    )

    duration_seconds = perf_counter() - started
    payload = build_backup_restore_rehearsal_report(
        project_root=project_root,
        actor=args.actor,
        source_runtime_root=source_runtime_root,
        source_config_root=source_config_root,
        backup_snapshot_root=backup_snapshot_root,
        restored_runtime_root=restored_runtime_root,
        restored_config_root=restored_config_root,
        release_manifest_path=args.release_manifest,
        rehearsal_duration_seconds=duration_seconds,
    )
    output_path = write_backup_restore_rehearsal_report(payload, report_file)

    print(f"backup_restore_rehearsal_report_written={output_path}")
    print(f"backup_restore_rehearsal_status={payload['status']}")
    print(f"backup_restore_missing_items={payload['missing_backup_items']}")
    print(f"backup_restore_failed_checks={payload['failed_restore_checks']}")
    return 0 if payload["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
