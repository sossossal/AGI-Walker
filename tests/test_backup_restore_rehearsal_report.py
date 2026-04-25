from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.security_posture_contracts import (
    build_backup_restore_rehearsal_report,
    validate_backup_restore_rehearsal_report,
    write_backup_restore_rehearsal_report,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _seed_project_root(project_root: Path) -> None:
    (project_root / "deployment").mkdir(parents=True, exist_ok=True)
    (project_root / "deployment" / "compose.env.example").write_text(
        "AGI_WALKER_RUNTIME_ROOT=./runtime\n",
        encoding="utf-8",
    )
    (project_root / "deployment" / "web_panel.env.example").write_text(
        "AGI_WALKER_WEB_PORT=8000\n",
        encoding="utf-8",
    )


def _seed_rehearsal_dirs(base_root: Path) -> tuple[Path, Path, Path, Path, Path]:
    source_runtime_root = base_root / "source_runtime"
    source_config_root = base_root / "source_config"
    backup_snapshot_root = base_root / "backup_snapshot"
    restored_runtime_root = base_root / "restored_runtime"
    restored_config_root = base_root / "restored_config"

    for path in [
        source_runtime_root / "db",
        source_runtime_root / "workflow_runs" / "run-001",
        source_runtime_root / "workflow_archive" / "archive-001",
        source_runtime_root / "backups",
        source_config_root,
        backup_snapshot_root / "db",
        backup_snapshot_root / "workflow_runs" / "run-001",
        backup_snapshot_root / "workflow_archive" / "archive-001",
        backup_snapshot_root / "backups",
        restored_runtime_root / "db",
        restored_runtime_root / "workflow_runs" / "run-001",
        restored_runtime_root / "workflow_archive" / "archive-001",
        restored_runtime_root / "backups",
        restored_config_root,
    ]:
        path.mkdir(parents=True, exist_ok=True)

    (source_runtime_root / "db" / "app.sqlite3").write_text("db\n", encoding="utf-8")
    (source_runtime_root / "workflow_runs" / "run-001" / "result.json").write_text(
        '{"status":"passed"}\n',
        encoding="utf-8",
    )
    (
        source_runtime_root / "workflow_archive" / "archive-001" / "manifest.json"
    ).write_text('{"archived":true}\n', encoding="utf-8")
    (source_runtime_root / "backups" / "README.txt").write_text(
        "backup seed\n", encoding="utf-8"
    )
    (source_config_root / "compose.env").write_text(
        "AGI_WALKER_RUNTIME_ROOT=./runtime\n", encoding="utf-8"
    )
    (source_config_root / "web_panel.env").write_text(
        "AGI_WALKER_WEB_PORT=8000\n", encoding="utf-8"
    )

    for relative_path in [
        Path("db/app.sqlite3"),
        Path("workflow_runs/run-001/result.json"),
        Path("workflow_archive/archive-001/manifest.json"),
        Path("backups/README.txt"),
    ]:
        target_backup = backup_snapshot_root / relative_path
        target_backup.parent.mkdir(parents=True, exist_ok=True)
        target_backup.write_text(
            (source_runtime_root / relative_path).read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        target_restored = restored_runtime_root / relative_path
        target_restored.parent.mkdir(parents=True, exist_ok=True)
        target_restored.write_text(
            (source_runtime_root / relative_path).read_text(encoding="utf-8"),
            encoding="utf-8",
        )

    for file_name in ["compose.env", "web_panel.env"]:
        (backup_snapshot_root / file_name).write_text(
            (source_config_root / file_name).read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        (restored_config_root / file_name).write_text(
            (source_config_root / file_name).read_text(encoding="utf-8"),
            encoding="utf-8",
        )

    return (
        source_runtime_root,
        source_config_root,
        backup_snapshot_root,
        restored_runtime_root,
        restored_config_root,
    )


def test_backup_restore_rehearsal_report_round_trip(tmp_path: Path) -> None:
    _seed_project_root(tmp_path)
    (
        source_runtime_root,
        source_config_root,
        backup_snapshot_root,
        restored_runtime_root,
        restored_config_root,
    ) = _seed_rehearsal_dirs(tmp_path / "test_env" / "rehearsal")

    payload = build_backup_restore_rehearsal_report(
        project_root=tmp_path,
        actor="test-runner",
        source_runtime_root=source_runtime_root,
        source_config_root=source_config_root,
        backup_snapshot_root=backup_snapshot_root,
        restored_runtime_root=restored_runtime_root,
        restored_config_root=restored_config_root,
        rehearsal_duration_seconds=1.5,
    )

    assert validate_backup_restore_rehearsal_report(payload) == []
    assert payload["status"] == "passed"
    assert payload["missing_backup_items"] == 0
    assert payload["failed_restore_checks"] == 0

    output_path = write_backup_restore_rehearsal_report(
        payload,
        tmp_path / "test_env" / "security" / "backup_restore_rehearsal_report.json",
    )
    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_backup_restore_rehearsal_report(saved) == []
    assert saved["status"] == "passed"


def test_run_backup_restore_rehearsal_script_writes_report(tmp_path: Path) -> None:
    _seed_project_root(tmp_path)
    output_root = tmp_path / "test_env" / "security" / "backup_restore_rehearsal"
    report_path = tmp_path / "test_env" / "security" / "backup_restore_rehearsal_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_backup_restore_rehearsal.py",
            "--project-root",
            str(tmp_path),
            "--output-root",
            str(output_root),
            "--report-file",
            str(report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "backup_restore_rehearsal_report_written=" in result.stdout
    assert "backup_restore_rehearsal_status=passed" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert validate_backup_restore_rehearsal_report(payload) == []
    assert payload["status"] == "passed"
