from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _init_git_repo(tmp_path: Path) -> Path:
    repo_root = tmp_path / "git_source"
    repo_root.mkdir(parents=True, exist_ok=True)
    runtime_path = (
        repo_root
        / ".agi_data"
        / "workflows"
        / "artifacts"
        / "robot_creation_pipeline_smoke_real"
        / "01_create_model.json"
    )
    runtime_path.parent.mkdir(parents=True, exist_ok=True)
    runtime_path.write_text('{"status":"old"}\n', encoding="utf-8")
    generated_path = repo_root / "godot_project" / ".godot_agent_index.json"
    generated_path.parent.mkdir(parents=True, exist_ok=True)
    generated_path.write_text('{"symbols":[]}\n', encoding="utf-8")
    commands = [
        ["git", "init"],
        ["git", "config", "user.name", "Codex Test"],
        ["git", "config", "user.email", "codex@example.com"],
        ["git", "add", "."],
        ["git", "commit", "-m", "init"],
    ]
    for command in commands:
        result = subprocess.run(
            command,
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        assert result.returncode == 0, result.stderr
    return repo_root


def test_tracked_artifact_review_report_summarizes_candidates(tmp_path: Path) -> None:
    source_root = _init_git_repo(tmp_path)
    runtime_path = (
        source_root
        / ".agi_data"
        / "workflows"
        / "artifacts"
        / "robot_creation_pipeline_smoke_real"
        / "01_create_model.json"
    )
    runtime_path.write_text(
        '{\n  "schema_version": "1.0",\n  "artifact_type": "workflow_step"\n}\n',
        encoding="utf-8",
    )
    generated_path = source_root / "godot_project" / ".godot_agent_index.json"
    generated_path.write_text(
        '{\n  "symbols": ["_init"],\n  "refs": []\n}\n',
        encoding="utf-8",
    )

    cleanup_report_path = tmp_path / "cleanup" / "worktree_cleanup_report.json"
    cleanup_result = subprocess.run(
        [
            sys.executable,
            "tools/build_worktree_cleanup_report.py",
            "--source-root",
            str(source_root),
            "--report-file",
            str(cleanup_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    assert cleanup_result.returncode == 0, cleanup_result.stderr

    report_path = tmp_path / "review" / "tracked_artifact_review_report.json"
    result = subprocess.run(
        [
            sys.executable,
            "tools/build_tracked_artifact_review_report.py",
            "--source-root",
            str(source_root),
            "--cleanup-report",
            str(cleanup_report_path),
            "--report-file",
            str(report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    assert "tracked_artifact_review_candidates=2" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "tracked_artifact_review_report"
    assert payload["tracked_candidate_count"] == 2

    entries = {item["path"]: item for item in payload["entries"]}
    assert (
        ".agi_data/workflows/artifacts/robot_creation_pipeline_smoke_real/01_create_model.json"
        in entries
    )
    assert "godot_project/.godot_agent_index.json" in entries
    assert entries["godot_project/.godot_agent_index.json"]["recommendation"]["action"] == (
        "review_tracking_policy"
    )
    assert entries[
        ".agi_data/workflows/artifacts/robot_creation_pipeline_smoke_real/01_create_model.json"
    ]["recommendation"]["action"] == "review_then_revert_or_rebaseline"
    assert entries["godot_project/.godot_agent_index.json"]["diff_lines_added"] > 0


def test_tracked_artifact_review_report_handles_no_candidates(tmp_path: Path) -> None:
    source_root = _init_git_repo(tmp_path)
    cleanup_report_path = tmp_path / "cleanup" / "worktree_cleanup_report.json"
    cleanup_result = subprocess.run(
        [
            sys.executable,
            "tools/build_worktree_cleanup_report.py",
            "--source-root",
            str(source_root),
            "--report-file",
            str(cleanup_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    assert cleanup_result.returncode == 0, cleanup_result.stderr

    report_path = tmp_path / "review" / "tracked_artifact_review_report.json"
    result = subprocess.run(
        [
            sys.executable,
            "tools/build_tracked_artifact_review_report.py",
            "--source-root",
            str(source_root),
            "--cleanup-report",
            str(cleanup_report_path),
            "--report-file",
            str(report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["tracked_candidate_count"] == 0
    assert payload["entries"] == []
    assert payload["next_step_plan"] == ["当前没有 tracked runtime/generated artifact 候选。"]
