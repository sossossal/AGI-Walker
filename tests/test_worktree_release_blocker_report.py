from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _init_git_repo(tmp_path: Path) -> Path:
    repo_root = tmp_path / "git_source"
    repo_root.mkdir(parents=True, exist_ok=True)
    (repo_root / "README.md").write_text("# cleanup repo\n", encoding="utf-8")
    commands = [
        ["git", "init"],
        ["git", "config", "user.name", "Codex Test"],
        ["git", "config", "user.email", "codex@example.com"],
        ["git", "add", "README.md"],
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


def test_worktree_release_blocker_report_marks_clean_repo(tmp_path: Path) -> None:
    source_root = _init_git_repo(tmp_path)
    output_root = tmp_path / "worktree_release_blocker"
    report_path = output_root / "worktree_release_blocker_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_worktree_release_blocker.py",
            "--source-root",
            str(source_root),
            "--output-root",
            str(output_root),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    assert "worktree_release_blocker_status=ready" in result.stdout
    assert "worktree_release_blocker_clean=true" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "worktree_release_blocker_report"
    assert payload["status"] == "ready"
    assert payload["clean_worktree"] is True
    assert payload["tracked_review_status"] == "not_required"
    assert Path(payload["cleanup_report_path"]).name == "worktree_cleanup_report.json"
    assert Path(payload["tracked_review_report_path"]).name == "tracked_artifact_review_report.json"


def test_worktree_release_blocker_report_runs_tracked_review_when_required(
    tmp_path: Path,
) -> None:
    source_root = _init_git_repo(tmp_path)
    runtime_path = source_root / ".agi_data" / "sessions" / "session-1.json"
    runtime_path.parent.mkdir(parents=True, exist_ok=True)
    runtime_path.write_text('{"status":"tracked"}\n', encoding="utf-8")
    generated_path = source_root / "configs" / "generated" / "probe.generated.json"
    generated_path.parent.mkdir(parents=True, exist_ok=True)
    generated_path.write_text('{"status":"tracked"}\n', encoding="utf-8")
    subprocess.run(
        ["git", "add", "."],
        cwd=str(source_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )
    subprocess.run(
        ["git", "commit", "-m", "track artifacts"],
        cwd=str(source_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )
    runtime_path.write_text('{"status":"tracked-updated"}\n', encoding="utf-8")
    generated_path.write_text('{"status":"tracked-updated"}\n', encoding="utf-8")

    output_root = tmp_path / "worktree_release_blocker"
    report_path = output_root / "worktree_release_blocker_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_worktree_release_blocker.py",
            "--source-root",
            str(source_root),
            "--output-root",
            str(output_root),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    assert "worktree_release_blocker_status=blocked" in result.stdout
    assert "worktree_release_blocker_clean=false" in result.stdout
    assert "worktree_release_blocker_tracked_review_status=ready" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["clean_worktree"] is False
    assert payload["staged_paths"] == 0
    assert payload["unstaged_tracked_paths"] == 2
    assert payload["staged_and_unstaged_paths"] == 0
    assert payload["tracked_review_candidate_count"] == 2
    assert payload["tracked_review_required"] is True
    assert payload["tracked_review_status"] == "ready"
    assert payload["tracked_review_entries"] == 2
    assert Path(payload["cleanup_report_path"]).exists()
    assert Path(payload["tracked_review_report_path"]).exists()
    assert payload["status_summary"]
