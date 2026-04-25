from __future__ import annotations

import subprocess
from pathlib import Path

from agi_walker.core.api.release_ops_contracts import WorktreeReleaseBlockerRequest
from agi_walker.ops.worktree import execute_worktree_release_blocker


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


def test_execute_worktree_release_blocker_marks_clean_repo(tmp_path: Path) -> None:
    source_root = _init_git_repo(tmp_path)
    output_root = tmp_path / "worktree_release_blocker"

    result = execute_worktree_release_blocker(
        WorktreeReleaseBlockerRequest(
            source_root=str(source_root),
            output_root=str(output_root),
        )
    )

    assert result.payload["status"] == "ready"
    assert result.payload["clean_worktree"] is True
    assert result.payload["tracked_review_status"] == "not_required"
    assert result.report_path.exists()
    assert result.cleanup_report_path.exists()


def test_execute_worktree_release_blocker_runs_tracked_review_when_required(
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

    result = execute_worktree_release_blocker(
        WorktreeReleaseBlockerRequest(
            source_root=str(source_root),
            output_root=str(tmp_path / "worktree_release_blocker"),
        )
    )

    assert result.payload["status"] == "blocked"
    assert result.payload["clean_worktree"] is False
    assert result.payload["staged_paths"] == 0
    assert result.payload["unstaged_tracked_paths"] == 2
    assert result.payload["staged_and_unstaged_paths"] == 0
    assert result.payload["tracked_review_candidate_count"] == 2
    assert result.payload["tracked_review_status"] == "ready"
    assert result.tracked_review_report_path.exists()
    assert len(result.tracked_review_payload.get("entries", [])) == 2
