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


def test_worktree_cleanup_report_categorizes_paths(tmp_path: Path) -> None:
    source_root = _init_git_repo(tmp_path)
    (source_root / "README.md").write_text("# modified repo\n", encoding="utf-8")
    runtime_path = source_root / ".agi_data" / "sessions" / "session-1.json"
    runtime_path.parent.mkdir(parents=True, exist_ok=True)
    runtime_path.write_text("{}", encoding="utf-8")
    generated_path = source_root / "configs" / "generated" / "probe.json"
    generated_path.parent.mkdir(parents=True, exist_ok=True)
    generated_path.write_text("{}", encoding="utf-8")
    unknown_path = source_root / "misc" / "note.txt"
    unknown_path.parent.mkdir(parents=True, exist_ok=True)
    unknown_path.write_text("todo", encoding="utf-8")

    report_path = tmp_path / "worktree_cleanup" / "worktree_cleanup_report.json"
    result = subprocess.run(
        [
            sys.executable,
            "tools/build_worktree_cleanup_report.py",
            "--source-root",
            str(source_root),
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
    assert "worktree_cleanup_clean=false" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "worktree_cleanup_report"
    assert payload["clean_worktree"] is False
    assert payload["total_paths"] == 4
    assert payload["staged_paths"] == 0
    assert payload["unstaged_tracked_paths"] == 1
    assert payload["staged_and_unstaged_paths"] == 0
    assert payload["tracked_review_candidate_count"] == 0
    assert payload["tracked_review_candidate_paths"] == []
    tracked_review_report_path = Path(payload["tracked_review_report_path"])
    assert tracked_review_report_path.parent == report_path.parent
    assert tracked_review_report_path.name == "tracked_artifact_review_report.json"
    assert payload["tracked_review_command"] is None

    entries = {item["path"]: item for item in payload["paths"]}
    assert entries["README.md"]["category"] == "manual_review"
    assert entries[".agi_data/sessions/session-1.json"]["category"] == "runtime_artifact"
    assert entries["configs/generated/probe.json"]["category"] == "generated_artifact"
    assert entries["misc/note.txt"]["category"] == "unknown"

    summary = {item["id"]: item for item in payload["category_summary"]}
    assert summary["manual_review"]["count"] == 1
    assert summary["runtime_artifact"]["count"] == 1
    assert summary["generated_artifact"]["count"] == 1
    assert summary["unknown"]["count"] == 1
    status_summary = {item["id"]: item for item in payload["status_summary"]}
    assert status_summary["modified"]["count"] == 1
    assert status_summary["untracked"]["count"] == 3
    assert len(payload["next_step_plan"]) == 4
    assert "运行时产物" in "".join(payload["next_step_plan"])


def test_worktree_cleanup_report_treats_root_markdown_as_manual_review(
    tmp_path: Path,
) -> None:
    source_root = _init_git_repo(tmp_path)
    runbook_path = source_root / "PRODUCTION_DEPLOYMENT_RUNBOOK.md"
    runbook_path.write_text("# runbook\n", encoding="utf-8")
    note_path = source_root / "agentization.md"
    note_path.write_text("# agentization\n", encoding="utf-8")

    report_path = tmp_path / "worktree_cleanup" / "worktree_cleanup_report.json"
    result = subprocess.run(
        [
            sys.executable,
            "tools/build_worktree_cleanup_report.py",
            "--source-root",
            str(source_root),
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
    entries = {item["path"]: item for item in payload["paths"]}
    assert entries["PRODUCTION_DEPLOYMENT_RUNBOOK.md"]["category"] == "manual_review"
    assert entries["agentization.md"]["category"] == "manual_review"
    summary = {item["id"]: item for item in payload["category_summary"]}
    assert "unknown" not in summary


def test_worktree_cleanup_report_marks_clean_repo(tmp_path: Path) -> None:
    source_root = _init_git_repo(tmp_path)
    report_path = tmp_path / "worktree_cleanup" / "worktree_cleanup_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_worktree_cleanup_report.py",
            "--source-root",
            str(source_root),
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
    assert "worktree_cleanup_clean=true" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["clean_worktree"] is True
    assert payload["total_paths"] == 0
    assert payload["staged_paths"] == 0
    assert payload["unstaged_tracked_paths"] == 0
    assert payload["staged_and_unstaged_paths"] == 0
    assert payload["tracked_review_candidate_count"] == 0
    assert payload["tracked_review_candidate_paths"] == []
    assert payload["tracked_review_command"] is None
    assert payload["category_summary"] == []
    assert payload["status_summary"] == []
    assert payload["next_step_plan"] == ["工作区已 clean，可继续 stable promotion。"]


def test_worktree_cleanup_report_surfaces_tracked_review_follow_up(tmp_path: Path) -> None:
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

    report_path = tmp_path / "worktree_cleanup" / "worktree_cleanup_report.json"
    result = subprocess.run(
        [
            sys.executable,
            "tools/build_worktree_cleanup_report.py",
            "--source-root",
            str(source_root),
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
    assert "worktree_cleanup_tracked_review_candidates=2" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["tracked_review_candidate_count"] == 2
    assert payload["tracked_review_candidate_paths"] == [
        ".agi_data/sessions/session-1.json",
        "configs/generated/probe.generated.json",
    ]
    assert "python tools/build_tracked_artifact_review_report.py" in (
        payload["tracked_review_command"] or ""
    )
    assert "tracked_artifact_review_report.py" in "".join(payload["next_step_plan"])


def test_worktree_cleanup_report_summarizes_staged_and_unstaged_split(
    tmp_path: Path,
) -> None:
    source_root = _init_git_repo(tmp_path)
    readme_path = source_root / "README.md"
    readme_path.write_text("# staged repo\n", encoding="utf-8")
    subprocess.run(
        ["git", "add", "README.md"],
        cwd=str(source_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )
    readme_path.write_text("# staged and unstaged repo\n", encoding="utf-8")
    portal_path = source_root / "web_panel" / "server.py"
    portal_path.parent.mkdir(parents=True, exist_ok=True)
    portal_path.write_text("print('portal')\n", encoding="utf-8")

    report_path = tmp_path / "worktree_cleanup" / "worktree_cleanup_report.json"
    result = subprocess.run(
        [
            sys.executable,
            "tools/build_worktree_cleanup_report.py",
            "--source-root",
            str(source_root),
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
    assert payload["staged_paths"] == 1
    assert payload["unstaged_tracked_paths"] == 1
    assert payload["staged_and_unstaged_paths"] == 1
    status_summary = {item["id"]: item for item in payload["status_summary"]}
    assert status_summary["staged_and_unstaged"]["count"] == 1
    assert status_summary["untracked"]["count"] == 1
    assert "当前已有 1 个路径进入暂存区" in payload["next_step_plan"][0]
