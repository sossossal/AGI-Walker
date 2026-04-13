from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _init_git_repo(tmp_path: Path, *, tag: str | None = None) -> tuple[Path, str]:
    repo_root = tmp_path / "git_source"
    repo_root.mkdir(parents=True, exist_ok=True)
    (repo_root / "README.md").write_text("# stable promotion repo\n", encoding="utf-8")
    for command in [
        ["git", "init"],
        ["git", "config", "user.name", "Codex Test"],
        ["git", "config", "user.email", "codex@example.com"],
        ["git", "add", "README.md"],
        ["git", "commit", "-m", "init"],
    ]:
        result = subprocess.run(
            command,
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        assert result.returncode == 0, result.stderr
    if tag is not None:
        result = subprocess.run(
            ["git", "tag", tag],
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )
        assert result.returncode == 0, result.stderr

    head = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=str(repo_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    ).stdout.strip()
    return repo_root, head


def _seed_live_evidence(project_root: Path) -> None:
    reports = {
        project_root / "test_env" / "distributed_smoke" / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "checks": [{"name": "compose_build", "status": "pass"}],
        },
        project_root / "test_env" / "godot_headless_smoke" / "headless_smoke_report.json": {
            "status": "passed"
        },
        project_root / "test_env" / "ros2_bridge_smoke" / "ros2_bridge_smoke_report.json": {
            "status": "passed"
        },
    }
    for path, payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


def test_stable_promotion_checklist_reports_pending_prerequisites(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root, head = _init_git_repo(tmp_path)
    _seed_live_evidence(project_root)
    report_path = project_root / "test_env" / "stable_promotion" / "checklist.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_stable_promotion_checklist.py",
            "--current-version",
            "2026.04.12-rc6",
            "--stable-version",
            "2026.04.12",
            "--project-root",
            str(project_root),
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
    assert "stable_promotion_gate=blocked" in result.stdout
    assert "stable_promotion_blocking_steps=3" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "stable_promotion_checklist"
    assert payload["stable_release_gate"] == "blocked"
    assert payload["current_head_commit"] == head
    assert payload["ready_to_promote"] is False

    step_ids = {step["id"]: step for step in payload["steps"]}
    assert step_ids["stable_approval"]["status"] == "pending"
    assert step_ids["git_source_binding"]["status"] == "pending"
    assert step_ids["clean_worktree"]["status"] == "done"
    assert step_ids["version_tag"]["status"] == "pending"
    assert step_ids["build_stable_manifest"]["ready_to_run"] is False
    assert "git tag 2026.04.12" in step_ids["version_tag"]["command"]


def test_stable_promotion_checklist_reports_ready_to_promote(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root, head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    report_path = project_root / "test_env" / "stable_promotion" / "checklist.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_stable_promotion_checklist.py",
            "--current-version",
            "2026.04.12-rc6",
            "--stable-version",
            "2026.04.12",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--approval-status",
            "approved",
            "--approved-by",
            "release-manager",
            "--approved-at",
            "2026-04-12T12:30:00+00:00",
            "--approval-notes",
            "stable signoff",
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
    assert "stable_promotion_gate=ready" in result.stdout
    assert "stable_promotion_blocking_steps=0" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["stable_release_gate"] == "ready"
    assert payload["ready_to_promote"] is True
    assert payload["current_head_commit"] == head
    assert payload["matched_version_tag"] == "2026.04.12"

    step_ids = {step["id"]: step for step in payload["steps"]}
    assert step_ids["stable_approval"]["status"] == "done"
    assert step_ids["git_source_binding"]["status"] == "done"
    assert step_ids["clean_worktree"]["status"] == "done"
    assert step_ids["version_tag"]["status"] == "done"
    assert step_ids["build_stable_manifest"]["ready_to_run"] is True
    assert "--channel stable" in step_ids["build_stable_manifest"]["command"]


def test_stable_promotion_checklist_blocks_on_dirty_worktree(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root, _head = _init_git_repo(tmp_path, tag="2026.04.12")
    (source_root / "README.md").write_text("# dirty repo\n", encoding="utf-8")
    _seed_live_evidence(project_root)
    report_path = project_root / "test_env" / "stable_promotion" / "checklist.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_stable_promotion_checklist.py",
            "--current-version",
            "2026.04.12-rc6",
            "--stable-version",
            "2026.04.12",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--approval-status",
            "approved",
            "--approved-by",
            "release-manager",
            "--approved-at",
            "2026-04-12T12:30:00+00:00",
            "--approval-notes",
            "stable signoff",
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
    assert "stable_promotion_gate=blocked" in result.stdout
    assert "stable_promotion_blocking_steps=1" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    step_ids = {step["id"]: step for step in payload["steps"]}
    assert payload["ready_to_promote"] is False
    assert step_ids["stable_approval"]["status"] == "done"
    assert step_ids["git_source_binding"]["status"] == "done"
    assert step_ids["clean_worktree"]["status"] == "pending"
    assert "tools/build_worktree_cleanup_report.py" in step_ids["clean_worktree"]["command"]
    assert step_ids["version_tag"]["status"] == "done"
    assert step_ids["build_stable_manifest"]["ready_to_run"] is False
