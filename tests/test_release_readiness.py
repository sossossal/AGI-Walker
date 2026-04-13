from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _init_git_repo(tmp_path: Path, *, tag: str | None = None) -> Path:
    repo_root = tmp_path / "git_source"
    repo_root.mkdir(parents=True, exist_ok=True)
    (repo_root / "README.md").write_text("# readiness repo\n", encoding="utf-8")
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
    return repo_root


def _seed_live_evidence(project_root: Path) -> None:
    reports = {
        project_root / "test_env" / "distributed_smoke" / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "checks": [
                {"name": "compose_build", "status": "pass"},
                {"name": "compose_up", "status": "pass"},
            ],
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


def test_release_readiness_reports_missing_tag_and_approval(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root = _init_git_repo(tmp_path, tag=None)
    _seed_live_evidence(project_root)
    report_path = project_root / "test_env" / "release_readiness" / "readiness.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/check_release_readiness.py",
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
    assert "release_readiness_written=" in result.stdout
    assert "rc_release_gate=ready" in result.stdout
    assert "stable_release_gate=blocked" in result.stdout
    assert report_path.exists()

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    stable_preview = next(item for item in payload["previews"] if item["channel"] == "stable")
    assert stable_preview["release_gate_status"] == "blocked"
    assert Path(stable_preview["manifest_path"]).parent == report_path.parent
    assert any("git tag 2026.04.12" in item for item in stable_preview["next_actions"])
    assert any("补齐 stable 签核" in item for item in stable_preview["next_actions"])


def test_release_readiness_reports_ready_when_tag_and_approval_present(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    report_path = project_root / "test_env" / "release_readiness" / "readiness.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/check_release_readiness.py",
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
    assert "stable_release_gate=ready" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    stable_preview = next(item for item in payload["previews"] if item["channel"] == "stable")
    assert stable_preview["release_gate_status"] == "ready"
    assert Path(stable_preview["manifest_path"]).parent == report_path.parent
    assert stable_preview["release_source"]["version_tag_matches"] is True
    assert stable_preview["next_actions"] == [
        "门禁已就绪，可生成最终 manifest: python tools/build_release_artifact.py --version 2026.04.12 --channel stable --build-id stable-readiness-2026.04.12"
    ]


def test_release_readiness_reports_worktree_cleanup_action_for_dirty_repo(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root = _init_git_repo(tmp_path, tag="2026.04.12")
    (source_root / "README.md").write_text("# dirty repo\n", encoding="utf-8")
    _seed_live_evidence(project_root)
    report_path = project_root / "test_env" / "release_readiness" / "readiness.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/check_release_readiness.py",
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
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    stable_preview = next(item for item in payload["previews"] if item["channel"] == "stable")
    assert stable_preview["release_gate_status"] == "blocked"
    assert any(
        "tools/build_worktree_cleanup_report.py" in item
        for item in stable_preview["next_actions"]
    )
