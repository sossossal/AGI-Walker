from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import validate_release_manifest_artifact


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _repo_git_source(version: str) -> dict[str, str | bool | None]:
    def _capture(*argv: str) -> str:
        result = subprocess.run(
            list(argv),
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=True,
        )
        return result.stdout.strip()

    tag_output = _capture("git", "tag", "--points-at", "HEAD")
    worktree_output = _capture("git", "status", "--short")
    tags = [line.strip() for line in tag_output.splitlines() if line.strip()]
    worktree_lines = [line.strip() for line in worktree_output.splitlines() if line.strip()]
    matched_version_tag = next(
        (tag for tag in tags if tag in {version, f"v{version}"}),
        None,
    )
    return {
        "resolved_from_git": True,
        "commit_sha": _capture("git", "rev-parse", "HEAD"),
        "short_commit_sha": _capture("git", "rev-parse", "--short=12", "HEAD"),
        "git_tag": matched_version_tag or (tags[0] if tags else None),
        "matched_version_tag": matched_version_tag,
        "worktree_clean": not worktree_lines,
        "worktree_status_summary": (
            f"{len(worktree_lines)} pending path(s): "
            + ", ".join(worktree_lines[:3])
            + (", ..." if len(worktree_lines) > 3 else "")
            if worktree_lines
            else None
        ),
        "version_tag_matches": matched_version_tag is not None,
    }


def _init_git_repo(tmp_path: Path, *, tag: str | None = None) -> dict[str, str | bool | None]:
    repo_root = tmp_path / "git_source"
    repo_root.mkdir(parents=True, exist_ok=True)
    (repo_root / "README.md").write_text("# temp repo\n", encoding="utf-8")
    for argv in [
        ["git", "init"],
        ["git", "config", "user.name", "Codex Test"],
        ["git", "config", "user.email", "codex@example.com"],
        ["git", "add", "README.md"],
        ["git", "commit", "-m", "init"],
    ]:
        result = subprocess.run(
            argv,
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

    def _capture(*argv: str) -> str:
        result = subprocess.run(
            list(argv),
            cwd=str(repo_root),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=True,
        )
        return result.stdout.strip()

    return {
        "root": str(repo_root),
        "resolved_from_git": True,
        "commit_sha": _capture("git", "rev-parse", "HEAD"),
        "short_commit_sha": _capture("git", "rev-parse", "--short=12", "HEAD"),
        "git_tag": tag,
        "matched_version_tag": tag,
        "worktree_clean": True,
        "worktree_status_summary": None,
        "version_tag_matches": tag is not None,
    }


def test_build_release_artifact_script_writes_valid_manifest(tmp_path: Path) -> None:
    output_path = tmp_path / "release_manifest.json"
    distributed_report = (
        tmp_path / "test_env" / "distributed_smoke" / "distributed_smoke_report.json"
    )
    distributed_report.parent.mkdir(parents=True, exist_ok=True)
    distributed_report.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "status": "passed",
                "actor_id": "actor-ci",
                "checks": [
                    {"name": "compose_build", "status": "pass"},
                    {"name": "compose_up", "status": "pass"},
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_release_artifact.py",
            "--version",
            "2026.04.12-rc1",
            "--channel",
            "rc",
            "--build-id",
            "build-20260412-001",
            "--release-summary",
            "阶段五发布门禁闭环。",
            "--project-root",
            str(tmp_path),
            "--output",
            str(output_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    assert "release_manifest_written=" in result.stdout
    assert "release_gate_status=ready_with_limitations" in result.stdout
    assert "release_source_worktree_clean=" in result.stdout
    assert output_path.exists()

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(payload) == []
    assert payload["build_id"] == "build-20260412-001"
    assert payload["release_policy"]["channel"] == "rc"
    assert payload["release_source"]["resolved_from_git"] is True
    assert payload["release_gate"]["blocked_evidence"] == 0
    assert payload["release_gate"]["blocked_optional_evidence"] == 0
    distributed_domain = next(
        item
        for item in payload["capability_matrix"]["domains"]
        if item["id"] == "distributed_runtime"
    )
    assert distributed_domain["status"] == "ready"


def test_build_release_artifact_uses_release_notes_metadata_when_summary_omitted(
    tmp_path: Path,
) -> None:
    output_path = tmp_path / "release_manifest.json"
    changelog_path = tmp_path / "RELEASE_NOTES.md"
    changelog_path.write_text(
        "\n".join(
            [
                "# AGI-Walker 2026.04.12-rc6 Release Notes",
                "",
                "发布摘要：所有 live 证据已附带，release gate 已提升为 ready。",
                "",
                "## 概览",
                "",
                "这是一份测试用 release notes。",
                "",
            ]
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_release_artifact.py",
            "--version",
            "2026.04.12-rc6",
            "--channel",
            "rc",
            "--build-id",
            "build-20260412-006",
            "--project-root",
            str(tmp_path),
            "--changelog",
            "RELEASE_NOTES.md",
            "--output",
            str(output_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(payload) == []
    assert (
        payload["release_summary"]
        == "所有 live 证据已附带，release gate 已提升为 ready。"
    )
    assert payload["changelog"]["title"] == "AGI-Walker 2026.04.12-rc6 Release Notes"


def test_build_release_artifact_stable_requires_approval_metadata(
    tmp_path: Path,
) -> None:
    git_source = _init_git_repo(tmp_path, tag="2026.04.12")
    output_path = tmp_path / "release_manifest.json"
    reports = {
        "test_env/distributed_smoke/distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "checks": [{"name": "compose_build", "status": "pass"}],
        },
        "test_env/godot_headless_smoke/headless_smoke_report.json": {"status": "passed"},
        "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json": {"status": "passed"},
    }
    for relative_path, payload in reports.items():
        report_path = tmp_path / relative_path
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_release_artifact.py",
            "--version",
            "2026.04.12",
            "--channel",
            "stable",
            "--build-id",
            "build-20260412-stable",
            "--release-summary",
            "stable 发布验证。",
            "--source-root",
            git_source["root"],
            "--approval-status",
            "approved",
            "--approved-by",
            "release-manager",
            "--approved-at",
            "2026-04-12T12:30:00+00:00",
            "--approval-notes",
            "stable signoff",
            "--project-root",
            str(tmp_path),
            "--output",
            str(output_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    assert "release_gate_status=ready" in result.stdout
    assert f"release_source_commit_sha={git_source['commit_sha']}" in result.stdout
    assert "release_source_worktree_clean=true" in result.stdout
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(payload) == []
    assert payload["channel"] == "stable"
    assert payload["release_policy"]["requires_release_approval"] is True
    assert payload["release_policy"]["requires_git_source_binding"] is True
    assert payload["release_policy"]["requires_clean_worktree"] is True
    assert payload["release_policy"]["requires_version_tag_match"] is True
    assert payload["release_approval"]["status"] == "approved"
    assert payload["release_approval"]["commit_sha"] == git_source["commit_sha"]
    assert payload["release_source"] == {
        key: value for key, value in git_source.items() if key != "root"
    }
    assert payload["release_gate"]["release_approval_ready"] == 1
    assert payload["release_gate"]["release_source_ready"] == 1
    assert payload["release_gate"]["release_worktree_ready"] == 1
    assert payload["release_gate"]["release_version_tag_ready"] == 1
