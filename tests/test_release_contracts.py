import json
import subprocess
from pathlib import Path

import pytest

from agi_walker.core.api.release_contracts import (
    RELEASE_CONTRACT_VERSION,
    apply_release_test_evidence_to_capability_matrix,
    build_release_manifest_artifact,
    default_release_test_evidence,
    hydrate_release_test_evidence,
    validate_release_manifest_artifact,
    write_release_manifest_artifact,
)


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
    (repo_root / "README.md").write_text("# stable release test repo\n", encoding="utf-8")
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
            check=False,
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
            check=False,
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


def _valid_release_manifest(*, project_root: Path | None = None):
    return build_release_manifest_artifact(
        build_id="build-20260412-001",
        version="2026.04.12-rc1",
        channel="rc",
        release_summary="阶段五发布门禁闭环。",
        generated_at="2026-04-12T12:00:00+00:00",
        project_root=project_root,
        source_root=PROJECT_ROOT,
    )


def test_release_manifest_accepts_canonical_payload(tmp_path):
    payload = _valid_release_manifest(project_root=tmp_path)

    assert payload["schema_version"] == RELEASE_CONTRACT_VERSION
    assert payload["artifact_type"] == "release_manifest"
    assert payload["release_gate_status"] == "ready_with_limitations"
    assert payload["release_policy"] == {
        "channel": "rc",
        "allows_opt_in_evidence": False,
        "allows_diagnostic_ready_domains": False,
        "requires_release_approval": False,
        "requires_git_source_binding": False,
        "requires_clean_worktree": False,
        "requires_version_tag_match": False,
        "summary": "RC releases require optional live evidence and diagnostic-ready domains to be fully closed before the gate becomes ready.",
    }
    assert payload["release_approval"] == {
        "status": "not_required",
        "required": False,
        "approved_by": None,
        "approved_at": None,
        "commit_sha": None,
        "notes": None,
    }
    assert payload["release_source"] == _repo_git_source("2026.04.12-rc1")
    assert payload["release_gate"] == {
        "required_evidence": 3,
        "passed_required_evidence": 3,
        "blocked_evidence": 0,
        "blocked_optional_evidence": 0,
        "opt_in_evidence": 3,
        "diagnostic_ready_domains": 1,
        "release_approval_required": 0,
        "release_approval_ready": 1,
        "release_source_required": 0,
        "release_source_ready": 1,
        "release_worktree_required": 0,
        "release_worktree_ready": 1,
        "release_version_tag_required": 0,
        "release_version_tag_ready": 1,
    }
    assert validate_release_manifest_artifact(payload) == []

    output_path = write_release_manifest_artifact(
        payload, tmp_path / "release_manifest.json"
    )
    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(saved) == []
    assert saved["version"] == "2026.04.12-rc1"


def test_release_manifest_rejects_invalid_payload():
    errors = validate_release_manifest_artifact(
        {
            "schema_version": "bad",
            "artifact_type": "release_manifest",
            "build_id": "",
            "version": "",
            "channel": "nightly",
            "release_policy": {"channel": "dev"},
            "release_approval": {"status": "pending"},
            "release_source": {"resolved_from_git": "yes"},
            "release_summary": "",
            "generated_at": "",
            "release_gate_status": "green",
            "release_gate": {
                "required_evidence": -1,
            },
            "changelog": {"path": "", "title": ""},
            "contract_versions": [{"name": "", "version": ""}],
            "capability_matrix": {},
            "test_evidence": [{"name": "", "required": "yes", "status": "done"}],
            "known_limitations": [""],
        }
    )

    assert "schema_version must be '1.0', got 'bad'" in errors
    assert "channel must be one of ['dev', 'rc', 'stable']" in errors
    assert "release_gate_status must be one of ['blocked', 'ready', 'ready_with_limitations']" in errors
    assert "known_limitations[1] must be a non-empty string" in errors


def test_write_release_manifest_rejects_invalid_payload(tmp_path):
    with pytest.raises(ValueError, match="missing required fields"):
        write_release_manifest_artifact(
            {"artifact_type": "release_manifest"}, tmp_path / "bad.json"
        )


def test_release_manifest_hydrates_distributed_report_and_updates_matrix(
    tmp_path: Path,
):
    report_path = (
        tmp_path / "test_env" / "distributed_smoke" / "distributed_smoke_report.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "status": "passed",
                "actor_id": "actor-ci",
                "checks": [
                    {"name": "compose_build", "status": "pass"},
                    {"name": "compose_up", "status": "pass"},
                    {"name": "web_panel_status", "status": "pass"},
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )

    evidence = hydrate_release_test_evidence(project_root=tmp_path)
    distributed = next(
        item for item in evidence if item["name"] == "distributed_runtime_live"
    )
    assert distributed["status"] == "passed"
    assert "3/3 checks passed" in distributed["summary"]

    payload = build_release_manifest_artifact(
        build_id="build-20260412-001",
        version="2026.04.12-rc1",
        channel="rc",
        release_summary="阶段五发布门禁闭环。",
        generated_at="2026-04-12T12:00:00+00:00",
        project_root=tmp_path,
        source_root=PROJECT_ROOT,
    )

    assert payload["release_gate"]["blocked_evidence"] == 0
    distributed_domain = next(
        item
        for item in payload["capability_matrix"]["domains"]
        if item["id"] == "distributed_runtime"
    )
    assert distributed_domain["status"] == "ready"
    assert payload["capability_matrix"]["summary"]["diagnostic_ready_domains"] == 0


def test_apply_release_test_evidence_to_capability_matrix_preserves_default_when_missing():
    matrix = apply_release_test_evidence_to_capability_matrix(
        test_evidence=default_release_test_evidence()
    )

    distributed_domain = next(
        item for item in matrix["domains"] if item["id"] == "distributed_runtime"
    )
    assert distributed_domain["status"] == "diagnostic_ready"


def test_release_manifest_becomes_ready_when_all_live_evidence_pass(tmp_path: Path):
    reports = {
        tmp_path / "test_env" / "distributed_smoke" / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "actor_id": "actor-ci",
            "checks": [
                {"name": "compose_build", "status": "pass"},
                {"name": "compose_up", "status": "pass"},
                {"name": "web_panel_status", "status": "pass"},
            ],
        },
        tmp_path / "test_env" / "godot_headless_smoke" / "headless_smoke_report.json": {
            "status": "passed"
        },
        tmp_path / "test_env" / "ros2_bridge_smoke" / "ros2_bridge_smoke_report.json": {
            "status": "passed"
        },
    }
    for path, payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, ensure_ascii=False),
            encoding="utf-8",
        )

    payload = _valid_release_manifest(project_root=tmp_path)

    assert payload["release_gate_status"] == "ready"
    assert payload["release_gate"] == {
        "required_evidence": 3,
        "passed_required_evidence": 3,
        "blocked_evidence": 0,
        "blocked_optional_evidence": 0,
        "opt_in_evidence": 0,
        "diagnostic_ready_domains": 0,
        "release_approval_required": 0,
        "release_approval_ready": 1,
        "release_source_required": 0,
        "release_source_ready": 1,
        "release_worktree_required": 0,
        "release_worktree_ready": 1,
        "release_version_tag_required": 0,
        "release_version_tag_ready": 1,
    }
    assert payload["known_limitations"]


def test_dev_release_policy_allows_open_opt_in_evidence(tmp_path: Path):
    payload = build_release_manifest_artifact(
        build_id="build-20260412-007",
        version="2026.04.12-dev1",
        channel="dev",
        release_summary="开发通道验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        project_root=tmp_path,
        source_root=PROJECT_ROOT,
    )

    assert payload["release_policy"] == {
        "channel": "dev",
        "allows_opt_in_evidence": True,
        "allows_diagnostic_ready_domains": True,
        "requires_release_approval": False,
        "requires_git_source_binding": False,
        "requires_clean_worktree": False,
        "requires_version_tag_match": False,
        "summary": "Dev releases may remain ready when required evidence passes, even if optional live evidence or diagnostic-ready domains are still open.",
    }
    assert payload["release_gate_status"] == "ready"
    assert payload["release_gate"] == {
        "required_evidence": 3,
        "passed_required_evidence": 3,
        "blocked_evidence": 0,
        "blocked_optional_evidence": 0,
        "opt_in_evidence": 3,
        "diagnostic_ready_domains": 1,
        "release_approval_required": 0,
        "release_approval_ready": 1,
        "release_source_required": 0,
        "release_source_ready": 1,
        "release_worktree_required": 0,
        "release_worktree_ready": 1,
        "release_version_tag_required": 0,
        "release_version_tag_ready": 1,
    }


def test_stable_release_requires_explicit_approval(tmp_path: Path):
    payload = build_release_manifest_artifact(
        build_id="build-20260412-008",
        version="2026.04.12",
        channel="stable",
        release_summary="稳定通道验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        project_root=tmp_path,
        source_root=PROJECT_ROOT,
    )

    assert payload["release_policy"] == {
        "channel": "stable",
        "allows_opt_in_evidence": False,
        "allows_diagnostic_ready_domains": False,
        "requires_release_approval": True,
        "requires_git_source_binding": True,
        "requires_clean_worktree": True,
        "requires_version_tag_match": True,
        "summary": "Stable releases require optional live evidence, diagnostic-ready domains, explicit release approval, Git HEAD binding, a clean worktree, and a matching version tag before the gate becomes ready.",
    }
    assert payload["release_approval"] == {
        "status": "pending",
        "required": True,
        "approved_by": None,
        "approved_at": None,
        "commit_sha": None,
        "notes": None,
    }
    assert payload["release_source"] == _repo_git_source("2026.04.12")
    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["release_approval_required"] == 1
    assert payload["release_gate"]["release_approval_ready"] == 0
    assert payload["release_gate"]["release_source_required"] == 1
    assert payload["release_gate"]["release_source_ready"] == 0
    assert payload["release_gate"]["release_worktree_required"] == 1
    assert payload["release_gate"]["release_worktree_ready"] == 0
    assert payload["release_gate"]["release_version_tag_required"] == 1
    assert payload["release_gate"]["release_version_tag_ready"] == 0


def test_stable_release_becomes_ready_after_approval_and_live_evidence(tmp_path: Path):
    repo_source = _init_git_repo(tmp_path, tag="2026.04.12")
    reports = {
        tmp_path / "test_env" / "distributed_smoke" / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "actor_id": "actor-ci",
            "checks": [
                {"name": "compose_build", "status": "pass"},
                {"name": "compose_up", "status": "pass"},
                {"name": "web_panel_status", "status": "pass"},
            ],
        },
        tmp_path / "test_env" / "godot_headless_smoke" / "headless_smoke_report.json": {
            "status": "passed"
        },
        tmp_path / "test_env" / "ros2_bridge_smoke" / "ros2_bridge_smoke_report.json": {
            "status": "passed"
        },
    }
    for path, report_payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report_payload, ensure_ascii=False), encoding="utf-8")

    payload = build_release_manifest_artifact(
        build_id="build-20260412-009",
        version="2026.04.12",
        channel="stable",
        release_summary="稳定通道签核验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "notes": "stable signoff",
        },
        release_source={key: value for key, value in repo_source.items() if key != "root"},
        project_root=tmp_path,
        source_root=repo_source["root"],
    )

    assert payload["release_gate_status"] == "ready"
    assert payload["release_approval"]["commit_sha"] == repo_source["commit_sha"]
    assert payload["release_source"] == {
        key: value for key, value in repo_source.items() if key != "root"
    }
    assert payload["release_gate"] == {
        "required_evidence": 3,
        "passed_required_evidence": 3,
        "blocked_evidence": 0,
        "blocked_optional_evidence": 0,
        "opt_in_evidence": 0,
        "diagnostic_ready_domains": 0,
        "release_approval_required": 1,
        "release_approval_ready": 1,
        "release_source_required": 1,
        "release_source_ready": 1,
        "release_worktree_required": 1,
        "release_worktree_ready": 1,
        "release_version_tag_required": 1,
        "release_version_tag_ready": 1,
    }


def test_stable_release_blocks_when_approval_commit_does_not_match_head(tmp_path: Path):
    reports = {
        tmp_path / "test_env" / "distributed_smoke" / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "checks": [{"name": "compose_build", "status": "pass"}],
        },
        tmp_path / "test_env" / "godot_headless_smoke" / "headless_smoke_report.json": {
            "status": "passed"
        },
        tmp_path / "test_env" / "ros2_bridge_smoke" / "ros2_bridge_smoke_report.json": {
            "status": "passed"
        },
    }
    for path, report_payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report_payload, ensure_ascii=False), encoding="utf-8")

    payload = build_release_manifest_artifact(
        build_id="build-20260412-010",
        version="2026.04.12",
        channel="stable",
        release_summary="稳定通道签核验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "commit_sha": "deadbeef",
            "notes": "stable signoff",
        },
        project_root=tmp_path,
        source_root=PROJECT_ROOT,
    )

    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["release_approval_ready"] == 1
    assert payload["release_gate"]["release_source_ready"] == 0
    assert payload["release_gate"]["release_worktree_ready"] == 0


def test_stable_release_blocks_without_matching_version_tag(tmp_path: Path):
    repo_source = _repo_git_source("2026.04.12")
    reports = {
        tmp_path / "test_env" / "distributed_smoke" / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "checks": [{"name": "compose_build", "status": "pass"}],
        },
        tmp_path / "test_env" / "godot_headless_smoke" / "headless_smoke_report.json": {
            "status": "passed"
        },
        tmp_path / "test_env" / "ros2_bridge_smoke" / "ros2_bridge_smoke_report.json": {
            "status": "passed"
        },
    }
    for path, report_payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report_payload, ensure_ascii=False), encoding="utf-8")

    payload = build_release_manifest_artifact(
        build_id="build-20260412-011",
        version="2026.04.12",
        channel="stable",
        release_summary="稳定通道 tag 验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "notes": "stable signoff",
        },
        release_source=repo_source,
        project_root=tmp_path,
        source_root=PROJECT_ROOT,
    )

    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["release_source_ready"] == 1
    assert payload["release_gate"]["release_worktree_ready"] == 0
    assert payload["release_gate"]["release_version_tag_ready"] == 0


def test_stable_release_blocks_with_dirty_worktree_even_when_tag_and_approval_match(
    tmp_path: Path,
):
    repo_source = _init_git_repo(tmp_path, tag="2026.04.12")
    dirty_source = {key: value for key, value in repo_source.items() if key != "root"}
    dirty_source["worktree_clean"] = False
    dirty_source["worktree_status_summary"] = "1 pending path(s): M README.md"
    reports = {
        tmp_path / "test_env/distributed_smoke/distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "checks": [{"name": "compose_build", "status": "pass"}],
        },
        tmp_path / "test_env/godot_headless_smoke/headless_smoke_report.json": {
            "status": "passed"
        },
        tmp_path / "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json": {
            "status": "passed"
        },
    }
    for path, report_payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report_payload, ensure_ascii=False), encoding="utf-8")

    payload = build_release_manifest_artifact(
        build_id="build-20260412-012",
        version="2026.04.12",
        channel="stable",
        release_summary="稳定通道 dirty worktree 验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "notes": "stable signoff",
        },
        release_source=dirty_source,
        project_root=tmp_path,
        source_root=repo_source["root"],
    )

    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["release_source_ready"] == 1
    assert payload["release_gate"]["release_worktree_required"] == 1
    assert payload["release_gate"]["release_worktree_ready"] == 0
