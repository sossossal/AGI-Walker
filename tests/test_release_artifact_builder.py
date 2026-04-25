from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_extension_execution_actuals_artifact,
    build_extension_execution_instance_artifact,
    build_extension_execution_schedule_artifact,
    build_release_evidence_report,
    validate_release_evidence_report,
    validate_release_manifest_artifact,
    write_extension_execution_actuals_artifact,
    write_extension_execution_instance_artifact,
    write_extension_execution_schedule_artifact,
    write_release_evidence_report,
)
from agi_walker.core.api.security_posture_contracts import (
    build_vulnerability_exception_report,
    write_vulnerability_exception_report,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _seed_customer_delivery_docs(project_root: Path) -> None:
    docs = {
        project_root / "README.md": "# Acceptance Bundle\n",
        project_root / "docs" / "CURRENT_STATUS.md": "# Current Status\n",
        project_root / "docs" / "guides" / "RELEASE_GUIDE.md": "# Release Guide\n",
        project_root / "docs" / "guides" / "DEPLOYMENT_MATRIX.md": "# Deployment Matrix\n",
        project_root
        / "docs"
        / "guides"
        / "CUSTOMER_INSTALLATION_GUIDE.md": "# Customer Installation Guide\n",
        project_root / "docs" / "guides" / "SUPPORT_MATRIX.md": "# Support Matrix\n",
        project_root / "docs" / "guides" / "CAPACITY_AND_SCALE.md": "# Capacity And Scale\n",
        project_root
        / "docs"
        / "guides"
        / "CUSTOMER_ACCEPTANCE_CHECKLIST.md": "# Customer Acceptance Checklist\n",
        project_root / "docs" / "guides" / "KNOWN_LIMITATIONS.md": "# Known Limitations\n",
        project_root / "docs" / "guides" / "SECURITY_BASELINE.md": "# Security Baseline\n",
        project_root / "docs" / "guides" / "AUDIT_TRAIL_POLICY.md": "# Audit Trail Policy\n",
        project_root / "docs" / "guides" / "BACKUP_RESTORE_RUNBOOK.md": "# Backup Restore Runbook\n",
        project_root / "docs" / "guides" / "INCIDENT_RESPONSE_MATRIX.md": "# Incident Response Matrix\n",
        project_root / "PRODUCTION_DEPLOYMENT_RUNBOOK.md": "# Production Runbook\n",
    }
    for path, content in docs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")


def _write_clean_checkout_smoke_report(project_root: Path) -> Path:
    report_path = (
        project_root / "test_env" / "release_evidence" / "clean_checkout_smoke_report.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "clean_checkout_smoke_report",
                "status": "passed",
                "generated_at": "2026-04-13T10:18:29+00:00",
                "source_root": str(project_root),
                "checkout_root": str(project_root / "test_env" / "clean_checkout_smoke" / "checkout"),
                "output_root": str(project_root / "test_env" / "clean_checkout_smoke"),
                "version": "2026.04.12",
                "tag": "2026.04.12",
                "runs": 2,
                "command_template": [
                    "python",
                    "tests/run_smoke_tests.py",
                    "--output-root",
                    "{run_output_root}",
                ],
                "checkout_commit_sha": "clean-checkout-commit",
                "seeded_evidence_paths": [],
                "checks": [
                    {
                        "name": "workspace_snapshot_copied",
                        "status": "pass",
                        "detail": "copied workspace snapshot",
                    },
                    {
                        "name": "clean_checkout_initialized",
                        "status": "pass",
                        "detail": "temporary Git repo initialized",
                    },
                    {
                        "name": "sequential_smoke_runs_are_clean",
                        "status": "pass",
                        "detail": "2 sequential smoke run(s) completed with empty git status after each run",
                    },
                ],
                "run_reports": [
                    {
                        "run_index": 1,
                        "status": "passed",
                        "command": ["python", "tests/run_smoke_tests.py"],
                        "run_output_root": "run_01",
                        "exit_code": 0,
                        "stdout_path": "logs/run_01.stdout.txt",
                        "stderr_path": "logs/run_01.stderr.txt",
                        "worktree_clean": True,
                        "dirty_paths": [],
                    },
                    {
                        "run_index": 2,
                        "status": "passed",
                        "command": ["python", "tests/run_smoke_tests.py"],
                        "run_output_root": "run_02",
                        "exit_code": 0,
                        "stdout_path": "logs/run_02.stdout.txt",
                        "stderr_path": "logs/run_02.stderr.txt",
                        "worktree_clean": True,
                        "dirty_paths": [],
                    },
                ],
                "failure_reason": None,
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return report_path


def _seed_structured_required_evidence(project_root: Path) -> None:
    report_root = project_root / "test_env" / "release_evidence"
    report_root.mkdir(parents=True, exist_ok=True)
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="non_live_gate",
            status="passed",
            summary="non_live_gate pytest evidence passed: 854 passed, 4 skipped, 3 deselected.",
            command='python -m pytest -m "not live" -q',
            generated_at="2026-04-15T10:00:00+00:00",
            metrics={"passed": 854, "skipped": 4, "deselected": 3},
            source_commit_sha="release-artifact-builder",
        ),
        report_root / "non_live_gate_report.json",
    )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="release_contracts_and_capability_matrix",
            status="passed",
            summary="release_contracts_and_capability_matrix pytest evidence passed: 48 passed.",
            command="python -m pytest tests/test_release_contracts.py tests/test_release_artifact_builder.py tests/test_capability_matrix.py tests/test_mcp_tools.py tests/test_mcp_server.py tests/test_web_panel_aux_apis.py -q",
            generated_at="2026-04-15T10:02:00+00:00",
            metrics={"passed": 48},
            source_commit_sha="release-artifact-builder",
        ),
        report_root / "release_contracts_and_capability_matrix_report.json",
    )
    operations_root = report_root / "operations"
    operations_root.mkdir(parents=True, exist_ok=True)
    for file_name, evidence_name, summary, metrics in [
        (
            "extension_on_call_rehearsal_report.json",
            "extension_on_call_rehearsal",
            "extension_on_call_rehearsal evidence passed: 3 actionable profiles, 6 handoff record(s), 2 unique artifact path(s).",
            {"actionable_profiles": 3, "handoff_records": 6, "unique_artifact_paths": 2},
        ),
        (
            "extension_exception_review_schedule_report.json",
            "extension_exception_review_schedule",
            "extension_exception_review_schedule evidence passed: 3 actionable profiles, 6 review step(s), active_exceptions=1, next_expiry=2026-05-15T00:00:00+00:00.",
            {
                "actionable_profiles": 3,
                "review_steps": 6,
                "active_exception_count": 1,
                "next_exception_expiry": "2026-05-15T00:00:00+00:00",
            },
        ),
        (
            "extension_escalation_closure_report.json",
            "extension_escalation_closure",
            "extension_escalation_closure evidence passed: 3 actionable profiles, 6 closure step(s), 3 unique artifact path(s).",
            {"actionable_profiles": 3, "closure_steps": 6, "unique_artifact_paths": 3},
        ),
        (
            "release_ops_execution_report.json",
            "release_ops_execution",
            "release op stable_promotion_checklist completed via control plane with canonical evidence wrapper.",
            {
                "action": "stable_promotion_checklist",
                "policy_level": "local_safe_refresh",
                "policy_profile": "local_safe_refresh",
                "request_type": "StablePromotionChecklistRequest",
                "status": "ready",
                "event_count": 3,
                "output_path": "test_env/stable_promotion_ready/stable_promotion_checklist.json",
            },
        ),
        (
            "customer_external_bindings_confirmation_report.json",
            "customer_external_bindings_confirmation",
            "customer_external_bindings_confirmation evidence passed: All external binding sections are confirmed.",
            {
                "external_bindings_status": "ready",
                "confirmed_sections": [
                    "approval_identity",
                    "archive_target",
                    "due_trigger",
                ],
                "confirmation_tickets": ["CHG-RELEASE-ARTIFACT"],
            },
        ),
    ]:
        write_release_evidence_report(
            build_release_evidence_report(
                evidence_name=evidence_name,
                status="passed",
                summary=summary,
                command="python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations",
                generated_at="2026-04-15T10:03:00+00:00",
                metrics=metrics,
                source_commit_sha="release-artifact-builder",
                control_plane_session={
                    "engagement_id": "release-artifact-builder-session",
                    "window_id": "window-release-artifact",
                    "change_ticket": "CHG-RELEASE-ARTIFACT",
                    "channel": "ops-cli",
                }
                if evidence_name == "release_ops_execution"
                else None,
                control_plane_event_stream={
                    "path": "test_env/release_ops/stable_promotion.jsonl",
                    "event_count": 3,
                }
                if evidence_name == "release_ops_execution"
                else None,
            ),
            operations_root / file_name,
        )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            generated_at="2026-04-15T10:04:00+00:00",
            engagement_id="release-artifact-builder-canonical",
            customer_name="AGI-Walker Customer",
            site_name="primary-site",
            change_ticket="CHG-RELEASE-ARTIFACT",
            window_id="window-release-artifact",
            window_start_at="2026-04-15T10:30:00+00:00",
            window_end_at="2026-04-15T12:30:00+00:00",
            delivery_root="test_env/release_delivery/release_artifact_builder",
            closure_archive_root="test_env/release_delivery/release_artifact_builder/closure_archive",
            exception_review_due_at="2026-05-15T00:00:00+00:00",
        ),
        operations_root / "extension_execution_instance.json",
    )
    write_extension_execution_schedule_artifact(
        build_extension_execution_schedule_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_schedule.json",
            instance_artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
        ),
        operations_root / "extension_execution_schedule.json",
    )
    write_extension_execution_actuals_artifact(
        build_extension_execution_actuals_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
            schedule_artifact_path="test_env/release_evidence/operations/extension_execution_schedule.json",
            generated_at="2026-04-15T10:05:00+00:00",
        ),
        operations_root / "extension_execution_actuals.json",
    )


def _seed_industrial_security_reports(project_root: Path) -> None:
    security_root = project_root / "test_env" / "release_evidence" / "security"
    security_root.mkdir(parents=True, exist_ok=True)
    for path, payload in {
        security_root / "sbom.json": {
            "schema_version": "1.0",
            "artifact_type": "sbom_artifact",
            "generated_at": "2026-04-15T10:05:00+00:00",
            "project_root": str(project_root),
            "project_name": "agi-walker",
            "project_version": "2026.04.15-rc",
            "dependency_sources": ["pyproject.toml"],
            "components": [
                {
                    "name": "fastapi",
                    "requirement": "fastapi>=0.110",
                    "source": "pyproject.toml",
                    "group": "project.dependencies",
                }
            ],
            "component_count": 1,
        },
        security_root / "python_vuln_scan_report.json": {
            "schema_version": "1.0",
            "artifact_type": "vulnerability_scan_report",
            "generated_at": "2026-04-15T10:06:00+00:00",
            "scan_name": "python_dependencies",
            "target": "pyproject.toml",
            "status": "passed",
            "summary": "Python dependency scan passed.",
            "command": "pip-audit --format json",
            "scanner": "pip-audit",
        },
        security_root / "container_vuln_scan_report.json": {
            "schema_version": "1.0",
            "artifact_type": "vulnerability_scan_report",
            "generated_at": "2026-04-15T10:07:00+00:00",
            "scan_name": "container_images",
            "target": "deployment/docker-compose.yml",
            "status": "passed",
            "summary": "Container vulnerability scan passed.",
            "command": "trivy image --format json",
            "scanner": "trivy",
        },
        security_root / "backup_restore_rehearsal_report.json": {
            "schema_version": "1.0",
            "artifact_type": "backup_restore_rehearsal_report",
            "generated_at": "2026-04-15T10:08:00+00:00",
            "project_root": str(project_root),
            "actor": "release-artifact-builder",
            "status": "passed",
            "summary": "Backup and restore rehearsal passed.",
            "release_manifest_path": None,
            "source_runtime_root": str(project_root / "runtime" / "source"),
            "source_config_root": str(project_root / "config" / "source"),
            "backup_snapshot_root": str(project_root / "backup"),
            "restored_runtime_root": str(project_root / "runtime" / "restored"),
            "restored_config_root": str(project_root / "config" / "restored"),
            "backup_items": [
                {
                    "name": "db",
                    "source_path": "runtime/db",
                    "backup_path": "backup/db",
                    "required": True,
                    "exists": True,
                }
            ],
            "restore_checks": [
                {
                    "name": "db",
                    "source_path": "runtime/db",
                    "restored_path": "restored/db",
                    "required": True,
                    "passed": True,
                }
            ],
            "missing_backup_items": 0,
            "failed_restore_checks": 0,
            "rpo_target": "24 hours",
            "rto_target": "4 hours",
            "rehearsal_duration_seconds": 1.25,
            "next_actions": [],
        },
    }.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


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
    _write_clean_checkout_smoke_report(tmp_path)
    _seed_structured_required_evidence(tmp_path)
    _seed_customer_delivery_docs(tmp_path)
    _seed_industrial_security_reports(tmp_path)
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
    assert "release_manifest_release_ops_execution=passed/3" in result.stdout
    assert "release_manifest_control_plane_events=3" in result.stdout
    assert "release_manifest_control_plane_surface=passed/3" in result.stdout
    assert "release_source_worktree_clean=" in result.stdout
    assert "customer_delivery_status=ready" in result.stdout
    assert "customer_delivery_phase_e_docs=4/4" in result.stdout
    assert "customer_delivery_release_ops_execution=passed/3" in result.stdout
    assert "industrial_delivery_status=ready" in result.stdout
    assert "industrial_delivery_release_ops_execution=passed/3" in result.stdout
    assert "industrial_delivery_evidence=3/3" in result.stdout
    assert "industrial_delivery_vuln_scan_status=passed" in result.stdout
    assert "extension_execution_evidence_status=ready" in result.stdout
    assert "extension_execution_instance_status=ready" in result.stdout
    assert "extension_execution_schedule_status=ready" in result.stdout
    assert "extension_execution_actuals_status=ready" in result.stdout
    assert output_path.exists()

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(payload) == []
    assert payload["build_id"] == "build-20260412-001"
    assert payload["release_policy"]["channel"] == "rc"
    assert payload["release_source"]["resolved_from_git"] is True
    assert payload["release_gate"]["blocked_evidence"] == 0
    assert payload["release_gate"]["blocked_optional_evidence"] == 0
    assert payload["release_ops_execution"]["status"] == "passed"
    assert payload["release_ops_execution"]["event_count"] == 3
    assert payload["control_plane_event_stream"]["event_count"] == 3
    assert payload["control_plane_surface"]["status"] == "passed"
    assert payload["control_plane_surface"]["event_count"] == 3
    assert payload["control_plane_surface"]["release_ops_execution"]["status"] == "passed"
    assert payload["customer_delivery_surface"]["status"] == "ready"
    assert payload["customer_delivery_surface"]["release_ops_execution"]["status"] == "passed"
    assert payload["customer_delivery_surface"]["release_ops_execution"]["event_count"] == 3
    assert payload["customer_delivery_surface"]["control_plane_event_stream"]["event_count"] == 3
    assert payload["industrial_delivery_gate"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["release_ops_execution"]["status"] == "passed"
    assert payload["industrial_delivery_gate"]["release_ops_execution"]["event_count"] == 3
    assert payload["industrial_delivery_gate"]["control_plane_event_stream"]["event_count"] == 3
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
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
    _write_clean_checkout_smoke_report(tmp_path)
    _seed_structured_required_evidence(tmp_path)
    _seed_customer_delivery_docs(tmp_path)
    _seed_industrial_security_reports(tmp_path)
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
    _write_clean_checkout_smoke_report(tmp_path)
    _seed_structured_required_evidence(tmp_path)
    _seed_customer_delivery_docs(tmp_path)
    _seed_industrial_security_reports(tmp_path)
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
    assert "release_manifest_release_ops_execution=passed/3" in result.stdout
    assert "release_manifest_control_plane_events=3" in result.stdout
    assert "release_manifest_control_plane_surface=passed/3" in result.stdout
    assert f"release_source_commit_sha={git_source['commit_sha']}" in result.stdout
    assert "release_source_worktree_clean=true" in result.stdout
    assert "industrial_delivery_status=ready" in result.stdout
    assert "customer_delivery_release_ops_execution=passed/3" in result.stdout
    assert "industrial_delivery_release_ops_execution=passed/3" in result.stdout
    assert "extension_execution_evidence_status=ready" in result.stdout
    assert "extension_execution_instance_status=ready" in result.stdout
    assert "extension_execution_schedule_status=ready" in result.stdout
    assert "extension_execution_actuals_status=ready" in result.stdout
    assert "extension_execution_actuals_status=ready" in result.stdout
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
    assert payload["release_ops_execution"]["status"] == "passed"
    assert payload["release_ops_execution"]["event_count"] == 3
    assert payload["control_plane_surface"]["status"] == "passed"
    assert payload["control_plane_surface"]["event_count"] == 3
    assert payload["customer_delivery_surface"]["release_ops_execution"]["status"] == "passed"
    assert payload["customer_delivery_surface"]["release_ops_execution"]["event_count"] == 3
    assert payload["industrial_delivery_gate"]["release_ops_execution"]["status"] == "passed"
    assert payload["industrial_delivery_gate"]["release_ops_execution"]["event_count"] == 3
    assert payload["industrial_delivery_gate"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3


def test_build_release_artifact_industrial_requires_delivery_gates(
    tmp_path: Path,
) -> None:
    git_source = _init_git_repo(tmp_path, tag="2026.04.12")
    output_path = tmp_path / "release_manifest.json"
    _write_clean_checkout_smoke_report(tmp_path)
    _seed_structured_required_evidence(tmp_path)
    _seed_customer_delivery_docs(tmp_path)
    _seed_industrial_security_reports(tmp_path)
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
            "industrial",
            "--build-id",
            "build-20260412-industrial",
            "--release-summary",
            "industrial 发布验证。",
            "--source-root",
            git_source["root"],
            "--approval-status",
            "approved",
            "--approved-by",
            "release-manager",
            "--approved-at",
            "2026-04-12T12:30:00+00:00",
            "--approval-notes",
            "industrial signoff",
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
    assert "customer_delivery_status=ready" in result.stdout
    assert "industrial_delivery_status=ready" in result.stdout
    assert "extension_execution_evidence_status=ready" in result.stdout
    assert "extension_execution_instance_status=ready" in result.stdout
    assert "extension_execution_schedule_status=ready" in result.stdout

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(payload) == []
    assert payload["channel"] == "industrial"
    assert payload["release_policy"]["requires_customer_delivery_surface"] is True
    assert payload["release_policy"]["requires_industrial_delivery_gate"] is True
    assert payload["release_gate"]["customer_delivery_required"] == 1
    assert payload["release_gate"]["customer_delivery_ready"] == 1
    assert payload["release_gate"]["industrial_delivery_required"] == 1
    assert payload["release_gate"]["industrial_delivery_ready"] == 1
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3


def test_build_extension_execution_evidence_script_writes_valid_reports(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    exception_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json"
    )
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-16T10:00:00+00:00",
            exceptions=[
                {
                    "id": "exception-001",
                    "scope": "container_images",
                    "component": "deployment-web-panel-distributed",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "current_version": "2026.04.16",
                    "vulnerability_ids": ["CVE-2026-0001"],
                    "severities": ["HIGH"],
                    "only_without_fix_version": True,
                    "justification": "waiting for upstream fix",
                    "ticket": "SEC-123",
                    "approved_by": "security-lead",
                    "approved_at": "2026-04-15T09:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                }
            ],
        ),
        exception_report_path,
    )
    operations_root = project_root / "test_env" / "release_evidence" / "operations"
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            generated_at="2026-04-16T10:00:00+00:00",
            engagement_id="customer-001",
            customer_name="Customer 001",
            site_name="primary-site",
            change_ticket="CHG-CUSTOMER-001",
            window_id="window-001",
            window_start_at="2026-04-16T10:00:00+00:00",
            window_end_at="2026-04-16T12:00:00+00:00",
            delivery_root="test_env/release_delivery/customer_001",
            closure_archive_root="test_env/release_delivery/customer_001/closure_archive",
            exception_review_due_at="2026-05-15T00:00:00+00:00",
        ),
        operations_root / "extension_execution_instance.json",
    )
    write_extension_execution_schedule_artifact(
        build_extension_execution_schedule_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_schedule.json",
            instance_artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            generated_at="2026-04-16T10:00:00+00:00",
        ),
        operations_root / "extension_execution_schedule.json",
    )
    write_extension_execution_actuals_artifact(
        build_extension_execution_actuals_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
            schedule_artifact_path="test_env/release_evidence/operations/extension_execution_schedule.json",
            generated_at="2026-04-16T10:00:00+00:00",
            external_bindings={
                "config_path": "deployment/customer_delivery.external_bindings.customer.json",
                "approval_identity": {
                    "binding_state": "confirmed",
                    "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
                    "source_type": "customer_ticket_registry",
                    "reference": "customer-001/window-001/customer_operator",
                    "confirmed_by": "release-manager",
                    "confirmed_at": "2026-04-16T12:10:00+00:00",
                    "confirmation_ticket": "CHG-CUSTOMER-001",
                },
                "archive_target": {
                    "binding_state": "confirmed",
                    "binding_type": "customer_sharepoint_archive",
                    "binding_reference_base": "sharepoint://customer-001/windows/window-001",
                    "confirmed_by": "release-manager",
                    "confirmed_at": "2026-04-16T12:10:00+00:00",
                    "confirmation_ticket": "CHG-CUSTOMER-001",
                },
                "due_trigger": {
                    "binding_state": "confirmed",
                    "binding_type": "customer_service_now_schedule",
                    "binding_reference_base": "servicenow://customer-001/windows/window-001",
                    "checked_at": "2026-04-16T12:09:00+00:00",
                    "confirmed_by": "release-manager",
                    "confirmed_at": "2026-04-16T12:10:00+00:00",
                    "confirmation_ticket": "CHG-CUSTOMER-001",
                },
            },
        ),
        operations_root / "extension_execution_actuals.json",
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_extension_execution_evidence.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(project_root),
            "--output-root",
            "test_env/release_evidence/operations",
            "--vulnerability-exception-report",
            str(exception_report_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "extension_execution_report_written[extension_on_call_rehearsal]=" in result.stdout
    assert (
        "extension_execution_report_written[extension_exception_review_schedule]="
        in result.stdout
    )
    assert "extension_execution_report_written[extension_escalation_closure]=" in result.stdout
    assert (
        "extension_execution_report_written[customer_external_bindings_confirmation]="
        in result.stdout
    )
    assert "extension_execution_evidence_reports=4/4" in result.stdout
    assert "extension_execution_evidence_status=ready" in result.stdout

    reports_root = project_root / "test_env" / "release_evidence" / "operations"
    reports_by_name = {
        "extension_on_call_rehearsal": reports_root
        / "extension_on_call_rehearsal_report.json",
        "extension_exception_review_schedule": reports_root
        / "extension_exception_review_schedule_report.json",
        "extension_escalation_closure": reports_root
        / "extension_escalation_closure_report.json",
        "customer_external_bindings_confirmation": reports_root
        / "customer_external_bindings_confirmation_report.json",
    }
    for name, path in reports_by_name.items():
        payload = json.loads(path.read_text(encoding="utf-8"))
        assert validate_release_evidence_report(payload) == []
        assert payload["evidence_name"] == name
        assert payload["status"] == "passed"

    manifest_output_path = project_root / "test_env" / "release" / "release_manifest.json"
    manifest_result = subprocess.run(
        [
            sys.executable,
            "tools/build_release_artifact.py",
            "--version",
            "2026.04.16-rc1",
            "--channel",
            "rc",
            "--build-id",
            "build-20260416-001",
            "--release-summary",
            "extension execution evidence smoke.",
            "--project-root",
            str(project_root),
            "--output",
            str(manifest_output_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert manifest_result.returncode == 0, manifest_result.stderr
    manifest_payload = json.loads(manifest_output_path.read_text(encoding="utf-8"))
    assert manifest_payload["extension_execution_evidence"]["status"] == "ready"
    assert manifest_payload["extension_execution_evidence"]["ready_reports"] == 4


def test_build_extension_execution_instance_script_writes_valid_artifact(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_extension_execution_instance.py",
            "--project-root",
            str(project_root),
            "--output",
            "test_env/release_evidence/operations/extension_execution_instance.json",
            "--engagement-id",
            "customer-001",
            "--customer-name",
            "Example Customer",
            "--site-name",
            "lagos-site",
            "--change-ticket",
            "CHG-001",
            "--window-id",
            "window-001",
            "--window-start-at",
            "2026-04-16T10:00:00+00:00",
            "--window-end-at",
            "2026-04-16T12:00:00+00:00",
            "--delivery-root",
            "test_env/release_delivery/customer_001",
            "--closure-archive-root",
            "test_env/release_delivery/customer_001/closure_archive",
            "--exception-review-due-at",
            "2026-05-15T00:00:00+00:00",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "extension_execution_instance_written=" in result.stdout
    assert "extension_execution_instance_status=ready" in result.stdout
    assert "extension_execution_instance_profiles=3/3" in result.stdout

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "extension_execution_instance"
    assert payload["status"] == "ready"
    assert payload["engagement_id"] == "customer-001"
    assert payload["exception_review_due_at"] == "2026-05-15T00:00:00+00:00"
    assert payload["ready_profiles"] == 3

    schedule_output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_schedule.json"
    )
    schedule_result = subprocess.run(
        [
            sys.executable,
            "tools/build_extension_execution_schedule.py",
            "--project-root",
            str(project_root),
            "--output",
            "test_env/release_evidence/operations/extension_execution_schedule.json",
            "--instance-artifact",
            "test_env/release_evidence/operations/extension_execution_instance.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert schedule_result.returncode == 0, schedule_result.stderr
    assert "extension_execution_schedule_written=" in schedule_result.stdout
    assert "extension_execution_schedule_status=ready" in schedule_result.stdout
    assert "extension_execution_schedule_profiles=3/3" in schedule_result.stdout

    schedule_payload = json.loads(schedule_output_path.read_text(encoding="utf-8"))
    assert schedule_payload["artifact_type"] == "extension_execution_schedule"
    assert schedule_payload["status"] == "ready"
    assert schedule_payload["window_trigger_at"] == "2026-04-16T10:00:00+00:00"
    assert schedule_payload["closure_archive_due_at"] == "2026-04-16T12:00:00+00:00"
    assert schedule_payload["ready_profiles"] == 3

    external_bindings_config_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.json"
    )
    external_bindings_config_path.parent.mkdir(parents=True, exist_ok=True)
    external_bindings_config_path.write_text(
        json.dumps(
            {
                "approval_identity": {
                    "binding_state": "confirmed",
                    "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
                    "source_type": "customer_ticket_registry",
                    "reference": "customer-001/window-001/customer_operator",
                    "system_name": "Customer Ticket Registry",
                    "portal_url": "https://tickets.example/customers/customer-001/window-001",
                    "confirmed_by": "release-manager",
                    "confirmed_at": "2026-04-16T12:10:00+00:00",
                    "confirmation_ticket": "CHG-CUSTOMER-001",
                },
                "archive_target": {
                    "binding_state": "confirmed",
                    "binding_type": "customer_sharepoint_archive",
                    "binding_reference_base": "sharepoint://customer-001/windows/window-001",
                    "system_name": "Customer SharePoint",
                    "portal_url": "https://sharepoint.example/customer-001/windows/window-001",
                    "confirmed_by": "release-manager",
                    "confirmed_at": "2026-04-16T12:10:00+00:00",
                    "confirmation_ticket": "CHG-CUSTOMER-001",
                },
                "due_trigger": {
                    "binding_state": "confirmed",
                    "binding_type": "customer_service_now_schedule",
                    "binding_reference_base": "servicenow://customer-001/windows/window-001",
                    "checked_at": "2026-04-16T12:09:00+00:00",
                    "system_name": "Customer ServiceNow",
                    "portal_url": "https://servicenow.example/customer-001/windows/window-001",
                    "confirmed_by": "release-manager",
                    "confirmed_at": "2026-04-16T12:10:00+00:00",
                    "confirmation_ticket": "CHG-CUSTOMER-001",
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    actuals_output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_actuals.json"
    )
    actuals_result = subprocess.run(
        [
            sys.executable,
            "tools/build_extension_execution_actuals.py",
            "--project-root",
            str(project_root),
            "--output",
            "test_env/release_evidence/operations/extension_execution_actuals.json",
            "--schedule-artifact",
            "test_env/release_evidence/operations/extension_execution_schedule.json",
            "--external-bindings-config",
            "deployment/customer_delivery.external_bindings.json",
            "--window-trigger-recorded-by",
            "delivery_lead",
            "--signoff-recorded-by",
            "customer_operator",
            "--residual-risk-reviewed-by",
            "delivery_lead",
            "--closure-archived-by",
            "rollback_owner",
            "--exception-review-due-at",
            "2026-05-20T00:00:00+00:00",
            "--closure-archive-due-at",
            "2026-04-16T12:15:00+00:00",
            "--due-trigger-checked-at",
            "2026-04-16T12:10:00+00:00",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert actuals_result.returncode == 0, actuals_result.stderr
    assert "extension_execution_actuals_written=" in actuals_result.stdout
    assert "extension_execution_actuals_status=ready" in actuals_result.stdout
    assert "extension_execution_actuals_profiles=3/3" in actuals_result.stdout
    assert "extension_execution_actuals_external_bindings_status=ready" in actuals_result.stdout
    assert (
        "extension_execution_actuals_external_bindings_follow_up_required=false"
        in actuals_result.stdout
    )
    assert (
        "extension_execution_actuals_window_trigger_recorded_at=2026-04-16T10:00:00+00:00"
        in actuals_result.stdout
    )
    assert "extension_execution_actuals_signoff_recorded_by=customer_operator" in actuals_result.stdout
    assert (
        "extension_execution_actuals_approval_identity_source_path="
        "test_env/release_delivery/customer_001/approval_identity_source.json"
        in actuals_result.stdout
    )
    assert (
        "extension_execution_actuals_archive_target_binding_type="
        "customer_sharepoint_archive"
        in actuals_result.stdout
    )
    assert (
        "extension_execution_actuals_exception_review_due_at=2026-05-20T00:00:00+00:00"
        in actuals_result.stdout
    )
    assert (
        "extension_execution_actuals_due_trigger_binding_type="
        "customer_service_now_schedule"
        in actuals_result.stdout
    )
    assert (
        "extension_execution_actuals_due_trigger_checked_at=2026-04-16T12:10:00+00:00"
        in actuals_result.stdout
    )
    assert (
        "extension_execution_actuals_external_bindings_config="
        "deployment/customer_delivery.external_bindings.json"
        in actuals_result.stdout
    )

    actuals_payload = json.loads(actuals_output_path.read_text(encoding="utf-8"))
    assert actuals_payload["artifact_type"] == "extension_execution_actuals"
    assert actuals_payload["status"] == "ready"
    assert actuals_payload["window_trigger_recorded_at"] == "2026-04-16T10:00:00+00:00"
    assert actuals_payload["signoff_recorded_by"] == "customer_operator"
    assert actuals_payload["approval_identity_source_type"] == "customer_ticket_registry"
    assert (
        actuals_payload["approval_identity_reference"]
        == "customer-001/window-001/customer_operator"
    )
    assert actuals_payload["archive_target_binding_type"] == "customer_sharepoint_archive"
    assert (
        actuals_payload["archive_target_binding_reference_base"]
        == "sharepoint://customer-001/windows/window-001"
    )
    assert actuals_payload["due_trigger_binding_type"] == "customer_service_now_schedule"
    assert (
        actuals_payload["due_trigger_binding_reference_base"]
        == "servicenow://customer-001/windows/window-001"
    )
    assert actuals_payload["due_trigger_checked_at"] == "2026-04-16T12:10:00+00:00"
    assert actuals_payload["exception_review_due_at"] == "2026-05-20T00:00:00+00:00"
    assert actuals_payload["closure_archive_due_at"] == "2026-04-16T12:15:00+00:00"
    assert actuals_payload["ready_profiles"] == 3
    assert actuals_payload["external_bindings_status"] == "ready"
    assert actuals_payload["external_bindings_follow_up_required"] is False
    assert actuals_payload["external_bindings_declared_count"] == 3
    assert actuals_payload["external_bindings_ready_count"] == 3
    assert actuals_payload["external_bindings_placeholder_count"] == 0
    assert actuals_payload["external_bindings_confirmed_count"] == 3
    assert actuals_payload["external_bindings_missing_sections"] == []
    assert actuals_payload["external_bindings_placeholder_sections"] == []
    assert actuals_payload["external_bindings_unconfirmed_sections"] == []
    assert actuals_payload["external_bindings_confirmation_missing_sections"] == []
    assert actuals_payload["external_bindings_confirmed_by"] == ["release-manager"]
    assert actuals_payload["external_bindings_confirmation_tickets"] == ["CHG-CUSTOMER-001"]
    assert actuals_payload["external_bindings_last_confirmed_at"] == "2026-04-16T12:10:00+00:00"
    assert actuals_payload["external_bindings"]["config_path"] == (
        "deployment/customer_delivery.external_bindings.json"
    )
    assert (
        actuals_payload["external_bindings"]["approval_identity"]["source_path"]
        == "test_env/release_delivery/customer_001/approval_identity_source.json"
    )
    assert (
        actuals_payload["external_bindings"]["approval_identity"]["reference"]
        == "customer-001/window-001/customer_operator"
    )
    assert (
        actuals_payload["external_bindings"]["approval_identity"]["system_name"]
        == "Customer Ticket Registry"
    )
    assert (
        actuals_payload["external_bindings"]["archive_target"]["portal_url"]
        == "https://sharepoint.example/customer-001/windows/window-001"
    )
    assert (
        actuals_payload["external_bindings"]["due_trigger"]["portal_url"]
        == "https://servicenow.example/customer-001/windows/window-001"
    )
    distributed_actuals = next(
        item
        for item in actuals_payload["profiles"]
        if item["id"] == "distributed_profile"
    )
    assert distributed_actuals["signoff_due_at"] == "2026-04-16T12:00:00+00:00"
    assert distributed_actuals["exception_review_due_at"] == "2026-05-20T00:00:00+00:00"
    assert distributed_actuals["closure_archive_due_at"] == "2026-04-16T12:15:00+00:00"
    assert distributed_actuals["approval_identity_source_path"].endswith(
        "approval_identity_source.json"
    )
    assert distributed_actuals["window_trigger_record_path"].endswith("window_trigger.json")
    assert distributed_actuals["signoff_record_path"].endswith("signoff.json")
    assert distributed_actuals["exception_review_record_path"].endswith(
        "exception_review.json"
    )
    assert distributed_actuals["residual_risk_review_record_path"].endswith(
        "residual_risk_review.json"
    )
    assert distributed_actuals["archive_target_path"].endswith("archive_target.json")
    assert (
        distributed_actuals["archive_target_binding_reference"]
        == "sharepoint://customer-001/windows/window-001/distributed_profile"
    )
    assert distributed_actuals["due_trigger_check_path"].endswith("due_trigger_check.json")
    assert (
        distributed_actuals["due_trigger_binding_reference"]
        == "servicenow://customer-001/windows/window-001/distributed_profile"
    )
    assert distributed_actuals["closure_index_path"].endswith("index.json")
    assert distributed_actuals["closure_manifest_path"].endswith("closure_manifest.json")
    assert distributed_actuals["signoff_owner_role"] == "delivery_lead"
    assert distributed_actuals["exception_review_owner_role"] == "delivery_lead"
    assert distributed_actuals["closure_archive_owner_role"] == "rollback_owner"
    for relative_path in [
        actuals_payload["approval_identity_source_path"],
        distributed_actuals["window_trigger_record_path"],
        distributed_actuals["signoff_record_path"],
        distributed_actuals["exception_review_record_path"],
        distributed_actuals["residual_risk_review_record_path"],
        distributed_actuals["archive_target_path"],
        distributed_actuals["due_trigger_check_path"],
        distributed_actuals["closure_index_path"],
        distributed_actuals["closure_manifest_path"],
    ]:
        assert (project_root / relative_path).exists()
    archive_target_payload = json.loads(
        (project_root / distributed_actuals["archive_target_path"]).read_text(
            encoding="utf-8"
        )
    )
    approval_identity_payload = json.loads(
        (project_root / actuals_payload["approval_identity_source_path"]).read_text(
            encoding="utf-8"
        )
    )
    assert approval_identity_payload["binding_config_path"] == (
        "deployment/customer_delivery.external_bindings.json"
    )
    assert approval_identity_payload["external_binding"]["system_name"] == (
        "Customer Ticket Registry"
    )
    assert approval_identity_payload["external_binding_confirmation"]["confirmed_by"] == (
        "release-manager"
    )
    assert archive_target_payload["binding_type"] == "customer_sharepoint_archive"
    assert (
        archive_target_payload["binding_reference"]
        == "sharepoint://customer-001/windows/window-001/distributed_profile"
    )
    assert archive_target_payload["binding_config_path"] == (
        "deployment/customer_delivery.external_bindings.json"
    )
    assert archive_target_payload["external_binding"]["system_name"] == (
        "Customer SharePoint"
    )
    assert archive_target_payload["external_binding_confirmation"]["confirmation_ticket"] == (
        "CHG-CUSTOMER-001"
    )
    due_trigger_payload = json.loads(
        (project_root / distributed_actuals["due_trigger_check_path"]).read_text(
            encoding="utf-8"
        )
    )
    assert due_trigger_payload["binding_type"] == "customer_service_now_schedule"
    assert (
        due_trigger_payload["binding_reference"]
        == "servicenow://customer-001/windows/window-001/distributed_profile"
    )
    assert due_trigger_payload["binding_config_path"] == (
        "deployment/customer_delivery.external_bindings.json"
    )
    assert due_trigger_payload["external_binding"]["system_name"] == (
        "Customer ServiceNow"
    )
    assert due_trigger_payload["external_binding_confirmation"]["binding_state"] == (
        "confirmed"
    )

    manifest_output_path = project_root / "test_env" / "release" / "release_manifest.json"
    manifest_result = subprocess.run(
        [
            sys.executable,
            "tools/build_release_artifact.py",
            "--version",
            "2026.04.16-rc1",
            "--channel",
            "rc",
            "--build-id",
            "build-20260416-002",
            "--release-summary",
            "extension execution instance smoke.",
            "--project-root",
            str(project_root),
            "--output",
            str(manifest_output_path),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert manifest_result.returncode == 0, manifest_result.stderr
    manifest_payload = json.loads(manifest_output_path.read_text(encoding="utf-8"))
    assert manifest_payload["extension_execution_instance"]["status"] == "ready"
    assert manifest_payload["extension_execution_instance"]["engagement_id"] == "customer-001"
    assert manifest_payload["extension_execution_instance"]["ready_profiles"] == 3
    assert manifest_payload["extension_execution_schedule"]["status"] == "ready"
    assert manifest_payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert manifest_payload["extension_execution_actuals"]["status"] == "ready"
    assert manifest_payload["extension_execution_actuals"]["ready_profiles"] == 3
    assert manifest_payload["extension_execution_actuals"]["signoff_recorded_by"] == "customer_operator"
    assert (
        manifest_payload["extension_execution_actuals"]["approval_identity_reference"]
        == "customer-001/window-001/customer_operator"
    )
    assert (
        manifest_payload["extension_execution_actuals"]["archive_target_binding_type"]
        == "customer_sharepoint_archive"
    )
    assert (
        manifest_payload["extension_execution_actuals"]["due_trigger_binding_type"]
        == "customer_service_now_schedule"
    )
    assert (
        manifest_payload["extension_execution_actuals"]["exception_review_due_at"]
        == "2026-05-20T00:00:00+00:00"
    )
    assert (
        manifest_payload["extension_execution_actuals"]["external_bindings"][
            "config_path"
        ]
        == "deployment/customer_delivery.external_bindings.json"
    )


def test_build_extension_execution_actuals_rejects_invalid_external_bindings_config(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    config_path = project_root / "deployment" / "customer_delivery.external_bindings.json"
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(
        json.dumps({"approval_identity": "not-an-object"}, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_extension_execution_actuals.py",
            "--project-root",
            str(project_root),
            "--output",
            "test_env/release_evidence/operations/extension_execution_actuals.json",
            "--schedule-artifact",
            "test_env/release_evidence/operations/extension_execution_schedule.json",
            "--external-bindings-config",
            "deployment/customer_delivery.external_bindings.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode != 0
    assert (
        "external bindings config section 'approval_identity' must be an object"
        in result.stderr
    )
