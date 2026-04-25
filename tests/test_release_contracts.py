import json
import subprocess
from pathlib import Path

import pytest

from agi_walker.core.api.release_contracts import (
    RELEASE_CONTRACT_VERSION,
    apply_release_test_evidence_to_capability_matrix,
    build_customer_delivery_surface,
    build_extension_execution_actuals_artifact,
    build_extension_execution_instance_artifact,
    build_extension_execution_schedule_artifact,
    build_industrial_delivery_gate,
    build_release_evidence_report,
    build_release_manifest_artifact,
    default_release_test_evidence,
    hydrate_release_test_evidence,
    read_release_control_plane_surface,
    validate_customer_delivery_surface,
    validate_extension_execution_actuals,
    validate_industrial_delivery_gate,
    validate_release_manifest_artifact,
    write_extension_execution_actuals_artifact,
    write_extension_execution_instance_artifact,
    write_extension_execution_schedule_artifact,
    write_release_evidence_report,
    write_release_manifest_artifact,
)
from agi_walker.core.api.security_posture_contracts import (
    build_security_posture_report,
    build_vulnerability_exception_report,
    build_vulnerability_remediation_report,
    build_vulnerability_scan_report,
    write_security_posture_report,
    write_vulnerability_exception_report,
    write_vulnerability_remediation_report,
    write_vulnerability_scan_report,
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


def _write_clean_checkout_smoke_report(
    project_root: Path,
    *,
    status: str = "passed",
    runs: int = 2,
    failure_reason: str | None = None,
) -> Path:
    report_path = (
        project_root / "test_env" / "release_evidence" / "clean_checkout_smoke_report.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    run_reports = []
    for run_index in range(1, runs + 1):
        run_reports.append(
            {
                "run_index": run_index,
                "status": "passed" if status == "passed" else "blocked",
                "command": [
                    "python",
                    "tests/run_smoke_tests.py",
                    "--output-root",
                    f"run_{run_index:02d}",
                ],
                "run_output_root": f"run_{run_index:02d}",
                "exit_code": 0 if status == "passed" else 1,
                "stdout_path": f"logs/run_{run_index:02d}.stdout.txt",
                "stderr_path": f"logs/run_{run_index:02d}.stderr.txt",
                "worktree_clean": status == "passed",
                "dirty_paths": [] if status == "passed" else ["M README.md"],
            }
        )
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "clean_checkout_smoke_report",
                "status": status,
                "generated_at": "2026-04-13T10:18:29+00:00",
                "source_root": str(project_root),
                "checkout_root": str(project_root / "test_env" / "clean_checkout_smoke" / "checkout"),
                "output_root": str(project_root / "test_env" / "clean_checkout_smoke"),
                "version": "2026.04.12",
                "tag": "2026.04.12",
                "runs": runs,
                "command_template": [
                    "python",
                    "tests/run_smoke_tests.py",
                    "--output-root",
                    "{run_output_root}",
                ],
                "checkout_commit_sha": "clean-checkout-commit",
                "seeded_evidence_paths": [
                    "test_env/release_evidence/non_live_gate_report.json"
                ],
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
                        "status": "pass" if status == "passed" else "fail",
                        "detail": (
                            f"{runs} sequential smoke run(s) completed with empty git status after each run"
                            if status == "passed"
                            else "smoke run dirtied the checkout"
                        ),
                    },
                ],
                "run_reports": run_reports,
                "failure_reason": failure_reason,
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
            source_commit_sha="release-contracts-commit",
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
            source_commit_sha="release-contracts-commit",
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
                "confirmation_tickets": ["CHG-RELEASE-CONTRACTS"],
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
                source_commit_sha="release-contracts-commit",
                control_plane_session={
                    "engagement_id": "release-contracts-session",
                    "window_id": "window-release-contracts",
                    "change_ticket": "CHG-RELEASE-CONTRACTS",
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
            engagement_id="release-contracts-canonical",
            customer_name="AGI-Walker Customer",
            site_name="primary-site",
            change_ticket="CHG-RELEASE-CONTRACTS",
            window_id="window-release-contracts",
            window_start_at="2026-04-15T10:30:00+00:00",
            window_end_at="2026-04-15T12:30:00+00:00",
            delivery_root="test_env/release_delivery/release_contracts",
            closure_archive_root="test_env/release_delivery/release_contracts/closure_archive",
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
    (security_root / "sbom.json").write_text(
        json.dumps(
            {
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
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    for path, payload in {
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
            "actor": "release-contracts-test",
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
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )


def _seed_exception_covered_industrial_security_reports(project_root: Path) -> None:
    _seed_industrial_security_reports(project_root)
    security_root = project_root / "test_env" / "release_evidence" / "security"
    python_report_path = security_root / "python_vuln_scan_report.json"
    container_report_path = security_root / "container_vuln_scan_report.json"
    backup_restore_path = security_root / "backup_restore_rehearsal_report.json"
    sbom_path = security_root / "sbom.json"

    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="python_dependencies",
            target="pyproject.toml",
            status="passed",
            summary="python clean",
            command="pip-audit --format json",
            scanner="pip-audit",
            finding_count=0,
            affected_component_count=0,
        ),
        python_report_path,
    )
    container_raw_root = security_root / "container_vuln_scan_report_raw"
    container_raw_root.mkdir(parents=True, exist_ok=True)
    (container_raw_root / "deployment-web-panel-distributed.json").write_text(
        json.dumps(
            {
                "ArtifactName": "deployment-web-panel-distributed",
                "Results": [
                    {
                        "Target": "debian:stable",
                        "Vulnerabilities": [
                            {
                                "PkgName": "libsystemd0",
                                "InstalledVersion": "257.8-1",
                                "FixedVersion": "",
                                "VulnerabilityID": "CVE-1",
                                "Severity": "HIGH",
                            }
                        ],
                    }
                ],
            },
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="container_images",
            target="deployment/docker-compose.yml",
            status="blocked",
            summary="trivy reported 1 finding(s) across 1 affected component(s).",
            command="trivy image --format json",
            scanner="trivy",
            report_format="trivy-json",
            raw_report_path=str(container_raw_root.relative_to(project_root)),
            finding_count=1,
            affected_component_count=1,
        ),
        container_report_path,
    )

    exception_report_path = security_root / "vulnerability_exception_report.json"
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-15T12:00:00+00:00",
            exceptions=[
                {
                    "id": "container-libsystemd0-no-fix",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "vulnerability_ids": ["CVE-1"],
                    "justification": "Approved temporary exception for residual deployment image risk.",
                    "approved_by": "security-reviewer",
                    "approved_at": "2026-04-15T11:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                }
            ],
        ),
        exception_report_path,
    )

    remediation_path = security_root / "vulnerability_remediation_report.json"
    write_vulnerability_remediation_report(
        build_vulnerability_remediation_report(
            project_root=project_root,
            python_vuln_report_path=python_report_path,
            container_vuln_report_path=container_report_path,
            vulnerability_exception_report_path=exception_report_path,
        ),
        remediation_path,
    )

    posture_path = security_root / "security_posture_report.json"
    write_security_posture_report(
        build_security_posture_report(
            project_root=project_root,
            sbom_path=sbom_path,
            python_vuln_report_path=python_report_path,
            container_vuln_report_path=container_report_path,
            backup_restore_rehearsal_report_path=backup_restore_path,
            vulnerability_remediation_report_path=remediation_path,
            vulnerability_exception_report_path=exception_report_path,
        ),
        posture_path,
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
    if project_root is not None:
        _write_clean_checkout_smoke_report(project_root)
        _seed_structured_required_evidence(project_root)
        _seed_customer_delivery_docs(project_root)
        _seed_industrial_security_reports(project_root)
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
        "requires_customer_delivery_surface": False,
        "requires_industrial_delivery_gate": False,
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
    assert payload["customer_delivery_surface"]["status"] == "ready"
    assert payload["customer_delivery_surface"]["phase_e_documents_ready"] == 4
    assert payload["customer_delivery_surface"]["release_ops_execution"]["status"] == "passed"
    assert payload["customer_delivery_surface"]["release_ops_execution"]["event_count"] == 3
    assert payload["customer_delivery_surface"]["control_plane_event_stream"]["event_count"] == 3
    assert validate_customer_delivery_surface(payload["customer_delivery_surface"]) == []
    assert payload["industrial_delivery_gate"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["evidence_attested"] is True
    assert payload["industrial_delivery_gate"]["vuln_scan_status"] == "passed"
    assert payload["industrial_delivery_gate"]["release_ops_execution"]["status"] == "passed"
    assert payload["industrial_delivery_gate"]["release_ops_execution"]["event_count"] == 3
    assert payload["industrial_delivery_gate"]["control_plane_event_stream"]["event_count"] == 3
    assert validate_industrial_delivery_gate(payload["industrial_delivery_gate"]) == []
    assert payload["release_ops_execution"]["status"] == "passed"
    assert payload["release_ops_execution"]["event_count"] == 3
    assert payload["control_plane_event_stream"]["event_count"] == 3
    assert payload["control_plane_surface"]["status"] == "passed"
    assert payload["control_plane_surface"]["event_count"] == 3
    assert payload["control_plane_surface"]["release_ops_execution"]["status"] == "passed"
    assert payload["control_plane_surface"]["control_plane_event_stream"]["event_count"] == 3
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_evidence"]["actionable_profiles"] == 3
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert (
        payload["extension_execution_instance"]["exception_review_due_at"]
        == "2026-05-15T00:00:00+00:00"
    )
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert (
        payload["extension_execution_schedule"]["closure_archive_due_at"]
        == "2026-04-15T12:30:00+00:00"
    )
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["window_triggers_recorded"] == 3
    assert (
        payload["extension_execution_actuals"]["exception_review_due_at"]
        == "2026-05-15T00:00:00+00:00"
    )
    assert (
        payload["extension_execution_actuals"]["closure_archive_due_at"]
        == "2026-04-15T12:30:00+00:00"
    )
    assert payload["extension_execution_actuals"]["approval_identity_source_type"] == (
        "customer_signoff_registry"
    )
    assert payload["extension_execution_actuals"]["approval_identity_source_path"].endswith(
        "approval_identity_source.json"
    )
    assert payload["extension_execution_actuals"]["archive_target_binding_type"] == (
        "customer_archive_destination"
    )
    assert (
        payload["extension_execution_actuals"]["archive_target_binding_reference_base"]
        == "archive://release-contracts-canonical/window-release-contracts"
    )
    assert payload["extension_execution_actuals"]["due_trigger_binding_type"] == (
        "customer_due_trigger_schedule"
    )
    assert (
        payload["extension_execution_actuals"]["due_trigger_binding_reference_base"]
        == "schedule://release-contracts-canonical/window-release-contracts"
    )
    assert (
        payload["extension_execution_actuals"]["due_trigger_checked_at"]
        == "2026-04-15T12:30:00+00:00"
    )
    assert payload["extension_execution_actuals"]["signoff_recorded_by"] == "delivery_lead"
    assert (
        payload["extension_execution_actuals"]["closure_archived_by"]
        == "rollback_owner"
    )
    distributed_actuals = next(
        item
        for item in payload["extension_execution_actuals"]["profiles"]
        if item["id"] == "distributed_profile"
    )
    assert distributed_actuals["exception_review_record_path"].endswith(
        "exception_review.json"
    )
    assert distributed_actuals["approval_identity_source_path"].endswith(
        "approval_identity_source.json"
    )
    assert distributed_actuals["archive_target_path"].endswith("archive_target.json")
    assert (
        distributed_actuals["archive_target_binding_reference"]
        == "archive://release-contracts-canonical/window-release-contracts/distributed_profile"
    )
    assert distributed_actuals["due_trigger_check_path"].endswith("due_trigger_check.json")
    assert (
        distributed_actuals["due_trigger_binding_reference"]
        == "schedule://release-contracts-canonical/window-release-contracts/distributed_profile"
    )
    assert distributed_actuals["closure_index_path"].endswith("index.json")
    assert distributed_actuals["signoff_owner_role"] == "delivery_lead"
    assert distributed_actuals["closure_archive_owner_role"] == "rollback_owner"
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
        "customer_delivery_required": 0,
        "customer_delivery_ready": 1,
        "industrial_delivery_required": 0,
        "industrial_delivery_ready": 1,
    }
    assert validate_release_manifest_artifact(payload) == []


def test_read_release_control_plane_surface_prefers_manifest_surface(
    tmp_path: Path,
) -> None:
    payload = _valid_release_manifest(project_root=tmp_path)
    manifest_path = tmp_path / "test_env" / "release" / "release_manifest.json"
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    write_release_manifest_artifact(payload, manifest_path)

    surface = read_release_control_plane_surface(
        project_root=tmp_path,
        manifest_path=manifest_path,
    )

    assert surface["source"] == "release_manifest"
    assert surface["manifest_path"] == str(manifest_path)
    assert surface["control_plane_surface"]["status"] == "passed"
    assert surface["control_plane_surface"]["event_count"] == 3
    assert (
        surface["control_plane_surface"]["release_ops_execution"]["status"] == "passed"
    )
    assert (
        surface["control_plane_surface"]["control_plane_event_stream"]["event_count"]
        == 3
    )

    output_path = write_release_manifest_artifact(
        payload, tmp_path / "release_manifest.json"
    )
    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(saved) == []
    assert saved["version"] == "2026.04.12-rc1"


def test_extension_execution_actuals_flags_placeholder_external_bindings(
    tmp_path: Path,
) -> None:
    payload = build_extension_execution_actuals_artifact(
        project_root=tmp_path,
        artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
        generated_at="2026-04-17T16:03:49+00:00",
        window_trigger_recorded_by="delivery_lead",
        signoff_recorded_by="customer_operator",
        residual_risk_reviewed_by="delivery_lead",
        closure_archived_by="rollback_owner",
        external_bindings={
            "config_path": "deployment/customer_delivery.external_bindings.json",
            "approval_identity": {
                "source_type": "customer_signoff_registry",
                "system_name": "Customer Signoff Registry",
                "integration_notes": "Replace with the customer-owned approval registry before industrial promotion.",
            },
            "archive_target": {
                "binding_type": "customer_archive_destination",
                "system_name": "Customer Archive Destination",
                "integration_notes": "Replace with the customer-owned archive destination before industrial promotion.",
            },
            "due_trigger": {
                "binding_type": "customer_due_trigger_schedule",
                "system_name": "Customer Due Trigger Schedule",
                "integration_notes": "Replace with the customer-owned due-trigger scheduler before industrial promotion.",
            },
        },
    )

    assert payload["status"] == "blocked"
    assert payload["external_bindings_status"] == "placeholder"
    assert payload["external_bindings_follow_up_required"] is True
    assert payload["external_bindings_declared_count"] == 3
    assert payload["external_bindings_ready_count"] == 0
    assert payload["external_bindings_placeholder_count"] == 3
    assert payload["external_bindings_confirmed_count"] == 0
    assert payload["external_bindings_missing_sections"] == []
    assert payload["external_bindings_placeholder_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert payload["external_bindings_draft_sections"] == []
    assert payload["external_bindings_confirmed_sections"] == []
    assert payload["external_bindings_confirmation_missing_sections"] == []
    assert payload["external_bindings_confirmed_by"] == []
    assert payload["external_bindings_confirmation_tickets"] == []
    assert payload["external_bindings_last_confirmed_at"] is None
    assert "Replace placeholder sections" in payload["external_bindings_summary"]


def test_extension_execution_actuals_tracks_draft_external_bindings_sections(
    tmp_path: Path,
) -> None:
    payload = build_extension_execution_actuals_artifact(
        project_root=tmp_path,
        artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
        generated_at="2026-04-17T16:03:49+00:00",
        window_trigger_recorded_by="delivery_lead",
        signoff_recorded_by="customer_operator",
        residual_risk_reviewed_by="delivery_lead",
        closure_archived_by="rollback_owner",
        external_bindings={
            "config_path": "deployment/customer_delivery.external_bindings.customer.json",
            "approval_identity": {
                "binding_state": "draft",
                "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
                "source_type": "customer_ticket_registry",
                "reference": "customer-001/window-001/customer_operator",
            },
            "archive_target": {
                "binding_state": "draft",
                "binding_type": "customer_archive_destination",
                "binding_reference_base": "archive://customer-001/windows/window-001",
            },
            "due_trigger": {
                "binding_state": "draft",
                "binding_type": "customer_due_trigger_schedule",
                "binding_reference_base": "schedule://customer-001/windows/window-001",
            },
        },
    )

    assert payload["external_bindings_status"] == "placeholder"
    assert payload["external_bindings_follow_up_required"] is True
    assert payload["external_bindings_confirmed_count"] == 0
    assert payload["external_bindings_draft_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert payload["external_bindings_confirmed_sections"] == []
    assert payload["external_bindings_confirmation_missing_sections"] == []
    assert "Draft sections" in payload["external_bindings_summary"]


def test_extension_execution_actuals_tracks_unconfirmed_external_bindings_sections(
    tmp_path: Path,
) -> None:
    payload = build_extension_execution_actuals_artifact(
        project_root=tmp_path,
        artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
        generated_at="2026-04-17T16:03:49+00:00",
        window_trigger_recorded_by="delivery_lead",
        signoff_recorded_by="customer_operator",
        residual_risk_reviewed_by="delivery_lead",
        closure_archived_by="rollback_owner",
        external_bindings={
            "config_path": "deployment/customer_delivery.external_bindings.customer.json",
            "approval_identity": {
                "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
                "source_type": "customer_ticket_registry",
                "reference": "customer-001/window-001/customer_operator",
            },
            "archive_target": {
                "binding_type": "customer_archive_destination",
                "binding_reference_base": "archive://customer-001/windows/window-001",
            },
            "due_trigger": {
                "binding_type": "customer_due_trigger_schedule",
                "binding_reference_base": "schedule://customer-001/windows/window-001",
            },
        },
    )

    assert payload["external_bindings_status"] == "placeholder"
    assert payload["external_bindings_follow_up_required"] is True
    assert payload["external_bindings_ready_count"] == 0
    assert payload["external_bindings_placeholder_sections"] == []
    assert payload["external_bindings_unconfirmed_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert "Unconfirmed sections" in payload["external_bindings_summary"]


def test_extension_execution_actuals_tracks_confirmed_external_bindings_audit_metadata(
    tmp_path: Path,
) -> None:
    payload = build_extension_execution_actuals_artifact(
        project_root=tmp_path,
        artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
        generated_at="2026-04-17T16:03:49+00:00",
        window_trigger_recorded_by="delivery_lead",
        signoff_recorded_by="customer_operator",
        residual_risk_reviewed_by="delivery_lead",
        closure_archived_by="rollback_owner",
        external_bindings={
            "config_path": "deployment/customer_delivery.external_bindings.customer.json",
            "approval_identity": {
                "binding_state": "confirmed",
                "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
                "source_type": "customer_ticket_registry",
                "reference": "customer-001/window-001/customer_operator",
                "confirmed_by": "release-manager",
                "confirmed_at": "2026-04-17T15:59:00+00:00",
                "confirmation_ticket": "CHG-CUSTOMER-001",
            },
            "archive_target": {
                "binding_state": "confirmed",
                "binding_type": "customer_archive_destination",
                "binding_reference_base": "archive://customer-001/windows/window-001",
                "confirmed_by": "release-manager",
                "confirmed_at": "2026-04-17T16:00:00+00:00",
                "confirmation_ticket": "CHG-CUSTOMER-001",
            },
            "due_trigger": {
                "binding_state": "confirmed",
                "binding_type": "customer_due_trigger_schedule",
                "binding_reference_base": "schedule://customer-001/windows/window-001",
                "confirmed_by": "release-manager",
                "confirmed_at": "2026-04-17T16:01:00+00:00",
                "confirmation_ticket": "CHG-CUSTOMER-001",
            },
        },
    )

    assert payload["external_bindings_status"] == "ready"
    assert payload["external_bindings_follow_up_required"] is False
    assert payload["external_bindings_confirmed_count"] == 3
    assert payload["external_bindings_confirmed_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert payload["external_bindings_confirmation_missing_sections"] == []
    assert payload["external_bindings_confirmed_by"] == ["release-manager"]
    assert payload["external_bindings_confirmation_tickets"] == ["CHG-CUSTOMER-001"]
    assert payload["external_bindings_last_confirmed_at"] == "2026-04-17T16:01:00+00:00"
    assert "Confirmed sections: approval_identity, archive_target, due_trigger." in payload[
        "external_bindings_summary"
    ]
    assert "Confirmation tickets: CHG-CUSTOMER-001." in payload["external_bindings_summary"]


def test_validate_extension_execution_actuals_rejects_confirmed_bindings_without_confirmation_metadata(
    tmp_path: Path,
) -> None:
    payload = build_extension_execution_actuals_artifact(
        project_root=tmp_path,
        artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
        generated_at="2026-04-17T16:03:49+00:00",
        window_trigger_recorded_by="delivery_lead",
        signoff_recorded_by="customer_operator",
        residual_risk_reviewed_by="delivery_lead",
        closure_archived_by="rollback_owner",
        external_bindings={
            "config_path": "deployment/customer_delivery.external_bindings.customer.json",
            "approval_identity": {
                "binding_state": "confirmed",
                "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
                "source_type": "customer_ticket_registry",
                "reference": "customer-001/window-001/customer_operator",
            }
        },
    )

    errors = validate_extension_execution_actuals(payload)
    assert (
        "extension_execution_actuals.external_bindings.approval_identity.confirmed_by "
        "must be a non-empty string when binding_state is 'confirmed'"
    ) in errors
    assert (
        "extension_execution_actuals.external_bindings.approval_identity.confirmed_at "
        "must be a non-empty string when binding_state is 'confirmed'"
    ) in errors
    assert (
        "extension_execution_actuals.external_bindings.approval_identity.confirmation_ticket "
        "must be a non-empty string when binding_state is 'confirmed'"
    ) in errors


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
            "customer_delivery_surface": {"status": "green"},
            "industrial_delivery_gate": {"status": "green"},
        }
    )

    assert "schema_version must be '1.0', got 'bad'" in errors
    assert "channel must be one of ['dev', 'industrial', 'rc', 'stable']" in errors
    assert "release_gate_status must be one of ['blocked', 'ready', 'ready_with_limitations']" in errors
    assert "known_limitations[1] must be a non-empty string" in errors
    assert "customer_delivery_surface.status must be one of ['blocked', 'ready']" in errors
    assert "industrial_delivery_gate.status must be one of ['blocked', 'ready']" in errors


def test_write_release_manifest_rejects_invalid_payload(tmp_path):
    with pytest.raises(ValueError, match="missing required fields"):
        write_release_manifest_artifact(
            {"artifact_type": "release_manifest"}, tmp_path / "bad.json"
        )


def test_customer_delivery_surface_tracks_phase_e_docs(tmp_path: Path) -> None:
    _seed_customer_delivery_docs(tmp_path)

    payload = build_customer_delivery_surface(project_root=tmp_path)
    distributed_profile = next(
        item
        for item in payload["extension_support_surface"]["profiles"]
        if item["id"] == "distributed_profile"
    )
    ros2_profile = next(
        item
        for item in payload["extension_support_surface"]["profiles"]
        if item["id"] == "ros2_bridge_extension"
    )
    godot_profile = next(
        item
        for item in payload["extension_support_surface"]["profiles"]
        if item["id"] == "godot_extension"
    )

    assert payload["status"] == "ready"
    assert payload["required_documents"] == 14
    assert payload["required_documents_ready"] == 14
    assert payload["support_matrix_attached"] is True
    assert payload["capacity_declaration_attached"] is True
    assert payload["customer_acceptance_checklist_attached"] is True
    assert payload["known_limitations_attached"] is True
    assert payload["extension_support_surface"]["status"] == "ready"
    assert payload["extension_support_surface"]["required_profiles"] == 4
    assert payload["extension_support_surface"]["declared_profiles"] == 4
    assert "PRODUCTION_DEPLOYMENT_RUNBOOK.md" in distributed_profile["runbook_entrypoints"]
    assert "docs/guides/DISTRIBUTED_GUIDE.md" in distributed_profile["runbook_entrypoints"]
    assert distributed_profile["execution_template"]["rollback_owner_role"] == "rollback_owner"
    assert distributed_profile["execution_template"]["handoff_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["watch_owner_role"] == "customer_operator"
    assert distributed_profile["execution_template"]["on_call_handoff_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["residual_risk_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["exception_review_owner_role"] == "delivery_lead"
    assert (
        distributed_profile["execution_template"]["incident_escalation_owner_role"]
        == "customer_operator"
    )
    assert distributed_profile["execution_template"]["escalation_closure_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["rollback_evidence_owner_role"] == "rollback_owner"
    assert distributed_profile["execution_template"]["operator_roles"][0]["id"] == "delivery_lead"
    assert distributed_profile["execution_template"]["upgrade_window_steps"][0]["order"] == 1
    assert (
        distributed_profile["execution_template"]["handoff_checkpoints"][1]["owner_role"]
        == "customer_operator"
    )
    assert (
        distributed_profile["execution_template"]["signoff_checkpoints"][0]["required_artifact"]
        == "test_env/distributed_smoke/distributed_smoke_report.json"
    )
    assert (
        distributed_profile["execution_template"]["watch_actions"][0]["owner_role"]
        == "customer_operator"
    )
    assert (
        distributed_profile["execution_template"]["on_call_handoff_records"][1][
            "required_artifact"
        ]
        == "test_env/distributed_smoke/distributed_smoke_report.json"
    )
    assert (
        distributed_profile["execution_template"]["residual_risk_handoff_steps"][1][
            "required_artifact"
        ]
        == "test_env/release_evidence/security/security_posture_report.json"
    )
    assert (
        distributed_profile["execution_template"]["exception_review_steps"][0][
            "required_artifact"
        ]
        == "deployment/security/vulnerability_exceptions.input.json"
    )
    assert (
        distributed_profile["execution_template"]["incident_escalation_steps"][0][
            "escalation_target"
        ]
        == "docs/guides/INCIDENT_RESPONSE_MATRIX.md"
    )
    assert (
        distributed_profile["execution_template"]["escalation_closure_steps"][0][
            "required_artifact"
        ]
        == "test_env/industrial_promotion_ready/industrial_promotion_checklist.json"
    )
    assert (
        distributed_profile["execution_template"]["rollback_steps"][1]["owner_role"]
        == "rollback_owner"
    )
    assert (
        distributed_profile["execution_template"]["rollback_evidence_archive_steps"][0][
            "target_path"
        ]
        == "test_env/release/customer_acceptance_bundle.json"
    )
    assert distributed_profile["deployment_commands"][0].startswith("docker compose")
    assert distributed_profile["acceptance_checks"][2].startswith(
        "python tests/run_distributed_smoke.py"
    )
    assert "workflow_archive" in distributed_profile["rollback_prerequisites"][0]
    assert "docs/ros2/ROS2_QUICK_START.md" in ros2_profile["runbook_entrypoints"]
    assert ros2_profile["execution_template"]["operator_roles"][1]["label"] == "Customer Operator"
    assert ros2_profile["execution_template"]["upgrade_window_steps"][1]["owner_role"] == "delivery_lead"
    assert ros2_profile["execution_template"]["watch_owner_role"] == "customer_operator"
    assert ros2_profile["execution_template"]["on_call_handoff_owner_role"] == "delivery_lead"
    assert (
        ros2_profile["execution_template"]["signoff_checkpoints"][0]["required_artifact"]
        == "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json"
    )
    assert (
        ros2_profile["execution_template"]["exception_review_steps"][1][
            "required_artifact"
        ]
        == "test_env/release_evidence/security/vulnerability_exception_report.json"
    )
    assert (
        ros2_profile["execution_template"]["incident_escalation_steps"][1][
            "escalation_target"
        ]
        == "rollback_owner"
    )
    assert (
        ros2_profile["execution_template"]["escalation_closure_steps"][1][
            "required_artifact"
        ]
        == "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json"
    )
    assert ros2_profile["deployment_commands"][1].startswith("ros2 launch")
    assert "AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1" in ros2_profile["acceptance_checks"][2]
    assert "ROS2 Humble" in ros2_profile["rollback_prerequisites"][0]
    assert "docs/guides/GODOT_TESTING_GUIDE.md" in godot_profile["runbook_entrypoints"]
    assert godot_profile["execution_template"]["operator_roles"][2]["id"] == "rollback_owner"
    assert "backend selection" in godot_profile["execution_template"]["rollback_steps"][1]["title"]
    assert godot_profile["execution_template"]["on_call_handoff_owner_role"] == "delivery_lead"
    assert godot_profile["execution_template"]["residual_risk_owner_role"] == "delivery_lead"
    assert (
        godot_profile["execution_template"]["exception_review_steps"][0][
            "required_artifact"
        ]
        == "deployment/security/vulnerability_exceptions.input.json"
    )
    assert (
        godot_profile["execution_template"]["rollback_evidence_archive_steps"][1][
            "target_path"
        ]
        == "test_env/godot_headless_smoke/headless_smoke_report.json"
    )
    assert (
        godot_profile["execution_template"]["escalation_closure_steps"][1][
            "required_artifact"
        ]
        == "test_env/godot_headless_smoke/headless_smoke_report.json"
    )
    assert "GODOT_EXECUTABLE" in godot_profile["deployment_commands"][0]
    assert "AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1" in godot_profile["acceptance_checks"][0]
    assert "legacy 或 godot-agent" in godot_profile["rollback_prerequisites"][1]
    assert validate_customer_delivery_surface(payload) == []
    assert payload["release_ops_execution"]["status"] == "blocked"
    assert "release_ops_execution=blocked" in payload["summary"]

    (tmp_path / "docs" / "guides" / "CAPACITY_AND_SCALE.md").unlink()
    payload = build_customer_delivery_surface(project_root=tmp_path)

    assert payload["status"] == "blocked"
    assert payload["capacity_declaration_attached"] is False
    assert "docs/guides/CAPACITY_AND_SCALE.md" in payload["missing_phase_e_documents"]
    assert validate_customer_delivery_surface(payload) == []


def test_customer_delivery_surface_hydrates_release_ops_execution_when_present(
    tmp_path: Path,
) -> None:
    _seed_customer_delivery_docs(tmp_path)
    _seed_structured_required_evidence(tmp_path)

    payload = build_customer_delivery_surface(project_root=tmp_path)

    assert payload["status"] == "ready"
    assert payload["release_ops_execution"]["status"] == "passed"
    assert payload["release_ops_execution"]["event_count"] == 3
    assert payload["control_plane_event_stream"]["event_count"] == 3
    assert payload["control_plane_session"]["engagement_id"] == "release-contracts-session"
    assert "release_ops_execution=passed/3" in payload["summary"]
    assert "control_plane_events=3" in payload["summary"]
    assert validate_customer_delivery_surface(payload) == []


def test_industrial_delivery_gate_tracks_security_and_evidence(tmp_path: Path) -> None:
    _seed_customer_delivery_docs(tmp_path)
    _write_clean_checkout_smoke_report(tmp_path)
    _seed_structured_required_evidence(tmp_path)
    _seed_industrial_security_reports(tmp_path)

    payload = build_industrial_delivery_gate(project_root=tmp_path)

    assert payload["status"] == "ready"
    assert payload["deployment_package_status"] == "ready"
    assert payload["evidence_attested"] is True
    assert payload["sbom_attached"] is True
    assert payload["vuln_scan_status"] == "passed"
    assert payload["backup_restore_verified"] is True
    assert payload["customer_delivery_surface_status"] == "ready"
    assert payload["extension_support_surface_status"] == "ready"
    assert payload["required_extension_profiles"] == 4
    assert payload["declared_extension_profiles"] == 4
    assert payload["release_ops_execution"]["status"] == "passed"
    assert payload["release_ops_execution"]["event_count"] == 3
    assert payload["control_plane_event_stream"]["event_count"] == 3
    assert "release_ops_execution=passed/3" in payload["summary"]
    assert "extension_support=ready (4/4)" in payload["summary"]
    assert validate_industrial_delivery_gate(payload) == []

    (tmp_path / "test_env" / "release_evidence" / "security" / "sbom.json").unlink()
    blocked_payload = build_industrial_delivery_gate(project_root=tmp_path)

    assert blocked_payload["status"] == "blocked"
    assert blocked_payload["sbom_attached"] is False
    assert "test_env/release_evidence/security/sbom.json" in blocked_payload["missing_requirements"]
    assert validate_industrial_delivery_gate(blocked_payload) == []


def test_industrial_delivery_gate_accepts_exception_covered_findings(tmp_path: Path) -> None:
    _seed_customer_delivery_docs(tmp_path)
    _write_clean_checkout_smoke_report(tmp_path)
    _seed_structured_required_evidence(tmp_path)
    _seed_exception_covered_industrial_security_reports(tmp_path)

    payload = build_industrial_delivery_gate(project_root=tmp_path)

    assert payload["status"] == "ready"
    assert payload["vuln_scan_status"] == "passed"
    assert payload["evidence_attested"] is True
    assert "ready remediation report" in payload["summary"]
    assert validate_industrial_delivery_gate(payload) == []


def test_release_manifest_hydrates_distributed_report_and_updates_matrix(
    tmp_path: Path,
):
    _write_clean_checkout_smoke_report(tmp_path)
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
        "customer_delivery_required": 0,
        "customer_delivery_ready": 1,
        "industrial_delivery_required": 0,
        "industrial_delivery_ready": 1,
    }
    assert payload["known_limitations"]


def test_dev_release_policy_allows_open_opt_in_evidence(tmp_path: Path):
    _write_clean_checkout_smoke_report(tmp_path)
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
        "requires_customer_delivery_surface": False,
        "requires_industrial_delivery_gate": False,
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
        "customer_delivery_required": 0,
        "customer_delivery_ready": 0,
        "industrial_delivery_required": 0,
        "industrial_delivery_ready": 0,
    }


def test_stable_release_requires_explicit_approval(tmp_path: Path):
    repo_source = _init_git_repo(tmp_path)
    _write_clean_checkout_smoke_report(tmp_path)
    payload = build_release_manifest_artifact(
        build_id="build-20260412-008",
        version="2026.04.12",
        channel="stable",
        release_summary="稳定通道验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        project_root=tmp_path,
        source_root=repo_source["root"],
    )

    assert payload["release_policy"] == {
        "channel": "stable",
        "allows_opt_in_evidence": False,
        "allows_diagnostic_ready_domains": False,
        "requires_release_approval": True,
        "requires_git_source_binding": True,
        "requires_clean_worktree": True,
        "requires_version_tag_match": True,
        "requires_customer_delivery_surface": False,
        "requires_industrial_delivery_gate": False,
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
    assert payload["release_source"] == {
        key: value for key, value in repo_source.items() if key != "root"
    }
    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["release_approval_required"] == 1
    assert payload["release_gate"]["release_approval_ready"] == 0
    assert payload["release_gate"]["release_source_required"] == 1
    assert payload["release_gate"]["release_source_ready"] == 0
    assert payload["release_gate"]["release_worktree_required"] == 1
    assert payload["release_gate"]["release_worktree_ready"] == 1
    assert payload["release_gate"]["release_version_tag_required"] == 1
    assert payload["release_gate"]["release_version_tag_ready"] == 0


def test_stable_release_becomes_ready_after_approval_and_live_evidence(tmp_path: Path):
    repo_source = _init_git_repo(tmp_path, tag="2026.04.12")
    _write_clean_checkout_smoke_report(tmp_path)
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
        "customer_delivery_required": 0,
        "customer_delivery_ready": 0,
        "industrial_delivery_required": 0,
        "industrial_delivery_ready": 0,
    }


def test_stable_release_blocks_when_approval_commit_does_not_match_head(tmp_path: Path):
    repo_source = _init_git_repo(tmp_path, tag="2026.04.12")
    _write_clean_checkout_smoke_report(tmp_path)
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
        source_root=repo_source["root"],
    )

    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["release_approval_ready"] == 1
    assert payload["release_gate"]["release_source_ready"] == 0
    assert payload["release_gate"]["release_worktree_ready"] == 1
    assert payload["release_gate"]["release_version_tag_ready"] == 1


def test_stable_release_blocks_without_matching_version_tag(tmp_path: Path):
    repo_source = _init_git_repo(tmp_path)
    _write_clean_checkout_smoke_report(tmp_path)
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
        project_root=tmp_path,
        source_root=repo_source["root"],
    )

    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["release_source_ready"] == 1
    assert payload["release_gate"]["release_worktree_ready"] == 1
    assert payload["release_gate"]["release_version_tag_ready"] == 0


def test_stable_release_blocks_with_dirty_worktree_even_when_tag_and_approval_match(
    tmp_path: Path,
):
    repo_source = _init_git_repo(tmp_path, tag="2026.04.12")
    _write_clean_checkout_smoke_report(tmp_path)
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


def test_industrial_release_requires_customer_delivery_and_industrial_gate(
    tmp_path: Path,
):
    repo_source = _init_git_repo(tmp_path, tag="2026.04.12")
    _write_clean_checkout_smoke_report(tmp_path)
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
        build_id="build-20260412-013",
        version="2026.04.12",
        channel="industrial",
        release_summary="工业交付通道验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "notes": "industrial signoff",
        },
        project_root=tmp_path,
        source_root=repo_source["root"],
    )

    assert payload["release_policy"] == {
        "channel": "industrial",
        "allows_opt_in_evidence": False,
        "allows_diagnostic_ready_domains": False,
        "requires_release_approval": True,
        "requires_git_source_binding": True,
        "requires_clean_worktree": True,
        "requires_version_tag_match": True,
        "requires_customer_delivery_surface": True,
        "requires_industrial_delivery_gate": True,
        "summary": "Industrial releases require the full stable gate plus ready customer delivery and industrial delivery surfaces before the gate becomes ready.",
    }
    assert payload["customer_delivery_surface"]["status"] == "blocked"
    assert payload["industrial_delivery_gate"]["status"] == "blocked"
    assert payload["release_gate_status"] == "blocked"
    assert payload["release_gate"]["customer_delivery_required"] == 1
    assert payload["release_gate"]["customer_delivery_ready"] == 0
    assert payload["release_gate"]["industrial_delivery_required"] == 1
    assert payload["release_gate"]["industrial_delivery_ready"] == 0
    assert validate_release_manifest_artifact(payload) == []


def test_industrial_release_becomes_ready_when_delivery_gates_close(tmp_path: Path):
    repo_source = _init_git_repo(tmp_path, tag="2026.04.12")
    _write_clean_checkout_smoke_report(tmp_path)
    _seed_structured_required_evidence(tmp_path)
    _seed_customer_delivery_docs(tmp_path)
    _seed_industrial_security_reports(tmp_path)
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
        build_id="build-20260412-014",
        version="2026.04.12",
        channel="industrial",
        release_summary="工业交付通道闭环验证。",
        generated_at="2026-04-12T12:00:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "notes": "industrial signoff",
        },
        project_root=tmp_path,
        source_root=repo_source["root"],
    )

    assert payload["release_gate_status"] == "ready"
    assert payload["customer_delivery_surface"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["exception_reviews_scheduled"] == 3
    assert payload["extension_execution_actuals"]["archive_targets_ready"] == 3
    assert payload["extension_execution_actuals"]["archive_target_bindings_ready"] == 3
    assert payload["extension_execution_actuals"]["due_trigger_checks_ready"] == 3
    assert payload["extension_execution_actuals"]["due_trigger_bindings_ready"] == 3
    assert payload["extension_execution_actuals"]["closure_indexes_ready"] == 3
    assert payload["release_gate"]["customer_delivery_required"] == 1
    assert payload["release_gate"]["customer_delivery_ready"] == 1
    assert payload["release_gate"]["industrial_delivery_required"] == 1
    assert payload["release_gate"]["industrial_delivery_ready"] == 1
    assert validate_release_manifest_artifact(payload) == []
