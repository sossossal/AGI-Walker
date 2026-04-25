from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_external_mainline_execution_plan_artifact,
    build_external_mainline_input_checklist_report,
    build_customer_acceptance_bundle_artifact,
    build_industrial_delivery_rehearsal_report_artifact,
    build_extension_execution_actuals_artifact,
    build_extension_execution_instance_artifact,
    build_extension_execution_schedule_artifact,
    build_release_evidence_report,
    build_release_manifest_artifact,
    validate_customer_acceptance_bundle_artifact,
    write_external_mainline_execution_plan_artifact,
    write_industrial_delivery_rehearsal_report_artifact,
    write_extension_execution_actuals_artifact,
    write_extension_execution_instance_artifact,
    write_extension_execution_schedule_artifact,
    write_release_evidence_report,
    write_release_manifest_artifact,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _seed_required_docs(project_root: Path) -> None:
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


def _seed_required_evidence(project_root: Path) -> None:
    report_root = project_root / "test_env" / "release_evidence"
    report_root.mkdir(parents=True, exist_ok=True)
    security_root = report_root / "security"
    security_root.mkdir(parents=True, exist_ok=True)
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="non_live_gate",
            status="passed",
            summary="non_live_gate pytest evidence passed: 802 passed, 3 skipped, 3 deselected.",
            command='python -m pytest -m "not live" -q',
            source_commit_sha="acceptance-commit",
        ),
        report_root / "non_live_gate_report.json",
    )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="release_contracts_and_capability_matrix",
            status="passed",
            summary="release_contracts_and_capability_matrix pytest evidence passed: 38 passed.",
            command="python -m pytest tests/test_release_contracts.py tests/test_release_artifact_builder.py tests/test_capability_matrix.py tests/test_mcp_tools.py tests/test_mcp_server.py tests/test_web_panel_aux_apis.py -q",
            source_commit_sha="acceptance-commit",
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
    ]:
        write_release_evidence_report(
            build_release_evidence_report(
                evidence_name=evidence_name,
                status="passed",
                summary=summary,
                command="python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations",
                source_commit_sha="acceptance-commit",
                metrics=metrics,
            ),
            operations_root / file_name,
        )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="customer_external_bindings_confirmation",
            status="passed",
            summary=(
                "customer_external_bindings_confirmation evidence passed: "
                "All external binding sections are confirmed."
            ),
            command=(
                "python tools/build_customer_external_bindings_confirmation_report.py "
                "--output test_env/release_evidence/operations/"
                "customer_external_bindings_confirmation_report.json "
                "--actuals-artifact test_env/release_evidence/operations/"
                "extension_execution_actuals.json"
            ),
            source_commit_sha="acceptance-commit",
            metrics={
                "external_bindings_status": "ready",
                "confirmed_sections": [
                    "approval_identity",
                    "archive_target",
                    "due_trigger",
                ],
                "confirmation_tickets": ["CHG-ACCEPTANCE-BUNDLE"],
            },
        ),
        operations_root / "customer_external_bindings_confirmation_report.json",
    )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="customer_external_bindings_closure",
            status="passed",
            summary=(
                "customer_external_bindings_closure evidence passed: "
                "approval_identity, archive_target, due_trigger confirmed "
                "and collect_release_evidence completed."
            ),
            command=(
                "python tools/run_customer_external_bindings_closure.py "
                "--config deployment/customer_delivery.external_bindings.customer.json "
                "--section approval_identity --section archive_target --section due_trigger "
                "--confirmed-by release-manager --confirmation-ticket CHG-ACCEPTANCE-BUNDLE"
            ),
            source_commit_sha="acceptance-commit",
            metrics={
                "selected_sections": [
                    "approval_identity",
                    "archive_target",
                    "due_trigger",
                ],
                "failed_steps": [],
                "collect_release_evidence": True,
                "confirmation_report_exists": True,
                "actuals_exists": True,
            },
        ),
        operations_root / "customer_external_bindings_closure_report.json",
    )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="release_ops_execution",
            status="passed",
            summary=(
                "release op stable_promotion_checklist completed via control plane "
                "with canonical evidence wrapper."
            ),
            command=(
                "python tools/run_release_ops.py stable_promotion_checklist "
                "--request-file test_env/release_ops/stable_promotion_request.json "
                "--event-stream-file test_env/release_ops/stable_promotion.jsonl "
                "--evidence-report-file test_env/release_evidence/operations/"
                "release_ops_execution_report.json"
            ),
            source_commit_sha="acceptance-commit",
            metrics={
                "action": "stable_promotion_checklist",
                "policy_level": "local_safe_refresh",
                "policy_profile": "local_safe_refresh",
                "request_type": "StablePromotionChecklistRequest",
                "status": "ready",
                "event_count": 3,
                "output_path": "test_env/stable_promotion_ready/stable_promotion_checklist.json",
            },
            control_plane_session={
                "engagement_id": "acceptance-bundle-session",
                "window_id": "window-acceptance",
                "change_ticket": "CHG-ACCEPTANCE-BUNDLE",
                "channel": "ops-cli",
            },
            control_plane_event_stream={
                "path": "test_env/release_ops/stable_promotion.jsonl",
                "event_count": 3,
            },
        ),
        operations_root / "release_ops_execution_report.json",
    )
    write_external_mainline_execution_plan_artifact(
        build_external_mainline_execution_plan_artifact(
            project_root=project_root,
            control_plane_session={
                "engagement_id": "acceptance-bundle-session",
                "window_id": "window-acceptance",
                "change_ticket": "CHG-ACCEPTANCE-BUNDLE",
                "channel": "ops-cli",
            },
            control_plane_event_stream={
                "path": "test_env/release_evidence/operations/release_ops.events.jsonl",
                "event_count": 3,
            },
        ),
        operations_root / "external_mainline_execution_plan.json",
    )
    write_release_evidence_report(
        build_external_mainline_input_checklist_report(
            project_root=project_root,
            output_path="test_env/release_evidence/operations/external_mainline_input_checklist_report.json",
            external_mainline_execution_plan_path="test_env/release_evidence/operations/external_mainline_execution_plan.json",
            control_plane_session={
                "engagement_id": "acceptance-bundle-session",
                "window_id": "window-acceptance",
                "change_ticket": "CHG-ACCEPTANCE-BUNDLE",
                "channel": "ops-cli",
            },
            control_plane_event_stream={
                "path": "test_env/release_evidence/operations/release_ops.events.jsonl",
                "event_count": 3,
            },
        ),
        operations_root / "external_mainline_input_checklist_report.json",
    )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="passed",
            summary=(
                "vulnerability_exception_review evidence passed: "
                "1 active exception(s) require review inside the 30-day window "
                "before 2026-05-15T00:00:00+00:00."
            ),
            command=(
                "python tools/build_vulnerability_exception_review_report.py "
                "--output test_env/release_evidence/security/"
                "vulnerability_exception_review_report.json "
                "--exception-report test_env/release_evidence/security/"
                "vulnerability_exception_report.json"
            ),
            source_commit_sha="acceptance-commit",
            metrics={
                "review_due_exception_count": 1,
                "review_due_exception_ids": ["webpanel-libsystemd0-no-fix"],
                "review_due_exception_tickets": ["SEC-ACCEPTANCE-BUNDLE"],
                "review_follow_up_required": True,
                "review_candidate_count": 1,
            },
        ),
        security_root / "vulnerability_exception_review_report.json",
    )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            engagement_id="acceptance-bundle-canonical",
            customer_name="AGI-Walker Customer",
            site_name="primary-site",
            change_ticket="CHG-ACCEPTANCE-BUNDLE",
            window_id="window-acceptance-bundle",
            window_start_at="2026-04-13T12:00:00+00:00",
            window_end_at="2026-04-13T14:00:00+00:00",
            delivery_root="test_env/release_delivery/acceptance_bundle",
            closure_archive_root="test_env/release_delivery/acceptance_bundle/closure_archive",
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
            generated_at="2026-04-13T12:00:00+00:00",
        ),
        operations_root / "extension_execution_actuals.json",
    )
    clean_checkout_report = report_root / "clean_checkout_smoke_report.json"
    clean_checkout_report.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "clean_checkout_smoke_report",
                "status": "passed",
                "generated_at": "2026-04-13T12:00:00+00:00",
                "source_root": str(project_root),
                "checkout_root": str(project_root / "test_env" / "clean_checkout_smoke" / "checkout"),
                "output_root": str(project_root / "test_env" / "clean_checkout_smoke"),
                "version": "2026.04.13-rc-evidence",
                "tag": "2026.04.13-rc-evidence",
                "runs": 2,
                "command_template": [
                    "python",
                    "tests/run_smoke_tests.py",
                    "--output-root",
                    "{run_output_root}",
                ],
                "checkout_commit_sha": "acceptance-clean-checkout",
                "seeded_evidence_paths": [],
                "checks": [
                    {
                        "name": "sequential_smoke_runs_are_clean",
                        "status": "pass",
                        "detail": "2 sequential smoke run(s) completed with empty git status after each run",
                    }
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
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (security_root / "sbom.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "sbom_artifact",
                "generated_at": "2026-04-13T12:00:00+00:00",
                "project_root": str(project_root),
                "project_name": "agi-walker",
                "project_version": "2026.04.13-rc-evidence",
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
    (security_root / "python_vuln_scan_report.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "vulnerability_scan_report",
                "generated_at": "2026-04-13T12:00:00+00:00",
                "scan_name": "python_dependencies",
                "target": "pyproject.toml",
                "status": "passed",
                "summary": "Python dependency review completed.",
                "command": "manual placeholder",
                "scanner": "manual-review",
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (security_root / "container_vuln_scan_report.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "vulnerability_scan_report",
                "generated_at": "2026-04-13T12:00:00+00:00",
                "scan_name": "container_images",
                "target": "deployment/docker-compose.yml",
                "status": "passed",
                "summary": "Container image review completed.",
                "command": "manual placeholder",
                "scanner": "manual-review",
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (security_root / "security_posture_report.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "security_posture_report",
                "generated_at": "2026-04-13T12:00:00+00:00",
                "project_root": str(project_root),
                "posture_status": "ready",
                "summary": "Security posture is ready.",
                "sbom": {
                    "path": str(security_root / "sbom.json"),
                    "exists": True,
                    "component_count": 1,
                },
                "vulnerability_reports": [
                    {
                        "name": "python_dependencies",
                        "path": str(security_root / "python_vuln_scan_report.json"),
                        "required": True,
                        "exists": True,
                        "status": "passed",
                        "scanner": "manual-review",
                    },
                    {
                        "name": "container_images",
                        "path": str(security_root / "container_vuln_scan_report.json"),
                        "required": True,
                        "exists": True,
                        "status": "passed",
                        "scanner": "manual-review",
                    },
                ],
                "baseline_documents": [
                    {
                        "name": "security_baseline",
                        "path": str(project_root / "docs" / "guides" / "SECURITY_BASELINE.md"),
                        "required": True,
                        "exists": True,
                    }
                ],
                "backup_restore_rehearsal": {
                    "path": str(security_root / "backup_restore_rehearsal_report.json"),
                    "exists": True,
                    "status": "passed",
                    "rehearsal_duration_seconds": 1.5,
                },
                "vulnerability_exception_report": {
                    "path": str(security_root / "vulnerability_exception_report.json"),
                    "exists": True,
                    "active_exception_count": 31,
                    "expired_exception_count": 0,
                    "stale_exception_count": 0,
                    "stale_exception_ids": [],
                    "review_window_days": 30,
                    "review_due_exception_count": 31,
                    "next_exception_expiry": "2026-05-15T00:00:00+00:00",
                    "review_status": "review_due",
                },
                "accepted_vulnerability_findings": 104,
                "unresolved_vulnerability_findings": 0,
                "missing_vulnerability_reports": 0,
                "blocked_vulnerability_reports": 0,
                "missing_documents": 0,
                "missing_backup_restore_rehearsal_reports": 0,
                "blocked_backup_restore_rehearsal_reports": 0,
                "next_actions": [],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    (security_root / "backup_restore_rehearsal_report.json").write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "backup_restore_rehearsal_report",
                "generated_at": "2026-04-13T12:00:00+00:00",
                "project_root": str(project_root),
                "actor": "test-runner",
                "status": "passed",
                "summary": "Backup and restore rehearsal passed.",
                "release_manifest_path": None,
                "source_runtime_root": str(project_root / "test_env" / "backup_restore" / "source_runtime"),
                "source_config_root": str(project_root / "test_env" / "backup_restore" / "source_config"),
                "backup_snapshot_root": str(project_root / "test_env" / "backup_restore" / "backup_snapshot"),
                "restored_runtime_root": str(project_root / "test_env" / "backup_restore" / "restored_runtime"),
                "restored_config_root": str(project_root / "test_env" / "backup_restore" / "restored_config"),
                "backup_items": [
                    {
                        "name": "db",
                        "source_path": "source/db",
                        "backup_path": "backup/db",
                        "required": True,
                        "exists": True,
                    }
                ],
                "restore_checks": [
                    {
                        "name": "db",
                        "source_path": "source/db",
                        "restored_path": "restored/db",
                        "required": True,
                        "passed": True,
                    }
                ],
                "missing_backup_items": 0,
                "failed_restore_checks": 0,
                "rpo_target": "24 hours",
                "rto_target": "4 hours",
                "rehearsal_duration_seconds": 1.5,
                "next_actions": [],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


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


def _write_ready_manifest(project_root: Path) -> Path:
    _seed_required_docs(project_root)
    _seed_required_evidence(project_root)
    manifest_path = project_root / "test_env" / "release" / "release_manifest.json"
    payload = build_release_manifest_artifact(
        build_id="build-acceptance-test",
        version="2026.04.13-rc-evidence",
        channel="dev",
        release_summary="customer acceptance bundle test",
        project_root=project_root,
        source_root=project_root,
    )
    assert payload["release_gate_status"] == "ready"
    return write_release_manifest_artifact(payload, manifest_path)


def _init_git_repo(tmp_path: Path, *, tag: str | None = None) -> Path:
    repo_root = tmp_path / "git_source"
    repo_root.mkdir(parents=True, exist_ok=True)
    (repo_root / "README.md").write_text("# acceptance repo\n", encoding="utf-8")
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
    return repo_root


def _write_ready_industrial_manifest(project_root: Path, source_root: Path) -> Path:
    _seed_required_docs(project_root)
    _seed_required_evidence(project_root)
    _seed_live_evidence(project_root)
    manifest_path = (
        project_root / "test_env" / "release" / "release_manifest_industrial.json"
    )
    payload = build_release_manifest_artifact(
        build_id="build-acceptance-industrial",
        version="2026.04.13",
        channel="industrial",
        release_summary="customer acceptance industrial bundle test",
        generated_at="2026-04-13T12:31:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-13T12:30:00+00:00",
            "notes": "industrial signoff",
        },
        project_root=project_root,
        source_root=source_root,
    )
    assert payload["release_gate_status"] == "ready"
    return write_release_manifest_artifact(payload, manifest_path)


def test_customer_acceptance_bundle_artifact_is_valid_for_ready_manifest(
    tmp_path: Path,
) -> None:
    manifest_path = _write_ready_manifest(tmp_path)
    manifest_payload = json.loads(manifest_path.read_text(encoding="utf-8"))

    bundle = build_customer_acceptance_bundle_artifact(
        release_manifest=manifest_payload,
        manifest_path=manifest_path,
        project_root=tmp_path,
    )

    assert validate_customer_acceptance_bundle_artifact(bundle) == []
    assert bundle["bundle_status"] == "ready"
    assert bundle["release_manifest"]["release_gate_status"] == "ready"
    assert bundle["known_limitations"] == manifest_payload["known_limitations"]
    assert bundle["extension_support_surface"]["status"] == "ready"
    assert bundle["extension_support_surface"]["declared_profiles"] == 4
    assert bundle["extension_execution_plan"]["status"] == "ready"
    assert bundle["extension_execution_plan"]["actionable_profiles"] == 3
    assert bundle["extension_execution_evidence"]["status"] == "ready"
    assert bundle["extension_execution_evidence"]["ready_reports"] == 4
    assert bundle["extension_execution_instance"]["status"] == "ready"
    assert bundle["extension_execution_instance"]["ready_profiles"] == 3
    assert bundle["extension_execution_schedule"]["status"] == "ready"
    assert bundle["extension_execution_schedule"]["ready_profiles"] == 3
    assert bundle["extension_execution_actuals"]["status"] == "ready"
    assert bundle["extension_execution_actuals"]["ready_profiles"] == 3
    distributed_profile = next(
        item
        for item in bundle["extension_execution_plan"]["profiles"]
        if item["id"] == "distributed_profile"
    )
    assert "PRODUCTION_DEPLOYMENT_RUNBOOK.md" in distributed_profile["runbook_entrypoints"]
    assert "docs/guides/DISTRIBUTED_GUIDE.md" in distributed_profile["runbook_entrypoints"]
    assert distributed_profile["execution_template"]["rollback_owner_role"] == "rollback_owner"
    assert distributed_profile["execution_template"]["handoff_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["watch_owner_role"] == "customer_operator"
    assert distributed_profile["execution_template"]["on_call_handoff_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["residual_risk_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["exception_review_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["escalation_closure_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["rollback_evidence_owner_role"] == "rollback_owner"
    assert distributed_profile["execution_template"]["operator_roles"][1]["id"] == "customer_operator"
    assert distributed_profile["execution_template"]["upgrade_window_steps"][2]["owner_role"] == "customer_operator"
    assert (
        distributed_profile["execution_template"]["signoff_checkpoints"][1][
            "required_artifact"
        ]
        == "test_env/industrial_promotion_ready/industrial_promotion_checklist.json"
    )
    assert (
        distributed_profile["execution_template"]["on_call_handoff_records"][0][
            "required_artifact"
        ]
        == "test_env/release/customer_acceptance_bundle.json"
    )
    assert (
        distributed_profile["execution_template"]["exception_review_steps"][1][
            "required_artifact"
        ]
        == "test_env/release_evidence/security/vulnerability_exception_report.json"
    )
    assert (
        distributed_profile["execution_template"]["incident_escalation_steps"][0][
            "escalation_target"
        ]
        == "docs/guides/INCIDENT_RESPONSE_MATRIX.md"
    )
    assert (
        distributed_profile["execution_template"]["escalation_closure_steps"][1][
            "required_artifact"
        ]
        == "test_env/distributed_smoke/distributed_smoke_report.json"
    )
    assert distributed_profile["deployment_commands"][0].startswith("docker compose")
    assert distributed_profile["acceptance_checks"][1].endswith("/api/distributed/status")
    assert "workflow_archive" in distributed_profile["rollback_prerequisites"][0]
    assert (
        "python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json"
        in bundle["recommended_commands"]
    )
    assert (
        'AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv'
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_external_mainline_inputs.py --output "
        "deployment/external_mainline.inputs.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/run_external_mainline_execution_plan.py --output "
        "test_env/release_evidence/operations/external_mainline_execution_plan.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_external_mainline_execution_plan.py --output "
        "test_env/release_evidence/operations/external_mainline_execution_plan.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_external_mainline_input_checklist.py --output "
        "test_env/release_evidence/operations/external_mainline_input_checklist_report.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_customer_external_bindings_config.py --output deployment/customer_delivery.external_bindings.customer.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/confirm_customer_external_bindings.py --config deployment/customer_delivery.external_bindings.customer.json"
        " --section approval_identity --section archive_target --section due_trigger"
        " --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json"
        " --section approval_identity --section archive_target --section due_trigger"
        " --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json"
        " --external-bindings-config deployment/customer_delivery.external_bindings.customer.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_customer_external_bindings_confirmation_report.py --output "
        "test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json "
        "--actuals-artifact test_env/release_evidence/operations/extension_execution_actuals.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_vulnerability_exception_review_report.py --output "
        "test_env/release_evidence/security/vulnerability_exception_review_report.json "
        "--exception-report test_env/release_evidence/security/vulnerability_exception_report.json"
        in bundle["recommended_commands"]
    )
    assert [item["name"] for item in bundle["acceptance_documents"]] == [
        "readme",
        "current_status",
        "release_guide",
        "deployment_matrix",
        "customer_installation_guide",
        "support_matrix",
        "capacity_and_scale",
        "customer_acceptance_checklist",
        "known_limitations_guide",
        "production_runbook",
        "security_baseline",
        "audit_trail_policy",
        "backup_restore_runbook",
        "incident_response_matrix",
    ]
    assert [item["name"] for item in bundle["required_evidence"]] == [
        "clean_checkout_smoke",
        "non_live_gate",
        "release_contracts_and_capability_matrix",
    ]
    assert all(item["exists"] is True for item in bundle["acceptance_documents"])
    reports_by_name = {item["name"]: item for item in bundle["acceptance_reports"]}
    assert reports_by_name["security_posture_report"]["exists"] is True
    assert reports_by_name["security_posture_report"]["status"] == "ready"
    assert reports_by_name["security_posture_report"]["exception_review_status"] == "review_due"
    assert reports_by_name["security_posture_report"]["review_due_exception_count"] == 31
    assert reports_by_name["security_posture_report"]["stale_exception_count"] == 0
    assert (
        reports_by_name["security_posture_report"]["next_exception_expiry"]
        == "2026-05-15T00:00:00+00:00"
    )
    assert reports_by_name["sbom_artifact"]["status"] == "ready"
    assert reports_by_name["python_vulnerability_scan_report"]["status"] == "passed"
    assert reports_by_name["container_vulnerability_scan_report"]["status"] == "passed"
    assert reports_by_name["backup_restore_rehearsal_report"]["status"] == "passed"
    assert reports_by_name["vulnerability_exception_review"]["status"] == "passed"
    assert (
        reports_by_name["vulnerability_exception_review"]["metrics"][
            "review_due_exception_count"
        ]
        == 1
    )
    assert bundle["vulnerability_exception_review"]["status"] == "passed"
    assert (
        bundle["vulnerability_exception_review"]["metrics"]["review_due_exception_count"]
        == 1
    )
    assert reports_by_name["customer_external_bindings_closure"]["status"] == "passed"
    assert reports_by_name["external_mainline_execution_plan"]["exists"] is True
    assert reports_by_name["external_mainline_execution_plan"]["status"] == "ready"
    assert reports_by_name["external_mainline_input_checklist"]["exists"] is True
    assert reports_by_name["external_mainline_input_checklist"]["status"] == "passed"
    assert reports_by_name["release_ops_execution"]["exists"] is True
    assert reports_by_name["release_ops_execution"]["status"] == "passed"
    assert bundle["external_mainline_execution_plan"]["status"] == "ready"
    assert (
        bundle["external_mainline_execution_plan"]["ready_to_run_steps"]
        == reports_by_name["external_mainline_execution_plan"]["ready_to_run_steps"]
    )
    assert bundle["external_mainline_input_checklist"]["status"] == "passed"
    assert bundle["release_ops_execution"]["status"] == "passed"
    assert bundle["release_ops_execution"]["metrics"]["event_count"] == 3
    assert bundle["control_plane_session"]["engagement_id"] == "acceptance-bundle-session"
    assert bundle["control_plane_event_stream"]["event_count"] == 3
    assert "external_mainline=ready/1/2/0/0." in bundle["summary"]
    assert "external_mainline_input_checklist=passed/" in bundle["summary"]
    assert "release_ops_execution=passed/3" in bundle["summary"]
    assert "control_plane_events=3" in bundle["summary"]
    assert (
        reports_by_name["customer_external_bindings_closure"]["metrics"][
            "collect_release_evidence"
        ]
        is True
    )
    assert reports_by_name["customer_external_bindings_confirmation"]["status"] == "passed"
    assert (
        reports_by_name["customer_external_bindings_confirmation"]["metrics"][
            "external_bindings_status"
        ]
        == "ready"
    )
    assert reports_by_name["extension_on_call_rehearsal"]["status"] == "passed"
    assert reports_by_name["extension_exception_review_schedule"]["status"] == "passed"
    assert reports_by_name["extension_escalation_closure"]["status"] == "passed"


def test_build_customer_acceptance_bundle_script_writes_bundle(tmp_path: Path) -> None:
    manifest_path = _write_ready_manifest(tmp_path)
    output_path = tmp_path / "test_env" / "release" / "customer_acceptance_bundle.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_acceptance_bundle.py",
            "--manifest",
            str(manifest_path),
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
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "customer_acceptance_bundle_written=" in result.stdout
    assert "customer_acceptance_bundle_status=ready" in result.stdout
    assert "customer_acceptance_bundle_reports_present=" in result.stdout
    assert "customer_acceptance_bundle_security_posture=ready" in result.stdout
    assert "customer_acceptance_bundle_vulnerability_exception_review=passed/1" in result.stdout
    assert (
        "customer_acceptance_bundle_external_mainline_execution_plan=ready/1/2/0/0"
        in result.stdout
    )
    assert (
        "customer_acceptance_bundle_external_mainline_input_checklist=passed/"
        in result.stdout
    )
    assert "customer_acceptance_bundle_release_ops_execution=passed/3" in result.stdout
    assert "customer_acceptance_bundle_control_plane_events=3" in result.stdout
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_customer_acceptance_bundle_artifact(payload) == []
    assert payload["control_plane_event_stream"]["event_count"] == 3


def test_customer_acceptance_bundle_blocks_when_required_documents_missing(
    tmp_path: Path,
) -> None:
    manifest_path = _write_ready_manifest(tmp_path)
    (tmp_path / "docs" / "guides" / "SECURITY_BASELINE.md").unlink()
    manifest_payload = json.loads(manifest_path.read_text(encoding="utf-8"))

    bundle = build_customer_acceptance_bundle_artifact(
        release_manifest=manifest_payload,
        manifest_path=manifest_path,
        project_root=tmp_path,
    )

    assert validate_customer_acceptance_bundle_artifact(bundle) == []
    assert bundle["bundle_status"] == "blocked"
    security_baseline = next(
        item
        for item in bundle["acceptance_documents"]
        if item["name"] == "security_baseline"
    )
    assert security_baseline["exists"] is False


def test_customer_acceptance_bundle_uses_industrial_reports_for_industrial_channel(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root = _init_git_repo(tmp_path, tag="2026.04.13")
    manifest_path = _write_ready_industrial_manifest(project_root, source_root)
    manifest_payload = json.loads(manifest_path.read_text(encoding="utf-8"))

    industrial_readiness_path = (
        project_root
        / "test_env"
        / "industrial_release_readiness_ready"
        / "industrial_release_readiness_report.json"
    )
    industrial_readiness_path.parent.mkdir(parents=True, exist_ok=True)
    industrial_readiness_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "industrial_release_readiness_report",
                "current_version": "2026.04.13",
                "industrial_version": "2026.04.13",
                "industrial_release_gate": "ready",
                "summary": "industrial release readiness is ready.",
                "previews": [],
                "next_step_plan": [],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    industrial_promotion_path = (
        project_root
        / "test_env"
        / "industrial_promotion_ready"
        / "industrial_promotion_checklist.json"
    )
    industrial_promotion_path.parent.mkdir(parents=True, exist_ok=True)
    industrial_promotion_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "industrial_promotion_checklist",
                "industrial_version": "2026.04.13",
                "industrial_release_gate": "ready",
                "blocking_steps": 0,
                "ready_to_promote": True,
                "summary": "industrial promotion 前置项已闭合，最终 manifest 生成命令可执行。",
                "steps": [],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    industrial_delivery_rehearsal_report_path = (
        project_root / "industrial_delivery_rehearsal_report.json"
    )
    write_industrial_delivery_rehearsal_report_artifact(
        build_industrial_delivery_rehearsal_report_artifact(
            release_rehearsal_report={
                "status": "passed",
                "version": "2026.04.13",
                "tag": "2026.04.13",
                "source_root": str(source_root),
                "git_source": {"commit_sha": "industrial-acceptance-commit"},
                "release_gate_status": "ready",
                "customer_delivery_status": "ready",
                "industrial_delivery_status": "ready",
                "security_release_preflight": {
                    "status": "passed",
                    "summary": "security preflight passed",
                    "metrics": {
                        "vulnerability_exception_review_report_status": "passed",
                        "vulnerability_exception_review_candidate_count": 1,
                    },
                },
                "vulnerability_exception_review": {
                    "status": "passed",
                    "summary": "vulnerability exception review status=passed, candidates=1.",
                    "review_candidate_count": 1,
                },
                "external_mainline_execution_plan": {
                    "status": "ready",
                    "summary": "external mainline execution plan ready.",
                    "completed_steps": 0,
                    "ready_to_run_steps": 0,
                    "waiting_external_input_steps": 3,
                    "blocked_steps": 0,
                },
                "industrial_manifest": {
                    "status": "ready",
                    "summary": "industrial manifest ready",
                },
                "industrial_release_readiness": {
                    "status": "ready",
                    "summary": "industrial readiness ready",
                },
                "industrial_promotion_checklist": {
                    "status": "ready",
                    "summary": "industrial promotion ready",
                },
                "industrial_customer_acceptance_bundle": {
                    "status": "ready",
                    "summary": "industrial bundle ready",
                },
                "extension_execution_plan": {
                    "status": "ready",
                    "summary": "plan ready",
                },
                "extension_execution_evidence": {
                    "status": "ready",
                    "summary": "evidence ready",
                },
                "extension_execution_instance": {
                    "status": "ready",
                    "summary": "instance ready",
                },
                "extension_execution_schedule": {
                    "status": "ready",
                    "summary": "schedule ready",
                },
                "extension_execution_actuals": {
                    "status": "ready",
                    "summary": "actuals ready",
                },
                "industrial_delivery_artifact_paths": [
                    {
                        "name": "release_manifest_industrial.json",
                        "path": str(manifest_path),
                    }
                ],
                "delivery_rehearsal_stages": [
                    {
                        "id": "new_environment_install",
                        "status": "pass",
                        "summary": "new environment install passed",
                        "artifact_paths": ["artifacts/new_environment_install.json"],
                    },
                    {
                        "id": "smoke",
                        "status": "pass",
                        "summary": "smoke passed",
                        "artifact_paths": ["artifacts/smoke.json"],
                    },
                    {
                        "id": "live_evidence",
                        "status": "pass",
                        "summary": "live evidence passed",
                        "artifact_paths": ["artifacts/live_evidence.json"],
                    },
                    {
                        "id": "upgrade",
                        "status": "pass",
                        "summary": "upgrade passed",
                        "artifact_paths": ["artifacts/upgrade.json"],
                    },
                    {
                        "id": "rollback",
                        "status": "pass",
                        "summary": "rollback passed",
                        "artifact_paths": ["artifacts/rollback.json"],
                    },
                    {
                        "id": "backup_restore",
                        "status": "pass",
                        "summary": "backup restore passed",
                        "artifact_paths": ["artifacts/backup_restore.json"],
                    },
                ],
            },
            release_rehearsal_report_path=project_root
            / "release_rehearsal_report.json",
        ),
        industrial_delivery_rehearsal_report_path,
    )

    bundle = build_customer_acceptance_bundle_artifact(
        release_manifest=manifest_payload,
        manifest_path=manifest_path,
        project_root=project_root,
    )

    assert validate_customer_acceptance_bundle_artifact(bundle) == []
    reports_by_name = {item["name"]: item for item in bundle["acceptance_reports"]}
    assert bundle["extension_support_surface"]["status"] == "ready"
    assert bundle["extension_support_surface"]["declared_profiles"] == 4
    assert bundle["extension_execution_plan"]["status"] == "ready"
    assert bundle["extension_execution_plan"]["actionable_profiles"] == 3
    assert bundle["extension_execution_evidence"]["status"] == "ready"
    assert bundle["extension_execution_evidence"]["ready_reports"] == 4
    assert bundle["extension_execution_instance"]["status"] == "ready"
    assert bundle["extension_execution_instance"]["ready_profiles"] == 3
    assert bundle["extension_execution_schedule"]["status"] == "ready"
    assert bundle["extension_execution_schedule"]["ready_profiles"] == 3
    assert bundle["extension_execution_actuals"]["status"] == "ready"
    assert bundle["extension_execution_actuals"]["ready_profiles"] == 3
    ros2_profile = next(
        item
        for item in bundle["extension_execution_plan"]["profiles"]
        if item["id"] == "ros2_bridge_extension"
    )
    assert "docs/ros2/ROS2_QUICK_START.md" in ros2_profile["runbook_entrypoints"]
    assert ros2_profile["execution_template"]["rollback_steps"][0]["owner_role"] == "customer_operator"
    assert ros2_profile["execution_template"]["handoff_owner_role"] == "delivery_lead"
    assert ros2_profile["execution_template"]["watch_owner_role"] == "customer_operator"
    assert ros2_profile["deployment_commands"][1].startswith("ros2 launch")
    assert "AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1" in ros2_profile["acceptance_checks"][2]
    assert "industrial_release_readiness" in reports_by_name
    assert "industrial_promotion_checklist" in reports_by_name
    assert "industrial_delivery_rehearsal_report" in reports_by_name
    assert "customer_external_bindings_closure" in reports_by_name
    assert "external_mainline_execution_plan" in reports_by_name
    assert "customer_external_bindings_confirmation" in reports_by_name
    assert "vulnerability_exception_review" in reports_by_name
    assert "extension_on_call_rehearsal" in reports_by_name
    assert "extension_exception_review_schedule" in reports_by_name
    assert "extension_escalation_closure" in reports_by_name
    assert "release_readiness" not in reports_by_name
    assert "stable_promotion_checklist" not in reports_by_name
    assert reports_by_name["industrial_release_readiness"]["exists"] is True
    assert reports_by_name["industrial_release_readiness"]["status"] == "ready"
    assert reports_by_name["industrial_promotion_checklist"]["exists"] is True
    assert reports_by_name["industrial_promotion_checklist"]["status"] == "ready"
    assert reports_by_name["industrial_delivery_rehearsal_report"]["exists"] is True
    assert reports_by_name["industrial_delivery_rehearsal_report"]["status"] == "ready"
    assert reports_by_name["industrial_delivery_rehearsal_report"]["stage_summary"] == {
        "total": 6,
        "passed": 6,
        "failed": 0,
    }
    assert reports_by_name["customer_external_bindings_closure"]["status"] == "passed"
    assert reports_by_name["external_mainline_execution_plan"]["status"] == "ready"
    assert reports_by_name["customer_external_bindings_confirmation"]["status"] == "passed"
    assert reports_by_name["vulnerability_exception_review"]["status"] == "passed"
    assert bundle["vulnerability_exception_review"]["status"] == "passed"
    assert bundle["external_mainline_execution_plan"]["status"] == "ready"
    assert (
        bundle["external_mainline_execution_plan"]["ready_to_run_steps"]
        == reports_by_name["external_mainline_execution_plan"]["ready_to_run_steps"]
    )
    assert "external_mainline=ready/1/2/0/0." in bundle["summary"]
    assert (
        "python tools/build_industrial_delivery_rehearsal_report.py --rehearsal-report ..."
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/run_external_mainline_execution_plan.py --output "
        "test_env/release_evidence/operations/external_mainline_execution_plan.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_vulnerability_exception_review_report.py --output "
        "test_env/release_evidence/security/vulnerability_exception_review_report.json "
        "--exception-report test_env/release_evidence/security/vulnerability_exception_report.json"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json"
        " --section approval_identity --section archive_target --section due_trigger"
        " --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>"
        in bundle["recommended_commands"]
    )
    assert (
        "python tools/build_customer_external_bindings_confirmation_report.py --output "
        "test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json "
        "--actuals-artifact test_env/release_evidence/operations/extension_execution_actuals.json"
        in bundle["recommended_commands"]
    )
