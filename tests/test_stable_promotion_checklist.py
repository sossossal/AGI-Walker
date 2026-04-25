from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_external_mainline_execution_plan_artifact,
    build_extension_execution_actuals_artifact,
    build_extension_execution_instance_artifact,
    build_extension_execution_schedule_artifact,
    build_release_evidence_report,
    build_release_manifest_artifact,
    default_external_mainline_execution_plan_path,
    write_external_mainline_execution_plan_artifact,
    write_extension_execution_actuals_artifact,
    write_extension_execution_instance_artifact,
    write_extension_execution_schedule_artifact,
    write_release_evidence_report,
    write_release_manifest_artifact,
)
from tests.test_release_readiness import (
    _seed_customer_external_bindings_closure_report,
    _seed_external_mainline_input_checklist,
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


def _seed_clean_checkout_evidence(project_root: Path) -> None:
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
                "checkout_commit_sha": "stable-promotion-clean-checkout",
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
            source_commit_sha="stable-promotion",
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
            source_commit_sha="stable-promotion",
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
                "confirmation_tickets": ["CHG-STABLE-PROMOTION"],
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
                source_commit_sha="stable-promotion",
            ),
            operations_root / file_name,
        )
    instance_path = operations_root / "extension_execution_instance.json"
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            generated_at="2026-04-15T10:04:00+00:00",
            engagement_id="stable-promotion-canonical",
            customer_name="AGI-Walker Customer",
            site_name="primary-site",
            change_ticket="CHG-STABLE-PROMOTION",
            window_id="window-stable-promotion",
            window_start_at="2026-04-15T10:30:00+00:00",
            window_end_at="2026-04-15T12:30:00+00:00",
            delivery_root="test_env/release_delivery/stable_promotion",
            closure_archive_root="test_env/release_delivery/stable_promotion/closure_archive",
            exception_review_due_at="2026-05-15T00:00:00+00:00",
        ),
        instance_path,
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
            "project_version": "2026.04.15-stable",
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
            "actor": "stable-promotion",
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


def _seed_security_preflight_report(
    project_root: Path,
    *,
    status: str = "passed",
    summary: str | None = None,
    metrics: dict[str, object] | None = None,
) -> Path:
    report_path = (
        project_root / "test_env" / "release_evidence" / "security_release_preflight_report.json"
    )
    payload = build_release_evidence_report(
        evidence_name="security_release_preflight",
        status=status,
        summary=summary
        or (
            "security posture is ready."
            if status == "passed"
            else "security posture remains blocked."
        ),
        command=(
            "python tools/run_security_release_preflight.py "
            "--output-root test_env/release_evidence "
            "--report-file test_env/release_evidence/security_release_preflight_report.json"
        ),
        metrics=metrics,
    )
    return write_release_evidence_report(payload, report_path)


def _write_ready_stable_manifest(project_root: Path, source_root: Path) -> Path:
    manifest_path = project_root / "test_env" / "release" / "release_manifest_stable.json"
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    payload = build_release_manifest_artifact(
        build_id="build-20260412-stable",
        version="2026.04.12",
        channel="stable",
        release_summary="stable signoff",
        generated_at="2026-04-12T12:31:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "notes": "stable signoff",
        },
        project_root=project_root,
        source_root=source_root,
    )
    assert payload["release_gate_status"] == "ready"
    return write_release_manifest_artifact(payload, manifest_path)


def _seed_external_mainline_execution_plan(project_root: Path) -> Path:
    output_path = project_root / default_external_mainline_execution_plan_path()
    payload = build_external_mainline_execution_plan_artifact(project_root=project_root)
    return write_external_mainline_execution_plan_artifact(payload, output_path)


def test_stable_promotion_checklist_reports_pending_prerequisites(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, head = _init_git_repo(tmp_path)
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
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
    assert "stable_extension_execution_actuals=ready" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "stable_promotion_checklist"
    assert payload["stable_release_gate"] == "blocked"
    assert payload["current_head_commit"] == head
    assert payload["ready_to_promote"] is False

    step_ids = {step["id"]: step for step in payload["steps"]}
    assert payload["customer_delivery_surface"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
    assert step_ids["stable_approval"]["status"] == "pending"
    assert step_ids["customer_delivery_surface"]["status"] == "done"
    assert step_ids["industrial_delivery_gate"]["status"] == "done"
    assert step_ids["extension_execution_actuals"]["status"] == "done"
    assert step_ids["extension_execution_actuals"]["blocking"] is False
    assert step_ids["git_source_binding"]["status"] == "pending"
    assert step_ids["clean_worktree"]["status"] == "done"
    assert step_ids["version_tag"]["status"] == "pending"
    assert step_ids["build_stable_manifest"]["ready_to_run"] is False
    assert "git tag 2026.04.12" in step_ids["version_tag"]["command"]


def test_stable_promotion_checklist_reports_ready_to_promote(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
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
    assert "stable_extension_execution_actuals=ready" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["stable_release_gate"] == "ready"
    assert payload["ready_to_promote"] is True
    assert payload["current_head_commit"] == head
    assert payload["matched_version_tag"] == "2026.04.12"
    assert payload["customer_delivery_surface"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3

    step_ids = {step["id"]: step for step in payload["steps"]}
    assert step_ids["customer_delivery_surface"]["status"] == "done"
    assert step_ids["industrial_delivery_gate"]["status"] == "done"
    assert step_ids["extension_execution_actuals"]["status"] == "done"
    assert step_ids["stable_approval"]["status"] == "done"
    assert step_ids["git_source_binding"]["status"] == "done"
    assert step_ids["clean_worktree"]["status"] == "done"
    assert step_ids["version_tag"]["status"] == "done"
    assert step_ids["build_stable_manifest"]["ready_to_run"] is True
    assert "--channel stable" in step_ids["build_stable_manifest"]["command"]
    assert "客户窗口执行留痕" in step_ids["extension_execution_actuals"]["title"]


def test_stable_promotion_checklist_surfaces_confirm_action_for_existing_customer_bindings(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    operations_root = project_root / "test_env" / "release_evidence" / "operations"
    write_extension_execution_actuals_artifact(
        build_extension_execution_actuals_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_actuals.json",
            schedule_artifact_path="test_env/release_evidence/operations/extension_execution_schedule.json",
            generated_at="2026-04-15T10:05:00+00:00",
            external_bindings={
                "config_path": "deployment/customer_delivery.external_bindings.customer.json",
                "approval_identity": {
                    "source_type": "customer_ticket_registry",
                    "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
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
        ),
        operations_root / "extension_execution_actuals.json",
    )
    _seed_customer_external_bindings_closure_report(
        project_root,
        failed_steps=["customer_external_bindings_overrides_missing"],
    )
    report_path = (
        project_root
        / "test_env"
        / "stable_promotion"
        / "checklist_external_bindings.json"
    )

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
        check=False,
    )

    assert result.returncode == 0, result.stderr
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    step_ids = {step["id"]: step for step in payload["steps"]}
    bindings_step = step_ids["extension_external_bindings"]
    assert bindings_step["status"] == "pending"
    assert bindings_step["blocking"] is False
    assert bindings_step["ready_to_run"] is True
    assert (
        bindings_step["command"]
        == "python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>"
    )
    assert "run_customer_external_bindings_closure.py" in bindings_step["summary"]
    assert "existing config" in bindings_step["summary"]
    assert "customer_external_bindings_overrides_missing" in bindings_step["summary"]
    assert "customer_external_bindings_closure_report.json" in bindings_step["summary"]


def test_stable_promotion_checklist_surfaces_non_blocking_exception_review_step(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(
        project_root,
        metrics={
            "vulnerability_exception_review_status": "review_due",
            "vulnerability_exception_review_due": 31,
            "vulnerability_exception_next_expiry": "2026-05-15T00:00:00+00:00",
            "vulnerability_exception_review_report_status": "passed",
            "vulnerability_exception_review_candidate_count": 31,
            "vulnerability_exception_review_report_path": "test_env/release_evidence/security/vulnerability_exception_review_report.json",
            "review_due_vulnerability_exception_ids": [
                "webpanel-distributed-libsystemd0-no-fix",
                "webpanel-distributed-libudev1-no-fix",
            ],
            "review_due_vulnerability_exception_tickets": [
                "SEC-2026-04-15-WEBPANEL-DIST-NOFIX"
            ],
            "expired_vulnerability_exceptions": 0,
        },
    )
    _seed_industrial_security_reports(project_root)
    report_path = project_root / "test_env" / "stable_promotion" / "checklist_review_due.json"

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
    assert payload["ready_to_promote"] is True
    steps = {step["id"]: step for step in payload["steps"]}
    review_step = steps["vulnerability_exception_review"]
    assert review_step["status"] == "pending"
    assert review_step["blocking"] is False
    assert review_step["ready_to_run"] is True
    assert "2026-05-15T00:00:00+00:00" in review_step["summary"]
    assert "review_report=passed/31" in review_step["summary"]
    assert "SEC-2026-04-15-WEBPANEL-DIST-NOFIX" in review_step["summary"]
    assert "build_vulnerability_exception_review_report.py" in review_step["command"]


def test_stable_promotion_checklist_surfaces_external_mainline_step(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    _seed_external_mainline_execution_plan(project_root)
    checklist_report = _seed_external_mainline_input_checklist(project_root)
    report_path = (
        project_root
        / "test_env"
        / "stable_promotion"
        / "checklist_external_mainline.json"
    )

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
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "stable_external_mainline_execution_plan=ready/0/2/1/0" in result.stdout
    checklist_payload = json.loads(checklist_report.read_text(encoding="utf-8"))
    assert (
        "stable_external_mainline_input_checklist="
        f"{checklist_payload['status']}"
        in result.stdout
    )

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["external_mainline_execution_plan"]["status"] == "ready"
    assert payload["external_mainline_execution_plan"]["ready_to_run_steps"] == 2
    assert payload["external_mainline_execution_plan"]["waiting_external_input_steps"] == 1
    assert (
        payload["external_mainline_input_checklist"]["status"]
        == checklist_payload["status"]
    )
    assert "external_mainline=ready/0/2/1/0" in payload["summary"]
    assert "external_mainline_input_checklist=" in payload["summary"]
    assert (
        "刷新剩余外部主线计划并自动执行可安全部分"
        in payload["next_step_plan"]
    )
    assert "刷新剩余外部主线输入缺口清单" in payload["next_step_plan"]
    steps = {step["id"]: step for step in payload["steps"]}
    external_mainline_step = steps["external_mainline_execution_plan"]
    assert external_mainline_step["status"] == "pending"
    assert external_mainline_step["blocking"] is False
    assert external_mainline_step["ready_to_run"] is True
    assert "run_external_mainline_execution_plan.py" in external_mainline_step["command"]
    assert "ready_to_run_steps=2" in external_mainline_step["summary"]
    assert "waiting_external_input_steps=1" in external_mainline_step["summary"]
    checklist_step = steps["external_mainline_input_checklist"]
    assert checklist_step["status"] == "pending"
    assert checklist_step["blocking"] is False
    assert checklist_step["ready_to_run"] is True
    assert "build_external_mainline_input_checklist.py" in checklist_step["command"]


def test_stable_promotion_checklist_surfaces_stale_exception_step(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(
        project_root,
        status="blocked",
        summary=(
            "Security posture remains blocked because some active no-fix vulnerability "
            "exceptions are stale and the matching findings now advertise fix versions."
        ),
        metrics={
            "stale_vulnerability_exceptions": 2,
            "stale_vulnerability_exception_ids": [
                "webpanel-distributed-libsystemd0-no-fix",
                "webpanel-distributed-libudev1-no-fix",
            ],
            "vulnerability_exception_review_report_status": "passed",
            "vulnerability_exception_review_candidate_count": 2,
            "vulnerability_exception_review_report_path": "test_env/release_evidence/security/vulnerability_exception_review_report.json",
        },
    )
    _seed_industrial_security_reports(project_root)
    report_path = project_root / "test_env" / "stable_promotion" / "checklist_stale.json"

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
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    steps = {step["id"]: step for step in payload["steps"]}
    review_step = steps["vulnerability_exception_review"]
    assert review_step["status"] == "pending"
    assert review_step["blocking"] is False
    assert review_step["ready_to_run"] is True
    assert review_step["depends_on"] == []
    assert "2 条 active no-fix exception 已失效" in review_step["summary"]
    assert "webpanel-distributed-libsystemd0-no-fix" in review_step["summary"]
    assert "review_report=passed/2" in review_step["summary"]
    assert "build_vulnerability_exception_review_report.py" in review_step["command"]


def test_stable_promotion_checklist_blocks_on_dirty_worktree(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _head = _init_git_repo(tmp_path, tag="2026.04.12")
    (source_root / "README.md").write_text("# dirty repo\n", encoding="utf-8")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
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
    assert "stable_worktree_release_blocker=blocked" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    step_ids = {step["id"]: step for step in payload["steps"]}
    assert payload["ready_to_promote"] is False
    assert payload["worktree_release_blocker"]["status"] == "blocked"
    assert step_ids["stable_approval"]["status"] == "done"
    assert step_ids["git_source_binding"]["status"] == "done"
    assert step_ids["industrial_delivery_gate"]["status"] == "done"
    assert step_ids["clean_worktree"]["status"] == "pending"
    assert "tools/run_worktree_release_blocker.py" in step_ids["clean_worktree"]["command"]
    assert "worktree release blocker" in step_ids["clean_worktree"]["summary"]
    assert step_ids["version_tag"]["status"] == "done"
    assert step_ids["build_stable_manifest"]["ready_to_run"] is False


def test_stable_promotion_checklist_accepts_approval_manifest(tmp_path: Path) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    approval_manifest = _write_ready_stable_manifest(project_root, source_root)
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
            "--approval-manifest",
            str(approval_manifest),
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
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["ready_to_promote"] is True
    assert payload["approval_manifest_path"] == str(approval_manifest)
    assert payload["current_head_commit"] == head
    assert payload["customer_delivery_surface"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["status"] == "ready"

    step_ids = {step["id"]: step for step in payload["steps"]}
    assert step_ids["customer_delivery_surface"]["status"] == "done"
    assert step_ids["industrial_delivery_gate"]["status"] == "done"
    assert step_ids["build_stable_manifest"]["status"] == "done"
    assert "ready stable manifest" in step_ids["build_stable_manifest"]["summary"]
    assert step_ids["build_stable_manifest"]["command"] is None


def test_stable_promotion_checklist_blocks_when_security_preflight_is_blocked(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(
        project_root,
        status="blocked",
        summary="security posture remains blocked.",
    )
    _seed_industrial_security_reports(project_root)
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
    assert "stable_promotion_blocking_steps=1" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["stable_release_gate"] == "ready"
    assert payload["ready_to_promote"] is False
    assert payload["current_head_commit"] == head

    step_ids = {step["id"]: step for step in payload["steps"]}
    assert step_ids["customer_delivery_surface"]["status"] == "done"
    assert step_ids["industrial_delivery_gate"]["status"] == "done"
    assert step_ids["security_release_preflight"]["status"] == "pending"
    assert step_ids["security_release_preflight"]["blocking"] is True
    assert "run_security_release_preflight.py" in step_ids["security_release_preflight"]["command"]
    assert step_ids["build_stable_manifest"]["ready_to_run"] is False


def test_stable_promotion_checklist_surfaces_customer_delivery_step_without_blocking(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root, head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
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
    assert payload["ready_to_promote"] is True
    assert payload["current_head_commit"] == head
    assert payload["customer_delivery_surface"]["status"] == "blocked"
    assert payload["industrial_delivery_gate"]["status"] == "blocked"
    assert "确认 Phase E 客户交付文档集齐备" in payload["next_step_plan"]
    assert "确认 industrial delivery gate 已闭合" in payload["next_step_plan"]

    step_ids = {step["id"]: step for step in payload["steps"]}
    assert step_ids["customer_delivery_surface"]["status"] == "pending"
    assert step_ids["customer_delivery_surface"]["blocking"] is False
    assert step_ids["industrial_delivery_gate"]["status"] == "pending"
    assert step_ids["industrial_delivery_gate"]["blocking"] is False
    assert step_ids["build_stable_manifest"]["ready_to_run"] is True
