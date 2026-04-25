from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_external_mainline_execution_plan_artifact,
    build_industrial_delivery_rehearsal_report_artifact,
    build_release_evidence_report,
    validate_external_mainline_execution_plan_artifact,
    write_industrial_delivery_rehearsal_report_artifact,
    write_release_evidence_report,
)
from agi_walker.core.api.security_posture_contracts import (
    build_vulnerability_exception_report,
    write_vulnerability_exception_report,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _stage(stage_id: str, *, status: str = "pass") -> dict[str, object]:
    return {
        "id": stage_id,
        "status": status,
        "summary": f"{stage_id} {status}",
        "artifact_paths": [f"artifacts/{stage_id}.json"],
    }


def _ready_release_rehearsal_report() -> dict[str, object]:
    return {
        "status": "passed",
        "version": "2026.04.17-industrial-rehearsal",
        "tag": "2026.04.17-industrial-rehearsal",
        "source_root": "D:/tmp/rehearsal/git_source",
        "git_source": {"commit_sha": "abc123def456"},
        "release_gate_status": "ready",
        "customer_delivery_status": "ready",
        "industrial_delivery_status": "ready",
        "security_release_preflight": {
            "status": "passed",
            "summary": "security preflight passed",
            "report_path": "test_env/release_evidence/security_release_preflight_report.json",
            "metrics": {
                "vulnerability_exception_review_report_status": "passed",
                "vulnerability_exception_review_candidate_count": 31,
                "vulnerability_exception_review_report_path": (
                    "test_env/release_evidence/security/"
                    "vulnerability_exception_review_report.json"
                ),
            },
        },
        "vulnerability_exception_review": {
            "status": "passed",
            "summary": "vulnerability exception review status=passed, candidates=31.",
            "review_candidate_count": 31,
            "report_path": (
                "test_env/release_evidence/security/"
                "vulnerability_exception_review_report.json"
            ),
        },
        "customer_external_bindings_closure": {
            "status": "passed",
            "summary": "customer external bindings closure passed.",
            "report_path": (
                "test_env/release_evidence/operations/"
                "customer_external_bindings_closure_report.json"
            ),
        },
        "external_mainline_execution_plan": {
            "status": "ready",
            "summary": (
                "External mainline execution plan ready: completed=1, "
                "ready_to_run=1, waiting_external_input=1, blocked=0, "
                "auto_executable=3."
            ),
            "report_path": (
                "test_env/release_evidence/operations/"
                "external_mainline_execution_plan.json"
            ),
            "completed_steps": 1,
            "ready_to_run_steps": 1,
            "waiting_external_input_steps": 1,
            "blocked_steps": 0,
        },
        "external_mainline_input_checklist": {
            "status": "blocked",
            "summary": (
                "external_mainline_input_checklist blocked: customer_missing=1, "
                "vulnerability_missing=3, industrial_missing=0, waiting_steps=1, "
                "ready_steps=1, completed_steps=1."
            ),
            "report_path": (
                "test_env/release_evidence/operations/"
                "external_mainline_input_checklist_report.json"
            ),
            "missing_input_count": 4,
            "waiting_external_input_steps": [
                "vulnerability_exception_replacement"
            ],
            "ready_to_run_steps": [
                "industrial_delivery_live_evidence"
            ],
            "completed_steps": [
                "customer_external_bindings_closure"
            ],
        },
        "release_ops_execution": {
            "status": "passed",
            "summary": (
                "release op external_mainline_execution completed via rehearsal "
                "control plane evidence wrapper."
            ),
            "report_path": (
                "test_env/release_evidence/operations/"
                "release_ops_execution_report.json"
            ),
            "event_count": 3,
            "action": "external_mainline_execution",
            "policy_level": "requires_attestation",
            "policy_profile": "requires_attestation",
            "request_type": "ExternalMainlineExecutionRequest",
        },
        "industrial_manifest": {
            "status": "ready",
            "summary": "industrial manifest ready",
            "manifest_path": "release_manifest_industrial.json",
        },
        "industrial_release_readiness": {
            "status": "ready",
            "summary": "industrial readiness ready",
            "report_path": "test_env/industrial_release_readiness_ready/industrial_release_readiness_report.json",
        },
        "industrial_promotion_checklist": {
            "status": "ready",
            "summary": "industrial promotion ready",
            "blocking_steps": 0,
            "report_path": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
        },
        "industrial_customer_acceptance_bundle": {
            "status": "ready",
            "summary": "industrial customer acceptance bundle ready",
            "bundle_path": "test_env/release/customer_acceptance_bundle_industrial.json",
        },
        "extension_execution_plan": {
            "status": "ready",
            "summary": "extension execution plan ready",
        },
        "extension_execution_evidence": {
            "status": "ready",
            "summary": "extension execution evidence ready",
        },
        "extension_execution_instance": {
            "status": "ready",
            "summary": "extension execution instance ready",
        },
        "extension_execution_schedule": {
            "status": "ready",
            "summary": "extension execution schedule ready",
        },
        "extension_execution_actuals": {
            "status": "ready",
            "summary": "extension execution actuals ready",
        },
        "industrial_delivery_artifact_paths": [
            {
                "name": "release_manifest_industrial.json",
                "path": "release_manifest_industrial.json",
            }
        ],
        "delivery_rehearsal_stages": [
            _stage("new_environment_install"),
            _stage("smoke"),
            _stage("live_evidence"),
            _stage("upgrade"),
            _stage("rollback"),
            _stage("backup_restore"),
        ],
        "control_plane_session": {
            "engagement_id": "release-rehearsal-industrial",
            "window_id": "2026.04.17-industrial-rehearsal-external-mainline-release-ops",
            "change_ticket": "release-rehearsal-industrial-change",
            "channel": "industrial",
        },
        "control_plane_event_stream": {
            "path": "test_env/release_ops/release_rehearsal_external_mainline.jsonl",
            "event_count": 3,
        },
    }


def _write_industrial_report(project_root: Path) -> Path:
    report_path = (
        project_root
        / "test_env"
        / "release_rehearsal_industrial"
        / "industrial_delivery_rehearsal_report.json"
    )
    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=_ready_release_rehearsal_report(),
        release_rehearsal_report_path=(
            "test_env/release_rehearsal_industrial/release_rehearsal_report.json"
        ),
    )
    return write_industrial_delivery_rehearsal_report_artifact(payload, report_path)


def _write_customer_config(project_root: Path, *, confirmed: bool) -> Path:
    config_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
    )
    config_path.parent.mkdir(parents=True, exist_ok=True)
    state = "confirmed" if confirmed else "draft"
    section = {
        "binding_state": state,
        "system_name": "Customer System",
    }
    if confirmed:
        section.update(
            {
                "confirmed_by": "delivery_lead",
                "confirmed_at": "2026-04-18T11:00:00+00:00",
                "confirmation_ticket": "CHG-001",
            }
        )
    payload = {
        "approval_identity": dict(section),
        "archive_target": dict(section),
        "due_trigger": dict(section),
    }
    config_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return config_path


def test_build_external_mainline_execution_plan_tracks_waiting_external_inputs(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_customer_config(project_root, confirmed=False)
    closure_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_closure_report.json"
    )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="customer_external_bindings_closure",
            status="blocked",
            summary="customer external bindings closure blocked: failed_steps=customer_external_bindings_overrides_missing.",
            command=(
                "python tools/run_customer_external_bindings_closure.py --config "
                "deployment/customer_delivery.external_bindings.customer.json"
            ),
            metrics={
                "failed_steps": ["customer_external_bindings_overrides_missing"],
            },
        ),
        closure_report_path,
    )
    review_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_review_report.json"
    )
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="passed",
            summary="vulnerability exception review evidence passed: 31 active exception(s) require review inside the 30-day window before 2026-05-15T00:00:00+00:00.",
            command="python tools/build_vulnerability_exception_review_report.py",
            metrics={
                "review_candidate_count": 31,
                "review_follow_up_required": True,
                "next_exception_expiry": "2026-05-15T00:00:00+00:00",
            },
        ),
        review_report_path,
    )
    _write_industrial_report(project_root)

    payload = build_external_mainline_execution_plan_artifact(project_root=project_root)

    assert payload["artifact_type"] == "external_mainline_execution_plan"
    assert payload["status"] == "ready"
    assert payload["completed_steps"] == 0
    assert payload["ready_to_run_steps"] == 0
    assert payload["waiting_external_input_steps"] == 3
    assert payload["blocked_steps"] == 0
    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["customer_external_bindings_closure"]["status"] == "waiting_external_input"
    assert "customer_external_bindings_overrides_missing" in steps["customer_external_bindings_closure"]["blocking_inputs"]
    assert steps["customer_external_bindings_closure"]["source_report_path"] == (
        "test_env/release_evidence/operations/customer_external_bindings_closure_report.json"
    )
    assert steps["customer_external_bindings_closure"]["artifact_paths"][0] == (
        "deployment/customer_delivery.external_bindings.customer.json"
    )
    assert steps["vulnerability_exception_replacement"]["status"] == "waiting_external_input"
    assert (
        "在 2026-05-15T00:00:00+00:00 前完成 replacement / review"
        in steps["vulnerability_exception_replacement"]["blocking_inputs"]
    )
    assert steps["vulnerability_exception_replacement"]["source_report_path"] == (
        "test_env/release_evidence/security/vulnerability_exception_review_report.json"
    )
    assert steps["industrial_delivery_live_evidence"]["status"] == "waiting_external_input"
    assert steps["industrial_delivery_live_evidence"]["source_report_path"] == (
        "test_env/release_rehearsal_industrial/industrial_delivery_rehearsal_report.json"
    )
    assert validate_external_mainline_execution_plan_artifact(payload) == []


def test_build_external_mainline_execution_plan_marks_confirmed_customer_config_ready_to_run(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_customer_config(project_root, confirmed=True)

    payload = build_external_mainline_execution_plan_artifact(project_root=project_root)

    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["customer_external_bindings_closure"]["status"] == "ready_to_run"
    assert steps["vulnerability_exception_replacement"]["status"] == "ready_to_run"
    assert steps["industrial_delivery_live_evidence"]["status"] == "ready_to_run"
    assert payload["ready_to_run_steps"] == 3
    assert validate_external_mainline_execution_plan_artifact(payload) == []


def test_build_external_mainline_execution_plan_uses_project_relative_missing_closure_path(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_customer_config(project_root, confirmed=False)

    payload = build_external_mainline_execution_plan_artifact(project_root=project_root)

    steps = {item["id"]: item for item in payload["steps"]}
    customer_summary = steps["customer_external_bindings_closure"]["summary"]
    assert steps["customer_external_bindings_closure"]["status"] == "waiting_external_input"
    assert (
        customer_summary
        == "customer_external_bindings_closure report is missing: "
        "test_env/release_evidence/operations/customer_external_bindings_closure_report.json"
    )
    assert str(project_root) not in customer_summary
    assert validate_external_mainline_execution_plan_artifact(payload) == []


def test_build_external_mainline_execution_plan_marks_industrial_live_evidence_ready_when_inputs_complete(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_customer_config(project_root, confirmed=False)
    _write_industrial_report(project_root)

    payload = build_external_mainline_execution_plan_artifact(
        project_root=project_root,
        industrial_live_evidence_inputs={
            "enabled": True,
            "target_environment": "customer-prod-eu-west-1",
            "access_method": "VPN + Bastion",
            "install_entrypoint": "docs/runbooks/install.md",
            "upgrade_entrypoint": "docs/runbooks/upgrade.md",
            "rollback_entrypoint": "docs/runbooks/rollback.md",
            "backup_restore_entrypoint": "docs/runbooks/backup-restore.md",
            "closure_archive_root": "archive://customer-prod/release-window-001",
            "evidence_output_root": "test_env/industrial_live_evidence/customer-prod",
        },
    )

    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["industrial_delivery_live_evidence"]["status"] == "ready_to_run"
    assert steps["industrial_delivery_live_evidence"]["managed_inputs_ready"] is True
    assert steps["industrial_delivery_live_evidence"]["blocking_inputs"] == []
    assert (
        steps["industrial_delivery_live_evidence"]["managed_inputs"]["target_environment"]
        == "customer-prod-eu-west-1"
    )
    assert validate_external_mainline_execution_plan_artifact(payload) == []


def test_run_external_mainline_execution_plan_refreshes_review_and_writes_plan(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    exception_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json"
    )
    exception_report_path.parent.mkdir(parents=True, exist_ok=True)
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-18T12:00:00+00:00",
            exceptions=[
                {
                    "id": "webpanel-libsystemd0-no-fix",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "only_without_fix_version": True,
                    "justification": "Temporary exception while upstream fix is unavailable.",
                    "approved_by": "security-reviewer",
                    "approved_at": "2026-04-15T11:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                    "ticket": "SEC-301",
                }
            ],
        ),
        exception_report_path,
    )
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_external_mainline_execution_plan.py",
            "--project-root",
            str(project_root),
            "--output",
            str(output_path),
            "--skip-customer-external-bindings-closure",
            "--industrial-delivery-rehearsal-report",
            "test_env/release_rehearsal_industrial/industrial_delivery_rehearsal_report.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "external_mainline_input_checklist_report_written=" in result.stdout
    assert "external_mainline_input_checklist_status=blocked" in result.stdout
    assert "external_mainline_execution_plan_inputs_file=" in result.stdout
    assert (
        "external_mainline_execution_plan_industrial_live_evidence_inputs_ready=false"
        in result.stdout
    )
    assert "external_mainline_execution_plan_status=ready" in result.stdout
    assert "external_mainline_execution_plan_failures=0" in result.stdout
    assert (
        "external_mainline_execution_plan_executed_steps=vulnerability_exception_review_refresh"
        in result.stdout
    )
    assert (
        "external_mainline_step_vulnerability_exception_replacement=waiting_external_input"
        in result.stdout
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["vulnerability_exception_replacement"]["status"] == "waiting_external_input"
    assert steps["customer_external_bindings_closure"]["status"] == "waiting_external_input"


def test_run_external_mainline_execution_plan_skip_managed_inputs_preserves_legacy_mode(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    exception_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json"
    )
    exception_report_path.parent.mkdir(parents=True, exist_ok=True)
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-18T12:00:00+00:00",
            exceptions=[
                {
                    "id": "webpanel-libsystemd0-no-fix",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "only_without_fix_version": True,
                    "justification": "Temporary exception while upstream fix is unavailable.",
                    "approved_by": "security-reviewer",
                    "approved_at": "2026-04-15T11:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                    "ticket": "SEC-301",
                }
            ],
        ),
        exception_report_path,
    )
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_external_mainline_execution_plan.py",
            "--project-root",
            str(project_root),
            "--output",
            str(output_path),
            "--skip-managed-inputs",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "external_mainline_input_checklist_report_written=" in result.stdout
    assert "external_mainline_execution_plan_inputs_file=" not in result.stdout
    assert not (project_root / "deployment" / "external_mainline.inputs.json").exists()
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["customer_external_bindings_closure"]["status"] == "waiting_external_input"


def test_run_external_mainline_execution_plan_supports_inputs_file(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    exception_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json"
    )
    exception_report_path.parent.mkdir(parents=True, exist_ok=True)
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-18T12:00:00+00:00",
            exceptions=[
                {
                    "id": "webpanel-libsystemd0-no-fix",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "only_without_fix_version": True,
                    "justification": "Temporary exception while upstream fix is unavailable.",
                    "approved_by": "security-reviewer",
                    "approved_at": "2026-04-15T11:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                    "ticket": "SEC-301",
                }
            ],
        ),
        exception_report_path,
    )
    overrides_path = (
        project_root
        / "deployment"
        / "customer_delivery.external_bindings.customer.overrides.json"
    )
    overrides_path.parent.mkdir(parents=True, exist_ok=True)
    overrides_path.write_text(
        json.dumps(
            {
                "approval_identity": {
                    "source_type": "customer_signoff_registry",
                    "reference": "signoff://customer-001/window-001",
                    "system_name": "Customer Signoff Registry",
                },
                "archive_target": {
                    "binding_type": "s3_archive_bucket",
                    "binding_reference_base": "s3://customer-001/window-001",
                    "system_name": "Customer Archive Bucket",
                },
                "due_trigger": {
                    "binding_type": "customer_scheduler_job",
                    "binding_reference_base": "scheduler://customer-001/window-001",
                    "system_name": "Customer Scheduler",
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    inputs_path = project_root / "deployment" / "external_mainline.inputs.json"
    inputs_path.write_text(
        json.dumps(
            {
                "customer_external_bindings": {
                    "enabled": True,
                    "overrides_file": "deployment/customer_delivery.external_bindings.customer.overrides.json",
                    "confirmed_by": "release-manager",
                    "confirmation_ticket": "CHG-CUSTOMER-001",
                    "skip_collect_release_evidence": True,
                },
                "vulnerability_exception_review": {
                    "enabled": True,
                },
                "industrial_rehearsal": {
                    "refresh": False,
                    "report_path": "test_env/release_rehearsal_industrial/industrial_delivery_rehearsal_report.json",
                },
                "industrial_live_evidence": {
                    "enabled": True,
                    "target_environment": "customer-prod-eu-west-1",
                    "access_method": "VPN + Bastion",
                    "install_entrypoint": "docs/runbooks/install.md",
                    "upgrade_entrypoint": "docs/runbooks/upgrade.md",
                    "rollback_entrypoint": "docs/runbooks/rollback.md",
                    "backup_restore_entrypoint": "docs/runbooks/backup-restore.md",
                    "closure_archive_root": "archive://customer-prod/release-window-001",
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_external_mainline_execution_plan.py",
            "--project-root",
            str(project_root),
            "--output",
            str(output_path),
            "--inputs-file",
            "deployment/external_mainline.inputs.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "external_mainline_input_checklist_report_written=" in result.stdout
    assert "external_mainline_execution_plan_inputs_file=" in result.stdout
    assert (
        "external_mainline_execution_plan_executed_steps="
        "vulnerability_exception_review_refresh,customer_external_bindings_closure"
        in result.stdout
    )
    assert (
        "external_mainline_execution_plan_industrial_live_evidence_inputs_ready=true"
        in result.stdout
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["customer_external_bindings_closure"]["status"] == "completed"
    assert steps["vulnerability_exception_replacement"]["status"] == "waiting_external_input"
    assert steps["industrial_delivery_live_evidence"]["status"] == "ready_to_run"


def test_run_external_mainline_execution_plan_bootstraps_missing_inputs_file(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    exception_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json"
    )
    exception_report_path.parent.mkdir(parents=True, exist_ok=True)
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-18T12:00:00+00:00",
            exceptions=[
                {
                    "id": "webpanel-libsystemd0-no-fix",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "only_without_fix_version": True,
                    "justification": "Temporary exception while upstream fix is unavailable.",
                    "approved_by": "security-reviewer",
                    "approved_at": "2026-04-15T11:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                    "ticket": "SEC-301",
                }
            ],
        ),
        exception_report_path,
    )
    inputs_path = project_root / "deployment" / "external_mainline.inputs.json"
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_external_mainline_execution_plan.py",
            "--project-root",
            str(project_root),
            "--output",
            str(output_path),
            "--inputs-file",
            "deployment/external_mainline.inputs.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "external_mainline_input_checklist_report_written=" in result.stdout
    assert (
        "external_mainline_execution_plan_inputs_file_bootstrapped="
        in result.stdout
    )
    assert inputs_path.is_file()
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["customer_external_bindings_closure"]["status"] == "waiting_external_input"
    assert steps["vulnerability_exception_replacement"]["status"] == "waiting_external_input"


def test_run_external_mainline_execution_plan_refreshes_default_managed_inputs_file(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    exception_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json"
    )
    exception_report_path.parent.mkdir(parents=True, exist_ok=True)
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-18T12:00:00+00:00",
            exceptions=[
                {
                    "id": "webpanel-libsystemd0-no-fix",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "only_without_fix_version": True,
                    "justification": "Temporary exception while upstream fix is unavailable.",
                    "approved_by": "security-reviewer",
                    "approved_at": "2026-04-15T11:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                    "ticket": "SEC-301",
                }
            ],
        ),
        exception_report_path,
    )
    inputs_path = project_root / "deployment" / "external_mainline.inputs.json"
    inputs_path.parent.mkdir(parents=True, exist_ok=True)
    inputs_path.write_text("{}\n", encoding="utf-8")
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_external_mainline_execution_plan.py",
            "--project-root",
            str(project_root),
            "--output",
            str(output_path),
            "--inputs-file",
            "deployment/external_mainline.inputs.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "external_mainline_input_checklist_report_written=" in result.stdout
    assert (
        "external_mainline_execution_plan_inputs_file_refreshed="
        in result.stdout
    )
    refreshed_inputs = json.loads(inputs_path.read_text(encoding="utf-8"))
    assert "customer_external_bindings" in refreshed_inputs
    assert "industrial_rehearsal" in refreshed_inputs
    assert "industrial_live_evidence" in refreshed_inputs


def test_run_external_mainline_execution_plan_ignores_placeholder_confirmation_values(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    _write_customer_config(project_root, confirmed=False)
    exception_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json"
    )
    exception_report_path.parent.mkdir(parents=True, exist_ok=True)
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at="2026-04-18T12:00:00+00:00",
            exceptions=[
                {
                    "id": "webpanel-libsystemd0-no-fix",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "only_without_fix_version": True,
                    "justification": "Temporary exception while upstream fix is unavailable.",
                    "approved_by": "security-reviewer",
                    "approved_at": "2026-04-15T11:00:00+00:00",
                    "expires_at": "2026-05-15T00:00:00+00:00",
                    "ticket": "SEC-301",
                }
            ],
        ),
        exception_report_path,
    )
    overrides_path = (
        project_root
        / "deployment"
        / "customer_delivery.external_bindings.customer.overrides.json"
    )
    overrides_path.parent.mkdir(parents=True, exist_ok=True)
    overrides_path.write_text(
        json.dumps(
            {
                "approval_identity": {
                    "source_type": "customer_signoff_registry",
                    "reference": "signoff://customer-001/window-001",
                    "system_name": "Customer Signoff Registry",
                },
                "archive_target": {
                    "binding_type": "s3_archive_bucket",
                    "binding_reference_base": "s3://customer-001/window-001",
                    "system_name": "Customer Archive Bucket",
                },
                "due_trigger": {
                    "binding_type": "customer_scheduler_job",
                    "binding_reference_base": "scheduler://customer-001/window-001",
                    "system_name": "Customer Scheduler",
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    inputs_path = project_root / "deployment" / "external_mainline.inputs.json"
    inputs_path.write_text(
        json.dumps(
            {
                "customer_external_bindings": {
                    "enabled": True,
                    "config": "deployment/customer_delivery.external_bindings.customer.json",
                    "overrides_file": "deployment/customer_delivery.external_bindings.customer.overrides.json",
                    "confirmed_by": "<confirmed-by>",
                    "confirmation_ticket": "<confirmation-ticket>",
                },
                "vulnerability_exception_review": {
                    "enabled": True,
                },
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_external_mainline_execution_plan.py",
            "--project-root",
            str(project_root),
            "--output",
            str(output_path),
            "--inputs-file",
            "deployment/external_mainline.inputs.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "external_mainline_input_checklist_report_written=" in result.stdout
    assert (
        "external_mainline_execution_plan_executed_steps="
        "vulnerability_exception_review_refresh"
        in result.stdout
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["customer_external_bindings_closure"]["status"] == "waiting_external_input"


def test_build_external_mainline_inputs_derives_defaults_from_existing_artifacts(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    _write_customer_config(project_root, confirmed=True)
    overrides_path = (
        project_root
        / "deployment"
        / "customer_delivery.external_bindings.customer.overrides.json"
    )
    overrides_path.parent.mkdir(parents=True, exist_ok=True)
    overrides_path.write_text("{}\n", encoding="utf-8")
    output_path = project_root / "deployment" / "external_mainline.inputs.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_external_mainline_inputs.py",
            "--project-root",
            str(project_root),
            "--output",
            "deployment/external_mainline.inputs.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "external_mainline_inputs_written=" in result.stdout
    assert (
        "external_mainline_inputs_run_command="
        "python tools/run_external_mainline_execution_plan.py --output "
        "test_env/release_evidence/operations/external_mainline_execution_plan.json"
        in result.stdout
    )
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert (
        payload["customer_external_bindings"]["config"]
        == "deployment/customer_delivery.external_bindings.customer.json"
    )
    assert (
        payload["customer_external_bindings"]["overrides_file"]
        == "deployment/customer_delivery.external_bindings.customer.overrides.json"
    )
    assert payload["customer_external_bindings"]["confirmed_by"] == "delivery_lead"
    assert payload["customer_external_bindings"]["confirmation_ticket"] == "CHG-001"
    assert (
        payload["industrial_rehearsal"]["report_path"]
        == "test_env/release_rehearsal_industrial/industrial_delivery_rehearsal_report.json"
    )
    assert payload["industrial_rehearsal"]["version"] == "2026.04.17-industrial-rehearsal"
    assert payload["industrial_live_evidence"]["target_environment"] == "<target-environment>"
    assert (
        payload["industrial_live_evidence"]["install_entrypoint"]
        == "<install-command-or-runbook>"
    )


def test_build_external_mainline_input_checklist_reports_missing_inputs(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _write_industrial_report(project_root)
    inputs_path = project_root / "deployment" / "external_mainline.inputs.json"
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_input_checklist_report.json"
    )

    subprocess.run(
        [
            sys.executable,
            "tools/build_external_mainline_inputs.py",
            "--project-root",
            str(project_root),
            "--output",
            "deployment/external_mainline.inputs.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=True,
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_external_mainline_input_checklist.py",
            "--project-root",
            str(project_root),
            "--inputs-file",
            "deployment/external_mainline.inputs.json",
            "--output",
            "test_env/release_evidence/operations/external_mainline_input_checklist_report.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 1, result.stderr
    assert "external_mainline_input_checklist_report_written=" in result.stdout
    assert "external_mainline_input_checklist_status=blocked" in result.stdout
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["evidence_name"] == "external_mainline_input_checklist"
    assert payload["status"] == "blocked"
    metrics = payload["metrics"]
    assert "customer_external_bindings_closure" in metrics["waiting_external_input_steps"]
    assert "industrial_delivery_live_evidence" in metrics["waiting_external_input_steps"]
    assert "vulnerability_exception_replacement" in metrics["ready_to_run_steps"]
    assert "confirmed_by" in metrics["customer_missing_inputs"]
    assert "真实客户环境标识" in metrics["industrial_missing_inputs"]
    assert metrics["missing_input_count"] > 0
