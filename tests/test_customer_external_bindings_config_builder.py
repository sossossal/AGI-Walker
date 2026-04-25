from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_extension_execution_schedule_artifact,
    build_extension_execution_instance_artifact,
    validate_release_evidence_report,
    write_extension_execution_schedule_artifact,
    write_extension_execution_instance_artifact,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def test_build_customer_external_bindings_config_from_instance_artifact(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            engagement_id="customer-001",
            customer_name="Customer 001",
            site_name="primary-site",
            change_ticket="CHG-CUSTOMER-001",
            window_id="window-001",
            window_start_at="2026-04-18T10:00:00+00:00",
            window_end_at="2026-04-18T12:00:00+00:00",
            delivery_root="test_env/release_delivery/customer_001",
            closure_archive_root="test_env/release_delivery/customer_001/closure_archive",
            exception_review_due_at="2026-05-15T00:00:00+00:00",
        ),
        instance_path,
    )

    output_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
    )
    result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_config.py",
            "--project-root",
            str(project_root),
            "--output",
            "deployment/customer_delivery.external_bindings.customer.json",
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

    assert result.returncode == 0, result.stderr
    assert "customer_external_bindings_config_written=" in result.stdout
    assert "customer_external_bindings_config_binding_state=draft" in result.stdout
    assert (
        "customer_external_bindings_config_rebuild_actuals_command="
        "python tools/build_extension_execution_actuals.py --output "
        "test_env/release_evidence/operations/extension_execution_actuals.json "
        "--external-bindings-config deployment/customer_delivery.external_bindings.customer.json"
        in result.stdout
    )

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["approval_identity"]["binding_state"] == "draft"
    assert payload["approval_identity"]["source_path"] == (
        "test_env/release_delivery/customer_001/approval_identity_source.json"
    )
    assert payload["approval_identity"]["reference"] == "customer-001/window-001/customer_operator"
    assert payload["approval_identity"]["system_name"] == "Customer 001 Approval Registry"
    assert "Replace generated approval metadata" in payload["approval_identity"]["integration_notes"]
    assert payload["archive_target"]["binding_state"] == "draft"
    assert (
        payload["archive_target"]["binding_reference_base"]
        == "archive://customer-001/windows/window-001"
    )
    assert payload["due_trigger"]["binding_state"] == "draft"
    assert (
        payload["due_trigger"]["binding_reference_base"]
        == "schedule://customer-001/windows/window-001"
    )
    assert payload["due_trigger"]["checked_at"] == "2026-04-18T10:00:00+00:00"


def test_generated_customer_external_bindings_config_stays_draft_in_actuals(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    schedule_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_schedule.json"
    )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            engagement_id="customer-001",
            customer_name="Customer 001",
            site_name="primary-site",
            change_ticket="CHG-CUSTOMER-001",
            window_id="window-001",
            window_start_at="2026-04-18T10:00:00+00:00",
            window_end_at="2026-04-18T12:00:00+00:00",
            delivery_root="test_env/release_delivery/customer_001",
            closure_archive_root="test_env/release_delivery/customer_001/closure_archive",
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
        schedule_path,
    )
    config_result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_config.py",
            "--project-root",
            str(project_root),
            "--output",
            "deployment/customer_delivery.external_bindings.customer.json",
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
    assert config_result.returncode == 0, config_result.stderr

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
            "deployment/customer_delivery.external_bindings.customer.json",
            "--window-trigger-recorded-by",
            "delivery_lead",
            "--signoff-recorded-by",
            "customer_operator",
            "--residual-risk-reviewed-by",
            "delivery_lead",
            "--closure-archived-by",
            "rollback_owner",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert actuals_result.returncode != 0
    assert "extension_execution_actuals_external_bindings_status=placeholder" in actuals_result.stdout
    assert (
        "extension_execution_actuals_external_bindings_follow_up_required=true"
        in actuals_result.stdout
    )
    payload = json.loads(actuals_output_path.read_text(encoding="utf-8"))
    assert payload["external_bindings_status"] == "placeholder"
    assert payload["external_bindings_follow_up_required"] is True
    assert payload["external_bindings_draft_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert "Draft sections" in payload["external_bindings_summary"]

    confirmation_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_confirmation_report.json"
    )
    confirmation_result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_confirmation_report.py",
            "--project-root",
            str(project_root),
            "--actuals-artifact",
            "test_env/release_evidence/operations/extension_execution_actuals.json",
            "--output",
            "test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert confirmation_result.returncode != 0
    assert (
        "customer_external_bindings_confirmation_report_status=blocked"
        in confirmation_result.stdout
    )
    confirmation_payload = json.loads(
        confirmation_report_path.read_text(encoding="utf-8")
    )
    assert validate_release_evidence_report(confirmation_payload) == []
    assert confirmation_payload["evidence_name"] == "customer_external_bindings_confirmation"
    assert confirmation_payload["status"] == "blocked"
    assert confirmation_payload["metrics"]["external_bindings_status"] == "placeholder"
    assert confirmation_payload["metrics"]["draft_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]


def test_confirm_customer_external_bindings_marks_sections_confirmed(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    schedule_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_schedule.json"
    )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            engagement_id="customer-001",
            customer_name="Customer 001",
            site_name="primary-site",
            change_ticket="CHG-CUSTOMER-001",
            window_id="window-001",
            window_start_at="2026-04-18T10:00:00+00:00",
            window_end_at="2026-04-18T12:00:00+00:00",
            delivery_root="test_env/release_delivery/customer_001",
            closure_archive_root="test_env/release_delivery/customer_001/closure_archive",
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
        schedule_path,
    )

    config_result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_config.py",
            "--project-root",
            str(project_root),
            "--output",
            "deployment/customer_delivery.external_bindings.customer.json",
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
    assert config_result.returncode == 0, config_result.stderr

    confirm_result = subprocess.run(
        [
            sys.executable,
            "tools/confirm_customer_external_bindings.py",
            "--project-root",
            str(project_root),
            "--config",
            "deployment/customer_delivery.external_bindings.customer.json",
            "--confirmed-by",
            "release-manager",
            "--confirmation-ticket",
            "CHG-CUSTOMER-001",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert confirm_result.returncode == 0, confirm_result.stderr
    assert "customer_external_bindings_confirmed=" in confirm_result.stdout
    assert (
        "customer_external_bindings_confirmed_sections="
        "approval_identity,archive_target,due_trigger"
        in confirm_result.stdout
    )
    assert "customer_external_bindings_confirmed_by=release-manager" in confirm_result.stdout
    assert (
        "customer_external_bindings_rebuild_actuals_command="
        "python tools/build_extension_execution_actuals.py --output "
        "test_env/release_evidence/operations/extension_execution_actuals.json "
        "--external-bindings-config deployment/customer_delivery.external_bindings.customer.json"
        in confirm_result.stdout
    )

    config_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
    )
    payload = json.loads(config_path.read_text(encoding="utf-8"))
    for section in ("approval_identity", "archive_target", "due_trigger"):
        assert payload[section]["binding_state"] == "confirmed"
        assert payload[section]["confirmed_by"] == "release-manager"
        assert payload[section]["confirmation_ticket"] == "CHG-CUSTOMER-001"
        assert payload[section]["confirmed_at"]

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
            "deployment/customer_delivery.external_bindings.customer.json",
            "--window-trigger-recorded-by",
            "delivery_lead",
            "--signoff-recorded-by",
            "customer_operator",
            "--residual-risk-reviewed-by",
            "delivery_lead",
            "--closure-archived-by",
            "rollback_owner",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert actuals_result.returncode in {0, 1}, actuals_result.stderr
    assert "extension_execution_actuals_external_bindings_status=ready" in actuals_result.stdout
    actuals_payload = json.loads(actuals_output_path.read_text(encoding="utf-8"))
    assert actuals_payload["external_bindings_status"] == "ready"
    assert actuals_payload["external_bindings_follow_up_required"] is False
    assert actuals_payload["external_bindings_draft_sections"] == []
    assert actuals_payload["external_bindings_confirmed_count"] == 3
    assert actuals_payload["external_bindings_confirmed_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert actuals_payload["external_bindings_confirmation_missing_sections"] == []
    assert actuals_payload["external_bindings_confirmed_by"] == ["release-manager"]
    assert actuals_payload["external_bindings_confirmation_tickets"] == ["CHG-CUSTOMER-001"]
    assert actuals_payload["external_bindings_last_confirmed_at"]

    approval_identity_source_payload = json.loads(
        (
            project_root
            / "test_env"
            / "release_delivery"
            / "customer_001"
            / "approval_identity_source.json"
        ).read_text(encoding="utf-8")
    )
    assert approval_identity_source_payload["external_binding_confirmation"] == {
        "binding_state": "confirmed",
        "confirmed_by": "release-manager",
        "confirmed_at": payload["approval_identity"]["confirmed_at"],
        "confirmation_ticket": "CHG-CUSTOMER-001",
    }

    confirmation_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_confirmation_report.json"
    )
    confirmation_result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_confirmation_report.py",
            "--project-root",
            str(project_root),
            "--actuals-artifact",
            "test_env/release_evidence/operations/extension_execution_actuals.json",
            "--output",
            "test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert confirmation_result.returncode == 0, confirmation_result.stderr
    assert (
        "customer_external_bindings_confirmation_report_status=passed"
        in confirmation_result.stdout
    )
    confirmation_payload = json.loads(
        confirmation_report_path.read_text(encoding="utf-8")
    )
    assert validate_release_evidence_report(confirmation_payload) == []
    assert confirmation_payload["evidence_name"] == "customer_external_bindings_confirmation"
    assert confirmation_payload["status"] == "passed"
    assert confirmation_payload["metrics"]["external_bindings_status"] == "ready"
    assert confirmation_payload["metrics"]["confirmed_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert confirmation_payload["metrics"]["confirmation_tickets"] == [
        "CHG-CUSTOMER-001"
    ]


def test_confirm_customer_external_bindings_rejects_unready_sections(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            engagement_id="customer-001",
            customer_name="Customer 001",
            site_name="primary-site",
            change_ticket="CHG-CUSTOMER-001",
            window_id="window-001",
            window_start_at="2026-04-18T10:00:00+00:00",
            window_end_at="2026-04-18T12:00:00+00:00",
            delivery_root="test_env/release_delivery/customer_001",
            closure_archive_root="test_env/release_delivery/customer_001/closure_archive",
            exception_review_due_at="2026-05-15T00:00:00+00:00",
        ),
        instance_path,
    )

    config_result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_config.py",
            "--project-root",
            str(project_root),
            "--output",
            "deployment/customer_delivery.external_bindings.customer.json",
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
    assert config_result.returncode == 0, config_result.stderr

    confirm_result = subprocess.run(
        [
            sys.executable,
            "tools/confirm_customer_external_bindings.py",
            "--project-root",
            str(project_root),
            "--config",
            "deployment/customer_delivery.external_bindings.customer.json",
            "--section",
            "archive_target",
            "--confirmed-by",
            "release-manager",
            "--confirmation-ticket",
            "CHG-CUSTOMER-001",
            "--set",
            "archive_target.binding_reference_base=",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert confirm_result.returncode != 0
    assert (
        "section 'archive_target' is missing required ready fields after overrides"
        in confirm_result.stderr
    )


def test_run_customer_external_bindings_closure_generates_draft_and_blocks_without_overrides(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_customer_external_bindings_closure.py",
            "--project-root",
            str(project_root),
            "--confirmed-by",
            "release-manager",
            "--confirmation-ticket",
            "CHG-CUSTOMER-001",
            "--skip-collect-release-evidence",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode != 0
    assert "customer_external_bindings_closure_instance_generated=true" in result.stdout
    assert "customer_external_bindings_closure_schedule_generated=true" in result.stdout
    assert "customer_external_bindings_closure_config_generated=true" in result.stdout
    assert "customer_external_bindings_closure_status=blocked" in result.stdout
    assert "customer_external_bindings_closure_report_written=" in result.stdout
    assert (
        "customer_external_bindings_closure_failed_steps="
        "customer_external_bindings_overrides_missing"
        in result.stdout
    )
    assert "--overrides-file / --set overrides" in result.stdout
    config_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
    )
    assert config_path.is_file()
    actuals_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_actuals.json"
    )
    assert not actuals_path.exists()
    closure_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_closure_report.json"
    )
    closure_report_payload = json.loads(
        closure_report_path.read_text(encoding="utf-8")
    )
    assert validate_release_evidence_report(closure_report_payload) == []
    assert closure_report_payload["status"] == "blocked"
    assert (
        closure_report_payload["metrics"]["failed_steps"]
        == ["customer_external_bindings_overrides_missing"]
    )
    assert closure_report_payload["metrics"]["generated_config"] is True
    assert closure_report_payload["metrics"]["actuals_exists"] is False


def test_run_customer_external_bindings_closure_confirms_bindings_with_overrides(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_customer_external_bindings_closure.py",
            "--project-root",
            str(project_root),
            "--confirmed-by",
            "release-manager",
            "--confirmation-ticket",
            "CHG-CUSTOMER-001",
            "--skip-collect-release-evidence",
            "--set",
            "approval_identity.source_type=customer_signoff_registry",
            "--set",
            "approval_identity.reference=signoff://customer-001/window-001",
            "--set",
            "approval_identity.system_name=Customer Signoff Registry",
            "--set",
            "archive_target.binding_type=s3_archive_bucket",
            "--set",
            "archive_target.binding_reference_base=s3://customer-001/window-001",
            "--set",
            "archive_target.system_name=Customer Archive Bucket",
            "--set",
            "due_trigger.binding_type=customer_scheduler_job",
            "--set",
            "due_trigger.binding_reference_base=scheduler://customer-001/window-001",
            "--set",
            "due_trigger.system_name=Customer Scheduler",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "customer_external_bindings_closure_instance_generated=true" in result.stdout
    assert "customer_external_bindings_closure_schedule_generated=true" in result.stdout
    assert "customer_external_bindings_closure_config_generated=true" in result.stdout
    assert "customer_external_bindings_closure_report_written=" in result.stdout
    assert "customer_external_bindings_closure_status=passed" in result.stdout

    config_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
    )
    config_payload = json.loads(config_path.read_text(encoding="utf-8"))
    assert config_payload["approval_identity"]["binding_state"] == "confirmed"
    assert (
        config_payload["approval_identity"]["reference"]
        == "signoff://customer-001/window-001"
    )
    assert (
        config_payload["archive_target"]["binding_reference_base"]
        == "s3://customer-001/window-001"
    )
    assert (
        config_payload["due_trigger"]["binding_reference_base"]
        == "scheduler://customer-001/window-001"
    )

    actuals_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_actuals.json"
    )
    actuals_payload = json.loads(actuals_path.read_text(encoding="utf-8"))
    assert actuals_payload["external_bindings_status"] == "ready"
    assert actuals_payload["external_bindings_follow_up_required"] is False
    assert actuals_payload["external_bindings_confirmed_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    assert actuals_payload["external_bindings_confirmation_tickets"] == [
        "CHG-CUSTOMER-001"
    ]

    confirmation_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_confirmation_report.json"
    )
    confirmation_payload = json.loads(
        confirmation_report_path.read_text(encoding="utf-8")
    )
    assert confirmation_payload["status"] == "passed"
    assert confirmation_payload["metrics"]["confirmed_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]
    closure_report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_closure_report.json"
    )
    closure_report_payload = json.loads(
        closure_report_path.read_text(encoding="utf-8")
    )
    assert validate_release_evidence_report(closure_report_payload) == []
    assert closure_report_payload["status"] == "passed"
    assert closure_report_payload["metrics"]["actuals_exists"] is True
    assert closure_report_payload["metrics"]["confirmation_report_exists"] is True
    assert closure_report_payload["metrics"]["selected_sections"] == [
        "approval_identity",
        "archive_target",
        "due_trigger",
    ]


def test_confirm_customer_external_bindings_supports_overrides_file(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            engagement_id="customer-001",
            customer_name="Customer 001",
            site_name="primary-site",
            change_ticket="CHG-CUSTOMER-001",
            window_id="window-001",
            window_start_at="2026-04-18T10:00:00+00:00",
            window_end_at="2026-04-18T12:00:00+00:00",
            delivery_root="test_env/release_delivery/customer_001",
            closure_archive_root="test_env/release_delivery/customer_001/closure_archive",
            exception_review_due_at="2026-05-15T00:00:00+00:00",
        ),
        instance_path,
    )

    config_result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_config.py",
            "--project-root",
            str(project_root),
            "--output",
            "deployment/customer_delivery.external_bindings.customer.json",
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
    assert config_result.returncode == 0, config_result.stderr

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
                    "reference": "signoff://customer-001/window-001",
                    "system_name": "Customer Signoff Registry",
                },
                "archive_target": {
                    "binding_reference_base": "s3://customer-001/window-001",
                    "system_name": "Customer Archive Bucket",
                },
                "due_trigger": {
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

    confirm_result = subprocess.run(
        [
            sys.executable,
            "tools/confirm_customer_external_bindings.py",
            "--project-root",
            str(project_root),
            "--config",
            "deployment/customer_delivery.external_bindings.customer.json",
            "--overrides-file",
            "deployment/customer_delivery.external_bindings.customer.overrides.json",
            "--confirmed-by",
            "release-manager",
            "--confirmation-ticket",
            "CHG-CUSTOMER-001",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert confirm_result.returncode == 0, confirm_result.stderr
    assert (
        "customer_external_bindings_overrides_file="
        f"{overrides_path}"
        in confirm_result.stdout
    )
    payload = json.loads(
        (
            project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
        ).read_text(encoding="utf-8")
    )
    assert payload["approval_identity"]["reference"] == "signoff://customer-001/window-001"
    assert payload["archive_target"]["binding_reference_base"] == "s3://customer-001/window-001"
    assert payload["due_trigger"]["binding_reference_base"] == "scheduler://customer-001/window-001"
    assert payload["approval_identity"]["binding_state"] == "confirmed"


def test_run_customer_external_bindings_closure_supports_overrides_file(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
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

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_customer_external_bindings_closure.py",
            "--project-root",
            str(project_root),
            "--confirmed-by",
            "release-manager",
            "--confirmation-ticket",
            "CHG-CUSTOMER-001",
            "--skip-collect-release-evidence",
            "--overrides-file",
            "deployment/customer_delivery.external_bindings.customer.overrides.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (
        "customer_external_bindings_closure_overrides_file="
        f"{overrides_path}"
        in result.stdout
    )
    assert "customer_external_bindings_closure_report_written=" in result.stdout
    assert "customer_external_bindings_closure_status=passed" in result.stdout
    actuals_payload = json.loads(
        (
            project_root
            / "test_env"
            / "release_evidence"
            / "operations"
            / "extension_execution_actuals.json"
        ).read_text(encoding="utf-8")
    )
    assert actuals_payload["external_bindings_status"] == "ready"
    assert actuals_payload["external_bindings_confirmation_tickets"] == [
        "CHG-CUSTOMER-001"
    ]
    closure_report_payload = json.loads(
        (
            project_root
            / "test_env"
            / "release_evidence"
            / "operations"
            / "customer_external_bindings_closure_report.json"
        ).read_text(encoding="utf-8")
    )
    assert validate_release_evidence_report(closure_report_payload) == []
    assert closure_report_payload["status"] == "passed"
    assert (
        closure_report_payload["metrics"]["overrides_file"]
        == str(overrides_path)
    )


def test_build_extension_execution_actuals_rejects_confirmed_bindings_without_confirmation_metadata(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)

    config_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
    )
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(
        json.dumps(
            {
                "approval_identity": {
                    "binding_state": "confirmed",
                    "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
                    "source_type": "customer_ticket_registry",
                    "reference": "customer-001/window-001/customer_operator",
                }
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
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
            "deployment/customer_delivery.external_bindings.customer.json",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert actuals_result.returncode != 0
    assert (
        "external bindings config section 'approval_identity' is marked confirmed but is missing "
        "confirmed_by, confirmed_at, confirmation_ticket"
        in actuals_result.stderr
    )


def test_build_extension_execution_actuals_skips_placeholder_approval_source_path(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    schedule_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_schedule.json"
    )
    write_extension_execution_instance_artifact(
        build_extension_execution_instance_artifact(
            project_root=project_root,
            artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
            engagement_id="customer-001",
            customer_name="Customer 001",
            site_name="primary-site",
            change_ticket="CHG-CUSTOMER-001",
            window_id="window-001",
            window_start_at="2026-04-18T10:00:00+00:00",
            window_end_at="2026-04-18T12:00:00+00:00",
            delivery_root="test_env/release_delivery/customer_001",
            closure_archive_root="test_env/release_delivery/customer_001/closure_archive",
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
        schedule_path,
    )

    config_path = (
        project_root / "deployment" / "customer_delivery.external_bindings.customer.json"
    )
    config_path.parent.mkdir(parents=True, exist_ok=True)
    config_path.write_text(
        json.dumps(
            {
                "approval_identity": {
                    "binding_state": "draft",
                    "source_path": "<export-root>/changes/CHG-2026-0001/approval.json",
                    "source_type": "customer_signoff_registry",
                    "reference": "change://<customer-tenant>/changes/CHG-2026-0001",
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
                    "checked_at": "2026-04-18T12:00:00+00:00",
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
            "deployment/customer_delivery.external_bindings.customer.json",
            "--window-trigger-recorded-by",
            "delivery_lead",
            "--signoff-recorded-by",
            "customer_operator",
            "--residual-risk-reviewed-by",
            "delivery_lead",
            "--closure-archived-by",
            "rollback_owner",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert actuals_result.returncode != 0
    assert "WinError 123" not in actuals_result.stderr
    assert "OSError" not in actuals_result.stderr
    assert "extension_execution_actuals_skipped_artifact=approval_identity_source" in actuals_result.stdout
    assert "extension_execution_actuals_external_bindings_status=placeholder" in actuals_result.stdout
    actuals_payload = json.loads(actuals_output_path.read_text(encoding="utf-8"))
    assert actuals_payload["external_bindings_status"] == "placeholder"
    assert actuals_payload["approval_identity_source_path"] == (
        "<export-root>/changes/CHG-2026-0001/approval.json"
    )
    assert not (project_root / "<export-root>").exists()


def test_build_customer_external_bindings_config_rejects_invalid_instance_artifact(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    instance_path.parent.mkdir(parents=True, exist_ok=True)
    instance_path.write_text(
        json.dumps({"schema_version": "1.0", "artifact_type": "extension_execution_instance"})
        + "\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_customer_external_bindings_config.py",
            "--project-root",
            str(project_root),
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

    assert result.returncode != 0
    assert "instance artifact is invalid" in result.stderr
