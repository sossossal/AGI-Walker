from __future__ import annotations

import json
import shutil
import subprocess
import sys
from pathlib import Path
from uuid import uuid4

from agi_walker.core.api.release_contracts import validate_release_manifest_artifact


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def test_run_release_rehearsal_script_generates_ready_stable_manifest(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "release_rehearsal"
    report_path = output_root / "release_rehearsal_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/run_release_rehearsal.py",
            "--version",
            "2026.04.12-rehearsal",
            "--build-id",
            "release-rehearsal-test",
            "--output-root",
            str(output_root),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert result.returncode == 0, result.stderr
    assert "release_rehearsal_written=" in result.stdout
    assert "industrial_delivery_rehearsal_report_written=" in result.stdout
    assert "industrial_delivery_rehearsal_status=ready" in result.stdout
    assert "industrial_delivery_vulnerability_exception_review=passed/1" in result.stdout
    assert "industrial_delivery_customer_external_bindings_closure=passed" in result.stdout
    assert "industrial_delivery_external_mainline_execution_plan=ready/" in result.stdout
    assert "industrial_delivery_release_ops_execution=passed/3" in result.stdout
    assert "release_rehearsal_control_plane_events=3" in result.stdout
    assert "industrial_delivery_control_plane_events=3" in result.stdout
    assert "release_rehearsal_gate=ready" in result.stdout
    assert report_path.exists()

    report_payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert report_payload["status"] == "passed"
    assert report_payload["tag"] == "2026.04.12-rehearsal"
    assert report_payload["release_gate_status"] == "ready"
    assert report_payload["customer_delivery_status"] == "ready"
    assert report_payload["industrial_delivery_status"] == "ready"
    assert report_payload["control_plane_session"]["engagement_id"] == "release-rehearsal-test"
    assert report_payload["control_plane_event_stream"]["event_count"] == 3
    assert report_payload["extension_execution_plan"]["status"] == "ready"
    assert report_payload["extension_execution_plan"]["actionable_profiles"] == 3
    assert report_payload["extension_execution_evidence"]["status"] == "ready"
    assert report_payload["extension_execution_evidence"]["ready_reports"] == 4
    assert report_payload["extension_execution_instance"]["status"] == "ready"
    assert report_payload["extension_execution_instance"]["ready_profiles"] == 3
    assert report_payload["extension_execution_schedule"]["status"] == "ready"
    assert report_payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert report_payload["extension_execution_actuals"]["status"] == "ready"
    assert report_payload["extension_execution_actuals"]["ready_profiles"] == 3
    assert report_payload["extension_execution_actuals"]["external_bindings_status"] == "ready"
    assert (
        report_payload["extension_execution_actuals"]["external_bindings_follow_up_required"]
        is False
    )
    assert report_payload["security_release_preflight"]["status"] == "passed"
    assert (
        report_payload["security_release_preflight"]["metrics"][
            "stale_vulnerability_exceptions"
        ]
        == 0
    )
    assert report_payload["vulnerability_exception_review"]["status"] == "passed"
    assert (
        report_payload["vulnerability_exception_review"]["review_candidate_count"] >= 1
    )
    assert report_payload["customer_external_bindings_closure"]["status"] == "passed"
    assert report_payload["external_mainline_execution_plan"]["status"] == "ready"
    assert report_payload["external_mainline_execution_plan"]["waiting_external_input_steps"] >= 0
    assert report_payload["release_ops_execution"]["status"] == "passed"
    assert report_payload["release_ops_execution"]["event_count"] == 3
    assert (
        report_payload["external_mainline_execution_plan"]["control_plane_session"][
            "engagement_id"
        ]
        == "release-rehearsal-test"
    )
    assert (
        report_payload["external_mainline_execution_plan"]["control_plane_event_stream"][
            "event_count"
        ]
        == 3
    )
    assert report_payload["industrial_manifest"]["status"] == "ready"
    assert report_payload["industrial_release_readiness"]["status"] == "ready"
    assert report_payload["industrial_promotion_checklist"]["status"] == "ready"
    assert report_payload["industrial_promotion_checklist"]["blocking_steps"] == 0
    assert report_payload["industrial_customer_acceptance_bundle"]["status"] == "ready"
    assert (
        report_payload["industrial_customer_acceptance_bundle"][
            "vulnerability_exception_review"
        ]["status"]
        == "passed"
    )
    assert (
        report_payload["industrial_customer_acceptance_bundle"][
            "external_mainline_execution_plan"
        ]["status"]
        == "ready"
    )
    assert (
        report_payload["industrial_customer_acceptance_bundle"][
            "external_mainline_execution_plan"
        ]["control_plane_session"]["engagement_id"]
        == "release-rehearsal-test"
    )
    assert (
        report_payload["industrial_customer_acceptance_bundle"][
            "external_mainline_input_checklist"
        ]["status"]
        == "blocked"
    )
    assert (
        report_payload["industrial_customer_acceptance_bundle"][
            "external_mainline_input_checklist"
        ]["control_plane_session"]["engagement_id"]
        == "release-rehearsal-test"
    )
    assert (
        report_payload["industrial_customer_acceptance_bundle"][
            "release_ops_execution"
        ]["status"]
        == "passed"
    )
    assert (
        report_payload["industrial_customer_acceptance_bundle"][
            "release_ops_execution"
        ]["event_count"]
        == 3
    )
    assert (
        report_payload["industrial_customer_acceptance_bundle"]["reports_present"]
        == report_payload["industrial_customer_acceptance_bundle"]["reports_total"]
    )
    assert report_payload["extension_execution_actuals"]["external_bindings"]["config_path"] == (
        "deployment/customer_delivery.external_bindings.rehearsal.json"
    )
    assert len(report_payload["industrial_delivery_artifact_paths"]) == 5
    assert [item["name"] for item in report_payload["industrial_delivery_artifact_paths"]] == [
        "release_manifest_industrial.json",
        "industrial_release_readiness_report.json",
        "industrial_promotion_checklist.json",
        "customer_acceptance_bundle_industrial.json",
        "industrial_delivery_rehearsal_report.json",
    ]
    assert [item["id"] for item in report_payload["delivery_rehearsal_stages"]] == [
        "new_environment_install",
        "smoke",
        "live_evidence",
        "upgrade",
        "rollback",
        "backup_restore",
    ]
    assert all(
        item["status"] == "pass" for item in report_payload["delivery_rehearsal_stages"]
    )
    assert all(
        item["artifact_paths"] for item in report_payload["delivery_rehearsal_stages"]
    )
    distributed_profile = next(
        item
        for item in report_payload["extension_execution_plan"]["profiles"]
        if item["id"] == "distributed_profile"
    )
    assert distributed_profile["execution_template"]["rollback_owner_role"] == "rollback_owner"
    assert distributed_profile["execution_template"]["upgrade_window_steps"][0]["owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["handoff_owner_role"] == "delivery_lead"
    assert distributed_profile["execution_template"]["watch_owner_role"] == "customer_operator"
    assert distributed_profile["execution_template"]["on_call_handoff_owner_role"] == "delivery_lead"
    assert (
        distributed_profile["execution_template"]["signoff_checkpoints"][0][
            "required_artifact"
        ]
        == "test_env/distributed_smoke/distributed_smoke_report.json"
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
        distributed_profile["execution_template"]["escalation_closure_steps"][1][
            "required_artifact"
        ]
        == "test_env/distributed_smoke/distributed_smoke_report.json"
    )
    assert report_payload["document_paths"]
    assert any(
        item["name"] == "stable_gate_ready" and item["status"] == "pass"
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "customer_delivery_ready" and item["status"] == "pass"
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "industrial_delivery_ready" and item["status"] == "pass"
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "extension_execution_plan_ready" and item["status"] == "pass"
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "extension_execution_plan_ready"
        and "exception review steps" in item["detail"]
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "extension_execution_evidence_ready"
        and "on-call rehearsal" in item["detail"]
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "extension_execution_instance_ready"
        and "delivery window" in item["detail"]
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "extension_execution_schedule_ready"
        and "window triggers" in item["detail"]
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "extension_execution_actuals_ready"
        and "window trigger records" in item["detail"]
        for item in report_payload["checks"]
    )
    assert any(
        item["name"] == "extension_execution_actuals_ready"
        and "external bindings status=ready" in item["detail"]
        for item in report_payload["checks"]
    )

    manifest_path = Path(report_payload["manifest_path"])
    assert manifest_path.exists()
    manifest_payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert validate_release_manifest_artifact(manifest_payload) == []
    assert manifest_payload["channel"] == "stable"
    assert manifest_payload["release_gate_status"] == "ready"
    assert manifest_payload["customer_delivery_surface"]["status"] == "ready"
    assert manifest_payload["industrial_delivery_gate"]["status"] == "ready"
    assert manifest_payload["extension_execution_evidence"]["status"] == "ready"
    assert manifest_payload["extension_execution_instance"]["status"] == "ready"
    assert manifest_payload["extension_execution_schedule"]["status"] == "ready"
    assert manifest_payload["extension_execution_actuals"]["status"] == "ready"
    assert manifest_payload["extension_execution_actuals"]["external_bindings"]["config_path"] == (
        "deployment/customer_delivery.external_bindings.rehearsal.json"
    )
    assert manifest_payload["release_source"]["version_tag_matches"] is True
    industrial_manifest_path = Path(
        report_payload["industrial_manifest"]["manifest_path"]
    )
    assert industrial_manifest_path.exists()
    industrial_manifest_payload = json.loads(
        industrial_manifest_path.read_text(encoding="utf-8")
    )
    assert validate_release_manifest_artifact(industrial_manifest_payload) == []
    assert industrial_manifest_payload["channel"] == "industrial"
    assert industrial_manifest_payload["release_gate_status"] == "ready"
    assert industrial_manifest_payload["customer_delivery_surface"]["status"] == "ready"
    assert industrial_manifest_payload["industrial_delivery_gate"]["status"] == "ready"
    industrial_readiness_report_path = Path(
        report_payload["industrial_release_readiness"]["report_path"]
    )
    assert industrial_readiness_report_path.exists()
    industrial_readiness_payload = json.loads(
        industrial_readiness_report_path.read_text(encoding="utf-8")
    )
    assert (
        industrial_readiness_payload["artifact_type"]
        == "industrial_release_readiness_report"
    )
    assert industrial_readiness_payload["industrial_release_gate"] == "ready"
    assert industrial_readiness_payload["extension_execution_actuals"]["status"] == "ready"
    industrial_promotion_report_path = Path(
        report_payload["industrial_promotion_checklist"]["report_path"]
    )
    assert industrial_promotion_report_path.exists()
    industrial_promotion_payload = json.loads(
        industrial_promotion_report_path.read_text(encoding="utf-8")
    )
    assert (
        industrial_promotion_payload["artifact_type"]
        == "industrial_promotion_checklist"
    )
    assert industrial_promotion_payload["ready_to_promote"] is True
    assert industrial_promotion_payload["blocking_steps"] == 0
    assert (
        industrial_promotion_payload["extension_execution_actuals"][
            "external_bindings_status"
        ]
        == "ready"
    )
    promotion_steps = {
        item["id"]: item for item in industrial_promotion_payload["steps"]
    }
    assert promotion_steps["extension_external_bindings"]["status"] == "done"
    assert promotion_steps["extension_external_bindings"]["blocking"] is False
    assert promotion_steps["extension_external_bindings"]["ready_to_run"] is False
    assert promotion_steps["extension_external_bindings"]["command"] is None
    industrial_bundle_path = Path(
        report_payload["industrial_customer_acceptance_bundle"]["bundle_path"]
    )
    assert industrial_bundle_path.exists()
    industrial_bundle_payload = json.loads(
        industrial_bundle_path.read_text(encoding="utf-8")
    )
    assert industrial_bundle_payload["artifact_type"] == "customer_acceptance_bundle"
    assert industrial_bundle_payload["bundle_status"] == "ready"
    acceptance_reports = {
        item["name"]: item for item in industrial_bundle_payload["acceptance_reports"]
    }
    assert acceptance_reports["industrial_release_readiness"]["exists"] is True
    assert acceptance_reports["industrial_release_readiness"]["status"] == "ready"
    assert acceptance_reports["industrial_promotion_checklist"]["exists"] is True
    assert acceptance_reports["industrial_promotion_checklist"]["status"] == "ready"
    assert acceptance_reports["industrial_delivery_rehearsal_report"]["exists"] is True
    assert acceptance_reports["industrial_delivery_rehearsal_report"]["status"] == "ready"
    assert acceptance_reports["customer_external_bindings_closure"]["exists"] is True
    assert acceptance_reports["customer_external_bindings_closure"]["status"] == "passed"
    assert acceptance_reports["external_mainline_execution_plan"]["exists"] is True
    assert acceptance_reports["external_mainline_execution_plan"]["status"] == "ready"
    assert (
        acceptance_reports["external_mainline_execution_plan"]["control_plane_session"][
            "engagement_id"
        ]
        == "release-rehearsal-test"
    )
    assert acceptance_reports["external_mainline_input_checklist"]["exists"] is True
    assert acceptance_reports["external_mainline_input_checklist"]["status"] == "blocked"
    assert (
        acceptance_reports["external_mainline_input_checklist"]["control_plane_session"][
            "engagement_id"
        ]
        == "release-rehearsal-test"
    )
    assert acceptance_reports["release_ops_execution"]["exists"] is True
    assert acceptance_reports["release_ops_execution"]["status"] == "passed"
    assert acceptance_reports["customer_external_bindings_confirmation"]["exists"] is True
    assert acceptance_reports["customer_external_bindings_confirmation"]["status"] == "passed"
    assert acceptance_reports["vulnerability_exception_review"]["exists"] is True
    assert acceptance_reports["vulnerability_exception_review"]["status"] == "passed"
    assert (
        industrial_bundle_payload["extension_execution_actuals"]["external_bindings_status"]
        == "ready"
    )
    assert (
        "python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json"
        in industrial_bundle_payload["recommended_commands"]
    )
    assert (
        "python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --external-bindings-config deployment/customer_delivery.external_bindings.rehearsal.json"
        in industrial_bundle_payload["recommended_commands"]
    )
    assert (
        "python tools/build_customer_external_bindings_confirmation_report.py --output test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json --actuals-artifact test_env/release_evidence/operations/extension_execution_actuals.json"
        in industrial_bundle_payload["recommended_commands"]
    )
    industrial_delivery_rehearsal_report_path = (
        output_root / "industrial_delivery_rehearsal_report.json"
    )
    assert industrial_delivery_rehearsal_report_path.exists()
    industrial_delivery_rehearsal_payload = json.loads(
        industrial_delivery_rehearsal_report_path.read_text(encoding="utf-8")
    )
    assert (
        industrial_delivery_rehearsal_payload["artifact_type"]
        == "industrial_delivery_rehearsal_report"
    )
    assert industrial_delivery_rehearsal_payload["status"] == "ready"
    assert industrial_delivery_rehearsal_payload["stage_summary"] == {
        "total": 6,
        "passed": 6,
        "failed": 0,
    }
    assert (
        industrial_delivery_rehearsal_payload["vulnerability_exception_review"]["status"]
        == "passed"
    )
    assert industrial_delivery_rehearsal_payload["release_ops_execution"]["status"] == "passed"
    assert industrial_delivery_rehearsal_payload["release_ops_execution"]["event_count"] == 3
    assert (
        industrial_delivery_rehearsal_payload["control_plane_session"]["engagement_id"]
        == "release-rehearsal-test"
    )
    assert (
        industrial_delivery_rehearsal_payload["control_plane_event_stream"]["event_count"]
        == 3
    )
    assert (
        industrial_delivery_rehearsal_payload["vulnerability_exception_review"][
            "review_candidate_count"
        ]
        >= 1
    )
    assert (
        industrial_delivery_rehearsal_payload["customer_external_bindings_closure"][
            "status"
        ]
        == "passed"
    )
    assert (
        industrial_delivery_rehearsal_payload["extension_execution_actuals"][
            "external_bindings_status"
        ]
        == "ready"
    )
    assert (
        industrial_delivery_rehearsal_payload["industrial_manifest"]["summary"]
        == "industrial manifest status=ready."
    )
    assert (
        industrial_delivery_rehearsal_payload["release_rehearsal_report_path"]
        == str(report_path)
    )
    assert (
        output_root / "deployment" / "customer_delivery.external_bindings.rehearsal.json"
    ).exists()
    assert (
        output_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_closure_report.json"
    ).exists()
    assert (
        output_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "customer_external_bindings_confirmation_report.json"
    ).exists()


def test_run_release_rehearsal_script_is_repeatable_for_same_output_root(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "release_rehearsal"

    for _ in range(2):
        result = subprocess.run(
            [
                sys.executable,
                "tools/run_release_rehearsal.py",
                "--version",
                "2026.04.12-rehearsal",
                "--build-id",
                "release-rehearsal-repeatable",
                "--output-root",
                str(output_root),
            ],
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

        assert result.returncode == 0, result.stderr
        assert "industrial_delivery_rehearsal_status=ready" in result.stdout
        assert "release_rehearsal_gate=ready" in result.stdout

    report_path = output_root / "release_rehearsal_report.json"
    report_payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert report_payload["customer_delivery_status"] == "ready"
    assert report_payload["industrial_delivery_status"] == "ready"
    assert report_payload["extension_execution_plan"]["status"] == "ready"
    assert report_payload["extension_execution_instance"]["status"] == "ready"
    assert report_payload["extension_execution_schedule"]["status"] == "ready"
    assert report_payload["extension_execution_actuals"]["status"] == "ready"
    assert report_payload["security_release_preflight"]["status"] == "passed"
    assert report_payload["industrial_manifest"]["status"] == "ready"
    assert report_payload["industrial_release_readiness"]["status"] == "ready"
    assert report_payload["industrial_promotion_checklist"]["status"] == "ready"
    assert report_payload["industrial_customer_acceptance_bundle"]["status"] == "ready"
    assert len(report_payload["delivery_rehearsal_stages"]) == 6
    assert (
        report_payload["extension_execution_plan"]["profiles"][0]["execution_template"][
            "rollback_owner_role"
        ]
        == "rollback_owner"
    )
    assert (
        report_payload["extension_execution_plan"]["profiles"][0]["execution_template"][
            "rollback_evidence_owner_role"
        ]
        == "rollback_owner"
    )
    assert (
        report_payload["extension_execution_plan"]["profiles"][0]["execution_template"][
            "watch_owner_role"
        ]
        == "customer_operator"
    )


def test_run_release_rehearsal_script_accepts_relative_output_root() -> None:
    relative_output_root = Path("test_env") / f"release_rehearsal_relative_{uuid4().hex[:8]}"
    output_root = (PROJECT_ROOT / relative_output_root).resolve()

    shutil.rmtree(output_root, ignore_errors=True)
    try:
        result = subprocess.run(
            [
                sys.executable,
                "tools/run_release_rehearsal.py",
                "--version",
                "2026.04.15-rehearsal-relative",
                "--build-id",
                "release-rehearsal-relative",
                "--output-root",
                str(relative_output_root),
            ],
            cwd=str(PROJECT_ROOT),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

        assert result.returncode == 0, result.stderr
        assert "industrial_delivery_rehearsal_status=ready" in result.stdout
        assert "release_rehearsal_gate=ready" in result.stdout

        report_path = output_root / "release_rehearsal_report.json"
        assert report_path.exists()
        report_payload = json.loads(report_path.read_text(encoding="utf-8"))
        assert report_payload["status"] == "passed"
        assert report_payload["customer_delivery_status"] == "ready"
        assert report_payload["industrial_delivery_status"] == "ready"
        assert report_payload["extension_execution_plan"]["status"] == "ready"
        assert report_payload["extension_execution_instance"]["status"] == "ready"
        assert report_payload["extension_execution_schedule"]["status"] == "ready"
        assert report_payload["extension_execution_actuals"]["status"] == "ready"
        assert report_payload["security_release_preflight"]["status"] == "passed"
        assert report_payload["industrial_manifest"]["status"] == "ready"
        assert report_payload["industrial_release_readiness"]["status"] == "ready"
        assert report_payload["industrial_promotion_checklist"]["status"] == "ready"
        assert report_payload["industrial_customer_acceptance_bundle"]["status"] == "ready"
        assert (
            report_payload["industrial_customer_acceptance_bundle"][
                "vulnerability_exception_review"
            ]["status"]
            == "passed"
        )
        assert (
            report_payload["industrial_customer_acceptance_bundle"][
                "external_mainline_execution_plan"
            ]["status"]
            == "ready"
        )
        assert len(report_payload["delivery_rehearsal_stages"]) == 6
        assert (
            report_payload["extension_execution_plan"]["profiles"][0]["execution_template"][
                "rollback_owner_role"
            ]
            == "rollback_owner"
        )
        assert (
            report_payload["extension_execution_plan"]["profiles"][0]["execution_template"][
                "handoff_owner_role"
            ]
            == "delivery_lead"
        )
        assert (
            report_payload["extension_execution_plan"]["profiles"][0]["execution_template"][
                "residual_risk_owner_role"
            ]
            == "delivery_lead"
        )

        manifest_path = output_root / "release_manifest.json"
        assert manifest_path.exists()
        manifest_payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        assert manifest_payload["customer_delivery_surface"]["status"] == "ready"
        assert manifest_payload["industrial_delivery_gate"]["status"] == "ready"
        industrial_manifest_path = output_root / "release_manifest_industrial.json"
        assert industrial_manifest_path.exists()
        industrial_bundle_path = (
            output_root
            / "test_env"
            / "release"
            / "customer_acceptance_bundle_industrial.json"
        )
        assert industrial_bundle_path.exists()
        industrial_delivery_rehearsal_report_path = (
            output_root / "industrial_delivery_rehearsal_report.json"
        )
        assert industrial_delivery_rehearsal_report_path.exists()
    finally:
        shutil.rmtree(output_root, ignore_errors=True)
