from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_external_mainline_execution_plan_artifact,
    build_extension_execution_actuals_artifact,
    build_release_manifest_artifact,
    default_external_mainline_execution_plan_path,
    write_external_mainline_execution_plan_artifact,
    write_extension_execution_actuals_artifact,
    write_release_manifest_artifact,
)
from tests.test_release_readiness import _seed_customer_external_bindings_closure_report
from tests.test_release_readiness import _seed_external_mainline_input_checklist
from tests.test_stable_promotion_checklist import (
    _init_git_repo,
    _seed_clean_checkout_evidence,
    _seed_customer_delivery_docs,
    _seed_industrial_security_reports,
    _seed_live_evidence,
    _seed_security_preflight_report,
    _seed_structured_required_evidence,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _write_ready_industrial_manifest(project_root: Path, source_root: Path) -> Path:
    manifest_path = (
        project_root / "test_env" / "release" / "release_manifest_industrial.json"
    )
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    payload = build_release_manifest_artifact(
        build_id="build-20260412-industrial",
        version="2026.04.12",
        channel="industrial",
        release_summary="industrial signoff",
        generated_at="2026-04-12T12:31:00+00:00",
        release_approval={
            "status": "approved",
            "approved_by": "release-manager",
            "approved_at": "2026-04-12T12:30:00+00:00",
            "notes": "industrial signoff",
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


def test_industrial_promotion_checklist_blocks_on_delivery_surfaces(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root, head = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    report_path = (
        project_root
        / "test_env"
        / "industrial_promotion"
        / "industrial_promotion_checklist.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_promotion_checklist.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
            "--approval-status",
            "approved",
            "--approved-by",
            "release-manager",
            "--approved-at",
            "2026-04-12T12:30:00+00:00",
            "--commit-sha",
            head,
            "--approval-notes",
            "industrial signoff",
            "--output-root",
            str(report_path.parent),
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
    assert "industrial_promotion_checklist_written=" in result.stdout
    assert "industrial_promotion_gate=blocked" in result.stdout
    assert "industrial_promotion_blocking_steps=2" in result.stdout
    assert "industrial_promotion_ready_to_promote=false" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "industrial_promotion_checklist"
    assert payload["blocking_steps"] == 2
    assert payload["ready_to_promote"] is False
    assert payload["customer_delivery_surface"]["extension_support_surface"]["status"] == "blocked"
    assert payload["industrial_delivery_gate"]["extension_support_surface_status"] == "blocked"
    assert payload["extension_execution_plan"]["status"] == "blocked"
    assert payload["extension_execution_instance"]["status"] == "blocked"
    assert payload["extension_execution_schedule"]["status"] == "blocked"
    assert payload["extension_execution_actuals"]["status"] == "blocked"
    steps = {item["id"]: item for item in payload["steps"]}
    customer_delivery = steps["customer_delivery_surface"]
    industrial_delivery = steps["industrial_delivery_gate"]
    extension_execution = steps["extension_execution_plan"]
    assert customer_delivery["required"] is True
    assert customer_delivery["blocking"] is True
    assert customer_delivery["status"] == "pending"
    assert industrial_delivery["required"] is True
    assert industrial_delivery["blocking"] is True
    assert industrial_delivery["status"] == "pending"
    assert extension_execution["blocking"] is False
    assert extension_execution["status"] == "pending"
    assert steps["extension_execution_instance"]["blocking"] is False
    assert steps["extension_execution_instance"]["status"] == "pending"
    assert steps["extension_execution_schedule"]["blocking"] is False
    assert steps["extension_execution_schedule"]["status"] == "pending"
    assert steps["extension_execution_actuals"]["blocking"] is False
    assert steps["extension_execution_actuals"]["status"] == "pending"


def test_industrial_promotion_checklist_reports_worktree_release_blocker_for_dirty_repo(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _ = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    approval_manifest = _write_ready_industrial_manifest(project_root, source_root)
    (source_root / "README.md").write_text("# dirty repo\n", encoding="utf-8")
    report_path = (
        project_root
        / "test_env"
        / "industrial_promotion"
        / "industrial_promotion_checklist_dirty_worktree.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_promotion_checklist.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
            "--approval-manifest",
            str(approval_manifest),
            "--security-preflight-report",
            str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
            "--output-root",
            str(report_path.parent),
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
    assert "industrial_worktree_release_blocker=blocked" in result.stdout
    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["worktree_release_blocker"]["status"] == "blocked"
    steps = {step["id"]: step for step in payload["steps"]}
    assert steps["clean_worktree"]["status"] == "pending"
    assert "tools/run_worktree_release_blocker.py" in steps["clean_worktree"]["command"]
    assert "worktree release blocker" in steps["clean_worktree"]["summary"]


def test_industrial_promotion_checklist_reports_ready_to_promote(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _ = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    approval_manifest = _write_ready_industrial_manifest(project_root, source_root)
    report_path = (
        project_root
        / "test_env"
        / "industrial_promotion"
        / "industrial_promotion_checklist_ready.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_promotion_checklist.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
            "--approval-manifest",
            str(approval_manifest),
            "--security-preflight-report",
            str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
            "--output-root",
            str(report_path.parent),
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
    assert "industrial_promotion_gate=ready" in result.stdout
    assert "industrial_promotion_blocking_steps=0" in result.stdout
    assert "industrial_promotion_ready_to_promote=true" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "industrial_promotion_checklist"
    assert payload["approval_manifest_path"] == str(approval_manifest)
    assert payload["blocking_steps"] == 0
    assert payload["ready_to_promote"] is True
    assert payload["customer_delivery_surface"]["extension_support_surface"]["declared_profiles"] == 4
    assert payload["industrial_delivery_gate"]["declared_extension_profiles"] == 4
    assert payload["extension_execution_plan"]["status"] == "ready"
    assert payload["extension_execution_plan"]["actionable_profiles"] == 3
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
    kubernetes_profile = next(
        item
        for item in payload["extension_execution_plan"]["profiles"]
        if item["id"] == "kubernetes_delivery"
    )
    assert kubernetes_profile["status"] == "not_supported"
    assert "PRODUCTION_DEPLOYMENT_RUNBOOK.md" in kubernetes_profile["runbook_entrypoints"]
    assert kubernetes_profile["execution_template"]["rollback_owner_role"] == "rollback_owner"
    assert kubernetes_profile["execution_template"]["handoff_owner_role"] == "delivery_lead"
    assert kubernetes_profile["execution_template"]["watch_owner_role"] == "delivery_lead"
    assert kubernetes_profile["execution_template"]["on_call_handoff_owner_role"] == "delivery_lead"
    assert kubernetes_profile["execution_template"]["exception_review_owner_role"] == "delivery_lead"
    assert kubernetes_profile["execution_template"]["escalation_closure_owner_role"] == "delivery_lead"
    assert kubernetes_profile["execution_template"]["operator_roles"][0]["id"] == "delivery_lead"
    assert (
        kubernetes_profile["execution_template"]["signoff_checkpoints"][0][
            "required_artifact"
        ]
        == "docs/guides/SUPPORT_MATRIX.md"
    )
    assert (
        kubernetes_profile["execution_template"]["incident_escalation_steps"][0][
            "escalation_target"
        ]
        == "docs/guides/INCIDENT_RESPONSE_MATRIX.md"
    )
    assert (
        kubernetes_profile["execution_template"]["escalation_closure_steps"][0][
            "required_artifact"
        ]
        == "test_env/industrial_promotion_ready/industrial_promotion_checklist.json"
    )
    assert kubernetes_profile["deployment_commands"][0].startswith("No supported deployment command")
    assert "Docker Compose baseline" in kubernetes_profile["rollback_prerequisites"][0]
    steps = {item["id"]: item for item in payload["steps"]}
    assert steps["customer_delivery_surface"]["status"] == "done"
    assert steps["customer_delivery_surface"]["blocking"] is False
    assert steps["industrial_delivery_gate"]["status"] == "done"
    assert steps["industrial_delivery_gate"]["blocking"] is False
    assert steps["extension_execution_plan"]["status"] == "done"
    assert steps["extension_execution_plan"]["blocking"] is False
    assert steps["extension_execution_evidence"]["status"] == "done"
    assert steps["extension_execution_evidence"]["blocking"] is False
    assert steps["extension_execution_instance"]["status"] == "done"
    assert steps["extension_execution_instance"]["blocking"] is False
    assert steps["extension_execution_schedule"]["status"] == "done"
    assert steps["extension_execution_schedule"]["blocking"] is False
    assert steps["extension_execution_actuals"]["status"] == "done"
    assert steps["extension_execution_actuals"]["blocking"] is False
    assert "值班风险闭环证据" in steps["extension_execution_plan"]["title"]
    assert "扩展执行留痕报告" in steps["extension_execution_evidence"]["title"]
    assert "客户实例化扩展执行面" in steps["extension_execution_instance"]["title"]
    assert "客户窗口排程与 closure 归档面" in steps["extension_execution_schedule"]["title"]
    assert "客户窗口执行留痕" in steps["extension_execution_actuals"]["title"]
    assert steps["build_industrial_manifest"]["status"] == "done"


def test_industrial_promotion_checklist_surfaces_non_blocking_exception_review_step(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _ = _init_git_repo(tmp_path, tag="2026.04.12")
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
    approval_manifest = _write_ready_industrial_manifest(project_root, source_root)
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
    report_path = (
        project_root
        / "test_env"
        / "industrial_promotion"
        / "industrial_promotion_checklist_review_due.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_promotion_checklist.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
            "--approval-manifest",
            str(approval_manifest),
            "--security-preflight-report",
            str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
            "--output-root",
            str(report_path.parent),
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
    assert "industrial_promotion_gate=ready" in result.stdout
    assert "industrial_promotion_blocking_steps=0" in result.stdout

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


def test_industrial_promotion_checklist_surfaces_external_mainline_step(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _ = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    _seed_external_mainline_execution_plan(project_root)
    checklist_report = _seed_external_mainline_input_checklist(project_root)
    approval_manifest = _write_ready_industrial_manifest(project_root, source_root)
    report_path = (
        project_root
        / "test_env"
        / "industrial_promotion"
        / "industrial_promotion_checklist_external_mainline.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_promotion_checklist.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
            "--approval-manifest",
            str(approval_manifest),
            "--security-preflight-report",
            str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
            "--output-root",
            str(report_path.parent),
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
    assert "industrial_external_mainline_execution_plan=ready/0/2/1/0" in result.stdout
    checklist_payload = json.loads(checklist_report.read_text(encoding="utf-8"))
    assert (
        "industrial_external_mainline_input_checklist="
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


def test_industrial_promotion_checklist_surfaces_confirm_action_for_existing_customer_bindings(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, head = _init_git_repo(tmp_path, tag="2026.04.12")
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
                    "binding_state": "draft",
                    "source_type": "customer_ticket_registry",
                    "source_path": "test_env/release_delivery/customer_001/approval_identity_source.json",
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
        / "industrial_promotion"
        / "industrial_promotion_checklist_external_bindings.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_promotion_checklist.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
            "--approval-status",
            "approved",
            "--approved-by",
            "release-manager",
            "--approved-at",
            "2026-04-12T12:30:00+00:00",
            "--commit-sha",
            head,
            "--approval-notes",
            "industrial signoff",
            "--output-root",
            str(report_path.parent),
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
    steps = {step["id"]: step for step in payload["steps"]}
    bindings_step = steps["extension_external_bindings"]
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


def test_industrial_promotion_checklist_surfaces_stale_exception_step(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _ = _init_git_repo(tmp_path, tag="2026.04.12")
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
    approval_manifest = _write_ready_industrial_manifest(project_root, source_root)
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
    report_path = (
        project_root
        / "test_env"
        / "industrial_promotion"
        / "industrial_promotion_checklist_stale.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_promotion_checklist.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
            "--approval-manifest",
            str(approval_manifest),
            "--security-preflight-report",
            str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
            "--output-root",
            str(report_path.parent),
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
