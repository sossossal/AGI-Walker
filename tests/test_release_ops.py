from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

from agi_walker.core.api.release_contracts import validate_release_evidence_report
from agi_walker.core.api.release_ops_contracts import (
    DEFAULT_RELEASE_OP_POLICY_PROFILE,
    ReleaseOpRequest,
    ReleaseOpSessionContext,
)
from agi_walker.ops.release_ops import (
    execute_release_op,
    list_release_ops_actions,
    list_release_ops_policy_profiles,
    list_release_ops_request_templates,
)
from tests.test_stable_promotion_checklist import (
    _init_git_repo,
    _seed_clean_checkout_evidence,
    _seed_customer_delivery_docs,
    _seed_industrial_security_reports,
    _seed_live_evidence,
    _seed_security_preflight_report,
    _seed_structured_required_evidence,
    _write_ready_stable_manifest,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _seed_ready_stable_promotion_project(tmp_path: Path) -> tuple[Path, Path, Path]:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _ = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    approval_manifest = _write_ready_stable_manifest(project_root, source_root)
    return project_root, source_root, approval_manifest


def test_list_release_ops_actions_exposes_policy_levels() -> None:
    actions = {item["action"]: item for item in list_release_ops_actions()}

    assert actions["stable_promotion_checklist"]["policy_level"] == "local_safe_refresh"
    assert actions["external_mainline_execution"]["policy_level"] == "requires_attestation"
    assert actions["release_rehearsal"]["request_type"] == "ReleaseRehearsalRequest"


def test_list_release_ops_policy_profiles_exposes_default_profile() -> None:
    profiles = {
        item["policy_profile"]: item for item in list_release_ops_policy_profiles()
    }

    assert profiles["read_only"]["default"] == "false"
    assert profiles["local_safe_refresh"]["default"] == "true"
    assert "deterministic local report refresh" in profiles["local_safe_refresh"][
        "description"
    ]


def test_list_release_ops_request_templates_exposes_defaults() -> None:
    templates = {
        item["action"]: item for item in list_release_ops_request_templates()
    }

    external_mainline = templates["external_mainline_execution"]
    assert external_mainline["policy_level"] == "requires_attestation"
    assert external_mainline["default_policy_profile"] == "local_safe_refresh"
    assert "project_root" in external_mainline["required_fields"]
    assert external_mainline["request_template"]["output"].endswith(
        "external_mainline_execution_plan.json"
    )


def test_execute_release_op_dispatches_stable_promotion_checklist(
    tmp_path: Path,
) -> None:
    project_root, source_root, approval_manifest = _seed_ready_stable_promotion_project(
        tmp_path
    )

    result = execute_release_op(
        ReleaseOpRequest(
            action="stable_promotion_checklist",
            request={
                "current_version": "2026.04.12-rc6",
                "stable_version": "2026.04.12",
                "project_root": str(project_root),
                "source_root": str(source_root),
                "approval_manifest": str(approval_manifest),
                "security_preflight_report": str(
                    project_root
                    / "test_env"
                    / "release_evidence"
                    / "security_release_preflight_report.json"
                ),
            },
        )
    )

    assert result.action == "stable_promotion_checklist"
    assert result.policy_level == "local_safe_refresh"
    assert result.policy_profile == DEFAULT_RELEASE_OP_POLICY_PROFILE
    assert result.request_type == "StablePromotionChecklistRequest"
    assert result.status == "ready"
    assert result.output_path is not None and result.output_path.exists()
    assert result.payload["artifact_type"] == "stable_promotion_checklist"
    assert result.payload["ready_to_promote"] is True


def test_execute_release_op_writes_event_stream_and_session_context(
    tmp_path: Path,
) -> None:
    project_root, source_root, approval_manifest = _seed_ready_stable_promotion_project(
        tmp_path / "event_stream"
    )
    event_stream_file = tmp_path / "release_ops.jsonl"

    result = execute_release_op(
        ReleaseOpRequest(
            action="stable_promotion_checklist",
            request={
                "current_version": "2026.04.12-rc6",
                "stable_version": "2026.04.12",
                "project_root": str(project_root),
                "source_root": str(source_root),
                "approval_manifest": str(approval_manifest),
                "security_preflight_report": str(
                    project_root
                    / "test_env"
                    / "release_evidence"
                    / "security_release_preflight_report.json"
                ),
            },
            session=ReleaseOpSessionContext(
                engagement_id="eng-20260421",
                window_id="window-rc",
                change_ticket="CHG-4242",
                channel="stable",
            ),
            event_stream_file=str(event_stream_file),
        )
    )

    assert result.session == {
        "engagement_id": "eng-20260421",
        "window_id": "window-rc",
        "change_ticket": "CHG-4242",
        "channel": "stable",
    }
    assert result.event_stream_path == event_stream_file
    assert result.event_count == 3
    events = [
        json.loads(line)
        for line in event_stream_file.read_text(encoding="utf-8").splitlines()
    ]
    assert [event["event_type"] for event in events] == [
        "action_started",
        "action_completed",
        "artifact_written",
    ]
    assert events[0]["session"]["engagement_id"] == "eng-20260421"
    assert events[-1]["payload"]["output_path"] == str(result.output_path)
    saved_payload = json.loads(result.output_path.read_text(encoding="utf-8"))
    assert saved_payload["control_plane_session"]["engagement_id"] == "eng-20260421"
    assert saved_payload["control_plane_event_stream"]["event_count"] == result.event_count


def test_execute_release_op_writes_canonical_evidence_report(
    tmp_path: Path,
) -> None:
    project_root, source_root, approval_manifest = _seed_ready_stable_promotion_project(
        tmp_path / "evidence_report"
    )
    evidence_report_file = tmp_path / "release_ops_execution_report.json"

    result = execute_release_op(
        ReleaseOpRequest(
            action="stable_promotion_checklist",
            request={
                "current_version": "2026.04.12-rc6",
                "stable_version": "2026.04.12",
                "project_root": str(project_root),
                "source_root": str(source_root),
                "approval_manifest": str(approval_manifest),
                "security_preflight_report": str(
                    project_root
                    / "test_env"
                    / "release_evidence"
                    / "security_release_preflight_report.json"
                ),
            },
            session=ReleaseOpSessionContext(
                engagement_id="eng-evidence",
                window_id="window-evidence",
                change_ticket="CHG-EVIDENCE",
                channel="stable",
            ),
            event_stream_file=str(tmp_path / "release_ops_execution.jsonl"),
            evidence_report_file=str(evidence_report_file),
        )
    )

    assert result.evidence_report_path == evidence_report_file
    assert result.evidence_report_payload["artifact_type"] == "release_evidence_report"
    assert validate_release_evidence_report(result.evidence_report_payload) == []
    saved_payload = json.loads(evidence_report_file.read_text(encoding="utf-8"))
    assert validate_release_evidence_report(saved_payload) == []
    assert saved_payload["evidence_name"] == "release_ops_execution"
    assert saved_payload["metrics"]["action"] == "stable_promotion_checklist"
    assert saved_payload["metrics"]["request_type"] == "StablePromotionChecklistRequest"
    assert saved_payload["metrics"]["event_count"] == result.event_count
    assert saved_payload["control_plane_session"]["engagement_id"] == "eng-evidence"
    assert saved_payload["control_plane_event_stream"]["event_count"] == result.event_count


def test_execute_release_op_rewrites_release_readiness_with_control_plane_metadata(
    tmp_path: Path,
) -> None:
    project_root, source_root, approval_manifest = _seed_ready_stable_promotion_project(
        tmp_path / "readiness_action"
    )

    result = execute_release_op(
        ReleaseOpRequest(
            action="release_readiness",
            request={
                "current_version": "2026.04.12-rc6",
                "stable_version": "2026.04.12",
                "project_root": str(project_root),
                "source_root": str(source_root),
                "approval_manifest": str(approval_manifest),
                "security_preflight_report": str(
                    project_root
                    / "test_env"
                    / "release_evidence"
                    / "security_release_preflight_report.json"
                ),
            },
            session=ReleaseOpSessionContext(
                engagement_id="eng-readiness-op",
                window_id="window-readiness-op",
                change_ticket="CHG-READINESS-OP",
                channel="stable",
            ),
            event_stream_file=str(tmp_path / "release_readiness.jsonl"),
        )
    )

    saved_payload = json.loads(result.output_path.read_text(encoding="utf-8"))
    assert saved_payload["artifact_type"] == "release_readiness_report"
    assert saved_payload["control_plane_session"]["engagement_id"] == "eng-readiness-op"
    assert saved_payload["control_plane_event_stream"]["event_count"] == result.event_count


def test_execute_release_op_rejects_unknown_action() -> None:
    with pytest.raises(ValueError, match="unsupported release op action"):
        execute_release_op(ReleaseOpRequest(action="unknown_action"))


def test_execute_release_op_enforces_policy_profile(tmp_path: Path) -> None:
    event_stream_file = tmp_path / "release_ops_denied.jsonl"
    with pytest.raises(ValueError, match="requires policy profile requires_attestation"):
        execute_release_op(
            ReleaseOpRequest(
                action="external_mainline_execution",
                session=ReleaseOpSessionContext(engagement_id="eng-denied"),
                event_stream_file=str(event_stream_file),
            )
        )
    events = [
        json.loads(line)
        for line in event_stream_file.read_text(encoding="utf-8").splitlines()
    ]
    assert [event["event_type"] for event in events] == [
        "action_started",
        "policy_denied",
    ]
    assert events[-1]["payload"]["required_policy_level"] == "requires_attestation"


def test_execute_release_op_rewrites_external_mainline_plan_with_control_plane_metadata(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    request_file = project_root / "deployment" / "external_mainline.inputs.json"
    request_file.parent.mkdir(parents=True, exist_ok=True)
    request_file.write_text(
        json.dumps(
            {
                "customer_external_bindings": {"enabled": False},
                "vulnerability_exception_review": {"enabled": False},
                "industrial_rehearsal": {"refresh": False},
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    result = execute_release_op(
        ReleaseOpRequest(
            action="external_mainline_execution",
            request={
                "project_root": str(project_root),
                "inputs_file": "deployment/external_mainline.inputs.json",
                "skip_managed_inputs": True,
                "skip_customer_external_bindings_closure": True,
                "skip_vulnerability_exception_review_refresh": True,
                "output": (
                    "test_env/release_evidence/operations/"
                    "external_mainline_execution_plan.json"
                ),
                "external_mainline_input_checklist_report": (
                    "test_env/release_evidence/operations/"
                    "external_mainline_input_checklist_report.json"
                ),
                "customer_config": "deployment/customer_delivery.external_bindings.customer.json",
                "customer_external_bindings_closure_report": (
                    "test_env/release_evidence/operations/"
                    "customer_external_bindings_closure_report.json"
                ),
                "vulnerability_exception_review_report": (
                    "test_env/release_evidence/security/"
                    "vulnerability_exception_review_report.json"
                ),
                "industrial_delivery_rehearsal_report": (
                    "test_env/release_rehearsal_industrial/"
                    "industrial_delivery_rehearsal_report.json"
                ),
            },
            policy_profile="requires_attestation",
            session=ReleaseOpSessionContext(
                engagement_id="eng-mainline",
                window_id="window-mainline",
                change_ticket="CHG-MAINLINE",
                channel="industrial",
            ),
            event_stream_file=str(tmp_path / "external_mainline.jsonl"),
        )
    )

    saved_payload = json.loads(result.output_path.read_text(encoding="utf-8"))
    assert saved_payload["control_plane_session"]["engagement_id"] == "eng-mainline"
    assert saved_payload["control_plane_event_stream"]["event_count"] == result.event_count
    checklist_payload = json.loads(
        (
            project_root
            / "test_env"
            / "release_evidence"
            / "operations"
            / "external_mainline_input_checklist_report.json"
        ).read_text(encoding="utf-8")
    )
    assert checklist_payload["control_plane_session"]["engagement_id"] == "eng-mainline"
    assert checklist_payload["control_plane_event_stream"]["event_count"] == result.event_count


def test_execute_release_op_rewrites_worktree_release_blocker_with_control_plane_metadata(
    tmp_path: Path,
) -> None:
    source_root, _ = _init_git_repo(tmp_path / "worktree_source", tag="2026.04.12")
    result = execute_release_op(
        ReleaseOpRequest(
            action="worktree_release_blocker",
            request={
                "source_root": str(source_root),
                "output_root": str(tmp_path / "worktree_release_blocker"),
            },
            session=ReleaseOpSessionContext(
                engagement_id="eng-worktree",
                window_id="window-worktree",
                change_ticket="CHG-WORKTREE",
                channel="stable",
            ),
            event_stream_file=str(tmp_path / "worktree_release_blocker.jsonl"),
        )
    )

    saved_payload = json.loads(result.output_path.read_text(encoding="utf-8"))
    assert saved_payload["artifact_type"] == "worktree_release_blocker_report"
    assert saved_payload["control_plane_session"]["engagement_id"] == "eng-worktree"
    assert saved_payload["control_plane_event_stream"]["event_count"] == result.event_count


def test_run_release_ops_cli_lists_actions_and_executes_action(tmp_path: Path) -> None:
    actions_file = tmp_path / "release_ops_actions.json"
    list_result = subprocess.run(
        [
            sys.executable,
            "tools/run_release_ops.py",
            "--list-actions",
            "--result-file",
            str(actions_file),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert list_result.returncode == 0, list_result.stderr
    assert "release_ops_actions=" in list_result.stdout
    actions_payload = json.loads(actions_file.read_text(encoding="utf-8"))
    assert any(item["action"] == "stable_promotion_checklist" for item in actions_payload)

    profiles_file = tmp_path / "release_ops_policy_profiles.json"
    profiles_result = subprocess.run(
        [
            sys.executable,
            "tools/run_release_ops.py",
            "--list-policy-profiles",
            "--result-file",
            str(profiles_file),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert profiles_result.returncode == 0, profiles_result.stderr
    assert "release_ops_policy_profiles=" in profiles_result.stdout
    profiles_payload = json.loads(profiles_file.read_text(encoding="utf-8"))
    assert any(
        item["policy_profile"] == DEFAULT_RELEASE_OP_POLICY_PROFILE
        for item in profiles_payload
    )

    project_root, source_root, approval_manifest = _seed_ready_stable_promotion_project(
        tmp_path / "action_run"
    )
    request_file = tmp_path / "stable_promotion_request.json"
    request_file.write_text(
        json.dumps(
            {
                "current_version": "2026.04.12-rc6",
                "stable_version": "2026.04.12",
                "project_root": str(project_root),
                "source_root": str(source_root),
                "approval_manifest": str(approval_manifest),
                "security_preflight_report": str(
                    project_root
                    / "test_env"
                    / "release_evidence"
                    / "security_release_preflight_report.json"
                ),
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    result_file = tmp_path / "stable_promotion_result.json"
    evidence_report_file = tmp_path / "stable_promotion_evidence.json"
    action_result = subprocess.run(
        [
            sys.executable,
            "tools/run_release_ops.py",
            "stable_promotion_checklist",
            "--engagement-id",
            "eng-cli",
            "--window-id",
            "window-cli",
            "--change-ticket",
            "CHG-9000",
            "--channel",
            "stable",
            "--event-stream-file",
            str(tmp_path / "stable_promotion_events.jsonl"),
            "--evidence-report-file",
            str(evidence_report_file),
            "--request-file",
            str(request_file),
            "--result-file",
            str(result_file),
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert action_result.returncode == 0, action_result.stderr
    assert "release_op_action=stable_promotion_checklist" in action_result.stdout
    assert "release_op_policy_level=local_safe_refresh" in action_result.stdout
    assert (
        f"release_op_policy_profile={DEFAULT_RELEASE_OP_POLICY_PROFILE}"
        in action_result.stdout
    )
    assert "release_op_session_context=" in action_result.stdout
    assert "release_op_event_stream_path=" in action_result.stdout
    assert "release_op_event_count=3" in action_result.stdout
    assert f"release_op_evidence_report_path={evidence_report_file}" in action_result.stdout
    payload = json.loads(result_file.read_text(encoding="utf-8"))
    assert payload["action"] == "stable_promotion_checklist"
    assert payload["policy_level"] == "local_safe_refresh"
    assert payload["policy_profile"] == DEFAULT_RELEASE_OP_POLICY_PROFILE
    assert payload["session"]["engagement_id"] == "eng-cli"
    assert payload["event_count"] == 3
    assert payload["status"] == "ready"
    assert payload["payload"]["ready_to_promote"] is True
    assert payload["evidence_report_path"] == str(evidence_report_file)
    assert payload["evidence_report_payload"]["evidence_name"] == "release_ops_execution"
    evidence_payload = json.loads(evidence_report_file.read_text(encoding="utf-8"))
    assert validate_release_evidence_report(evidence_payload) == []
    assert evidence_payload["metrics"]["action"] == "stable_promotion_checklist"

    denied_result = subprocess.run(
        [
            sys.executable,
            "tools/run_release_ops.py",
            "external_mainline_execution",
        ],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert denied_result.returncode != 0
    assert "requires policy profile requires_attestation" in denied_result.stderr
