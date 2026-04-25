from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_industrial_delivery_rehearsal_report_artifact,
    validate_industrial_delivery_rehearsal_report_artifact,
    write_industrial_delivery_rehearsal_report_artifact,
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
            "summary": "external mainline execution plan ready.",
            "report_path": (
                "test_env/release_evidence/operations/"
                "external_mainline_execution_plan.json"
            ),
            "completed_steps": 0,
            "ready_to_run_steps": 0,
            "waiting_external_input_steps": 3,
            "blocked_steps": 0,
            "control_plane_session": {
                "engagement_id": "release-rehearsal-industrial",
                "window_id": "2026.04.17-industrial-rehearsal-external-mainline",
                "change_ticket": "release-rehearsal-industrial-change",
                "channel": "industrial",
            },
            "control_plane_event_stream": {
                "path": "test_env/release_ops/release_rehearsal_external_mainline.jsonl",
                "event_count": 3,
            },
        },
        "external_mainline_input_checklist": {
            "status": "blocked",
            "summary": (
                "external mainline input checklist blocked: customer_missing=2, "
                "vulnerability_missing=1, industrial_missing=1, waiting_steps=3, "
                "ready_steps=0, completed_steps=0."
            ),
            "report_path": (
                "test_env/release_evidence/operations/"
                "external_mainline_input_checklist_report.json"
            ),
            "missing_input_count": 4,
            "waiting_external_input_steps": [
                "customer_external_bindings_closure",
                "vulnerability_exception_replacement",
                "industrial_delivery_live_evidence",
            ],
            "ready_to_run_steps": [],
            "completed_steps": [],
            "control_plane_session": {
                "engagement_id": "release-rehearsal-industrial",
                "window_id": "2026.04.17-industrial-rehearsal-external-mainline-inputs",
                "change_ticket": "release-rehearsal-industrial-change",
                "channel": "industrial",
            },
            "control_plane_event_stream": {
                "path": "test_env/release_ops/release_rehearsal_external_mainline.jsonl",
                "event_count": 3,
            },
        },
        "release_ops_execution": {
            "status": "passed",
            "summary": "release op external_mainline_execution completed via rehearsal control plane evidence wrapper.",
            "report_path": (
                "test_env/release_evidence/operations/"
                "release_ops_execution_report.json"
            ),
            "event_count": 3,
            "action": "external_mainline_execution",
            "policy_level": "requires_attestation",
            "policy_profile": "requires_attestation",
            "request_type": "ExternalMainlineExecutionRequest",
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
            },
            {
                "name": "industrial_release_readiness_report.json",
                "path": "test_env/industrial_release_readiness_ready/industrial_release_readiness_report.json",
            },
            {
                "name": "industrial_promotion_checklist.json",
                "path": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
            },
            {
                "name": "customer_acceptance_bundle_industrial.json",
                "path": "test_env/release/customer_acceptance_bundle_industrial.json",
            },
        ],
        "delivery_rehearsal_stages": [
            _stage("new_environment_install"),
            _stage("smoke"),
            _stage("live_evidence"),
            _stage("upgrade"),
            _stage("rollback"),
            _stage("backup_restore"),
        ],
    }


def test_build_industrial_delivery_rehearsal_report_artifact_ready() -> None:
    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=_ready_release_rehearsal_report(),
        release_rehearsal_report_path="test_env/release_rehearsal/release_rehearsal_report.json",
    )

    assert payload["artifact_type"] == "industrial_delivery_rehearsal_report"
    assert payload["status"] == "ready"
    assert payload["stage_summary"] == {"total": 6, "passed": 6, "failed": 0}
    assert payload["control_plane_session"]["engagement_id"] == "release-rehearsal-industrial"
    assert payload["control_plane_event_stream"]["event_count"] == 3
    assert payload["vulnerability_exception_review"]["status"] == "passed"
    assert payload["vulnerability_exception_review"]["review_candidate_count"] == 31
    assert payload["customer_external_bindings_closure"]["status"] == "passed"
    assert payload["external_mainline_execution_plan"]["status"] == "ready"
    assert payload["external_mainline_execution_plan"]["waiting_external_input_steps"] == 3
    assert (
        payload["external_mainline_execution_plan"]["control_plane_session"][
            "engagement_id"
        ]
        == "release-rehearsal-industrial"
    )
    assert (
        payload["external_mainline_execution_plan"]["control_plane_event_stream"][
            "event_count"
        ]
        == 3
    )
    assert payload["external_mainline_input_checklist"]["status"] == "blocked"
    assert payload["external_mainline_input_checklist"]["missing_input_count"] == 4
    assert (
        payload["external_mainline_input_checklist"]["control_plane_session"][
            "engagement_id"
        ]
        == "release-rehearsal-industrial"
    )
    assert (
        payload["external_mainline_input_checklist"]["control_plane_event_stream"][
            "event_count"
        ]
        == 3
    )
    assert payload["release_ops_execution"]["status"] == "passed"
    assert payload["release_ops_execution"]["event_count"] == 3
    assert "external_mainline_input_checklist=blocked/4/3/0/0" in payload["summary"]
    assert "release_ops_execution=passed/3" in payload["summary"]
    assert "control_plane_events=3" in payload["summary"]
    assert payload["industrial_manifest"]["status"] == "ready"
    assert payload["industrial_customer_acceptance_bundle"]["status"] == "ready"
    assert [item["id"] for item in payload["delivery_rehearsal_stages"]] == [
        "new_environment_install",
        "smoke",
        "live_evidence",
        "upgrade",
        "rollback",
        "backup_restore",
    ]
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_normalizes_missing_stage_data() -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report["status"] = "failed"
    rehearsal_report["delivery_rehearsal_stages"] = [{"id": "smoke"}]
    rehearsal_report["industrial_promotion_checklist"] = {}

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path="test_env/release_rehearsal/release_rehearsal_report.json",
    )

    assert payload["status"] == "blocked"
    assert payload["stage_summary"] == {"total": 6, "passed": 0, "failed": 6}
    assert payload["industrial_promotion_checklist"]["status"] == "blocked"
    assert payload["delivery_rehearsal_stages"][0]["id"] == "new_environment_install"
    assert payload["delivery_rehearsal_stages"][1]["id"] == "smoke"
    assert payload["delivery_rehearsal_stages"][1]["status"] == "fail"
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_component_summary() -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report["industrial_manifest"] = {
        "status": "ready",
        "manifest_path": "release_manifest_industrial.json",
    }
    rehearsal_report["industrial_customer_acceptance_bundle"] = {
        "status": "ready",
        "bundle_path": "test_env/release/customer_acceptance_bundle_industrial.json",
    }

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path="test_env/release_rehearsal/release_rehearsal_report.json",
    )

    assert payload["industrial_manifest"]["summary"] == "industrial manifest status=ready."
    assert (
        payload["industrial_customer_acceptance_bundle"]["summary"]
        == "industrial customer acceptance bundle status=ready."
    )
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_review_from_preflight_metrics() -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report.pop("vulnerability_exception_review")

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path="test_env/release_rehearsal/release_rehearsal_report.json",
    )

    assert payload["vulnerability_exception_review"]["status"] == "passed"
    assert payload["vulnerability_exception_review"]["review_candidate_count"] == 31
    assert (
        payload["vulnerability_exception_review"]["summary"]
        == "vulnerability exception review status=passed, candidates=31."
    )
    assert payload["customer_external_bindings_closure"]["status"] == "passed"
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_closure_from_bundle(
    tmp_path: Path,
) -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report.pop("customer_external_bindings_closure")
    bundle_path = (
        tmp_path / "test_env" / "release" / "customer_acceptance_bundle_industrial.json"
    )
    bundle_path.parent.mkdir(parents=True, exist_ok=True)
    bundle_path.write_text(
        json.dumps(
            {
                "acceptance_reports": [
                    {
                        "name": "customer_external_bindings_closure",
                        "status": "passed",
                        "summary": "customer external bindings closure passed.",
                        "path": (
                            "test_env/release_evidence/operations/"
                            "customer_external_bindings_closure_report.json"
                        ),
                        "resolved_report_path": str(
                            tmp_path
                            / "test_env"
                            / "release_evidence"
                            / "operations"
                            / "customer_external_bindings_closure_report.json"
                        ),
                        "metrics": {"failed_steps": []},
                    }
                ]
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    rehearsal_report["industrial_customer_acceptance_bundle"]["bundle_path"] = (
        "test_env/release/customer_acceptance_bundle_industrial.json"
    )

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path=tmp_path / "release_rehearsal_report.json",
    )

    assert payload["customer_external_bindings_closure"]["status"] == "passed"
    assert (
        payload["customer_external_bindings_closure"]["report_path"]
        == str(
            tmp_path
            / "test_env"
            / "release_evidence"
            / "operations"
            / "customer_external_bindings_closure_report.json"
        )
    )
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_external_mainline_from_bundle(
    tmp_path: Path,
) -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report.pop("external_mainline_execution_plan")
    rehearsal_report["industrial_customer_acceptance_bundle"] = {
        "status": "ready",
        "bundle_path": "test_env/release/customer_acceptance_bundle_industrial.json",
        "external_mainline_execution_plan": {
            "status": "ready",
            "summary": "external mainline execution plan ready.",
            "report_path": str(
                tmp_path
                / "test_env"
                / "release_evidence"
                / "operations"
                / "external_mainline_execution_plan.json"
            ),
            "completed_steps": 0,
            "ready_to_run_steps": 0,
            "waiting_external_input_steps": 3,
            "blocked_steps": 0,
        },
    }

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path=tmp_path / "release_rehearsal_report.json",
    )

    assert payload["external_mainline_execution_plan"]["status"] == "ready"
    assert payload["external_mainline_execution_plan"]["waiting_external_input_steps"] == 3
    assert (
        payload["external_mainline_execution_plan"]["report_path"]
        == str(
            tmp_path
            / "test_env"
            / "release_evidence"
            / "operations"
            / "external_mainline_execution_plan.json"
        )
    )
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_external_mainline_from_bundle_top_level(
    tmp_path: Path,
) -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report.pop("external_mainline_execution_plan")
    bundle_path = (
        tmp_path / "test_env" / "release" / "customer_acceptance_bundle_industrial.json"
    )
    bundle_path.parent.mkdir(parents=True, exist_ok=True)
    bundle_path.write_text(
        json.dumps(
            {
                "external_mainline_execution_plan": {
                    "status": "ready",
                    "summary": "external mainline execution plan ready.",
                    "report_path": str(
                        tmp_path
                        / "test_env"
                        / "release_evidence"
                        / "operations"
                        / "external_mainline_execution_plan.json"
                    ),
                    "completed_steps": 0,
                    "ready_to_run_steps": 0,
                    "waiting_external_input_steps": 3,
                    "blocked_steps": 0,
                }
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    rehearsal_report["industrial_customer_acceptance_bundle"]["bundle_path"] = (
        "test_env/release/customer_acceptance_bundle_industrial.json"
    )

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path=tmp_path / "release_rehearsal_report.json",
    )

    assert payload["external_mainline_execution_plan"]["status"] == "ready"
    assert payload["external_mainline_execution_plan"]["waiting_external_input_steps"] == 3
    assert (
        payload["external_mainline_execution_plan"]["report_path"]
        == str(
            tmp_path
            / "test_env"
            / "release_evidence"
            / "operations"
            / "external_mainline_execution_plan.json"
        )
    )
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_release_ops_execution_from_bundle_top_level(
    tmp_path: Path,
) -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report.pop("release_ops_execution")
    bundle_path = (
        tmp_path / "test_env" / "release" / "customer_acceptance_bundle_industrial.json"
    )
    bundle_path.parent.mkdir(parents=True, exist_ok=True)
    bundle_path.write_text(
        json.dumps(
            {
                "release_ops_execution": {
                    "status": "passed",
                    "summary": "release op stable_promotion_checklist completed via control plane with canonical evidence wrapper.",
                    "report_path": str(
                        tmp_path
                        / "test_env"
                        / "release_evidence"
                        / "operations"
                        / "release_ops_execution_report.json"
                    ),
                    "event_count": 3,
                    "action": "stable_promotion_checklist",
                    "policy_level": "local_safe_refresh",
                    "policy_profile": "local_safe_refresh",
                    "request_type": "StablePromotionChecklistRequest",
                }
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    rehearsal_report["industrial_customer_acceptance_bundle"]["bundle_path"] = (
        "test_env/release/customer_acceptance_bundle_industrial.json"
    )

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path=tmp_path / "release_rehearsal_report.json",
    )

    assert payload["release_ops_execution"]["status"] == "passed"
    assert payload["release_ops_execution"]["event_count"] == 3
    assert (
        payload["release_ops_execution"]["report_path"]
        == str(
            tmp_path
            / "test_env"
            / "release_evidence"
            / "operations"
            / "release_ops_execution_report.json"
        )
    )
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_external_mainline_from_bundle_acceptance_reports_fallback(
    tmp_path: Path,
) -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report.pop("external_mainline_execution_plan")
    bundle_path = (
        tmp_path / "test_env" / "release" / "customer_acceptance_bundle_industrial.json"
    )
    bundle_path.parent.mkdir(parents=True, exist_ok=True)
    bundle_path.write_text(
        json.dumps(
            {
                "acceptance_reports": [
                    {
                        "name": "external_mainline_execution_plan",
                        "status": "ready",
                        "summary": "external mainline execution plan ready.",
                        "path": (
                            "test_env/release_evidence/operations/"
                            "external_mainline_execution_plan.json"
                        ),
                        "resolved_report_path": str(
                            tmp_path
                            / "test_env"
                            / "release_evidence"
                            / "operations"
                            / "external_mainline_execution_plan.json"
                        ),
                        "completed_steps": 0,
                        "ready_to_run_steps": 0,
                        "waiting_external_input_steps": 3,
                        "blocked_steps": 0,
                    }
                ]
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    rehearsal_report["industrial_customer_acceptance_bundle"]["bundle_path"] = (
        "test_env/release/customer_acceptance_bundle_industrial.json"
    )

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path=tmp_path / "release_rehearsal_report.json",
    )

    assert payload["external_mainline_execution_plan"]["status"] == "ready"
    assert payload["external_mainline_execution_plan"]["waiting_external_input_steps"] == 3
    assert (
        payload["external_mainline_execution_plan"]["report_path"]
        == str(
            tmp_path
            / "test_env"
            / "release_evidence"
            / "operations"
            / "external_mainline_execution_plan.json"
        )
    )
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_artifact_derives_input_checklist_from_bundle_top_level(
    tmp_path: Path,
) -> None:
    rehearsal_report = _ready_release_rehearsal_report()
    rehearsal_report.pop("external_mainline_input_checklist")
    bundle_path = (
        tmp_path / "test_env" / "release" / "customer_acceptance_bundle_industrial.json"
    )
    bundle_path.parent.mkdir(parents=True, exist_ok=True)
    bundle_path.write_text(
        json.dumps(
            {
                "external_mainline_input_checklist": {
                    "status": "blocked",
                    "summary": "external mainline input checklist blocked.",
                    "report_path": str(
                        tmp_path
                        / "test_env"
                        / "release_evidence"
                        / "operations"
                        / "external_mainline_input_checklist_report.json"
                    ),
                    "metrics": {
                        "missing_input_count": 4,
                        "waiting_external_input_steps": [
                            "customer_external_bindings_closure",
                            "vulnerability_exception_replacement",
                            "industrial_delivery_live_evidence",
                        ],
                        "ready_to_run_steps": [],
                        "completed_steps": [],
                    },
                }
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    rehearsal_report["industrial_customer_acceptance_bundle"]["bundle_path"] = (
        "test_env/release/customer_acceptance_bundle_industrial.json"
    )

    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=rehearsal_report,
        release_rehearsal_report_path=tmp_path / "release_rehearsal_report.json",
    )

    assert payload["external_mainline_input_checklist"]["status"] == "blocked"
    assert payload["external_mainline_input_checklist"]["missing_input_count"] == 4
    assert (
        payload["external_mainline_input_checklist"]["report_path"]
        == str(
            tmp_path
            / "test_env"
            / "release_evidence"
            / "operations"
            / "external_mainline_input_checklist_report.json"
        )
    )
    assert payload["external_mainline_input_checklist"]["waiting_external_input_steps"] == [
        "customer_external_bindings_closure",
        "vulnerability_exception_replacement",
        "industrial_delivery_live_evidence",
    ]
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_build_industrial_delivery_rehearsal_report_cli(tmp_path: Path) -> None:
    rehearsal_report_path = tmp_path / "release_rehearsal_report.json"
    rehearsal_report_path.write_text(
        json.dumps(_ready_release_rehearsal_report(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    output_path = tmp_path / "industrial_delivery_rehearsal_report.json"

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_industrial_delivery_rehearsal_report.py",
            "--rehearsal-report",
            str(rehearsal_report_path),
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
    assert "industrial_delivery_rehearsal_report_written=" in result.stdout
    assert "industrial_delivery_rehearsal_status=ready" in result.stdout
    assert "industrial_delivery_rehearsal_stages=6/6" in result.stdout
    assert "industrial_delivery_vulnerability_exception_review=passed/31" in result.stdout
    assert "industrial_delivery_customer_external_bindings_closure=passed" in result.stdout
    assert "industrial_delivery_external_mainline_execution_plan=ready/0/0/3/0" in result.stdout
    assert "industrial_delivery_external_mainline_input_checklist=blocked/4/3/0/0" in result.stdout
    assert "industrial_delivery_release_ops_execution=passed/3" in result.stdout
    assert "industrial_delivery_control_plane_events=3" in result.stdout
    assert output_path.exists()

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_industrial_delivery_rehearsal_report_artifact(payload) == []


def test_write_industrial_delivery_rehearsal_report_artifact_round_trip(
    tmp_path: Path,
) -> None:
    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=_ready_release_rehearsal_report(),
        release_rehearsal_report_path="test_env/release_rehearsal/release_rehearsal_report.json",
    )

    output_path = write_industrial_delivery_rehearsal_report_artifact(
        payload,
        tmp_path / "industrial_delivery_rehearsal_report.json",
    )

    saved = json.loads(output_path.read_text(encoding="utf-8"))
    assert validate_industrial_delivery_rehearsal_report_artifact(saved) == []
    assert saved["status"] == "ready"
