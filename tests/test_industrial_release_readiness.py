from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_external_mainline_execution_plan_artifact,
    build_release_manifest_artifact,
    default_external_mainline_execution_plan_path,
    write_external_mainline_execution_plan_artifact,
    write_release_manifest_artifact,
)
from tests.test_release_readiness import (
    _init_git_repo,
    _seed_clean_checkout_evidence,
    _seed_customer_delivery_docs,
    _seed_external_mainline_input_checklist,
    _seed_industrial_security_reports,
    _seed_live_evidence,
    _seed_security_preflight_report,
    _seed_structured_required_evidence,
)


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _industrial_exception_review_metrics() -> dict[str, object]:
    return {
        "vulnerability_exception_review_report_status": "passed",
        "vulnerability_exception_review_candidate_count": 31,
        "vulnerability_exception_review_report_path": (
            "test_env/release_evidence/security/"
            "vulnerability_exception_review_report.json"
        ),
        "review_due_vulnerability_exception_ids": [
            "CVE-2026-0001",
            "CVE-2026-0002",
        ],
        "review_due_vulnerability_exception_tickets": [
            "SEC-2026-04-15-WEBPANEL-DIST-NOFIX",
        ],
    }


def _write_ready_industrial_manifest(project_root: Path, source_root: Path) -> Path:
    manifest_path = (
        project_root / "test_env" / "release" / "release_manifest_industrial.json"
    )
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(
        project_root,
        metrics=_industrial_exception_review_metrics(),
    )
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


def test_industrial_release_readiness_reports_missing_tag_and_approval(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root = _init_git_repo(tmp_path, tag=None)
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(
        project_root,
        metrics=_industrial_exception_review_metrics(),
    )
    _seed_industrial_security_reports(project_root)
    _seed_external_mainline_execution_plan(project_root)
    checklist_report = _seed_external_mainline_input_checklist(project_root)
    report_path = (
        project_root
        / "test_env"
        / "industrial_release_readiness"
        / "industrial_readiness.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/check_industrial_release_readiness.py",
            "--project-root",
            str(project_root),
            "--source-root",
            str(source_root),
            "--current-version",
            "2026.04.12",
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
    assert "industrial_release_readiness_written=" in result.stdout
    assert "industrial_release_gate=blocked" in result.stdout
    assert "industrial_security_preflight=passed" in result.stdout
    assert "industrial_vulnerability_exception_review=passed/31" in result.stdout
    assert "industrial_external_mainline_execution_plan=ready/" in result.stdout
    checklist_payload = json.loads(checklist_report.read_text(encoding="utf-8"))
    assert (
        "industrial_external_mainline_input_checklist="
        f"{checklist_payload['status']}"
        in result.stdout
    )
    assert "industrial_worktree_release_blocker=ready/0/0" in result.stdout
    assert "industrial_customer_delivery=ready" in result.stdout
    assert "industrial_industrial_delivery=ready" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["artifact_type"] == "industrial_release_readiness_report"
    assert payload["industrial_release_gate"] == "blocked"
    assert payload["vulnerability_exception_review"]["status"] == "passed"
    assert payload["vulnerability_exception_review"]["review_candidate_count"] == 31
    assert payload["external_mainline_execution_plan"]["status"] == "ready"
    assert (
        payload["external_mainline_input_checklist"]["status"]
        == checklist_payload["status"]
    )
    assert payload["worktree_release_blocker"]["status"] == "ready"
    assert payload["customer_delivery_surface"]["status"] == "ready"
    assert payload["customer_delivery_surface"]["extension_support_surface"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["status"] == "ready"
    assert payload["industrial_delivery_gate"]["extension_support_surface_status"] == "ready"
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
    preview = payload["previews"][0]
    assert preview["channel"] == "industrial"
    assert preview["release_gate_status"] == "blocked"
    assert preview["extension_execution_evidence"]["status"] == "ready"
    assert preview["extension_execution_evidence"]["ready_reports"] == 4
    assert preview["extension_execution_instance"]["status"] == "ready"
    assert preview["extension_execution_instance"]["ready_profiles"] == 3
    assert preview["extension_execution_schedule"]["status"] == "ready"
    assert preview["extension_execution_schedule"]["ready_profiles"] == 3
    assert preview["extension_execution_actuals"]["status"] == "ready"
    assert preview["extension_execution_actuals"]["ready_profiles"] == 3
    assert preview["vulnerability_exception_review"]["status"] == "passed"
    assert preview["vulnerability_exception_review"]["review_candidate_count"] == 31
    assert preview["external_mainline_execution_plan"]["status"] == "ready"
    assert (
        preview["external_mainline_input_checklist"]["status"]
        == checklist_payload["status"]
    )
    assert preview["worktree_release_blocker"]["status"] == "ready"
    assert "exception_review=passed/31" in payload["summary"]
    assert "external_mainline=ready/" in payload["summary"]
    assert "external_mainline_input_checklist=" in payload["summary"]
    assert "worktree=ready/0/0" in payload["summary"]
    assert any("git tag 2026.04.12" in item for item in preview["next_actions"])
    assert any("补齐 industrial 签核" in item for item in preview["next_actions"])


def test_industrial_release_readiness_accepts_approval_manifest(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(
        project_root,
        metrics=_industrial_exception_review_metrics(),
    )
    _seed_industrial_security_reports(project_root)
    _seed_external_mainline_execution_plan(project_root)
    checklist_report = _seed_external_mainline_input_checklist(project_root)
    approval_manifest = _write_ready_industrial_manifest(project_root, source_root)
    report_path = (
        project_root
        / "test_env"
        / "industrial_release_readiness"
        / "industrial_readiness_ready.json"
    )

    result = subprocess.run(
        [
            sys.executable,
            "tools/check_industrial_release_readiness.py",
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
    assert "industrial_release_gate=ready" in result.stdout
    assert "industrial_security_preflight=passed" in result.stdout
    assert "industrial_vulnerability_exception_review=passed/31" in result.stdout
    assert "industrial_external_mainline_execution_plan=ready/" in result.stdout
    checklist_payload = json.loads(checklist_report.read_text(encoding="utf-8"))
    assert (
        "industrial_external_mainline_input_checklist="
        f"{checklist_payload['status']}"
        in result.stdout
    )
    assert "industrial_worktree_release_blocker=ready/0/0" in result.stdout
    assert "industrial_customer_delivery=ready" in result.stdout
    assert "industrial_industrial_delivery=ready" in result.stdout

    payload = json.loads(report_path.read_text(encoding="utf-8"))
    assert payload["approval_manifest_path"] == str(approval_manifest)
    assert payload["industrial_release_gate"] == "ready"
    assert payload["summary"].startswith(
        "industrial release readiness is ready: exception_review=passed/31, "
        "external_mainline=ready/"
    )
    assert payload["vulnerability_exception_review"]["status"] == "passed"
    assert payload["vulnerability_exception_review"]["review_candidate_count"] == 31
    assert payload["external_mainline_execution_plan"]["status"] == "ready"
    assert (
        payload["external_mainline_input_checklist"]["status"]
        == checklist_payload["status"]
    )
    assert payload["worktree_release_blocker"]["status"] == "ready"
    preview = payload["previews"][0]
    godot_profile = next(
        item
        for item in preview["customer_delivery_surface"]["extension_support_surface"]["profiles"]
        if item["id"] == "godot_extension"
    )
    assert preview["release_gate_status"] == "ready"
    assert preview["customer_delivery_surface"]["extension_support_surface"]["declared_profiles"] == 4
    assert preview["industrial_delivery_gate"]["declared_extension_profiles"] == 4
    assert payload["extension_execution_evidence"]["status"] == "ready"
    assert payload["extension_execution_evidence"]["ready_reports"] == 4
    assert payload["extension_execution_instance"]["status"] == "ready"
    assert payload["extension_execution_instance"]["ready_profiles"] == 3
    assert payload["extension_execution_schedule"]["status"] == "ready"
    assert payload["extension_execution_schedule"]["ready_profiles"] == 3
    assert payload["extension_execution_actuals"]["status"] == "ready"
    assert payload["extension_execution_actuals"]["ready_profiles"] == 3
    assert preview["extension_execution_evidence"]["status"] == "ready"
    assert preview["extension_execution_evidence"]["ready_reports"] == 4
    assert preview["extension_execution_instance"]["status"] == "ready"
    assert preview["extension_execution_instance"]["ready_profiles"] == 3
    assert preview["extension_execution_schedule"]["status"] == "ready"
    assert preview["extension_execution_schedule"]["ready_profiles"] == 3
    assert preview["extension_execution_actuals"]["status"] == "ready"
    assert preview["extension_execution_actuals"]["ready_profiles"] == 3
    assert preview["vulnerability_exception_review"]["status"] == "passed"
    assert preview["vulnerability_exception_review"]["review_candidate_count"] == 31
    assert preview["external_mainline_execution_plan"]["status"] == "ready"
    assert (
        preview["external_mainline_input_checklist"]["status"]
        == checklist_payload["status"]
    )
    assert preview["worktree_release_blocker"]["status"] == "ready"
    assert "GODOT_EXECUTABLE" in godot_profile["deployment_commands"][0]
    assert "AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1" in godot_profile["acceptance_checks"][0]
    assert preview["release_source"]["matched_version_tag"] == "2026.04.12"
    assert "worktree=ready/0/0" in payload["summary"]
    assert any(
        item.startswith("门禁已就绪，可生成最终 manifest:")
        for item in preview["next_actions"]
    )
    assert any(
        "run_external_mainline_execution_plan.py" in item
        for item in preview["next_actions"]
    )
