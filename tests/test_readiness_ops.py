from __future__ import annotations

from pathlib import Path

from agi_walker.core.api.release_ops_contracts import (
    IndustrialReleaseReadinessRequest,
    ReleaseReadinessRequest,
)
from agi_walker.ops.readiness import (
    execute_industrial_release_readiness,
    execute_release_readiness,
)
from tests.test_industrial_release_readiness import (
    _industrial_exception_review_metrics,
    _write_ready_industrial_manifest,
)
from tests.test_release_readiness import (
    _init_git_repo,
    _seed_clean_checkout_evidence,
    _seed_customer_delivery_docs,
    _seed_external_mainline_execution_plan,
    _seed_external_mainline_input_checklist,
    _seed_industrial_security_reports,
    _seed_live_evidence,
    _seed_security_preflight_report,
    _seed_structured_required_evidence,
)


def test_execute_release_readiness_returns_structured_result(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root = _init_git_repo(tmp_path, tag=None)
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)
    _seed_external_mainline_execution_plan(project_root)
    _seed_external_mainline_input_checklist(project_root)
    report_path = project_root / "test_env" / "release_readiness" / "readiness.json"

    result = execute_release_readiness(
        ReleaseReadinessRequest(
            current_version="2026.04.12-rc6",
            stable_version="2026.04.12",
            project_root=str(project_root),
            source_root=str(source_root),
            output_root=str(report_path.parent),
            report_file=str(report_path),
        )
    )

    assert result.report_path == report_path
    assert result.current_version == "2026.04.12-rc6"
    assert result.stable_version == "2026.04.12"
    assert result.rc_preview["release_gate_status"] == "ready"
    assert result.stable_preview["release_gate_status"] == "blocked"
    assert result.stable_preview["external_mainline_execution_plan"]["status"] == "ready"
    assert result.stable_preview["external_mainline_input_checklist"]["status"] in {
        "passed",
        "blocked",
    }
    assert result.payload["artifact_type"] == "release_readiness_report"
    assert result.payload["stable_release_gate"] == "blocked"
    assert "external_mainline=ready/" in result.payload["summary"]


def test_execute_industrial_release_readiness_returns_structured_result(
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
    _seed_external_mainline_input_checklist(project_root)
    approval_manifest = _write_ready_industrial_manifest(project_root, source_root)
    report_path = (
        project_root
        / "test_env"
        / "industrial_release_readiness"
        / "industrial_readiness.json"
    )

    result = execute_industrial_release_readiness(
        IndustrialReleaseReadinessRequest(
            current_version="2026.04.12",
            industrial_version="2026.04.12",
            project_root=str(project_root),
            source_root=str(source_root),
            output_root=str(report_path.parent),
            report_file=str(report_path),
            approval_manifest=str(approval_manifest),
            security_preflight_report=str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
        )
    )

    assert result.report_path == report_path
    assert result.current_version == "2026.04.12"
    assert result.industrial_version == "2026.04.12"
    assert result.industrial_preview["release_gate_status"] == "ready"
    assert result.industrial_preview["vulnerability_exception_review"]["status"] == "passed"
    assert (
        result.industrial_preview["vulnerability_exception_review"]["review_candidate_count"]
        == 31
    )
    assert result.industrial_preview["external_mainline_execution_plan"]["status"] == "ready"
    assert result.payload["artifact_type"] == "industrial_release_readiness_report"
    assert result.payload["industrial_release_gate"] == "ready"
    assert result.payload["approval_manifest_path"] == str(approval_manifest)
    assert "external_mainline=ready/" in result.payload["summary"]
