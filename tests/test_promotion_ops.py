from __future__ import annotations

from pathlib import Path

from agi_walker.core.api.release_ops_contracts import (
    IndustrialPromotionChecklistRequest,
    StablePromotionChecklistRequest,
)
from agi_walker.ops.promotion import (
    execute_industrial_promotion_checklist,
    execute_stable_promotion_checklist,
)
from tests.test_industrial_promotion_checklist import (
    _write_ready_industrial_manifest,
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


def test_execute_stable_promotion_checklist_reports_ready_to_promote(
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
    approval_manifest = _write_ready_stable_manifest(project_root, source_root)

    result = execute_stable_promotion_checklist(
        StablePromotionChecklistRequest(
            current_version="2026.04.12-rc6",
            stable_version="2026.04.12",
            project_root=str(project_root),
            source_root=str(source_root),
            approval_manifest=str(approval_manifest),
            security_preflight_report=str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
        )
    )

    assert result.report_path.exists()
    assert result.payload["artifact_type"] == "stable_promotion_checklist"
    assert result.payload["ready_to_promote"] is True
    assert result.payload["blocking_steps"] == 0
    assert result.payload["external_mainline_execution_plan"]["status"] == "missing"


def test_execute_stable_promotion_checklist_reports_blocked_gate(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    _seed_customer_delivery_docs(project_root)
    source_root, _ = _init_git_repo(tmp_path)
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)
    _seed_industrial_security_reports(project_root)

    result = execute_stable_promotion_checklist(
        StablePromotionChecklistRequest(
            current_version="2026.04.12-rc6",
            stable_version="2026.04.12",
            project_root=str(project_root),
            source_root=str(source_root),
        )
    )

    assert result.report_path.exists()
    assert result.payload["stable_release_gate"] == "blocked"
    assert result.payload["ready_to_promote"] is False
    assert result.payload["blocking_steps"] >= 1


def test_execute_industrial_promotion_checklist_reports_ready_to_promote(
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

    result = execute_industrial_promotion_checklist(
        IndustrialPromotionChecklistRequest(
            current_version="2026.04.12",
            project_root=str(project_root),
            source_root=str(source_root),
            approval_manifest=str(approval_manifest),
            security_preflight_report=str(
                project_root
                / "test_env"
                / "release_evidence"
                / "security_release_preflight_report.json"
            ),
        )
    )

    assert result.report_path.exists()
    assert result.payload["artifact_type"] == "industrial_promotion_checklist"
    assert result.payload["ready_to_promote"] is True
    assert result.payload["blocking_steps"] == 0
    assert result.payload["extension_execution_plan"]["status"] == "ready"


def test_execute_industrial_promotion_checklist_reports_blocked_gate(
    tmp_path: Path,
) -> None:
    project_root = tmp_path / "project_root"
    project_root.mkdir(parents=True, exist_ok=True)
    source_root, _ = _init_git_repo(tmp_path, tag="2026.04.12")
    _seed_live_evidence(project_root)
    _seed_clean_checkout_evidence(project_root)
    _seed_structured_required_evidence(project_root)
    _seed_security_preflight_report(project_root)

    result = execute_industrial_promotion_checklist(
        IndustrialPromotionChecklistRequest(
            current_version="2026.04.12",
            project_root=str(project_root),
            source_root=str(source_root),
        )
    )

    assert result.report_path.exists()
    assert result.payload["industrial_release_gate"] == "blocked"
    assert result.payload["ready_to_promote"] is False
    assert result.payload["blocking_steps"] >= 1
