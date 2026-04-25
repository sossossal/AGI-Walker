"""Deterministic orchestration for customer acceptance bundle generation."""

from __future__ import annotations

import json
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_customer_acceptance_bundle_artifact,
    validate_release_manifest_artifact,
    write_customer_acceptance_bundle_artifact,
)
from agi_walker.core.api.release_ops_contracts import (
    CustomerAcceptanceBundleRequest,
    CustomerAcceptanceBundleResult,
)


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


PROJECT_ROOT = _find_repo_root()


def _resolve_project_path(path: str | Path, project_root: str | Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return Path(project_root) / candidate


def execute_customer_acceptance_bundle(
    request: CustomerAcceptanceBundleRequest,
) -> CustomerAcceptanceBundleResult:
    project_root = _resolve_project_path(request.project_root, PROJECT_ROOT).resolve()
    manifest_path = _resolve_project_path(request.manifest, project_root).resolve()
    if not manifest_path.is_file():
        raise ValueError(f"--manifest does not exist: {manifest_path}")

    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ValueError(f"--manifest is unreadable: {exc}") from exc

    manifest_errors = validate_release_manifest_artifact(payload)
    if manifest_errors:
        raise ValueError(f"--manifest is invalid: {'; '.join(manifest_errors)}")

    bundle_payload = build_customer_acceptance_bundle_artifact(
        release_manifest=payload,
        manifest_path=manifest_path,
        project_root=str(project_root),
        readiness_report_path=request.readiness_report,
        promotion_checklist_path=request.promotion_checklist,
        security_posture_report_path=request.security_posture_report,
        sbom_artifact_path=request.sbom_artifact,
        python_vulnerability_scan_report_path=request.python_vuln_report,
        container_vulnerability_scan_report_path=request.container_vuln_report,
        vulnerability_exception_report_path=request.vulnerability_exception_report,
        vulnerability_exception_review_report_path=request.vulnerability_exception_review_report,
        customer_external_bindings_closure_report_path=request.customer_external_bindings_closure_report,
        external_mainline_execution_plan_path=request.external_mainline_execution_plan,
        external_mainline_input_checklist_report_path=request.external_mainline_input_checklist_report,
        release_ops_execution_report_path=request.release_ops_execution_report,
        backup_restore_rehearsal_report_path=request.backup_restore_report,
        industrial_delivery_rehearsal_report_path=request.industrial_delivery_rehearsal_report,
    )
    output_path = write_customer_acceptance_bundle_artifact(
        bundle_payload,
        _resolve_project_path(request.output, project_root),
    )
    return CustomerAcceptanceBundleResult(
        payload=bundle_payload,
        output_path=Path(output_path),
        manifest_path=manifest_path,
    )


__all__ = ["execute_customer_acceptance_bundle"]
