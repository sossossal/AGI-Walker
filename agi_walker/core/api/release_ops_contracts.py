"""Structured contracts for release-ops orchestration services."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

RELEASE_OP_POLICY_ORDER = (
    "read_only",
    "local_safe_refresh",
    "external_mutation",
    "requires_attestation",
)
RELEASE_OP_POLICY_LEVELS = frozenset(RELEASE_OP_POLICY_ORDER)
RELEASE_OP_POLICY_PROFILES = frozenset(RELEASE_OP_POLICY_ORDER)
DEFAULT_RELEASE_OP_POLICY_PROFILE = "local_safe_refresh"


@dataclass(slots=True)
class ExternalMainlineExecutionRequest:
    """Normalized request for the external-mainline orchestration service."""

    project_root: str
    inputs_file: str | None
    skip_managed_inputs: bool
    output: str
    external_mainline_input_checklist_report: str
    customer_config: str
    customer_external_bindings_closure_report: str
    vulnerability_exception_review_report: str
    industrial_delivery_rehearsal_report: str
    skip_vulnerability_exception_review_refresh: bool = False
    skip_customer_external_bindings_closure: bool = False
    customer_section: list[str] | None = None
    customer_confirmed_by: str | None = None
    customer_confirmed_at: str | None = None
    customer_confirmation_ticket: str | None = None
    customer_confirmation_notes: str | None = None
    customer_confirmation_evidence: str | None = None
    customer_overrides_file: str | None = None
    customer_set: list[str] | None = None
    skip_customer_collect_release_evidence: bool = False
    refresh_industrial_rehearsal: bool = False
    industrial_rehearsal_version: str | None = None
    industrial_rehearsal_build_id: str | None = None
    industrial_rehearsal_output_root: str = "test_env/release_rehearsal_industrial"
    industrial_live_evidence_inputs: dict[str, Any] | None = None


@dataclass(slots=True)
class ExternalMainlineExecutionResult:
    """Structured result returned by the external-mainline orchestration service."""

    payload: dict[str, Any]
    output_path: Path
    checklist_payload: dict[str, Any]
    checklist_path: Path
    resolved_inputs_path: Path | None
    managed_inputs_sync_status: str | None = None
    executed_steps: list[str] = field(default_factory=list)
    skipped_steps: list[str] = field(default_factory=list)
    failures: list[str] = field(default_factory=list)
    industrial_live_evidence_inputs_ready: bool | None = None


@dataclass(slots=True)
class ReleaseReadinessRequest:
    """Normalized request for the rc/stable release-readiness service."""

    current_version: str | None
    stable_version: str | None
    project_root: str
    source_root: str
    changelog: str = "RELEASE_NOTES.md"
    output_root: str = "test_env/release_readiness"
    report_file: str | None = None
    approval_status: str | None = None
    approved_by: str | None = None
    approved_at: str | None = None
    commit_sha: str | None = None
    approval_notes: str | None = None
    approval_manifest: str | None = None
    security_preflight_report: str | None = None


@dataclass(slots=True)
class ReleaseReadinessResult:
    """Structured result returned by the rc/stable release-readiness service."""

    payload: dict[str, Any]
    report_path: Path
    current_version: str
    stable_version: str
    rc_preview: dict[str, Any]
    stable_preview: dict[str, Any]


@dataclass(slots=True)
class IndustrialReleaseReadinessRequest:
    """Normalized request for the industrial release-readiness service."""

    current_version: str | None
    industrial_version: str | None
    project_root: str
    source_root: str
    changelog: str = "RELEASE_NOTES.md"
    output_root: str = "test_env/industrial_release_readiness"
    report_file: str | None = None
    approval_status: str | None = None
    approved_by: str | None = None
    approved_at: str | None = None
    commit_sha: str | None = None
    approval_notes: str | None = None
    approval_manifest: str | None = None
    security_preflight_report: str | None = None


@dataclass(slots=True)
class IndustrialReleaseReadinessResult:
    """Structured result returned by the industrial release-readiness service."""

    payload: dict[str, Any]
    report_path: Path
    current_version: str
    industrial_version: str
    industrial_preview: dict[str, Any]


@dataclass(slots=True)
class ReleaseRehearsalRequest:
    """Normalized request for the stable release-rehearsal service."""

    version: str = "2026.04.12-rehearsal"
    tag: str | None = None
    build_id: str = "release-rehearsal"
    output_root: str = "test_env/release_rehearsal"
    report_file: str | None = None
    external_bindings_config_source: str = (
        "deployment/customer_delivery.external_bindings.rehearsal.json"
    )


@dataclass(slots=True)
class ReleaseRehearsalResult:
    """Structured result returned by the stable release-rehearsal service."""

    payload: dict[str, Any]
    report_path: Path
    industrial_delivery_rehearsal_payload: dict[str, Any]
    industrial_delivery_rehearsal_report_path: Path
    tag: str
    gate_status: str


@dataclass(slots=True)
class WorktreeReleaseBlockerRequest:
    """Normalized request for the worktree release-blocker service."""

    source_root: str
    output_root: str = "test_env/worktree_cleanup"
    cleanup_report: str | None = None
    review_report: str | None = None
    report_file: str | None = None


@dataclass(slots=True)
class WorktreeReleaseBlockerResult:
    """Structured result returned by the worktree release-blocker service."""

    payload: dict[str, Any]
    report_path: Path
    cleanup_payload: dict[str, Any]
    cleanup_report_path: Path
    tracked_review_payload: dict[str, Any]
    tracked_review_report_path: Path


@dataclass(slots=True)
class CustomerAcceptanceBundleRequest:
    """Normalized request for the customer-acceptance bundle service."""

    manifest: str = "test_env/release/release_manifest_stable.json"
    project_root: str = "."
    output: str = "test_env/release/customer_acceptance_bundle.json"
    readiness_report: str | None = None
    promotion_checklist: str | None = None
    security_posture_report: str | None = None
    sbom_artifact: str | None = None
    python_vuln_report: str | None = None
    container_vuln_report: str | None = None
    vulnerability_exception_report: str | None = None
    vulnerability_exception_review_report: str | None = None
    customer_external_bindings_closure_report: str | None = None
    external_mainline_execution_plan: str | None = None
    external_mainline_input_checklist_report: str | None = None
    release_ops_execution_report: str | None = None
    backup_restore_report: str | None = None
    industrial_delivery_rehearsal_report: str | None = None


@dataclass(slots=True)
class CustomerAcceptanceBundleResult:
    """Structured result returned by the customer-acceptance bundle service."""

    payload: dict[str, Any]
    output_path: Path
    manifest_path: Path


@dataclass(slots=True)
class IndustrialDeliveryRehearsalReportRequest:
    """Normalized request for the industrial delivery rehearsal report service."""

    rehearsal_report: str = "test_env/release_rehearsal/release_rehearsal_report.json"
    output: str | None = None


@dataclass(slots=True)
class IndustrialDeliveryRehearsalReportResult:
    """Structured result returned by the industrial delivery rehearsal report service."""

    payload: dict[str, Any]
    output_path: Path
    rehearsal_report_path: Path


@dataclass(slots=True)
class StablePromotionChecklistRequest:
    """Normalized request for the stable promotion checklist service."""

    current_version: str | None = None
    stable_version: str | None = None
    project_root: str = "."
    source_root: str = "."
    changelog: str = "RELEASE_NOTES.md"
    output_root: str = "test_env/stable_promotion"
    readiness_report: str | None = None
    refresh_readiness: bool = False
    report_file: str | None = None
    approval_status: str | None = None
    approved_by: str | None = None
    approved_at: str | None = None
    commit_sha: str | None = None
    approval_notes: str | None = None
    security_preflight_report: str | None = None
    approval_manifest: str | None = None


@dataclass(slots=True)
class StablePromotionChecklistResult:
    """Structured result returned by the stable promotion checklist service."""

    payload: dict[str, Any]
    report_path: Path
    readiness_payload: dict[str, Any]
    readiness_report_path: Path
    stable_preview: dict[str, Any]


@dataclass(slots=True)
class IndustrialPromotionChecklistRequest:
    """Normalized request for the industrial promotion checklist service."""

    current_version: str | None = None
    industrial_version: str | None = None
    project_root: str = "."
    source_root: str = "."
    changelog: str = "RELEASE_NOTES.md"
    output_root: str = "test_env/industrial_promotion"
    readiness_report: str | None = None
    refresh_readiness: bool = False
    report_file: str | None = None
    approval_status: str | None = None
    approved_by: str | None = None
    approved_at: str | None = None
    commit_sha: str | None = None
    approval_notes: str | None = None
    security_preflight_report: str | None = None
    approval_manifest: str | None = None


@dataclass(slots=True)
class IndustrialPromotionChecklistResult:
    """Structured result returned by the industrial promotion checklist service."""

    payload: dict[str, Any]
    report_path: Path
    readiness_payload: dict[str, Any]
    readiness_report_path: Path
    industrial_preview: dict[str, Any]


@dataclass(slots=True)
class ReleaseOpSessionContext:
    """Lightweight delivery-session context for release-ops control-plane runs."""

    engagement_id: str | None = None
    window_id: str | None = None
    change_ticket: str | None = None
    channel: str | None = None


@dataclass(slots=True)
class ReleaseOpRequest:
    """Normalized request for the unified release-ops control-plane dispatcher."""

    action: str
    request: dict[str, Any] = field(default_factory=dict)
    policy_profile: str = DEFAULT_RELEASE_OP_POLICY_PROFILE
    session: ReleaseOpSessionContext = field(default_factory=ReleaseOpSessionContext)
    event_stream_file: str | None = None
    evidence_report_file: str | None = None


@dataclass(slots=True)
class ReleaseOpResult:
    """Structured result returned by the unified release-ops control-plane dispatcher."""

    action: str
    policy_level: str
    policy_profile: str
    request_type: str
    status: str
    summary: str
    payload: dict[str, Any]
    output_path: Path | None
    evidence_report_path: Path | None = None
    evidence_report_payload: dict[str, Any] = field(default_factory=dict)
    session: dict[str, str] = field(default_factory=dict)
    event_stream_path: Path | None = None
    event_count: int = 0


__all__ = [
    "CustomerAcceptanceBundleRequest",
    "CustomerAcceptanceBundleResult",
    "DEFAULT_RELEASE_OP_POLICY_PROFILE",
    "ExternalMainlineExecutionRequest",
    "ExternalMainlineExecutionResult",
    "IndustrialDeliveryRehearsalReportRequest",
    "IndustrialDeliveryRehearsalReportResult",
    "IndustrialPromotionChecklistRequest",
    "IndustrialPromotionChecklistResult",
    "IndustrialReleaseReadinessRequest",
    "IndustrialReleaseReadinessResult",
    "RELEASE_OP_POLICY_LEVELS",
    "RELEASE_OP_POLICY_ORDER",
    "RELEASE_OP_POLICY_PROFILES",
    "ReleaseRehearsalRequest",
    "ReleaseRehearsalResult",
    "ReleaseOpRequest",
    "ReleaseOpResult",
    "ReleaseOpSessionContext",
    "ReleaseReadinessRequest",
    "ReleaseReadinessResult",
    "StablePromotionChecklistRequest",
    "StablePromotionChecklistResult",
    "WorktreeReleaseBlockerRequest",
    "WorktreeReleaseBlockerResult",
]
