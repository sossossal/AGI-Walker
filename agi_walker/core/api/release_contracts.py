"""Stable contracts for release manifests and gate evidence."""

from __future__ import annotations

import json
import subprocess
from collections.abc import Mapping, Sequence
from datetime import datetime
from pathlib import Path
from typing import Any

from agi_walker.core.api.capability_matrix import (
    CAPABILITY_MATRIX_VERSION,
    build_capability_matrix_artifact,
    validate_capability_matrix_artifact,
)
from agi_walker.core.api.security_posture_contracts import (
    BACKUP_RESTORE_REHEARSAL_REPORT_ARTIFACT_TYPE,
    DEFAULT_VULNERABILITY_EXCEPTION_REVIEW_WINDOW_DAYS,
    SBOM_ARTIFACT_TYPE,
    SECURITY_POSTURE_REPORT_ARTIFACT_TYPE,
    VULNERABILITY_REMEDIATION_REPORT_ARTIFACT_TYPE,
    VULNERABILITY_SCAN_REPORT_ARTIFACT_TYPE,
    VULNERABILITY_SCAN_STATUSES,
    validate_backup_restore_rehearsal_report,
    validate_sbom_artifact,
    validate_vulnerability_remediation_report,
    validate_security_posture_report,
    validate_vulnerability_exception_report,
    validate_vulnerability_scan_report,
)
from agi_walker.core.api.training_contracts import TRAINING_CONTRACT_VERSION
from agi_walker.core.api.workflow_contracts import (
    WORKFLOW_CONTRACT_VERSION,
    to_jsonable,
)

RELEASE_CONTRACT_VERSION = "1.0"
RELEASE_ARTIFACT_TYPE = "release_manifest"
RELEASE_EVIDENCE_REPORT_VERSION = "1.0"
RELEASE_EVIDENCE_REPORT_ARTIFACT_TYPE = "release_evidence_report"
CUSTOMER_ACCEPTANCE_BUNDLE_VERSION = "1.0"
CUSTOMER_ACCEPTANCE_BUNDLE_ARTIFACT_TYPE = "customer_acceptance_bundle"
INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_VERSION = "1.0"
INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_ARTIFACT_TYPE = (
    "industrial_delivery_rehearsal_report"
)
EXTERNAL_MAINLINE_EXECUTION_PLAN_VERSION = "1.0"
EXTERNAL_MAINLINE_EXECUTION_PLAN_ARTIFACT_TYPE = "external_mainline_execution_plan"
EXTENSION_EXECUTION_INSTANCE_VERSION = "1.0"
EXTENSION_EXECUTION_INSTANCE_ARTIFACT_TYPE = "extension_execution_instance"
EXTENSION_EXECUTION_SCHEDULE_VERSION = "1.0"
EXTENSION_EXECUTION_SCHEDULE_ARTIFACT_TYPE = "extension_execution_schedule"
EXTENSION_EXECUTION_ACTUALS_VERSION = "1.0"
EXTENSION_EXECUTION_ACTUALS_ARTIFACT_TYPE = "extension_execution_actuals"

RELEASE_CHANNELS = {"dev", "rc", "stable", "industrial"}
RELEASE_GATE_STATUSES = {"ready", "ready_with_limitations", "blocked"}
RELEASE_EVIDENCE_STATUSES = {"passed", "blocked", "opt_in"}
RELEASE_APPROVAL_STATUSES = {"not_required", "pending", "approved"}
CUSTOMER_ACCEPTANCE_BUNDLE_STATUSES = {"ready", "blocked"}
INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_STATUSES = {"ready", "blocked"}
EXTERNAL_MAINLINE_EXECUTION_PLAN_STATUSES = {"ready", "blocked"}
EXTERNAL_MAINLINE_EXECUTION_STEP_STATUSES = {
    "completed",
    "ready_to_run",
    "waiting_external_input",
    "blocked",
}
CUSTOMER_DELIVERY_SURFACE_STATUSES = {"ready", "blocked"}
INDUSTRIAL_DELIVERY_GATE_STATUSES = {"ready", "blocked"}
INDUSTRIAL_DELIVERY_COMPONENT_STATUSES = {"ready", "blocked"}
EXTENSION_SUPPORT_SURFACE_STATUSES = {"ready", "blocked"}
EXTENSION_SUPPORT_PROFILE_STATUSES = {"supported", "conditional", "not_supported"}
EXTENSION_EXECUTION_EVIDENCE_STATUSES = {"ready", "blocked"}
EXTENSION_EXECUTION_INSTANCE_STATUSES = {"ready", "blocked"}
EXTENSION_EXECUTION_SCHEDULE_STATUSES = {"ready", "blocked"}
EXTENSION_EXECUTION_ACTUALS_STATUSES = {"ready", "blocked"}
INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_STATUSES = {"pass", "fail"}
INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS = (
    "new_environment_install",
    "smoke",
    "live_evidence",
    "upgrade",
    "rollback",
    "backup_restore",
)

DISTRIBUTED_RELEASE_LIMITATION = (
    "Distributed runtime remains diagnostic_ready until a passing distributed smoke report is attached to release evidence."
)
DISTRIBUTED_DOMAIN_LIMITATION = (
    "A passing distributed smoke report is still required to confirm actor discovery and learner action loop in the target Docker environment."
)

RELEASE_MANIFEST_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "build_id",
    "version",
    "channel",
    "release_policy",
    "release_approval",
    "release_source",
    "release_summary",
    "generated_at",
    "release_gate_status",
    "release_gate",
    "changelog",
    "contract_versions",
    "capability_matrix",
    "test_evidence",
    "known_limitations",
    "customer_delivery_surface",
    "industrial_delivery_gate",
    "release_ops_execution",
    "control_plane_surface",
    "extension_execution_evidence",
    "extension_execution_instance",
    "extension_execution_schedule",
    "extension_execution_actuals",
}
RELEASE_GATE_REQUIRED_FIELDS = {
    "required_evidence",
    "passed_required_evidence",
    "blocked_evidence",
    "blocked_optional_evidence",
    "opt_in_evidence",
    "diagnostic_ready_domains",
    "release_approval_required",
    "release_approval_ready",
    "release_source_required",
    "release_source_ready",
    "release_worktree_required",
    "release_worktree_ready",
    "release_version_tag_required",
    "release_version_tag_ready",
    "customer_delivery_required",
    "customer_delivery_ready",
    "industrial_delivery_required",
    "industrial_delivery_ready",
}
RELEASE_POLICY_REQUIRED_FIELDS = {
    "channel",
    "allows_opt_in_evidence",
    "allows_diagnostic_ready_domains",
    "requires_release_approval",
    "requires_git_source_binding",
    "requires_clean_worktree",
    "requires_version_tag_match",
    "requires_customer_delivery_surface",
    "requires_industrial_delivery_gate",
    "summary",
}
RELEASE_APPROVAL_REQUIRED_FIELDS = {
    "status",
    "required",
    "approved_by",
    "approved_at",
    "commit_sha",
    "notes",
}
RELEASE_SOURCE_REQUIRED_FIELDS = {
    "resolved_from_git",
    "commit_sha",
    "short_commit_sha",
    "git_tag",
    "matched_version_tag",
    "worktree_clean",
    "worktree_status_summary",
    "version_tag_matches",
}
RELEASE_CHANGELOG_REQUIRED_FIELDS = {"path", "title"}
RELEASE_CONTRACT_VERSION_REQUIRED_FIELDS = {"name", "version"}
RELEASE_TEST_EVIDENCE_REQUIRED_FIELDS = {
    "name",
    "required",
    "status",
    "summary",
    "command",
}
RELEASE_EVIDENCE_REPORT_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "evidence_name",
    "status",
    "summary",
    "command",
    "generated_at",
}
CUSTOMER_ACCEPTANCE_BUNDLE_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "generated_at",
    "bundle_status",
    "summary",
    "version",
    "channel",
    "build_id",
    "release_manifest",
    "known_limitations",
    "extension_support_surface",
    "extension_execution_plan",
    "extension_execution_evidence",
    "extension_execution_instance",
    "extension_execution_schedule",
    "extension_execution_actuals",
    "required_evidence",
    "optional_evidence",
    "acceptance_documents",
    "acceptance_reports",
    "recommended_commands",
}
INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "generated_at",
    "status",
    "summary",
    "version",
    "tag",
    "source_root",
    "source_commit_sha",
    "release_rehearsal_status",
    "release_rehearsal_report_path",
    "release_gate_status",
    "customer_delivery_status",
    "industrial_delivery_status",
    "security_release_preflight",
    "vulnerability_exception_review",
    "customer_external_bindings_closure",
    "external_mainline_execution_plan",
    "industrial_manifest",
    "industrial_release_readiness",
    "industrial_promotion_checklist",
    "industrial_customer_acceptance_bundle",
    "extension_execution_plan",
    "extension_execution_evidence",
    "extension_execution_instance",
    "extension_execution_schedule",
    "extension_execution_actuals",
    "delivery_rehearsal_stages",
    "stage_summary",
    "industrial_delivery_artifact_paths",
}
INDUSTRIAL_DELIVERY_REHEARSAL_COMPONENT_FIELDS = {"status", "summary"}
INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_FIELDS = {
    "id",
    "status",
    "summary",
    "artifact_paths",
}
INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_SUMMARY_FIELDS = {
    "total",
    "passed",
    "failed",
}
CONTROL_PLANE_SURFACE_REQUIRED_FIELDS = {
    "status",
    "summary",
    "release_ops_execution",
}
EXTERNAL_MAINLINE_EXECUTION_PLAN_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "generated_at",
    "status",
    "summary",
    "completed_steps",
    "ready_to_run_steps",
    "waiting_external_input_steps",
    "blocked_steps",
    "auto_executable_steps",
    "steps",
}
EXTERNAL_MAINLINE_EXECUTION_STEP_REQUIRED_FIELDS = {
    "id",
    "label",
    "status",
    "summary",
    "command",
    "auto_executable",
    "external_required",
    "automation_scope",
    "source_report_path",
    "source_report_status",
    "blocking_inputs",
    "artifact_paths",
}
CUSTOMER_ACCEPTANCE_RELEASE_MANIFEST_FIELDS = {
    "path",
    "release_gate_status",
    "release_summary",
    "generated_at",
    "source_commit_sha",
}
CUSTOMER_ACCEPTANCE_ITEM_FIELDS = {
    "name",
    "path",
    "required",
    "exists",
}
EXTENSION_EXECUTION_EVIDENCE_REQUIRED_FIELDS = {
    "status",
    "summary",
    "required_reports",
    "ready_reports",
    "declared_profiles",
    "actionable_profiles",
    "on_call_rehearsal_attested",
    "exception_review_scheduled",
    "escalation_closure_attested",
    "external_bindings_confirmation_attested",
    "missing_reports",
    "reports",
}
EXTENSION_EXECUTION_INSTANCE_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "generated_at",
    "artifact_path",
    "exists",
    "status",
    "summary",
    "engagement_id",
    "customer_name",
    "site_name",
    "change_ticket",
    "window_id",
    "window_start_at",
    "window_end_at",
    "delivery_root",
    "closure_archive_root",
    "exception_review_due_at",
    "declared_profiles",
    "actionable_profiles",
    "ready_profiles",
    "missing_profiles",
    "profiles",
}
EXTENSION_EXECUTION_INSTANCE_PROFILE_REQUIRED_FIELDS = {
    "id",
    "label",
    "declared",
    "actionable",
    "handoff_record_path",
    "watch_log_path",
    "exception_review_record_path",
    "escalation_closure_record_path",
    "closure_archive_root",
    "closure_index_path",
}
EXTENSION_EXECUTION_SCHEDULE_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "generated_at",
    "artifact_path",
    "exists",
    "status",
    "summary",
    "engagement_id",
    "customer_name",
    "site_name",
    "change_ticket",
    "window_id",
    "window_start_at",
    "window_end_at",
    "window_trigger_at",
    "signoff_due_at",
    "exception_review_due_at",
    "closure_archive_due_at",
    "delivery_root",
    "closure_archive_root",
    "declared_profiles",
    "actionable_profiles",
    "ready_profiles",
    "missing_profiles",
    "profiles",
}
EXTENSION_EXECUTION_SCHEDULE_PROFILE_REQUIRED_FIELDS = {
    "id",
    "label",
    "declared",
    "actionable",
    "window_trigger_at",
    "signoff_due_at",
    "exception_review_due_at",
    "closure_archive_due_at",
    "window_trigger_record_path",
    "handoff_record_path",
    "watch_log_path",
    "signoff_record_path",
    "exception_review_record_path",
    "residual_risk_review_record_path",
    "escalation_closure_record_path",
    "closure_archive_root",
    "closure_index_path",
    "closure_manifest_path",
}
EXTENSION_EXECUTION_ACTUALS_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "generated_at",
    "artifact_path",
    "exists",
    "status",
    "summary",
    "engagement_id",
    "customer_name",
    "site_name",
    "change_ticket",
    "window_id",
    "approval_identity_source_path",
    "approval_identity_source_type",
    "approval_identity_reference",
    "archive_target_binding_type",
    "archive_target_binding_reference_base",
    "exception_review_due_at",
    "closure_archive_due_at",
    "due_trigger_binding_type",
    "due_trigger_binding_reference_base",
    "due_trigger_checked_at",
    "window_trigger_recorded_at",
    "window_trigger_recorded_by",
    "signoff_recorded_at",
    "signoff_recorded_by",
    "residual_risk_reviewed_at",
    "residual_risk_reviewed_by",
    "closure_archived_at",
    "closure_archived_by",
    "delivery_root",
    "closure_archive_root",
    "declared_profiles",
    "actionable_profiles",
    "ready_profiles",
    "window_triggers_recorded",
    "signoffs_recorded",
    "exception_reviews_scheduled",
    "residual_risk_reviews_recorded",
    "archive_targets_ready",
    "archive_target_bindings_ready",
    "due_trigger_checks_ready",
    "due_trigger_bindings_ready",
    "closure_indexes_ready",
    "closures_archived",
    "missing_profiles",
    "profiles",
}
EXTENSION_EXECUTION_ACTUALS_PROFILE_REQUIRED_FIELDS = {
    "id",
    "label",
    "declared",
    "actionable",
    "signoff_due_at",
    "exception_review_due_at",
    "closure_archive_due_at",
    "approval_identity_source_path",
    "window_trigger_record_path",
    "signoff_record_path",
    "exception_review_record_path",
    "residual_risk_review_record_path",
    "archive_target_path",
    "archive_target_binding_reference",
    "due_trigger_check_path",
    "due_trigger_binding_reference",
    "closure_index_path",
    "closure_manifest_path",
    "closure_archive_root",
    "signoff_owner_role",
    "exception_review_owner_role",
    "closure_archive_owner_role",
}
CUSTOMER_DELIVERY_SURFACE_REQUIRED_FIELDS = {
    "status",
    "summary",
    "required_documents",
    "required_documents_ready",
    "phase_e_documents",
    "phase_e_documents_ready",
    "support_matrix_attached",
    "capacity_declaration_attached",
    "customer_acceptance_checklist_attached",
    "known_limitations_attached",
    "extension_support_surface",
    "missing_required_documents",
    "missing_phase_e_documents",
    "phase_e_focus_documents",
    "documents",
}
INDUSTRIAL_DELIVERY_GATE_REQUIRED_FIELDS = {
    "status",
    "summary",
    "deployment_package_status",
    "evidence_attested",
    "required_evidence",
    "attested_required_evidence",
    "sbom_attached",
    "vuln_scan_status",
    "backup_restore_verified",
    "support_matrix_attached",
    "capacity_declaration_attached",
    "customer_acceptance_checklist_attached",
    "known_limitations_attached",
    "customer_delivery_surface_status",
    "extension_support_surface_status",
    "required_extension_profiles",
    "declared_extension_profiles",
    "required_deployment_documents",
    "ready_deployment_documents",
    "missing_requirements",
    "deployment_documents",
    "security_reports",
    "release_ops_execution",
}
CUSTOMER_DELIVERY_PHASE_E_FOCUS_DOCUMENTS = (
    ("support_matrix_attached", "support_matrix"),
    ("capacity_declaration_attached", "capacity_and_scale"),
    ("customer_acceptance_checklist_attached", "customer_acceptance_checklist"),
    ("known_limitations_attached", "known_limitations_guide"),
)
INDUSTRIAL_DELIVERY_DEPLOYMENT_DOCUMENT_NAMES = {
    "deployment_matrix",
    "customer_installation_guide",
    "production_runbook",
    "security_baseline",
    "audit_trail_policy",
    "backup_restore_runbook",
    "incident_response_matrix",
}
INDUSTRIAL_DELIVERY_SECURITY_REPORT_NAMES = (
    "sbom_artifact",
    "python_vulnerability_scan_report",
    "container_vulnerability_scan_report",
    "backup_restore_rehearsal_report",
)
EXTENSION_SUPPORT_DECLARATION_DOCUMENT_NAMES = (
    "support_matrix",
    "known_limitations_guide",
    "customer_acceptance_checklist",
)
EXTENSION_SUPPORT_SURFACE_REQUIRED_FIELDS = {
    "status",
    "summary",
    "required_profiles",
    "declared_profiles",
    "conditional_profiles",
    "not_supported_profiles",
    "missing_documents",
    "profiles",
}
EXTENSION_SUPPORT_PROFILE_REQUIRED_FIELDS = {
    "id",
    "label",
    "status",
    "declared",
    "default_delivery",
    "requires_special_acceptance",
    "baseline",
    "required_evidence",
    "reference_paths",
    "runbook_entrypoints",
    "execution_template",
    "deployment_commands",
    "acceptance_checks",
    "rollback_prerequisites",
    "summary",
    "non_support_scope",
}
EXTENSION_EXECUTION_PLAN_STATUSES = {"ready", "blocked"}
EXTENSION_EXECUTION_PLAN_REQUIRED_FIELDS = {
    "status",
    "summary",
    "declared_profiles",
    "actionable_profiles",
    "special_acceptance_profiles",
    "profiles",
}
EXTENSION_EXECUTION_PROFILE_REQUIRED_FIELDS = {
    "id",
    "label",
    "status",
    "declared",
    "actionable",
    "default_delivery",
    "requires_special_acceptance",
    "runbook_entrypoints",
    "execution_template",
    "deployment_commands",
    "acceptance_checks",
    "rollback_prerequisites",
}
EXTENSION_EXECUTION_TEMPLATE_REQUIRED_FIELDS = {
    "operator_roles",
    "upgrade_window_steps",
    "rollback_owner_role",
    "rollback_steps",
    "handoff_owner_role",
    "handoff_checkpoints",
    "signoff_checkpoints",
    "watch_owner_role",
    "watch_actions",
    "on_call_handoff_owner_role",
    "on_call_handoff_records",
    "residual_risk_owner_role",
    "residual_risk_handoff_steps",
    "exception_review_owner_role",
    "exception_review_steps",
    "incident_escalation_owner_role",
    "incident_escalation_steps",
    "escalation_closure_owner_role",
    "escalation_closure_steps",
    "rollback_evidence_owner_role",
    "rollback_evidence_archive_steps",
}
EXTENSION_EXECUTION_TEMPLATE_ROLE_REQUIRED_FIELDS = {
    "id",
    "label",
    "responsibility",
}
EXTENSION_EXECUTION_TEMPLATE_STEP_REQUIRED_FIELDS = {
    "order",
    "title",
    "owner_role",
}
EXTENSION_EXECUTION_TEMPLATE_SIGNOFF_REQUIRED_FIELDS = {
    "order",
    "title",
    "owner_role",
    "required_artifact",
}
EXTENSION_EXECUTION_TEMPLATE_ARCHIVE_REQUIRED_FIELDS = {
    "order",
    "title",
    "owner_role",
    "target_path",
}
EXTENSION_EXECUTION_TEMPLATE_ESCALATION_REQUIRED_FIELDS = {
    "order",
    "title",
    "owner_role",
    "escalation_target",
}
EXTENSION_SUPPORT_PROFILE_SPECS = (
    {
        "id": "distributed_profile",
        "label": "Distributed Profile",
        "status": "conditional",
        "default_delivery": False,
        "requires_special_acceptance": True,
        "baseline": "Docker Compose distributed profile plus explicit runtime preparation.",
        "required_evidence": ("distributed_runtime_live",),
        "reference_documents": (
            "support_matrix",
            "known_limitations_guide",
            "customer_acceptance_checklist",
        ),
        "runbook_entrypoints": (
            "PRODUCTION_DEPLOYMENT_RUNBOOK.md",
            "docs/guides/CUSTOMER_INSTALLATION_GUIDE.md",
            "docs/guides/DISTRIBUTED_GUIDE.md",
        ),
        "execution_template": {
            "operator_roles": (
                {
                    "id": "delivery_lead",
                    "label": "Delivery Lead",
                    "responsibility": "Own the distributed rollout sequence, confirm bundle/runbook alignment, and capture onsite evidence.",
                },
                {
                    "id": "customer_operator",
                    "label": "Customer Operator",
                    "responsibility": "Provide the approved change window, host/runtime access, and confirm distributed endpoint reachability.",
                },
                {
                    "id": "rollback_owner",
                    "label": "Rollback Owner",
                    "responsibility": "Own rollback to the previous distributed tag and configuration snapshot when acceptance fails.",
                },
            ),
            "upgrade_window_steps": (
                {
                    "order": 1,
                    "title": "Freeze the distributed change window and confirm actor/Godot endpoint snapshots plus runbook entrypoints.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Apply the distributed deployment commands and record the exact compose profile parameters used onsite.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 3,
                    "title": "Run system and distributed acceptance probes, then attach the distributed smoke report to the delivery evidence set.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 4,
                    "title": "Capture sign-off for the distributed window or hand control to the rollback owner before reopening traffic.",
                    "owner_role": "rollback_owner",
                },
            ),
            "handoff_owner_role": "delivery_lead",
            "handoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Transfer the active compose profile, actor endpoint snapshot, and customer bundle links to the onsite operator before the window leaves the delivery team.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Confirm the customer operator has the current smoke artifact path, rollback contact path, and accepted runtime variable set before reopening traffic.",
                    "owner_role": "customer_operator",
                },
            ),
            "signoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Do not sign off until distributed smoke and /api/distributed/status checks are attached to the acceptance package.",
                    "owner_role": "customer_operator",
                    "required_artifact": "test_env/distributed_smoke/distributed_smoke_report.json",
                },
                {
                    "order": 2,
                    "title": "Close the change window only after the promotion checklist and bundle both reference the same distributed rollout state.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
                },
            ),
            "watch_owner_role": "customer_operator",
            "watch_actions": (
                {
                    "order": 1,
                    "title": "Monitor /api/distributed/status and the distributed compose logs during the first post-cutover watch window.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 2,
                    "title": "Keep the distributed smoke report and active endpoint snapshot available until the first watch window closes cleanly.",
                    "owner_role": "delivery_lead",
                },
            ),
            "on_call_handoff_owner_role": "delivery_lead",
            "on_call_handoff_records": (
                {
                    "order": 1,
                    "title": "Record the first distributed watch-window outcome, active operator, and escalation contact in the customer acceptance bundle before the delivery team leaves the bridge.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Attach the distributed smoke report and active endpoint snapshot that the steady-state on-call team must inherit.",
                    "owner_role": "customer_operator",
                    "required_artifact": "test_env/distributed_smoke/distributed_smoke_report.json",
                },
            ),
            "residual_risk_owner_role": "delivery_lead",
            "residual_risk_handoff_steps": (
                {
                    "order": 1,
                    "title": "Hand off the distributed support boundary and opt-in limitations before the onsite team owns the runtime.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/guides/KNOWN_LIMITATIONS.md",
                },
                {
                    "order": 2,
                    "title": "Hand off the current security posture or accepted exceptions if residual risk is still active for the release package.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release_evidence/security/security_posture_report.json",
                },
            ),
            "exception_review_owner_role": "delivery_lead",
            "exception_review_steps": (
                {
                    "order": 1,
                    "title": "Review the current distributed exception input and expiry dates before closing the onsite watch window.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "deployment/security/vulnerability_exceptions.input.json",
                },
                {
                    "order": 2,
                    "title": "Attach the current exception coverage record before the distributed residual risk is accepted for steady-state support.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release_evidence/security/vulnerability_exception_report.json",
                },
            ),
            "incident_escalation_owner_role": "customer_operator",
            "incident_escalation_steps": (
                {
                    "order": 1,
                    "title": "Escalate distributed runtime degradation through the incident response matrix before the watch window is closed.",
                    "owner_role": "customer_operator",
                    "escalation_target": "docs/guides/INCIDENT_RESPONSE_MATRIX.md",
                },
                {
                    "order": 2,
                    "title": "If actor or Godot endpoint drift blocks recovery, transfer control to the rollback owner and suspend promotion.",
                    "owner_role": "delivery_lead",
                    "escalation_target": "rollback_owner",
                },
            ),
            "escalation_closure_owner_role": "delivery_lead",
            "escalation_closure_steps": (
                {
                    "order": 1,
                    "title": "Record the final distributed escalation decision, owner, and reopen criteria in the industrial promotion checklist before the incident is marked closed.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
                },
                {
                    "order": 2,
                    "title": "Attach the recovered distributed smoke or rollback evidence before clearing the distributed escalation chain.",
                    "owner_role": "rollback_owner",
                    "required_artifact": "test_env/distributed_smoke/distributed_smoke_report.json",
                },
            ),
            "rollback_owner_role": "rollback_owner",
            "rollback_steps": (
                {
                    "order": 1,
                    "title": "Freeze distributed traffic and stop the current distributed profile before restoring the previous baseline.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 2,
                    "title": "Restore the previous distributed tag, configuration snapshot, and runtime variables, then relaunch the supported services.",
                    "owner_role": "rollback_owner",
                },
                {
                    "order": 3,
                    "title": "Rerun base system and distributed health checks and archive rollback evidence into the release bundle.",
                    "owner_role": "delivery_lead",
                },
            ),
            "rollback_evidence_owner_role": "rollback_owner",
            "rollback_evidence_archive_steps": (
                {
                    "order": 1,
                    "title": "Archive the distributed rollback command log, previous profile snapshot, and restored env file set with the customer acceptance bundle.",
                    "owner_role": "rollback_owner",
                    "target_path": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Archive the recovered distributed health output and smoke evidence with the rollback record before closing the incident.",
                    "owner_role": "delivery_lead",
                    "target_path": "test_env/distributed_smoke/distributed_smoke_report.json",
                },
            ),
        },
        "summary": (
            "Distributed profile is opt-in and only supported when explicitly enabled on top of the base Compose deployment."
        ),
        "deployment_commands": (
            "docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed",
            "$env:AGI_WALKER_GODOT_HOST='host.docker.internal'; $env:AGI_WALKER_GODOT_PORT='9000'; $env:AGI_WALKER_SIDECAR_ACTOR_ID='actor_customer_1'",
        ),
        "acceptance_checks": (
            "Invoke-RestMethod http://127.0.0.1:8081/api/system/status",
            "Invoke-RestMethod http://127.0.0.1:8081/api/distributed/status",
            "python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json",
        ),
        "rollback_prerequisites": (
            "确认上一可回滚 tag 已记录，并已备份 <runtime-root>/db、workflow_runs 和 workflow_archive。",
            "保留 distributed profile 所需的 compose.env、web_panel.env 与 actor/Godot endpoint 配置快照。",
        ),
        "non_support_scope": (
            "Not a zero-config default deployment surface.",
            "Not a substitute for the base Web Panel control plane.",
        ),
    },
    {
        "id": "ros2_bridge_extension",
        "label": "ROS2 Bridge Extension",
        "status": "conditional",
        "default_delivery": False,
        "requires_special_acceptance": True,
        "baseline": "ROS2 Humble runtime plus dedicated bridge node wiring.",
        "required_evidence": ("ros2_bridge_live",),
        "reference_documents": (
            "support_matrix",
            "known_limitations_guide",
            "customer_acceptance_checklist",
        ),
        "runbook_entrypoints": (
            "PRODUCTION_DEPLOYMENT_RUNBOOK.md",
            "docs/guides/CUSTOMER_INSTALLATION_GUIDE.md",
            "docs/ros2/ROS2_QUICK_START.md",
        ),
        "execution_template": {
            "operator_roles": (
                {
                    "id": "delivery_lead",
                    "label": "Delivery Lead",
                    "responsibility": "Coordinate ROS2 bridge rollout scope, verify onsite evidence collection, and keep the bundle aligned with the approved runtime plan.",
                },
                {
                    "id": "customer_operator",
                    "label": "Customer Operator",
                    "responsibility": "Provide ROS2 Humble workspace access, runtime parameters, and confirm topic/service reachability during the window.",
                },
                {
                    "id": "rollback_owner",
                    "label": "Rollback Owner",
                    "responsibility": "Own shutdown of the current bridge node and restoration of the previous ROS2 launch and parameter set.",
                },
            ),
            "upgrade_window_steps": (
                {
                    "order": 1,
                    "title": "Freeze the ROS2 change window and verify the target workspace, Humble distro, and launch parameters against the approved runbook.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Build the ROS2 workspace, start the bridge launch file, and record the exact launch and parameter inputs used onsite.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 3,
                    "title": "Run topic, service, and bridge smoke checks and attach the resulting live evidence to the acceptance package.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 4,
                    "title": "Confirm bridge acceptance or hand execution to the rollback owner before the window is released.",
                    "owner_role": "rollback_owner",
                },
            ),
            "handoff_owner_role": "delivery_lead",
            "handoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Transfer the active ROS2 workspace path, launch parameters, and approved operator contacts to the onsite operator before the bridge is handed over.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Confirm the customer operator has the current smoke artifact path and rollback contact route before the window leaves bridge rollout mode.",
                    "owner_role": "customer_operator",
                },
            ),
            "signoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Do not sign off until ROS2 topic/service checks and bridge smoke evidence are attached to the acceptance package.",
                    "owner_role": "customer_operator",
                    "required_artifact": "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json",
                },
                {
                    "order": 2,
                    "title": "Close the bridge change window only after the customer bundle records the same ROS2 launch path and onsite runtime owner.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release/customer_acceptance_bundle.json",
                },
            ),
            "watch_owner_role": "customer_operator",
            "watch_actions": (
                {
                    "order": 1,
                    "title": "Monitor bridge topic/service reachability and ROS2 launch health during the first post-cutover watch window.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 2,
                    "title": "Keep the active launch parameters and bridge smoke evidence available until the first watch window closes cleanly.",
                    "owner_role": "delivery_lead",
                },
            ),
            "on_call_handoff_owner_role": "delivery_lead",
            "on_call_handoff_records": (
                {
                    "order": 1,
                    "title": "Record the first ROS2 bridge watch-window outcome, runtime owner, and escalation contact in the customer acceptance bundle before handoff completes.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Attach the bridge smoke report and accepted launch/parameter baseline that the steady-state on-call team must inherit.",
                    "owner_role": "customer_operator",
                    "required_artifact": "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json",
                },
            ),
            "residual_risk_owner_role": "delivery_lead",
            "residual_risk_handoff_steps": (
                {
                    "order": 1,
                    "title": "Hand off the supported ROS2 distro boundary and extension limitations before the onsite team owns the bridge path.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/guides/SUPPORT_MATRIX.md",
                },
                {
                    "order": 2,
                    "title": "Hand off the current bridge validation baseline and accepted residuals for the active release package.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/ros2/ROS2_QUICK_START.md",
                },
            ),
            "exception_review_owner_role": "delivery_lead",
            "exception_review_steps": (
                {
                    "order": 1,
                    "title": "Review the current exception input and expiry dates before the ROS2 bridge handoff leaves the delivery team.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "deployment/security/vulnerability_exceptions.input.json",
                },
                {
                    "order": 2,
                    "title": "Attach the current exception coverage record before the ROS2 bridge residual risk is accepted for steady-state support.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release_evidence/security/vulnerability_exception_report.json",
                },
            ),
            "incident_escalation_owner_role": "customer_operator",
            "incident_escalation_steps": (
                {
                    "order": 1,
                    "title": "Escalate ROS2 bridge degradation through the incident response matrix before the watch window is closed.",
                    "owner_role": "customer_operator",
                    "escalation_target": "docs/guides/INCIDENT_RESPONSE_MATRIX.md",
                },
                {
                    "order": 2,
                    "title": "If bridge recovery fails, transfer control to the rollback owner and suspend further promotion activity.",
                    "owner_role": "delivery_lead",
                    "escalation_target": "rollback_owner",
                },
            ),
            "escalation_closure_owner_role": "delivery_lead",
            "escalation_closure_steps": (
                {
                    "order": 1,
                    "title": "Record the final ROS2 bridge escalation decision, owner, and reopen criteria in the industrial promotion checklist before the incident is marked closed.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
                },
                {
                    "order": 2,
                    "title": "Attach the recovered ROS2 smoke or rollback evidence before clearing the bridge escalation chain.",
                    "owner_role": "rollback_owner",
                    "required_artifact": "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json",
                },
            ),
            "rollback_owner_role": "rollback_owner",
            "rollback_steps": (
                {
                    "order": 1,
                    "title": "Stop the active ROS2 bridge launch and freeze further topic/service changes in the customer environment.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 2,
                    "title": "Restore the previously accepted ROS2 launch files, parameters, and workspace artifacts, then relaunch the baseline bridge path.",
                    "owner_role": "rollback_owner",
                },
                {
                    "order": 3,
                    "title": "Recheck topic and service reachability and archive the rollback confirmation with the release evidence set.",
                    "owner_role": "delivery_lead",
                },
            ),
            "rollback_evidence_owner_role": "rollback_owner",
            "rollback_evidence_archive_steps": (
                {
                    "order": 1,
                    "title": "Archive the restored launch files, parameter snapshot, and rollback command log with the acceptance bundle.",
                    "owner_role": "rollback_owner",
                    "target_path": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Archive the recovered ROS2 smoke or connectivity evidence before the rollback window is closed.",
                    "owner_role": "delivery_lead",
                    "target_path": "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json",
                },
            ),
        },
        "summary": (
            "ROS2 bridge is an extension-node surface and is only supported with a ROS2 Humble runtime plus dedicated live evidence."
        ),
        "deployment_commands": (
            "cd hardware/ros2_ws && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash",
            "ros2 launch agi_walker_ros2 agi_walker.launch.py",
        ),
        "acceptance_checks": (
            "ros2 topic list",
            "ros2 service call /start_simulation std_srvs/srv/Trigger",
            'AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv',
        ),
        "rollback_prerequisites": (
            "确认目标环境仍为 ROS2 Humble，并保留回滚前 bridge launch、params.yaml 与工作区 build 结果。",
            "回滚前先停止 bridge 节点并记录当前 topic/service 连通性。",
        ),
        "non_support_scope": (
            "Other ROS2 distros are outside the formal customer matrix.",
        ),
    },
    {
        "id": "godot_extension",
        "label": "Godot Extension",
        "status": "conditional",
        "default_delivery": False,
        "requires_special_acceptance": True,
        "baseline": "External Godot TCP target or headless environment prepared by the delivery team.",
        "required_evidence": ("godot_headless_live",),
        "reference_documents": (
            "support_matrix",
            "known_limitations_guide",
            "customer_acceptance_checklist",
        ),
        "runbook_entrypoints": (
            "PRODUCTION_DEPLOYMENT_RUNBOOK.md",
            "docs/guides/CUSTOMER_INSTALLATION_GUIDE.md",
            "docs/guides/GODOT_TESTING_GUIDE.md",
        ),
        "execution_template": {
            "operator_roles": (
                {
                    "id": "delivery_lead",
                    "label": "Delivery Lead",
                    "responsibility": "Coordinate Godot extension rollout scope, confirm headless or external runtime prerequisites, and capture onsite evidence.",
                },
                {
                    "id": "customer_operator",
                    "label": "Customer Operator",
                    "responsibility": "Provide the approved Godot executable, scene path, TCP target, and runtime access during the change window.",
                },
                {
                    "id": "rollback_owner",
                    "label": "Rollback Owner",
                    "responsibility": "Own restoration of the previously accepted Godot scene, backend selection, and runtime endpoint configuration.",
                },
            ),
            "upgrade_window_steps": (
                {
                    "order": 1,
                    "title": "Freeze the Godot change window and verify executable path, scene selection, backend mode, and TCP target against the approved runbook.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Apply the Godot environment variables, start the target runtime, and record the exact executable and scene combination used onsite.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 3,
                    "title": "Run the Godot headless and Web integration acceptance checks and attach their reports to the delivery package.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 4,
                    "title": "Confirm the accepted Godot runtime path or transfer control to the rollback owner before the window closes.",
                    "owner_role": "rollback_owner",
                },
            ),
            "handoff_owner_role": "delivery_lead",
            "handoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Transfer the accepted Godot executable, scene path, backend mode, and TCP endpoint snapshot to the onsite operator before handoff.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Confirm the customer operator has the headless smoke path, rollback contact route, and current scene ownership before the runtime is handed over.",
                    "owner_role": "customer_operator",
                },
            ),
            "signoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Do not sign off until the Godot headless and Web integration evidence are attached to the acceptance package.",
                    "owner_role": "customer_operator",
                    "required_artifact": "test_env/godot_headless_smoke/headless_smoke_report.json",
                },
                {
                    "order": 2,
                    "title": "Close the Godot window only after the customer bundle records the accepted scene, backend mode, and operator handoff state.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release/customer_acceptance_bundle.json",
                },
            ),
            "watch_owner_role": "customer_operator",
            "watch_actions": (
                {
                    "order": 1,
                    "title": "Monitor the active Godot runtime path, headless artifact directory, and Web integration health during the first post-cutover watch window.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 2,
                    "title": "Keep the active scene, backend mode, and headless smoke evidence available until the first watch window closes cleanly.",
                    "owner_role": "delivery_lead",
                },
            ),
            "on_call_handoff_owner_role": "delivery_lead",
            "on_call_handoff_records": (
                {
                    "order": 1,
                    "title": "Record the first Godot watch-window outcome, accepted runtime owner, and escalation contact in the customer acceptance bundle before the delivery team leaves the runtime path.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Attach the headless smoke report and accepted scene/backend baseline that the steady-state on-call team must inherit.",
                    "owner_role": "customer_operator",
                    "required_artifact": "test_env/godot_headless_smoke/headless_smoke_report.json",
                },
            ),
            "residual_risk_owner_role": "delivery_lead",
            "residual_risk_handoff_steps": (
                {
                    "order": 1,
                    "title": "Hand off the opt-in Godot support boundary and current environment limitations before the onsite team owns the runtime.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/guides/KNOWN_LIMITATIONS.md",
                },
                {
                    "order": 2,
                    "title": "Hand off the accepted Godot validation baseline and current scene/backend contract for the release package.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/guides/GODOT_TESTING_GUIDE.md",
                },
            ),
            "exception_review_owner_role": "delivery_lead",
            "exception_review_steps": (
                {
                    "order": 1,
                    "title": "Review the current exception input and expiry dates before the Godot runtime leaves onsite rollout mode.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "deployment/security/vulnerability_exceptions.input.json",
                },
                {
                    "order": 2,
                    "title": "Attach the current exception coverage record before the Godot residual risk is accepted for steady-state support.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release_evidence/security/vulnerability_exception_report.json",
                },
            ),
            "incident_escalation_owner_role": "customer_operator",
            "incident_escalation_steps": (
                {
                    "order": 1,
                    "title": "Escalate Godot runtime or endpoint degradation through the incident response matrix before the watch window is closed.",
                    "owner_role": "customer_operator",
                    "escalation_target": "docs/guides/INCIDENT_RESPONSE_MATRIX.md",
                },
                {
                    "order": 2,
                    "title": "If the runtime cannot be stabilized, transfer control to the rollback owner and suspend further promotion activity.",
                    "owner_role": "delivery_lead",
                    "escalation_target": "rollback_owner",
                },
            ),
            "escalation_closure_owner_role": "delivery_lead",
            "escalation_closure_steps": (
                {
                    "order": 1,
                    "title": "Record the final Godot escalation decision, owner, and reopen criteria in the industrial promotion checklist before the incident is marked closed.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
                },
                {
                    "order": 2,
                    "title": "Attach the recovered Godot smoke or rollback evidence before clearing the runtime escalation chain.",
                    "owner_role": "rollback_owner",
                    "required_artifact": "test_env/godot_headless_smoke/headless_smoke_report.json",
                },
            ),
            "rollback_owner_role": "rollback_owner",
            "rollback_steps": (
                {
                    "order": 1,
                    "title": "Stop the active Godot runtime path and freeze endpoint changes before rollback begins.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 2,
                    "title": "Restore the previously accepted scene, executable path, backend selection, and TCP settings, then restart the baseline runtime.",
                    "owner_role": "rollback_owner",
                },
                {
                    "order": 3,
                    "title": "Rerun the agreed health checks for the recovered Godot path and archive rollback confirmation into the delivery evidence set.",
                    "owner_role": "delivery_lead",
                },
            ),
            "rollback_evidence_owner_role": "rollback_owner",
            "rollback_evidence_archive_steps": (
                {
                    "order": 1,
                    "title": "Archive the restored scene, executable path, backend selection, and rollback command log with the customer bundle.",
                    "owner_role": "rollback_owner",
                    "target_path": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Archive the recovered Godot smoke evidence before the rollback window is closed.",
                    "owner_role": "delivery_lead",
                    "target_path": "test_env/godot_headless_smoke/headless_smoke_report.json",
                },
            ),
        },
        "summary": (
            "Godot extension is an opt-in environment and requires explicit runtime preparation before delivery or acceptance."
        ),
        "deployment_commands": (
            "$env:GODOT_EXECUTABLE='<path-to-godot>'; $env:AGI_WALKER_GODOT_HEADLESS_SCENE='demo_generated_biped.tscn'",
            "$env:AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR='test_env/godot_headless_smoke'",
        ),
        "acceptance_checks": (
            'AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv',
            "python -m pytest tests/test_web_godot_session_bridge.py tests/test_web_panel_integration_routes.py -q",
        ),
        "rollback_prerequisites": (
            "确认当前 Godot scene、可执行文件路径和 TCP 端口配置已留档，可恢复到上一个已验收组合。",
            "回滚前保留上一个可工作的 Web backend 选择（legacy 或 godot-agent）与对应外部 Godot 环境。",
        ),
        "non_support_scope": (
            "Zero-config Godot provisioning is not part of the default delivery surface.",
        ),
    },
    {
        "id": "kubernetes_delivery",
        "label": "Helm / Kubernetes Delivery",
        "status": "not_supported",
        "default_delivery": False,
        "requires_special_acceptance": False,
        "baseline": "No current productionized baseline.",
        "required_evidence": (),
        "reference_documents": (
            "support_matrix",
            "known_limitations_guide",
        ),
        "runbook_entrypoints": (
            "PRODUCTION_DEPLOYMENT_RUNBOOK.md",
            "docs/guides/SUPPORT_MATRIX.md",
            "docs/guides/KNOWN_LIMITATIONS.md",
        ),
        "execution_template": {
            "operator_roles": (
                {
                    "id": "delivery_lead",
                    "label": "Delivery Lead",
                    "responsibility": "Own stop-ship decisions when an unsupported Kubernetes request appears and realign the delivery scope to the supported baseline.",
                },
                {
                    "id": "customer_operator",
                    "label": "Customer Operator",
                    "responsibility": "Confirm the customer request has been redirected away from unsupported Kubernetes rollout steps and back to the supported Compose path.",
                },
                {
                    "id": "rollback_owner",
                    "label": "Rollback Owner",
                    "responsibility": "Own cancellation of unsupported rollout actions and restoration of the supported Compose acceptance scope.",
                },
            ),
            "upgrade_window_steps": (
                {
                    "order": 1,
                    "title": "Check the requested topology against the support matrix and mark Kubernetes delivery as outside the supported change window.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Pause the onsite rollout, redirect the request to the Docker Compose baseline, and record the scope correction in the acceptance log.",
                    "owner_role": "customer_operator",
                },
                {
                    "order": 3,
                    "title": "Only reopen delivery after the request is rewritten to the supported Compose path and the unsupported Kubernetes steps are removed.",
                    "owner_role": "rollback_owner",
                },
            ),
            "handoff_owner_role": "delivery_lead",
            "handoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Transfer the stop-ship decision, support-matrix reference, and corrected Compose scope to the customer operator before any onsite action resumes.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Confirm the customer operator understands the unsupported topology has been redirected to the supported Compose path.",
                    "owner_role": "customer_operator",
                },
            ),
            "signoff_checkpoints": (
                {
                    "order": 1,
                    "title": "Do not sign off until the support matrix and known limitations explicitly record Kubernetes as unsupported for this delivery.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/guides/SUPPORT_MATRIX.md",
                },
                {
                    "order": 2,
                    "title": "Only close the corrected window after the unsupported request has been rewritten to the Compose baseline and logged in the acceptance package.",
                    "owner_role": "customer_operator",
                    "required_artifact": "docs/guides/KNOWN_LIMITATIONS.md",
                },
            ),
            "watch_owner_role": "delivery_lead",
            "watch_actions": (
                {
                    "order": 1,
                    "title": "Keep the stop-ship decision active until the unsupported request is fully redirected to the Compose baseline.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 2,
                    "title": "Monitor the acceptance package to ensure no unsupported Kubernetes rollout step reappears during the corrected window.",
                    "owner_role": "customer_operator",
                },
            ),
            "on_call_handoff_owner_role": "delivery_lead",
            "on_call_handoff_records": (
                {
                    "order": 1,
                    "title": "Record the corrected Compose scope, stop-ship owner, and customer on-call contact in the acceptance bundle before onsite activity resumes.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Attach the support-boundary artifacts that the steady-state on-call team must inherit for the corrected scope.",
                    "owner_role": "customer_operator",
                    "required_artifact": "docs/guides/SUPPORT_MATRIX.md",
                },
            ),
            "residual_risk_owner_role": "delivery_lead",
            "residual_risk_handoff_steps": (
                {
                    "order": 1,
                    "title": "Hand off the unsupported topology boundary before the customer team resumes activity on the corrected Compose scope.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/guides/SUPPORT_MATRIX.md",
                },
                {
                    "order": 2,
                    "title": "Hand off the known limitations page so the unsupported request cannot be reintroduced as an implied commitment.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "docs/guides/KNOWN_LIMITATIONS.md",
                },
            ),
            "exception_review_owner_role": "delivery_lead",
            "exception_review_steps": (
                {
                    "order": 1,
                    "title": "Review the current exception input and expiry dates before the corrected Compose scope is handed over as the only supported path.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "deployment/security/vulnerability_exceptions.input.json",
                },
                {
                    "order": 2,
                    "title": "Attach the current exception coverage record before the corrected scope is accepted as the active steady-state baseline.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/release_evidence/security/vulnerability_exception_report.json",
                },
            ),
            "incident_escalation_owner_role": "delivery_lead",
            "incident_escalation_steps": (
                {
                    "order": 1,
                    "title": "Escalate any attempt to reopen Kubernetes rollout work through the incident response matrix and stop promotion immediately.",
                    "owner_role": "delivery_lead",
                    "escalation_target": "docs/guides/INCIDENT_RESPONSE_MATRIX.md",
                },
                {
                    "order": 2,
                    "title": "If scope drift continues, transfer control to the rollback owner and keep the stop-ship state in place.",
                    "owner_role": "delivery_lead",
                    "escalation_target": "rollback_owner",
                },
            ),
            "escalation_closure_owner_role": "delivery_lead",
            "escalation_closure_steps": (
                {
                    "order": 1,
                    "title": "Record the final stop-ship decision and corrected delivery scope in the industrial promotion checklist before the escalation is marked closed.",
                    "owner_role": "delivery_lead",
                    "required_artifact": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
                },
                {
                    "order": 2,
                    "title": "Attach the corrected support-boundary evidence before clearing the Kubernetes stop-ship state.",
                    "owner_role": "rollback_owner",
                    "required_artifact": "docs/guides/KNOWN_LIMITATIONS.md",
                },
            ),
            "rollback_owner_role": "rollback_owner",
            "rollback_steps": (
                {
                    "order": 1,
                    "title": "Cancel the unsupported Kubernetes rollout request and freeze further cluster-side execution.",
                    "owner_role": "rollback_owner",
                },
                {
                    "order": 2,
                    "title": "Restore the supported Docker Compose acceptance scope, runbook references, and sign-off path.",
                    "owner_role": "delivery_lead",
                },
                {
                    "order": 3,
                    "title": "Confirm the release package no longer references unsupported Kubernetes rollout actions before promotion resumes.",
                    "owner_role": "customer_operator",
                },
            ),
            "rollback_evidence_owner_role": "rollback_owner",
            "rollback_evidence_archive_steps": (
                {
                    "order": 1,
                    "title": "Archive the stop-ship decision and corrected Compose scope with the customer acceptance bundle before promotion resumes.",
                    "owner_role": "rollback_owner",
                    "target_path": "test_env/release/customer_acceptance_bundle.json",
                },
                {
                    "order": 2,
                    "title": "Archive the updated support-boundary evidence so the unsupported request cannot be reintroduced silently.",
                    "owner_role": "delivery_lead",
                    "target_path": "docs/guides/KNOWN_LIMITATIONS.md",
                },
            ),
        },
        "summary": (
            "Helm / Kubernetes remains outside the current customer delivery contract and is not part of the formal industrial release surface."
        ),
        "deployment_commands": (
            "No supported deployment command; fall back to the Docker Compose baseline in docs/guides/CUSTOMER_INSTALLATION_GUIDE.md.",
        ),
        "acceptance_checks": (
            "Confirm the delivery request uses the Docker Compose acceptance path instead of a Kubernetes-specific rollout.",
        ),
        "rollback_prerequisites": (
            "If a customer requests Kubernetes delivery, stop release signoff and return to the supported Docker Compose baseline before promotion.",
        ),
        "non_support_scope": (
            "Historical templates do not form a support commitment.",
        ),
    },
)


def _build_extension_execution_template(
    template: Mapping[str, Any] | None,
) -> dict[str, Any]:
    payload = dict(template) if isinstance(template, Mapping) else {}
    operator_roles_payload = payload.get("operator_roles")
    operator_roles_items = (
        list(operator_roles_payload)
        if isinstance(operator_roles_payload, Sequence)
        and not isinstance(operator_roles_payload, (str, bytes))
        else []
    )
    operator_roles = [
        {
            "id": str(item.get("id") or "").strip(),
            "label": str(item.get("label") or "").strip(),
            "responsibility": str(item.get("responsibility") or "").strip(),
        }
        for item in operator_roles_items
        if isinstance(item, Mapping)
    ]

    def _build_step_items(items: Any) -> list[dict[str, Any]]:
        step_items = (
            list(items)
            if isinstance(items, Sequence) and not isinstance(items, (str, bytes))
            else []
        )
        built_steps: list[dict[str, Any]] = []
        for item in step_items:
            if not isinstance(item, Mapping):
                continue
            built_steps.append(
                {
                    "order": int(item.get("order") or 0),
                    "title": str(item.get("title") or "").strip(),
                    "owner_role": str(item.get("owner_role") or "").strip(),
                }
            )
        return built_steps

    def _build_signoff_items(items: Any) -> list[dict[str, Any]]:
        step_items = (
            list(items)
            if isinstance(items, Sequence) and not isinstance(items, (str, bytes))
            else []
        )
        built_items: list[dict[str, Any]] = []
        for item in step_items:
            if not isinstance(item, Mapping):
                continue
            built_items.append(
                {
                    "order": int(item.get("order") or 0),
                    "title": str(item.get("title") or "").strip(),
                    "owner_role": str(item.get("owner_role") or "").strip(),
                    "required_artifact": str(item.get("required_artifact") or "").strip(),
                }
            )
        return built_items

    def _build_archive_items(items: Any) -> list[dict[str, Any]]:
        step_items = (
            list(items)
            if isinstance(items, Sequence) and not isinstance(items, (str, bytes))
            else []
        )
        built_items: list[dict[str, Any]] = []
        for item in step_items:
            if not isinstance(item, Mapping):
                continue
            built_items.append(
                {
                    "order": int(item.get("order") or 0),
                    "title": str(item.get("title") or "").strip(),
                    "owner_role": str(item.get("owner_role") or "").strip(),
                    "target_path": str(item.get("target_path") or "").strip(),
                }
            )
        return built_items

    def _build_escalation_items(items: Any) -> list[dict[str, Any]]:
        step_items = (
            list(items)
            if isinstance(items, Sequence) and not isinstance(items, (str, bytes))
            else []
        )
        built_items: list[dict[str, Any]] = []
        for item in step_items:
            if not isinstance(item, Mapping):
                continue
            built_items.append(
                {
                    "order": int(item.get("order") or 0),
                    "title": str(item.get("title") or "").strip(),
                    "owner_role": str(item.get("owner_role") or "").strip(),
                    "escalation_target": str(
                        item.get("escalation_target") or ""
                    ).strip(),
                }
            )
        return built_items

    return {
        "operator_roles": operator_roles,
        "upgrade_window_steps": _build_step_items(payload.get("upgrade_window_steps")),
        "handoff_owner_role": str(payload.get("handoff_owner_role") or "").strip(),
        "handoff_checkpoints": _build_step_items(payload.get("handoff_checkpoints")),
        "signoff_checkpoints": _build_signoff_items(payload.get("signoff_checkpoints")),
        "watch_owner_role": str(payload.get("watch_owner_role") or "").strip(),
        "watch_actions": _build_step_items(payload.get("watch_actions")),
        "on_call_handoff_owner_role": str(
            payload.get("on_call_handoff_owner_role") or ""
        ).strip(),
        "on_call_handoff_records": _build_signoff_items(
            payload.get("on_call_handoff_records")
        ),
        "residual_risk_owner_role": str(
            payload.get("residual_risk_owner_role") or ""
        ).strip(),
        "residual_risk_handoff_steps": _build_signoff_items(
            payload.get("residual_risk_handoff_steps")
        ),
        "exception_review_owner_role": str(
            payload.get("exception_review_owner_role") or ""
        ).strip(),
        "exception_review_steps": _build_signoff_items(
            payload.get("exception_review_steps")
        ),
        "incident_escalation_owner_role": str(
            payload.get("incident_escalation_owner_role") or ""
        ).strip(),
        "incident_escalation_steps": _build_escalation_items(
            payload.get("incident_escalation_steps")
        ),
        "escalation_closure_owner_role": str(
            payload.get("escalation_closure_owner_role") or ""
        ).strip(),
        "escalation_closure_steps": _build_signoff_items(
            payload.get("escalation_closure_steps")
        ),
        "rollback_owner_role": str(payload.get("rollback_owner_role") or "").strip(),
        "rollback_steps": _build_step_items(payload.get("rollback_steps")),
        "rollback_evidence_owner_role": str(
            payload.get("rollback_evidence_owner_role") or ""
        ).strip(),
        "rollback_evidence_archive_steps": _build_archive_items(
            payload.get("rollback_evidence_archive_steps")
        ),
    }


def _validate_extension_execution_template(payload: Any, *, prefix: str) -> list[str]:
    if not isinstance(payload, Mapping):
        return [f"{prefix} must be an object"]

    errors: list[str] = []
    missing = sorted(EXTENSION_EXECUTION_TEMPLATE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"{prefix} missing required fields: {', '.join(missing)}")

    operator_roles = payload.get("operator_roles")
    role_ids: set[str] = set()
    if not isinstance(operator_roles, list) or not operator_roles:
        errors.append(f"{prefix}.operator_roles must be a non-empty list")
    else:
        for index, item in enumerate(operator_roles, start=1):
            role_prefix = f"{prefix}.operator_roles[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{role_prefix} must be an object")
                continue
            missing_fields = sorted(
                EXTENSION_EXECUTION_TEMPLATE_ROLE_REQUIRED_FIELDS - set(item)
            )
            if missing_fields:
                errors.append(
                    f"{role_prefix} missing required fields: {', '.join(missing_fields)}"
                )
            role_id = item.get("id")
            if not _is_non_empty_string(role_id):
                errors.append(f"{role_prefix}.id must be a non-empty string")
            else:
                role_ids.add(str(role_id))
            if not _is_non_empty_string(item.get("label")):
                errors.append(f"{role_prefix}.label must be a non-empty string")
            if not _is_non_empty_string(item.get("responsibility")):
                errors.append(
                    f"{role_prefix}.responsibility must be a non-empty string"
                )

    rollback_owner_role = payload.get("rollback_owner_role")
    if not _is_non_empty_string(rollback_owner_role):
        errors.append(f"{prefix}.rollback_owner_role must be a non-empty string")
    elif role_ids and str(rollback_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.rollback_owner_role must match one of operator_roles[*].id"
        )
    handoff_owner_role = payload.get("handoff_owner_role")
    if not _is_non_empty_string(handoff_owner_role):
        errors.append(f"{prefix}.handoff_owner_role must be a non-empty string")
    elif role_ids and str(handoff_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.handoff_owner_role must match one of operator_roles[*].id"
        )
    watch_owner_role = payload.get("watch_owner_role")
    if not _is_non_empty_string(watch_owner_role):
        errors.append(f"{prefix}.watch_owner_role must be a non-empty string")
    elif role_ids and str(watch_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.watch_owner_role must match one of operator_roles[*].id"
        )
    on_call_handoff_owner_role = payload.get("on_call_handoff_owner_role")
    if not _is_non_empty_string(on_call_handoff_owner_role):
        errors.append(
            f"{prefix}.on_call_handoff_owner_role must be a non-empty string"
        )
    elif role_ids and str(on_call_handoff_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.on_call_handoff_owner_role must match one of operator_roles[*].id"
        )
    residual_risk_owner_role = payload.get("residual_risk_owner_role")
    if not _is_non_empty_string(residual_risk_owner_role):
        errors.append(
            f"{prefix}.residual_risk_owner_role must be a non-empty string"
        )
    elif role_ids and str(residual_risk_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.residual_risk_owner_role must match one of operator_roles[*].id"
        )
    exception_review_owner_role = payload.get("exception_review_owner_role")
    if not _is_non_empty_string(exception_review_owner_role):
        errors.append(
            f"{prefix}.exception_review_owner_role must be a non-empty string"
        )
    elif role_ids and str(exception_review_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.exception_review_owner_role must match one of operator_roles[*].id"
        )
    incident_escalation_owner_role = payload.get("incident_escalation_owner_role")
    if not _is_non_empty_string(incident_escalation_owner_role):
        errors.append(
            f"{prefix}.incident_escalation_owner_role must be a non-empty string"
        )
    elif role_ids and str(incident_escalation_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.incident_escalation_owner_role must match one of operator_roles[*].id"
        )
    escalation_closure_owner_role = payload.get("escalation_closure_owner_role")
    if not _is_non_empty_string(escalation_closure_owner_role):
        errors.append(
            f"{prefix}.escalation_closure_owner_role must be a non-empty string"
        )
    elif role_ids and str(escalation_closure_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.escalation_closure_owner_role must match one of operator_roles[*].id"
        )
    rollback_evidence_owner_role = payload.get("rollback_evidence_owner_role")
    if not _is_non_empty_string(rollback_evidence_owner_role):
        errors.append(
            f"{prefix}.rollback_evidence_owner_role must be a non-empty string"
        )
    elif role_ids and str(rollback_evidence_owner_role) not in role_ids:
        errors.append(
            f"{prefix}.rollback_evidence_owner_role must match one of operator_roles[*].id"
        )

    for list_name in [
        "upgrade_window_steps",
        "rollback_steps",
        "handoff_checkpoints",
        "watch_actions",
    ]:
        values = payload.get(list_name)
        if not isinstance(values, list) or not values:
            errors.append(f"{prefix}.{list_name} must be a non-empty list")
            continue
        for index, item in enumerate(values, start=1):
            step_prefix = f"{prefix}.{list_name}[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{step_prefix} must be an object")
                continue
            missing_fields = sorted(
                EXTENSION_EXECUTION_TEMPLATE_STEP_REQUIRED_FIELDS - set(item)
            )
            if missing_fields:
                errors.append(
                    f"{step_prefix} missing required fields: {', '.join(missing_fields)}"
                )
            if not _is_positive_int(item.get("order")):
                errors.append(f"{step_prefix}.order must be a positive integer")
            if not _is_non_empty_string(item.get("title")):
                errors.append(f"{step_prefix}.title must be a non-empty string")
            owner_role = item.get("owner_role")
            if not _is_non_empty_string(owner_role):
                errors.append(f"{step_prefix}.owner_role must be a non-empty string")
            elif role_ids and str(owner_role) not in role_ids:
                errors.append(
                    f"{step_prefix}.owner_role must match one of operator_roles[*].id"
                )
    for list_name in [
        "signoff_checkpoints",
        "on_call_handoff_records",
        "residual_risk_handoff_steps",
        "exception_review_steps",
        "escalation_closure_steps",
    ]:
        values = payload.get(list_name)
        if not isinstance(values, list) or not values:
            errors.append(f"{prefix}.{list_name} must be a non-empty list")
            continue
        for index, item in enumerate(values, start=1):
            step_prefix = f"{prefix}.{list_name}[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{step_prefix} must be an object")
                continue
            missing_fields = sorted(
                EXTENSION_EXECUTION_TEMPLATE_SIGNOFF_REQUIRED_FIELDS - set(item)
            )
            if missing_fields:
                errors.append(
                    f"{step_prefix} missing required fields: {', '.join(missing_fields)}"
                )
            if not _is_positive_int(item.get("order")):
                errors.append(f"{step_prefix}.order must be a positive integer")
            if not _is_non_empty_string(item.get("title")):
                errors.append(f"{step_prefix}.title must be a non-empty string")
            owner_role = item.get("owner_role")
            if not _is_non_empty_string(owner_role):
                errors.append(f"{step_prefix}.owner_role must be a non-empty string")
            elif role_ids and str(owner_role) not in role_ids:
                errors.append(
                    f"{step_prefix}.owner_role must match one of operator_roles[*].id"
                )
            if not _is_non_empty_string(item.get("required_artifact")):
                errors.append(
                    f"{step_prefix}.required_artifact must be a non-empty string"
                )
    for list_name in ["incident_escalation_steps"]:
        values = payload.get(list_name)
        if not isinstance(values, list) or not values:
            errors.append(f"{prefix}.{list_name} must be a non-empty list")
            continue
        for index, item in enumerate(values, start=1):
            step_prefix = f"{prefix}.{list_name}[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{step_prefix} must be an object")
                continue
            missing_fields = sorted(
                EXTENSION_EXECUTION_TEMPLATE_ESCALATION_REQUIRED_FIELDS - set(item)
            )
            if missing_fields:
                errors.append(
                    f"{step_prefix} missing required fields: {', '.join(missing_fields)}"
                )
            if not _is_positive_int(item.get("order")):
                errors.append(f"{step_prefix}.order must be a positive integer")
            if not _is_non_empty_string(item.get("title")):
                errors.append(f"{step_prefix}.title must be a non-empty string")
            owner_role = item.get("owner_role")
            if not _is_non_empty_string(owner_role):
                errors.append(f"{step_prefix}.owner_role must be a non-empty string")
            elif role_ids and str(owner_role) not in role_ids:
                errors.append(
                    f"{step_prefix}.owner_role must match one of operator_roles[*].id"
                )
            if not _is_non_empty_string(item.get("escalation_target")):
                errors.append(
                    f"{step_prefix}.escalation_target must be a non-empty string"
                )
    for list_name in ["rollback_evidence_archive_steps"]:
        values = payload.get(list_name)
        if not isinstance(values, list) or not values:
            errors.append(f"{prefix}.{list_name} must be a non-empty list")
            continue
        for index, item in enumerate(values, start=1):
            step_prefix = f"{prefix}.{list_name}[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{step_prefix} must be an object")
                continue
            missing_fields = sorted(
                EXTENSION_EXECUTION_TEMPLATE_ARCHIVE_REQUIRED_FIELDS - set(item)
            )
            if missing_fields:
                errors.append(
                    f"{step_prefix} missing required fields: {', '.join(missing_fields)}"
                )
            if not _is_positive_int(item.get("order")):
                errors.append(f"{step_prefix}.order must be a positive integer")
            if not _is_non_empty_string(item.get("title")):
                errors.append(f"{step_prefix}.title must be a non-empty string")
            owner_role = item.get("owner_role")
            if not _is_non_empty_string(owner_role):
                errors.append(f"{step_prefix}.owner_role must be a non-empty string")
            elif role_ids and str(owner_role) not in role_ids:
                errors.append(
                    f"{step_prefix}.owner_role must match one of operator_roles[*].id"
                )
            if not _is_non_empty_string(item.get("target_path")):
                errors.append(f"{step_prefix}.target_path must be a non-empty string")
    return errors


def default_release_contract_versions() -> list[dict[str, str]]:
    return [
        {"name": "release_manifest", "version": RELEASE_CONTRACT_VERSION},
        {"name": "capability_matrix", "version": CAPABILITY_MATRIX_VERSION},
        {"name": "workflow", "version": WORKFLOW_CONTRACT_VERSION},
        {"name": "training_run", "version": TRAINING_CONTRACT_VERSION},
        {"name": "distributed_monitor", "version": "1.0"},
        {"name": "distributed_smoke_report", "version": "1.0"},
        {"name": "godot_session_status", "version": "1.0"},
    ]


def default_release_test_evidence() -> list[dict[str, Any]]:
    return [
        {
            "name": "clean_checkout_smoke",
            "required": True,
            "status": "blocked",
            "summary": "Canonical release evidence now requires a passing clean-checkout smoke report that proves the default smoke runner leaves the checkout clean after sequential runs.",
            "command": "python tools/run_clean_checkout_smoke.py --output-root test_env/release_evidence/clean_checkout_smoke --report-file test_env/release_evidence/clean_checkout_smoke_report.json",
            "artifact_path": "test_env/release_evidence/clean_checkout_smoke_report.json",
        },
        {
            "name": "non_live_gate",
            "required": True,
            "status": "passed",
            "summary": "Default non-live pytest gate is expected to pass; attach a structured report to attest the latest pytest summary.",
            "command": 'python -m pytest -m "not live" -q',
            "artifact_path": "test_env/release_evidence/non_live_gate_report.json",
        },
        {
            "name": "release_contracts_and_capability_matrix",
            "required": True,
            "status": "passed",
            "summary": "Capability matrix, MCP surface, Web routes, and release contracts are covered by targeted tests.",
            "command": "python -m pytest tests/test_release_contracts.py tests/test_release_artifact_builder.py tests/test_capability_matrix.py tests/test_mcp_tools.py tests/test_mcp_server.py tests/test_web_panel_aux_apis.py -q",
            "artifact_path": "test_env/release_evidence/release_contracts_and_capability_matrix_report.json",
        },
        {
            "name": "distributed_runtime_live",
            "required": False,
            "status": "opt_in",
            "summary": "Attach a passing distributed smoke report to promote distributed runtime from diagnostic_ready to ready.",
            "command": "python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json",
            "artifact_path": "test_env/distributed_smoke/distributed_smoke_report.json",
        },
        {
            "name": "godot_headless_live",
            "required": False,
            "status": "opt_in",
            "summary": "Godot headless smoke is intentionally opt-in and requires a local Godot executable plus scene assets.",
            "command": 'AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv',
            "artifact_path": "test_env/godot_headless_smoke/headless_smoke_report.json",
        },
        {
            "name": "ros2_bridge_live",
            "required": False,
            "status": "opt_in",
            "summary": "ROS2 bridge smoke is intentionally opt-in and requires a ROS2 Humble runtime.",
            "command": 'AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv',
            "artifact_path": "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json",
        },
    ]


def default_customer_acceptance_documents() -> list[dict[str, Any]]:
    return [
        {"name": "readme", "path": "README.md", "required": True},
        {"name": "current_status", "path": "docs/CURRENT_STATUS.md", "required": True},
        {"name": "release_guide", "path": "docs/guides/RELEASE_GUIDE.md", "required": True},
        {
            "name": "deployment_matrix",
            "path": "docs/guides/DEPLOYMENT_MATRIX.md",
            "required": True,
        },
        {
            "name": "customer_installation_guide",
            "path": "docs/guides/CUSTOMER_INSTALLATION_GUIDE.md",
            "required": True,
        },
        {
            "name": "support_matrix",
            "path": "docs/guides/SUPPORT_MATRIX.md",
            "required": True,
        },
        {
            "name": "capacity_and_scale",
            "path": "docs/guides/CAPACITY_AND_SCALE.md",
            "required": True,
        },
        {
            "name": "customer_acceptance_checklist",
            "path": "docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md",
            "required": True,
        },
        {
            "name": "known_limitations_guide",
            "path": "docs/guides/KNOWN_LIMITATIONS.md",
            "required": True,
        },
        {
            "name": "production_runbook",
            "path": "PRODUCTION_DEPLOYMENT_RUNBOOK.md",
            "required": True,
        },
        {
            "name": "security_baseline",
            "path": "docs/guides/SECURITY_BASELINE.md",
            "required": True,
        },
        {
            "name": "audit_trail_policy",
            "path": "docs/guides/AUDIT_TRAIL_POLICY.md",
            "required": True,
        },
        {
            "name": "backup_restore_runbook",
            "path": "docs/guides/BACKUP_RESTORE_RUNBOOK.md",
            "required": True,
        },
        {
            "name": "incident_response_matrix",
            "path": "docs/guides/INCIDENT_RESPONSE_MATRIX.md",
            "required": True,
        },
    ]


def default_customer_acceptance_reports(channel: str) -> list[dict[str, Any]]:
    reports: list[dict[str, Any]] = []
    if channel == "industrial":
        reports.extend(
            [
                {
                    "name": "industrial_release_readiness",
                    "path": "test_env/industrial_release_readiness_ready/industrial_release_readiness_report.json",
                    "required": False,
                },
                {
                    "name": "industrial_promotion_checklist",
                    "path": "test_env/industrial_promotion_ready/industrial_promotion_checklist.json",
                    "required": False,
                },
                {
                    "name": "industrial_delivery_rehearsal_report",
                    "path": "industrial_delivery_rehearsal_report.json",
                    "required": False,
                },
            ]
        )
    else:
        reports.append(
            {
                "name": "release_readiness",
                "path": "test_env/release_readiness_ready/release_readiness_report.json",
                "required": False,
            }
        )
        if channel == "stable":
            reports.append(
                {
                    "name": "stable_promotion_checklist",
                    "path": "test_env/stable_promotion_ready/stable_promotion_checklist.json",
                    "required": False,
                }
            )
    reports.extend(
        [
            {
                "name": "security_posture_report",
                "path": "test_env/release_evidence/security/security_posture_report.json",
                "required": False,
            },
            {
                "name": "sbom_artifact",
                "path": "test_env/release_evidence/security/sbom.json",
                "required": False,
            },
            {
                "name": "python_vulnerability_scan_report",
                "path": "test_env/release_evidence/security/python_vuln_scan_report.json",
                "required": False,
            },
            {
                "name": "container_vulnerability_scan_report",
                "path": "test_env/release_evidence/security/container_vuln_scan_report.json",
                "required": False,
            },
            {
                "name": "vulnerability_exception_report",
                "path": "test_env/release_evidence/security/vulnerability_exception_report.json",
                "required": False,
            },
            {
                "name": "vulnerability_exception_review",
                "path": default_vulnerability_exception_review_report_path(),
                "required": False,
            },
            {
                "name": "customer_external_bindings_closure",
                "path": default_customer_external_bindings_closure_report_path(),
                "required": False,
            },
            {
                "name": "external_mainline_execution_plan",
                "path": default_external_mainline_execution_plan_path(),
                "required": False,
            },
            {
                "name": "external_mainline_input_checklist",
                "path": default_external_mainline_input_checklist_report_path(),
                "required": False,
            },
            {
                "name": "release_ops_execution",
                "path": default_release_ops_execution_report_path(),
                "required": False,
            },
            {
                "name": "backup_restore_rehearsal_report",
                "path": "test_env/release_evidence/security/backup_restore_rehearsal_report.json",
                "required": False,
            },
            *default_extension_execution_evidence_reports(),
        ]
    )
    return reports


def default_extension_execution_evidence_reports() -> list[dict[str, Any]]:
    return [
        {
            "name": "extension_on_call_rehearsal",
            "path": "test_env/release_evidence/operations/extension_on_call_rehearsal_report.json",
            "required": False,
        },
        {
            "name": "extension_exception_review_schedule",
            "path": "test_env/release_evidence/operations/extension_exception_review_schedule_report.json",
            "required": False,
        },
        {
            "name": "extension_escalation_closure",
            "path": "test_env/release_evidence/operations/extension_escalation_closure_report.json",
            "required": False,
        },
        {
            "name": "customer_external_bindings_confirmation",
            "path": default_customer_external_bindings_confirmation_report_path(),
            "required": False,
        },
    ]


def default_extension_execution_instance_artifact_path() -> str:
    return "test_env/release_evidence/operations/extension_execution_instance.json"


def default_extension_execution_schedule_artifact_path() -> str:
    return "test_env/release_evidence/operations/extension_execution_schedule.json"


def default_extension_execution_actuals_artifact_path() -> str:
    return "test_env/release_evidence/operations/extension_execution_actuals.json"


def default_placeholder_external_bindings_config_path() -> str:
    return "deployment/customer_delivery.external_bindings.json"


def default_rehearsal_external_bindings_config_path() -> str:
    return "deployment/customer_delivery.external_bindings.rehearsal.json"


def default_customer_external_bindings_config_path() -> str:
    return "deployment/customer_delivery.external_bindings.customer.json"


def default_customer_external_bindings_confirmation_report_path() -> str:
    return (
        "test_env/release_evidence/operations/"
        "customer_external_bindings_confirmation_report.json"
    )


def default_customer_external_bindings_closure_report_path() -> str:
    return (
        "test_env/release_evidence/operations/"
        "customer_external_bindings_closure_report.json"
    )


def default_vulnerability_exception_review_report_path() -> str:
    return "test_env/release_evidence/security/vulnerability_exception_review_report.json"


def default_external_mainline_execution_plan_path() -> str:
    return (
        "test_env/release_evidence/operations/"
        "external_mainline_execution_plan.json"
    )


def default_external_mainline_input_checklist_report_path() -> str:
    return (
        "test_env/release_evidence/operations/"
        "external_mainline_input_checklist_report.json"
    )


def default_release_ops_execution_report_path() -> str:
    return (
        "test_env/release_evidence/operations/"
        "release_ops_execution_report.json"
    )


def default_release_manifest_artifact_path() -> str:
    return "test_env/release/release_manifest.json"


def default_external_mainline_inputs_path() -> str:
    return "deployment/external_mainline.inputs.json"


def default_canonical_industrial_delivery_rehearsal_report_path() -> str:
    return (
        "test_env/release_rehearsal_industrial/"
        "industrial_delivery_rehearsal_report.json"
    )


def build_run_worktree_release_blocker_command(
    *,
    source_root: Any = None,
    output_root: Any = None,
    cleanup_report_path: Any = None,
    tracked_review_report_path: Any = None,
    report_path: Any = None,
) -> str:
    resolved_source_root = _artifact_optional_string(source_root) or "."
    resolved_output_root = _artifact_optional_string(output_root) or str(
        Path("test_env") / "worktree_cleanup"
    )
    default_cleanup_report_path = str(
        Path(resolved_output_root) / "worktree_cleanup_report.json"
    )
    default_tracked_review_report_path = str(
        Path(resolved_output_root) / "tracked_artifact_review_report.json"
    )
    default_report_path = str(
        Path(resolved_output_root) / "worktree_release_blocker_report.json"
    )
    resolved_cleanup_report = (
        _artifact_optional_string(cleanup_report_path) or default_cleanup_report_path
    )
    resolved_tracked_review_report = (
        _artifact_optional_string(tracked_review_report_path)
        or default_tracked_review_report_path
    )
    resolved_report_path = _artifact_optional_string(report_path) or default_report_path

    command = "python tools/run_worktree_release_blocker.py"
    if resolved_source_root != ".":
        command += f" --source-root {resolved_source_root}"
    if resolved_output_root != str(Path("test_env") / "worktree_cleanup"):
        command += f" --output-root {resolved_output_root}"
    if resolved_cleanup_report != default_cleanup_report_path:
        command += f" --cleanup-report {resolved_cleanup_report}"
    if resolved_tracked_review_report != default_tracked_review_report_path:
        command += f" --review-report {resolved_tracked_review_report}"
    if resolved_report_path != default_report_path:
        command += f" --report-file {resolved_report_path}"
    return command


def default_industrial_delivery_reports() -> list[dict[str, Any]]:
    return [
        {
            "name": "security_posture_report",
            "path": "test_env/release_evidence/security/security_posture_report.json",
            "required": False,
        },
        {
            "name": "vulnerability_remediation_report",
            "path": "test_env/release_evidence/security/vulnerability_remediation_report.json",
            "required": False,
        },
        {
            "name": "sbom_artifact",
            "path": "test_env/release_evidence/security/sbom.json",
            "required": True,
        },
        {
            "name": "python_vulnerability_scan_report",
            "path": "test_env/release_evidence/security/python_vuln_scan_report.json",
            "required": True,
        },
        {
            "name": "container_vulnerability_scan_report",
            "path": "test_env/release_evidence/security/container_vuln_scan_report.json",
            "required": True,
        },
        {
            "name": "backup_restore_rehearsal_report",
            "path": "test_env/release_evidence/security/backup_restore_rehearsal_report.json",
            "required": True,
        },
    ]


def build_extension_support_surface(
    *,
    project_root: str | Path | None = None,
    acceptance_documents: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    hydrated_documents = _hydrate_customer_acceptance_items(
        acceptance_documents or default_customer_acceptance_documents(),
        resolved_root,
    )
    documents_by_name = {
        str(item.get("name")): dict(item)
        for item in hydrated_documents
        if _is_non_empty_string(item.get("name"))
    }
    declaration_documents = [
        dict(documents_by_name.get(name, {"name": name}))
        for name in EXTENSION_SUPPORT_DECLARATION_DOCUMENT_NAMES
    ]
    missing_documents = [
        str(item.get("path"))
        if _is_non_empty_string(item.get("path"))
        else str(item.get("name"))
        for item in declaration_documents
        if item.get("exists") is not True
    ]

    profiles: list[dict[str, Any]] = []
    declared_profiles = 0
    conditional_profiles = 0
    not_supported_profiles = 0
    for spec in EXTENSION_SUPPORT_PROFILE_SPECS:
        reference_paths: list[str] = []
        declared = True
        for document_name in spec["reference_documents"]:
            document = dict(documents_by_name.get(document_name, {"name": document_name}))
            path = document.get("path")
            reference_paths.append(
                str(path)
                if _is_non_empty_string(path)
                else str(document.get("name"))
            )
            if document.get("exists") is not True:
                declared = False
        if declared:
            declared_profiles += 1
        if spec["status"] == "conditional":
            conditional_profiles += 1
        if spec["status"] == "not_supported":
            not_supported_profiles += 1
        profiles.append(
            {
                "id": spec["id"],
                "label": spec["label"],
                "status": spec["status"],
                "declared": declared,
                "default_delivery": spec["default_delivery"],
                "requires_special_acceptance": spec["requires_special_acceptance"],
                "baseline": spec["baseline"],
                "required_evidence": list(spec["required_evidence"]),
                "reference_paths": reference_paths,
                "runbook_entrypoints": list(spec["runbook_entrypoints"]),
                "execution_template": _build_extension_execution_template(
                    spec.get("execution_template")
                ),
                "deployment_commands": list(spec["deployment_commands"]),
                "acceptance_checks": list(spec["acceptance_checks"]),
                "rollback_prerequisites": list(spec["rollback_prerequisites"]),
                "summary": spec["summary"],
                "non_support_scope": list(spec["non_support_scope"]),
            }
        )

    required_profiles = len(EXTENSION_SUPPORT_PROFILE_SPECS)
    status = "ready" if declared_profiles == required_profiles else "blocked"
    summary = (
        f"Extension support surface {status}: "
        f"{declared_profiles}/{required_profiles} profiles declared, "
        f"conditional={conditional_profiles}, not_supported={not_supported_profiles}."
    )
    if missing_documents:
        summary += " Missing boundary documents: " + ", ".join(missing_documents) + "."

    return {
        "status": status,
        "summary": summary,
        "required_profiles": required_profiles,
        "declared_profiles": declared_profiles,
        "conditional_profiles": conditional_profiles,
        "not_supported_profiles": not_supported_profiles,
        "missing_documents": list(dict.fromkeys(missing_documents)),
        "profiles": to_jsonable(profiles),
    }


def validate_extension_support_surface(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["extension_support_surface must be an object"]

    errors: list[str] = []
    missing = sorted(EXTENSION_SUPPORT_SURFACE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "extension_support_surface missing required fields: " + ", ".join(missing)
        )

    if payload.get("status") not in EXTENSION_SUPPORT_SURFACE_STATUSES:
        errors.append(
            "extension_support_surface.status must be one of "
            f"{sorted(EXTENSION_SUPPORT_SURFACE_STATUSES)}"
        )
    if "summary" in payload and not _is_non_empty_string(payload.get("summary")):
        errors.append("extension_support_surface.summary must be a non-empty string")

    for field in [
        "required_profiles",
        "declared_profiles",
        "conditional_profiles",
        "not_supported_profiles",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(f"extension_support_surface.{field} must be a non-negative integer")
    if (
        _is_non_negative_int(payload.get("required_profiles"))
        and _is_non_negative_int(payload.get("declared_profiles"))
        and payload.get("declared_profiles") > payload.get("required_profiles")
    ):
        errors.append(
            "extension_support_surface.declared_profiles must be <= required_profiles"
        )

    missing_documents = payload.get("missing_documents")
    if not isinstance(missing_documents, list):
        errors.append("extension_support_surface.missing_documents must be a list")
    else:
        for index, item in enumerate(missing_documents, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    f"extension_support_surface.missing_documents[{index}] must be a non-empty string"
                )

    profiles = payload.get("profiles")
    if not isinstance(profiles, list):
        errors.append("extension_support_surface.profiles must be a list")
        return errors

    for index, item in enumerate(profiles, start=1):
        prefix = f"extension_support_surface.profiles[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        missing_fields = sorted(EXTENSION_SUPPORT_PROFILE_REQUIRED_FIELDS - set(item))
        if missing_fields:
            errors.append(f"{prefix} missing required fields: {', '.join(missing_fields)}")
        if not _is_non_empty_string(item.get("id")):
            errors.append(f"{prefix}.id must be a non-empty string")
        if not _is_non_empty_string(item.get("label")):
            errors.append(f"{prefix}.label must be a non-empty string")
        if item.get("status") not in EXTENSION_SUPPORT_PROFILE_STATUSES:
            errors.append(
                f"{prefix}.status must be one of {sorted(EXTENSION_SUPPORT_PROFILE_STATUSES)}"
            )
        for field in ["declared", "default_delivery", "requires_special_acceptance"]:
            if field in item and not isinstance(item.get(field), bool):
                errors.append(f"{prefix}.{field} must be a boolean")
        if not _is_non_empty_string(item.get("baseline")):
            errors.append(f"{prefix}.baseline must be a non-empty string")
        if not _is_non_empty_string(item.get("summary")):
            errors.append(f"{prefix}.summary must be a non-empty string")
        for list_name in [
            "required_evidence",
            "reference_paths",
            "runbook_entrypoints",
            "deployment_commands",
            "acceptance_checks",
            "rollback_prerequisites",
            "non_support_scope",
        ]:
            values = item.get(list_name)
            if not isinstance(values, list):
                errors.append(f"{prefix}.{list_name} must be a list")
                continue
            for list_index, list_item in enumerate(values, start=1):
                if not _is_non_empty_string(list_item):
                    errors.append(
                        f"{prefix}.{list_name}[{list_index}] must be a non-empty string"
                    )
        errors.extend(
            _validate_extension_execution_template(
                item.get("execution_template"),
                prefix=f"{prefix}.execution_template",
            )
        )
    return errors


def build_extension_execution_plan(
    extension_support_surface: Mapping[str, Any] | None,
) -> dict[str, Any]:
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else {}
    )
    profiles_payload = surface.get("profiles")
    profile_items = (
        list(profiles_payload)
        if isinstance(profiles_payload, Sequence) and not isinstance(profiles_payload, (str, bytes))
        else []
    )
    plan_profiles: list[dict[str, Any]] = []
    declared_profiles = 0
    actionable_profiles = 0
    special_acceptance_profiles = 0
    for item in profile_items:
        if not isinstance(item, Mapping):
            continue
        declared = item.get("declared") is True
        status = str(item.get("status") or "not_supported")
        actionable = declared and status != "not_supported"
        requires_special_acceptance = item.get("requires_special_acceptance") is True
        if declared:
            declared_profiles += 1
        if actionable:
            actionable_profiles += 1
        if declared and requires_special_acceptance:
            special_acceptance_profiles += 1
        plan_profiles.append(
            {
                "id": str(item.get("id") or ""),
                "label": str(item.get("label") or ""),
                "status": status,
                "declared": declared,
                "actionable": actionable,
                "default_delivery": item.get("default_delivery") is True,
                "requires_special_acceptance": requires_special_acceptance,
                "runbook_entrypoints": [
                    str(command).strip()
                    for command in item.get("runbook_entrypoints", [])
                    if _is_non_empty_string(command)
                ],
                "execution_template": _build_extension_execution_template(
                    item.get("execution_template")
                    if isinstance(item.get("execution_template"), Mapping)
                    else {}
                ),
                "deployment_commands": [
                    str(command).strip()
                    for command in item.get("deployment_commands", [])
                    if _is_non_empty_string(command)
                ],
                "acceptance_checks": [
                    str(command).strip()
                    for command in item.get("acceptance_checks", [])
                    if _is_non_empty_string(command)
                ],
                "rollback_prerequisites": [
                    str(command).strip()
                    for command in item.get("rollback_prerequisites", [])
                    if _is_non_empty_string(command)
                ],
            }
        )

    status = (
        "ready"
        if surface.get("status") == "ready"
        and declared_profiles == _coerce_non_negative_int(surface.get("declared_profiles"))
        else "blocked"
    )
    summary = (
        f"Extension execution plan {status}: "
        f"{declared_profiles} declared profiles, "
        f"{actionable_profiles} actionable profiles, "
        f"{special_acceptance_profiles} requiring special acceptance, "
        "onsite execution templates, watch actions, on-call handoff records, residual risk handoffs, exception review steps, escalation closure evidence, signoff checkpoints, and rollback evidence duties attached."
    )
    return {
        "status": status,
        "summary": summary,
        "declared_profiles": declared_profiles,
        "actionable_profiles": actionable_profiles,
        "special_acceptance_profiles": special_acceptance_profiles,
        "profiles": to_jsonable(plan_profiles),
    }


def validate_extension_execution_plan(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["extension_execution_plan must be an object"]

    errors: list[str] = []
    missing = sorted(EXTENSION_EXECUTION_PLAN_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "extension_execution_plan missing required fields: " + ", ".join(missing)
        )
    if payload.get("status") not in EXTENSION_EXECUTION_PLAN_STATUSES:
        errors.append(
            "extension_execution_plan.status must be one of "
            f"{sorted(EXTENSION_EXECUTION_PLAN_STATUSES)}"
        )
    if "summary" in payload and not _is_non_empty_string(payload.get("summary")):
        errors.append("extension_execution_plan.summary must be a non-empty string")
    for field in [
        "declared_profiles",
        "actionable_profiles",
        "special_acceptance_profiles",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(f"extension_execution_plan.{field} must be a non-negative integer")
    if (
        _is_non_negative_int(payload.get("declared_profiles"))
        and _is_non_negative_int(payload.get("actionable_profiles"))
        and payload.get("actionable_profiles") > payload.get("declared_profiles")
    ):
        errors.append(
            "extension_execution_plan.actionable_profiles must be <= declared_profiles"
        )
    if (
        _is_non_negative_int(payload.get("declared_profiles"))
        and _is_non_negative_int(payload.get("special_acceptance_profiles"))
        and payload.get("special_acceptance_profiles") > payload.get("declared_profiles")
    ):
        errors.append(
            "extension_execution_plan.special_acceptance_profiles must be <= declared_profiles"
        )

    profiles = payload.get("profiles")
    if not isinstance(profiles, list):
        errors.append("extension_execution_plan.profiles must be a list")
        return errors

    for index, item in enumerate(profiles, start=1):
        prefix = f"extension_execution_plan.profiles[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        missing_fields = sorted(EXTENSION_EXECUTION_PROFILE_REQUIRED_FIELDS - set(item))
        if missing_fields:
            errors.append(f"{prefix} missing required fields: {', '.join(missing_fields)}")
        for field in ["id", "label"]:
            if field in item and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if item.get("status") not in EXTENSION_SUPPORT_PROFILE_STATUSES:
            errors.append(
                f"{prefix}.status must be one of {sorted(EXTENSION_SUPPORT_PROFILE_STATUSES)}"
            )
        for field in [
            "declared",
            "actionable",
            "default_delivery",
            "requires_special_acceptance",
        ]:
            if field in item and not isinstance(item.get(field), bool):
                errors.append(f"{prefix}.{field} must be a boolean")
        for list_name in [
            "runbook_entrypoints",
            "deployment_commands",
            "acceptance_checks",
            "rollback_prerequisites",
        ]:
            values = item.get(list_name)
            if not isinstance(values, list):
                errors.append(f"{prefix}.{list_name} must be a list")
                continue
            for list_index, list_item in enumerate(values, start=1):
                if not _is_non_empty_string(list_item):
                    errors.append(
                        f"{prefix}.{list_name}[{list_index}] must be a non-empty string"
                    )
        errors.extend(
            _validate_extension_execution_template(
                item.get("execution_template"),
                prefix=f"{prefix}.execution_template",
            )
        )
    return errors


def _build_extension_execution_evidence_payload(
    *,
    extension_support_surface: Mapping[str, Any] | None,
    reports: Sequence[Mapping[str, Any]] | None,
) -> dict[str, Any]:
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else {}
    )
    plan = build_extension_execution_plan(surface)
    hydrated_reports = [
        dict(item) for item in reports if isinstance(item, Mapping)
    ] if reports is not None else []
    reports_by_name = {
        str(item.get("name")): item
        for item in hydrated_reports
        if _is_non_empty_string(item.get("name"))
    }
    required_reports = default_extension_execution_evidence_reports()
    required_report_names = [str(item["name"]) for item in required_reports]

    def _is_ready(name: str) -> bool:
        current = reports_by_name.get(name, {})
        return (
            current.get("exists") is True
            and current.get("status") == "passed"
        )

    on_call_rehearsal_attested = _is_ready("extension_on_call_rehearsal")
    exception_review_scheduled = _is_ready("extension_exception_review_schedule")
    escalation_closure_attested = _is_ready("extension_escalation_closure")
    external_bindings_confirmation_attested = _is_ready(
        "customer_external_bindings_confirmation"
    )
    ready_reports = sum(
        1
        for flag in [
            on_call_rehearsal_attested,
            exception_review_scheduled,
            escalation_closure_attested,
            external_bindings_confirmation_attested,
        ]
        if flag
    )
    missing_reports: list[str] = []
    for item in required_reports:
        current = reports_by_name.get(str(item["name"]), {})
        if current.get("exists") is True and current.get("status") == "passed":
            continue
        path_value = current.get("path") or item.get("path") or item.get("name")
        missing_reports.append(str(path_value))
    missing_reports = list(dict.fromkeys(missing_reports))

    status = (
        "ready"
        if plan.get("status") == "ready" and ready_reports == len(required_report_names)
        else "blocked"
    )
    summary = (
        f"Extension execution evidence {status}: "
        f"{ready_reports}/{len(required_report_names)} reports passed, "
        f"{plan.get('declared_profiles', 0)} declared profiles, "
        f"{plan.get('actionable_profiles', 0)} actionable profiles."
    )
    if missing_reports:
        summary += " Missing reports: " + ", ".join(missing_reports) + "."

    return {
        "status": status,
        "summary": summary,
        "required_reports": len(required_report_names),
        "ready_reports": ready_reports,
        "declared_profiles": _coerce_non_negative_int(plan.get("declared_profiles")),
        "actionable_profiles": _coerce_non_negative_int(plan.get("actionable_profiles")),
        "on_call_rehearsal_attested": on_call_rehearsal_attested,
        "exception_review_scheduled": exception_review_scheduled,
        "escalation_closure_attested": escalation_closure_attested,
        "external_bindings_confirmation_attested": external_bindings_confirmation_attested,
        "missing_reports": missing_reports,
        "reports": to_jsonable(hydrated_reports),
    }


def build_extension_execution_evidence(
    *,
    project_root: str | Path | None = None,
    extension_support_surface: Mapping[str, Any] | None = None,
    reports: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else dict(
            build_customer_delivery_surface(project_root=resolved_root).get(
                "extension_support_surface",
                {},
            )
        )
    )
    hydrated_reports = (
        [dict(item) for item in reports if isinstance(item, Mapping)]
        if reports is not None
        else _hydrate_customer_acceptance_items(
            default_extension_execution_evidence_reports(),
            resolved_root,
        )
    )
    return _build_extension_execution_evidence_payload(
        extension_support_surface=surface,
        reports=hydrated_reports,
    )


def validate_extension_execution_evidence(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["extension_execution_evidence must be an object"]

    errors: list[str] = []
    missing = sorted(EXTENSION_EXECUTION_EVIDENCE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "extension_execution_evidence missing required fields: " + ", ".join(missing)
        )
    if payload.get("status") not in EXTENSION_EXECUTION_EVIDENCE_STATUSES:
        errors.append(
            "extension_execution_evidence.status must be one of "
            f"{sorted(EXTENSION_EXECUTION_EVIDENCE_STATUSES)}"
        )
    if "summary" in payload and not _is_non_empty_string(payload.get("summary")):
        errors.append("extension_execution_evidence.summary must be a non-empty string")
    for field in [
        "required_reports",
        "ready_reports",
        "declared_profiles",
        "actionable_profiles",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(
                f"extension_execution_evidence.{field} must be a non-negative integer"
            )
    if (
        _is_non_negative_int(payload.get("required_reports"))
        and _is_non_negative_int(payload.get("ready_reports"))
        and payload.get("ready_reports") > payload.get("required_reports")
    ):
        errors.append(
            "extension_execution_evidence.ready_reports must be <= required_reports"
        )
    if (
        _is_non_negative_int(payload.get("declared_profiles"))
        and _is_non_negative_int(payload.get("actionable_profiles"))
        and payload.get("actionable_profiles") > payload.get("declared_profiles")
    ):
        errors.append(
            "extension_execution_evidence.actionable_profiles must be <= declared_profiles"
        )
    for field in [
        "on_call_rehearsal_attested",
        "exception_review_scheduled",
        "escalation_closure_attested",
        "external_bindings_confirmation_attested",
    ]:
        if field in payload and not isinstance(payload.get(field), bool):
            errors.append(f"extension_execution_evidence.{field} must be a boolean")
    missing_reports = payload.get("missing_reports")
    if not isinstance(missing_reports, list):
        errors.append("extension_execution_evidence.missing_reports must be a list")
    else:
        for index, item in enumerate(missing_reports, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    f"extension_execution_evidence.missing_reports[{index}] must be a non-empty string"
                )
    errors.extend(
        _validate_customer_acceptance_items_list(
            "extension_execution_evidence.reports",
            payload.get("reports"),
        )
    )
    return errors


def _artifact_optional_string(value: Any) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str):
        value = str(value)
    normalized = value.strip()
    return normalized or None


def _contract_path(root: str | Path, *parts: str) -> str:
    return Path(root).joinpath(*parts).as_posix()


def resolve_customer_external_bindings_config_path(config_path: Any = None) -> str:
    candidate = _artifact_optional_string(config_path)
    if candidate and candidate not in {
        default_placeholder_external_bindings_config_path(),
        default_rehearsal_external_bindings_config_path(),
    }:
        return candidate
    return default_customer_external_bindings_config_path()


def build_customer_external_bindings_config_command(
    *,
    output_path: Any = None,
    instance_artifact_path: Any = None,
) -> str:
    resolved_output = resolve_customer_external_bindings_config_path(output_path)
    resolved_instance = _artifact_optional_string(
        instance_artifact_path
    ) or default_extension_execution_instance_artifact_path()
    return (
        "python tools/build_customer_external_bindings_config.py "
        f"--output {resolved_output} "
        f"--instance-artifact {resolved_instance}"
    )


def build_extension_execution_actuals_command(
    *,
    output_path: Any = None,
    external_bindings_config_path: Any = None,
) -> str:
    resolved_output = _artifact_optional_string(
        output_path
    ) or default_extension_execution_actuals_artifact_path()
    command = (
        "python tools/build_extension_execution_actuals.py "
        f"--output {resolved_output}"
    )
    resolved_external_bindings = _artifact_optional_string(external_bindings_config_path)
    if resolved_external_bindings:
        command += f" --external-bindings-config {resolved_external_bindings}"
    return command


EXTENSION_EXTERNAL_BINDING_SECTION_IDS = (
    "approval_identity",
    "archive_target",
    "due_trigger",
)
EXTENSION_EXTERNAL_BINDING_STATUSES = {
    "missing",
    "placeholder",
    "partial",
    "ready",
}
EXTENSION_EXTERNAL_BINDING_STATES = {"draft", "confirmed"}


def build_confirm_customer_external_bindings_command(
    *,
    config_path: Any = None,
    sections: Sequence[str] | None = None,
) -> str:
    resolved_config = resolve_customer_external_bindings_config_path(config_path)
    normalized_sections = [
        section
        for section in (sections or EXTENSION_EXTERNAL_BINDING_SECTION_IDS)
        if section in EXTENSION_EXTERNAL_BINDING_SECTION_IDS
    ]
    command = (
        "python tools/confirm_customer_external_bindings.py "
        f"--config {resolved_config}"
    )
    for section in normalized_sections:
        command += f" --section {section}"
    command += " --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>"
    return command


def build_customer_external_bindings_confirmation_report_command(
    *,
    output_path: Any = None,
    actuals_artifact_path: Any = None,
) -> str:
    resolved_output = _artifact_optional_string(output_path)
    if resolved_output is None:
        resolved_output = default_customer_external_bindings_confirmation_report_path()
    resolved_actuals = _artifact_optional_string(actuals_artifact_path)
    if resolved_actuals is None:
        resolved_actuals = default_extension_execution_actuals_artifact_path()
    return (
        "python tools/build_customer_external_bindings_confirmation_report.py "
        f"--output {resolved_output} --actuals-artifact {resolved_actuals}"
    )


def build_vulnerability_exception_review_report_command(
    *,
    output_path: Any = None,
    exception_report_path: Any = None,
) -> str:
    resolved_output = _artifact_optional_string(output_path)
    if resolved_output is None:
        resolved_output = default_vulnerability_exception_review_report_path()
    resolved_exception_report = _artifact_optional_string(exception_report_path)
    if resolved_exception_report is None:
        resolved_exception_report = (
            "test_env/release_evidence/security/vulnerability_exception_report.json"
        )
    return (
        "python tools/build_vulnerability_exception_review_report.py "
        f"--output {resolved_output} --exception-report {resolved_exception_report}"
    )


def build_external_mainline_execution_plan_command(
    *,
    output_path: Any = None,
    customer_external_bindings_closure_report_path: Any = None,
    vulnerability_exception_review_report_path: Any = None,
    industrial_delivery_rehearsal_report_path: Any = None,
    customer_config_path: Any = None,
) -> str:
    resolved_output = _artifact_optional_string(output_path)
    if resolved_output is None:
        resolved_output = default_external_mainline_execution_plan_path()
    resolved_closure_report = _artifact_optional_string(
        customer_external_bindings_closure_report_path
    )
    if resolved_closure_report is None:
        resolved_closure_report = default_customer_external_bindings_closure_report_path()
    resolved_review_report = _artifact_optional_string(
        vulnerability_exception_review_report_path
    )
    if resolved_review_report is None:
        resolved_review_report = default_vulnerability_exception_review_report_path()
    resolved_industrial_report = _artifact_optional_string(
        industrial_delivery_rehearsal_report_path
    )
    if resolved_industrial_report is None:
        resolved_industrial_report = (
            default_canonical_industrial_delivery_rehearsal_report_path()
        )
    resolved_customer_config = resolve_customer_external_bindings_config_path(
        customer_config_path
    )
    command = (
        "python tools/build_external_mainline_execution_plan.py "
        f"--output {resolved_output}"
    )
    if (
        resolved_closure_report
        != default_customer_external_bindings_closure_report_path()
    ):
        command += (
            " --customer-external-bindings-closure-report "
            f"{resolved_closure_report}"
        )
    if resolved_review_report != default_vulnerability_exception_review_report_path():
        command += (
            " --vulnerability-exception-review-report "
            f"{resolved_review_report}"
        )
    if (
        resolved_industrial_report
        != default_canonical_industrial_delivery_rehearsal_report_path()
    ):
        command += (
            " --industrial-delivery-rehearsal-report "
            f"{resolved_industrial_report}"
        )
    if resolved_customer_config != default_customer_external_bindings_config_path():
        command += f" --customer-config {resolved_customer_config}"
    return command


def build_external_mainline_inputs_command(
    *,
    output_path: Any = None,
    customer_config_path: Any = None,
    customer_overrides_file_path: Any = None,
    industrial_delivery_rehearsal_report_path: Any = None,
) -> str:
    resolved_output = _artifact_optional_string(output_path)
    if resolved_output is None:
        resolved_output = default_external_mainline_inputs_path()
    resolved_customer_config = resolve_customer_external_bindings_config_path(
        customer_config_path
    )
    resolved_customer_overrides = _artifact_optional_string(
        customer_overrides_file_path
    ) or "deployment/customer_delivery.external_bindings.customer.overrides.json"
    resolved_industrial_report = _artifact_optional_string(
        industrial_delivery_rehearsal_report_path
    )
    if resolved_industrial_report is None:
        resolved_industrial_report = (
            default_canonical_industrial_delivery_rehearsal_report_path()
        )
    command = (
        "python tools/build_external_mainline_inputs.py "
        f"--output {resolved_output}"
    )
    if resolved_customer_config != default_customer_external_bindings_config_path():
        command += f" --customer-config {resolved_customer_config}"
    if (
        resolved_customer_overrides
        != "deployment/customer_delivery.external_bindings.customer.overrides.json"
    ):
        command += f" --customer-overrides-file {resolved_customer_overrides}"
    if (
        resolved_industrial_report
        != default_canonical_industrial_delivery_rehearsal_report_path()
    ):
        command += (
            " --industrial-delivery-rehearsal-report "
            f"{resolved_industrial_report}"
        )
    return command


def build_external_mainline_input_checklist_command(
    *,
    output_path: Any = None,
    inputs_file: Any = None,
    external_mainline_execution_plan_path: Any = None,
) -> str:
    resolved_output = _artifact_optional_string(output_path)
    if resolved_output is None:
        resolved_output = default_external_mainline_input_checklist_report_path()
    resolved_inputs_file = _artifact_optional_string(inputs_file)
    if resolved_inputs_file is None:
        resolved_inputs_file = default_external_mainline_inputs_path()
    resolved_plan_path = _artifact_optional_string(external_mainline_execution_plan_path)
    if resolved_plan_path is None:
        resolved_plan_path = default_external_mainline_execution_plan_path()
    command = (
        "python tools/build_external_mainline_input_checklist.py "
        f"--output {resolved_output}"
    )
    if resolved_inputs_file != default_external_mainline_inputs_path():
        command += f" --inputs-file {resolved_inputs_file}"
    if resolved_plan_path != default_external_mainline_execution_plan_path():
        command += f" --external-mainline-execution-plan {resolved_plan_path}"
    return command


def build_run_external_mainline_execution_plan_command(
    *,
    output_path: Any = None,
    inputs_file: Any = None,
    customer_config_path: Any = None,
    customer_external_bindings_closure_report_path: Any = None,
    vulnerability_exception_review_report_path: Any = None,
    industrial_delivery_rehearsal_report_path: Any = None,
) -> str:
    resolved_output = _artifact_optional_string(output_path)
    if resolved_output is None:
        resolved_output = default_external_mainline_execution_plan_path()
    resolved_inputs_file = _artifact_optional_string(inputs_file)
    if resolved_inputs_file is None:
        resolved_inputs_file = default_external_mainline_inputs_path()
    resolved_customer_config = resolve_customer_external_bindings_config_path(
        customer_config_path
    )
    resolved_closure_report = _artifact_optional_string(
        customer_external_bindings_closure_report_path
    )
    if resolved_closure_report is None:
        resolved_closure_report = default_customer_external_bindings_closure_report_path()
    resolved_review_report = _artifact_optional_string(
        vulnerability_exception_review_report_path
    )
    if resolved_review_report is None:
        resolved_review_report = default_vulnerability_exception_review_report_path()
    resolved_industrial_report = _artifact_optional_string(
        industrial_delivery_rehearsal_report_path
    )
    if resolved_industrial_report is None:
        resolved_industrial_report = (
            default_canonical_industrial_delivery_rehearsal_report_path()
        )
    command = "python tools/run_external_mainline_execution_plan.py"
    if resolved_inputs_file != default_external_mainline_inputs_path():
        command += f" --inputs-file {resolved_inputs_file}"
    command += f" --output {resolved_output}"
    if resolved_customer_config != default_customer_external_bindings_config_path():
        command += f" --customer-config {resolved_customer_config}"
    if (
        resolved_closure_report
        != default_customer_external_bindings_closure_report_path()
    ):
        command += (
            " --customer-external-bindings-closure-report "
            f"{resolved_closure_report}"
        )
    if resolved_review_report != default_vulnerability_exception_review_report_path():
        command += (
            " --vulnerability-exception-review-report "
            f"{resolved_review_report}"
        )
    if (
        resolved_industrial_report
        != default_canonical_industrial_delivery_rehearsal_report_path()
    ):
        command += (
            " --industrial-delivery-rehearsal-report "
            f"{resolved_industrial_report}"
        )
    return command


def build_run_customer_external_bindings_closure_command(
    *,
    config_path: Any = None,
    instance_artifact_path: Any = None,
    schedule_artifact_path: Any = None,
    actuals_artifact_path: Any = None,
    confirmation_report_output_path: Any = None,
    closure_report_output_path: Any = None,
    sections: Sequence[str] | None = None,
    collect_output_root: Any = None,
    skip_collect_release_evidence: bool = False,
) -> str:
    resolved_config = resolve_customer_external_bindings_config_path(config_path)
    resolved_instance = _artifact_optional_string(
        instance_artifact_path
    ) or default_extension_execution_instance_artifact_path()
    resolved_schedule = _artifact_optional_string(
        schedule_artifact_path
    ) or default_extension_execution_schedule_artifact_path()
    resolved_actuals = _artifact_optional_string(
        actuals_artifact_path
    ) or default_extension_execution_actuals_artifact_path()
    resolved_confirmation_report = _artifact_optional_string(
        confirmation_report_output_path
    ) or default_customer_external_bindings_confirmation_report_path()
    resolved_closure_report = _artifact_optional_string(
        closure_report_output_path
    ) or default_customer_external_bindings_closure_report_path()
    resolved_collect_output_root = _artifact_optional_string(collect_output_root)

    normalized_sections = [
        section
        for section in (sections or EXTENSION_EXTERNAL_BINDING_SECTION_IDS)
        if section in EXTENSION_EXTERNAL_BINDING_SECTION_IDS
    ]
    command = (
        "python tools/run_customer_external_bindings_closure.py "
        f"--config {resolved_config}"
    )
    if resolved_instance != default_extension_execution_instance_artifact_path():
        command += f" --instance-artifact {resolved_instance}"
    if resolved_schedule != default_extension_execution_schedule_artifact_path():
        command += f" --schedule-artifact {resolved_schedule}"
    if resolved_actuals != default_extension_execution_actuals_artifact_path():
        command += f" --actuals-output {resolved_actuals}"
    if (
        resolved_confirmation_report
        != default_customer_external_bindings_confirmation_report_path()
    ):
        command += (
            " --confirmation-report-output "
            f"{resolved_confirmation_report}"
        )
    if (
        resolved_closure_report
        != default_customer_external_bindings_closure_report_path()
    ):
        command += (
            " --closure-report-output "
            f"{resolved_closure_report}"
        )
    if resolved_collect_output_root:
        command += f" --collect-output-root {resolved_collect_output_root}"
    if skip_collect_release_evidence:
        command += " --skip-collect-release-evidence"
    for section in normalized_sections:
        command += f" --section {section}"
    command += " --confirmed-by <confirmed-by> --confirmation-ticket <confirmation-ticket>"
    return command


def _external_binding_section_state(payload: Mapping[str, Any]) -> str | None:
    state = _artifact_optional_string(payload.get("binding_state"))
    if state in EXTENSION_EXTERNAL_BINDING_STATES:
        return state
    return None


def _external_binding_confirmation_missing_fields(
    payload: Mapping[str, Any],
) -> list[str]:
    return [
        field
        for field in ("confirmed_by", "confirmed_at", "confirmation_ticket")
        if not _is_non_empty_string(payload.get(field))
    ]


def _external_binding_confirmation_complete(payload: Mapping[str, Any]) -> bool:
    return not _external_binding_confirmation_missing_fields(payload)


def _latest_confirmed_at(values: Sequence[str]) -> str | None:
    latest_value: str | None = None
    latest_datetime: datetime | None = None
    for raw in values:
        value = _artifact_optional_string(raw)
        if value is None:
            continue
        normalized = value.replace("Z", "+00:00")
        try:
            parsed = datetime.fromisoformat(normalized)
        except ValueError:
            continue
        if latest_datetime is None or parsed > latest_datetime:
            latest_datetime = parsed
            latest_value = value
    return latest_value


def _normalize_extension_external_bindings(bindings: Any) -> dict[str, Any] | None:
    if not isinstance(bindings, Mapping):
        return None

    normalized: dict[str, Any] = {}
    config_path = _artifact_optional_string(bindings.get("config_path"))
    if _is_non_empty_string(config_path):
        normalized["config_path"] = config_path

    for field in ("approval_identity", "archive_target", "due_trigger"):
        value = bindings.get(field)
        if not isinstance(value, Mapping):
            continue
        normalized[field] = {
            str(key): to_jsonable(item)
            for key, item in value.items()
            if _is_non_empty_string(key) and item is not None
        }

    return normalized or None


def _external_binding_section_has_ready_fields(
    section_id: str, payload: Mapping[str, Any]
) -> bool:
    if section_id == "approval_identity":
        return any(
            _is_non_empty_string(payload.get(field)) for field in ("source_path", "reference")
        )
    return _is_non_empty_string(payload.get("binding_reference_base"))


def _external_binding_section_ready(section_id: str, payload: Mapping[str, Any]) -> bool:
    binding_state = _external_binding_section_state(payload)
    if binding_state != "confirmed":
        return False
    if not _external_binding_confirmation_complete(payload):
        return False
    return _external_binding_section_has_ready_fields(section_id, payload)


def _build_extension_external_bindings_details(
    bindings: Any,
) -> dict[str, Any]:
    normalized = _normalize_extension_external_bindings(bindings)
    config_path = (
        _artifact_optional_string(normalized.get("config_path"))
        if isinstance(normalized, Mapping)
        else None
    )
    declared_sections: list[str] = []
    ready_sections: list[str] = []
    placeholder_sections: list[str] = []
    missing_sections: list[str] = []
    draft_sections: list[str] = []
    unconfirmed_sections: list[str] = []
    confirmed_sections: list[str] = []
    confirmation_missing_sections: list[str] = []
    confirmed_by: list[str] = []
    confirmation_tickets: list[str] = []
    confirmed_at_values: list[str] = []

    for section_id in EXTENSION_EXTERNAL_BINDING_SECTION_IDS:
        section_payload = (
            normalized.get(section_id) if isinstance(normalized, Mapping) else None
        )
        if not isinstance(section_payload, Mapping):
            missing_sections.append(section_id)
            continue
        declared_sections.append(section_id)
        binding_state = _external_binding_section_state(section_payload)
        has_ready_fields = _external_binding_section_has_ready_fields(
            section_id, section_payload
        )
        if binding_state == "draft":
            draft_sections.append(section_id)
        elif binding_state == "confirmed":
            if _external_binding_confirmation_complete(section_payload):
                confirmed_sections.append(section_id)
                confirmed_by_value = _artifact_optional_string(section_payload.get("confirmed_by"))
                if confirmed_by_value and confirmed_by_value not in confirmed_by:
                    confirmed_by.append(confirmed_by_value)
                confirmation_ticket_value = _artifact_optional_string(
                    section_payload.get("confirmation_ticket")
                )
                if (
                    confirmation_ticket_value
                    and confirmation_ticket_value not in confirmation_tickets
                ):
                    confirmation_tickets.append(confirmation_ticket_value)
                confirmed_at_value = _artifact_optional_string(section_payload.get("confirmed_at"))
                if confirmed_at_value:
                    confirmed_at_values.append(confirmed_at_value)
            else:
                confirmation_missing_sections.append(section_id)
        elif has_ready_fields:
            unconfirmed_sections.append(section_id)
        if _external_binding_section_ready(section_id, section_payload):
            ready_sections.append(section_id)
        elif not has_ready_fields:
            placeholder_sections.append(section_id)

    last_confirmed_at = _latest_confirmed_at(confirmed_at_values)
    confirmation_summary = ""
    if confirmed_sections:
        confirmation_summary += " Confirmed sections: " + ", ".join(confirmed_sections) + "."
    if confirmation_missing_sections:
        confirmation_summary += (
            " Confirmation metadata missing: "
            + ", ".join(confirmation_missing_sections)
            + "."
        )
    if unconfirmed_sections:
        confirmation_summary += (
            " Unconfirmed sections: " + ", ".join(unconfirmed_sections) + "."
        )
    if confirmed_by:
        confirmation_summary += " Confirmed by: " + ", ".join(confirmed_by) + "."
    if confirmation_tickets:
        confirmation_summary += (
            " Confirmation tickets: " + ", ".join(confirmation_tickets) + "."
        )
    if last_confirmed_at:
        confirmation_summary += f" Last confirmed at: {last_confirmed_at}."

    if normalized is None:
        status = "missing"
    elif len(ready_sections) == len(EXTENSION_EXTERNAL_BINDING_SECTION_IDS):
        status = "ready"
    elif len(ready_sections) == 0 and not missing_sections:
        status = "placeholder"
    else:
        status = "partial"

    follow_up_required = status in {"placeholder", "partial"}
    config_summary = (
        f" Config: {config_path}."
        if _is_non_empty_string(config_path)
        else ""
    )
    if status == "ready":
        summary = (
            "External bindings ready: "
            f"{len(ready_sections)}/{len(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)} sections mapped to customer-owned systems."
            + config_summary
            + confirmation_summary
        )
    elif status == "placeholder":
        summary = (
            "External bindings placeholder: "
            f"{len(ready_sections)}/{len(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)} sections mapped to customer-owned systems."
            + (
                " Replace placeholder sections: "
                + ", ".join(placeholder_sections)
                + "."
                if placeholder_sections
                else ""
            )
            + (
                " Draft sections: " + ", ".join(draft_sections) + "."
                if draft_sections
                else ""
            )
            + config_summary
            + confirmation_summary
        )
    elif status == "partial":
        summary = (
            "External bindings partial: "
            f"{len(ready_sections)}/{len(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)} sections mapped to customer-owned systems."
            + (
                " Placeholder sections: "
                + ", ".join(placeholder_sections)
                + "."
                if placeholder_sections
                else ""
            )
            + (
                " Draft sections: " + ", ".join(draft_sections) + "."
                if draft_sections
                else ""
            )
            + (
                " Missing sections: " + ", ".join(missing_sections) + "."
                if missing_sections
                else ""
            )
            + config_summary
            + confirmation_summary
        )
    else:
        summary = "External bindings missing: no managed customer binding config attached."

    return {
        "external_bindings_status": status,
        "external_bindings_summary": summary,
        "external_bindings_follow_up_required": follow_up_required,
        "external_bindings_declared_count": len(declared_sections),
        "external_bindings_ready_count": len(ready_sections),
        "external_bindings_placeholder_count": len(placeholder_sections),
        "external_bindings_confirmed_count": len(confirmed_sections),
        "external_bindings_missing_sections": missing_sections,
        "external_bindings_placeholder_sections": placeholder_sections,
        "external_bindings_draft_sections": draft_sections,
        "external_bindings_unconfirmed_sections": unconfirmed_sections,
        "external_bindings_confirmed_sections": confirmed_sections,
        "external_bindings_confirmation_missing_sections": confirmation_missing_sections,
        "external_bindings_confirmed_by": confirmed_by,
        "external_bindings_confirmation_tickets": confirmation_tickets,
        "external_bindings_last_confirmed_at": last_confirmed_at,
    }


def _build_extension_execution_instance_payload(
    *,
    extension_support_surface: Mapping[str, Any] | None,
    artifact_path: str,
    exists: bool,
    instance: Mapping[str, Any] | None,
) -> dict[str, Any]:
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else {}
    )
    plan = build_extension_execution_plan(surface)
    instance_payload = dict(instance) if isinstance(instance, Mapping) else {}
    profile_items = instance_payload.get("profiles")
    instance_profiles = (
        [dict(item) for item in profile_items if isinstance(item, Mapping)]
        if isinstance(profile_items, Sequence) and not isinstance(profile_items, (str, bytes))
        else []
    )
    profiles_by_id = {
        str(item.get("id")): item
        for item in instance_profiles
        if _is_non_empty_string(item.get("id"))
    }
    hydrated_profiles: list[dict[str, Any]] = []
    actionable_profiles = 0
    ready_profiles = 0
    missing_profiles: list[str] = []
    for item in plan.get("profiles", []):
        if not isinstance(item, Mapping):
            continue
        profile_id = str(item.get("id") or "")
        profile_instance = profiles_by_id.get(profile_id, {})
        actionable = item.get("actionable") is True
        declared = item.get("declared") is True
        profile_payload = {
            "id": profile_id,
            "label": str(item.get("label") or ""),
            "declared": declared,
            "actionable": actionable,
            "handoff_record_path": _artifact_optional_string(
                profile_instance.get("handoff_record_path")
            ),
            "watch_log_path": _artifact_optional_string(
                profile_instance.get("watch_log_path")
            ),
            "exception_review_record_path": _artifact_optional_string(
                profile_instance.get("exception_review_record_path")
            ),
            "escalation_closure_record_path": _artifact_optional_string(
                profile_instance.get("escalation_closure_record_path")
            ),
            "closure_archive_root": _artifact_optional_string(
                profile_instance.get("closure_archive_root")
            ),
            "closure_index_path": _artifact_optional_string(
                profile_instance.get("closure_index_path")
            ),
        }
        if actionable:
            actionable_profiles += 1
            if all(
                _is_non_empty_string(profile_payload[field])
                for field in [
                    "handoff_record_path",
                    "watch_log_path",
                    "exception_review_record_path",
                    "escalation_closure_record_path",
                    "closure_archive_root",
                    "closure_index_path",
                ]
            ):
                ready_profiles += 1
            else:
                missing_profiles.append(profile_id)
        hydrated_profiles.append(profile_payload)

    metadata_fields = {
        "engagement_id": _artifact_optional_string(instance_payload.get("engagement_id")),
        "customer_name": _artifact_optional_string(instance_payload.get("customer_name")),
        "site_name": _artifact_optional_string(instance_payload.get("site_name")),
        "change_ticket": _artifact_optional_string(instance_payload.get("change_ticket")),
        "window_id": _artifact_optional_string(instance_payload.get("window_id")),
        "window_start_at": _artifact_optional_string(instance_payload.get("window_start_at")),
        "window_end_at": _artifact_optional_string(instance_payload.get("window_end_at")),
        "delivery_root": _artifact_optional_string(instance_payload.get("delivery_root")),
        "closure_archive_root": _artifact_optional_string(
            instance_payload.get("closure_archive_root")
        ),
        "exception_review_due_at": _artifact_optional_string(
            instance_payload.get("exception_review_due_at")
        ),
    }
    missing_metadata = [
        name for name, value in metadata_fields.items() if not _is_non_empty_string(value)
    ]
    missing_profiles = list(dict.fromkeys(missing_profiles))
    declared_profiles = _coerce_non_negative_int(plan.get("declared_profiles")) or 0
    generated_at = _artifact_optional_string(instance_payload.get("generated_at"))
    status = (
        "ready"
        if exists
        and plan.get("status") == "ready"
        and not missing_metadata
        and ready_profiles == actionable_profiles
        else "blocked"
    )
    summary = (
        f"Extension execution instance {status}: "
        f"{ready_profiles}/{actionable_profiles} actionable profiles materialized, "
        f"engagement={metadata_fields['engagement_id'] or 'missing'}, "
        f"window={metadata_fields['window_id'] or 'missing'}."
    )
    if missing_metadata:
        summary += " Missing metadata: " + ", ".join(missing_metadata) + "."
    if missing_profiles:
        summary += " Profiles missing concrete paths: " + ", ".join(missing_profiles) + "."

    return {
        "schema_version": EXTENSION_EXECUTION_INSTANCE_VERSION,
        "artifact_type": EXTENSION_EXECUTION_INSTANCE_ARTIFACT_TYPE,
        "generated_at": generated_at,
        "artifact_path": artifact_path,
        "exists": exists,
        "status": status,
        "summary": summary,
        **metadata_fields,
        "declared_profiles": declared_profiles,
        "actionable_profiles": actionable_profiles,
        "ready_profiles": ready_profiles,
        "missing_profiles": missing_profiles,
        "profiles": to_jsonable(hydrated_profiles),
    }


def build_extension_execution_instance_artifact(
    *,
    project_root: str | Path | None = None,
    extension_support_surface: Mapping[str, Any] | None = None,
    artifact_path: str | Path | None = None,
    generated_at: str | None = None,
    engagement_id: str,
    customer_name: str,
    site_name: str,
    change_ticket: str,
    window_id: str,
    window_start_at: str,
    window_end_at: str,
    delivery_root: str,
    closure_archive_root: str,
    exception_review_due_at: str,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else dict(
            build_customer_delivery_surface(project_root=resolved_root).get(
                "extension_support_surface",
                {},
            )
        )
    )
    plan = build_extension_execution_plan(surface)
    plan_profiles = plan.get("profiles", [])
    instance_profiles: list[dict[str, Any]] = []
    for item in plan_profiles:
        if not isinstance(item, Mapping):
            continue
        profile_id = str(item.get("id") or "")
        profile_root = _contract_path(delivery_root, "profiles", profile_id)
        profile_archive_root = _contract_path(closure_archive_root, profile_id)
        instance_profiles.append(
            {
                "id": profile_id,
                "handoff_record_path": _contract_path(profile_root, "handoff.json"),
                "watch_log_path": _contract_path(profile_root, "watch.log"),
                "exception_review_record_path": _contract_path(
                    profile_root,
                    "exception_review.json",
                ),
                "escalation_closure_record_path": _contract_path(
                    profile_root,
                    "escalation_closure.json",
                ),
                "closure_archive_root": profile_archive_root,
                "closure_index_path": _contract_path(
                    profile_archive_root,
                    "index.json",
                ),
            }
        )
    payload = {
        "generated_at": generated_at or datetime.now().isoformat(),
        "engagement_id": engagement_id,
        "customer_name": customer_name,
        "site_name": site_name,
        "change_ticket": change_ticket,
        "window_id": window_id,
        "window_start_at": window_start_at,
        "window_end_at": window_end_at,
        "delivery_root": Path(delivery_root).as_posix(),
        "closure_archive_root": Path(closure_archive_root).as_posix(),
        "exception_review_due_at": exception_review_due_at,
        "profiles": instance_profiles,
    }
    return _build_extension_execution_instance_payload(
        extension_support_surface=surface,
        artifact_path=(
            Path(artifact_path).as_posix()
            if artifact_path is not None
            else default_extension_execution_instance_artifact_path()
        ),
        exists=True,
        instance=payload,
    )


def build_extension_execution_instance(
    *,
    project_root: str | Path | None = None,
    extension_support_surface: Mapping[str, Any] | None = None,
    artifact_path: str | Path | None = None,
    instance_artifact: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else dict(
            build_customer_delivery_surface(project_root=resolved_root).get(
                "extension_support_surface",
                {},
            )
        )
    )
    artifact_reference = (
        Path(artifact_path).as_posix()
        if artifact_path is not None
        else default_extension_execution_instance_artifact_path()
    )
    payload = dict(instance_artifact) if isinstance(instance_artifact, Mapping) else None
    exists = False
    if payload is not None:
        exists = payload.get("exists") is True
        artifact_reference = str(payload.get("artifact_path") or artifact_reference)
    else:
        resolved_artifact_path = _resolve_release_artifact_path(
            artifact_reference,
            resolved_root,
        )
        if resolved_artifact_path.is_file():
            exists = True
            try:
                loaded = json.loads(resolved_artifact_path.read_text(encoding="utf-8"))
                payload = dict(loaded) if isinstance(loaded, Mapping) else {}
            except Exception:
                payload = {}

    return _build_extension_execution_instance_payload(
        extension_support_surface=surface,
        artifact_path=artifact_reference,
        exists=exists,
        instance=payload,
    )


def validate_extension_execution_instance(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["extension_execution_instance must be an object"]

    errors: list[str] = []
    missing = sorted(EXTENSION_EXECUTION_INSTANCE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "extension_execution_instance missing required fields: " + ", ".join(missing)
        )
    if payload.get("schema_version") != EXTENSION_EXECUTION_INSTANCE_VERSION:
        errors.append(
            "extension_execution_instance.schema_version must be "
            f"{EXTENSION_EXECUTION_INSTANCE_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != EXTENSION_EXECUTION_INSTANCE_ARTIFACT_TYPE:
        errors.append(
            "extension_execution_instance.artifact_type must be "
            f"{EXTENSION_EXECUTION_INSTANCE_ARTIFACT_TYPE!r}"
        )
    if "exists" in payload and not isinstance(payload.get("exists"), bool):
        errors.append("extension_execution_instance.exists must be a boolean")
    if payload.get("status") not in EXTENSION_EXECUTION_INSTANCE_STATUSES:
        errors.append(
            "extension_execution_instance.status must be one of "
            f"{sorted(EXTENSION_EXECUTION_INSTANCE_STATUSES)}"
        )
    for field in [
        "generated_at",
        "artifact_path",
        "summary",
        "engagement_id",
        "customer_name",
        "site_name",
        "change_ticket",
        "window_id",
        "window_start_at",
        "window_end_at",
        "delivery_root",
        "closure_archive_root",
        "exception_review_due_at",
    ]:
        if field in payload and payload.get(field) is not None and not _is_non_empty_string(
            payload.get(field)
        ):
            errors.append(
                f"extension_execution_instance.{field} must be null or a non-empty string"
            )
    for field in ["declared_profiles", "actionable_profiles", "ready_profiles"]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(
                f"extension_execution_instance.{field} must be a non-negative integer"
            )
    if (
        _is_non_negative_int(payload.get("actionable_profiles"))
        and _is_non_negative_int(payload.get("ready_profiles"))
        and payload.get("ready_profiles") > payload.get("actionable_profiles")
    ):
        errors.append(
            "extension_execution_instance.ready_profiles must be <= actionable_profiles"
        )
    missing_profiles = payload.get("missing_profiles")
    if not isinstance(missing_profiles, list):
        errors.append("extension_execution_instance.missing_profiles must be a list")
    else:
        for index, item in enumerate(missing_profiles, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    "extension_execution_instance.missing_profiles"
                    f"[{index}] must be a non-empty string"
                )
    profiles = payload.get("profiles")
    if not isinstance(profiles, list):
        errors.append("extension_execution_instance.profiles must be a list")
        return errors
    for index, item in enumerate(profiles, start=1):
        prefix = f"extension_execution_instance.profiles[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        missing_fields = sorted(
            EXTENSION_EXECUTION_INSTANCE_PROFILE_REQUIRED_FIELDS - set(item)
        )
        if missing_fields:
            errors.append(f"{prefix} missing required fields: {', '.join(missing_fields)}")
        for field in ["id", "label"]:
            if field in item and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        for field in ["declared", "actionable"]:
            if field in item and not isinstance(item.get(field), bool):
                errors.append(f"{prefix}.{field} must be a boolean")
        for field in [
            "handoff_record_path",
            "watch_log_path",
            "exception_review_record_path",
            "escalation_closure_record_path",
            "closure_archive_root",
            "closure_index_path",
        ]:
            if field in item and item.get(field) is not None and not _is_non_empty_string(
                item.get(field)
            ):
                errors.append(f"{prefix}.{field} must be null or a non-empty string")
    if payload.get("status") == "ready":
        for field in [
            "engagement_id",
            "customer_name",
            "site_name",
            "change_ticket",
            "window_id",
            "window_start_at",
            "window_end_at",
            "delivery_root",
            "closure_archive_root",
            "exception_review_due_at",
        ]:
            if not _is_non_empty_string(payload.get(field)):
                errors.append(
                    f"extension_execution_instance.{field} is required when status is 'ready'"
                )
        if payload.get("exists") is not True:
            errors.append("extension_execution_instance.exists must be true when status is 'ready'")
        if payload.get("missing_profiles") != []:
            errors.append(
                "extension_execution_instance.missing_profiles must be empty when status is 'ready'"
            )
    return errors


def write_extension_execution_instance_artifact(
    payload: Mapping[str, Any],
    path: str | Path,
) -> Path:
    errors = validate_extension_execution_instance(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _build_extension_execution_schedule_payload(
    *,
    extension_support_surface: Mapping[str, Any] | None,
    artifact_path: str,
    exists: bool,
    schedule: Mapping[str, Any] | None,
) -> dict[str, Any]:
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else {}
    )
    plan = build_extension_execution_plan(surface)
    schedule_payload = dict(schedule) if isinstance(schedule, Mapping) else {}
    profile_items = schedule_payload.get("profiles")
    schedule_profiles = (
        [dict(item) for item in profile_items if isinstance(item, Mapping)]
        if isinstance(profile_items, Sequence) and not isinstance(profile_items, (str, bytes))
        else []
    )
    profiles_by_id = {
        str(item.get("id")): item
        for item in schedule_profiles
        if _is_non_empty_string(item.get("id"))
    }
    hydrated_profiles: list[dict[str, Any]] = []
    actionable_profiles = 0
    ready_profiles = 0
    missing_profiles: list[str] = []
    for item in plan.get("profiles", []):
        if not isinstance(item, Mapping):
            continue
        profile_id = str(item.get("id") or "")
        profile_schedule = profiles_by_id.get(profile_id, {})
        actionable = item.get("actionable") is True
        declared = item.get("declared") is True
        profile_payload = {
            "id": profile_id,
            "label": str(item.get("label") or ""),
            "declared": declared,
            "actionable": actionable,
            "window_trigger_at": _artifact_optional_string(
                profile_schedule.get("window_trigger_at")
            ),
            "signoff_due_at": _artifact_optional_string(
                profile_schedule.get("signoff_due_at")
            ),
            "exception_review_due_at": _artifact_optional_string(
                profile_schedule.get("exception_review_due_at")
            ),
            "closure_archive_due_at": _artifact_optional_string(
                profile_schedule.get("closure_archive_due_at")
            ),
            "window_trigger_record_path": _artifact_optional_string(
                profile_schedule.get("window_trigger_record_path")
            ),
            "handoff_record_path": _artifact_optional_string(
                profile_schedule.get("handoff_record_path")
            ),
            "watch_log_path": _artifact_optional_string(
                profile_schedule.get("watch_log_path")
            ),
            "signoff_record_path": _artifact_optional_string(
                profile_schedule.get("signoff_record_path")
            ),
            "exception_review_record_path": _artifact_optional_string(
                profile_schedule.get("exception_review_record_path")
            ),
            "residual_risk_review_record_path": _artifact_optional_string(
                profile_schedule.get("residual_risk_review_record_path")
            ),
            "escalation_closure_record_path": _artifact_optional_string(
                profile_schedule.get("escalation_closure_record_path")
            ),
            "closure_archive_root": _artifact_optional_string(
                profile_schedule.get("closure_archive_root")
            ),
            "closure_index_path": _artifact_optional_string(
                profile_schedule.get("closure_index_path")
            ),
            "closure_manifest_path": _artifact_optional_string(
                profile_schedule.get("closure_manifest_path")
            ),
        }
        if actionable:
            actionable_profiles += 1
            if all(
                _is_non_empty_string(profile_payload[field])
                for field in [
                    "window_trigger_at",
                    "signoff_due_at",
                    "exception_review_due_at",
                    "closure_archive_due_at",
                    "window_trigger_record_path",
                    "handoff_record_path",
                    "watch_log_path",
                    "signoff_record_path",
                    "exception_review_record_path",
                    "residual_risk_review_record_path",
                    "escalation_closure_record_path",
                    "closure_archive_root",
                    "closure_index_path",
                    "closure_manifest_path",
                ]
            ):
                ready_profiles += 1
            else:
                missing_profiles.append(profile_id)
        hydrated_profiles.append(profile_payload)

    metadata_fields = {
        "engagement_id": _artifact_optional_string(schedule_payload.get("engagement_id")),
        "customer_name": _artifact_optional_string(schedule_payload.get("customer_name")),
        "site_name": _artifact_optional_string(schedule_payload.get("site_name")),
        "change_ticket": _artifact_optional_string(schedule_payload.get("change_ticket")),
        "window_id": _artifact_optional_string(schedule_payload.get("window_id")),
        "window_start_at": _artifact_optional_string(
            schedule_payload.get("window_start_at")
        ),
        "window_end_at": _artifact_optional_string(schedule_payload.get("window_end_at")),
        "window_trigger_at": _artifact_optional_string(
            schedule_payload.get("window_trigger_at")
        ),
        "signoff_due_at": _artifact_optional_string(
            schedule_payload.get("signoff_due_at")
        ),
        "exception_review_due_at": _artifact_optional_string(
            schedule_payload.get("exception_review_due_at")
        ),
        "closure_archive_due_at": _artifact_optional_string(
            schedule_payload.get("closure_archive_due_at")
        ),
        "delivery_root": _artifact_optional_string(schedule_payload.get("delivery_root")),
        "closure_archive_root": _artifact_optional_string(
            schedule_payload.get("closure_archive_root")
        ),
    }
    missing_metadata = [
        name for name, value in metadata_fields.items() if not _is_non_empty_string(value)
    ]
    missing_profiles = list(dict.fromkeys(missing_profiles))
    declared_profiles = _coerce_non_negative_int(plan.get("declared_profiles")) or 0
    generated_at = _artifact_optional_string(schedule_payload.get("generated_at"))
    status = (
        "ready"
        if exists
        and plan.get("status") == "ready"
        and not missing_metadata
        and ready_profiles == actionable_profiles
        else "blocked"
    )
    summary = (
        f"Extension execution schedule {status}: "
        f"{ready_profiles}/{actionable_profiles} actionable profiles scheduled, "
        f"window_trigger={metadata_fields['window_trigger_at'] or 'missing'}, "
        f"closure_due={metadata_fields['closure_archive_due_at'] or 'missing'}."
    )
    if missing_metadata:
        summary += " Missing metadata: " + ", ".join(missing_metadata) + "."
    if missing_profiles:
        summary += " Profiles missing scheduled checkpoints: " + ", ".join(missing_profiles) + "."

    return {
        "schema_version": EXTENSION_EXECUTION_SCHEDULE_VERSION,
        "artifact_type": EXTENSION_EXECUTION_SCHEDULE_ARTIFACT_TYPE,
        "generated_at": generated_at,
        "artifact_path": artifact_path,
        "exists": exists,
        "status": status,
        "summary": summary,
        **metadata_fields,
        "declared_profiles": declared_profiles,
        "actionable_profiles": actionable_profiles,
        "ready_profiles": ready_profiles,
        "missing_profiles": missing_profiles,
        "profiles": to_jsonable(hydrated_profiles),
    }


def build_extension_execution_schedule_artifact(
    *,
    project_root: str | Path | None = None,
    extension_support_surface: Mapping[str, Any] | None = None,
    artifact_path: str | Path | None = None,
    generated_at: str | None = None,
    instance_artifact: Mapping[str, Any] | None = None,
    instance_artifact_path: str | Path | None = None,
    window_trigger_at: str | None = None,
    signoff_due_at: str | None = None,
    closure_archive_due_at: str | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else dict(
            build_customer_delivery_surface(project_root=resolved_root).get(
                "extension_support_surface",
                {},
            )
        )
    )
    instance = build_extension_execution_instance(
        project_root=resolved_root,
        extension_support_surface=surface,
        artifact_path=instance_artifact_path,
        instance_artifact=instance_artifact,
    )
    instance_profiles = instance.get("profiles", [])
    schedule_profiles: list[dict[str, Any]] = []
    default_window_trigger_at = (
        window_trigger_at
        or _artifact_optional_string(instance.get("window_start_at"))
        or _artifact_optional_string(instance.get("window_trigger_at"))
    )
    default_signoff_due_at = (
        signoff_due_at or _artifact_optional_string(instance.get("window_end_at"))
    )
    default_closure_archive_due_at = (
        closure_archive_due_at or _artifact_optional_string(instance.get("window_end_at"))
    )
    for item in instance_profiles:
        if not isinstance(item, Mapping):
            continue
        profile_id = str(item.get("id") or "")
        handoff_record_path = _artifact_optional_string(item.get("handoff_record_path"))
        watch_log_path = _artifact_optional_string(item.get("watch_log_path"))
        exception_review_record_path = _artifact_optional_string(
            item.get("exception_review_record_path")
        )
        escalation_closure_record_path = _artifact_optional_string(
            item.get("escalation_closure_record_path")
        )
        closure_archive_root = _artifact_optional_string(item.get("closure_archive_root"))
        closure_index_path = _artifact_optional_string(item.get("closure_index_path"))
        profile_root = (
            Path(handoff_record_path).parent.as_posix()
            if _is_non_empty_string(handoff_record_path)
            else _contract_path(
                _artifact_optional_string(instance.get("delivery_root")) or "",
                "profiles",
                profile_id,
            )
        )
        profile_archive_root = (
            closure_archive_root
            if _is_non_empty_string(closure_archive_root)
            else _contract_path(
                _artifact_optional_string(instance.get("closure_archive_root")) or "",
                profile_id,
            )
        )
        schedule_profiles.append(
            {
                "id": profile_id,
                "window_trigger_at": default_window_trigger_at,
                "signoff_due_at": default_signoff_due_at,
                "exception_review_due_at": _artifact_optional_string(
                    instance.get("exception_review_due_at")
                ),
                "closure_archive_due_at": default_closure_archive_due_at,
                "window_trigger_record_path": _contract_path(
                    profile_root,
                    "window_trigger.json",
                ),
                "handoff_record_path": handoff_record_path,
                "watch_log_path": watch_log_path,
                "signoff_record_path": _contract_path(profile_root, "signoff.json"),
                "exception_review_record_path": exception_review_record_path,
                "residual_risk_review_record_path": _contract_path(
                    profile_root,
                    "residual_risk_review.json",
                ),
                "escalation_closure_record_path": escalation_closure_record_path,
                "closure_archive_root": profile_archive_root,
                "closure_index_path": closure_index_path,
                "closure_manifest_path": _contract_path(
                    profile_archive_root,
                    "closure_manifest.json",
                ),
            }
        )
    payload = {
        "generated_at": generated_at or datetime.now().isoformat(),
        "engagement_id": instance.get("engagement_id"),
        "customer_name": instance.get("customer_name"),
        "site_name": instance.get("site_name"),
        "change_ticket": instance.get("change_ticket"),
        "window_id": instance.get("window_id"),
        "window_start_at": instance.get("window_start_at"),
        "window_end_at": instance.get("window_end_at"),
        "window_trigger_at": default_window_trigger_at,
        "signoff_due_at": default_signoff_due_at,
        "exception_review_due_at": instance.get("exception_review_due_at"),
        "closure_archive_due_at": default_closure_archive_due_at,
        "delivery_root": instance.get("delivery_root"),
        "closure_archive_root": instance.get("closure_archive_root"),
        "profiles": schedule_profiles,
    }
    return _build_extension_execution_schedule_payload(
        extension_support_surface=surface,
        artifact_path=(
            Path(artifact_path).as_posix()
            if artifact_path is not None
            else default_extension_execution_schedule_artifact_path()
        ),
        exists=True,
        schedule=payload,
    )


def build_extension_execution_schedule(
    *,
    project_root: str | Path | None = None,
    extension_support_surface: Mapping[str, Any] | None = None,
    artifact_path: str | Path | None = None,
    schedule_artifact: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else dict(
            build_customer_delivery_surface(project_root=resolved_root).get(
                "extension_support_surface",
                {},
            )
        )
    )
    artifact_reference = (
        Path(artifact_path).as_posix()
        if artifact_path is not None
        else default_extension_execution_schedule_artifact_path()
    )
    payload = dict(schedule_artifact) if isinstance(schedule_artifact, Mapping) else None
    exists = False
    if payload is not None:
        exists = payload.get("exists") is True
        artifact_reference = str(payload.get("artifact_path") or artifact_reference)
    else:
        resolved_artifact_path = _resolve_release_artifact_path(
            artifact_reference,
            resolved_root,
        )
        if resolved_artifact_path.is_file():
            exists = True
            try:
                loaded = json.loads(resolved_artifact_path.read_text(encoding="utf-8"))
                payload = dict(loaded) if isinstance(loaded, Mapping) else {}
            except Exception:
                payload = {}

    return _build_extension_execution_schedule_payload(
        extension_support_surface=surface,
        artifact_path=artifact_reference,
        exists=exists,
        schedule=payload,
    )


def validate_extension_execution_schedule(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["extension_execution_schedule must be an object"]

    errors: list[str] = []
    missing = sorted(EXTENSION_EXECUTION_SCHEDULE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "extension_execution_schedule missing required fields: " + ", ".join(missing)
        )
    if payload.get("schema_version") != EXTENSION_EXECUTION_SCHEDULE_VERSION:
        errors.append(
            "extension_execution_schedule.schema_version must be "
            f"{EXTENSION_EXECUTION_SCHEDULE_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != EXTENSION_EXECUTION_SCHEDULE_ARTIFACT_TYPE:
        errors.append(
            "extension_execution_schedule.artifact_type must be "
            f"{EXTENSION_EXECUTION_SCHEDULE_ARTIFACT_TYPE!r}"
        )
    if "exists" in payload and not isinstance(payload.get("exists"), bool):
        errors.append("extension_execution_schedule.exists must be a boolean")
    if payload.get("status") not in EXTENSION_EXECUTION_SCHEDULE_STATUSES:
        errors.append(
            "extension_execution_schedule.status must be one of "
            f"{sorted(EXTENSION_EXECUTION_SCHEDULE_STATUSES)}"
        )
    for field in [
        "generated_at",
        "artifact_path",
        "summary",
        "engagement_id",
        "customer_name",
        "site_name",
        "change_ticket",
        "window_id",
        "window_start_at",
        "window_end_at",
        "window_trigger_at",
        "signoff_due_at",
        "exception_review_due_at",
        "closure_archive_due_at",
        "delivery_root",
        "closure_archive_root",
    ]:
        if field in payload and payload.get(field) is not None and not _is_non_empty_string(
            payload.get(field)
        ):
            errors.append(
                f"extension_execution_schedule.{field} must be null or a non-empty string"
            )
    for field in ["declared_profiles", "actionable_profiles", "ready_profiles"]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(
                f"extension_execution_schedule.{field} must be a non-negative integer"
            )
    if (
        _is_non_negative_int(payload.get("actionable_profiles"))
        and _is_non_negative_int(payload.get("ready_profiles"))
        and payload.get("ready_profiles") > payload.get("actionable_profiles")
    ):
        errors.append(
            "extension_execution_schedule.ready_profiles must be <= actionable_profiles"
        )
    missing_profiles = payload.get("missing_profiles")
    if not isinstance(missing_profiles, list):
        errors.append("extension_execution_schedule.missing_profiles must be a list")
    else:
        for index, item in enumerate(missing_profiles, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    "extension_execution_schedule.missing_profiles"
                    f"[{index}] must be a non-empty string"
                )
    profiles = payload.get("profiles")
    if not isinstance(profiles, list):
        errors.append("extension_execution_schedule.profiles must be a list")
        return errors
    for index, item in enumerate(profiles, start=1):
        prefix = f"extension_execution_schedule.profiles[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        missing_fields = sorted(
            EXTENSION_EXECUTION_SCHEDULE_PROFILE_REQUIRED_FIELDS - set(item)
        )
        if missing_fields:
            errors.append(f"{prefix} missing required fields: {', '.join(missing_fields)}")
        for field in ["id", "label"]:
            if field in item and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        for field in ["declared", "actionable"]:
            if field in item and not isinstance(item.get(field), bool):
                errors.append(f"{prefix}.{field} must be a boolean")
        for field in [
            "window_trigger_at",
            "signoff_due_at",
            "exception_review_due_at",
            "closure_archive_due_at",
            "window_trigger_record_path",
            "handoff_record_path",
            "watch_log_path",
            "signoff_record_path",
            "exception_review_record_path",
            "residual_risk_review_record_path",
            "escalation_closure_record_path",
            "closure_archive_root",
            "closure_index_path",
            "closure_manifest_path",
        ]:
            if field in item and item.get(field) is not None and not _is_non_empty_string(
                item.get(field)
            ):
                errors.append(f"{prefix}.{field} must be null or a non-empty string")
    if payload.get("status") == "ready":
        for field in [
            "engagement_id",
            "customer_name",
            "site_name",
            "change_ticket",
            "window_id",
            "window_start_at",
            "window_end_at",
            "window_trigger_at",
            "signoff_due_at",
            "exception_review_due_at",
            "closure_archive_due_at",
            "delivery_root",
            "closure_archive_root",
        ]:
            if not _is_non_empty_string(payload.get(field)):
                errors.append(
                    f"extension_execution_schedule.{field} is required when status is 'ready'"
                )
        if payload.get("exists") is not True:
            errors.append("extension_execution_schedule.exists must be true when status is 'ready'")
        if payload.get("missing_profiles") != []:
            errors.append(
                "extension_execution_schedule.missing_profiles must be empty when status is 'ready'"
            )
    return errors


def write_extension_execution_schedule_artifact(
    payload: Mapping[str, Any],
    path: str | Path,
) -> Path:
    errors = validate_extension_execution_schedule(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _build_extension_execution_actuals_payload(
    *,
    extension_support_surface: Mapping[str, Any] | None,
    artifact_path: str,
    exists: bool,
    actuals: Mapping[str, Any] | None,
    schedule: Mapping[str, Any] | None,
) -> dict[str, Any]:
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else {}
    )
    plan = build_extension_execution_plan(surface)
    schedule_payload = dict(schedule) if isinstance(schedule, Mapping) else {}
    actuals_payload = dict(actuals) if isinstance(actuals, Mapping) else {}
    external_bindings = _normalize_extension_external_bindings(
        actuals_payload.get("external_bindings")
    )
    profile_items = actuals_payload.get("profiles")
    actual_profiles = (
        [dict(item) for item in profile_items if isinstance(item, Mapping)]
        if isinstance(profile_items, Sequence) and not isinstance(profile_items, (str, bytes))
        else []
    )
    schedule_profile_items = schedule_payload.get("profiles")
    schedule_profiles = (
        [dict(item) for item in schedule_profile_items if isinstance(item, Mapping)]
        if isinstance(schedule_profile_items, Sequence)
        and not isinstance(schedule_profile_items, (str, bytes))
        else []
    )
    profiles_by_id = {
        str(item.get("id")): item
        for item in actual_profiles
        if _is_non_empty_string(item.get("id"))
    }
    schedule_profiles_by_id = {
        str(item.get("id")): item
        for item in schedule_profiles
        if _is_non_empty_string(item.get("id"))
    }

    def _build_template(payload: Mapping[str, Any]) -> dict[str, Any]:
        template = payload.get("execution_template")
        return dict(template) if isinstance(template, Mapping) else {}

    def _first_owner(items: Any) -> str | None:
        if not isinstance(items, Sequence) or isinstance(items, (str, bytes)):
            return None
        for item in items:
            if not isinstance(item, Mapping):
                continue
            owner_role = _artifact_optional_string(item.get("owner_role"))
            if _is_non_empty_string(owner_role):
                return owner_role
        return None

    def _last_owner(items: Any) -> str | None:
        if not isinstance(items, Sequence) or isinstance(items, (str, bytes)):
            return None
        for item in reversed(list(items)):
            if not isinstance(item, Mapping):
                continue
            owner_role = _artifact_optional_string(item.get("owner_role"))
            if _is_non_empty_string(owner_role):
                return owner_role
        return None

    def _default_contract_file(root_value: Any, *parts: str) -> str | None:
        root_path = _artifact_optional_string(root_value)
        if not _is_non_empty_string(root_path):
            return None
        return _contract_path(root_path, *parts)

    def _join_binding_reference(base: Any, suffix: str) -> str | None:
        base_reference = _artifact_optional_string(base)
        if not _is_non_empty_string(base_reference):
            return None
        return base_reference.rstrip("/") + "/" + suffix

    actionable_plan_profiles = [
        item
        for item in plan.get("profiles", [])
        if isinstance(item, Mapping) and item.get("actionable") is True
    ]
    first_actionable_profile = actionable_plan_profiles[0] if actionable_plan_profiles else {}
    first_actionable_template = (
        _build_template(first_actionable_profile)
        if isinstance(first_actionable_profile, Mapping)
        else {}
    )
    default_window_trigger_recorded_by = (
        _artifact_optional_string(actuals_payload.get("window_trigger_recorded_by"))
        or _first_owner(first_actionable_template.get("upgrade_window_steps"))
        or _artifact_optional_string(first_actionable_template.get("handoff_owner_role"))
    )
    default_signoff_recorded_by = (
        _artifact_optional_string(actuals_payload.get("signoff_recorded_by"))
        or _last_owner(first_actionable_template.get("signoff_checkpoints"))
        or _artifact_optional_string(first_actionable_template.get("handoff_owner_role"))
    )
    default_residual_risk_reviewed_by = (
        _artifact_optional_string(actuals_payload.get("residual_risk_reviewed_by"))
        or _artifact_optional_string(first_actionable_template.get("residual_risk_owner_role"))
        or _artifact_optional_string(first_actionable_template.get("exception_review_owner_role"))
    )
    default_closure_archived_by = (
        _artifact_optional_string(actuals_payload.get("closure_archived_by"))
        or _artifact_optional_string(first_actionable_template.get("rollback_evidence_owner_role"))
        or _artifact_optional_string(first_actionable_template.get("escalation_closure_owner_role"))
    )
    default_delivery_root = _artifact_optional_string(actuals_payload.get("delivery_root")) or _artifact_optional_string(
        schedule_payload.get("delivery_root")
    )
    default_approval_identity_source_path = (
        _artifact_optional_string(actuals_payload.get("approval_identity_source_path"))
        or _default_contract_file(default_delivery_root, "approval_identity_source.json")
    )
    default_approval_identity_source_type = (
        _artifact_optional_string(actuals_payload.get("approval_identity_source_type"))
        or "customer_signoff_registry"
    )
    default_approval_identity_reference = (
        _artifact_optional_string(actuals_payload.get("approval_identity_reference"))
        or (
            ":".join(
                value
                for value in [
                    _artifact_optional_string(actuals_payload.get("engagement_id"))
                    or _artifact_optional_string(schedule_payload.get("engagement_id")),
                    _artifact_optional_string(actuals_payload.get("window_id"))
                    or _artifact_optional_string(schedule_payload.get("window_id")),
                    default_signoff_recorded_by,
                ]
                if _is_non_empty_string(value)
            )
            or None
        )
    )
    default_due_trigger_checked_at = (
        _artifact_optional_string(actuals_payload.get("due_trigger_checked_at"))
        or _artifact_optional_string(actuals_payload.get("residual_risk_reviewed_at"))
        or _artifact_optional_string(actuals_payload.get("closure_archived_at"))
        or _artifact_optional_string(schedule_payload.get("signoff_due_at"))
    )
    default_archive_target_binding_type = (
        _artifact_optional_string(actuals_payload.get("archive_target_binding_type"))
        or "customer_archive_destination"
    )
    default_archive_target_binding_reference_base = (
        _artifact_optional_string(
            actuals_payload.get("archive_target_binding_reference_base")
        )
        or (
            "archive://"
            + "/".join(
                value
                for value in [
                    _artifact_optional_string(actuals_payload.get("engagement_id"))
                    or _artifact_optional_string(schedule_payload.get("engagement_id")),
                    _artifact_optional_string(actuals_payload.get("window_id"))
                    or _artifact_optional_string(schedule_payload.get("window_id")),
                ]
                if _is_non_empty_string(value)
            )
            or None
        )
    )
    default_due_trigger_binding_type = (
        _artifact_optional_string(actuals_payload.get("due_trigger_binding_type"))
        or "customer_due_trigger_schedule"
    )
    default_due_trigger_binding_reference_base = (
        _artifact_optional_string(
            actuals_payload.get("due_trigger_binding_reference_base")
        )
        or (
            "schedule://"
            + "/".join(
                value
                for value in [
                    _artifact_optional_string(actuals_payload.get("engagement_id"))
                    or _artifact_optional_string(schedule_payload.get("engagement_id")),
                    _artifact_optional_string(actuals_payload.get("window_id"))
                    or _artifact_optional_string(schedule_payload.get("window_id")),
                ]
                if _is_non_empty_string(value)
            )
            or None
        )
    )
    hydrated_profiles: list[dict[str, Any]] = []
    actionable_profiles = 0
    ready_profiles = 0
    window_triggers_recorded = 0
    signoffs_recorded = 0
    exception_reviews_scheduled = 0
    residual_risk_reviews_recorded = 0
    archive_targets_ready = 0
    archive_target_bindings_ready = 0
    due_trigger_checks_ready = 0
    due_trigger_bindings_ready = 0
    closure_indexes_ready = 0
    closures_archived = 0
    missing_profiles: list[str] = []
    for item in plan.get("profiles", []):
        if not isinstance(item, Mapping):
            continue
        profile_id = str(item.get("id") or "")
        profile_actuals = profiles_by_id.get(profile_id, {})
        profile_schedule = schedule_profiles_by_id.get(profile_id, {})
        profile_template = _build_template(item)
        actionable = item.get("actionable") is True
        declared = item.get("declared") is True
        signoff_owner_role = (
            _artifact_optional_string(profile_actuals.get("signoff_owner_role"))
            or _last_owner(profile_template.get("signoff_checkpoints"))
            or _artifact_optional_string(profile_template.get("handoff_owner_role"))
        )
        exception_review_owner_role = (
            _artifact_optional_string(profile_actuals.get("exception_review_owner_role"))
            or _artifact_optional_string(profile_template.get("exception_review_owner_role"))
            or _artifact_optional_string(profile_template.get("residual_risk_owner_role"))
        )
        closure_archive_owner_role = (
            _artifact_optional_string(profile_actuals.get("closure_archive_owner_role"))
            or _artifact_optional_string(profile_template.get("rollback_evidence_owner_role"))
            or _artifact_optional_string(profile_template.get("escalation_closure_owner_role"))
        )
        profile_window_trigger_record_path = _artifact_optional_string(
            profile_actuals.get("window_trigger_record_path")
        ) or _artifact_optional_string(profile_schedule.get("window_trigger_record_path"))
        profile_root = (
            Path(profile_window_trigger_record_path).parent.as_posix()
            if _is_non_empty_string(profile_window_trigger_record_path)
            else _default_contract_file(default_delivery_root, "profiles", profile_id)
        )
        profile_closure_archive_root = _artifact_optional_string(
            profile_actuals.get("closure_archive_root")
        ) or _artifact_optional_string(profile_schedule.get("closure_archive_root"))
        archive_target_binding_reference = (
            _artifact_optional_string(profile_actuals.get("archive_target_binding_reference"))
            or _join_binding_reference(default_archive_target_binding_reference_base, profile_id)
        )
        due_trigger_binding_reference = (
            _artifact_optional_string(profile_actuals.get("due_trigger_binding_reference"))
            or _join_binding_reference(default_due_trigger_binding_reference_base, profile_id)
        )
        profile_payload = {
            "id": profile_id,
            "label": str(item.get("label") or ""),
            "declared": declared,
            "actionable": actionable,
            "signoff_due_at": _artifact_optional_string(
                profile_actuals.get("signoff_due_at")
            )
            or _artifact_optional_string(profile_schedule.get("signoff_due_at")),
            "exception_review_due_at": _artifact_optional_string(
                profile_actuals.get("exception_review_due_at")
            )
            or _artifact_optional_string(actuals_payload.get("exception_review_due_at"))
            or _artifact_optional_string(profile_schedule.get("exception_review_due_at")),
            "closure_archive_due_at": _artifact_optional_string(
                profile_actuals.get("closure_archive_due_at")
            )
            or _artifact_optional_string(actuals_payload.get("closure_archive_due_at"))
            or _artifact_optional_string(profile_schedule.get("closure_archive_due_at")),
            "approval_identity_source_path": _artifact_optional_string(
                profile_actuals.get("approval_identity_source_path")
            )
            or default_approval_identity_source_path,
            "window_trigger_record_path": profile_window_trigger_record_path,
            "signoff_record_path": _artifact_optional_string(
                profile_actuals.get("signoff_record_path")
            )
            or _artifact_optional_string(profile_schedule.get("signoff_record_path")),
            "exception_review_record_path": _artifact_optional_string(
                profile_actuals.get("exception_review_record_path")
            )
            or _artifact_optional_string(profile_schedule.get("exception_review_record_path")),
            "residual_risk_review_record_path": _artifact_optional_string(
                profile_actuals.get("residual_risk_review_record_path")
            )
            or _artifact_optional_string(
                profile_schedule.get("residual_risk_review_record_path")
            ),
            "archive_target_path": _artifact_optional_string(
                profile_actuals.get("archive_target_path")
            )
            or _default_contract_file(profile_closure_archive_root, "archive_target.json"),
            "archive_target_binding_reference": archive_target_binding_reference,
            "due_trigger_check_path": _artifact_optional_string(
                profile_actuals.get("due_trigger_check_path")
            )
            or _default_contract_file(profile_root, "due_trigger_check.json"),
            "due_trigger_binding_reference": due_trigger_binding_reference,
            "closure_index_path": _artifact_optional_string(
                profile_actuals.get("closure_index_path")
            )
            or _artifact_optional_string(profile_schedule.get("closure_index_path")),
            "closure_manifest_path": _artifact_optional_string(
                profile_actuals.get("closure_manifest_path")
            )
            or _artifact_optional_string(profile_schedule.get("closure_manifest_path")),
            "closure_archive_root": profile_closure_archive_root,
            "signoff_owner_role": signoff_owner_role,
            "exception_review_owner_role": exception_review_owner_role,
            "closure_archive_owner_role": closure_archive_owner_role,
        }
        if actionable:
            actionable_profiles += 1
            if _is_non_empty_string(profile_payload["window_trigger_record_path"]):
                window_triggers_recorded += 1
            if _is_non_empty_string(profile_payload["signoff_record_path"]):
                signoffs_recorded += 1
            if _is_non_empty_string(profile_payload["exception_review_record_path"]):
                exception_reviews_scheduled += 1
            if _is_non_empty_string(profile_payload["residual_risk_review_record_path"]):
                residual_risk_reviews_recorded += 1
            if _is_non_empty_string(profile_payload["archive_target_path"]):
                archive_targets_ready += 1
            if _is_non_empty_string(profile_payload["archive_target_binding_reference"]):
                archive_target_bindings_ready += 1
            if _is_non_empty_string(profile_payload["due_trigger_check_path"]):
                due_trigger_checks_ready += 1
            if _is_non_empty_string(profile_payload["due_trigger_binding_reference"]):
                due_trigger_bindings_ready += 1
            if _is_non_empty_string(profile_payload["closure_index_path"]):
                closure_indexes_ready += 1
            if _is_non_empty_string(profile_payload["closure_manifest_path"]):
                closures_archived += 1
            if all(
                _is_non_empty_string(profile_payload[field])
                for field in [
                    "signoff_due_at",
                    "exception_review_due_at",
                    "closure_archive_due_at",
                    "approval_identity_source_path",
                    "window_trigger_record_path",
                    "signoff_record_path",
                    "exception_review_record_path",
                    "residual_risk_review_record_path",
                    "archive_target_path",
                    "archive_target_binding_reference",
                    "due_trigger_check_path",
                    "due_trigger_binding_reference",
                    "closure_index_path",
                    "closure_manifest_path",
                    "closure_archive_root",
                    "signoff_owner_role",
                    "exception_review_owner_role",
                    "closure_archive_owner_role",
                ]
            ):
                ready_profiles += 1
            else:
                missing_profiles.append(profile_id)
        hydrated_profiles.append(profile_payload)

    metadata_fields = {
        "engagement_id": _artifact_optional_string(actuals_payload.get("engagement_id")),
        "customer_name": _artifact_optional_string(actuals_payload.get("customer_name")),
        "site_name": _artifact_optional_string(actuals_payload.get("site_name")),
        "change_ticket": _artifact_optional_string(actuals_payload.get("change_ticket")),
        "window_id": _artifact_optional_string(actuals_payload.get("window_id")),
        "approval_identity_source_path": default_approval_identity_source_path,
        "approval_identity_source_type": default_approval_identity_source_type,
        "approval_identity_reference": default_approval_identity_reference,
        "archive_target_binding_type": default_archive_target_binding_type,
        "archive_target_binding_reference_base": default_archive_target_binding_reference_base,
        "exception_review_due_at": _artifact_optional_string(
            actuals_payload.get("exception_review_due_at")
        )
        or _artifact_optional_string(schedule_payload.get("exception_review_due_at")),
        "closure_archive_due_at": _artifact_optional_string(
            actuals_payload.get("closure_archive_due_at")
        )
        or _artifact_optional_string(schedule_payload.get("closure_archive_due_at")),
        "due_trigger_binding_type": default_due_trigger_binding_type,
        "due_trigger_binding_reference_base": default_due_trigger_binding_reference_base,
        "due_trigger_checked_at": default_due_trigger_checked_at,
        "window_trigger_recorded_at": _artifact_optional_string(
            actuals_payload.get("window_trigger_recorded_at")
        ),
        "window_trigger_recorded_by": default_window_trigger_recorded_by,
        "signoff_recorded_at": _artifact_optional_string(
            actuals_payload.get("signoff_recorded_at")
        ),
        "signoff_recorded_by": default_signoff_recorded_by,
        "residual_risk_reviewed_at": _artifact_optional_string(
            actuals_payload.get("residual_risk_reviewed_at")
        ),
        "residual_risk_reviewed_by": default_residual_risk_reviewed_by,
        "closure_archived_at": _artifact_optional_string(
            actuals_payload.get("closure_archived_at")
        ),
        "closure_archived_by": default_closure_archived_by,
        "delivery_root": _artifact_optional_string(actuals_payload.get("delivery_root")),
        "closure_archive_root": _artifact_optional_string(
            actuals_payload.get("closure_archive_root")
        ),
    }
    missing_metadata = [
        name for name, value in metadata_fields.items() if not _is_non_empty_string(value)
    ]
    missing_profiles = list(dict.fromkeys(missing_profiles))
    declared_profiles = _coerce_non_negative_int(plan.get("declared_profiles")) or 0
    generated_at = _artifact_optional_string(actuals_payload.get("generated_at"))
    external_bindings_details = _build_extension_external_bindings_details(
        actuals_payload.get("external_bindings")
    )
    status = (
        "ready"
        if exists
        and plan.get("status") == "ready"
        and not missing_metadata
        and ready_profiles == actionable_profiles
        else "blocked"
    )
    summary = (
        f"Extension execution actuals {status}: "
        f"{ready_profiles}/{actionable_profiles} actionable profiles attested, "
        f"window_triggers={window_triggers_recorded}/{actionable_profiles}, "
        f"signoffs={signoffs_recorded}/{actionable_profiles}, "
        f"exception_reviews={exception_reviews_scheduled}/{actionable_profiles}, "
        f"archive_targets={archive_targets_ready}/{actionable_profiles}, "
        f"archive_bindings={archive_target_bindings_ready}/{actionable_profiles}, "
        f"due_trigger_checks={due_trigger_checks_ready}/{actionable_profiles}, "
        f"due_trigger_bindings={due_trigger_bindings_ready}/{actionable_profiles}, "
        f"closure_indexes={closure_indexes_ready}/{actionable_profiles}, "
        f"closures={closures_archived}/{actionable_profiles}."
    )
    if missing_metadata:
        summary += " Missing metadata: " + ", ".join(missing_metadata) + "."
    if missing_profiles:
        summary += " Profiles missing executed records: " + ", ".join(missing_profiles) + "."
    summary += " " + external_bindings_details["external_bindings_summary"]

    payload = {
        "schema_version": EXTENSION_EXECUTION_ACTUALS_VERSION,
        "artifact_type": EXTENSION_EXECUTION_ACTUALS_ARTIFACT_TYPE,
        "generated_at": generated_at,
        "artifact_path": artifact_path,
        "exists": exists,
        "status": status,
        "summary": summary,
        **metadata_fields,
        "declared_profiles": declared_profiles,
        "actionable_profiles": actionable_profiles,
        "ready_profiles": ready_profiles,
        "window_triggers_recorded": window_triggers_recorded,
        "signoffs_recorded": signoffs_recorded,
        "exception_reviews_scheduled": exception_reviews_scheduled,
        "residual_risk_reviews_recorded": residual_risk_reviews_recorded,
        "archive_targets_ready": archive_targets_ready,
        "archive_target_bindings_ready": archive_target_bindings_ready,
        "due_trigger_checks_ready": due_trigger_checks_ready,
        "due_trigger_bindings_ready": due_trigger_bindings_ready,
        "closure_indexes_ready": closure_indexes_ready,
        "closures_archived": closures_archived,
        **external_bindings_details,
        "missing_profiles": missing_profiles,
        "profiles": to_jsonable(hydrated_profiles),
    }
    if external_bindings is not None:
        payload["external_bindings"] = external_bindings
    return payload


def build_extension_execution_actuals_artifact(
    *,
    project_root: str | Path | None = None,
    extension_support_surface: Mapping[str, Any] | None = None,
    artifact_path: str | Path | None = None,
    generated_at: str | None = None,
    schedule_artifact: Mapping[str, Any] | None = None,
    schedule_artifact_path: str | Path | None = None,
    window_trigger_recorded_at: str | None = None,
    signoff_recorded_at: str | None = None,
    residual_risk_reviewed_at: str | None = None,
    closure_archived_at: str | None = None,
    window_trigger_recorded_by: str | None = None,
    signoff_recorded_by: str | None = None,
    residual_risk_reviewed_by: str | None = None,
    closure_archived_by: str | None = None,
    exception_review_due_at: str | None = None,
    closure_archive_due_at: str | None = None,
    approval_identity_source_path: str | None = None,
    approval_identity_source_type: str | None = None,
    approval_identity_reference: str | None = None,
    archive_target_binding_type: str | None = None,
    archive_target_binding_reference_base: str | None = None,
    due_trigger_binding_type: str | None = None,
    due_trigger_binding_reference_base: str | None = None,
    due_trigger_checked_at: str | None = None,
    external_bindings: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else dict(
            build_customer_delivery_surface(project_root=resolved_root).get(
                "extension_support_surface",
                {},
            )
        )
    )
    schedule = build_extension_execution_schedule(
        project_root=resolved_root,
        extension_support_surface=surface,
        artifact_path=schedule_artifact_path,
        schedule_artifact=schedule_artifact,
    )
    schedule_profiles = schedule.get("profiles", [])
    actual_profiles: list[dict[str, Any]] = []
    default_window_trigger_recorded_at = (
        window_trigger_recorded_at
        or _artifact_optional_string(schedule.get("window_trigger_at"))
    )
    default_signoff_recorded_at = (
        signoff_recorded_at or _artifact_optional_string(schedule.get("signoff_due_at"))
    )
    default_residual_risk_reviewed_at = (
        residual_risk_reviewed_at
        or _artifact_optional_string(schedule.get("signoff_due_at"))
    )
    default_closure_archived_at = (
        closure_archived_at
        or _artifact_optional_string(schedule.get("closure_archive_due_at"))
    )
    default_delivery_root = _artifact_optional_string(schedule.get("delivery_root"))
    default_approval_identity_source_path = (
        approval_identity_source_path
        or (
            _contract_path(default_delivery_root, "approval_identity_source.json")
            if _is_non_empty_string(default_delivery_root)
            else None
        )
    )
    default_approval_identity_source_type = (
        approval_identity_source_type or "customer_signoff_registry"
    )
    default_approval_identity_reference = (
        approval_identity_reference
        or (
            ":".join(
                value
                for value in [
                    _artifact_optional_string(schedule.get("engagement_id")),
                    _artifact_optional_string(schedule.get("window_id")),
                    signoff_recorded_by,
                ]
                if _is_non_empty_string(value)
            )
            or None
        )
    )
    default_archive_target_binding_type = (
        archive_target_binding_type or "customer_archive_destination"
    )
    default_archive_target_binding_reference_base = (
        archive_target_binding_reference_base
        or (
            "archive://"
            + "/".join(
                value
                for value in [
                    _artifact_optional_string(schedule.get("engagement_id")),
                    _artifact_optional_string(schedule.get("window_id")),
                ]
                if _is_non_empty_string(value)
            )
            or None
        )
    )
    default_due_trigger_binding_type = (
        due_trigger_binding_type or "customer_due_trigger_schedule"
    )
    default_due_trigger_binding_reference_base = (
        due_trigger_binding_reference_base
        or (
            "schedule://"
            + "/".join(
                value
                for value in [
                    _artifact_optional_string(schedule.get("engagement_id")),
                    _artifact_optional_string(schedule.get("window_id")),
                ]
                if _is_non_empty_string(value)
            )
            or None
        )
    )
    default_due_trigger_checked_at = (
        due_trigger_checked_at
        or default_closure_archived_at
        or default_residual_risk_reviewed_at
    )
    for item in schedule_profiles:
        if not isinstance(item, Mapping):
            continue
        profile_id = str(item.get("id") or "")
        window_trigger_record_path = item.get("window_trigger_record_path")
        profile_root = (
            Path(str(window_trigger_record_path)).parent.as_posix()
            if _is_non_empty_string(window_trigger_record_path)
            else (
                _contract_path(default_delivery_root, "profiles", profile_id)
                if _is_non_empty_string(default_delivery_root)
                else None
            )
        )
        closure_archive_root = item.get("closure_archive_root")
        actual_profiles.append(
            {
                "id": profile_id,
                "signoff_due_at": item.get("signoff_due_at"),
                "exception_review_due_at": (
                    exception_review_due_at or item.get("exception_review_due_at")
                ),
                "closure_archive_due_at": (
                    closure_archive_due_at or item.get("closure_archive_due_at")
                ),
                "approval_identity_source_path": default_approval_identity_source_path,
                "window_trigger_record_path": window_trigger_record_path,
                "signoff_record_path": item.get("signoff_record_path"),
                "exception_review_record_path": item.get("exception_review_record_path"),
                "residual_risk_review_record_path": item.get(
                    "residual_risk_review_record_path"
                ),
                "archive_target_path": (
                    _contract_path(str(closure_archive_root), "archive_target.json")
                    if _is_non_empty_string(closure_archive_root)
                    else None
                ),
                "archive_target_binding_reference": (
                    default_archive_target_binding_reference_base.rstrip("/") + "/" + profile_id
                    if _is_non_empty_string(default_archive_target_binding_reference_base)
                    else None
                ),
                "due_trigger_check_path": (
                    _contract_path(str(profile_root), "due_trigger_check.json")
                    if _is_non_empty_string(profile_root)
                    else None
                ),
                "due_trigger_binding_reference": (
                    default_due_trigger_binding_reference_base.rstrip("/") + "/" + profile_id
                    if _is_non_empty_string(default_due_trigger_binding_reference_base)
                    else None
                ),
                "closure_index_path": item.get("closure_index_path"),
                "closure_manifest_path": item.get("closure_manifest_path"),
                "closure_archive_root": closure_archive_root,
            }
        )
    payload = {
        "generated_at": generated_at or datetime.now().isoformat(),
        "engagement_id": schedule.get("engagement_id"),
        "customer_name": schedule.get("customer_name"),
        "site_name": schedule.get("site_name"),
        "change_ticket": schedule.get("change_ticket"),
        "window_id": schedule.get("window_id"),
        "approval_identity_source_path": default_approval_identity_source_path,
        "approval_identity_source_type": default_approval_identity_source_type,
        "approval_identity_reference": default_approval_identity_reference,
        "archive_target_binding_type": default_archive_target_binding_type,
        "archive_target_binding_reference_base": default_archive_target_binding_reference_base,
        "exception_review_due_at": exception_review_due_at
        or schedule.get("exception_review_due_at"),
        "closure_archive_due_at": closure_archive_due_at
        or schedule.get("closure_archive_due_at"),
        "due_trigger_binding_type": default_due_trigger_binding_type,
        "due_trigger_binding_reference_base": default_due_trigger_binding_reference_base,
        "due_trigger_checked_at": default_due_trigger_checked_at,
        "window_trigger_recorded_at": default_window_trigger_recorded_at,
        "window_trigger_recorded_by": window_trigger_recorded_by,
        "signoff_recorded_at": default_signoff_recorded_at,
        "signoff_recorded_by": signoff_recorded_by,
        "residual_risk_reviewed_at": default_residual_risk_reviewed_at,
        "residual_risk_reviewed_by": residual_risk_reviewed_by,
        "closure_archived_at": default_closure_archived_at,
        "closure_archived_by": closure_archived_by,
        "delivery_root": schedule.get("delivery_root"),
        "closure_archive_root": schedule.get("closure_archive_root"),
        "profiles": actual_profiles,
    }
    normalized_external_bindings = _normalize_extension_external_bindings(
        external_bindings
    )
    if normalized_external_bindings is not None:
        payload["external_bindings"] = normalized_external_bindings
    return _build_extension_execution_actuals_payload(
        extension_support_surface=surface,
        artifact_path=(
            Path(artifact_path).as_posix()
            if artifact_path is not None
            else default_extension_execution_actuals_artifact_path()
        ),
        exists=True,
        actuals=payload,
        schedule=schedule,
    )


def build_extension_execution_actuals(
    *,
    project_root: str | Path | None = None,
    extension_support_surface: Mapping[str, Any] | None = None,
    artifact_path: str | Path | None = None,
    actuals_artifact: Mapping[str, Any] | None = None,
    schedule_artifact: Mapping[str, Any] | None = None,
    schedule_artifact_path: str | Path | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    surface = (
        dict(extension_support_surface)
        if isinstance(extension_support_surface, Mapping)
        else dict(
            build_customer_delivery_surface(project_root=resolved_root).get(
                "extension_support_surface",
                {},
            )
        )
    )
    artifact_reference = (
        Path(artifact_path).as_posix()
        if artifact_path is not None
        else default_extension_execution_actuals_artifact_path()
    )
    payload = dict(actuals_artifact) if isinstance(actuals_artifact, Mapping) else None
    exists = False
    if payload is not None:
        exists = payload.get("exists") is True
        artifact_reference = str(payload.get("artifact_path") or artifact_reference)
    else:
        resolved_artifact_path = _resolve_release_artifact_path(
            artifact_reference,
            resolved_root,
        )
        if resolved_artifact_path.is_file():
            exists = True
            try:
                loaded = json.loads(resolved_artifact_path.read_text(encoding="utf-8"))
                payload = dict(loaded) if isinstance(loaded, Mapping) else {}
            except Exception:
                payload = {}
    schedule_payload = build_extension_execution_schedule(
        project_root=resolved_root,
        extension_support_surface=surface,
        artifact_path=schedule_artifact_path,
        schedule_artifact=schedule_artifact,
    )

    return _build_extension_execution_actuals_payload(
        extension_support_surface=surface,
        artifact_path=artifact_reference,
        exists=exists,
        actuals=payload,
        schedule=schedule_payload,
    )


def validate_extension_execution_actuals(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["extension_execution_actuals must be an object"]

    errors: list[str] = []
    missing = sorted(EXTENSION_EXECUTION_ACTUALS_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "extension_execution_actuals missing required fields: " + ", ".join(missing)
        )
    if payload.get("schema_version") != EXTENSION_EXECUTION_ACTUALS_VERSION:
        errors.append(
            "extension_execution_actuals.schema_version must be "
            f"{EXTENSION_EXECUTION_ACTUALS_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != EXTENSION_EXECUTION_ACTUALS_ARTIFACT_TYPE:
        errors.append(
            "extension_execution_actuals.artifact_type must be "
            f"{EXTENSION_EXECUTION_ACTUALS_ARTIFACT_TYPE!r}"
        )
    if "exists" in payload and not isinstance(payload.get("exists"), bool):
        errors.append("extension_execution_actuals.exists must be a boolean")
    if payload.get("status") not in EXTENSION_EXECUTION_ACTUALS_STATUSES:
        errors.append(
            "extension_execution_actuals.status must be one of "
            f"{sorted(EXTENSION_EXECUTION_ACTUALS_STATUSES)}"
        )
    for field in [
        "generated_at",
        "artifact_path",
        "summary",
        "engagement_id",
        "customer_name",
        "site_name",
        "change_ticket",
        "window_id",
        "approval_identity_source_path",
        "approval_identity_source_type",
        "approval_identity_reference",
        "archive_target_binding_type",
        "archive_target_binding_reference_base",
        "exception_review_due_at",
        "closure_archive_due_at",
        "due_trigger_binding_type",
        "due_trigger_binding_reference_base",
        "due_trigger_checked_at",
        "window_trigger_recorded_at",
        "window_trigger_recorded_by",
        "signoff_recorded_at",
        "signoff_recorded_by",
        "residual_risk_reviewed_at",
        "residual_risk_reviewed_by",
        "closure_archived_at",
        "closure_archived_by",
        "delivery_root",
        "closure_archive_root",
    ]:
        if field in payload and payload.get(field) is not None and not _is_non_empty_string(
            payload.get(field)
        ):
            errors.append(
                f"extension_execution_actuals.{field} must be null or a non-empty string"
            )
    for field in [
        "declared_profiles",
        "actionable_profiles",
        "ready_profiles",
        "window_triggers_recorded",
        "signoffs_recorded",
        "exception_reviews_scheduled",
        "residual_risk_reviews_recorded",
        "archive_targets_ready",
        "archive_target_bindings_ready",
        "due_trigger_checks_ready",
        "due_trigger_bindings_ready",
        "closure_indexes_ready",
        "closures_archived",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(
                f"extension_execution_actuals.{field} must be a non-negative integer"
            )
    if (
        _is_non_negative_int(payload.get("actionable_profiles"))
        and _is_non_negative_int(payload.get("ready_profiles"))
        and payload.get("ready_profiles") > payload.get("actionable_profiles")
    ):
        errors.append(
            "extension_execution_actuals.ready_profiles must be <= actionable_profiles"
        )
    missing_profiles = payload.get("missing_profiles")
    if not isinstance(missing_profiles, list):
        errors.append("extension_execution_actuals.missing_profiles must be a list")
    else:
        for index, item in enumerate(missing_profiles, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    "extension_execution_actuals.missing_profiles"
                    f"[{index}] must be a non-empty string"
                )
    external_bindings = payload.get("external_bindings")
    if external_bindings is not None:
        if not isinstance(external_bindings, Mapping):
            errors.append("extension_execution_actuals.external_bindings must be an object")
        else:
            if "config_path" in external_bindings and not _is_non_empty_string(
                external_bindings.get("config_path")
            ):
                errors.append(
                    "extension_execution_actuals.external_bindings.config_path must be a non-empty string"
                )
            for field in ("approval_identity", "archive_target", "due_trigger"):
                if field in external_bindings and not isinstance(
                    external_bindings.get(field), Mapping
                ):
                    errors.append(
                        "extension_execution_actuals.external_bindings."
                        f"{field} must be an object"
                    )
                section_payload = external_bindings.get(field)
                if isinstance(section_payload, Mapping):
                    binding_state = section_payload.get("binding_state")
                    if (
                        binding_state is not None
                        and binding_state not in EXTENSION_EXTERNAL_BINDING_STATES
                    ):
                        errors.append(
                            "extension_execution_actuals.external_bindings."
                            f"{field}.binding_state must be one of "
                            f"{sorted(EXTENSION_EXTERNAL_BINDING_STATES)}"
                        )
                    if binding_state == "confirmed":
                        for confirmation_field in (
                            "confirmed_by",
                            "confirmed_at",
                            "confirmation_ticket",
                        ):
                            if not _is_non_empty_string(
                                section_payload.get(confirmation_field)
                            ):
                                errors.append(
                                    "extension_execution_actuals.external_bindings."
                                    f"{field}.{confirmation_field} must be a non-empty "
                                    "string when binding_state is 'confirmed'"
                                )
                    for optional_field in ("confirmation_notes", "confirmation_evidence"):
                        if optional_field in section_payload and not _is_non_empty_string(
                            section_payload.get(optional_field)
                        ):
                            errors.append(
                                "extension_execution_actuals.external_bindings."
                                f"{field}.{optional_field} must be a non-empty string"
                            )
    external_bindings_status = payload.get("external_bindings_status")
    if external_bindings_status is not None and external_bindings_status not in EXTENSION_EXTERNAL_BINDING_STATUSES:
        errors.append(
            "extension_execution_actuals.external_bindings_status must be one of "
            f"{sorted(EXTENSION_EXTERNAL_BINDING_STATUSES)}"
        )
    if "external_bindings_summary" in payload and not _is_non_empty_string(
        payload.get("external_bindings_summary")
    ):
        errors.append(
            "extension_execution_actuals.external_bindings_summary must be a non-empty string"
        )
    if "external_bindings_follow_up_required" in payload and not isinstance(
        payload.get("external_bindings_follow_up_required"), bool
    ):
        errors.append(
            "extension_execution_actuals.external_bindings_follow_up_required must be a boolean"
        )
    for field in [
        "external_bindings_declared_count",
        "external_bindings_ready_count",
        "external_bindings_placeholder_count",
        "external_bindings_confirmed_count",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(
                f"extension_execution_actuals.{field} must be a non-negative integer"
            )
    for field in [
        "external_bindings_missing_sections",
        "external_bindings_placeholder_sections",
        "external_bindings_draft_sections",
        "external_bindings_unconfirmed_sections",
        "external_bindings_confirmed_sections",
        "external_bindings_confirmation_missing_sections",
        "external_bindings_confirmed_by",
        "external_bindings_confirmation_tickets",
    ]:
        value = payload.get(field)
        if value is not None:
            if not isinstance(value, list):
                errors.append(f"extension_execution_actuals.{field} must be a list")
            else:
                for index, item in enumerate(value, start=1):
                    if not _is_non_empty_string(item):
                        errors.append(
                            f"extension_execution_actuals.{field}[{index}] must be a non-empty string"
                        )
    if "external_bindings_last_confirmed_at" in payload and payload.get(
        "external_bindings_last_confirmed_at"
    ) is not None and not _is_non_empty_string(payload.get("external_bindings_last_confirmed_at")):
        errors.append(
            "extension_execution_actuals.external_bindings_last_confirmed_at must be a non-empty string"
        )
    if (
        external_bindings_status == "ready"
        and payload.get("external_bindings_follow_up_required") is True
    ):
        errors.append(
            "extension_execution_actuals.external_bindings_follow_up_required must be false when external_bindings_status is 'ready'"
        )
    if (
        external_bindings_status == "ready"
        and payload.get("external_bindings_confirmation_missing_sections")
    ):
        errors.append(
            "extension_execution_actuals.external_bindings_confirmation_missing_sections must be empty when external_bindings_status is 'ready'"
        )
    if (
        external_bindings_status == "ready"
        and payload.get("external_bindings_unconfirmed_sections")
    ):
        errors.append(
            "extension_execution_actuals.external_bindings_unconfirmed_sections must be empty when external_bindings_status is 'ready'"
        )
    profiles = payload.get("profiles")
    if not isinstance(profiles, list):
        errors.append("extension_execution_actuals.profiles must be a list")
        return errors
    for index, item in enumerate(profiles, start=1):
        prefix = f"extension_execution_actuals.profiles[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        missing_fields = sorted(
            EXTENSION_EXECUTION_ACTUALS_PROFILE_REQUIRED_FIELDS - set(item)
        )
        if missing_fields:
            errors.append(f"{prefix} missing required fields: {', '.join(missing_fields)}")
        for field in ["id", "label"]:
            if field in item and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        for field in ["declared", "actionable"]:
            if field in item and not isinstance(item.get(field), bool):
                errors.append(f"{prefix}.{field} must be a boolean")
        for field in [
            "signoff_due_at",
            "exception_review_due_at",
            "closure_archive_due_at",
            "approval_identity_source_path",
            "window_trigger_record_path",
            "signoff_record_path",
            "exception_review_record_path",
            "residual_risk_review_record_path",
            "archive_target_path",
            "archive_target_binding_reference",
            "due_trigger_check_path",
            "due_trigger_binding_reference",
            "closure_index_path",
            "closure_manifest_path",
            "closure_archive_root",
            "signoff_owner_role",
            "exception_review_owner_role",
            "closure_archive_owner_role",
        ]:
            if field in item and item.get(field) is not None and not _is_non_empty_string(
                item.get(field)
            ):
                errors.append(f"{prefix}.{field} must be null or a non-empty string")
    if payload.get("status") == "ready":
        for field in [
            "engagement_id",
            "customer_name",
            "site_name",
            "change_ticket",
            "window_id",
            "approval_identity_source_path",
            "approval_identity_source_type",
            "approval_identity_reference",
            "archive_target_binding_type",
            "archive_target_binding_reference_base",
            "exception_review_due_at",
            "closure_archive_due_at",
            "due_trigger_binding_type",
            "due_trigger_binding_reference_base",
            "due_trigger_checked_at",
            "window_trigger_recorded_at",
            "window_trigger_recorded_by",
            "signoff_recorded_at",
            "signoff_recorded_by",
            "residual_risk_reviewed_at",
            "residual_risk_reviewed_by",
            "closure_archived_at",
            "closure_archived_by",
            "delivery_root",
            "closure_archive_root",
        ]:
            if not _is_non_empty_string(payload.get(field)):
                errors.append(
                    f"extension_execution_actuals.{field} is required when status is 'ready'"
                )
        if payload.get("exists") is not True:
            errors.append("extension_execution_actuals.exists must be true when status is 'ready'")
        if payload.get("missing_profiles") != []:
            errors.append(
                "extension_execution_actuals.missing_profiles must be empty when status is 'ready'"
            )
        if isinstance(profiles, list):
            for index, item in enumerate(profiles, start=1):
                if not isinstance(item, Mapping) or item.get("actionable") is not True:
                    continue
                prefix = f"extension_execution_actuals.profiles[{index}]"
                for field in [
                    "signoff_due_at",
                    "exception_review_due_at",
                    "closure_archive_due_at",
                    "approval_identity_source_path",
                    "window_trigger_record_path",
                    "signoff_record_path",
                    "exception_review_record_path",
                    "residual_risk_review_record_path",
                    "archive_target_path",
                    "archive_target_binding_reference",
                    "due_trigger_check_path",
                    "due_trigger_binding_reference",
                    "closure_index_path",
                    "closure_manifest_path",
                    "closure_archive_root",
                    "signoff_owner_role",
                    "exception_review_owner_role",
                    "closure_archive_owner_role",
                ]:
                    if not _is_non_empty_string(item.get(field)):
                        errors.append(
                            f"{prefix}.{field} is required when status is 'ready'"
                        )
    return errors


def write_extension_execution_actuals_artifact(
    payload: Mapping[str, Any],
    path: str | Path,
) -> Path:
    errors = validate_extension_execution_actuals(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def build_customer_external_bindings_confirmation_report(
    *,
    project_root: str | Path | None = None,
    actuals_artifact_path: str | Path | None = None,
    output_path: str | Path | None = None,
    generated_at: str | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    resolved_actuals_contract_path = _artifact_optional_string(actuals_artifact_path)
    if resolved_actuals_contract_path is None:
        resolved_actuals_contract_path = default_extension_execution_actuals_artifact_path()
    resolved_output_contract_path = _artifact_optional_string(output_path)
    if resolved_output_contract_path is None:
        resolved_output_contract_path = (
            default_customer_external_bindings_confirmation_report_path()
        )
    resolved_actuals_path = _resolve_release_artifact_path(
        resolved_actuals_contract_path,
        resolved_root,
    )
    command = build_customer_external_bindings_confirmation_report_command(
        output_path=resolved_output_contract_path,
        actuals_artifact_path=resolved_actuals_contract_path,
    )
    base_metrics: dict[str, Any] = {
        "actuals_artifact_path": resolved_actuals_contract_path,
        "actuals_exists": resolved_actuals_path.is_file(),
    }
    if not resolved_actuals_path.is_file():
        return build_release_evidence_report(
            evidence_name="customer_external_bindings_confirmation",
            status="blocked",
            summary=(
                "customer_external_bindings_confirmation evidence blocked: "
                "extension execution actuals artifact is missing at "
                f"{resolved_actuals_contract_path}."
            ),
            command=command,
            generated_at=generated_at,
            metrics=base_metrics,
        )
    try:
        payload = json.loads(resolved_actuals_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return build_release_evidence_report(
            evidence_name="customer_external_bindings_confirmation",
            status="blocked",
            summary=(
                "customer_external_bindings_confirmation evidence blocked: "
                f"extension execution actuals artifact could not be parsed: {exc}"
            ),
            command=command,
            generated_at=generated_at,
            metrics=base_metrics,
        )
    errors = validate_extension_execution_actuals(payload)
    if errors:
        return build_release_evidence_report(
            evidence_name="customer_external_bindings_confirmation",
            status="blocked",
            summary=(
                "customer_external_bindings_confirmation evidence blocked: "
                "extension execution actuals artifact is invalid. "
                + "; ".join(errors)
            ),
            command=command,
            generated_at=generated_at,
            metrics=base_metrics,
        )

    actuals = dict(payload)
    external_bindings = (
        dict(actuals.get("external_bindings", {}))
        if isinstance(actuals.get("external_bindings"), Mapping)
        else {}
    )
    external_bindings_status = _artifact_optional_string(
        actuals.get("external_bindings_status")
    )
    confirmed_sections = [
        str(item).strip()
        for item in actuals.get("external_bindings_confirmed_sections", [])
        if _is_non_empty_string(item)
    ]
    draft_sections = [
        str(item).strip()
        for item in actuals.get("external_bindings_draft_sections", [])
        if _is_non_empty_string(item)
    ]
    unconfirmed_sections = [
        str(item).strip()
        for item in actuals.get("external_bindings_unconfirmed_sections", [])
        if _is_non_empty_string(item)
    ]
    confirmation_missing_sections = [
        str(item).strip()
        for item in actuals.get("external_bindings_confirmation_missing_sections", [])
        if _is_non_empty_string(item)
    ]
    placeholder_sections = [
        str(item).strip()
        for item in actuals.get("external_bindings_placeholder_sections", [])
        if _is_non_empty_string(item)
    ]
    missing_sections = [
        str(item).strip()
        for item in actuals.get("external_bindings_missing_sections", [])
        if _is_non_empty_string(item)
    ]
    summary_suffix = _artifact_optional_string(actuals.get("external_bindings_summary"))
    metrics = {
        **base_metrics,
        "actuals_status": _artifact_optional_string(actuals.get("status")),
        "external_bindings_status": external_bindings_status,
        "config_path": _artifact_optional_string(external_bindings.get("config_path")),
        "declared_sections": _coerce_non_negative_int(
            actuals.get("external_bindings_declared_count")
        ),
        "ready_sections": _coerce_non_negative_int(
            actuals.get("external_bindings_ready_count")
        ),
        "confirmed_sections": confirmed_sections,
        "draft_sections": draft_sections,
        "unconfirmed_sections": unconfirmed_sections,
        "confirmation_missing_sections": confirmation_missing_sections,
        "placeholder_sections": placeholder_sections,
        "missing_sections": missing_sections,
        "confirmed_by": [
            str(item).strip()
            for item in actuals.get("external_bindings_confirmed_by", [])
            if _is_non_empty_string(item)
        ],
        "confirmation_tickets": [
            str(item).strip()
            for item in actuals.get("external_bindings_confirmation_tickets", [])
            if _is_non_empty_string(item)
        ],
        "last_confirmed_at": _artifact_optional_string(
            actuals.get("external_bindings_last_confirmed_at")
        ),
    }
    if external_bindings_status == "ready":
        declared_sections = max(
            _coerce_non_negative_int(actuals.get("external_bindings_declared_count")),
            len(confirmed_sections),
        )
        return build_release_evidence_report(
            evidence_name="customer_external_bindings_confirmation",
            status="passed",
            summary=(
                "customer_external_bindings_confirmation evidence passed: "
                + (
                    summary_suffix
                    or f"{len(confirmed_sections)}/{declared_sections} sections confirmed."
                )
            ),
            command=command,
            generated_at=generated_at,
            metrics=metrics,
        )

    actionable_profiles = _coerce_non_negative_int(actuals.get("actionable_profiles"))
    if actionable_profiles == 0 and external_bindings_status is None:
        return build_release_evidence_report(
            evidence_name="customer_external_bindings_confirmation",
            status="passed",
            summary=(
                "customer_external_bindings_confirmation evidence passed: "
                "no actionable extension profiles declared."
            ),
            command=command,
            generated_at=generated_at,
            metrics=metrics,
        )

    pending_sets = [
        ("draft sections", draft_sections),
        ("unconfirmed sections", unconfirmed_sections),
        ("confirmation-missing sections", confirmation_missing_sections),
        ("placeholder sections", placeholder_sections),
        ("missing sections", missing_sections),
    ]
    pending_details = [
        f"{label}={','.join(values)}"
        for label, values in pending_sets
        if values
    ]
    detail_suffix = ""
    if pending_details:
        detail_suffix = " Pending: " + "; ".join(pending_details) + "."
    return build_release_evidence_report(
        evidence_name="customer_external_bindings_confirmation",
        status="blocked",
        summary=(
            "customer_external_bindings_confirmation evidence blocked: "
            + (
                summary_suffix
                or (
                    "external bindings are not confirmed for the customer delivery "
                    "window."
                )
            )
            + detail_suffix
        ),
        command=command,
        generated_at=generated_at,
        metrics=metrics,
    )


def _parse_release_iso_datetime(value: Any) -> datetime | None:
    if not _is_non_empty_string(value):
        return None
    try:
        return datetime.fromisoformat(str(value).strip())
    except ValueError:
        return None


def _normalize_release_datetimes(
    left: datetime | None, right: datetime | None
) -> tuple[datetime | None, datetime | None]:
    if left is None or right is None:
        return left, right
    normalized_left = left
    normalized_right = right
    if normalized_left.tzinfo is None and normalized_right.tzinfo is not None:
        normalized_left = normalized_left.replace(tzinfo=normalized_right.tzinfo)
    elif normalized_left.tzinfo is not None and normalized_right.tzinfo is None:
        normalized_right = normalized_right.replace(tzinfo=normalized_left.tzinfo)
    return normalized_left, normalized_right


def _days_until_release_datetime(
    target: datetime | None, reference: datetime | None
) -> float | None:
    normalized_target, normalized_reference = _normalize_release_datetimes(
        target,
        reference,
    )
    if normalized_target is None or normalized_reference is None:
        return None
    return (normalized_target - normalized_reference).total_seconds() / 86400.0


def _derive_vulnerability_exception_review_lists(
    payload: Mapping[str, Any],
) -> dict[str, list[str]]:
    review_due_exception_ids: list[str] = []
    review_due_exception_components: list[str] = []
    review_due_exception_tickets: list[str] = []
    expired_exception_ids: list[str] = []
    generated_at = _parse_release_iso_datetime(payload.get("generated_at"))
    review_window_days = payload.get("review_window_days")
    if not isinstance(review_window_days, int) or review_window_days < 0:
        review_window_days = DEFAULT_VULNERABILITY_EXCEPTION_REVIEW_WINDOW_DAYS
    exceptions = payload.get("exceptions")
    if not isinstance(exceptions, list):
        exceptions = []
    for item in exceptions:
        if not isinstance(item, Mapping):
            continue
        status = str(item.get("status") or "").strip()
        exception_id = str(item.get("id") or "").strip()
        component = str(item.get("component") or "").strip()
        ticket = str(item.get("ticket") or "").strip()
        expires_at = _parse_release_iso_datetime(item.get("expires_at"))
        if status == "expired":
            if exception_id:
                expired_exception_ids.append(exception_id)
            continue
        if status != "active":
            continue
        days_until_expiry = _days_until_release_datetime(expires_at, generated_at)
        if (
            days_until_expiry is None
            or days_until_expiry < 0
            or days_until_expiry > review_window_days
        ):
            continue
        if exception_id:
            review_due_exception_ids.append(exception_id)
        if component:
            review_due_exception_components.append(component)
        if ticket:
            review_due_exception_tickets.append(ticket)
    return {
        "review_due_exception_ids": review_due_exception_ids,
        "review_due_exception_components": review_due_exception_components,
        "review_due_exception_tickets": review_due_exception_tickets,
        "expired_exception_ids": expired_exception_ids,
    }


def build_vulnerability_exception_review_report(
    *,
    project_root: str | Path | None = None,
    exception_report_path: str | Path | None = None,
    output_path: str | Path | None = None,
    generated_at: str | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    resolved_exception_contract_path = _artifact_optional_string(exception_report_path)
    if resolved_exception_contract_path is None:
        resolved_exception_contract_path = (
            "test_env/release_evidence/security/vulnerability_exception_report.json"
        )
    resolved_output_contract_path = _artifact_optional_string(output_path)
    if resolved_output_contract_path is None:
        resolved_output_contract_path = default_vulnerability_exception_review_report_path()
    resolved_exception_path = _resolve_release_artifact_path(
        resolved_exception_contract_path,
        resolved_root,
    )
    command = build_vulnerability_exception_review_report_command(
        output_path=resolved_output_contract_path,
        exception_report_path=resolved_exception_contract_path,
    )
    base_metrics: dict[str, Any] = {
        "vulnerability_exception_report_path": resolved_exception_contract_path,
        "vulnerability_exception_report_exists": resolved_exception_path.is_file(),
    }
    if not resolved_exception_path.is_file():
        return build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="blocked",
            summary=(
                "vulnerability_exception_review evidence blocked: "
                "vulnerability exception report is missing at "
                f"{resolved_exception_contract_path}."
            ),
            command=command,
            generated_at=generated_at,
            metrics=base_metrics,
        )
    try:
        payload = json.loads(resolved_exception_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="blocked",
            summary=(
                "vulnerability_exception_review evidence blocked: "
                f"vulnerability exception report could not be parsed: {exc}"
            ),
            command=command,
            generated_at=generated_at,
            metrics=base_metrics,
        )
    errors = validate_vulnerability_exception_report(payload)
    if errors:
        return build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="blocked",
            summary=(
                "vulnerability_exception_review evidence blocked: "
                "vulnerability exception report is invalid. "
                + "; ".join(errors)
            ),
            command=command,
            generated_at=generated_at,
            metrics=base_metrics,
        )

    review_due_exception_ids = [
        str(item).strip()
        for item in payload.get("review_due_exception_ids", [])
        if _is_non_empty_string(item)
    ]
    review_due_exception_components = [
        str(item).strip()
        for item in payload.get("review_due_exception_components", [])
        if _is_non_empty_string(item)
    ]
    review_due_exception_tickets = [
        str(item).strip()
        for item in payload.get("review_due_exception_tickets", [])
        if _is_non_empty_string(item)
    ]
    expired_exception_ids = [
        str(item).strip()
        for item in payload.get("expired_exception_ids", [])
        if _is_non_empty_string(item)
    ]
    if (
        not review_due_exception_ids
        and not review_due_exception_components
        and not review_due_exception_tickets
        and not expired_exception_ids
    ):
        derived_review_lists = _derive_vulnerability_exception_review_lists(payload)
        review_due_exception_ids = derived_review_lists["review_due_exception_ids"]
        review_due_exception_components = derived_review_lists[
            "review_due_exception_components"
        ]
        review_due_exception_tickets = derived_review_lists[
            "review_due_exception_tickets"
        ]
        expired_exception_ids = derived_review_lists["expired_exception_ids"]

    active_exception_count = int(payload.get("active_exception_count") or 0)
    expired_exception_count = int(payload.get("expired_exception_count") or 0)
    review_due_exception_count = int(payload.get("review_due_exception_count") or 0)
    review_status = _artifact_optional_string(payload.get("review_status"))
    next_exception_expiry = _artifact_optional_string(payload.get("next_exception_expiry"))
    review_window_days = (
        int(payload.get("review_window_days"))
        if isinstance(payload.get("review_window_days"), int)
        and payload.get("review_window_days") >= 0
        else DEFAULT_VULNERABILITY_EXCEPTION_REVIEW_WINDOW_DAYS
    )
    review_follow_up_required = (
        review_due_exception_count > 0 or expired_exception_count > 0
    )
    review_candidate_count = review_due_exception_count + expired_exception_count
    metrics = {
        **base_metrics,
        "active_exception_count": active_exception_count,
        "expired_exception_count": expired_exception_count,
        "review_window_days": review_window_days,
        "review_due_exception_count": review_due_exception_count,
        "review_due_exception_ids": review_due_exception_ids,
        "review_due_exception_components": review_due_exception_components,
        "review_due_exception_tickets": review_due_exception_tickets,
        "expired_exception_ids": expired_exception_ids,
        "review_status": review_status,
        "next_exception_expiry": next_exception_expiry,
        "review_follow_up_required": review_follow_up_required,
        "review_candidate_count": review_candidate_count,
    }

    if expired_exception_count > 0:
        expired_preview = ", ".join(expired_exception_ids[:3])
        if len(expired_exception_ids) > 3:
            expired_preview += ", ..."
        return build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="blocked",
            summary=(
                "vulnerability_exception_review evidence blocked: "
                f"{expired_exception_count} vulnerability exception(s) are expired"
                + (
                    f" ({expired_preview})." if expired_preview else "."
                )
            ),
            command=command,
            generated_at=generated_at,
            metrics=metrics,
        )
    if review_due_exception_count > 0:
        return build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="passed",
            summary=(
                "vulnerability_exception_review evidence passed: "
                f"{review_due_exception_count} active exception(s) require review "
                f"inside the {review_window_days}-day window"
                + (
                    f" before {next_exception_expiry}."
                    if next_exception_expiry
                    else "."
                )
            ),
            command=command,
            generated_at=generated_at,
            metrics=metrics,
        )
    if active_exception_count > 0:
        return build_release_evidence_report(
            evidence_name="vulnerability_exception_review",
            status="passed",
            summary=(
                "vulnerability_exception_review evidence passed: "
                f"{active_exception_count} active exception(s) are tracked and none are "
                "inside the current review window."
            ),
            command=command,
            generated_at=generated_at,
            metrics=metrics,
        )
    return build_release_evidence_report(
        evidence_name="vulnerability_exception_review",
        status="passed",
        summary=(
            "vulnerability_exception_review evidence passed: "
            "no active vulnerability exceptions require review."
        ),
        command=command,
        generated_at=generated_at,
        metrics=metrics,
    )


def _external_mainline_string_list(value: Any) -> list[str]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        return []
    return [
        str(item).strip()
        for item in value
        if _is_non_empty_string(item)
    ]


def _load_external_mainline_release_evidence_preview(
    *,
    project_root: Path,
    report_path: str | Path,
    evidence_name: str,
) -> dict[str, Any]:
    resolved_report_path = _resolve_release_artifact_path(str(report_path), project_root)
    display_report_path = (
        _project_relative_artifact_path(resolved_report_path, project_root=project_root)
        or str(resolved_report_path)
    )
    preview: dict[str, Any] = {
        "path": str(resolved_report_path),
        "status": "missing",
        "summary": f"{evidence_name} report is missing: {display_report_path}",
        "command": "",
        "metrics": {},
    }
    if not resolved_report_path.is_file():
        return preview
    try:
        payload = json.loads(resolved_report_path.read_text(encoding="utf-8"))
    except Exception as exc:
        preview["status"] = "blocked"
        preview["summary"] = (
            f"{evidence_name} report is unreadable: {exc}"
        )
        return preview
    errors = validate_release_evidence_report(payload)
    if payload.get("evidence_name") != evidence_name:
        errors.append(
            f"{evidence_name} report must use evidence_name={evidence_name!r}"
        )
    if errors:
        preview["status"] = "blocked"
        preview["summary"] = (
            f"{evidence_name} report is invalid: {'; '.join(errors)}"
        )
        return preview
    preview["status"] = payload.get("status") or "blocked"
    preview["summary"] = (
        payload.get("summary")
        or f"{evidence_name} report has no summary."
    )
    preview["command"] = _artifact_optional_string(payload.get("command")) or ""
    preview["metrics"] = (
        dict(payload.get("metrics"))
        if isinstance(payload.get("metrics"), Mapping)
        else {}
    )
    return preview


def _load_external_mainline_customer_config_preview(
    *,
    project_root: Path,
    config_path: str | Path,
) -> dict[str, Any]:
    resolved_config_path = _resolve_release_artifact_path(str(config_path), project_root)
    preview: dict[str, Any] = {
        "path": str(resolved_config_path),
        "exists": resolved_config_path.is_file(),
        "valid": False,
        "summary": f"customer external bindings config is missing: {resolved_config_path}",
        "confirmed_sections": [],
        "draft_sections": [],
        "confirmation_missing_sections": [],
        "invalid_sections": [],
    }
    if not resolved_config_path.is_file():
        return preview
    try:
        payload = json.loads(resolved_config_path.read_text(encoding="utf-8"))
    except Exception as exc:
        preview["summary"] = (
            f"customer external bindings config is unreadable: {exc}"
        )
        return preview
    if not isinstance(payload, Mapping):
        preview["summary"] = "customer external bindings config must be a JSON object."
        return preview

    confirmed_sections: list[str] = []
    draft_sections: list[str] = []
    confirmation_missing_sections: list[str] = []
    invalid_sections: list[str] = []
    for section_id in EXTENSION_EXTERNAL_BINDING_SECTION_IDS:
        section_payload = payload.get(section_id)
        if not isinstance(section_payload, Mapping):
            invalid_sections.append(section_id)
            continue
        state = _external_binding_section_state(section_payload)
        if state == "confirmed":
            if _external_binding_confirmation_missing_fields(section_payload):
                confirmation_missing_sections.append(section_id)
            else:
                confirmed_sections.append(section_id)
        elif state == "draft":
            draft_sections.append(section_id)
        else:
            invalid_sections.append(section_id)

    preview["valid"] = not invalid_sections
    preview["confirmed_sections"] = confirmed_sections
    preview["draft_sections"] = draft_sections
    preview["confirmation_missing_sections"] = confirmation_missing_sections
    preview["invalid_sections"] = invalid_sections
    if invalid_sections:
        preview["summary"] = (
            "customer external bindings config is invalid for sections: "
            + ", ".join(invalid_sections)
        )
    elif len(confirmed_sections) == len(EXTENSION_EXTERNAL_BINDING_SECTION_IDS):
        preview["summary"] = (
            "customer external bindings config already contains confirmed metadata "
            "for every managed section."
        )
    elif draft_sections or confirmation_missing_sections:
        preview["summary"] = (
            "customer external bindings config still needs real customer metadata "
            "or confirmation for one or more managed sections."
        )
    else:
        preview["summary"] = "customer external bindings config is present."
    return preview


def _load_external_mainline_industrial_rehearsal_preview(
    *,
    project_root: Path,
    report_path: str | Path,
) -> dict[str, Any]:
    resolved_report_path = _resolve_release_artifact_path(str(report_path), project_root)
    preview: dict[str, Any] = {
        "path": str(resolved_report_path),
        "status": "missing",
        "summary": (
            "industrial delivery rehearsal report is missing: "
            f"{resolved_report_path}"
        ),
        "stage_summary": {},
        "version": None,
        "tag": None,
    }
    if not resolved_report_path.is_file():
        return preview
    try:
        payload = json.loads(resolved_report_path.read_text(encoding="utf-8"))
    except Exception as exc:
        preview["status"] = "blocked"
        preview["summary"] = (
            f"industrial delivery rehearsal report is unreadable: {exc}"
        )
        return preview
    errors = validate_industrial_delivery_rehearsal_report_artifact(payload)
    if errors:
        preview["status"] = "blocked"
        preview["summary"] = (
            "industrial delivery rehearsal report is invalid: "
            + "; ".join(errors)
        )
        return preview
    preview["status"] = payload.get("status") or "blocked"
    preview["summary"] = (
        payload.get("summary")
        or "industrial delivery rehearsal report has no summary."
    )
    preview["stage_summary"] = (
        dict(payload.get("stage_summary"))
        if isinstance(payload.get("stage_summary"), Mapping)
        else {}
    )
    preview["version"] = _artifact_optional_string(payload.get("version"))
    preview["tag"] = _artifact_optional_string(payload.get("tag"))
    return preview


EXTERNAL_MAINLINE_INDUSTRIAL_LIVE_EVIDENCE_REQUIRED_FIELDS: tuple[
    tuple[str, str], ...
] = (
    ("target_environment", "真实客户环境标识"),
    ("access_method", "真实客户环境访问方式"),
    ("install_entrypoint", "install 实际命令或入口"),
    ("upgrade_entrypoint", "upgrade 实际命令或入口"),
    ("rollback_entrypoint", "rollback 实际命令或入口"),
    ("backup_restore_entrypoint", "backup-restore 实际命令或入口"),
    ("closure_archive_root", "closure archive 与现场留痕目录"),
)


def _external_mainline_is_placeholder_string(value: Any) -> bool:
    if not _is_non_empty_string(value):
        return False
    normalized = str(value).strip()
    lowered = normalized.lower()
    if lowered in {"placeholder", "<placeholder>", "tbd", "<tbd>"}:
        return True
    return normalized.startswith("<") and normalized.endswith(">")


def _normalize_external_mainline_industrial_live_evidence_inputs(
    payload: Mapping[str, Any] | None,
) -> dict[str, Any]:
    if not isinstance(payload, Mapping):
        return {}

    normalized: dict[str, Any] = {}
    enabled = payload.get("enabled")
    if isinstance(enabled, bool):
        normalized["enabled"] = enabled

    for field in [
        "target_environment",
        "access_method",
        "install_entrypoint",
        "upgrade_entrypoint",
        "rollback_entrypoint",
        "backup_restore_entrypoint",
        "closure_archive_root",
        "evidence_output_root",
        "notes",
    ]:
        value = _artifact_optional_string(payload.get(field))
        if value:
            normalized[field] = value
    return normalized


def _load_external_mainline_inputs_payload(
    *,
    project_root: Path,
    inputs_path: str | Path,
) -> dict[str, Any]:
    resolved_inputs_path = _resolve_release_artifact_path(str(inputs_path), project_root)
    if not resolved_inputs_path.is_file():
        return {}
    try:
        payload = json.loads(resolved_inputs_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(payload, Mapping):
        return {}
    return dict(payload)


def _normalize_release_op_session_context_summary(
    session: Mapping[str, Any] | None,
) -> dict[str, str]:
    if not isinstance(session, Mapping):
        return {}
    normalized: dict[str, str] = {}
    for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
        value = session.get(field)
        if _is_non_empty_string(value):
            normalized[field] = str(value).strip()
    return normalized


def _normalize_release_op_event_stream_summary(
    event_stream: Mapping[str, Any] | None,
) -> dict[str, Any]:
    if not isinstance(event_stream, Mapping):
        return {}
    normalized: dict[str, Any] = {}
    path = event_stream.get("path")
    if _is_non_empty_string(path):
        normalized["path"] = str(path).strip()
    event_count = event_stream.get("event_count")
    if _is_non_negative_int(event_count):
        normalized["event_count"] = event_count
    return normalized if normalized.get("path") else {}


def _append_external_mainline_control_plane_summary(
    summary: str,
    *,
    control_plane_session: Mapping[str, Any] | None,
    control_plane_event_stream: Mapping[str, Any] | None,
) -> str:
    base = summary.strip()
    fragments: list[str] = []
    session = _normalize_release_op_session_context_summary(control_plane_session)
    if session:
        session_fields = [
            f"{field}={session[field]}"
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]
            if field in session
        ]
        if session_fields:
            fragments.append("control_plane_session=" + ",".join(session_fields))
    event_stream = _normalize_release_op_event_stream_summary(control_plane_event_stream)
    if _is_non_negative_int(event_stream.get("event_count")):
        fragments.append(
            f"control_plane_events={int(event_stream['event_count'])}"
        )
    if not fragments:
        return base
    if base.endswith("."):
        base = base[:-1]
    return f"{base}, {'; '.join(fragments)}."


def _aggregate_release_op_control_plane_surface(
    *components: Mapping[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    session: dict[str, Any] = {}
    event_stream: dict[str, Any] = {}
    for component in components:
        if not isinstance(component, Mapping):
            continue
        if not session:
            session = _normalize_release_op_session_context_summary(
                component.get("control_plane_session")
                if isinstance(component.get("control_plane_session"), Mapping)
                else None
            )
        if not event_stream:
            event_stream = _normalize_release_op_event_stream_summary(
                component.get("control_plane_event_stream")
                if isinstance(component.get("control_plane_event_stream"), Mapping)
                else None
            )
        if session and event_stream:
            break
    return session, event_stream


def _build_control_plane_surface_payload(
    *,
    release_ops_execution: Mapping[str, Any] | None,
    control_plane_session: Mapping[str, Any] | None = None,
    control_plane_event_stream: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    release_ops_execution_payload: dict[str, Any] = {}
    if isinstance(release_ops_execution, Mapping):
        _hydrate_release_ops_execution_component(
            release_ops_execution_payload,
            component_source=release_ops_execution,
            bundle_path=None,
        )
    if not _is_non_empty_string(release_ops_execution_payload.get("status")):
        release_ops_execution_payload = {
            "status": "blocked",
            "summary": "release ops execution is unavailable for the control plane surface.",
        }

    normalized_session = _normalize_release_op_session_context_summary(
        control_plane_session
    )
    normalized_event_stream = _normalize_release_op_event_stream_summary(
        control_plane_event_stream
    )
    status = (
        str(release_ops_execution_payload.get("status")).strip()
        if release_ops_execution_payload.get("status") in RELEASE_EVIDENCE_STATUSES
        else "blocked"
    )
    summary = (
        f"Control plane surface {status}: release_ops_execution="
        f"{_format_release_ops_execution_component_status(release_ops_execution_payload)}."
    )
    summary = _append_external_mainline_control_plane_summary(
        summary,
        control_plane_session=normalized_session,
        control_plane_event_stream=normalized_event_stream,
    )
    payload = {
        "status": status,
        "summary": summary,
        "release_ops_execution": to_jsonable(release_ops_execution_payload),
    }
    event_count = _coerce_non_negative_int(
        release_ops_execution_payload.get("event_count")
    )
    if event_count is None:
        event_count = _coerce_non_negative_int(normalized_event_stream.get("event_count"))
    if event_count is not None:
        payload["event_count"] = event_count
    if normalized_session:
        payload["control_plane_session"] = to_jsonable(normalized_session)
    if normalized_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(normalized_event_stream)
    return payload


def _validate_control_plane_surface(
    payload: Any,
    *,
    prefix: str,
) -> list[str]:
    if not isinstance(payload, Mapping):
        return [f"{prefix} must be an object"]

    errors: list[str] = []
    missing = sorted(CONTROL_PLANE_SURFACE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"{prefix} missing required fields: {', '.join(missing)}")
    status = payload.get("status")
    if status not in RELEASE_EVIDENCE_STATUSES:
        errors.append(f"{prefix}.status must be one of {sorted(RELEASE_EVIDENCE_STATUSES)}")
    if "summary" in payload and not _is_non_empty_string(payload.get("summary")):
        errors.append(f"{prefix}.summary must be a non-empty string")
    event_count = payload.get("event_count")
    if event_count is not None and not _is_non_negative_int(event_count):
        errors.append(f"{prefix}.event_count must be a non-negative integer when present")

    release_ops_execution = payload.get("release_ops_execution")
    if not isinstance(release_ops_execution, Mapping):
        errors.append(f"{prefix}.release_ops_execution must be an object")
    else:
        component_status = release_ops_execution.get("status")
        if component_status not in RELEASE_EVIDENCE_STATUSES:
            errors.append(
                f"{prefix}.release_ops_execution.status must be one of {sorted(RELEASE_EVIDENCE_STATUSES)}"
            )
        if "summary" in release_ops_execution and not _is_non_empty_string(
            release_ops_execution.get("summary")
        ):
            errors.append(
                f"{prefix}.release_ops_execution.summary must be a non-empty string"
            )
        if "metrics" in release_ops_execution and not isinstance(
            release_ops_execution.get("metrics"), Mapping
        ):
            errors.append(f"{prefix}.release_ops_execution.metrics must be an object")

    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append(f"{prefix}.control_plane_session must be an object")
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        f"{prefix}.control_plane_session.{field} must be a non-empty string when present"
                    )

    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append(f"{prefix}.control_plane_event_stream must be an object")
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    f"{prefix}.control_plane_event_stream.path must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    f"{prefix}.control_plane_event_stream.event_count must be a non-negative integer when present"
                )
    return errors


def read_release_control_plane_surface(
    *,
    project_root: str | Path | None = None,
    manifest_path: str | Path | None = None,
    release_ops_execution_report_path: str | Path | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    manifest_candidates: list[Path] = []
    if manifest_path is not None:
        explicit_manifest = Path(str(manifest_path))
        manifest_candidates.append(
            explicit_manifest
            if explicit_manifest.is_absolute()
            else resolved_root / explicit_manifest
        )
    else:
        for candidate in [
            default_release_manifest_artifact_path(),
            "test_env/release/release_manifest_stable.json",
            "test_env/release/release_manifest_industrial.json",
        ]:
            manifest_candidates.append(resolved_root / candidate)

    for candidate in manifest_candidates:
        if not candidate.is_file():
            continue
        try:
            payload = json.loads(candidate.read_text(encoding="utf-8"))
        except Exception as exc:
            return {
                "source": "release_manifest",
                "manifest_path": str(candidate),
                "control_plane_surface": {
                    "status": "blocked",
                    "summary": "release manifest control plane surface is unreadable: "
                    f"{exc}",
                    "release_ops_execution": {
                        "status": "blocked",
                        "summary": "release ops execution is unavailable because the manifest is unreadable.",
                    },
                },
            }
        validation_errors = validate_release_manifest_artifact(payload)
        if validation_errors:
            return {
                "source": "release_manifest",
                "manifest_path": str(candidate),
                "validation_errors": validation_errors,
                "control_plane_surface": {
                    "status": "blocked",
                    "summary": "release manifest control plane surface is invalid: "
                    + "; ".join(validation_errors),
                    "release_ops_execution": {
                        "status": "blocked",
                        "summary": "release ops execution is unavailable because the manifest is invalid.",
                    },
                },
            }
        embedded_surface = (
            payload.get("control_plane_surface")
            if isinstance(payload.get("control_plane_surface"), Mapping)
            else None
        )
        if embedded_surface is not None and not _validate_control_plane_surface(
            embedded_surface,
            prefix="control_plane_surface",
        ):
            return {
                "source": "release_manifest",
                "manifest_path": str(candidate),
                "control_plane_surface": to_jsonable(dict(embedded_surface)),
            }
        surface = _build_control_plane_surface_payload(
            release_ops_execution=payload.get("release_ops_execution")
            if isinstance(payload.get("release_ops_execution"), Mapping)
            else None,
            control_plane_session=payload.get("control_plane_session")
            if isinstance(payload.get("control_plane_session"), Mapping)
            else None,
            control_plane_event_stream=payload.get("control_plane_event_stream")
            if isinstance(payload.get("control_plane_event_stream"), Mapping)
            else None,
        )
        return {
            "source": "release_manifest",
            "manifest_path": str(candidate),
            "control_plane_surface": to_jsonable(surface),
        }

    resolved_report_path = Path(
        _artifact_optional_string(release_ops_execution_report_path)
        or default_release_ops_execution_report_path()
    )
    if not resolved_report_path.is_absolute():
        resolved_report_path = resolved_root / resolved_report_path
    hydrated_reports = _hydrate_customer_acceptance_items(
        [
            {
                "name": "release_ops_execution",
                "path": str(resolved_report_path),
                "required": False,
            }
        ],
        resolved_root,
    )
    release_ops_execution: dict[str, Any] = {}
    if hydrated_reports:
        _hydrate_release_ops_execution_component(
            release_ops_execution,
            component_source=hydrated_reports[0],
            bundle_path=None,
        )
    control_plane_session, control_plane_event_stream = (
        _aggregate_release_op_control_plane_surface(release_ops_execution)
    )
    surface = _build_control_plane_surface_payload(
        release_ops_execution=release_ops_execution,
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )
    return {
        "source": "release_ops_execution_report",
        "release_ops_execution_report_path": str(resolved_report_path),
        "control_plane_surface": to_jsonable(surface),
    }


def build_external_mainline_execution_plan_artifact(
    *,
    project_root: str | Path | None = None,
    customer_external_bindings_closure_report_path: str | Path | None = None,
    vulnerability_exception_review_report_path: str | Path | None = None,
    industrial_delivery_rehearsal_report_path: str | Path | None = None,
    customer_config_path: str | Path | None = None,
    industrial_live_evidence_inputs: Mapping[str, Any] | None = None,
    control_plane_session: Mapping[str, Any] | None = None,
    control_plane_event_stream: Mapping[str, Any] | None = None,
    generated_at: str | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    resolved_customer_config = resolve_customer_external_bindings_config_path(
        customer_config_path
    )
    resolved_closure_report = (
        _artifact_optional_string(customer_external_bindings_closure_report_path)
        or default_customer_external_bindings_closure_report_path()
    )
    resolved_review_report = (
        _artifact_optional_string(vulnerability_exception_review_report_path)
        or default_vulnerability_exception_review_report_path()
    )
    resolved_industrial_report = (
        _artifact_optional_string(industrial_delivery_rehearsal_report_path)
        or default_canonical_industrial_delivery_rehearsal_report_path()
    )

    customer_config_preview = _load_external_mainline_customer_config_preview(
        project_root=resolved_root,
        config_path=resolved_customer_config,
    )
    closure_preview = _load_external_mainline_release_evidence_preview(
        project_root=resolved_root,
        report_path=resolved_closure_report,
        evidence_name="customer_external_bindings_closure",
    )
    review_preview = _load_external_mainline_release_evidence_preview(
        project_root=resolved_root,
        report_path=resolved_review_report,
        evidence_name="vulnerability_exception_review",
    )
    industrial_preview = _load_external_mainline_industrial_rehearsal_preview(
        project_root=resolved_root,
        report_path=resolved_industrial_report,
    )
    industrial_live_inputs = _normalize_external_mainline_industrial_live_evidence_inputs(
        industrial_live_evidence_inputs
    )

    customer_blocking_inputs: list[str] = []
    customer_follow_up_summary = closure_preview.get("summary") or customer_config_preview.get(
        "summary"
    )
    customer_report_metrics = (
        closure_preview.get("metrics")
        if isinstance(closure_preview.get("metrics"), Mapping)
        else {}
    )
    customer_failed_steps = _external_mainline_string_list(
        customer_report_metrics.get("failed_steps")
    )
    if closure_preview.get("status") == "passed":
        customer_status = "completed"
        customer_summary = (
            closure_preview.get("summary")
            or "customer external bindings closure already passed."
        )
    elif customer_config_preview.get("valid") is not True and customer_config_preview.get("exists"):
        customer_status = "blocked"
        customer_summary = str(customer_config_preview.get("summary") or "")
        customer_blocking_inputs.append(
            "修复 deployment/customer_delivery.external_bindings.customer.json 的无效 section"
        )
    elif len(customer_config_preview.get("confirmed_sections", [])) == len(
        EXTENSION_EXTERNAL_BINDING_SECTION_IDS
    ):
        customer_status = "ready_to_run"
        customer_summary = (
            "customer external bindings config 已具备 confirmed metadata；"
            "运行 managed closure runner 即可重建 actuals / confirmation / release evidence。"
        )
    else:
        customer_status = "waiting_external_input"
        customer_summary = str(
            customer_follow_up_summary
            or "customer external bindings 仍缺真实客户系统元数据。"
        )
        if not customer_config_preview.get("exists"):
            customer_blocking_inputs.append("生成 customer-specific external bindings config")
        if customer_failed_steps:
            customer_blocking_inputs.extend(customer_failed_steps)
        if customer_config_preview.get("draft_sections"):
            customer_blocking_inputs.append(
                "补齐真实客户 approval/archive/due-trigger 元数据"
            )
        if customer_config_preview.get("confirmation_missing_sections"):
            customer_blocking_inputs.append(
                "为 confirmed section 补齐 confirmed_by / confirmed_at / confirmation_ticket"
            )
        customer_blocking_inputs.extend(["confirmed_by", "confirmation_ticket"])
    customer_step = {
        "id": "customer_external_bindings_closure",
        "label": "Customer External Bindings Closure",
        "status": customer_status,
        "summary": customer_summary,
        "command": build_run_customer_external_bindings_closure_command(
            config_path=resolved_customer_config
        ),
        "auto_executable": True,
        "external_required": True,
        "automation_scope": "managed_closure_chain",
        "source_report_path": _project_relative_artifact_path(
            closure_preview.get("path"),
            project_root=resolved_root,
        ),
        "source_report_status": closure_preview.get("status"),
        "blocking_inputs": sorted(dict.fromkeys(customer_blocking_inputs)),
        "artifact_paths": [
            path
            for path in [
                _project_relative_artifact_path(
                    customer_config_preview.get("path"),
                    project_root=resolved_root,
                ),
                _project_relative_artifact_path(
                    closure_preview.get("path"),
                    project_root=resolved_root,
                ),
                default_extension_execution_actuals_artifact_path(),
                default_customer_external_bindings_confirmation_report_path(),
            ]
            if _is_non_empty_string(path)
        ],
    }

    review_metrics = (
        review_preview.get("metrics")
        if isinstance(review_preview.get("metrics"), Mapping)
        else {}
    )
    review_candidate_count = (
        int(review_metrics.get("review_candidate_count"))
        if isinstance(review_metrics.get("review_candidate_count"), int)
        and review_metrics.get("review_candidate_count") >= 0
        else 0
    )
    review_follow_up_required = review_metrics.get("review_follow_up_required") is True
    review_blocking_inputs: list[str] = []
    if review_preview.get("status") == "blocked":
        review_status = "blocked"
        review_summary = (
            review_preview.get("summary")
            or "vulnerability exception review report is blocked."
        )
        review_blocking_inputs.append(
            "修复 vulnerability_exception_review_report.json 对应的输入或过期 exception"
        )
    elif review_preview.get("status") == "missing":
        review_status = "ready_to_run"
        review_summary = (
            "vulnerability exception review report 缺失；"
            "可先自动重建 canonical review，再判断是否需要替换 no-fix exceptions。"
        )
    elif review_follow_up_required or review_candidate_count > 0:
        review_status = "waiting_external_input"
        review_summary = str(
            review_preview.get("summary")
            or "active vulnerability exceptions still require review."
        )
        review_blocking_inputs.extend(
            [
                "最新 upstream fix 版本或重算后的 scanner 结果",
                "更新 deployment/security/vulnerability_exceptions.input.json",
            ]
        )
    else:
        review_status = "completed"
        review_summary = (
            review_preview.get("summary")
            or "vulnerability exception review has no pending follow-up."
        )
    next_exception_expiry = _artifact_optional_string(
        review_metrics.get("next_exception_expiry")
    )
    if next_exception_expiry and review_status == "waiting_external_input":
        review_blocking_inputs.append(
            f"在 {next_exception_expiry} 前完成 replacement / review"
        )
    review_step = {
        "id": "vulnerability_exception_replacement",
        "label": "Vulnerability Exception Replacement",
        "status": review_status,
        "summary": review_summary,
        "command": build_vulnerability_exception_review_report_command(
            output_path=resolved_review_report
        ),
        "auto_executable": True,
        "external_required": True,
        "automation_scope": "review_report_refresh",
        "source_report_path": _project_relative_artifact_path(
            review_preview.get("path"),
            project_root=resolved_root,
        ),
        "source_report_status": review_preview.get("status"),
        "blocking_inputs": sorted(dict.fromkeys(review_blocking_inputs)),
        "artifact_paths": [
            path
            for path in [
                _project_relative_artifact_path(
                    review_preview.get("path"),
                    project_root=resolved_root,
                ),
                "deployment/security/vulnerability_exceptions.input.json",
                "test_env/release_evidence/security_release_preflight_report.json",
            ]
            if _is_non_empty_string(path)
        ],
    }

    industrial_missing_inputs = [
        label
        for field, label in EXTERNAL_MAINLINE_INDUSTRIAL_LIVE_EVIDENCE_REQUIRED_FIELDS
        if _external_mainline_is_placeholder_string(industrial_live_inputs.get(field))
        or not _is_non_empty_string(industrial_live_inputs.get(field))
    ]
    industrial_inputs_ready = (
        industrial_live_inputs.get("enabled") is not False
        and len(industrial_missing_inputs) == 0
    )
    if industrial_preview.get("status") == "blocked":
        industrial_status = "blocked"
        industrial_summary = (
            industrial_preview.get("summary")
            or "industrial delivery rehearsal baseline is blocked."
        )
    elif industrial_preview.get("status") == "missing":
        industrial_status = "ready_to_run"
        industrial_summary = (
            "industrial delivery rehearsal baseline 缺失；"
            "可先自动重建 canonical rehearsal，再进入真实客户环境留痕。"
        )
    elif industrial_inputs_ready:
        industrial_status = "ready_to_run"
        industrial_summary = (
            "canonical industrial rehearsal baseline 已就绪，"
            "真实环境 install / upgrade / rollback / backup-restore 输入已齐备；"
            "下一步可按受管 entrypoint 执行现场留痕并回写 industrial readiness / bundle。"
        )
    else:
        industrial_status = "waiting_external_input"
        stage_summary = industrial_preview.get("stage_summary")
        passed_stages = (
            int(stage_summary.get("passed"))
            if isinstance(stage_summary, Mapping)
            and isinstance(stage_summary.get("passed"), int)
            else 0
        )
        total_stages = (
            int(stage_summary.get("total"))
            if isinstance(stage_summary, Mapping)
            and isinstance(stage_summary.get("total"), int)
            else len(INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS)
        )
        industrial_summary = (
            "canonical industrial rehearsal baseline 已就绪，"
            f"当前为 {passed_stages}/{total_stages} 阶段通过；"
            "下一步需要把 install / upgrade / rollback / backup-restore 留痕切到真实客户环境。"
        )
    industrial_command = (
        "python tools/run_release_rehearsal.py --version ... --build-id ... "
        "--output-root test_env/release_rehearsal_industrial"
    )
    industrial_step = {
        "id": "industrial_delivery_live_evidence",
        "label": "Industrial Delivery Live Evidence",
        "status": industrial_status,
        "summary": industrial_summary,
        "command": industrial_command,
        "auto_executable": True,
        "external_required": True,
        "automation_scope": "rehearsal_refresh_and_live_capture_stub",
        "source_report_path": _project_relative_artifact_path(
            industrial_preview.get("path"),
            project_root=resolved_root,
        ),
        "source_report_status": industrial_preview.get("status"),
        "blocking_inputs": industrial_missing_inputs
        if industrial_status == "waiting_external_input"
        else [],
        "managed_inputs_ready": industrial_inputs_ready,
        "managed_inputs": industrial_live_inputs,
        "artifact_paths": [
            path
            for path in [
                _project_relative_artifact_path(
                    industrial_preview.get("path"),
                    project_root=resolved_root,
                ),
                "test_env/release_rehearsal_industrial/release_rehearsal_report.json",
                "test_env/release_rehearsal_industrial/test_env/release/customer_acceptance_bundle_industrial.json",
            ]
            if _is_non_empty_string(path)
        ],
    }

    steps = [customer_step, review_step, industrial_step]
    completed_steps = sum(1 for item in steps if item["status"] == "completed")
    ready_to_run_steps = sum(1 for item in steps if item["status"] == "ready_to_run")
    waiting_external_input_steps = sum(
        1 for item in steps if item["status"] == "waiting_external_input"
    )
    blocked_steps = sum(1 for item in steps if item["status"] == "blocked")
    auto_executable_steps = sum(1 for item in steps if item["auto_executable"] is True)
    plan_status = "blocked" if blocked_steps > 0 else "ready"
    summary = (
        "External mainline execution plan "
        f"{plan_status}: completed={completed_steps}, "
        f"ready_to_run={ready_to_run_steps}, "
        f"waiting_external_input={waiting_external_input_steps}, "
        f"blocked={blocked_steps}, "
        f"auto_executable={auto_executable_steps}."
    )
    payload = {
        "schema_version": EXTERNAL_MAINLINE_EXECUTION_PLAN_VERSION,
        "artifact_type": EXTERNAL_MAINLINE_EXECUTION_PLAN_ARTIFACT_TYPE,
        "generated_at": generated_at or datetime.now().isoformat(),
        "status": plan_status,
        "summary": _append_external_mainline_control_plane_summary(
            summary,
            control_plane_session=control_plane_session,
            control_plane_event_stream=control_plane_event_stream,
        ),
        "completed_steps": completed_steps,
        "ready_to_run_steps": ready_to_run_steps,
        "waiting_external_input_steps": waiting_external_input_steps,
        "blocked_steps": blocked_steps,
        "auto_executable_steps": auto_executable_steps,
        "steps": to_jsonable(steps),
    }
    normalized_session = _normalize_release_op_session_context_summary(
        control_plane_session
    )
    if normalized_session:
        payload["control_plane_session"] = to_jsonable(normalized_session)
    normalized_event_stream = _normalize_release_op_event_stream_summary(
        control_plane_event_stream
    )
    if normalized_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(normalized_event_stream)
    return payload


def validate_external_mainline_execution_plan_artifact(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["external_mainline_execution_plan must be an object"]

    errors: list[str] = []
    missing = sorted(EXTERNAL_MAINLINE_EXECUTION_PLAN_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "external_mainline_execution_plan missing required fields: "
            + ", ".join(missing)
        )
    if payload.get("schema_version") != EXTERNAL_MAINLINE_EXECUTION_PLAN_VERSION:
        errors.append(
            "external_mainline_execution_plan.schema_version must be "
            f"{EXTERNAL_MAINLINE_EXECUTION_PLAN_VERSION!r}"
        )
    if payload.get("artifact_type") != EXTERNAL_MAINLINE_EXECUTION_PLAN_ARTIFACT_TYPE:
        errors.append(
            "external_mainline_execution_plan.artifact_type must be "
            f"{EXTERNAL_MAINLINE_EXECUTION_PLAN_ARTIFACT_TYPE!r}"
        )
    if payload.get("status") not in EXTERNAL_MAINLINE_EXECUTION_PLAN_STATUSES:
        errors.append(
            "external_mainline_execution_plan.status must be one of "
            f"{sorted(EXTERNAL_MAINLINE_EXECUTION_PLAN_STATUSES)}"
        )
    for field in [
        "summary",
        "generated_at",
    ]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(
                f"external_mainline_execution_plan.{field} must be a non-empty string"
            )
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
        "auto_executable_steps",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(
                f"external_mainline_execution_plan.{field} must be a non-negative integer"
            )
    steps = payload.get("steps")
    if not isinstance(steps, list):
        errors.append("external_mainline_execution_plan.steps must be a list")
        return errors
    for index, item in enumerate(steps, start=1):
        prefix = f"external_mainline_execution_plan.steps[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        missing_fields = sorted(EXTERNAL_MAINLINE_EXECUTION_STEP_REQUIRED_FIELDS - set(item))
        if missing_fields:
            errors.append(f"{prefix} missing required fields: {', '.join(missing_fields)}")
        for field in [
            "id",
            "label",
            "summary",
            "command",
            "automation_scope",
            "source_report_path",
            "source_report_status",
        ]:
            if field in item and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if item.get("status") not in EXTERNAL_MAINLINE_EXECUTION_STEP_STATUSES:
            errors.append(
                f"{prefix}.status must be one of {sorted(EXTERNAL_MAINLINE_EXECUTION_STEP_STATUSES)}"
            )
        for field in ["auto_executable", "external_required"]:
            if field in item and not isinstance(item.get(field), bool):
                errors.append(f"{prefix}.{field} must be a boolean")
        for list_name in ["blocking_inputs", "artifact_paths"]:
            values = item.get(list_name)
            if not isinstance(values, list):
                errors.append(f"{prefix}.{list_name} must be a list")
                continue
            for list_index, list_item in enumerate(values, start=1):
                if not _is_non_empty_string(list_item):
                    errors.append(
                        f"{prefix}.{list_name}[{list_index}] must be a non-empty string"
                    )
    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append(
                "external_mainline_execution_plan.control_plane_session must be an object"
            )
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        "external_mainline_execution_plan.control_plane_session."
                        f"{field} must be a non-empty string"
                    )
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append(
                "external_mainline_execution_plan.control_plane_event_stream must be an object"
            )
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    "external_mainline_execution_plan.control_plane_event_stream.path "
                    "must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    "external_mainline_execution_plan.control_plane_event_stream."
                    "event_count must be a non-negative integer"
                )
    if isinstance(steps, list):
        expected_counts = {
            "completed_steps": sum(1 for item in steps if isinstance(item, Mapping) and item.get("status") == "completed"),
            "ready_to_run_steps": sum(1 for item in steps if isinstance(item, Mapping) and item.get("status") == "ready_to_run"),
            "waiting_external_input_steps": sum(
                1
                for item in steps
                if isinstance(item, Mapping) and item.get("status") == "waiting_external_input"
            ),
            "blocked_steps": sum(1 for item in steps if isinstance(item, Mapping) and item.get("status") == "blocked"),
            "auto_executable_steps": sum(
                1 for item in steps if isinstance(item, Mapping) and item.get("auto_executable") is True
            ),
        }
        for field, expected in expected_counts.items():
            if (
                _is_non_negative_int(payload.get(field))
                and payload.get(field) != expected
            ):
                errors.append(
                    f"external_mainline_execution_plan.{field} must equal {expected}"
                )
    return errors


def write_external_mainline_execution_plan_artifact(
    payload: Mapping[str, Any],
    path: str | Path,
) -> Path:
    errors = validate_external_mainline_execution_plan_artifact(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def build_external_mainline_input_checklist_report(
    *,
    project_root: str | Path | None = None,
    inputs_file_path: str | Path | None = None,
    external_mainline_execution_plan_path: str | Path | None = None,
    output_path: str | Path | None = None,
    generated_at: str | None = None,
    control_plane_session: Mapping[str, Any] | None = None,
    control_plane_event_stream: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    resolved_inputs_path = (
        _artifact_optional_string(inputs_file_path)
        or default_external_mainline_inputs_path()
    )
    inputs_payload = _load_external_mainline_inputs_payload(
        project_root=resolved_root,
        inputs_path=resolved_inputs_path,
    )
    customer_inputs = (
        dict(inputs_payload.get("customer_external_bindings"))
        if isinstance(inputs_payload.get("customer_external_bindings"), Mapping)
        else {}
    )
    review_inputs = (
        dict(inputs_payload.get("vulnerability_exception_review"))
        if isinstance(inputs_payload.get("vulnerability_exception_review"), Mapping)
        else {}
    )
    industrial_inputs = (
        dict(inputs_payload.get("industrial_live_evidence"))
        if isinstance(inputs_payload.get("industrial_live_evidence"), Mapping)
        else {}
    )
    industrial_rehearsal_inputs = (
        dict(inputs_payload.get("industrial_rehearsal"))
        if isinstance(inputs_payload.get("industrial_rehearsal"), Mapping)
        else {}
    )

    plan_payload = build_external_mainline_execution_plan_artifact(
        project_root=resolved_root,
        customer_config_path=customer_inputs.get("config"),
        customer_external_bindings_closure_report_path=customer_inputs.get(
            "closure_report_output"
        ),
        vulnerability_exception_review_report_path=review_inputs.get("report_output"),
        industrial_delivery_rehearsal_report_path=industrial_rehearsal_inputs.get(
            "report_path"
        ),
        industrial_live_evidence_inputs=industrial_inputs,
    )

    steps = {
        str(item.get("id")): dict(item)
        for item in plan_payload.get("steps", [])
        if isinstance(item, Mapping) and _is_non_empty_string(item.get("id"))
    }
    customer_step = steps.get("customer_external_bindings_closure", {})
    review_step = steps.get("vulnerability_exception_replacement", {})
    industrial_step = steps.get("industrial_delivery_live_evidence", {})

    customer_missing_inputs = _external_mainline_string_list(
        customer_step.get("blocking_inputs")
    )
    if _external_mainline_is_placeholder_string(customer_inputs.get("confirmed_by")):
        customer_missing_inputs.append("confirmed_by")
    if _external_mainline_is_placeholder_string(
        customer_inputs.get("confirmation_ticket")
    ):
        customer_missing_inputs.append("confirmation_ticket")
    customer_overrides = _artifact_optional_string(customer_inputs.get("overrides_file"))
    if not customer_overrides:
        customer_missing_inputs.append("overrides_file")

    vulnerability_missing_inputs = _external_mainline_string_list(
        review_step.get("blocking_inputs")
    )

    industrial_missing_inputs = _external_mainline_string_list(
        industrial_step.get("blocking_inputs")
    )

    waiting_steps = [
        step_id
        for step_id, payload in steps.items()
        if payload.get("status") == "waiting_external_input"
    ]
    ready_steps = [
        step_id
        for step_id, payload in steps.items()
        if payload.get("status") == "ready_to_run"
    ]
    completed_steps = [
        step_id
        for step_id, payload in steps.items()
        if payload.get("status") == "completed"
    ]

    missing_total = (
        len(customer_missing_inputs)
        + len(vulnerability_missing_inputs)
        + len(industrial_missing_inputs)
    )
    status = (
        "passed"
        if not waiting_steps and plan_payload.get("blocked_steps", 0) == 0
        else "blocked"
    )
    summary = (
        f"external_mainline_input_checklist {status}: "
        f"customer_missing={len(dict.fromkeys(customer_missing_inputs))}, "
        f"vulnerability_missing={len(dict.fromkeys(vulnerability_missing_inputs))}, "
        f"industrial_missing={len(dict.fromkeys(industrial_missing_inputs))}, "
        f"waiting_steps={len(waiting_steps)}, ready_steps={len(ready_steps)}, "
        f"completed_steps={len(completed_steps)}."
    )
    summary = _append_external_mainline_control_plane_summary(
        summary,
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )

    return build_release_evidence_report(
        evidence_name="external_mainline_input_checklist",
        status=status,
        summary=summary,
        command=build_external_mainline_input_checklist_command(
            output_path=output_path,
            inputs_file=resolved_inputs_path,
            external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        ),
        generated_at=generated_at,
        metrics={
            "inputs_file": resolved_inputs_path,
            "external_mainline_execution_plan_path": _artifact_optional_string(
                external_mainline_execution_plan_path
            )
            or default_external_mainline_execution_plan_path(),
            "customer_missing_inputs": sorted(dict.fromkeys(customer_missing_inputs)),
            "vulnerability_missing_inputs": sorted(
                dict.fromkeys(vulnerability_missing_inputs)
            ),
            "industrial_missing_inputs": sorted(dict.fromkeys(industrial_missing_inputs)),
            "waiting_external_input_steps": waiting_steps,
            "ready_to_run_steps": ready_steps,
            "completed_steps": completed_steps,
            "missing_input_count": missing_total,
            "customer_overrides_file": customer_overrides,
            "industrial_target_environment": industrial_inputs.get("target_environment"),
        },
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )


def build_customer_delivery_surface(
    *,
    project_root: str | Path | None = None,
    acceptance_documents: Sequence[Mapping[str, Any]] | None = None,
    release_ops_execution_report_path: str | Path | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    required_docs = acceptance_documents or default_customer_acceptance_documents()
    hydrated_documents = _hydrate_customer_acceptance_items(required_docs, resolved_root)
    documents_by_name = {
        str(item.get("name")): dict(item)
        for item in hydrated_documents
        if _is_non_empty_string(item.get("name"))
    }
    required_documents = sum(1 for item in hydrated_documents if item["required"] is True)
    required_documents_ready = sum(
        1
        for item in hydrated_documents
        if item["required"] is True and item["exists"] is True
    )
    missing_required_documents = [
        str(item["path"])
        for item in hydrated_documents
        if item["required"] is True
        and item["exists"] is not True
        and _is_non_empty_string(item.get("path"))
    ]
    extension_support_surface = build_extension_support_surface(
        project_root=resolved_root,
        acceptance_documents=required_docs,
    )
    hydrated_release_ops_reports = _hydrate_customer_acceptance_items(
        [
            {
                "name": "release_ops_execution",
                "path": _artifact_optional_string(release_ops_execution_report_path)
                or default_release_ops_execution_report_path(),
                "required": False,
            }
        ],
        resolved_root,
    )
    release_ops_execution: dict[str, Any] = {}
    if hydrated_release_ops_reports:
        _hydrate_release_ops_execution_component(
            release_ops_execution,
            component_source=hydrated_release_ops_reports[0],
            bundle_path=None,
        )
    control_plane_session, control_plane_event_stream = (
        _aggregate_release_op_control_plane_surface(release_ops_execution)
    )

    phase_e_focus_documents: list[dict[str, Any]] = []
    phase_e_documents_ready = 0
    missing_phase_e_documents: list[str] = []
    attachment_flags: dict[str, bool] = {}
    for flag_name, document_name in CUSTOMER_DELIVERY_PHASE_E_FOCUS_DOCUMENTS:
        document = dict(documents_by_name.get(document_name, {"name": document_name}))
        exists = document.get("exists") is True
        attachment_flags[flag_name] = exists
        if exists:
            phase_e_documents_ready += 1
        else:
            missing_path = document.get("path")
            missing_phase_e_documents.append(
                str(missing_path)
                if _is_non_empty_string(missing_path)
                else document_name
            )
        phase_e_focus_documents.append(document)

    status = (
        "ready"
        if required_documents == required_documents_ready
        and extension_support_surface.get("status") == "ready"
        else "blocked"
    )
    summary = (
        f"Customer delivery surface {status}: "
        f"{required_documents_ready}/{required_documents} required documents present, "
        f"Phase E focus docs {phase_e_documents_ready}/{len(CUSTOMER_DELIVERY_PHASE_E_FOCUS_DOCUMENTS)} attached, "
        "extension support profiles "
        f"{extension_support_surface.get('declared_profiles', 0)}/{extension_support_surface.get('required_profiles', 0)} declared."
    )
    if release_ops_execution.get("status"):
        summary += (
            " release_ops_execution="
            f"{_format_release_ops_execution_component_status(release_ops_execution)}."
        )
    if missing_required_documents:
        summary += " Missing required docs: " + ", ".join(missing_required_documents) + "."
    summary = _append_external_mainline_control_plane_summary(
        summary,
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )

    payload = {
        "status": status,
        "summary": summary,
        "required_documents": required_documents,
        "required_documents_ready": required_documents_ready,
        "phase_e_documents": len(CUSTOMER_DELIVERY_PHASE_E_FOCUS_DOCUMENTS),
        "phase_e_documents_ready": phase_e_documents_ready,
        **attachment_flags,
        "extension_support_surface": to_jsonable(extension_support_surface),
        "missing_required_documents": missing_required_documents,
        "missing_phase_e_documents": missing_phase_e_documents,
        "phase_e_focus_documents": to_jsonable(phase_e_focus_documents),
        "documents": to_jsonable(hydrated_documents),
    }
    if release_ops_execution:
        payload["release_ops_execution"] = to_jsonable(release_ops_execution)
    if control_plane_session:
        payload["control_plane_session"] = to_jsonable(control_plane_session)
    if control_plane_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(control_plane_event_stream)
    return payload


def validate_customer_delivery_surface(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["customer_delivery_surface must be an object"]

    errors: list[str] = []
    missing = sorted(CUSTOMER_DELIVERY_SURFACE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "customer_delivery_surface missing required fields: " + ", ".join(missing)
        )

    if payload.get("status") not in CUSTOMER_DELIVERY_SURFACE_STATUSES:
        errors.append(
            "customer_delivery_surface.status must be one of "
            f"{sorted(CUSTOMER_DELIVERY_SURFACE_STATUSES)}"
        )
    if "summary" in payload and not _is_non_empty_string(payload.get("summary")):
        errors.append("customer_delivery_surface.summary must be a non-empty string")

    for field in [
        "required_documents",
        "required_documents_ready",
        "phase_e_documents",
        "phase_e_documents_ready",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(f"customer_delivery_surface.{field} must be a non-negative integer")
    if (
        _is_non_negative_int(payload.get("required_documents"))
        and _is_non_negative_int(payload.get("required_documents_ready"))
        and payload.get("required_documents_ready") > payload.get("required_documents")
    ):
        errors.append(
            "customer_delivery_surface.required_documents_ready must be <= required_documents"
        )
    if (
        _is_non_negative_int(payload.get("phase_e_documents"))
        and _is_non_negative_int(payload.get("phase_e_documents_ready"))
        and payload.get("phase_e_documents_ready") > payload.get("phase_e_documents")
    ):
        errors.append(
            "customer_delivery_surface.phase_e_documents_ready must be <= phase_e_documents"
        )

    for field in [
        "support_matrix_attached",
        "capacity_declaration_attached",
        "customer_acceptance_checklist_attached",
        "known_limitations_attached",
    ]:
        if field in payload and not isinstance(payload.get(field), bool):
            errors.append(f"customer_delivery_surface.{field} must be a boolean")

    for list_name in ["missing_required_documents", "missing_phase_e_documents"]:
        items = payload.get(list_name)
        if not isinstance(items, list):
            errors.append(f"customer_delivery_surface.{list_name} must be a list")
            continue
        for index, item in enumerate(items, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    f"customer_delivery_surface.{list_name}[{index}] must be a non-empty string"
                )

    errors.extend(
        _validate_customer_acceptance_items_list(
            "customer_delivery_surface.phase_e_focus_documents",
            payload.get("phase_e_focus_documents"),
        )
    )
    errors.extend(
        validate_extension_support_surface(payload.get("extension_support_surface"))
    )
    errors.extend(
        _validate_customer_acceptance_items_list(
            "customer_delivery_surface.documents",
            payload.get("documents"),
        )
    )
    release_ops_execution = payload.get("release_ops_execution")
    if release_ops_execution is not None:
        if not isinstance(release_ops_execution, Mapping):
            errors.append("customer_delivery_surface.release_ops_execution must be an object")
        else:
            status = release_ops_execution.get("status")
            if status not in RELEASE_EVIDENCE_STATUSES:
                errors.append(
                    "customer_delivery_surface.release_ops_execution.status must be one of "
                    f"{sorted(RELEASE_EVIDENCE_STATUSES)}"
                )
            if "summary" in release_ops_execution and not _is_non_empty_string(
                release_ops_execution.get("summary")
            ):
                errors.append(
                    "customer_delivery_surface.release_ops_execution.summary must be a non-empty string"
                )
            if "metrics" in release_ops_execution and not isinstance(
                release_ops_execution.get("metrics"), Mapping
            ):
                errors.append(
                    "customer_delivery_surface.release_ops_execution.metrics must be an object"
                )
    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append("customer_delivery_surface.control_plane_session must be an object")
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        "customer_delivery_surface.control_plane_session."
                        f"{field} must be a non-empty string when present"
                    )
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append(
                "customer_delivery_surface.control_plane_event_stream must be an object"
            )
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    "customer_delivery_surface.control_plane_event_stream.path must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    "customer_delivery_surface.control_plane_event_stream.event_count must be a non-negative integer when present"
                )
    return errors


def build_industrial_delivery_gate(
    *,
    project_root: str | Path | None = None,
    test_evidence: Sequence[Mapping[str, Any]] | None = None,
    customer_delivery_surface: Mapping[str, Any] | None = None,
    acceptance_documents: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    evidence = [
        dict(item)
        for item in (
            test_evidence
            if test_evidence is not None
            else hydrate_release_test_evidence(project_root=resolved_root)
        )
    ]
    customer_delivery_payload = dict(
        customer_delivery_surface
        if customer_delivery_surface is not None
        else build_customer_delivery_surface(
            project_root=resolved_root,
            acceptance_documents=acceptance_documents,
        )
    )
    required_docs = acceptance_documents or default_customer_acceptance_documents()
    deployment_documents = [
        dict(item)
        for item in required_docs
        if str(item.get("name")) in INDUSTRIAL_DELIVERY_DEPLOYMENT_DOCUMENT_NAMES
    ]
    hydrated_deployment_documents = _hydrate_customer_acceptance_items(
        deployment_documents,
        resolved_root,
    )
    security_reports = _hydrate_customer_acceptance_items(
        default_industrial_delivery_reports(),
        resolved_root,
    )
    return _build_industrial_delivery_gate_payload(
        test_evidence=evidence,
        customer_delivery_surface=customer_delivery_payload,
        deployment_documents=hydrated_deployment_documents,
        security_reports=security_reports,
    )


def validate_industrial_delivery_gate(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["industrial_delivery_gate must be an object"]

    errors: list[str] = []
    missing = sorted(INDUSTRIAL_DELIVERY_GATE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(
            "industrial_delivery_gate missing required fields: " + ", ".join(missing)
        )

    if payload.get("status") not in INDUSTRIAL_DELIVERY_GATE_STATUSES:
        errors.append(
            "industrial_delivery_gate.status must be one of "
            f"{sorted(INDUSTRIAL_DELIVERY_GATE_STATUSES)}"
        )
    if payload.get("deployment_package_status") not in INDUSTRIAL_DELIVERY_COMPONENT_STATUSES:
        errors.append(
            "industrial_delivery_gate.deployment_package_status must be one of "
            f"{sorted(INDUSTRIAL_DELIVERY_COMPONENT_STATUSES)}"
        )
    if "summary" in payload and not _is_non_empty_string(payload.get("summary")):
        errors.append("industrial_delivery_gate.summary must be a non-empty string")

    for field in [
        "required_evidence",
        "attested_required_evidence",
        "required_deployment_documents",
        "ready_deployment_documents",
    ]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(f"industrial_delivery_gate.{field} must be a non-negative integer")
    if (
        _is_non_negative_int(payload.get("required_evidence"))
        and _is_non_negative_int(payload.get("attested_required_evidence"))
        and payload.get("attested_required_evidence") > payload.get("required_evidence")
    ):
        errors.append(
            "industrial_delivery_gate.attested_required_evidence must be <= required_evidence"
        )
    if (
        _is_non_negative_int(payload.get("required_deployment_documents"))
        and _is_non_negative_int(payload.get("ready_deployment_documents"))
        and payload.get("ready_deployment_documents")
        > payload.get("required_deployment_documents")
    ):
        errors.append(
            "industrial_delivery_gate.ready_deployment_documents must be <= required_deployment_documents"
        )

    for field in [
        "evidence_attested",
        "sbom_attached",
        "backup_restore_verified",
        "support_matrix_attached",
        "capacity_declaration_attached",
        "customer_acceptance_checklist_attached",
        "known_limitations_attached",
    ]:
        if field in payload and not isinstance(payload.get(field), bool):
            errors.append(f"industrial_delivery_gate.{field} must be a boolean")

    if payload.get("vuln_scan_status") not in VULNERABILITY_SCAN_STATUSES:
        errors.append(
            "industrial_delivery_gate.vuln_scan_status must be one of "
            f"{sorted(VULNERABILITY_SCAN_STATUSES)}"
        )
    if payload.get("customer_delivery_surface_status") not in CUSTOMER_DELIVERY_SURFACE_STATUSES:
        errors.append(
            "industrial_delivery_gate.customer_delivery_surface_status must be one of "
            f"{sorted(CUSTOMER_DELIVERY_SURFACE_STATUSES)}"
        )
    if payload.get("extension_support_surface_status") not in EXTENSION_SUPPORT_SURFACE_STATUSES:
        errors.append(
            "industrial_delivery_gate.extension_support_surface_status must be one of "
            f"{sorted(EXTENSION_SUPPORT_SURFACE_STATUSES)}"
        )
    for field in ["required_extension_profiles", "declared_extension_profiles"]:
        if field in payload and not _is_non_negative_int(payload.get(field)):
            errors.append(f"industrial_delivery_gate.{field} must be a non-negative integer")
    if (
        _is_non_negative_int(payload.get("required_extension_profiles"))
        and _is_non_negative_int(payload.get("declared_extension_profiles"))
        and payload.get("declared_extension_profiles")
        > payload.get("required_extension_profiles")
    ):
        errors.append(
            "industrial_delivery_gate.declared_extension_profiles must be <= required_extension_profiles"
        )

    missing_requirements = payload.get("missing_requirements")
    if not isinstance(missing_requirements, list):
        errors.append("industrial_delivery_gate.missing_requirements must be a list")
    else:
        for index, item in enumerate(missing_requirements, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    f"industrial_delivery_gate.missing_requirements[{index}] must be a non-empty string"
                )

    errors.extend(
        _validate_customer_acceptance_items_list(
            "industrial_delivery_gate.deployment_documents",
            payload.get("deployment_documents"),
        )
    )
    errors.extend(
        _validate_customer_acceptance_items_list(
            "industrial_delivery_gate.security_reports",
            payload.get("security_reports"),
        )
    )
    release_ops_execution = payload.get("release_ops_execution")
    if not isinstance(release_ops_execution, Mapping):
        errors.append("industrial_delivery_gate.release_ops_execution must be an object")
    else:
        status = release_ops_execution.get("status")
        if status not in RELEASE_EVIDENCE_STATUSES:
            errors.append(
                "industrial_delivery_gate.release_ops_execution.status must be one of "
                f"{sorted(RELEASE_EVIDENCE_STATUSES)}"
            )
        if "summary" in release_ops_execution and not _is_non_empty_string(
            release_ops_execution.get("summary")
        ):
            errors.append(
                "industrial_delivery_gate.release_ops_execution.summary must be a non-empty string"
            )
        if "metrics" in release_ops_execution and not isinstance(
            release_ops_execution.get("metrics"), Mapping
        ):
            errors.append(
                "industrial_delivery_gate.release_ops_execution.metrics must be an object"
            )
    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append("industrial_delivery_gate.control_plane_session must be an object")
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        "industrial_delivery_gate.control_plane_session."
                        f"{field} must be a non-empty string when present"
                    )
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append(
                "industrial_delivery_gate.control_plane_event_stream must be an object"
            )
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    "industrial_delivery_gate.control_plane_event_stream.path must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    "industrial_delivery_gate.control_plane_event_stream.event_count must be a non-negative integer when present"
                )
    return errors


def hydrate_release_test_evidence(
    test_evidence: Sequence[Mapping[str, Any]] | None = None,
    *,
    project_root: str | Path | None = None,
) -> list[dict[str, Any]]:
    """Load known artifact reports and reflect their current evidence status."""
    base_evidence = [dict(item) for item in (test_evidence or default_release_test_evidence())]
    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    hydrated: list[dict[str, Any]] = []

    for item in base_evidence:
        current = dict(item)
        current["attested"] = False
        artifact_path = current.get("artifact_path")
        if _is_non_empty_string(artifact_path):
            resolved_path = _resolve_release_artifact_path(artifact_path, resolved_root)
            current["resolved_artifact_path"] = str(resolved_path)
            current["artifact_found"] = resolved_path.exists()
            if resolved_path.is_file():
                current.update(
                    _load_release_evidence_from_report(current["name"], resolved_path)
                )
        hydrated.append(current)

    return hydrated


def apply_release_test_evidence_to_capability_matrix(
    capability_matrix: Mapping[str, Any] | None = None,
    test_evidence: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    """Adjust the capability matrix when optional live evidence is available."""
    matrix = json.loads(
        json.dumps(
            to_jsonable(
                dict(capability_matrix)
                if capability_matrix is not None
                else build_capability_matrix_artifact()
            ),
            ensure_ascii=False,
        )
    )
    evidence_by_name = {item.get("name"): item for item in (test_evidence or [])}
    domains = matrix.get("domains", [])

    distributed_evidence = evidence_by_name.get("distributed_runtime_live", {})
    if distributed_evidence.get("status") == "passed":
        for domain in domains:
            if domain.get("id") != "distributed_runtime":
                continue
            domain["status"] = "ready"
            domain["summary"] = (
                "Compose entrypoints, smoke report, monitor schema, and live distributed smoke evidence are all available."
            )
            domain["known_limitations"] = [
                item
                for item in domain.get("known_limitations", [])
                if item != DISTRIBUTED_DOMAIN_LIMITATION
            ]
            verification = dict(domain.get("verification", {}))
            verification["live_evidence_status"] = "passed"
            verification["live_evidence_source"] = distributed_evidence.get(
                "resolved_artifact_path"
            ) or distributed_evidence.get("artifact_path")
            domain["verification"] = verification

        matrix["known_limitations"] = [
            item
            for item in matrix.get("known_limitations", [])
            if item != DISTRIBUTED_RELEASE_LIMITATION
        ]

    matrix["summary"] = _build_capability_summary(domains)
    return matrix


def build_release_manifest_artifact(
    *,
    build_id: str,
    version: str,
    channel: str,
    release_summary: str,
    changelog_path: str = "RELEASE_NOTES.md",
    changelog_title: str = "AGI-Walker Release Notes",
    generated_at: str | None = None,
    contract_versions: Sequence[Mapping[str, Any]] | None = None,
    capability_matrix: Mapping[str, Any] | None = None,
    test_evidence: Sequence[Mapping[str, Any]] | None = None,
    release_approval: Mapping[str, Any] | None = None,
    release_source: Mapping[str, Any] | None = None,
    known_limitations: Sequence[str] | None = None,
    customer_delivery_surface: Mapping[str, Any] | None = None,
    industrial_delivery_gate: Mapping[str, Any] | None = None,
    extension_execution_instance: Mapping[str, Any] | None = None,
    extension_execution_schedule: Mapping[str, Any] | None = None,
    extension_execution_actuals: Mapping[str, Any] | None = None,
    project_root: str | Path | None = None,
    source_root: str | Path | None = None,
) -> dict[str, Any]:
    evidence = hydrate_release_test_evidence(test_evidence, project_root=project_root)
    release_policy = _build_release_policy(channel)
    release_source_payload = _build_release_source(
        version=version,
        release_source=release_source,
        source_root=source_root,
    )
    approval_payload = _build_release_approval(
        channel,
        release_approval,
        release_source=release_source_payload,
    )
    matrix_payload = apply_release_test_evidence_to_capability_matrix(
        to_jsonable(
            dict(capability_matrix)
            if capability_matrix is not None
            else build_capability_matrix_artifact(generated_at=generated_at)
        ),
        evidence,
    )
    limitations = list(
        known_limitations
        if known_limitations is not None
        else _default_known_limitations(matrix_payload)
    )
    customer_delivery_payload = dict(
        customer_delivery_surface
        if customer_delivery_surface is not None
        else build_customer_delivery_surface(project_root=project_root)
    )
    industrial_delivery_payload = dict(
        industrial_delivery_gate
        if industrial_delivery_gate is not None
        else build_industrial_delivery_gate(
            project_root=project_root,
            test_evidence=evidence,
            customer_delivery_surface=customer_delivery_payload,
        )
    )
    release_ops_execution_payload: dict[str, Any] = {}
    for component_source in [
        industrial_delivery_payload.get("release_ops_execution"),
        customer_delivery_payload.get("release_ops_execution"),
    ]:
        if not isinstance(component_source, Mapping):
            continue
        _hydrate_release_ops_execution_component(
            release_ops_execution_payload,
            component_source=component_source,
            bundle_path=None,
        )
        if _is_non_empty_string(release_ops_execution_payload.get("status")) and _is_non_empty_string(
            release_ops_execution_payload.get("summary")
        ):
            break
    if not _is_non_empty_string(release_ops_execution_payload.get("status")):
        release_ops_execution_payload = {
            "status": "blocked",
            "summary": "release ops execution is unavailable for the release manifest.",
        }
    control_plane_session, control_plane_event_stream = (
        _aggregate_release_op_control_plane_surface(
            release_ops_execution_payload,
            customer_delivery_payload,
            industrial_delivery_payload,
        )
    )
    control_plane_surface = _build_control_plane_surface_payload(
        release_ops_execution=release_ops_execution_payload,
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )
    extension_execution_evidence_payload = build_extension_execution_evidence(
        project_root=project_root,
        extension_support_surface=customer_delivery_payload.get(
            "extension_support_surface",
            {},
        )
        if isinstance(customer_delivery_payload, Mapping)
        else {},
    )
    extension_execution_instance_payload = build_extension_execution_instance(
        project_root=project_root,
        extension_support_surface=customer_delivery_payload.get(
            "extension_support_surface",
            {},
        )
        if isinstance(customer_delivery_payload, Mapping)
        else {},
        instance_artifact=extension_execution_instance,
    )
    extension_execution_schedule_payload = build_extension_execution_schedule(
        project_root=project_root,
        extension_support_surface=customer_delivery_payload.get(
            "extension_support_surface",
            {},
        )
        if isinstance(customer_delivery_payload, Mapping)
        else {},
        schedule_artifact=extension_execution_schedule,
    )
    extension_execution_actuals_payload = build_extension_execution_actuals(
        project_root=project_root,
        extension_support_surface=customer_delivery_payload.get(
            "extension_support_surface",
            {},
        )
        if isinstance(customer_delivery_payload, Mapping)
        else {},
        actuals_artifact=extension_execution_actuals,
        schedule_artifact=extension_execution_schedule_payload,
    )
    gate = _build_release_gate(
        evidence,
        matrix_payload,
        approval_payload,
        release_source_payload,
        release_policy,
        customer_delivery_payload,
        industrial_delivery_payload,
    )
    payload = {
        "schema_version": RELEASE_CONTRACT_VERSION,
        "artifact_type": RELEASE_ARTIFACT_TYPE,
        "build_id": build_id,
        "version": version,
        "channel": channel,
        "release_policy": to_jsonable(release_policy),
        "release_approval": to_jsonable(approval_payload),
        "release_source": to_jsonable(release_source_payload),
        "release_summary": release_summary,
        "generated_at": generated_at or datetime.now().isoformat(),
        "release_gate_status": _resolve_release_gate_status(gate, release_policy),
        "release_gate": gate,
        "changelog": {
            "path": changelog_path,
            "title": changelog_title,
        },
        "contract_versions": to_jsonable(
            [
                dict(item)
                for item in (contract_versions or default_release_contract_versions())
            ]
        ),
        "capability_matrix": to_jsonable(matrix_payload),
        "test_evidence": to_jsonable(evidence),
        "known_limitations": to_jsonable(limitations),
        "customer_delivery_surface": to_jsonable(customer_delivery_payload),
        "industrial_delivery_gate": to_jsonable(industrial_delivery_payload),
        "release_ops_execution": to_jsonable(release_ops_execution_payload),
        "control_plane_surface": to_jsonable(control_plane_surface),
        "extension_execution_evidence": to_jsonable(
            extension_execution_evidence_payload
        ),
        "extension_execution_instance": to_jsonable(
            extension_execution_instance_payload
        ),
        "extension_execution_schedule": to_jsonable(
            extension_execution_schedule_payload
        ),
        "extension_execution_actuals": to_jsonable(
            extension_execution_actuals_payload
        ),
    }
    if control_plane_session:
        payload["control_plane_session"] = to_jsonable(control_plane_session)
    if control_plane_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(control_plane_event_stream)
    return payload


def validate_release_manifest_artifact(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["release manifest must be an object"]

    errors: list[str] = []
    missing = sorted(RELEASE_MANIFEST_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != RELEASE_CONTRACT_VERSION:
        errors.append(
            f"schema_version must be {RELEASE_CONTRACT_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != RELEASE_ARTIFACT_TYPE:
        errors.append(f"artifact_type must be {RELEASE_ARTIFACT_TYPE!r}")

    for key in ["build_id", "version", "channel", "release_summary", "generated_at"]:
        if key in payload and not _is_non_empty_string(payload.get(key)):
            errors.append(f"{key} must be a non-empty string")

    if payload.get("channel") not in RELEASE_CHANNELS:
        errors.append(f"channel must be one of {sorted(RELEASE_CHANNELS)}")
    if payload.get("release_gate_status") not in RELEASE_GATE_STATUSES:
        errors.append(
            f"release_gate_status must be one of {sorted(RELEASE_GATE_STATUSES)}"
        )

    release_policy = payload.get("release_policy")
    if not isinstance(release_policy, Mapping):
        errors.append("release_policy must be an object")
    else:
        missing_policy = sorted(RELEASE_POLICY_REQUIRED_FIELDS - set(release_policy))
        if missing_policy:
            errors.append(
                f"release_policy missing required fields: {', '.join(missing_policy)}"
            )
        if release_policy.get("channel") != payload.get("channel"):
            errors.append(
                f"release_policy.channel must match channel {payload.get('channel')!r}"
            )
        for field in ["allows_opt_in_evidence", "allows_diagnostic_ready_domains"]:
            if field in release_policy and not isinstance(release_policy.get(field), bool):
                errors.append(f"release_policy.{field} must be a boolean")
        if "requires_release_approval" in release_policy and not isinstance(
            release_policy.get("requires_release_approval"), bool
        ):
            errors.append("release_policy.requires_release_approval must be a boolean")
        if "requires_git_source_binding" in release_policy and not isinstance(
            release_policy.get("requires_git_source_binding"), bool
        ):
            errors.append("release_policy.requires_git_source_binding must be a boolean")
        if "requires_clean_worktree" in release_policy and not isinstance(
            release_policy.get("requires_clean_worktree"), bool
        ):
            errors.append("release_policy.requires_clean_worktree must be a boolean")
        if "requires_version_tag_match" in release_policy and not isinstance(
            release_policy.get("requires_version_tag_match"), bool
        ):
            errors.append("release_policy.requires_version_tag_match must be a boolean")
        if "requires_customer_delivery_surface" in release_policy and not isinstance(
            release_policy.get("requires_customer_delivery_surface"), bool
        ):
            errors.append(
                "release_policy.requires_customer_delivery_surface must be a boolean"
            )
        if "requires_industrial_delivery_gate" in release_policy and not isinstance(
            release_policy.get("requires_industrial_delivery_gate"), bool
        ):
            errors.append(
                "release_policy.requires_industrial_delivery_gate must be a boolean"
            )
        if "summary" in release_policy and not _is_non_empty_string(
            release_policy.get("summary")
        ):
            errors.append("release_policy.summary must be a non-empty string")

    release_approval = payload.get("release_approval")
    if not isinstance(release_approval, Mapping):
        errors.append("release_approval must be an object")
    else:
        missing_approval = sorted(RELEASE_APPROVAL_REQUIRED_FIELDS - set(release_approval))
        if missing_approval:
            errors.append(
                f"release_approval missing required fields: {', '.join(missing_approval)}"
            )
        if release_approval.get("status") not in RELEASE_APPROVAL_STATUSES:
            errors.append(
                f"release_approval.status must be one of {sorted(RELEASE_APPROVAL_STATUSES)}"
            )
        if "required" in release_approval and not isinstance(
            release_approval.get("required"), bool
        ):
            errors.append("release_approval.required must be a boolean")
        for field in ["approved_by", "approved_at", "commit_sha", "notes"]:
            if field in release_approval and release_approval.get(field) is not None:
                if not _is_non_empty_string(release_approval.get(field)):
                    errors.append(
                        f"release_approval.{field} must be null or a non-empty string"
                    )
        if release_approval.get("status") == "approved":
            for field in ["approved_by", "approved_at", "commit_sha"]:
                if not _is_non_empty_string(release_approval.get(field)):
                    errors.append(
                        f"release_approval.{field} is required when status is 'approved'"
                    )

    release_source = payload.get("release_source")
    if not isinstance(release_source, Mapping):
        errors.append("release_source must be an object")
    else:
        missing_source = sorted(RELEASE_SOURCE_REQUIRED_FIELDS - set(release_source))
        if missing_source:
            errors.append(
                f"release_source missing required fields: {', '.join(missing_source)}"
            )
        if "resolved_from_git" in release_source and not isinstance(
            release_source.get("resolved_from_git"), bool
        ):
            errors.append("release_source.resolved_from_git must be a boolean")
        for field in ["commit_sha", "short_commit_sha", "git_tag", "matched_version_tag"]:
            if field in release_source and release_source.get(field) is not None:
                if not _is_non_empty_string(release_source.get(field)):
                    errors.append(
                        f"release_source.{field} must be null or a non-empty string"
                    )
        if "worktree_clean" in release_source and not isinstance(
            release_source.get("worktree_clean"), bool
        ):
            errors.append("release_source.worktree_clean must be a boolean")
        if (
            "worktree_status_summary" in release_source
            and release_source.get("worktree_status_summary") is not None
            and not _is_non_empty_string(release_source.get("worktree_status_summary"))
        ):
            errors.append(
                "release_source.worktree_status_summary must be null or a non-empty string"
            )
        if "version_tag_matches" in release_source and not isinstance(
            release_source.get("version_tag_matches"), bool
        ):
            errors.append("release_source.version_tag_matches must be a boolean")
        if release_source.get("resolved_from_git"):
            for field in ["commit_sha", "short_commit_sha"]:
                if not _is_non_empty_string(release_source.get(field)):
                    errors.append(
                        f"release_source.{field} is required when resolved_from_git is true"
                    )

    changelog = payload.get("changelog")
    if not isinstance(changelog, Mapping):
        errors.append("changelog must be an object")
    else:
        missing_changelog = sorted(RELEASE_CHANGELOG_REQUIRED_FIELDS - set(changelog))
        if missing_changelog:
            errors.append(
                f"changelog missing required fields: {', '.join(missing_changelog)}"
            )
        for field in RELEASE_CHANGELOG_REQUIRED_FIELDS:
            if field in changelog and not _is_non_empty_string(changelog.get(field)):
                errors.append(f"changelog.{field} must be a non-empty string")

    gate = payload.get("release_gate")
    if not isinstance(gate, Mapping):
        errors.append("release_gate must be an object")
    else:
        missing_gate = sorted(RELEASE_GATE_REQUIRED_FIELDS - set(gate))
        if missing_gate:
            errors.append(
                f"release_gate missing required fields: {', '.join(missing_gate)}"
            )
        for field in RELEASE_GATE_REQUIRED_FIELDS:
            if field in gate and not _is_non_negative_int(gate.get(field)):
                errors.append(f"release_gate.{field} must be a non-negative integer")

    contract_versions = payload.get("contract_versions")
    if not isinstance(contract_versions, list):
        errors.append("contract_versions must be a list")
    else:
        for index, item in enumerate(contract_versions, start=1):
            prefix = f"contract_versions[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            missing_fields = sorted(RELEASE_CONTRACT_VERSION_REQUIRED_FIELDS - set(item))
            if missing_fields:
                errors.append(
                    f"{prefix} missing required fields: {', '.join(missing_fields)}"
                )
            for field in RELEASE_CONTRACT_VERSION_REQUIRED_FIELDS:
                if field in item and not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")

    matrix = payload.get("capability_matrix")
    if not isinstance(matrix, Mapping):
        errors.append("capability_matrix must be an object")
    else:
        errors.extend(validate_capability_matrix_artifact(matrix))

    evidence = payload.get("test_evidence")
    if not isinstance(evidence, list):
        errors.append("test_evidence must be a list")
        evidence = []
    else:
        for index, item in enumerate(evidence, start=1):
            prefix = f"test_evidence[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            missing_fields = sorted(RELEASE_TEST_EVIDENCE_REQUIRED_FIELDS - set(item))
            if missing_fields:
                errors.append(
                    f"{prefix} missing required fields: {', '.join(missing_fields)}"
                )
            for field in ["name", "status", "summary", "command"]:
                if field in item and not _is_non_empty_string(item.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")
            if "required" in item and not isinstance(item.get("required"), bool):
                errors.append(f"{prefix}.required must be a boolean")
            if item.get("status") not in RELEASE_EVIDENCE_STATUSES:
                errors.append(
                    f"{prefix}.status must be one of {sorted(RELEASE_EVIDENCE_STATUSES)}"
                )
            if "artifact_path" in item and item.get("artifact_path") is not None:
                if not _is_non_empty_string(item.get("artifact_path")):
                    errors.append(f"{prefix}.artifact_path must be null or a non-empty string")

    limitations = payload.get("known_limitations")
    if not isinstance(limitations, list):
        errors.append("known_limitations must be a list")
        limitations = []
    else:
        for index, item in enumerate(limitations, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    f"known_limitations[{index}] must be a non-empty string"
                )

    customer_delivery_surface = payload.get("customer_delivery_surface")
    errors.extend(validate_customer_delivery_surface(customer_delivery_surface))
    industrial_delivery_gate = payload.get("industrial_delivery_gate")
    errors.extend(validate_industrial_delivery_gate(industrial_delivery_gate))
    release_ops_execution = payload.get("release_ops_execution")
    if not isinstance(release_ops_execution, Mapping):
        errors.append("release_ops_execution must be an object")
    else:
        status = release_ops_execution.get("status")
        if status not in RELEASE_EVIDENCE_STATUSES:
            errors.append(
                "release_ops_execution.status must be one of "
                f"{sorted(RELEASE_EVIDENCE_STATUSES)}"
            )
        if "summary" in release_ops_execution and not _is_non_empty_string(
            release_ops_execution.get("summary")
        ):
            errors.append("release_ops_execution.summary must be a non-empty string")
        if "metrics" in release_ops_execution and not isinstance(
            release_ops_execution.get("metrics"), Mapping
        ):
            errors.append("release_ops_execution.metrics must be an object")
    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append("control_plane_session must be an object")
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        f"control_plane_session.{field} must be a non-empty string when present"
                    )
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append("control_plane_event_stream must be an object")
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    "control_plane_event_stream.path must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    "control_plane_event_stream.event_count must be a non-negative integer when present"
                )
    control_plane_surface = payload.get("control_plane_surface")
    errors.extend(
        _validate_control_plane_surface(
            control_plane_surface,
            prefix="control_plane_surface",
        )
    )
    extension_execution_evidence = payload.get("extension_execution_evidence")
    errors.extend(
        validate_extension_execution_evidence(extension_execution_evidence)
    )
    extension_execution_instance = payload.get("extension_execution_instance")
    errors.extend(
        validate_extension_execution_instance(extension_execution_instance)
    )
    extension_execution_schedule = payload.get("extension_execution_schedule")
    errors.extend(
        validate_extension_execution_schedule(extension_execution_schedule)
    )
    extension_execution_actuals = payload.get("extension_execution_actuals")
    errors.extend(
        validate_extension_execution_actuals(extension_execution_actuals)
    )

    if (
        not errors
        and isinstance(gate, Mapping)
        and isinstance(matrix, Mapping)
        and isinstance(release_policy, Mapping)
        and isinstance(release_approval, Mapping)
        and isinstance(release_source, Mapping)
        and isinstance(customer_delivery_surface, Mapping)
        and isinstance(industrial_delivery_gate, Mapping)
        and isinstance(extension_execution_instance, Mapping)
        and isinstance(extension_execution_schedule, Mapping)
        and isinstance(extension_execution_actuals, Mapping)
    ):
        expected_gate = _build_release_gate(
            evidence,
            matrix,
            release_approval,
            release_source,
            release_policy,
            customer_delivery_surface,
            industrial_delivery_gate,
        )
        for key, value in expected_gate.items():
            if gate.get(key) != value:
                errors.append(
                    f"release_gate.{key} must be {value!r}, got {gate.get(key)!r}"
                )
        expected_policy = _build_release_policy(str(payload.get("channel")))
        if dict(release_policy) != expected_policy:
            errors.append(
                f"release_policy must be {expected_policy!r}, got {dict(release_policy)!r}"
            )
        expected_approval = _build_release_approval(
            str(payload.get("channel")), release_approval
        )
        if dict(release_approval) != expected_approval:
            errors.append(
                f"release_approval must be {expected_approval!r}, got {dict(release_approval)!r}"
            )
        expected_status = _resolve_release_gate_status(expected_gate, expected_policy)
        if payload.get("release_gate_status") != expected_status:
            errors.append(
                f"release_gate_status must be {expected_status!r}, got {payload.get('release_gate_status')!r}"
            )
        expected_industrial_delivery_gate = _build_industrial_delivery_gate_payload(
            test_evidence=evidence,
            customer_delivery_surface=customer_delivery_surface,
            deployment_documents=industrial_delivery_gate.get("deployment_documents", []),
            security_reports=industrial_delivery_gate.get("security_reports", []),
        )
        for key, value in expected_industrial_delivery_gate.items():
            if industrial_delivery_gate.get(key) != value:
                errors.append(
                    "industrial_delivery_gate."
                    f"{key} must be {value!r}, got {industrial_delivery_gate.get(key)!r}"
                )
        expected_control_plane_surface = _build_control_plane_surface_payload(
            release_ops_execution=release_ops_execution,
            control_plane_session=payload.get("control_plane_session")
            if isinstance(payload.get("control_plane_session"), Mapping)
            else None,
            control_plane_event_stream=payload.get("control_plane_event_stream")
            if isinstance(payload.get("control_plane_event_stream"), Mapping)
            else None,
        )
        if isinstance(control_plane_surface, Mapping):
            for key, value in expected_control_plane_surface.items():
                if control_plane_surface.get(key) != value:
                    errors.append(
                        "control_plane_surface."
                        f"{key} must be {value!r}, got {control_plane_surface.get(key)!r}"
                    )
        expected_extension_execution_evidence = _build_extension_execution_evidence_payload(
            extension_support_surface=customer_delivery_surface.get(
                "extension_support_surface",
                {},
            )
            if isinstance(customer_delivery_surface.get("extension_support_surface"), Mapping)
            else {},
            reports=extension_execution_evidence.get("reports", [])
            if isinstance(extension_execution_evidence, Mapping)
            else [],
        )
        if isinstance(extension_execution_evidence, Mapping):
            for key, value in expected_extension_execution_evidence.items():
                if extension_execution_evidence.get(key) != value:
                    errors.append(
                        "extension_execution_evidence."
                        f"{key} must be {value!r}, got {extension_execution_evidence.get(key)!r}"
                    )
        expected_extension_execution_instance = _build_extension_execution_instance_payload(
            extension_support_surface=customer_delivery_surface.get(
                "extension_support_surface",
                {},
            )
            if isinstance(customer_delivery_surface.get("extension_support_surface"), Mapping)
            else {},
            artifact_path=str(
                extension_execution_instance.get("artifact_path")
                or default_extension_execution_instance_artifact_path()
            ),
            exists=extension_execution_instance.get("exists") is True,
            instance=extension_execution_instance,
        )
        for key, value in expected_extension_execution_instance.items():
            if extension_execution_instance.get(key) != value:
                errors.append(
                    "extension_execution_instance."
                    f"{key} must be {value!r}, got {extension_execution_instance.get(key)!r}"
                )
        expected_extension_execution_schedule = _build_extension_execution_schedule_payload(
            extension_support_surface=customer_delivery_surface.get(
                "extension_support_surface",
                {},
            )
            if isinstance(customer_delivery_surface.get("extension_support_surface"), Mapping)
            else {},
            artifact_path=str(
                extension_execution_schedule.get("artifact_path")
                or default_extension_execution_schedule_artifact_path()
            ),
            exists=extension_execution_schedule.get("exists") is True,
            schedule=extension_execution_schedule,
        )
        for key, value in expected_extension_execution_schedule.items():
            if extension_execution_schedule.get(key) != value:
                errors.append(
                    "extension_execution_schedule."
                    f"{key} must be {value!r}, got {extension_execution_schedule.get(key)!r}"
                )
        expected_extension_execution_actuals = _build_extension_execution_actuals_payload(
            extension_support_surface=customer_delivery_surface.get(
                "extension_support_surface",
                {},
            )
            if isinstance(customer_delivery_surface.get("extension_support_surface"), Mapping)
            else {},
            artifact_path=str(
                extension_execution_actuals.get("artifact_path")
                or default_extension_execution_actuals_artifact_path()
            ),
            exists=extension_execution_actuals.get("exists") is True,
            actuals=extension_execution_actuals,
            schedule=extension_execution_schedule,
        )
        for key, value in expected_extension_execution_actuals.items():
            if extension_execution_actuals.get(key) != value:
                errors.append(
                    "extension_execution_actuals."
                    f"{key} must be {value!r}, got {extension_execution_actuals.get(key)!r}"
                )

    return errors


def write_release_manifest_artifact(payload: Mapping[str, Any], path: str | Path) -> Path:
    errors = validate_release_manifest_artifact(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _build_industrial_delivery_gate_payload(
    *,
    test_evidence: Sequence[Mapping[str, Any]],
    customer_delivery_surface: Mapping[str, Any],
    deployment_documents: Sequence[Mapping[str, Any]],
    security_reports: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    required_evidence_items = [
        dict(item)
        for item in test_evidence
        if isinstance(item, Mapping) and item.get("required") is True
    ]
    required_evidence = len(required_evidence_items)
    attested_required_evidence = sum(
        1
        for item in required_evidence_items
        if item.get("status") == "passed" and item.get("attested") is True
    )
    evidence_attested = (
        required_evidence > 0 and attested_required_evidence == required_evidence
    )

    hydrated_deployment_documents = [
        dict(item) for item in deployment_documents if isinstance(item, Mapping)
    ]
    required_deployment_documents = sum(
        1 for item in hydrated_deployment_documents if item.get("required") is True
    )
    ready_deployment_documents = sum(
        1
        for item in hydrated_deployment_documents
        if item.get("required") is True and item.get("exists") is True
    )
    deployment_package_status = (
        "ready"
        if required_deployment_documents == ready_deployment_documents
        else "blocked"
    )

    hydrated_security_reports = [
        dict(item) for item in security_reports if isinstance(item, Mapping)
    ]
    reports_by_name = {
        str(item.get("name")): item
        for item in hydrated_security_reports
        if _is_non_empty_string(item.get("name"))
    }
    security_posture_report = reports_by_name.get("security_posture_report", {})
    remediation_report = reports_by_name.get("vulnerability_remediation_report", {})
    sbom_report = reports_by_name.get("sbom_artifact", {})
    sbom_attached = (
        sbom_report.get("exists") is True and sbom_report.get("status") == "ready"
    )

    python_vuln_report = reports_by_name.get("python_vulnerability_scan_report", {})
    container_vuln_report = reports_by_name.get(
        "container_vulnerability_scan_report",
        {},
    )
    vuln_reports = [python_vuln_report, container_vuln_report]
    vuln_reports_present = sum(1 for item in vuln_reports if item.get("exists") is True)
    all_vuln_reports_present = vuln_reports_present == len(vuln_reports)
    remediation_ready = (
        remediation_report.get("exists") is True
        and remediation_report.get("status") == "ready"
        and _coerce_non_negative_int(remediation_report.get("unresolved_finding_count"))
        == 0
    )
    posture_ready = (
        security_posture_report.get("exists") is True
        and security_posture_report.get("status") == "ready"
        and _coerce_non_negative_int(
            security_posture_report.get("blocked_vulnerability_execution_reports")
        )
        == 0
        and _coerce_non_negative_int(
            security_posture_report.get("unresolved_vulnerability_findings")
        )
        == 0
    )
    if vuln_reports_present == 0:
        vuln_scan_status = "not_run"
    elif all(
        item.get("exists") is True and item.get("status") == "passed"
        for item in vuln_reports
    ):
        vuln_scan_status = "passed"
    elif all_vuln_reports_present and (remediation_ready or posture_ready):
        vuln_scan_status = "passed"
    else:
        vuln_scan_status = "blocked"

    backup_restore_report = reports_by_name.get("backup_restore_rehearsal_report", {})
    backup_restore_verified = (
        backup_restore_report.get("exists") is True
        and backup_restore_report.get("status") == "passed"
    )

    support_matrix_attached = customer_delivery_surface.get("support_matrix_attached") is True
    capacity_declaration_attached = (
        customer_delivery_surface.get("capacity_declaration_attached") is True
    )
    customer_acceptance_checklist_attached = (
        customer_delivery_surface.get("customer_acceptance_checklist_attached") is True
    )
    known_limitations_attached = (
        customer_delivery_surface.get("known_limitations_attached") is True
    )
    customer_delivery_surface_status = str(
        customer_delivery_surface.get("status") or "blocked"
    )
    extension_support_surface = dict(
        customer_delivery_surface.get("extension_support_surface", {})
        if isinstance(customer_delivery_surface.get("extension_support_surface"), Mapping)
        else {}
    )
    extension_support_surface_status = str(
        extension_support_surface.get("status") or "blocked"
    )
    release_ops_execution = dict(
        customer_delivery_surface.get("release_ops_execution", {})
        if isinstance(customer_delivery_surface.get("release_ops_execution"), Mapping)
        else {}
    )
    control_plane_session, control_plane_event_stream = (
        _aggregate_release_op_control_plane_surface(
            release_ops_execution,
            customer_delivery_surface,
        )
    )
    required_extension_profiles = _coerce_non_negative_int(
        extension_support_surface.get("required_profiles")
    )
    declared_extension_profiles = _coerce_non_negative_int(
        extension_support_surface.get("declared_profiles")
    )

    missing_requirements: list[str] = [
        str(item.get("path"))
        for item in hydrated_deployment_documents
        if item.get("required") is True
        and item.get("exists") is not True
        and _is_non_empty_string(item.get("path"))
    ]
    if not evidence_attested:
        for item in required_evidence_items:
            if item.get("status") == "passed" and item.get("attested") is True:
                continue
            name = item.get("name")
            if _is_non_empty_string(name):
                missing_requirements.append(f"release_evidence:{name}")
    if not sbom_attached:
        sbom_path = sbom_report.get("path")
        missing_requirements.append(
            str(sbom_path)
            if _is_non_empty_string(sbom_path)
            else "test_env/release_evidence/security/sbom.json"
        )
    if vuln_scan_status != "passed":
        if not all_vuln_reports_present:
            for item in vuln_reports:
                if item.get("exists") is True:
                    continue
                path = item.get("path")
                name = item.get("name")
                missing_requirements.append(
                    str(path)
                    if _is_non_empty_string(path)
                    else (
                        str(name)
                        if _is_non_empty_string(name)
                        else "vulnerability_scan_report"
                    )
                )
        elif not remediation_ready:
            remediation_path = remediation_report.get("path")
            missing_requirements.append(
                str(remediation_path)
                if _is_non_empty_string(remediation_path)
                else "test_env/release_evidence/security/vulnerability_remediation_report.json"
            )
        elif not posture_ready:
            posture_path = security_posture_report.get("path")
            missing_requirements.append(
                str(posture_path)
                if _is_non_empty_string(posture_path)
                else "test_env/release_evidence/security/security_posture_report.json"
            )
    if not backup_restore_verified:
        backup_path = backup_restore_report.get("path")
        missing_requirements.append(
            str(backup_path)
            if _is_non_empty_string(backup_path)
            else "test_env/release_evidence/security/backup_restore_rehearsal_report.json"
        )
    if customer_delivery_surface_status != "ready":
        for field in ["missing_required_documents", "missing_phase_e_documents"]:
            for item in customer_delivery_surface.get(field, []):
                if _is_non_empty_string(item):
                    missing_requirements.append(str(item))
    if extension_support_surface_status != "ready":
        for item in extension_support_surface.get("missing_documents", []):
            if _is_non_empty_string(item):
                missing_requirements.append(str(item))
    missing_requirements = list(dict.fromkeys(missing_requirements))

    status = (
        "ready"
        if deployment_package_status == "ready"
        and evidence_attested
        and sbom_attached
        and vuln_scan_status == "passed"
        and backup_restore_verified
        and customer_delivery_surface_status == "ready"
        and extension_support_surface_status == "ready"
        else "blocked"
    )
    summary = (
        f"Industrial delivery gate {status}: "
        f"deployment_package={deployment_package_status} "
        f"({ready_deployment_documents}/{required_deployment_documents}), "
        f"required evidence attested {attested_required_evidence}/{required_evidence}, "
        f"sbom_attached={str(sbom_attached).lower()}, "
        f"vuln_scan_status={vuln_scan_status}, "
        f"backup_restore_verified={str(backup_restore_verified).lower()}, "
        f"customer_delivery={customer_delivery_surface_status}, "
        "extension_support="
        f"{extension_support_surface_status} "
        f"({declared_extension_profiles}/{required_extension_profiles})."
    )
    if release_ops_execution.get("status"):
        summary += (
            " release_ops_execution="
            f"{_format_release_ops_execution_component_status(release_ops_execution)}."
        )
    if vuln_scan_status == "passed" and not all(
        item.get("exists") is True and item.get("status") == "passed"
        for item in vuln_reports
    ):
        if remediation_ready:
            summary += " Vulnerability findings are closed by a ready remediation report."
        elif posture_ready:
            summary += " Vulnerability findings are closed by a ready security posture report."
    if missing_requirements:
        summary += " Missing requirements: " + ", ".join(missing_requirements) + "."
    summary = _append_external_mainline_control_plane_summary(
        summary,
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )

    payload = {
        "status": status,
        "summary": summary,
        "deployment_package_status": deployment_package_status,
        "evidence_attested": evidence_attested,
        "required_evidence": required_evidence,
        "attested_required_evidence": attested_required_evidence,
        "sbom_attached": sbom_attached,
        "vuln_scan_status": vuln_scan_status,
        "backup_restore_verified": backup_restore_verified,
        "support_matrix_attached": support_matrix_attached,
        "capacity_declaration_attached": capacity_declaration_attached,
        "customer_acceptance_checklist_attached": customer_acceptance_checklist_attached,
        "known_limitations_attached": known_limitations_attached,
        "customer_delivery_surface_status": customer_delivery_surface_status,
        "extension_support_surface_status": extension_support_surface_status,
        "required_extension_profiles": required_extension_profiles,
        "declared_extension_profiles": declared_extension_profiles,
        "required_deployment_documents": required_deployment_documents,
        "ready_deployment_documents": ready_deployment_documents,
        "missing_requirements": missing_requirements,
        "deployment_documents": to_jsonable(hydrated_deployment_documents),
        "security_reports": to_jsonable(hydrated_security_reports),
        "release_ops_execution": to_jsonable(release_ops_execution),
    }
    if control_plane_session:
        payload["control_plane_session"] = to_jsonable(control_plane_session)
    if control_plane_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(control_plane_event_stream)
    return payload


def build_customer_acceptance_bundle_artifact(
    *,
    release_manifest: Mapping[str, Any],
    manifest_path: str | Path,
    project_root: str | Path | None = None,
    generated_at: str | None = None,
    readiness_report_path: str | Path | None = None,
    promotion_checklist_path: str | Path | None = None,
    security_posture_report_path: str | Path | None = None,
    sbom_artifact_path: str | Path | None = None,
    python_vulnerability_scan_report_path: str | Path | None = None,
    container_vulnerability_scan_report_path: str | Path | None = None,
    vulnerability_exception_report_path: str | Path | None = None,
    vulnerability_exception_review_report_path: str | Path | None = None,
    customer_external_bindings_closure_report_path: str | Path | None = None,
    external_mainline_execution_plan_path: str | Path | None = None,
    external_mainline_input_checklist_report_path: str | Path | None = None,
    release_ops_execution_report_path: str | Path | None = None,
    backup_restore_rehearsal_report_path: str | Path | None = None,
    industrial_delivery_rehearsal_report_path: str | Path | None = None,
    acceptance_documents: Sequence[Mapping[str, Any]] | None = None,
) -> dict[str, Any]:
    manifest_errors = validate_release_manifest_artifact(release_manifest)
    if manifest_errors:
        raise ValueError(f"invalid release manifest: {'; '.join(manifest_errors)}")

    resolved_root = Path(project_root) if project_root is not None else Path.cwd()
    resolved_manifest_path = _resolve_release_artifact_path(str(manifest_path), resolved_root)
    required_docs = acceptance_documents or default_customer_acceptance_documents()
    hydrated_documents = _hydrate_customer_acceptance_items(required_docs, resolved_root)

    report_entries = default_customer_acceptance_reports(str(release_manifest.get("channel")))
    report_overrides = {
        "release_readiness": readiness_report_path,
        "industrial_release_readiness": readiness_report_path,
        "stable_promotion_checklist": promotion_checklist_path,
        "industrial_promotion_checklist": promotion_checklist_path,
        "security_posture_report": security_posture_report_path,
        "sbom_artifact": sbom_artifact_path,
        "python_vulnerability_scan_report": python_vulnerability_scan_report_path,
        "container_vulnerability_scan_report": container_vulnerability_scan_report_path,
        "vulnerability_exception_report": vulnerability_exception_report_path,
        "vulnerability_exception_review": vulnerability_exception_review_report_path,
        "customer_external_bindings_closure": customer_external_bindings_closure_report_path,
        "external_mainline_execution_plan": external_mainline_execution_plan_path,
        "external_mainline_input_checklist": external_mainline_input_checklist_report_path,
        "release_ops_execution": release_ops_execution_report_path,
        "backup_restore_rehearsal_report": backup_restore_rehearsal_report_path,
        "industrial_delivery_rehearsal_report": industrial_delivery_rehearsal_report_path,
    }
    hydrated_reports: list[dict[str, Any]] = []
    for item in report_entries:
        current = dict(item)
        override = report_overrides.get(str(current.get("name")))
        if override:
            current["path"] = str(override)
        hydrated_reports.extend(_hydrate_customer_acceptance_items([current], resolved_root))
    reports_by_name = {
        str(item.get("name")): dict(item)
        for item in hydrated_reports
        if _is_non_empty_string(item.get("name"))
    }

    required_evidence = [
        dict(item)
        for item in release_manifest.get("test_evidence", [])
        if isinstance(item, Mapping) and item.get("required") is True
    ]
    optional_evidence = [
        dict(item)
        for item in release_manifest.get("test_evidence", [])
        if isinstance(item, Mapping) and item.get("required") is not True
    ]
    required_docs_total = sum(1 for item in hydrated_documents if item["required"] is True)
    required_docs_ready = sum(
        1
        for item in hydrated_documents
        if item["required"] is True and item["exists"] is True
    )
    required_evidence_total = len(required_evidence)
    required_evidence_ready = sum(
        1 for item in required_evidence if item.get("status") == "passed"
    )
    bundle_status = (
        "ready"
        if release_manifest.get("release_gate_status") == "ready"
        and required_docs_total == required_docs_ready
        and required_evidence_total == required_evidence_ready
        else "blocked"
    )
    vulnerability_exception_review = dict(
        reports_by_name.get("vulnerability_exception_review", {})
    )
    external_mainline_execution_plan = dict(
        reports_by_name.get("external_mainline_execution_plan", {})
    )
    external_mainline_input_checklist = dict(
        reports_by_name.get("external_mainline_input_checklist", {})
    )
    release_ops_execution = dict(reports_by_name.get("release_ops_execution", {}))
    summary = (
        f"Customer acceptance bundle {bundle_status}: "
        f"{required_docs_ready}/{required_docs_total} required documents present, "
        f"{required_evidence_ready}/{required_evidence_total} required evidence items passed, "
        f"release gate={release_manifest.get('release_gate_status')}."
    )
    review_status = _artifact_optional_string(vulnerability_exception_review.get("status"))
    review_metrics = (
        vulnerability_exception_review.get("metrics")
        if isinstance(vulnerability_exception_review.get("metrics"), Mapping)
        else {}
    )
    review_candidate_count = _coerce_non_negative_int(
        review_metrics.get("review_candidate_count")
    )
    if review_candidate_count is None:
        review_candidate_count = _coerce_non_negative_int(
            review_metrics.get("review_due_exception_count")
        )
    if review_status:
        review_summary = (
            f"{review_status}/{review_candidate_count}"
            if review_candidate_count is not None
            else review_status
        )
        summary += f" exception_review={review_summary}."
    external_mainline_status = _artifact_optional_string(
        external_mainline_execution_plan.get("status")
    )
    external_mainline_counts = [
        _coerce_non_negative_int(external_mainline_execution_plan.get(field))
        for field in [
            "completed_steps",
            "ready_to_run_steps",
            "waiting_external_input_steps",
            "blocked_steps",
        ]
    ]
    if external_mainline_status:
        if all(value is not None for value in external_mainline_counts):
            summary += (
                " external_mainline="
                + "/".join(
                    [
                        external_mainline_status,
                        *[str(value) for value in external_mainline_counts],
                    ]
                )
                + "."
            )
        else:
            summary += f" external_mainline={external_mainline_status}."
    external_mainline_input_checklist_status = _artifact_optional_string(
        external_mainline_input_checklist.get("status")
    )
    checklist_metrics = (
        external_mainline_input_checklist.get("metrics")
        if isinstance(external_mainline_input_checklist.get("metrics"), Mapping)
        else {}
    )
    checklist_counts = [
        _coerce_non_negative_int(checklist_metrics.get("missing_input_count")),
        len(checklist_metrics.get("waiting_external_input_steps", []))
        if isinstance(checklist_metrics.get("waiting_external_input_steps"), list)
        else None,
        len(checklist_metrics.get("ready_to_run_steps", []))
        if isinstance(checklist_metrics.get("ready_to_run_steps"), list)
        else None,
        len(checklist_metrics.get("completed_steps", []))
        if isinstance(checklist_metrics.get("completed_steps"), list)
        else None,
    ]
    if external_mainline_input_checklist_status:
        if all(value is not None for value in checklist_counts):
            summary += (
                " external_mainline_input_checklist="
                + "/".join(
                    [
                        external_mainline_input_checklist_status,
                        *[str(value) for value in checklist_counts],
                    ]
                )
                + "."
            )
        else:
            summary += (
                " external_mainline_input_checklist="
                f"{external_mainline_input_checklist_status}."
            )
    release_ops_execution_status = _artifact_optional_string(
        release_ops_execution.get("status")
    )
    release_ops_execution_metrics = (
        release_ops_execution.get("metrics")
        if isinstance(release_ops_execution.get("metrics"), Mapping)
        else {}
    )
    release_ops_execution_event_count = _coerce_non_negative_int(
        release_ops_execution_metrics.get("event_count")
    )
    if release_ops_execution_event_count is None:
        release_ops_execution_event_stream = (
            release_ops_execution.get("control_plane_event_stream")
            if isinstance(release_ops_execution.get("control_plane_event_stream"), Mapping)
            else {}
        )
        release_ops_execution_event_count = _coerce_non_negative_int(
            release_ops_execution_event_stream.get("event_count")
        )
    if release_ops_execution_status:
        if release_ops_execution_event_count is not None:
            summary += (
                " release_ops_execution="
                f"{release_ops_execution_status}/{release_ops_execution_event_count}."
            )
        else:
            summary += f" release_ops_execution={release_ops_execution_status}."
    (
        control_plane_session,
        control_plane_event_stream,
    ) = _aggregate_release_op_control_plane_surface(
        release_ops_execution,
        external_mainline_execution_plan,
        external_mainline_input_checklist,
    )
    summary = _append_external_mainline_control_plane_summary(
        summary,
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )
    extension_support_surface = dict(
        release_manifest.get("customer_delivery_surface", {}).get(
            "extension_support_surface",
            {},
        )
        if isinstance(release_manifest.get("customer_delivery_surface"), Mapping)
        else {}
    )
    extension_execution_plan = build_extension_execution_plan(extension_support_surface)
    extension_execution_evidence = build_extension_execution_evidence(
        project_root=resolved_root,
        extension_support_surface=extension_support_surface,
    )
    extension_execution_instance = build_extension_execution_instance(
        project_root=resolved_root,
        extension_support_surface=extension_support_surface,
    )
    extension_execution_schedule = build_extension_execution_schedule(
        project_root=resolved_root,
        extension_support_surface=extension_support_surface,
    )
    extension_execution_actuals = build_extension_execution_actuals(
        project_root=resolved_root,
        extension_support_surface=extension_support_surface,
    )
    external_bindings_config_path = None
    if isinstance(extension_execution_actuals.get("external_bindings"), Mapping):
        external_bindings_config_path = _artifact_optional_string(
            extension_execution_actuals.get("external_bindings", {}).get("config_path")
        )
    external_bindings_status = str(
        extension_execution_actuals.get("external_bindings_status") or ""
    ).strip()
    draft_external_binding_sections = [
        str(item).strip()
        for item in extension_execution_actuals.get("external_bindings_draft_sections", [])
        if _is_non_empty_string(item)
    ]
    unconfirmed_external_binding_sections = [
        str(item).strip()
        for item in extension_execution_actuals.get(
            "external_bindings_unconfirmed_sections", []
        )
        if _is_non_empty_string(item)
    ]
    confirmation_missing_external_binding_sections = [
        str(item).strip()
        for item in extension_execution_actuals.get(
            "external_bindings_confirmation_missing_sections", []
        )
        if _is_non_empty_string(item)
    ]
    placeholder_external_binding_sections = [
        str(item).strip()
        for item in extension_execution_actuals.get(
            "external_bindings_placeholder_sections", []
        )
        if _is_non_empty_string(item)
    ]
    generated_external_bindings_path = None
    confirm_external_bindings_path = None
    confirm_external_binding_sections: list[str] = []
    if external_bindings_status in {"missing", "placeholder", "partial"}:
        resolved_external_bindings_path = resolve_customer_external_bindings_config_path(
            external_bindings_config_path
        )
        if (
            _is_non_empty_string(external_bindings_config_path)
            and external_bindings_config_path
            not in {
                default_placeholder_external_bindings_config_path(),
                default_rehearsal_external_bindings_config_path(),
            }
        ):
            confirm_external_bindings_path = resolved_external_bindings_path
            confirm_external_binding_sections = (
                draft_external_binding_sections
                or confirmation_missing_external_binding_sections
                or unconfirmed_external_binding_sections
                or placeholder_external_binding_sections
                or list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)
            )
        else:
            generated_external_bindings_path = resolved_external_bindings_path
            confirm_external_bindings_path = resolved_external_bindings_path
            confirm_external_binding_sections = (
                confirmation_missing_external_binding_sections
                or unconfirmed_external_binding_sections
                or placeholder_external_binding_sections
                or draft_external_binding_sections
                or list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)
            )
    extension_execution_actuals_command = build_extension_execution_actuals_command(
        output_path=extension_execution_actuals.get("artifact_path"),
        external_bindings_config_path=(
            confirm_external_bindings_path
            or generated_external_bindings_path
            or external_bindings_config_path
        ),
    )
    recommended_commands = [
        "python tools/collect_release_evidence.py",
        build_external_mainline_inputs_command(),
        build_run_external_mainline_execution_plan_command(),
        build_external_mainline_execution_plan_command(),
        build_external_mainline_input_checklist_command(),
        "python tools/build_sbom_artifact.py --output test_env/release_evidence/security/sbom.json",
        "python tools/run_backup_restore_rehearsal.py --output-root test_env/release_evidence/security/backup_restore_rehearsal --report-file test_env/release_evidence/security/backup_restore_rehearsal_report.json",
        "python tools/build_security_posture_report.py --output test_env/release_evidence/security/security_posture_report.json",
        build_vulnerability_exception_review_report_command(),
        "python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json",
        "python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json",
        "python tools/build_release_artifact.py --version ... --channel ... --build-id ...",
        "python tools/build_customer_acceptance_bundle.py --manifest ...",
    ]
    if generated_external_bindings_path is not None or confirm_external_bindings_path is not None:
        recommended_commands.append(
            build_run_customer_external_bindings_closure_command(
                config_path=(
                    confirm_external_bindings_path
                    or generated_external_bindings_path
                    or external_bindings_config_path
                ),
                instance_artifact_path=extension_execution_instance.get("artifact_path"),
                actuals_artifact_path=extension_execution_actuals.get("artifact_path"),
                sections=confirm_external_binding_sections,
            )
        )
    if generated_external_bindings_path is not None:
        recommended_commands.append(
            build_customer_external_bindings_config_command(
                output_path=generated_external_bindings_path,
                instance_artifact_path=extension_execution_instance.get("artifact_path"),
            )
        )
    if confirm_external_bindings_path is not None:
        recommended_commands.append(
            build_confirm_customer_external_bindings_command(
                config_path=confirm_external_bindings_path,
                sections=confirm_external_binding_sections,
            )
        )
    recommended_commands.append(extension_execution_actuals_command)
    recommended_commands.append(
        build_customer_external_bindings_confirmation_report_command(
            actuals_artifact_path=extension_execution_actuals.get("artifact_path")
        )
    )
    recommended_commands.append(
        "python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations"
    )
    if str(release_manifest.get("channel")) == "industrial":
        recommended_commands.append(
            "python tools/build_industrial_delivery_rehearsal_report.py --rehearsal-report ..."
        )
    for item in extension_execution_plan.get("profiles", []):
        if not isinstance(item, Mapping) or item.get("actionable") is not True:
            continue
        recommended_commands.extend(
            [
                str(command).strip()
                for command in item.get("deployment_commands", [])
                if _is_non_empty_string(command)
            ]
        )
        recommended_commands.extend(
            [
                str(command).strip()
                for command in item.get("acceptance_checks", [])
                if _is_non_empty_string(command)
            ]
        )

    payload = {
        "schema_version": CUSTOMER_ACCEPTANCE_BUNDLE_VERSION,
        "artifact_type": CUSTOMER_ACCEPTANCE_BUNDLE_ARTIFACT_TYPE,
        "generated_at": generated_at or datetime.now().isoformat(),
        "bundle_status": bundle_status,
        "summary": summary,
        "version": release_manifest.get("version"),
        "channel": release_manifest.get("channel"),
        "build_id": release_manifest.get("build_id"),
        "release_manifest": {
            "path": str(resolved_manifest_path),
            "release_gate_status": release_manifest.get("release_gate_status"),
            "release_summary": release_manifest.get("release_summary"),
            "generated_at": release_manifest.get("generated_at"),
            "source_commit_sha": (
                release_manifest.get("release_source", {}).get("commit_sha")
                if isinstance(release_manifest.get("release_source"), Mapping)
                else None
            ),
        },
        "known_limitations": to_jsonable(
            [
                str(item).strip()
                for item in release_manifest.get("known_limitations", [])
                if _is_non_empty_string(item)
            ]
        ),
        "extension_support_surface": to_jsonable(extension_support_surface),
        "extension_execution_plan": to_jsonable(extension_execution_plan),
        "extension_execution_evidence": to_jsonable(extension_execution_evidence),
        "extension_execution_instance": to_jsonable(extension_execution_instance),
        "extension_execution_schedule": to_jsonable(extension_execution_schedule),
        "extension_execution_actuals": to_jsonable(extension_execution_actuals),
        "vulnerability_exception_review": to_jsonable(vulnerability_exception_review),
        "external_mainline_execution_plan": to_jsonable(
            external_mainline_execution_plan
        ),
        "external_mainline_input_checklist": to_jsonable(
            external_mainline_input_checklist
        ),
        "release_ops_execution": to_jsonable(release_ops_execution),
        "required_evidence": to_jsonable(required_evidence),
        "optional_evidence": to_jsonable(optional_evidence),
        "acceptance_documents": to_jsonable(hydrated_documents),
        "acceptance_reports": to_jsonable(hydrated_reports),
        "recommended_commands": list(dict.fromkeys(recommended_commands)),
    }
    if control_plane_session:
        payload["control_plane_session"] = to_jsonable(control_plane_session)
    if control_plane_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(control_plane_event_stream)
    return payload


def validate_customer_acceptance_bundle_artifact(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["customer acceptance bundle must be an object"]

    errors: list[str] = []
    missing = sorted(CUSTOMER_ACCEPTANCE_BUNDLE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != CUSTOMER_ACCEPTANCE_BUNDLE_VERSION:
        errors.append(
            f"schema_version must be {CUSTOMER_ACCEPTANCE_BUNDLE_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != CUSTOMER_ACCEPTANCE_BUNDLE_ARTIFACT_TYPE:
        errors.append(
            f"artifact_type must be {CUSTOMER_ACCEPTANCE_BUNDLE_ARTIFACT_TYPE!r}"
        )
    if payload.get("bundle_status") not in CUSTOMER_ACCEPTANCE_BUNDLE_STATUSES:
        errors.append(
            f"bundle_status must be one of {sorted(CUSTOMER_ACCEPTANCE_BUNDLE_STATUSES)}"
        )
    for field in ["summary", "version", "channel", "build_id", "generated_at"]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")

    manifest = payload.get("release_manifest")
    if not isinstance(manifest, Mapping):
        errors.append("release_manifest must be an object")
    else:
        missing_manifest = sorted(CUSTOMER_ACCEPTANCE_RELEASE_MANIFEST_FIELDS - set(manifest))
        if missing_manifest:
            errors.append(
                "release_manifest missing required fields: "
                + ", ".join(missing_manifest)
            )
        for field in CUSTOMER_ACCEPTANCE_RELEASE_MANIFEST_FIELDS:
            if field in manifest and manifest.get(field) is not None and not _is_non_empty_string(
                manifest.get(field)
            ):
                errors.append(f"release_manifest.{field} must be null or a non-empty string")
        if manifest.get("release_gate_status") not in RELEASE_GATE_STATUSES:
            errors.append(
                f"release_manifest.release_gate_status must be one of {sorted(RELEASE_GATE_STATUSES)}"
            )

    for list_name in ["required_evidence", "optional_evidence"]:
        items = payload.get(list_name)
        if not isinstance(items, list):
            errors.append(f"{list_name} must be a list")
            continue
        for index, item in enumerate(items, start=1):
            prefix = f"{list_name}[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            missing_fields = sorted(RELEASE_TEST_EVIDENCE_REQUIRED_FIELDS - set(item))
            if missing_fields:
                errors.append(
                    f"{prefix} missing required fields: {', '.join(missing_fields)}"
                )

    known_limitations = payload.get("known_limitations")
    if not isinstance(known_limitations, list):
        errors.append("known_limitations must be a list")
    else:
        for index, item in enumerate(known_limitations, start=1):
            if not _is_non_empty_string(item):
                errors.append(
                    f"known_limitations[{index}] must be a non-empty string"
                )

    errors.extend(
        validate_extension_support_surface(payload.get("extension_support_surface"))
    )
    errors.extend(
        validate_extension_execution_plan(payload.get("extension_execution_plan"))
    )
    errors.extend(
        validate_extension_execution_evidence(payload.get("extension_execution_evidence"))
    )
    errors.extend(
        validate_extension_execution_instance(payload.get("extension_execution_instance"))
    )
    errors.extend(
        validate_extension_execution_schedule(payload.get("extension_execution_schedule"))
    )
    errors.extend(
        validate_extension_execution_actuals(payload.get("extension_execution_actuals"))
    )
    vulnerability_exception_review = payload.get("vulnerability_exception_review")
    if vulnerability_exception_review is not None:
        if not isinstance(vulnerability_exception_review, Mapping):
            errors.append("vulnerability_exception_review must be an object")
        else:
            status = vulnerability_exception_review.get("status")
            if status is not None and status not in RELEASE_EVIDENCE_STATUSES:
                errors.append(
                    "vulnerability_exception_review.status must be one of "
                    f"{sorted(RELEASE_EVIDENCE_STATUSES)}"
                )
            if "summary" in vulnerability_exception_review and not _is_non_empty_string(
                vulnerability_exception_review.get("summary")
            ):
                errors.append(
                    "vulnerability_exception_review.summary must be a non-empty string"
                )

    external_mainline_execution_plan = payload.get("external_mainline_execution_plan")
    if external_mainline_execution_plan is not None:
        if not isinstance(external_mainline_execution_plan, Mapping):
            errors.append("external_mainline_execution_plan must be an object")
        else:
            status = external_mainline_execution_plan.get("status")
            if (
                status is not None
                and status not in EXTERNAL_MAINLINE_EXECUTION_PLAN_STATUSES
            ):
                errors.append(
                    "external_mainline_execution_plan.status must be one of "
                    f"{sorted(EXTERNAL_MAINLINE_EXECUTION_PLAN_STATUSES)}"
                )
            if "summary" in external_mainline_execution_plan and not _is_non_empty_string(
                external_mainline_execution_plan.get("summary")
            ):
                errors.append(
                    "external_mainline_execution_plan.summary must be a non-empty string"
                )
            for field in [
                "completed_steps",
                "ready_to_run_steps",
                "waiting_external_input_steps",
                "blocked_steps",
            ]:
                if field in external_mainline_execution_plan and not _is_non_negative_int(
                    external_mainline_execution_plan.get(field)
                ):
                    errors.append(
                        f"external_mainline_execution_plan.{field} must be a non-negative integer"
                    )
    external_mainline_input_checklist = payload.get("external_mainline_input_checklist")
    if external_mainline_input_checklist is not None:
        if not isinstance(external_mainline_input_checklist, Mapping):
            errors.append("external_mainline_input_checklist must be an object")
        else:
            status = external_mainline_input_checklist.get("status")
            if status is not None and status not in RELEASE_EVIDENCE_STATUSES:
                errors.append(
                    "external_mainline_input_checklist.status must be one of "
                    f"{sorted(RELEASE_EVIDENCE_STATUSES)}"
                )
            if "summary" in external_mainline_input_checklist and not _is_non_empty_string(
                external_mainline_input_checklist.get("summary")
            ):
                errors.append(
                    "external_mainline_input_checklist.summary must be a non-empty string"
                )
    release_ops_execution = payload.get("release_ops_execution")
    if release_ops_execution is not None:
        if not isinstance(release_ops_execution, Mapping):
            errors.append("release_ops_execution must be an object")
        else:
            status = release_ops_execution.get("status")
            if status is not None and status not in RELEASE_EVIDENCE_STATUSES:
                errors.append(
                    "release_ops_execution.status must be one of "
                    f"{sorted(RELEASE_EVIDENCE_STATUSES)}"
                )
            if "summary" in release_ops_execution and not _is_non_empty_string(
                release_ops_execution.get("summary")
            ):
                errors.append("release_ops_execution.summary must be a non-empty string")
            if "metrics" in release_ops_execution and not isinstance(
                release_ops_execution.get("metrics"), Mapping
            ):
                errors.append("release_ops_execution.metrics must be an object")
    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append("control_plane_session must be an object")
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        f"control_plane_session.{field} must be a non-empty string when present"
                    )
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append("control_plane_event_stream must be an object")
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    "control_plane_event_stream.path must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    "control_plane_event_stream.event_count must be a non-negative integer when present"
                )

    for list_name in ["acceptance_documents", "acceptance_reports"]:
        errors.extend(_validate_customer_acceptance_items_list(list_name, payload.get(list_name)))

    commands = payload.get("recommended_commands")
    if not isinstance(commands, list) or not commands:
        errors.append("recommended_commands must be a non-empty list")
    else:
        for index, item in enumerate(commands, start=1):
            if not _is_non_empty_string(item):
                errors.append(f"recommended_commands[{index}] must be a non-empty string")

    return errors


def write_customer_acceptance_bundle_artifact(
    payload: Mapping[str, Any], path: str | Path
) -> Path:
    errors = validate_customer_acceptance_bundle_artifact(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _normalize_vulnerability_exception_review_rehearsal_component(
    component: Any,
    *,
    security_release_preflight: Mapping[str, Any],
) -> dict[str, Any]:
    payload = dict(component) if isinstance(component, Mapping) else {}
    metrics = (
        security_release_preflight.get("metrics")
        if isinstance(security_release_preflight, Mapping)
        else None
    )
    if isinstance(metrics, Mapping):
        review_status = str(
            metrics.get("vulnerability_exception_review_report_status") or ""
        ).strip()
        review_report_path = str(
            metrics.get("vulnerability_exception_review_report_path") or ""
        ).strip()
        review_candidate_count = _coerce_non_negative_int(
            metrics.get("vulnerability_exception_review_candidate_count")
        )

        def _coerce_string_list(value: Any) -> list[str]:
            if not isinstance(value, list):
                return []
            return [str(item).strip() for item in value if _is_non_empty_string(item)]

        if not _is_non_empty_string(payload.get("status")) and review_status:
            payload["status"] = review_status
        if not _is_non_empty_string(payload.get("report_path")) and review_report_path:
            payload["report_path"] = review_report_path
        if (
            payload.get("review_candidate_count") is None
            and review_candidate_count is not None
        ):
            payload["review_candidate_count"] = review_candidate_count
        if "review_due_exception_ids" not in payload:
            review_due_exception_ids = _coerce_string_list(
                metrics.get("review_due_vulnerability_exception_ids")
            )
            if review_due_exception_ids:
                payload["review_due_exception_ids"] = review_due_exception_ids
        if "review_due_exception_tickets" not in payload:
            review_due_exception_tickets = _coerce_string_list(
                metrics.get("review_due_vulnerability_exception_tickets")
            )
            if review_due_exception_tickets:
                payload["review_due_exception_tickets"] = review_due_exception_tickets
        if "expired_exception_ids" not in payload:
            expired_exception_ids = _coerce_string_list(
                metrics.get("expired_vulnerability_exception_ids")
            )
            if expired_exception_ids:
                payload["expired_exception_ids"] = expired_exception_ids
        if not _is_non_empty_string(payload.get("summary")) and review_status:
            summary = f"vulnerability exception review status={review_status}"
            if review_candidate_count is not None:
                summary += f", candidates={review_candidate_count}"
            payload["summary"] = summary + "."

    return _normalize_industrial_delivery_rehearsal_component(
        payload,
        default_status="blocked",
        default_summary="vulnerability exception review is missing from the rehearsal report.",
        component_label="vulnerability exception review",
    )


def _format_industrial_delivery_component_status(component: Mapping[str, Any]) -> str:
    status = str(component.get("status") or "unknown").strip() or "unknown"
    candidate_count = _coerce_non_negative_int(component.get("review_candidate_count"))
    if candidate_count is not None:
        return f"{status}/{candidate_count}"
    return status


def _normalize_customer_external_bindings_closure_rehearsal_component(
    component: Any,
    *,
    industrial_customer_acceptance_bundle: Mapping[str, Any],
    release_rehearsal_report_path: str | Path,
) -> dict[str, Any]:
    payload = dict(component) if isinstance(component, Mapping) else {}
    if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
        payload.get("summary")
    ):
        return to_jsonable(payload)

    bundle_path_value = industrial_customer_acceptance_bundle.get("bundle_path")
    if not _is_non_empty_string(bundle_path_value):
        return _normalize_industrial_delivery_rehearsal_component(
            payload,
            default_status="blocked",
            default_summary="customer external bindings closure is missing from the rehearsal report.",
            component_label="customer external bindings closure",
        )

    resolved_bundle_path = Path(str(bundle_path_value).strip())
    if not resolved_bundle_path.is_absolute():
        resolved_bundle_path = (
            Path(release_rehearsal_report_path).resolve().parent / resolved_bundle_path
        )
    if not resolved_bundle_path.is_file():
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "customer external bindings closure report is missing from the "
            f"industrial customer acceptance bundle: {resolved_bundle_path}",
        )
        return to_jsonable(payload)

    try:
        bundle_payload = json.loads(resolved_bundle_path.read_text(encoding="utf-8"))
    except Exception as exc:
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "customer external bindings closure report is unreadable from the "
            f"industrial customer acceptance bundle: {exc}",
        )
        return to_jsonable(payload)

    acceptance_reports = bundle_payload.get("acceptance_reports")
    if isinstance(acceptance_reports, list):
        for item in acceptance_reports:
            if not isinstance(item, Mapping):
                continue
            if item.get("name") != "customer_external_bindings_closure":
                continue
            report_path = item.get("resolved_report_path") or item.get("path")
            payload.setdefault(
                "status",
                str(item.get("status")).strip()
                if _is_non_empty_string(item.get("status"))
                else "blocked",
            )
            payload.setdefault(
                "summary",
                str(item.get("summary")).strip()
                if _is_non_empty_string(item.get("summary"))
                else "customer external bindings closure status is unavailable.",
            )
            payload.setdefault("bundle_path", str(resolved_bundle_path))
            if _is_non_empty_string(report_path):
                payload.setdefault("report_path", str(report_path).strip())
            metrics = item.get("metrics")
            if isinstance(metrics, Mapping):
                failed_steps = (
                    [
                        str(step).strip()
                        for step in metrics.get("failed_steps", [])
                        if _is_non_empty_string(step)
                    ]
                    if isinstance(metrics.get("failed_steps"), list)
                    else []
                )
                if failed_steps:
                    payload.setdefault("failed_steps", failed_steps)
            break

    return _normalize_industrial_delivery_rehearsal_component(
        payload,
        default_status="blocked",
        default_summary="customer external bindings closure is missing from the rehearsal report.",
        component_label="customer external bindings closure",
    )


def _hydrate_external_mainline_execution_plan_component(
    payload: dict[str, Any],
    *,
    component_source: Mapping[str, Any],
    bundle_path: str,
) -> None:
    payload.setdefault(
        "status",
        str(component_source.get("status")).strip()
        if _is_non_empty_string(component_source.get("status"))
        else "blocked",
    )
    payload.setdefault(
        "summary",
        str(component_source.get("summary")).strip()
        if _is_non_empty_string(component_source.get("summary"))
        else "external mainline execution plan status is unavailable.",
    )
    payload.setdefault("bundle_path", bundle_path)
    report_path = (
        component_source.get("report_path")
        or component_source.get("resolved_report_path")
        or component_source.get("path")
    )
    if _is_non_empty_string(report_path):
        payload.setdefault("report_path", str(report_path).strip())
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = component_source.get(field)
        if _is_non_negative_int(value):
            payload.setdefault(field, value)
    control_plane_session = _normalize_release_op_session_context_summary(
        component_source.get("control_plane_session")
        if isinstance(component_source.get("control_plane_session"), Mapping)
        else None
    )
    if control_plane_session:
        payload.setdefault("control_plane_session", to_jsonable(control_plane_session))
    control_plane_event_stream = _normalize_release_op_event_stream_summary(
        component_source.get("control_plane_event_stream")
        if isinstance(component_source.get("control_plane_event_stream"), Mapping)
        else None
    )
    if control_plane_event_stream:
        payload.setdefault(
            "control_plane_event_stream",
            to_jsonable(control_plane_event_stream),
        )


def _normalize_external_mainline_execution_plan_rehearsal_component(
    component: Any,
    *,
    industrial_customer_acceptance_bundle: Mapping[str, Any],
    release_rehearsal_report_path: str | Path,
) -> dict[str, Any]:
    payload = dict(component) if isinstance(component, Mapping) else {}
    if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
        payload.get("summary")
    ):
        return to_jsonable(payload)

    bundle_path_value = industrial_customer_acceptance_bundle.get("bundle_path")
    if not _is_non_empty_string(bundle_path_value):
        return _normalize_industrial_delivery_rehearsal_component(
            payload,
            default_status="blocked",
            default_summary="external mainline execution plan is missing from the rehearsal report.",
            component_label="external mainline execution plan",
        )

    resolved_bundle_path = Path(str(bundle_path_value).strip())
    embedded_component = industrial_customer_acceptance_bundle.get(
        "external_mainline_execution_plan"
    )
    if isinstance(embedded_component, Mapping):
        _hydrate_external_mainline_execution_plan_component(
            payload,
            component_source=embedded_component,
            bundle_path=str(resolved_bundle_path),
        )
        if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
            payload.get("summary")
        ):
            return _normalize_industrial_delivery_rehearsal_component(
                payload,
                default_status="blocked",
                default_summary=(
                    "external mainline execution plan is missing from the rehearsal report."
                ),
                component_label="external mainline execution plan",
            )
    if not resolved_bundle_path.is_absolute():
        resolved_bundle_path = (
            Path(release_rehearsal_report_path).resolve().parent / resolved_bundle_path
        )
    if not resolved_bundle_path.is_file():
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "external mainline execution plan is missing from the industrial "
            f"customer acceptance bundle: {resolved_bundle_path}",
        )
        return to_jsonable(payload)

    try:
        bundle_payload = json.loads(resolved_bundle_path.read_text(encoding="utf-8"))
    except Exception as exc:
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "external mainline execution plan is unreadable from the industrial "
            f"customer acceptance bundle: {exc}",
        )
        return to_jsonable(payload)

    inline_component = bundle_payload.get("external_mainline_execution_plan")
    if isinstance(inline_component, Mapping):
        _hydrate_external_mainline_execution_plan_component(
            payload,
            component_source=inline_component,
            bundle_path=str(resolved_bundle_path),
        )
        if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
            payload.get("summary")
        ):
            return _normalize_industrial_delivery_rehearsal_component(
                payload,
                default_status="blocked",
                default_summary=(
                    "external mainline execution plan is missing from the rehearsal report."
                ),
                component_label="external mainline execution plan",
            )

    acceptance_reports = bundle_payload.get("acceptance_reports")
    if isinstance(acceptance_reports, list):
        for item in acceptance_reports:
            if not isinstance(item, Mapping):
                continue
            if item.get("name") != "external_mainline_execution_plan":
                continue
            _hydrate_external_mainline_execution_plan_component(
                payload,
                component_source=item,
                bundle_path=str(resolved_bundle_path),
            )
            break

    return _normalize_industrial_delivery_rehearsal_component(
        payload,
        default_status="blocked",
        default_summary="external mainline execution plan is missing from the rehearsal report.",
        component_label="external mainline execution plan",
    )


def _hydrate_external_mainline_input_checklist_component(
    payload: dict[str, Any],
    *,
    component_source: Mapping[str, Any],
    bundle_path: str,
) -> None:
    payload.setdefault(
        "status",
        str(component_source.get("status")).strip()
        if _is_non_empty_string(component_source.get("status"))
        else "blocked",
    )
    payload.setdefault(
        "summary",
        str(component_source.get("summary")).strip()
        if _is_non_empty_string(component_source.get("summary"))
        else "external mainline input checklist status is unavailable.",
    )
    payload.setdefault("bundle_path", bundle_path)
    report_path = (
        component_source.get("report_path")
        or component_source.get("resolved_report_path")
        or component_source.get("path")
    )
    if _is_non_empty_string(report_path):
        payload.setdefault("report_path", str(report_path).strip())
    metrics = (
        component_source.get("metrics")
        if isinstance(component_source.get("metrics"), Mapping)
        else {}
    )
    missing_input_count = component_source.get("missing_input_count")
    if not _is_non_negative_int(missing_input_count):
        missing_input_count = metrics.get("missing_input_count")
    if _is_non_negative_int(missing_input_count):
        payload.setdefault("missing_input_count", missing_input_count)
    for field in [
        "waiting_external_input_steps",
        "ready_to_run_steps",
        "completed_steps",
    ]:
        values = component_source.get(field)
        if not isinstance(values, list):
            values = metrics.get(field)
        if isinstance(values, list):
            payload.setdefault(
                field,
                [
                    str(item).strip()
                    for item in values
                    if _is_non_empty_string(item)
                ],
            )
    control_plane_session = _normalize_release_op_session_context_summary(
        component_source.get("control_plane_session")
        if isinstance(component_source.get("control_plane_session"), Mapping)
        else None
    )
    if control_plane_session:
        payload.setdefault("control_plane_session", to_jsonable(control_plane_session))
    control_plane_event_stream = _normalize_release_op_event_stream_summary(
        component_source.get("control_plane_event_stream")
        if isinstance(component_source.get("control_plane_event_stream"), Mapping)
        else None
    )
    if control_plane_event_stream:
        payload.setdefault(
            "control_plane_event_stream",
            to_jsonable(control_plane_event_stream),
        )


def _normalize_external_mainline_input_checklist_rehearsal_component(
    component: Any,
    *,
    industrial_customer_acceptance_bundle: Mapping[str, Any],
    release_rehearsal_report_path: str | Path,
) -> dict[str, Any]:
    payload = dict(component) if isinstance(component, Mapping) else {}
    if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
        payload.get("summary")
    ):
        return to_jsonable(payload)

    bundle_path_value = industrial_customer_acceptance_bundle.get("bundle_path")
    if not _is_non_empty_string(bundle_path_value):
        return _normalize_industrial_delivery_rehearsal_component(
            payload,
            default_status="blocked",
            default_summary=(
                "external mainline input checklist is missing from the rehearsal report."
            ),
            component_label="external mainline input checklist",
        )

    resolved_bundle_path = Path(str(bundle_path_value).strip())
    embedded_component = industrial_customer_acceptance_bundle.get(
        "external_mainline_input_checklist"
    )
    if isinstance(embedded_component, Mapping):
        _hydrate_external_mainline_input_checklist_component(
            payload,
            component_source=embedded_component,
            bundle_path=str(resolved_bundle_path),
        )
        if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
            payload.get("summary")
        ):
            return _normalize_industrial_delivery_rehearsal_component(
                payload,
                default_status="blocked",
                default_summary=(
                    "external mainline input checklist is missing from the rehearsal report."
                ),
                component_label="external mainline input checklist",
            )
    if not resolved_bundle_path.is_absolute():
        resolved_bundle_path = (
            Path(release_rehearsal_report_path).resolve().parent / resolved_bundle_path
        )
    if not resolved_bundle_path.is_file():
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "external mainline input checklist is missing from the industrial "
            f"customer acceptance bundle: {resolved_bundle_path}",
        )
        return to_jsonable(payload)

    try:
        bundle_payload = json.loads(resolved_bundle_path.read_text(encoding="utf-8"))
    except Exception as exc:
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "external mainline input checklist is unreadable from the industrial "
            f"customer acceptance bundle: {exc}",
        )
        return to_jsonable(payload)

    inline_component = bundle_payload.get("external_mainline_input_checklist")
    if isinstance(inline_component, Mapping):
        _hydrate_external_mainline_input_checklist_component(
            payload,
            component_source=inline_component,
            bundle_path=str(resolved_bundle_path),
        )
        if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
            payload.get("summary")
        ):
            return _normalize_industrial_delivery_rehearsal_component(
                payload,
                default_status="blocked",
                default_summary=(
                    "external mainline input checklist is missing from the rehearsal report."
                ),
                component_label="external mainline input checklist",
            )

    acceptance_reports = bundle_payload.get("acceptance_reports")
    if isinstance(acceptance_reports, list):
        for item in acceptance_reports:
            if not isinstance(item, Mapping):
                continue
            if item.get("name") != "external_mainline_input_checklist":
                continue
            _hydrate_external_mainline_input_checklist_component(
                payload,
                component_source=item,
                bundle_path=str(resolved_bundle_path),
            )
            break

    return _normalize_industrial_delivery_rehearsal_component(
        payload,
        default_status="blocked",
        default_summary=(
            "external mainline input checklist is missing from the rehearsal report."
        ),
        component_label="external mainline input checklist",
    )


def _hydrate_release_ops_execution_component(
    payload: dict[str, Any],
    *,
    component_source: Mapping[str, Any],
    bundle_path: str | None,
) -> None:
    payload.setdefault(
        "status",
        str(component_source.get("status")).strip()
        if _is_non_empty_string(component_source.get("status"))
        else "blocked",
    )
    payload.setdefault(
        "summary",
        str(component_source.get("summary")).strip()
        if _is_non_empty_string(component_source.get("summary"))
        else "release ops execution status is unavailable.",
    )
    if _is_non_empty_string(bundle_path):
        payload.setdefault("bundle_path", str(bundle_path).strip())
    report_path = (
        component_source.get("report_path")
        or component_source.get("resolved_report_path")
        or component_source.get("path")
    )
    if _is_non_empty_string(report_path):
        payload.setdefault("report_path", str(report_path).strip())
    metrics = (
        component_source.get("metrics")
        if isinstance(component_source.get("metrics"), Mapping)
        else {}
    )
    event_count = component_source.get("event_count")
    if not _is_non_negative_int(event_count):
        event_count = metrics.get("event_count")
    if _is_non_negative_int(event_count):
        payload.setdefault("event_count", event_count)
    for field in ["action", "policy_level", "policy_profile", "request_type"]:
        value = component_source.get(field)
        if not _is_non_empty_string(value):
            value = metrics.get(field)
        if _is_non_empty_string(value):
            payload.setdefault(field, str(value).strip())
    control_plane_session = _normalize_release_op_session_context_summary(
        component_source.get("control_plane_session")
        if isinstance(component_source.get("control_plane_session"), Mapping)
        else None
    )
    if control_plane_session:
        payload.setdefault("control_plane_session", to_jsonable(control_plane_session))
    control_plane_event_stream = _normalize_release_op_event_stream_summary(
        component_source.get("control_plane_event_stream")
        if isinstance(component_source.get("control_plane_event_stream"), Mapping)
        else None
    )
    if control_plane_event_stream:
        payload.setdefault(
            "control_plane_event_stream",
            to_jsonable(control_plane_event_stream),
        )


def _normalize_release_ops_execution_rehearsal_component(
    component: Any,
    *,
    industrial_customer_acceptance_bundle: Mapping[str, Any],
    release_rehearsal_report_path: str | Path,
) -> dict[str, Any]:
    payload = dict(component) if isinstance(component, Mapping) else {}
    if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
        payload.get("summary")
    ):
        return to_jsonable(payload)

    bundle_path_value = industrial_customer_acceptance_bundle.get("bundle_path")
    if not _is_non_empty_string(bundle_path_value):
        return _normalize_industrial_delivery_rehearsal_component(
            payload,
            default_status="blocked",
            default_summary="release ops execution is missing from the rehearsal report.",
            component_label="release ops execution",
        )

    resolved_bundle_path = Path(str(bundle_path_value).strip())
    embedded_component = industrial_customer_acceptance_bundle.get("release_ops_execution")
    if isinstance(embedded_component, Mapping):
        _hydrate_release_ops_execution_component(
            payload,
            component_source=embedded_component,
            bundle_path=str(resolved_bundle_path),
        )
        if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
            payload.get("summary")
        ):
            return _normalize_industrial_delivery_rehearsal_component(
                payload,
                default_status="blocked",
                default_summary="release ops execution is missing from the rehearsal report.",
                component_label="release ops execution",
            )
    if not resolved_bundle_path.is_absolute():
        resolved_bundle_path = (
            Path(release_rehearsal_report_path).resolve().parent / resolved_bundle_path
        )
    if not resolved_bundle_path.is_file():
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "release ops execution is missing from the industrial customer acceptance bundle: "
            f"{resolved_bundle_path}",
        )
        return to_jsonable(payload)

    try:
        bundle_payload = json.loads(resolved_bundle_path.read_text(encoding="utf-8"))
    except Exception as exc:
        payload.setdefault("bundle_path", str(resolved_bundle_path))
        payload.setdefault("status", "blocked")
        payload.setdefault(
            "summary",
            "release ops execution is unreadable from the industrial customer acceptance bundle: "
            f"{exc}",
        )
        return to_jsonable(payload)

    inline_component = bundle_payload.get("release_ops_execution")
    if isinstance(inline_component, Mapping):
        _hydrate_release_ops_execution_component(
            payload,
            component_source=inline_component,
            bundle_path=str(resolved_bundle_path),
        )
        if _is_non_empty_string(payload.get("status")) and _is_non_empty_string(
            payload.get("summary")
        ):
            return _normalize_industrial_delivery_rehearsal_component(
                payload,
                default_status="blocked",
                default_summary="release ops execution is missing from the rehearsal report.",
                component_label="release ops execution",
            )

    acceptance_reports = bundle_payload.get("acceptance_reports")
    if isinstance(acceptance_reports, list):
        for item in acceptance_reports:
            if not isinstance(item, Mapping):
                continue
            if item.get("name") != "release_ops_execution":
                continue
            _hydrate_release_ops_execution_component(
                payload,
                component_source=item,
                bundle_path=str(resolved_bundle_path),
            )
            break

    return _normalize_industrial_delivery_rehearsal_component(
        payload,
        default_status="blocked",
        default_summary="release ops execution is missing from the rehearsal report.",
        component_label="release ops execution",
    )


def _format_external_mainline_execution_plan_component_status(
    component: Mapping[str, Any]
) -> str:
    status = str(component.get("status") or "unknown").strip() or "unknown"
    counts: list[str] = []
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = _coerce_non_negative_int(component.get(field))
        if value is None:
            return status
        counts.append(str(value))
    return "/".join([status, *counts])


def _format_external_mainline_input_checklist_component_status(
    component: Mapping[str, Any]
) -> str:
    status = str(component.get("status") or "unknown").strip() or "unknown"
    missing_input_count = _coerce_non_negative_int(component.get("missing_input_count"))
    if missing_input_count is None:
        return status
    counts: list[str] = [str(missing_input_count)]
    for field in [
        "waiting_external_input_steps",
        "ready_to_run_steps",
        "completed_steps",
    ]:
        value = component.get(field)
        if isinstance(value, list):
            counts.append(
                str(
                    len(
                        [
                            item
                            for item in value
                            if _is_non_empty_string(item)
                        ]
                    )
                )
            )
            continue
        coerced = _coerce_non_negative_int(value)
        if coerced is None:
            return status
        counts.append(str(coerced))
    return "/".join([status, *counts])


def _format_release_ops_execution_component_status(component: Mapping[str, Any]) -> str:
    status = str(component.get("status") or "unknown").strip() or "unknown"
    event_count = _coerce_non_negative_int(component.get("event_count"))
    if event_count is None:
        return status
    return f"{status}/{event_count}"


def build_industrial_delivery_rehearsal_report_artifact(
    *,
    release_rehearsal_report: Mapping[str, Any],
    release_rehearsal_report_path: str | Path,
    generated_at: str | None = None,
) -> dict[str, Any]:
    if not isinstance(release_rehearsal_report, Mapping):
        raise ValueError("release_rehearsal_report must be an object")

    stages = _normalize_industrial_delivery_rehearsal_stages(
        release_rehearsal_report.get("delivery_rehearsal_stages")
    )
    stage_summary = {
        "total": len(stages),
        "passed": sum(1 for item in stages if item.get("status") == "pass"),
        "failed": sum(1 for item in stages if item.get("status") == "fail"),
    }

    security_release_preflight = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("security_release_preflight"),
        default_status="blocked",
        default_summary="security release preflight is missing from the rehearsal report.",
        component_label="security release preflight",
    )
    vulnerability_exception_review = (
        _normalize_vulnerability_exception_review_rehearsal_component(
            release_rehearsal_report.get("vulnerability_exception_review"),
            security_release_preflight=security_release_preflight,
        )
    )
    industrial_manifest = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("industrial_manifest"),
        default_status="blocked",
        default_summary="industrial manifest is missing from the rehearsal report.",
        component_label="industrial manifest",
    )
    industrial_release_readiness = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("industrial_release_readiness"),
        default_status="blocked",
        default_summary="industrial release readiness is missing from the rehearsal report.",
        component_label="industrial release readiness",
    )
    industrial_promotion_checklist = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("industrial_promotion_checklist"),
        default_status="blocked",
        default_summary="industrial promotion checklist is missing from the rehearsal report.",
        component_label="industrial promotion checklist",
    )
    industrial_customer_acceptance_bundle = (
        _normalize_industrial_delivery_rehearsal_component(
            release_rehearsal_report.get("industrial_customer_acceptance_bundle"),
            default_status="blocked",
            default_summary="industrial customer acceptance bundle is missing from the rehearsal report.",
            component_label="industrial customer acceptance bundle",
        )
    )
    customer_external_bindings_closure = (
        _normalize_customer_external_bindings_closure_rehearsal_component(
            release_rehearsal_report.get("customer_external_bindings_closure"),
            industrial_customer_acceptance_bundle=industrial_customer_acceptance_bundle,
            release_rehearsal_report_path=release_rehearsal_report_path,
        )
    )
    external_mainline_execution_plan = (
        _normalize_external_mainline_execution_plan_rehearsal_component(
            release_rehearsal_report.get("external_mainline_execution_plan"),
            industrial_customer_acceptance_bundle=industrial_customer_acceptance_bundle,
            release_rehearsal_report_path=release_rehearsal_report_path,
        )
    )
    external_mainline_input_checklist = (
        _normalize_external_mainline_input_checklist_rehearsal_component(
            release_rehearsal_report.get("external_mainline_input_checklist"),
            industrial_customer_acceptance_bundle=industrial_customer_acceptance_bundle,
            release_rehearsal_report_path=release_rehearsal_report_path,
        )
    )
    release_ops_execution = _normalize_release_ops_execution_rehearsal_component(
        release_rehearsal_report.get("release_ops_execution"),
        industrial_customer_acceptance_bundle=industrial_customer_acceptance_bundle,
        release_rehearsal_report_path=release_rehearsal_report_path,
    )
    extension_execution_plan = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("extension_execution_plan"),
        default_status="blocked",
        default_summary="extension execution plan is missing from the rehearsal report.",
        component_label="extension execution plan",
    )
    extension_execution_evidence = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("extension_execution_evidence"),
        default_status="blocked",
        default_summary="extension execution evidence is missing from the rehearsal report.",
        component_label="extension execution evidence",
    )
    extension_execution_instance = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("extension_execution_instance"),
        default_status="blocked",
        default_summary="extension execution instance is missing from the rehearsal report.",
        component_label="extension execution instance",
    )
    extension_execution_schedule = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("extension_execution_schedule"),
        default_status="blocked",
        default_summary="extension execution schedule is missing from the rehearsal report.",
        component_label="extension execution schedule",
    )
    extension_execution_actuals = _normalize_industrial_delivery_rehearsal_component(
        release_rehearsal_report.get("extension_execution_actuals"),
        default_status="blocked",
        default_summary="extension execution actuals are missing from the rehearsal report.",
        component_label="extension execution actuals",
    )
    control_plane_session, control_plane_event_stream = (
        _aggregate_release_op_control_plane_surface(
            release_rehearsal_report,
            release_ops_execution,
            external_mainline_execution_plan,
            external_mainline_input_checklist,
        )
    )

    release_rehearsal_status = str(
        release_rehearsal_report.get("status") or "failed"
    ).strip()
    release_gate_status = str(
        release_rehearsal_report.get("release_gate_status") or "blocked"
    ).strip()
    customer_delivery_status = str(
        release_rehearsal_report.get("customer_delivery_status") or "blocked"
    ).strip()
    industrial_delivery_status = str(
        release_rehearsal_report.get("industrial_delivery_status") or "blocked"
    ).strip()

    status = (
        "ready"
        if release_rehearsal_status == "passed"
        and release_gate_status == "ready"
        and customer_delivery_status == "ready"
        and industrial_delivery_status == "ready"
        and security_release_preflight.get("status") == "passed"
        and vulnerability_exception_review.get("status") == "passed"
        and external_mainline_execution_plan.get("status") == "ready"
        and industrial_manifest.get("status") == "ready"
        and industrial_release_readiness.get("status") == "ready"
        and industrial_promotion_checklist.get("status") == "ready"
        and industrial_customer_acceptance_bundle.get("status") == "ready"
        and extension_execution_plan.get("status") == "ready"
        and extension_execution_evidence.get("status") == "ready"
        and extension_execution_instance.get("status") == "ready"
        and extension_execution_schedule.get("status") == "ready"
        and extension_execution_actuals.get("status") == "ready"
        and stage_summary["failed"] == 0
        else "blocked"
    )
    summary = (
        f"Industrial delivery rehearsal {status}: "
        f"stages={stage_summary['passed']}/{stage_summary['total']}, "
        f"security_preflight={security_release_preflight.get('status')}, "
        "exception_review="
        f"{_format_industrial_delivery_component_status(vulnerability_exception_review)}, "
        "external_mainline="
        f"{_format_external_mainline_execution_plan_component_status(external_mainline_execution_plan)}, "
        "external_mainline_input_checklist="
        f"{_format_external_mainline_input_checklist_component_status(external_mainline_input_checklist)}, "
        "release_ops_execution="
        f"{_format_release_ops_execution_component_status(release_ops_execution)}, "
        f"industrial_manifest={industrial_manifest.get('status')}, "
        f"industrial_readiness={industrial_release_readiness.get('status')}, "
        f"industrial_promotion={industrial_promotion_checklist.get('status')}, "
        "industrial_customer_bundle="
        f"{industrial_customer_acceptance_bundle.get('status')}, "
        "external_bindings_closure="
        f"{customer_external_bindings_closure.get('status')}."
    )
    summary = _append_external_mainline_control_plane_summary(
        summary,
        control_plane_session=control_plane_session,
        control_plane_event_stream=control_plane_event_stream,
    )

    git_source = (
        dict(release_rehearsal_report.get("git_source", {}))
        if isinstance(release_rehearsal_report.get("git_source"), Mapping)
        else {}
    )
    industrial_delivery_artifact_paths = _normalize_industrial_delivery_artifact_paths(
        release_rehearsal_report.get("industrial_delivery_artifact_paths")
    )

    payload = {
        "schema_version": INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_VERSION,
        "artifact_type": INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_ARTIFACT_TYPE,
        "generated_at": generated_at or datetime.now().isoformat(),
        "status": status,
        "summary": summary,
        "version": release_rehearsal_report.get("version"),
        "tag": release_rehearsal_report.get("tag"),
        "source_root": release_rehearsal_report.get("source_root"),
        "source_commit_sha": git_source.get("commit_sha"),
        "release_rehearsal_status": release_rehearsal_status,
        "release_rehearsal_report_path": str(release_rehearsal_report_path),
        "release_gate_status": release_gate_status,
        "customer_delivery_status": customer_delivery_status,
        "industrial_delivery_status": industrial_delivery_status,
        "security_release_preflight": security_release_preflight,
        "vulnerability_exception_review": vulnerability_exception_review,
        "customer_external_bindings_closure": customer_external_bindings_closure,
        "external_mainline_execution_plan": external_mainline_execution_plan,
        "external_mainline_input_checklist": external_mainline_input_checklist,
        "release_ops_execution": release_ops_execution,
        "industrial_manifest": industrial_manifest,
        "industrial_release_readiness": industrial_release_readiness,
        "industrial_promotion_checklist": industrial_promotion_checklist,
        "industrial_customer_acceptance_bundle": industrial_customer_acceptance_bundle,
        "extension_execution_plan": extension_execution_plan,
        "extension_execution_evidence": extension_execution_evidence,
        "extension_execution_instance": extension_execution_instance,
        "extension_execution_schedule": extension_execution_schedule,
        "extension_execution_actuals": extension_execution_actuals,
        "delivery_rehearsal_stages": stages,
        "stage_summary": stage_summary,
        "industrial_delivery_artifact_paths": industrial_delivery_artifact_paths,
    }
    if control_plane_session:
        payload["control_plane_session"] = to_jsonable(control_plane_session)
    if control_plane_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(control_plane_event_stream)
    return to_jsonable(payload)


def validate_industrial_delivery_rehearsal_report_artifact(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["industrial delivery rehearsal report must be an object"]

    errors: list[str] = []
    missing = sorted(INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_VERSION:
        errors.append(
            "schema_version must be "
            f"{INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_ARTIFACT_TYPE:
        errors.append(
            "artifact_type must be "
            f"{INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_ARTIFACT_TYPE!r}"
        )
    if payload.get("status") not in INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_STATUSES:
        errors.append(
            "status must be one of "
            f"{sorted(INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_STATUSES)}"
        )
    if payload.get("release_gate_status") not in RELEASE_GATE_STATUSES:
        errors.append(
            f"release_gate_status must be one of {sorted(RELEASE_GATE_STATUSES)}"
        )
    if payload.get("customer_delivery_status") not in CUSTOMER_DELIVERY_SURFACE_STATUSES:
        errors.append(
            "customer_delivery_status must be one of "
            f"{sorted(CUSTOMER_DELIVERY_SURFACE_STATUSES)}"
        )
    if payload.get("industrial_delivery_status") not in INDUSTRIAL_DELIVERY_GATE_STATUSES:
        errors.append(
            "industrial_delivery_status must be one of "
            f"{sorted(INDUSTRIAL_DELIVERY_GATE_STATUSES)}"
        )
    for field in [
        "generated_at",
        "summary",
        "version",
        "tag",
        "source_root",
        "source_commit_sha",
        "release_rehearsal_status",
        "release_rehearsal_report_path",
    ]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append("control_plane_session must be an object")
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        f"control_plane_session.{field} must be a non-empty string when present"
                    )
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append("control_plane_event_stream must be an object")
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    "control_plane_event_stream.path must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    "control_plane_event_stream.event_count must be a non-negative integer when present"
                )

    for field_name, allowed_statuses in [
        ("security_release_preflight", RELEASE_EVIDENCE_STATUSES),
        ("vulnerability_exception_review", RELEASE_EVIDENCE_STATUSES),
        ("customer_external_bindings_closure", RELEASE_EVIDENCE_STATUSES),
        ("external_mainline_execution_plan", EXTERNAL_MAINLINE_EXECUTION_PLAN_STATUSES),
        ("external_mainline_input_checklist", RELEASE_EVIDENCE_STATUSES),
        ("release_ops_execution", RELEASE_EVIDENCE_STATUSES),
        ("industrial_manifest", RELEASE_GATE_STATUSES),
        ("industrial_release_readiness", RELEASE_GATE_STATUSES),
        ("industrial_promotion_checklist", CUSTOMER_ACCEPTANCE_BUNDLE_STATUSES),
        ("industrial_customer_acceptance_bundle", CUSTOMER_ACCEPTANCE_BUNDLE_STATUSES),
        ("extension_execution_plan", EXTENSION_EXECUTION_PLAN_STATUSES),
        ("extension_execution_evidence", EXTENSION_EXECUTION_EVIDENCE_STATUSES),
        ("extension_execution_instance", EXTENSION_EXECUTION_INSTANCE_STATUSES),
        ("extension_execution_schedule", EXTENSION_EXECUTION_SCHEDULE_STATUSES),
        ("extension_execution_actuals", EXTENSION_EXECUTION_ACTUALS_STATUSES),
    ]:
        errors.extend(
            _validate_industrial_delivery_rehearsal_component(
                field_name,
                payload.get(field_name),
                allowed_statuses=allowed_statuses,
            )
        )

    stages = payload.get("delivery_rehearsal_stages")
    if not isinstance(stages, list):
        errors.append("delivery_rehearsal_stages must be a list")
    else:
        observed_ids: list[str] = []
        for index, item in enumerate(stages, start=1):
            prefix = f"delivery_rehearsal_stages[{index}]"
            if not isinstance(item, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            missing_stage_fields = sorted(
                INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_FIELDS - set(item)
            )
            if missing_stage_fields:
                errors.append(
                    f"{prefix} missing required fields: {', '.join(missing_stage_fields)}"
                )
            stage_id = item.get("id")
            if stage_id not in INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS:
                errors.append(
                    f"{prefix}.id must be one of {list(INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS)}"
                )
            elif stage_id in observed_ids:
                errors.append(f"{prefix}.id duplicates stage {stage_id!r}")
            else:
                observed_ids.append(str(stage_id))
            if item.get("status") not in INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_STATUSES:
                errors.append(
                    f"{prefix}.status must be one of {sorted(INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_STATUSES)}"
                )
            if "summary" in item and not _is_non_empty_string(item.get("summary")):
                errors.append(f"{prefix}.summary must be a non-empty string")
            artifact_paths = item.get("artifact_paths")
            if not isinstance(artifact_paths, list):
                errors.append(f"{prefix}.artifact_paths must be a list")
            else:
                for path_index, path_value in enumerate(artifact_paths, start=1):
                    if not _is_non_empty_string(path_value):
                        errors.append(
                            f"{prefix}.artifact_paths[{path_index}] must be a non-empty string"
                        )
        if observed_ids != list(INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS):
            errors.append(
                "delivery_rehearsal_stages must contain exactly these ids in order: "
                + ", ".join(INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS)
            )

    stage_summary = payload.get("stage_summary")
    if not isinstance(stage_summary, Mapping):
        errors.append("stage_summary must be an object")
    else:
        missing_stage_summary = sorted(
            INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_SUMMARY_FIELDS - set(stage_summary)
        )
        if missing_stage_summary:
            errors.append(
                "stage_summary missing required fields: "
                + ", ".join(missing_stage_summary)
            )
        for field in INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_SUMMARY_FIELDS:
            if field in stage_summary and not _is_non_negative_int(stage_summary.get(field)):
                errors.append(f"stage_summary.{field} must be a non-negative integer")
        if (
            _is_non_negative_int(stage_summary.get("total"))
            and _is_non_negative_int(stage_summary.get("passed"))
            and _is_non_negative_int(stage_summary.get("failed"))
            and stage_summary.get("passed") + stage_summary.get("failed")
            != stage_summary.get("total")
        ):
            errors.append("stage_summary.passed + stage_summary.failed must equal stage_summary.total")

    artifact_paths = payload.get("industrial_delivery_artifact_paths")
    errors.extend(
        _validate_customer_acceptance_items_list(
            "industrial_delivery_artifact_paths",
            artifact_paths,
        )
    )

    return errors


def write_industrial_delivery_rehearsal_report_artifact(
    payload: Mapping[str, Any], path: str | Path
) -> Path:
    errors = validate_industrial_delivery_rehearsal_report_artifact(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def build_release_evidence_report(
    *,
    evidence_name: str,
    status: str,
    summary: str,
    command: str,
    generated_at: str | None = None,
    exit_code: int | None = None,
    duration_seconds: float | int | None = None,
    metrics: Mapping[str, Any] | None = None,
    stdout_path: str | None = None,
    stderr_path: str | None = None,
    source_commit_sha: str | None = None,
    control_plane_session: Mapping[str, Any] | None = None,
    control_plane_event_stream: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    payload = {
        "schema_version": RELEASE_EVIDENCE_REPORT_VERSION,
        "artifact_type": RELEASE_EVIDENCE_REPORT_ARTIFACT_TYPE,
        "evidence_name": evidence_name,
        "status": status,
        "summary": summary,
        "command": command,
        "generated_at": generated_at or datetime.now().isoformat(),
        "exit_code": exit_code,
        "duration_seconds": duration_seconds,
        "metrics": to_jsonable(dict(metrics)) if metrics is not None else {},
        "stdout_path": stdout_path,
        "stderr_path": stderr_path,
        "source_commit_sha": source_commit_sha,
    }
    normalized_session = _normalize_release_op_session_context_summary(
        control_plane_session
    )
    if normalized_session:
        payload["control_plane_session"] = to_jsonable(normalized_session)
    normalized_event_stream = _normalize_release_op_event_stream_summary(
        control_plane_event_stream
    )
    if normalized_event_stream:
        payload["control_plane_event_stream"] = to_jsonable(normalized_event_stream)
    return payload


def validate_release_evidence_report(payload: Any) -> list[str]:
    if not isinstance(payload, Mapping):
        return ["release evidence report must be an object"]

    errors: list[str] = []
    missing = sorted(RELEASE_EVIDENCE_REPORT_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != RELEASE_EVIDENCE_REPORT_VERSION:
        errors.append(
            f"schema_version must be {RELEASE_EVIDENCE_REPORT_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != RELEASE_EVIDENCE_REPORT_ARTIFACT_TYPE:
        errors.append(
            f"artifact_type must be {RELEASE_EVIDENCE_REPORT_ARTIFACT_TYPE!r}"
        )
    if payload.get("status") not in RELEASE_EVIDENCE_STATUSES:
        errors.append(
            f"status must be one of {sorted(RELEASE_EVIDENCE_STATUSES)}"
        )
    for field in ["evidence_name", "summary", "command", "generated_at"]:
        if field in payload and not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if "exit_code" in payload and payload.get("exit_code") is not None:
        if not isinstance(payload.get("exit_code"), int) or isinstance(
            payload.get("exit_code"), bool
        ):
            errors.append("exit_code must be null or an integer")
    if "duration_seconds" in payload and payload.get("duration_seconds") is not None:
        duration_seconds = payload.get("duration_seconds")
        if not isinstance(duration_seconds, (int, float)) or isinstance(
            duration_seconds, bool
        ):
            errors.append("duration_seconds must be null or a number")
        elif duration_seconds < 0:
            errors.append("duration_seconds must be non-negative")
    if "metrics" in payload and not isinstance(payload.get("metrics"), Mapping):
        errors.append("metrics must be an object")
    for field in ["stdout_path", "stderr_path", "source_commit_sha"]:
        if field in payload and payload.get(field) is not None:
            if not _is_non_empty_string(payload.get(field)):
                errors.append(f"{field} must be null or a non-empty string")
    control_plane_session = payload.get("control_plane_session")
    if control_plane_session is not None:
        if not isinstance(control_plane_session, Mapping):
            errors.append("control_plane_session must be an object")
        else:
            for field in ["engagement_id", "window_id", "change_ticket", "channel"]:
                if field in control_plane_session and not _is_non_empty_string(
                    control_plane_session.get(field)
                ):
                    errors.append(
                        f"control_plane_session.{field} must be a non-empty string when present"
                    )
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if control_plane_event_stream is not None:
        if not isinstance(control_plane_event_stream, Mapping):
            errors.append("control_plane_event_stream must be an object")
        else:
            if not _is_non_empty_string(control_plane_event_stream.get("path")):
                errors.append(
                    "control_plane_event_stream.path must be a non-empty string"
                )
            if "event_count" in control_plane_event_stream and not _is_non_negative_int(
                control_plane_event_stream.get("event_count")
            ):
                errors.append(
                    "control_plane_event_stream.event_count must be a non-negative integer when present"
                )
    return errors


def write_release_evidence_report(payload: Mapping[str, Any], path: str | Path) -> Path:
    errors = validate_release_evidence_report(payload)
    if errors:
        raise ValueError("; ".join(errors))

    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(to_jsonable(dict(payload)), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return output_path


def _resolve_release_artifact_path(artifact_path: str, project_root: Path) -> Path:
    path = Path(artifact_path)
    if path.is_absolute():
        return path
    return project_root / path


def _project_relative_artifact_path(
    artifact_path: Any,
    *,
    project_root: Path,
) -> str | None:
    raw = _artifact_optional_string(artifact_path)
    if not raw:
        return None
    path = Path(raw)
    if not path.is_absolute():
        return path.as_posix()
    try:
        return path.resolve().relative_to(project_root.resolve()).as_posix()
    except ValueError:
        return str(path)


def _normalize_industrial_delivery_rehearsal_component(
    component: Any,
    *,
    default_status: str,
    default_summary: str,
    component_label: str,
) -> dict[str, Any]:
    payload = dict(component) if isinstance(component, Mapping) else {}
    if not _is_non_empty_string(payload.get("status")):
        payload["status"] = default_status
    if not _is_non_empty_string(payload.get("summary")):
        payload["summary"] = (
            f"{component_label} status={payload.get('status')}."
            if payload
            else default_summary
        )
    return to_jsonable(payload)


def _normalize_industrial_delivery_rehearsal_stages(stages: Any) -> list[dict[str, Any]]:
    stages_by_id: dict[str, Mapping[str, Any]] = {}
    if isinstance(stages, list):
        for item in stages:
            if not isinstance(item, Mapping):
                continue
            stage_id = item.get("id")
            if stage_id in INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS and stage_id not in stages_by_id:
                stages_by_id[str(stage_id)] = item

    normalized: list[dict[str, Any]] = []
    for stage_id in INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_IDS:
        item = stages_by_id.get(stage_id, {})
        artifact_paths = item.get("artifact_paths")
        normalized.append(
            {
                "id": stage_id,
                "status": (
                    item.get("status")
                    if item.get("status") in INDUSTRIAL_DELIVERY_REHEARSAL_STAGE_STATUSES
                    else "fail"
                ),
                "summary": (
                    str(item.get("summary")).strip()
                    if _is_non_empty_string(item.get("summary"))
                    else f"{stage_id} stage is missing from the release rehearsal report."
                ),
                "artifact_paths": [
                    str(path).strip()
                    for path in artifact_paths
                    if _is_non_empty_string(path)
                ]
                if isinstance(artifact_paths, list)
                else [],
            }
        )
    return normalized


def _normalize_industrial_delivery_artifact_paths(paths: Any) -> list[dict[str, Any]]:
    normalized: list[dict[str, Any]] = []
    if not isinstance(paths, list):
        return normalized
    for item in paths:
        if not isinstance(item, Mapping):
            continue
        name = item.get("name")
        path = item.get("path")
        if not (_is_non_empty_string(name) and _is_non_empty_string(path)):
            continue
        normalized.append(
            {
                "name": str(name).strip(),
                "path": str(path).strip(),
                "required": True,
                "exists": True,
            }
        )
    return normalized


def _validate_industrial_delivery_rehearsal_component(
    field_name: str,
    component: Any,
    *,
    allowed_statuses: set[str],
) -> list[str]:
    if not isinstance(component, Mapping):
        return [f"{field_name} must be an object"]
    errors: list[str] = []
    missing = sorted(INDUSTRIAL_DELIVERY_REHEARSAL_COMPONENT_FIELDS - set(component))
    if missing:
        errors.append(f"{field_name} missing required fields: {', '.join(missing)}")
    if component.get("status") not in allowed_statuses:
        errors.append(
            f"{field_name}.status must be one of {sorted(allowed_statuses)}"
        )
    if "summary" in component and not _is_non_empty_string(component.get("summary")):
        errors.append(f"{field_name}.summary must be a non-empty string")
    return errors


def _validate_customer_acceptance_items_list(
    list_name: str,
    items: Any,
) -> list[str]:
    errors: list[str] = []
    if not isinstance(items, list):
        return [f"{list_name} must be a list"]
    for index, item in enumerate(items, start=1):
        prefix = f"{list_name}[{index}]"
        if not isinstance(item, Mapping):
            errors.append(f"{prefix} must be an object")
            continue
        missing_fields = sorted(CUSTOMER_ACCEPTANCE_ITEM_FIELDS - set(item))
        if missing_fields:
            errors.append(
                f"{prefix} missing required fields: {', '.join(missing_fields)}"
            )
        for field in ["name", "path"]:
            if field in item and not _is_non_empty_string(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-empty string")
        if "required" in item and not isinstance(item.get("required"), bool):
            errors.append(f"{prefix}.required must be a boolean")
        if "exists" in item and not isinstance(item.get("exists"), bool):
            errors.append(f"{prefix}.exists must be a boolean")
    return errors


def _hydrate_customer_acceptance_items(
    items: Sequence[Mapping[str, Any]], project_root: Path
) -> list[dict[str, Any]]:
    hydrated: list[dict[str, Any]] = []
    for item in items:
        path_value = str(item.get("path", ""))
        resolved_path = _resolve_release_artifact_path(path_value, project_root)
        current = {
            "name": item.get("name"),
            "path": path_value,
            "required": item.get("required") is True,
            "exists": resolved_path.is_file(),
        }
        if resolved_path.suffix.lower() == ".json":
            current.update(
                _summarize_customer_acceptance_report(current["name"], resolved_path)
            )
        hydrated.append(current)
    return hydrated


def _summarize_customer_acceptance_report(name: Any, report_path: Path) -> dict[str, Any]:
    if not report_path.is_file():
        return {}

    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {
            "status": "invalid",
            "summary": f"Report could not be parsed: {exc}",
        }

    if not isinstance(payload, Mapping):
        return {
            "status": "invalid",
            "summary": "Report payload must be a JSON object.",
        }

    artifact_type = payload.get("artifact_type")
    if artifact_type == RELEASE_EVIDENCE_REPORT_ARTIFACT_TYPE:
        expected_name = str(name) if _is_non_empty_string(name) else None
        if expected_name is None:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": "Structured release evidence report is missing an expected name binding.",
            }
        return _summarize_structured_release_evidence(
            payload,
            expected_name=expected_name,
            report_path=report_path,
        )
    if artifact_type == SECURITY_POSTURE_REPORT_ARTIFACT_TYPE:
        errors = validate_security_posture_report(payload)
        if errors:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": f"Security posture report is invalid: {'; '.join(errors)}",
            }
        exception_report = payload.get("vulnerability_exception_report", {})
        return {
            "artifact_type": artifact_type,
            "status": payload.get("posture_status"),
            "summary": payload.get("summary"),
            "blocked_vulnerability_reports": payload.get("blocked_vulnerability_reports"),
            "blocked_vulnerability_execution_reports": payload.get(
                "blocked_vulnerability_execution_reports"
            ),
            "accepted_vulnerability_findings": payload.get(
                "accepted_vulnerability_findings"
            ),
            "unresolved_vulnerability_findings": payload.get(
                "unresolved_vulnerability_findings"
            ),
            "active_exception_count": (
                exception_report.get("active_exception_count")
                if isinstance(exception_report, Mapping)
                else None
            ),
            "expired_exception_count": (
                exception_report.get("expired_exception_count")
                if isinstance(exception_report, Mapping)
                else None
            ),
            "review_due_exception_count": (
                exception_report.get("review_due_exception_count")
                if isinstance(exception_report, Mapping)
                else None
            ),
            "stale_exception_count": (
                exception_report.get("stale_exception_count")
                if isinstance(exception_report, Mapping)
                else None
            ),
            "stale_exception_ids": (
                exception_report.get("stale_exception_ids")
                if isinstance(exception_report, Mapping)
                else None
            ),
            "next_exception_expiry": (
                exception_report.get("next_exception_expiry")
                if isinstance(exception_report, Mapping)
                else None
            ),
            "exception_review_status": (
                exception_report.get("review_status")
                if isinstance(exception_report, Mapping)
                else None
            ),
        }
    if artifact_type == VULNERABILITY_REMEDIATION_REPORT_ARTIFACT_TYPE:
        errors = validate_vulnerability_remediation_report(payload)
        if errors:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": "Vulnerability remediation report is invalid: "
                + "; ".join(errors),
            }
        return {
            "artifact_type": artifact_type,
            "status": payload.get("remediation_status"),
            "summary": payload.get("summary"),
            "accepted_finding_count": payload.get("accepted_finding_count"),
            "unresolved_finding_count": payload.get("unresolved_finding_count"),
            "stale_exception_count": (
                payload.get("vulnerability_exception_report", {}).get(
                    "stale_exception_count"
                )
                if isinstance(payload.get("vulnerability_exception_report"), Mapping)
                else None
            ),
        }
    if artifact_type == SBOM_ARTIFACT_TYPE:
        errors = validate_sbom_artifact(payload)
        if errors:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": f"SBOM artifact is invalid: {'; '.join(errors)}",
            }
        component_count = payload.get("component_count", 0)
        return {
            "artifact_type": artifact_type,
            "status": "ready",
            "summary": f"SBOM artifact present with {component_count} component(s).",
        }
    if artifact_type == VULNERABILITY_SCAN_REPORT_ARTIFACT_TYPE:
        errors = validate_vulnerability_scan_report(payload)
        if errors:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": f"Vulnerability scan report is invalid: {'; '.join(errors)}",
            }
        return {
            "artifact_type": artifact_type,
            "status": payload.get("status"),
            "summary": payload.get("summary"),
        }
    if artifact_type == BACKUP_RESTORE_REHEARSAL_REPORT_ARTIFACT_TYPE:
        errors = validate_backup_restore_rehearsal_report(payload)
        if errors:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": "Backup restore rehearsal report is invalid: "
                + "; ".join(errors),
            }
        return {
            "artifact_type": artifact_type,
            "status": payload.get("status"),
            "summary": payload.get("summary"),
        }
    if artifact_type == INDUSTRIAL_DELIVERY_REHEARSAL_REPORT_ARTIFACT_TYPE:
        errors = validate_industrial_delivery_rehearsal_report_artifact(payload)
        if errors:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": "Industrial delivery rehearsal report is invalid: "
                + "; ".join(errors),
            }
        return {
            "artifact_type": artifact_type,
            "status": payload.get("status"),
            "summary": payload.get("summary"),
            "stage_summary": payload.get("stage_summary"),
            "release_rehearsal_status": payload.get("release_rehearsal_status"),
        }
    if artifact_type == EXTERNAL_MAINLINE_EXECUTION_PLAN_ARTIFACT_TYPE:
        errors = validate_external_mainline_execution_plan_artifact(payload)
        if errors:
            return {
                "artifact_type": artifact_type,
                "status": "invalid",
                "summary": "External mainline execution plan is invalid: "
                + "; ".join(errors),
            }
        return {
            "artifact_type": artifact_type,
            "status": payload.get("status"),
            "summary": payload.get("summary"),
            "completed_steps": payload.get("completed_steps"),
            "ready_to_run_steps": payload.get("ready_to_run_steps"),
            "waiting_external_input_steps": payload.get("waiting_external_input_steps"),
            "blocked_steps": payload.get("blocked_steps"),
            "control_plane_session": payload.get("control_plane_session"),
            "control_plane_event_stream": payload.get("control_plane_event_stream"),
        }

    if _is_non_empty_string(str(name)) and str(name) == "release_readiness":
        return {
            "status": payload.get("stable_release_gate") or payload.get("rc_release_gate"),
            "summary": payload.get("summary")
            or (
                f"rc={payload.get('rc_release_gate')}, stable={payload.get('stable_release_gate')}"
                if payload.get("stable_release_gate") is not None
                else None
            ),
        }
    if _is_non_empty_string(str(name)) and str(name) == "industrial_release_readiness":
        return {
            "status": payload.get("industrial_release_gate"),
            "summary": payload.get("summary")
            or (
                "industrial="
                + str(payload.get("industrial_release_gate"))
                if payload.get("industrial_release_gate") is not None
                else None
            ),
        }
    if _is_non_empty_string(str(name)) and str(name) == "stable_promotion_checklist":
        return {
            "status": "ready" if payload.get("ready_to_promote") else "blocked",
            "summary": payload.get("summary")
            or (
                f"blocking_steps={payload.get('blocking_steps')}"
                if payload.get("blocking_steps") is not None
                else None
            ),
        }
    if _is_non_empty_string(str(name)) and str(name) == "industrial_promotion_checklist":
        return {
            "status": "ready" if payload.get("ready_to_promote") else "blocked",
            "summary": payload.get("summary")
            or (
                f"blocking_steps={payload.get('blocking_steps')}"
                if payload.get("blocking_steps") is not None
                else None
            ),
        }
    return {}


def _load_release_evidence_from_report(name: str, report_path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {
            "status": "blocked",
            "summary": f"Report could not be parsed: {exc}",
        }

    if payload.get("artifact_type") == RELEASE_EVIDENCE_REPORT_ARTIFACT_TYPE:
        return _summarize_structured_release_evidence(
            payload,
            expected_name=name,
            report_path=report_path,
        )
    if name == "clean_checkout_smoke":
        return _summarize_clean_checkout_smoke_evidence(payload, report_path=report_path)
    if name == "distributed_runtime_live":
        return _summarize_distributed_release_evidence(payload)
    if name == "godot_headless_live":
        return _summarize_live_smoke_evidence(
            payload,
            passed_message="Godot headless smoke passed with structured report evidence.",
        )
    if name == "ros2_bridge_live":
        return _summarize_live_smoke_evidence(
            payload,
            passed_message="ROS2 bridge smoke passed with structured report evidence.",
        )
    return {}


def _summarize_structured_release_evidence(
    payload: Mapping[str, Any], *, expected_name: str, report_path: Path
) -> dict[str, Any]:
    errors = validate_release_evidence_report(payload)
    if payload.get("evidence_name") != expected_name:
        errors.append(
            f"evidence_name must be {expected_name!r}, got {payload.get('evidence_name')!r}"
        )
    if errors:
        return {
            "status": "blocked",
            "summary": f"Structured evidence report is invalid: {'; '.join(errors)}",
            "attested": False,
        }

    return {
        "status": payload.get("status"),
        "summary": payload.get("summary"),
        "command": payload.get("command"),
        "attested": True,
        "observed_at": payload.get("generated_at"),
        "source_commit_sha": payload.get("source_commit_sha"),
        "metrics": payload.get("metrics", {}),
        "stdout_path": payload.get("stdout_path"),
        "stderr_path": payload.get("stderr_path"),
        "resolved_report_path": str(report_path),
        "control_plane_session": payload.get("control_plane_session"),
        "control_plane_event_stream": payload.get("control_plane_event_stream"),
    }


def _summarize_clean_checkout_smoke_evidence(
    payload: Mapping[str, Any], *, report_path: Path
) -> dict[str, Any]:
    if payload.get("artifact_type") != "clean_checkout_smoke_report":
        return {
            "status": "blocked",
            "summary": "clean checkout smoke report has an unexpected artifact_type.",
            "attested": False,
        }

    status = str(payload.get("status", "")).strip().lower()
    generated_at = payload.get("generated_at")
    runs = payload.get("runs")
    command_template = payload.get("command_template")
    checkout_commit_sha = payload.get("checkout_commit_sha")
    run_reports = payload.get("run_reports")
    failure_reason = payload.get("failure_reason")
    checks = payload.get("checks")

    if status not in {"passed", "blocked"}:
        return {
            "status": "blocked",
            "summary": f"clean checkout smoke report has invalid status {status!r}.",
            "attested": False,
        }
    if not isinstance(runs, int) or runs <= 0:
        return {
            "status": "blocked",
            "summary": "clean checkout smoke report must record a positive runs count.",
            "attested": False,
        }
    if not isinstance(run_reports, list) or len(run_reports) != runs:
        return {
            "status": "blocked",
            "summary": "clean checkout smoke report must include one run_report per run.",
            "attested": False,
        }
    if not isinstance(checks, list) or not checks:
        return {
            "status": "blocked",
            "summary": "clean checkout smoke report must include structured checks.",
            "attested": False,
        }

    clean_runs = sum(
        1
        for item in run_reports
        if isinstance(item, Mapping)
        and item.get("status") == "passed"
        and item.get("worktree_clean") is True
    )
    if status == "passed" and clean_runs != runs:
        return {
            "status": "blocked",
            "summary": "clean checkout smoke reported passed, but not every run left the worktree clean.",
            "attested": False,
        }

    command = (
        " ".join(str(item) for item in command_template)
        if isinstance(command_template, list) and command_template
        else "python tools/run_clean_checkout_smoke.py"
    )
    metrics = {
        "runs": runs,
        "clean_runs": clean_runs,
        "checks": len(checks),
    }
    if status == "passed":
        summary = (
            f"Clean checkout smoke passed: {runs} sequential smoke run(s) completed "
            "with clean git status after each run."
        )
    else:
        summary = (
            f"Clean checkout smoke blocked: {failure_reason}"
            if _is_non_empty_string(failure_reason)
            else "Clean checkout smoke blocked."
        )

    return {
        "status": status,
        "summary": summary,
        "command": command,
        "attested": True,
        "observed_at": generated_at,
        "source_commit_sha": checkout_commit_sha,
        "metrics": metrics,
        "resolved_report_path": str(report_path),
    }


def _summarize_distributed_release_evidence(payload: Mapping[str, Any]) -> dict[str, Any]:
    status = str(payload.get("status", "")).strip().lower()
    failed_check = payload.get("failed_check")
    checks = payload.get("checks", [])
    passed_checks = sum(1 for item in checks if item.get("status") == "pass")
    total_checks = len(checks)
    actor_id = payload.get("actor_id")

    if status == "passed":
        return {
            "status": "passed",
            "summary": (
                f"Distributed smoke passed: {passed_checks}/{total_checks} checks passed"
                + (f" for actor {actor_id}." if _is_non_empty_string(actor_id) else ".")
            ),
        }

    detail = payload.get("diagnostics", {}).get("detail")
    if status == "failed":
        return {
            "status": "blocked",
            "summary": (
                f"Distributed smoke failed at {failed_check or 'unknown'}"
                + (f": {detail}" if _is_non_empty_string(detail) else ".")
            ),
        }

    return {
        "status": "blocked",
        "summary": f"Distributed smoke report returned unsupported status {status!r}.",
    }


def _summarize_live_smoke_evidence(
    payload: Mapping[str, Any], *, passed_message: str
) -> dict[str, Any]:
    status = str(payload.get("status", "")).strip().lower()
    failure_stage = payload.get("failure_stage")
    message = payload.get("message")

    if status == "passed":
        return {"status": "passed", "summary": passed_message}
    if status == "skipped":
        return {
            "status": "opt_in",
            "summary": message or "Live smoke was skipped because its environment gate was not enabled.",
        }
    if status == "failed":
        suffix = f" at {failure_stage}" if _is_non_empty_string(failure_stage) else ""
        return {
            "status": "blocked",
            "summary": f"Live smoke failed{suffix}: {message or 'see structured report'}",
        }
    return {
        "status": "blocked",
        "summary": f"Live smoke report returned unsupported status {status!r}.",
    }


def _default_known_limitations(capability_matrix: Mapping[str, Any]) -> list[str]:
    limitations: list[str] = []
    for item in capability_matrix.get("known_limitations", []):
        if _is_non_empty_string(item) and item not in limitations:
            limitations.append(item)
    for domain in capability_matrix.get("domains", []):
        for item in domain.get("known_limitations", []):
            if _is_non_empty_string(item) and item not in limitations:
                limitations.append(item)
    return limitations


def _build_release_gate(
    evidence: Sequence[Mapping[str, Any]],
    capability_matrix: Mapping[str, Any],
    release_approval: Mapping[str, Any],
    release_source: Mapping[str, Any],
    release_policy: Mapping[str, Any],
    customer_delivery_surface: Mapping[str, Any],
    industrial_delivery_gate: Mapping[str, Any],
) -> dict[str, int]:
    required_evidence = sum(1 for item in evidence if item.get("required") is True)
    passed_required_evidence = sum(
        1
        for item in evidence
        if item.get("required") is True and item.get("status") == "passed"
    )
    blocked_evidence = sum(1 for item in evidence if item.get("status") == "blocked")
    blocked_optional_evidence = sum(
        1
        for item in evidence
        if item.get("required") is not True and item.get("status") == "blocked"
    )
    opt_in_evidence = sum(1 for item in evidence if item.get("status") == "opt_in")
    summary = capability_matrix.get("summary", {})
    diagnostic_ready_domains = (
        summary.get("diagnostic_ready_domains")
        if isinstance(summary.get("diagnostic_ready_domains"), int)
        else 0
    )
    release_approval_required = 1 if release_approval.get("required") else 0
    release_approval_ready = 1 if _is_release_approval_ready(release_approval) else 0
    release_source_required = 1 if release_approval.get("required") else 0
    release_source_ready = (
        1 if _is_release_source_ready(release_source, release_approval) else 0
    )
    release_worktree_required = (
        1 if release_policy.get("requires_clean_worktree", False) else 0
    )
    release_worktree_ready = (
        1 if _is_release_worktree_ready(release_source, release_policy) else 0
    )
    release_version_tag_required = 1 if release_approval.get("required") else 0
    release_version_tag_ready = (
        1 if _is_release_version_tag_ready(release_source, release_approval) else 0
    )
    customer_delivery_required = (
        1 if release_policy.get("requires_customer_delivery_surface", False) else 0
    )
    customer_delivery_ready = (
        1 if customer_delivery_surface.get("status") == "ready" else 0
    )
    industrial_delivery_required = (
        1 if release_policy.get("requires_industrial_delivery_gate", False) else 0
    )
    industrial_delivery_ready = (
        1 if industrial_delivery_gate.get("status") == "ready" else 0
    )
    return {
        "required_evidence": required_evidence,
        "passed_required_evidence": passed_required_evidence,
        "blocked_evidence": blocked_evidence,
        "blocked_optional_evidence": blocked_optional_evidence,
        "opt_in_evidence": opt_in_evidence,
        "diagnostic_ready_domains": diagnostic_ready_domains,
        "release_approval_required": release_approval_required,
        "release_approval_ready": release_approval_ready,
        "release_source_required": release_source_required,
        "release_source_ready": release_source_ready,
        "release_worktree_required": release_worktree_required,
        "release_worktree_ready": release_worktree_ready,
        "release_version_tag_required": release_version_tag_required,
        "release_version_tag_ready": release_version_tag_ready,
        "customer_delivery_required": customer_delivery_required,
        "customer_delivery_ready": customer_delivery_ready,
        "industrial_delivery_required": industrial_delivery_required,
        "industrial_delivery_ready": industrial_delivery_ready,
    }


def _build_capability_summary(domains: Sequence[Mapping[str, Any]]) -> dict[str, int]:
    ready_domains = sum(1 for item in domains if item.get("status") == "ready")
    diagnostic_ready_domains = sum(
        1 for item in domains if item.get("status") == "diagnostic_ready"
    )
    return {
        "total_domains": len(domains),
        "ready_domains": ready_domains,
        "diagnostic_ready_domains": diagnostic_ready_domains,
    }


def _build_release_policy(channel: str) -> dict[str, Any]:
    if channel == "dev":
        return {
            "channel": "dev",
            "allows_opt_in_evidence": True,
            "allows_diagnostic_ready_domains": True,
            "requires_release_approval": False,
            "requires_git_source_binding": False,
            "requires_clean_worktree": False,
            "requires_version_tag_match": False,
            "requires_customer_delivery_surface": False,
            "requires_industrial_delivery_gate": False,
            "summary": "Dev releases may remain ready when required evidence passes, even if optional live evidence or diagnostic-ready domains are still open.",
        }
    if channel == "rc":
        return {
            "channel": "rc",
            "allows_opt_in_evidence": False,
            "allows_diagnostic_ready_domains": False,
            "requires_release_approval": False,
            "requires_git_source_binding": False,
            "requires_clean_worktree": False,
            "requires_version_tag_match": False,
            "requires_customer_delivery_surface": False,
            "requires_industrial_delivery_gate": False,
            "summary": "RC releases require optional live evidence and diagnostic-ready domains to be fully closed before the gate becomes ready.",
        }
    if channel == "stable":
        return {
            "channel": "stable",
            "allows_opt_in_evidence": False,
            "allows_diagnostic_ready_domains": False,
            "requires_release_approval": True,
            "requires_git_source_binding": True,
            "requires_clean_worktree": True,
            "requires_version_tag_match": True,
            "requires_customer_delivery_surface": False,
            "requires_industrial_delivery_gate": False,
            "summary": "Stable releases require optional live evidence, diagnostic-ready domains, explicit release approval, Git HEAD binding, a clean worktree, and a matching version tag before the gate becomes ready.",
        }
    return {
        "channel": "industrial",
        "allows_opt_in_evidence": False,
        "allows_diagnostic_ready_domains": False,
        "requires_release_approval": True,
        "requires_git_source_binding": True,
        "requires_clean_worktree": True,
        "requires_version_tag_match": True,
        "requires_customer_delivery_surface": True,
        "requires_industrial_delivery_gate": True,
        "summary": "Industrial releases require the full stable gate plus ready customer delivery and industrial delivery surfaces before the gate becomes ready.",
    }


def _build_release_approval(
    channel: str,
    release_approval: Mapping[str, Any] | None = None,
    *,
    release_source: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    approval_required = channel in {"stable", "industrial"}
    base = {
        "status": "pending" if approval_required else "not_required",
        "required": approval_required,
        "approved_by": None,
        "approved_at": None,
        "commit_sha": None,
        "notes": None,
    }
    if release_approval is None:
        payload = dict(base)
    else:
        payload = dict(base)
        payload.update(dict(release_approval))

    if (
        payload.get("status") == "approved"
        and not _is_non_empty_string(payload.get("commit_sha"))
        and isinstance(release_source, Mapping)
        and _is_non_empty_string(release_source.get("commit_sha"))
    ):
        payload["commit_sha"] = release_source.get("commit_sha")
    return payload


def _is_release_approval_ready(release_approval: Mapping[str, Any]) -> bool:
    if not release_approval.get("required"):
        return True
    if release_approval.get("status") != "approved":
        return False
    return all(
        _is_non_empty_string(release_approval.get(field))
        for field in ["approved_by", "approved_at", "commit_sha"]
    )


def _build_release_source(
    *,
    version: str,
    release_source: Mapping[str, Any] | None = None,
    source_root: str | Path | None = None,
) -> dict[str, Any]:
    payload = {
        "resolved_from_git": False,
        "commit_sha": None,
        "short_commit_sha": None,
        "git_tag": None,
        "matched_version_tag": None,
        "worktree_clean": False,
        "worktree_status_summary": None,
        "version_tag_matches": False,
    }
    payload.update(_resolve_git_release_source(source_root, version=version))
    if release_source is not None:
        payload.update(dict(release_source))
    return payload


def _resolve_git_release_source(
    source_root: str | Path | None,
    *,
    version: str,
) -> dict[str, Any]:
    root = Path(source_root) if source_root is not None else Path.cwd()
    commit_sha = _run_git_capture(["git", "rev-parse", "HEAD"], cwd=root)
    short_commit_sha = _run_git_capture(["git", "rev-parse", "--short=12", "HEAD"], cwd=root)
    tags_output = _run_git_capture(["git", "tag", "--points-at", "HEAD"], cwd=root)
    tags = [line.strip() for line in (tags_output or "").splitlines() if line.strip()]
    worktree_output = _run_git_capture(["git", "status", "--short"], cwd=root)
    worktree_lines = [
        line.strip() for line in (worktree_output or "").splitlines() if line.strip()
    ]
    matched_version_tag = _match_version_tag(version, tags)
    git_tag = None
    if matched_version_tag is not None:
        git_tag = matched_version_tag
    elif tags:
        git_tag = tags[0]
    if not (_is_non_empty_string(commit_sha) and _is_non_empty_string(short_commit_sha)):
        return {}
    return {
        "resolved_from_git": True,
        "commit_sha": commit_sha,
        "short_commit_sha": short_commit_sha,
        "git_tag": git_tag,
        "matched_version_tag": matched_version_tag,
        "worktree_clean": not worktree_lines,
        "worktree_status_summary": _summarize_git_worktree(worktree_lines),
        "version_tag_matches": matched_version_tag is not None,
    }


def _run_git_capture(argv: Sequence[str], *, cwd: Path) -> str | None:
    try:
        result = subprocess.run(
            list(argv),
            cwd=str(cwd),
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            check=False,
        )
    except Exception:
        return None
    if result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value or None


def _is_release_source_ready(
    release_source: Mapping[str, Any],
    release_approval: Mapping[str, Any],
) -> bool:
    if not release_approval.get("required"):
        return True
    if not release_source.get("resolved_from_git"):
        return False
    return _release_approval_matches_source(release_approval, release_source)


def _is_release_worktree_ready(
    release_source: Mapping[str, Any],
    release_policy: Mapping[str, Any],
) -> bool:
    if not release_policy.get("requires_clean_worktree", False):
        return True
    if not release_source.get("resolved_from_git"):
        return False
    return bool(release_source.get("worktree_clean"))


def _is_release_version_tag_ready(
    release_source: Mapping[str, Any],
    release_approval: Mapping[str, Any],
) -> bool:
    if not release_approval.get("required"):
        return True
    return bool(release_source.get("version_tag_matches"))


def _match_version_tag(version: str, tags: Sequence[str]) -> str | None:
    expected_tags = {version, f"v{version}"}
    for tag in tags:
        if tag in expected_tags:
            return tag
    return None


def _summarize_git_worktree(lines: Sequence[str]) -> str | None:
    if not lines:
        return None
    preview = ", ".join(lines[:3])
    if len(lines) > 3:
        preview = f"{preview}, ..."
    return f"{len(lines)} pending path(s): {preview}"


def _release_approval_matches_source(
    release_approval: Mapping[str, Any],
    release_source: Mapping[str, Any],
) -> bool:
    approval_commit = release_approval.get("commit_sha")
    source_commit = release_source.get("commit_sha")
    source_short_commit = release_source.get("short_commit_sha")
    if not (
        _is_non_empty_string(approval_commit)
        and _is_non_empty_string(source_commit)
        and _is_non_empty_string(source_short_commit)
    ):
        return False

    normalized_commit = str(approval_commit).strip()
    normalized_source = str(source_commit).strip()
    normalized_short = str(source_short_commit).strip()
    return (
        normalized_commit == normalized_source
        or normalized_commit == normalized_short
        or normalized_source.startswith(normalized_commit)
    )


def _resolve_release_gate_status(
    gate: Mapping[str, Any], release_policy: Mapping[str, Any]
) -> str:
    if gate.get("passed_required_evidence") != gate.get("required_evidence"):
        return "blocked"
    if (
        release_policy.get("requires_release_approval", False)
        and gate.get("release_approval_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_git_source_binding", False)
        and gate.get("release_source_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_clean_worktree", False)
        and gate.get("release_worktree_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_version_tag_match", False)
        and gate.get("release_version_tag_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_customer_delivery_surface", False)
        and gate.get("customer_delivery_ready", 0) == 0
    ):
        return "blocked"
    if (
        release_policy.get("requires_industrial_delivery_gate", False)
        and gate.get("industrial_delivery_ready", 0) == 0
    ):
        return "blocked"
    if (
        gate.get("blocked_optional_evidence", 0) > 0
        or (
            not release_policy.get("allows_opt_in_evidence", False)
            and gate.get("opt_in_evidence", 0) > 0
        )
        or (
            not release_policy.get("allows_diagnostic_ready_domains", False)
            and gate.get("diagnostic_ready_domains", 0) > 0
        )
    ):
        return "ready_with_limitations"
    return "ready"


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _is_positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _coerce_non_negative_int(value: Any) -> int | None:
    return value if _is_non_negative_int(value) else None
