"""Stable contracts for phase-one workflow artifacts and core payloads."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import asdict, is_dataclass
import importlib.util
from pathlib import Path
from typing import Any

try:
    from .robot_schema import (
        ROBOT_MECHANICAL_SCHEMA_VERSION,
        normalize_robot_config_for_godot,
        validate_godot_robot_config,
    )
except ImportError:
    _robot_schema_path = Path(__file__).with_name("robot_schema.py")
    _robot_schema_spec = importlib.util.spec_from_file_location(
        "robot_schema", _robot_schema_path
    )
    if _robot_schema_spec is None or _robot_schema_spec.loader is None:
        raise
    _robot_schema = importlib.util.module_from_spec(_robot_schema_spec)
    _robot_schema_spec.loader.exec_module(_robot_schema)
    ROBOT_MECHANICAL_SCHEMA_VERSION = _robot_schema.ROBOT_MECHANICAL_SCHEMA_VERSION
    normalize_robot_config_for_godot = _robot_schema.normalize_robot_config_for_godot
    validate_godot_robot_config = _robot_schema.validate_godot_robot_config

WORKFLOW_CONTRACT_VERSION = "1.0"
DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION = "delivery_acceptance_gate.v1"
DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION = (
    "delivery_acceptance_gate_validation_summary.v1"
)

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_STATUSES = {
    "error",
    "success",
}

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_REQUIRED_FIELDS = {
    "summary_version",
    "status",
    "expanded_inputs_count",
    "inputs_count",
    "success_count",
    "skipped_count",
    "error_count",
    "errors",
}

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELDS = {
    "node_tree_manifest_sidecar_complete_count",
    "node_tree_manifest_sidecar_count",
    "node_tree_manifest_sidecar_incomplete_count",
    "node_tree_manifest_sidecar_invalid_count",
    "node_tree_manifest_sidecar_joint_path_count",
    "node_tree_manifest_sidecar_joints_planned_count",
    "node_tree_manifest_sidecar_path_incomplete_count",
    "node_tree_manifest_sidecar_path_map_mismatch_count",
    "node_tree_manifest_sidecar_path_map_mismatch_kind_counts",
    "node_tree_manifest_sidecar_part_path_count",
    "node_tree_manifest_sidecar_parts_planned_count",
    "node_tree_manifest_sidecar_valid_count",
    "node_tree_manifest_sidecar_validation_error_count",
    "node_tree_manifest_sidecars",
    "node_tree_manifest_sidecars_truncated",
    "summary_versions",
    "summary_versions_count",
    "summary_versions_truncated",
    "validation_summary_statuses",
    "validation_summary_statuses_count",
    "validation_summary_statuses_truncated",
}

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELD_TYPES = {
    "node_tree_manifest_sidecar_complete_count": "integer",
    "node_tree_manifest_sidecar_count": "integer",
    "node_tree_manifest_sidecar_incomplete_count": "integer",
    "node_tree_manifest_sidecar_invalid_count": "integer",
    "node_tree_manifest_sidecar_joint_path_count": "integer",
    "node_tree_manifest_sidecar_joints_planned_count": "integer",
    "node_tree_manifest_sidecar_path_incomplete_count": "integer",
    "node_tree_manifest_sidecar_path_map_mismatch_count": "integer",
    "node_tree_manifest_sidecar_path_map_mismatch_kind_counts": "object",
    "node_tree_manifest_sidecar_part_path_count": "integer",
    "node_tree_manifest_sidecar_parts_planned_count": "integer",
    "node_tree_manifest_sidecar_valid_count": "integer",
    "node_tree_manifest_sidecar_validation_error_count": "integer",
    "node_tree_manifest_sidecars": "list[object]",
    "node_tree_manifest_sidecars_truncated": "boolean",
    "summary_versions": "list[str]",
    "summary_versions_count": "integer",
    "summary_versions_truncated": "boolean",
    "validation_summary_statuses": "list[str]",
    "validation_summary_statuses_count": "integer",
    "validation_summary_statuses_truncated": "boolean",
}

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELDS = {
    "expected_node_tree_manifest_sidecar_complete_count",
    "expected_node_tree_manifest_sidecar_count",
    "expected_node_tree_manifest_sidecar_incomplete_count",
    "expected_node_tree_manifest_sidecar_invalid_count",
    "expected_node_tree_manifest_sidecar_joint_path_count",
    "expected_node_tree_manifest_sidecar_joints_planned_count",
    "expected_node_tree_manifest_sidecar_part_path_count",
    "expected_node_tree_manifest_sidecar_parts_planned_count",
    "expected_node_tree_manifest_sidecar_path_incomplete_count",
    "expected_node_tree_manifest_sidecar_path_map_mismatch_count",
    "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts",
    "expected_node_tree_manifest_sidecar_valid_count",
    "expected_node_tree_manifest_sidecar_validation_error_count",
    "expected_summary_versions_count",
    "expected_validation_summary_statuses_count",
    "expected_summary_versions",
    "missing_expected_summary_versions",
    "expected_validation_summary_statuses",
    "missing_expected_validation_summary_statuses",
    "allowed_summary_versions",
    "unexpected_summary_versions",
    "allowed_validation_summary_statuses",
    "unexpected_validation_summary_statuses",
    "forbidden_summary_versions",
    "present_forbidden_summary_versions",
    "forbidden_validation_summary_statuses",
    "present_forbidden_validation_summary_statuses",
}

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELD_TYPES = {
    "expected_node_tree_manifest_sidecar_complete_count": "integer|null",
    "expected_node_tree_manifest_sidecar_count": "integer|null",
    "expected_node_tree_manifest_sidecar_incomplete_count": "integer|null",
    "expected_node_tree_manifest_sidecar_invalid_count": "integer|null",
    "expected_node_tree_manifest_sidecar_joint_path_count": "integer|null",
    "expected_node_tree_manifest_sidecar_joints_planned_count": "integer|null",
    "expected_node_tree_manifest_sidecar_part_path_count": "integer|null",
    "expected_node_tree_manifest_sidecar_parts_planned_count": "integer|null",
    "expected_node_tree_manifest_sidecar_path_incomplete_count": "integer|null",
    "expected_node_tree_manifest_sidecar_path_map_mismatch_count": "integer|null",
    "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts": "object",
    "expected_node_tree_manifest_sidecar_valid_count": "integer|null",
    "expected_node_tree_manifest_sidecar_validation_error_count": "integer|null",
    "expected_summary_versions_count": "integer|null",
    "expected_validation_summary_statuses_count": "integer|null",
    "expected_summary_versions": "list[str]",
    "missing_expected_summary_versions": "list[str]",
    "expected_validation_summary_statuses": "list[str]",
    "missing_expected_validation_summary_statuses": "list[str]",
    "allowed_summary_versions": "list[str]",
    "unexpected_summary_versions": "list[str]",
    "allowed_validation_summary_statuses": "list[str]",
    "unexpected_validation_summary_statuses": "list[str]",
    "forbidden_summary_versions": "list[str]",
    "present_forbidden_summary_versions": "list[str]",
    "forbidden_validation_summary_statuses": "list[str]",
    "present_forbidden_validation_summary_statuses": "list[str]",
}

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELDS = {
    "fail_on_control_readback_missing",
    "fail_on_full_mechanical_gate_false",
    "fail_on_full_mechanical_gate_unknown",
    "fail_on_invalid_node_tree_manifest_sidecar",
    "fail_on_node_tree_manifest_sidecar_incomplete",
    "fail_on_node_tree_manifest_sidecar_path_incomplete",
    "fail_on_node_tree_manifest_sidecar_path_map_mismatch",
    "fail_on_node_tree_manifest_sidecar_validation_error",
    "fail_on_smoke_report_missing",
    "fail_on_smoke_report_read_error",
}

DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELD_TYPES = {
    field: "boolean"
    for field in DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELDS
}

DELIVERY_ACCEPTANCE_GATE_REQUIRED_FIELDS = {
    "contract_version",
    "source",
    "verification_scope",
    "required",
    "requires_full_mechanical_restoration_gate",
    "acceptance_profile",
    "acceptance_requirements",
    "passed",
    "exit_code",
    "level",
    "complete",
    "reasons",
    "reason_codes",
    "reason_details",
    "summary_counts",
}

DELIVERY_ACCEPTANCE_GATE_SOURCES = {
    "dynamic_godot_report_cli",
    "web_godot_delivery",
}

DELIVERY_ACCEPTANCE_GATE_VERIFICATION_SCOPES = {
    "godot_smoke_motion",
    "godot_load",
}

DELIVERY_ACCEPTANCE_GATE_ACCEPTANCE_PROFILES = {
    "custom",
    "full_mechanical_restoration",
    "web_godot_load",
}

DELIVERY_ACCEPTANCE_GATE_LEVELS = {
    "godot_verified",
    "static_only",
    "incomplete",
    "godot_load_verified",
}

DELIVERY_ACCEPTANCE_GATE_REASON_CODES = {
    "control_readback_missing",
    "fixed_lock_mismatch",
    "godot_delivery_failed",
    "incomplete_delivery",
    "incomplete_joints",
    "incomplete_joint_parameters",
    "incomplete_parts",
    "incomplete_restoration",
    "invalid_godot_smoke_report",
    "missing_dynamic_generation",
    "missing_full_mechanical_restoration_gate",
    "missing_godot_assembly_summary",
    "missing_godot_smoke",
    "missing_godot_smoke_report",
    "missing_static_node_tree_manifest_output",
    "no_inputs",
    "node_tree_fixed_lock_mismatch",
    "robot_errors",
    "smoke_failure_reasons",
    "static_only",
    "static_node_tree_incomplete",
    "unknown_delivery_provenance",
}

DELIVERY_ACCEPTANCE_GATE_REASON_DETAIL_FIELDS = {
    "code",
    "count",
    "inputs",
    "inputs_count",
    "inputs_truncated",
    "message",
}

DELIVERY_ACCEPTANCE_GATE_SOURCE_REASON_CODES = {
    "dynamic_godot_report_cli": {
        "control_readback_missing",
        "incomplete_delivery",
        "incomplete_joint_parameters",
        "invalid_godot_smoke_report",
        "missing_dynamic_generation",
        "missing_full_mechanical_restoration_gate",
        "missing_static_node_tree_manifest_output",
        "missing_godot_smoke",
        "missing_godot_smoke_report",
        "no_inputs",
        "node_tree_fixed_lock_mismatch",
        "robot_errors",
        "smoke_failure_reasons",
        "static_only",
        "static_node_tree_incomplete",
        "unknown_delivery_provenance",
    },
    "web_godot_delivery": {
        "fixed_lock_mismatch",
        "godot_delivery_failed",
        "incomplete_delivery",
        "incomplete_joints",
        "incomplete_joint_parameters",
        "incomplete_parts",
        "incomplete_restoration",
        "missing_godot_assembly_summary",
    },
}

DELIVERY_ACCEPTANCE_GATE_SOURCE_SCOPES = {
    "dynamic_godot_report_cli": "godot_smoke_motion",
    "web_godot_delivery": "godot_load",
}

DELIVERY_ACCEPTANCE_GATE_SOURCE_PROFILES = {
    "dynamic_godot_report_cli": {"custom", "full_mechanical_restoration"},
    "web_godot_delivery": {"web_godot_load"},
}

DELIVERY_ACCEPTANCE_GATE_SCOPE_LEVELS = {
    "godot_smoke_motion": {"godot_verified", "static_only", "incomplete"},
    "godot_load": {"godot_load_verified", "incomplete"},
}

DELIVERY_ACCEPTANCE_GATE_SCOPE_COMPLETE_LEVELS = {
    "godot_smoke_motion": "godot_verified",
    "godot_load": "godot_load_verified",
}

DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS = {
    "inputs_count",
    "success_count",
    "error_count",
    "live_smoke_count",
    "smoke_report_written_count",
    "smoke_report_missing_count",
    "smoke_report_read_error_count",
    "delivery_godot_verified_count",
    "delivery_static_only_count",
    "delivery_unverified_count",
    "delivery_dynamic_generation_count",
    "delivery_complete_count",
    "delivery_incomplete_count",
    "delivery_parameters_incomplete_count",
    "fixed_lock_checked_count",
    "fixed_lock_mismatch_count",
    "control_configured_count",
    "control_readback_checked_count",
    "control_readback_missing_count",
    "node_tree_fixed_lock_checked_count",
    "node_tree_fixed_lock_mismatch_count",
    "node_tree_fixed_locks_complete_count",
    "node_tree_fixed_locks_incomplete_count",
    "node_tree_gate_enabled_count",
    "node_tree_full_restoration_required_count",
    "node_tree_full_restoration_not_required_count",
    "static_topology_complete_count",
    "static_topology_incomplete_count",
    "static_topology_disconnected_parts_count",
    "static_topology_unreachable_parts_count",
    "static_topology_duplicate_child_endpoint_count",
    "static_topology_cycle_count",
    "static_node_tree_manifest_count",
    "static_node_tree_manifest_error_count",
    "static_node_tree_manifest_invalid_count",
    "static_node_tree_manifest_output_count",
    "static_node_tree_manifest_path_map_mismatch_count",
    "static_node_tree_manifest_valid_count",
    "static_node_tree_parts_planned_count",
    "static_node_tree_joints_planned_count",
    "static_node_tree_parameterized_joints_count",
    "static_node_tree_complete_count",
    "static_node_tree_incomplete_count",
    "static_node_tree_endpoint_paths_complete_count",
    "static_node_tree_endpoint_paths_incomplete_count",
    "static_node_tree_missing_endpoint_connections_count",
    "static_node_tree_missing_endpoint_parts_count",
    "static_node_tree_parameters_complete_count",
    "static_node_tree_parameters_incomplete_count",
    "mechanical_gate_enabled_count",
    "full_mechanical_restoration_required_count",
    "full_mechanical_restoration_not_required_count",
    "mechanical_behavior_evidence_count",
    "mechanical_behavior_complete_count",
    "mechanical_behavior_incomplete_count",
    "mechanical_behavior_residual_risk_count",
    "mechanical_behavior_threshold_failure_count",
    "mechanical_behavior_center_of_mass_available_count",
    "mechanical_behavior_contact_state_available_count",
    "mechanical_behavior_step_trace_artifact_count",
    "failure_reasons_count",
}

DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_FIELDS = {
    "node_tree_gate_check_counts",
    "mechanical_gate_check_counts",
    "static_node_tree_manifest_path_map_mismatch_kind_counts",
}

DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_KEY_VALUES = {
    "node_tree_gate_check_counts": {
        "class_mismatch",
        "fixed_lock_mismatch",
        "incomplete_node_tree",
        "missing_parameters",
        "physical_mismatch",
        "transform_mismatch",
    },
    "mechanical_gate_check_counts": {
        "control_parameter_readback",
        "full_node_tree_restoration",
        "joint_parameter_readback",
        "mechanical_restoration",
    },
    "static_node_tree_manifest_path_map_mismatch_kind_counts": {
        "duplicate",
        "missing",
        "root_mismatch",
        "unexpected",
        "value_mismatch",
    },
}

DELIVERY_ACCEPTANCE_GATE_SUMMARY_VALUE_SOURCE_FILTER_FIELDS = {
    "actual_expected_summary_values",
    "allowed_summary_value_source_excluded_sources",
    "allowed_summary_value_source_matched_sources",
    "expected_summary_value_source_excluded_count",
    "expected_summary_value_source_excluded_sources",
    "expected_summary_value_source_matched_count",
    "expected_summary_value_sources",
    "forbidden_summary_value_source_excluded_sources",
    "forbidden_summary_value_source_matched_sources",
    "missing_expected_summary_value_source_excluded_sources",
    "missing_expected_summary_value_sources",
    "mismatched_expected_summary_values",
    "present_forbidden_summary_value_source_excluded_sources",
    "present_forbidden_summary_value_source_matched_sources",
    "summary_value_source_excluded_count",
    "summary_value_source_excluded_sources",
    "summary_value_source_matched_count",
    "summary_value_source_matched_sources",
    "unexpected_summary_value_source_excluded_sources",
    "unexpected_summary_value_source_matched_sources",
}

DELIVERY_ACCEPTANCE_GATE_SUMMARY_VALUE_SOURCE_FILTER_FIELD_TYPES = {
    "actual_expected_summary_values": "object",
    "allowed_summary_value_source_excluded_sources": "list[str]",
    "allowed_summary_value_source_matched_sources": "list[str]",
    "expected_summary_value_source_excluded_count": "integer|null",
    "expected_summary_value_source_excluded_sources": "list[str]",
    "expected_summary_value_source_matched_count": "integer|null",
    "expected_summary_value_sources": "list[str]",
    "forbidden_summary_value_source_excluded_sources": "list[str]",
    "forbidden_summary_value_source_matched_sources": "list[str]",
    "missing_expected_summary_value_source_excluded_sources": "list[str]",
    "missing_expected_summary_value_sources": "list[str]",
    "mismatched_expected_summary_values": "object",
    "present_forbidden_summary_value_source_excluded_sources": "list[str]",
    "present_forbidden_summary_value_source_matched_sources": "list[str]",
    "summary_value_source_excluded_count": "integer|null",
    "summary_value_source_excluded_sources": "list[str]",
    "summary_value_source_matched_count": "integer|null",
    "summary_value_source_matched_sources": "list[str]",
    "unexpected_summary_value_source_excluded_sources": "list[str]",
    "unexpected_summary_value_source_matched_sources": "list[str]",
}

DELIVERY_ACCEPTANCE_GATE_SOURCE_SUMMARY_FIELDS = {
    "dynamic_godot_report_cli": (
        DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS
        | DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_FIELDS
    ),
    "web_godot_delivery": {
        "delivery_complete_count",
        "delivery_dynamic_generation_count",
        "delivery_godot_verified_count",
        "delivery_incomplete_count",
        "delivery_parameters_incomplete_count",
        "delivery_static_only_count",
        "delivery_unverified_count",
        "error_count",
        "failure_reasons_count",
        "fixed_lock_checked_count",
        "fixed_lock_mismatch_count",
        "full_mechanical_restoration_not_required_count",
        "full_mechanical_restoration_required_count",
        "inputs_count",
        "live_smoke_count",
        "mechanical_gate_check_counts",
        "mechanical_gate_enabled_count",
        "node_tree_fixed_lock_checked_count",
        "node_tree_fixed_lock_mismatch_count",
        "node_tree_fixed_locks_complete_count",
        "node_tree_fixed_locks_incomplete_count",
        "node_tree_full_restoration_not_required_count",
        "node_tree_full_restoration_required_count",
        "node_tree_gate_check_counts",
        "node_tree_gate_enabled_count",
        "static_node_tree_complete_count",
        "static_node_tree_endpoint_paths_complete_count",
        "static_node_tree_endpoint_paths_incomplete_count",
        "static_node_tree_incomplete_count",
        "static_node_tree_manifest_count",
        "static_node_tree_manifest_error_count",
        "static_node_tree_manifest_invalid_count",
        "static_node_tree_manifest_output_count",
        "static_node_tree_manifest_path_map_mismatch_count",
        "static_node_tree_manifest_path_map_mismatch_kind_counts",
        "static_node_tree_manifest_valid_count",
        "static_node_tree_parameters_complete_count",
        "static_node_tree_parameters_incomplete_count",
        "success_count",
    },
}

DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS = {
    "run_godot_smoke": False,
    "godot_verified_acceptance": False,
    "full_mechanical_restoration_gate": False,
    "full_mechanical_restoration_smoke_gate": False,
    "mechanical_restoration_complete": False,
    "joint_parameter_readback": False,
    "control_parameter_readback": False,
    "full_node_tree_restoration": False,
    "node_tree_complete": False,
    "node_tree_class_match": False,
    "node_tree_parameters_applied": False,
    "node_tree_transform_match": False,
    "node_tree_physical_match": False,
    "node_tree_fixed_lock_match": False,
    "joint_limit_violation_gate": False,
    "body_motion_gate": False,
    "joint_motion_delta_gate": False,
    "joint_motion_range_gate": False,
    "moving_joint_coverage_gate": False,
    "commanded_joint_response_gate": False,
    "action_target_consistency": False,
    "action_sequence_target_consistency": False,
    "unknown_action_target_gate": False,
    "invalid_action_target_gate": False,
    "action_target_coverage_gate": False,
    "control_action_coverage_gate": False,
    "nonzero_action_targets_gate": False,
    "action_transition_count_gate": False,
    "action_transition_delta_gate": False,
    "restoration_score_gate": False,
    "static_node_tree_complete": False,
    "static_node_tree_manifest_output": False,
}

DELIVERY_ACCEPTANCE_GATE_SOURCE_ENABLED_REQUIREMENTS = {
    "dynamic_godot_report_cli": set(DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS),
    "web_godot_delivery": {
        "godot_load",
        "joint_parameter_readback",
        "mechanical_restoration_complete",
        "node_tree_fixed_lock_match",
    },
}

DELIVERY_ACCEPTANCE_GATE_REQUIREMENT_SUMMARY_MAP_COUNTS = {
    "node_tree_gate_check_counts": {
        "node_tree_complete": "incomplete_node_tree",
        "node_tree_class_match": "class_mismatch",
        "node_tree_parameters_applied": "missing_parameters",
        "node_tree_transform_match": "transform_mismatch",
        "node_tree_physical_match": "physical_mismatch",
        "node_tree_fixed_lock_match": "fixed_lock_mismatch",
    },
    "mechanical_gate_check_counts": {
        "mechanical_restoration_complete": "mechanical_restoration",
        "joint_parameter_readback": "joint_parameter_readback",
        "control_parameter_readback": "control_parameter_readback",
        "full_node_tree_restoration": "full_node_tree_restoration",
    },
}

DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_EQUAL_INPUTS = {
    "delivery_complete_count",
    "delivery_dynamic_generation_count",
    "delivery_godot_verified_count",
    "static_topology_complete_count",
    "static_node_tree_manifest_count",
    "static_node_tree_manifest_valid_count",
    "static_node_tree_complete_count",
    "static_node_tree_endpoint_paths_complete_count",
    "static_node_tree_parameters_complete_count",
    "success_count",
}

DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_ZERO = {
    "control_readback_missing_count",
    "delivery_incomplete_count",
    "delivery_parameters_incomplete_count",
    "delivery_static_only_count",
    "delivery_unverified_count",
    "error_count",
    "failure_reasons_count",
    "fixed_lock_mismatch_count",
    "node_tree_fixed_lock_mismatch_count",
    "node_tree_fixed_locks_incomplete_count",
    "static_node_tree_manifest_invalid_count",
    "static_node_tree_manifest_error_count",
    "static_node_tree_manifest_path_map_mismatch_count",
    "static_topology_incomplete_count",
    "static_topology_disconnected_parts_count",
    "static_topology_unreachable_parts_count",
    "static_topology_duplicate_child_endpoint_count",
    "static_topology_cycle_count",
    "static_node_tree_incomplete_count",
    "static_node_tree_endpoint_paths_incomplete_count",
    "static_node_tree_missing_endpoint_connections_count",
    "static_node_tree_missing_endpoint_parts_count",
    "static_node_tree_parameters_incomplete_count",
}

DELIVERY_ACCEPTANCE_GATE_SCOPE_COMPLETE_SUMMARY_COUNTS = {
    "godot_smoke_motion": {
        "equal_inputs": {
            "live_smoke_count",
            "smoke_report_written_count",
        },
        "zero": {
            "smoke_report_missing_count",
            "smoke_report_read_error_count",
        },
    },
}

DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_SUM_RULES = (
    {
        "name": "result_counts",
        "counts": ("success_count", "error_count"),
        "total": "inputs_count",
        "message": "summary_counts.success_count + error_count must equal inputs_count",
    },
    {
        "name": "delivery_provenance_counts",
        "counts": (
            "delivery_godot_verified_count",
            "delivery_static_only_count",
            "delivery_unverified_count",
        ),
        "total": "inputs_count",
        "message": "summary_counts delivery provenance counts must equal inputs_count",
    },
    {
        "name": "delivery_completion_counts",
        "counts": ("delivery_complete_count", "delivery_incomplete_count"),
        "total": "inputs_count",
        "message": (
            "summary_counts.delivery_complete_count + delivery_incomplete_count "
            "must equal inputs_count"
        ),
    },
    {
        "name": "static_node_tree_manifest_validity_counts",
        "counts": (
            "static_node_tree_manifest_valid_count",
            "static_node_tree_manifest_invalid_count",
        ),
        "total": "static_node_tree_manifest_count",
        "message": (
            "summary_counts.static_node_tree_manifest_valid_count + "
            "static_node_tree_manifest_invalid_count must equal "
            "static_node_tree_manifest_count"
        ),
    },
)

DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_LTE_RULES = (
    {
        "count": "live_smoke_count",
        "max": "inputs_count",
    },
)

DELIVERY_ACCEPTANCE_GATE_SUMMARY_MAP_SUM_RULES = (
    {
        "map": "node_tree_gate_check_counts",
        "total": "node_tree_gate_enabled_count",
    },
    {
        "map": "mechanical_gate_check_counts",
        "total": "mechanical_gate_enabled_count",
    },
    {
        "map": "static_node_tree_manifest_path_map_mismatch_kind_counts",
        "total": "static_node_tree_manifest_path_map_mismatch_count",
    },
)

DELIVERY_ACCEPTANCE_GATE_COMPLETE_RESULT_REQUIREMENTS = {
    "passed": True,
    "exit_code": 0,
    "empty_lists": ("reasons", "reason_codes", "reason_details"),
}


def _complete_summary_counts_for_source_scope(
    source: str, scope: str
) -> dict[str, list[str]]:
    scope_counts = DELIVERY_ACCEPTANCE_GATE_SCOPE_COMPLETE_SUMMARY_COUNTS.get(
        scope,
        {"equal_inputs": set(), "zero": set()},
    )
    source_fields = DELIVERY_ACCEPTANCE_GATE_SOURCE_SUMMARY_FIELDS[source]
    return {
        "equal_inputs": sorted(
            (
                DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_EQUAL_INPUTS
                | scope_counts["equal_inputs"]
            )
            & source_fields
        ),
        "zero": sorted(
            (DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_ZERO | scope_counts["zero"])
            & source_fields
        ),
    }


def _summary_value_paths_for_fields(fields: set[str]) -> list[str]:
    paths = set(fields & DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS)
    for (
        map_field,
        key_values,
    ) in DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_KEY_VALUES.items():
        if map_field not in fields:
            continue
        paths.update(f"{map_field}.{key_value}" for key_value in key_values)
    return sorted(paths)


DELIVERY_ACCEPTANCE_GATE_SCHEMA = {
    "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
    "validation_summary_version": (DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION),
    "validation_summary_required_fields": sorted(
        DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_REQUIRED_FIELDS
    ),
    "validation_summary_status_values": sorted(
        DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_STATUSES
    ),
    "validation_summary_metadata_fields": sorted(
        DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELDS
    ),
    "validation_summary_metadata_field_types": dict(
        sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELD_TYPES.items())
    ),
    "validation_summary_constraint_fields": sorted(
        DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELDS
    ),
    "validation_summary_constraint_field_types": dict(
        sorted(
            DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELD_TYPES.items()
        )
    ),
    "validation_summary_policy_fields": sorted(
        DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELDS
    ),
    "validation_summary_policy_field_types": dict(
        sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELD_TYPES.items())
    ),
    "required": sorted(DELIVERY_ACCEPTANCE_GATE_REQUIRED_FIELDS),
    "source_values": sorted(DELIVERY_ACCEPTANCE_GATE_SOURCES),
    "verification_scope_values": sorted(DELIVERY_ACCEPTANCE_GATE_VERIFICATION_SCOPES),
    "acceptance_profile_values": sorted(DELIVERY_ACCEPTANCE_GATE_ACCEPTANCE_PROFILES),
    "level_values": sorted(DELIVERY_ACCEPTANCE_GATE_LEVELS),
    "source_scope_pairs": dict(sorted(DELIVERY_ACCEPTANCE_GATE_SOURCE_SCOPES.items())),
    "source_profile_values": {
        source: sorted(profiles)
        for source, profiles in sorted(DELIVERY_ACCEPTANCE_GATE_SOURCE_PROFILES.items())
    },
    "scope_level_values": {
        scope: sorted(levels)
        for scope, levels in sorted(DELIVERY_ACCEPTANCE_GATE_SCOPE_LEVELS.items())
    },
    "complete_level_by_scope": dict(
        sorted(DELIVERY_ACCEPTANCE_GATE_SCOPE_COMPLETE_LEVELS.items())
    ),
    "reason_code_values": sorted(DELIVERY_ACCEPTANCE_GATE_REASON_CODES),
    "reason_detail_fields": sorted(DELIVERY_ACCEPTANCE_GATE_REASON_DETAIL_FIELDS),
    "reason_code_values_by_source": {
        source: sorted(reason_codes)
        for source, reason_codes in sorted(
            DELIVERY_ACCEPTANCE_GATE_SOURCE_REASON_CODES.items()
        )
    },
    "requirement_fields": sorted(
        set(DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS) | {"godot_load"}
    ),
    "enabled_requirement_values_by_source": {
        source: sorted(requirements)
        for source, requirements in sorted(
            DELIVERY_ACCEPTANCE_GATE_SOURCE_ENABLED_REQUIREMENTS.items()
        )
    },
    "summary_count_fields": sorted(DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS),
    "summary_count_map_fields": sorted(
        DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_FIELDS
    ),
    "summary_count_map_key_values": {
        map_field: sorted(key_values)
        for map_field, key_values in sorted(
            DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_KEY_VALUES.items()
        )
    },
    "summary_value_paths": sorted(
        set(DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS)
        | {
            f"{map_field}.{key_value}"
            for map_field, key_values in DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_KEY_VALUES.items()
            for key_value in key_values
        }
    ),
    "summary_value_paths_by_source": {
        source: _summary_value_paths_for_fields(fields)
        for source, fields in sorted(
            DELIVERY_ACCEPTANCE_GATE_SOURCE_SUMMARY_FIELDS.items()
        )
    },
    "summary_value_source_filter_fields": sorted(
        DELIVERY_ACCEPTANCE_GATE_SUMMARY_VALUE_SOURCE_FILTER_FIELDS
    ),
    "summary_value_source_filter_field_types": dict(
        sorted(DELIVERY_ACCEPTANCE_GATE_SUMMARY_VALUE_SOURCE_FILTER_FIELD_TYPES.items())
    ),
    "requirement_summary_map_counts": {
        map_field: dict(sorted(requirement_map.items()))
        for map_field, requirement_map in sorted(
            DELIVERY_ACCEPTANCE_GATE_REQUIREMENT_SUMMARY_MAP_COUNTS.items()
        )
    },
    "requirement_summary_map_counts_by_source_scope": {
        source: {
            scope: (
                {
                    map_field: dict(sorted(requirement_map.items()))
                    for map_field, requirement_map in sorted(
                        DELIVERY_ACCEPTANCE_GATE_REQUIREMENT_SUMMARY_MAP_COUNTS.items()
                    )
                }
                if scope == "godot_smoke_motion"
                else {}
            )
        }
        for source, scope in sorted(DELIVERY_ACCEPTANCE_GATE_SOURCE_SCOPES.items())
    },
    "complete_summary_counts": {
        "equal_inputs": sorted(DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_EQUAL_INPUTS),
        "zero": sorted(DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_ZERO),
    },
    "complete_summary_counts_by_source": {
        source: {
            "equal_inputs": sorted(
                DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_EQUAL_INPUTS & fields
            ),
            "zero": sorted(DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_ZERO & fields),
        }
        for source, fields in sorted(
            DELIVERY_ACCEPTANCE_GATE_SOURCE_SUMMARY_FIELDS.items()
        )
    },
    "complete_summary_counts_by_source_scope": {
        source: {scope: _complete_summary_counts_for_source_scope(source, scope)}
        for source, scope in sorted(DELIVERY_ACCEPTANCE_GATE_SOURCE_SCOPES.items())
    },
    "complete_required_summary_fields_by_source_scope": {
        source: {
            scope: sorted(
                set(
                    _complete_summary_counts_for_source_scope(source, scope)[
                        "equal_inputs"
                    ]
                )
                | set(_complete_summary_counts_for_source_scope(source, scope)["zero"])
            )
        }
        for source, scope in sorted(DELIVERY_ACCEPTANCE_GATE_SOURCE_SCOPES.items())
    },
    "complete_required_summary_fields_source_scopes": [
        f"{source}/{scope}"
        for source, scope in sorted(DELIVERY_ACCEPTANCE_GATE_SOURCE_SCOPES.items())
    ],
    "scope_complete_summary_counts": {
        scope: {
            "equal_inputs": sorted(counts["equal_inputs"]),
            "zero": sorted(counts["zero"]),
        }
        for scope, counts in sorted(
            DELIVERY_ACCEPTANCE_GATE_SCOPE_COMPLETE_SUMMARY_COUNTS.items()
        )
    },
    "summary_count_sum_rules": [
        {
            "name": str(rule["name"]),
            "counts": list(rule["counts"]),
            "total": str(rule["total"]),
            "message": str(rule["message"]),
        }
        for rule in DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_SUM_RULES
    ],
    "summary_count_lte_rules": [
        {
            "count": str(rule["count"]),
            "max": str(rule["max"]),
        }
        for rule in DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_LTE_RULES
    ],
    "summary_map_sum_rules": [
        {
            "map": str(rule["map"]),
            "total": str(rule["total"]),
        }
        for rule in DELIVERY_ACCEPTANCE_GATE_SUMMARY_MAP_SUM_RULES
    ],
    "complete_result_requirements": {
        "passed": DELIVERY_ACCEPTANCE_GATE_COMPLETE_RESULT_REQUIREMENTS["passed"],
        "exit_code": DELIVERY_ACCEPTANCE_GATE_COMPLETE_RESULT_REQUIREMENTS["exit_code"],
        "empty_lists": list(
            DELIVERY_ACCEPTANCE_GATE_COMPLETE_RESULT_REQUIREMENTS["empty_lists"]
        ),
    },
    "summary_fields_by_source": {
        source: sorted(fields)
        for source, fields in sorted(
            DELIVERY_ACCEPTANCE_GATE_SOURCE_SUMMARY_FIELDS.items()
        )
    },
}


def build_delivery_acceptance_requirements(**overrides: bool) -> dict[str, bool]:
    """Return the stable delivery gate requirements map with explicit defaults."""
    requirements = dict(DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS)
    for key, value in overrides.items():
        if key not in requirements and key != "godot_load":
            raise ValueError(f"unknown delivery acceptance requirement: {key}")
        requirements[key] = bool(value)
    return requirements


def validate_delivery_acceptance_gate(payload: Any) -> list[str]:
    """Return validation errors for a delivery_acceptance_gate payload."""
    if not isinstance(payload, Mapping):
        return ["delivery_acceptance_gate must be an object"]

    errors: list[str] = []
    missing = sorted(DELIVERY_ACCEPTANCE_GATE_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("contract_version") != DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION:
        errors.append(
            "contract_version must be "
            f"{DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION!r}, "
            f"got {payload.get('contract_version')!r}"
        )
    for key in ["source", "verification_scope", "acceptance_profile", "level"]:
        if key in payload and not _is_non_empty_string(payload.get(key)):
            errors.append(f"{key} must be a non-empty string")
    _validate_allowed_text(
        errors,
        payload,
        "source",
        DELIVERY_ACCEPTANCE_GATE_SOURCES,
    )
    _validate_allowed_text(
        errors,
        payload,
        "verification_scope",
        DELIVERY_ACCEPTANCE_GATE_VERIFICATION_SCOPES,
    )
    _validate_allowed_text(
        errors,
        payload,
        "acceptance_profile",
        DELIVERY_ACCEPTANCE_GATE_ACCEPTANCE_PROFILES,
    )
    _validate_allowed_text(errors, payload, "level", DELIVERY_ACCEPTANCE_GATE_LEVELS)
    source = payload.get("source")
    verification_scope = payload.get("verification_scope")
    acceptance_profile = payload.get("acceptance_profile")
    level = payload.get("level")
    if _is_non_empty_string(source) and _is_non_empty_string(verification_scope):
        expected_scope = DELIVERY_ACCEPTANCE_GATE_SOURCE_SCOPES.get(str(source))
        if expected_scope is not None and verification_scope != expected_scope:
            errors.append(
                f"verification_scope must be {expected_scope!r} when source is "
                f"{source!r}"
            )
    if _is_non_empty_string(source) and _is_non_empty_string(acceptance_profile):
        allowed_profiles = DELIVERY_ACCEPTANCE_GATE_SOURCE_PROFILES.get(str(source))
        if allowed_profiles is not None and acceptance_profile not in allowed_profiles:
            errors.append(
                f"acceptance_profile must be one of "
                f"{', '.join(sorted(allowed_profiles))} when source is {source!r}"
            )
    if _is_non_empty_string(verification_scope) and _is_non_empty_string(level):
        allowed_levels = DELIVERY_ACCEPTANCE_GATE_SCOPE_LEVELS.get(
            str(verification_scope)
        )
        if allowed_levels is not None and level not in allowed_levels:
            errors.append(
                f"level must be one of {', '.join(sorted(allowed_levels))} "
                f"when verification_scope is {verification_scope!r}"
            )
        complete_level = DELIVERY_ACCEPTANCE_GATE_SCOPE_COMPLETE_LEVELS.get(
            str(verification_scope)
        )
        if complete_level is not None:
            if payload.get("complete") is True and level != complete_level:
                errors.append(
                    f"level must be {complete_level!r} when complete is true "
                    f"for verification_scope {verification_scope!r}"
                )
            if payload.get("complete") is False and level == complete_level:
                errors.append(f"complete must be true when level is {complete_level!r}")
    for key in [
        "required",
        "requires_full_mechanical_restoration_gate",
        "passed",
        "complete",
    ]:
        if key in payload and not isinstance(payload.get(key), bool):
            errors.append(f"{key} must be a boolean")
    if "exit_code" in payload and not _is_non_negative_int(payload.get("exit_code")):
        errors.append("exit_code must be a non-negative integer")

    requirements = payload.get("acceptance_requirements")
    if not isinstance(requirements, Mapping):
        errors.append("acceptance_requirements must be an object")
    else:
        allowed = set(DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS) | {"godot_load"}
        unknown = sorted(set(requirements) - allowed)
        if unknown:
            errors.append(
                f"acceptance_requirements contains unknown fields: {', '.join(unknown)}"
            )
        non_bool = sorted(
            str(key)
            for key, value in requirements.items()
            if not isinstance(value, bool)
        )
        if non_bool:
            errors.append(
                f"acceptance_requirements values must be booleans: {', '.join(non_bool)}"
            )
        if _is_non_empty_string(source):
            allowed_enabled_requirements = (
                DELIVERY_ACCEPTANCE_GATE_SOURCE_ENABLED_REQUIREMENTS.get(str(source))
            )
            if allowed_enabled_requirements is not None:
                invalid_enabled_requirements = sorted(
                    str(key)
                    for key, value in requirements.items()
                    if key in allowed
                    and value is True
                    and key not in allowed_enabled_requirements
                )
                if invalid_enabled_requirements:
                    errors.append(
                        "acceptance_requirements contains enabled fields not "
                        f"allowed for source {source!r}: "
                        f"{', '.join(invalid_enabled_requirements)}"
                    )
        if (
            payload.get("requires_full_mechanical_restoration_gate") is True
            and requirements.get("full_mechanical_restoration_gate") is not True
        ):
            errors.append(
                "acceptance_requirements.full_mechanical_restoration_gate must be "
                "true when requires_full_mechanical_restoration_gate is true"
            )
        if (
            payload.get("acceptance_profile") == "full_mechanical_restoration"
            and requirements.get("full_mechanical_restoration_gate") is not True
        ):
            errors.append(
                "acceptance_requirements.full_mechanical_restoration_gate must be "
                "true when acceptance_profile is 'full_mechanical_restoration'"
            )
        if (
            payload.get("acceptance_profile") == "full_mechanical_restoration"
            and payload.get("requires_full_mechanical_restoration_gate") is not True
        ):
            errors.append(
                "requires_full_mechanical_restoration_gate must be true when "
                "acceptance_profile is 'full_mechanical_restoration'"
            )
        if (
            payload.get("verification_scope") == "godot_load"
            and requirements.get("godot_load") is not True
        ):
            errors.append(
                "acceptance_requirements.godot_load must be true when "
                "verification_scope is 'godot_load'"
            )

    for key in ["reasons", "reason_codes", "reason_details"]:
        if key in payload and not isinstance(payload.get(key), list):
            errors.append(f"{key} must be a list")
    if isinstance(payload.get("reasons"), list):
        invalid_reasons = [
            str(index)
            for index, reason in enumerate(payload["reasons"], start=1)
            if not _is_non_empty_string(reason)
        ]
        if invalid_reasons:
            errors.append(
                "reasons entries must be non-empty strings: "
                f"{', '.join(invalid_reasons)}"
            )
        duplicate_reasons = _duplicate_non_empty_strings(payload["reasons"])
        if duplicate_reasons:
            errors.append(
                "reasons entries must be unique: " f"{'; '.join(duplicate_reasons)}"
            )
    if isinstance(payload.get("reason_codes"), list):
        invalid_reason_codes = [
            str(index)
            for index, code in enumerate(payload["reason_codes"], start=1)
            if not _is_non_empty_string(code)
        ]
        if invalid_reason_codes:
            errors.append(
                "reason_codes entries must be non-empty strings: "
                f"{', '.join(invalid_reason_codes)}"
            )
        duplicate_reason_codes = _duplicate_non_empty_strings(payload["reason_codes"])
        if duplicate_reason_codes:
            errors.append(
                "reason_codes entries must be unique: "
                f"{', '.join(duplicate_reason_codes)}"
            )
        unknown_reason_codes = sorted(
            {
                str(code)
                for code in payload["reason_codes"]
                if _is_non_empty_string(code)
                and code not in DELIVERY_ACCEPTANCE_GATE_REASON_CODES
            }
        )
        if unknown_reason_codes:
            errors.append(
                "reason_codes contains unknown values: "
                f"{', '.join(unknown_reason_codes)}"
            )
        if _is_non_empty_string(source):
            allowed_reason_codes = DELIVERY_ACCEPTANCE_GATE_SOURCE_REASON_CODES.get(
                str(source)
            )
            if allowed_reason_codes is not None:
                invalid_for_source = sorted(
                    {
                        str(code)
                        for code in payload["reason_codes"]
                        if _is_non_empty_string(code)
                        and code in DELIVERY_ACCEPTANCE_GATE_REASON_CODES
                        and code not in allowed_reason_codes
                    }
                )
                if invalid_for_source:
                    errors.append(
                        "reason_codes contains values not allowed for source "
                        f"{source!r}: {', '.join(invalid_for_source)}"
                    )
    detail_codes: list[str] = []
    detail_messages: list[str] = []
    if isinstance(payload.get("reason_details"), list):
        for index, detail in enumerate(payload["reason_details"], start=1):
            prefix = f"reason_details[{index}]"
            if not isinstance(detail, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            unknown_detail_fields = sorted(
                str(key)
                for key in detail
                if key not in DELIVERY_ACCEPTANCE_GATE_REASON_DETAIL_FIELDS
            )
            if unknown_detail_fields:
                errors.append(
                    f"{prefix} contains unknown fields: "
                    f"{', '.join(unknown_detail_fields)}"
                )
            if _is_non_empty_string(detail.get("code")):
                detail_code = str(detail["code"])
                detail_codes.append(detail_code)
                if detail_code not in DELIVERY_ACCEPTANCE_GATE_REASON_CODES:
                    errors.append(
                        f"{prefix}.code contains unknown value: {detail_code}"
                    )
                elif _is_non_empty_string(source):
                    allowed_reason_codes = (
                        DELIVERY_ACCEPTANCE_GATE_SOURCE_REASON_CODES.get(str(source))
                    )
                    if (
                        allowed_reason_codes is not None
                        and detail_code not in allowed_reason_codes
                    ):
                        errors.append(
                            f"{prefix}.code is not allowed for source "
                            f"{source!r}: {detail_code}"
                        )
            for key in ["code", "message"]:
                if not _is_non_empty_string(detail.get(key)):
                    errors.append(f"{prefix}.{key} must be a non-empty string")
            if _is_non_empty_string(detail.get("message")):
                detail_messages.append(str(detail["message"]))
            if not _is_non_negative_int(detail.get("count")):
                errors.append(f"{prefix}.count must be a non-negative integer")
            detail_inputs = detail.get("inputs")
            if not isinstance(detail_inputs, list):
                errors.append(f"{prefix}.inputs must be a list")
            else:
                invalid_inputs = [
                    str(input_index)
                    for input_index, input_value in enumerate(detail_inputs, start=1)
                    if not _is_non_empty_string(input_value)
                ]
                if invalid_inputs:
                    errors.append(
                        f"{prefix}.inputs entries must be non-empty strings: "
                        f"{', '.join(invalid_inputs)}"
                    )
                duplicate_inputs = _duplicate_non_empty_strings(detail_inputs)
                if duplicate_inputs:
                    errors.append(
                        f"{prefix}.inputs entries must be unique: "
                        f"{', '.join(duplicate_inputs)}"
                    )
            if not _is_non_negative_int(detail.get("inputs_count")):
                errors.append(f"{prefix}.inputs_count must be a non-negative integer")
            if not isinstance(detail.get("inputs_truncated"), bool):
                errors.append(f"{prefix}.inputs_truncated must be a boolean")
            if isinstance(detail_inputs, list) and _is_non_negative_int(
                detail.get("inputs_count")
            ):
                inputs_count = int(detail["inputs_count"])
                if inputs_count < len(detail_inputs):
                    errors.append(
                        f"{prefix}.inputs_count must be greater than or equal to "
                        "the inputs preview length"
                    )
                if detail.get("inputs_truncated") is False and inputs_count != len(
                    detail_inputs
                ):
                    errors.append(
                        f"{prefix}.inputs_count must equal the inputs preview length "
                        "when inputs_truncated is false"
                    )
            if _is_non_negative_int(detail.get("count")) and _is_non_negative_int(
                detail.get("inputs_count")
            ):
                if int(detail["count"]) < int(detail["inputs_count"]):
                    errors.append(
                        f"{prefix}.count must be greater than or equal to "
                        "inputs_count"
                    )
        duplicate_detail_codes = _duplicate_non_empty_strings(detail_codes)
        if duplicate_detail_codes:
            errors.append(
                "reason_details codes must be unique: "
                f"{', '.join(duplicate_detail_codes)}"
            )
        duplicate_detail_messages = _duplicate_non_empty_strings(detail_messages)
        if duplicate_detail_messages:
            errors.append(
                "reason_details messages must be unique: "
                f"{'; '.join(duplicate_detail_messages)}"
            )
    if isinstance(payload.get("reason_codes"), list) and isinstance(
        payload.get("reason_details"), list
    ):
        reason_codes = [
            str(code) for code in payload["reason_codes"] if _is_non_empty_string(code)
        ]
        missing_from_details = sorted(set(reason_codes) - set(detail_codes))
        missing_from_codes = sorted(set(detail_codes) - set(reason_codes))
        if missing_from_details:
            errors.append(
                "reason_codes missing matching reason_details: "
                f"{', '.join(missing_from_details)}"
            )
        if missing_from_codes:
            errors.append(
                "reason_details codes missing from reason_codes: "
                f"{', '.join(missing_from_codes)}"
            )
        if reason_codes != detail_codes:
            errors.append("reason_codes must match reason_details codes in order")
    if isinstance(payload.get("reasons"), list) and isinstance(
        payload.get("reason_details"), list
    ):
        reasons = [
            str(reason) for reason in payload["reasons"] if _is_non_empty_string(reason)
        ]
        missing_messages_from_details = sorted(set(reasons) - set(detail_messages))
        missing_messages_from_reasons = sorted(set(detail_messages) - set(reasons))
        if missing_messages_from_details:
            errors.append(
                "reasons missing matching reason_details messages: "
                f"{'; '.join(missing_messages_from_details)}"
            )
        if missing_messages_from_reasons:
            errors.append(
                "reason_details messages missing from reasons: "
                f"{'; '.join(missing_messages_from_reasons)}"
            )
        if reasons != detail_messages:
            errors.append("reasons must match reason_details messages in order")
    if _is_non_negative_int(payload.get("exit_code")) and isinstance(
        payload.get("passed"), bool
    ):
        expected_passed = payload["exit_code"] == 0
        if payload["passed"] is not expected_passed:
            errors.append("passed must equal whether exit_code is zero")
        if (
            payload.get("complete") is True
            and payload["passed"]
            is not DELIVERY_ACCEPTANCE_GATE_COMPLETE_RESULT_REQUIREMENTS["passed"]
        ):
            errors.append("passed must be true when complete is true")
        if (
            payload.get("complete") is True
            and int(payload["exit_code"])
            != DELIVERY_ACCEPTANCE_GATE_COMPLETE_RESULT_REQUIREMENTS["exit_code"]
        ):
            errors.append("exit_code must be zero when complete is true")
        if not expected_passed:
            for key in ["reasons", "reason_codes", "reason_details"]:
                if isinstance(payload.get(key), list) and not payload[key]:
                    errors.append(f"{key} must not be empty when exit_code is non-zero")
        if (
            payload.get("required") is True
            and payload.get("complete") is False
            and int(payload["exit_code"]) == 0
        ):
            errors.append("exit_code must be non-zero when required gate is incomplete")
        if (
            payload.get("requires_full_mechanical_restoration_gate") is True
            and payload.get("complete") is False
            and int(payload["exit_code"]) == 0
        ):
            errors.append(
                "exit_code must be non-zero when full mechanical restoration gate "
                "is incomplete"
            )
    if payload.get("complete") is True:
        for key in DELIVERY_ACCEPTANCE_GATE_COMPLETE_RESULT_REQUIREMENTS["empty_lists"]:
            if isinstance(payload.get(key), list) and payload[key]:
                errors.append(f"{key} must be empty when complete is true")

    summary_counts = payload.get("summary_counts")
    if not isinstance(summary_counts, Mapping):
        errors.append("summary_counts must be an object")
    else:
        allowed_summary_fields = (
            DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS
            | DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_FIELDS
        )
        unknown_summary_fields = sorted(set(summary_counts) - allowed_summary_fields)
        if unknown_summary_fields:
            errors.append(
                "summary_counts contains unknown fields: "
                f"{', '.join(str(key) for key in unknown_summary_fields)}"
            )
        if _is_non_empty_string(source):
            allowed_source_summary_fields = (
                DELIVERY_ACCEPTANCE_GATE_SOURCE_SUMMARY_FIELDS.get(str(source))
            )
            if allowed_source_summary_fields is not None:
                invalid_source_summary_fields = sorted(
                    {
                        str(key)
                        for key in summary_counts
                        if key in allowed_summary_fields
                        and key not in allowed_source_summary_fields
                    }
                )
                if invalid_source_summary_fields:
                    errors.append(
                        "summary_counts contains fields not allowed for source "
                        f"{source!r}: {', '.join(invalid_source_summary_fields)}"
                    )
        for key in sorted(DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS):
            if key in summary_counts and not _is_non_negative_int(summary_counts[key]):
                errors.append(f"summary_counts.{key} must be a non-negative integer")
        if (
            payload.get("complete") is True
            and _is_non_negative_int(summary_counts.get("inputs_count"))
            and int(summary_counts["inputs_count"]) == 0
        ):
            errors.append(
                "summary_counts.inputs_count must be greater than zero when "
                "complete is true"
            )
        if payload.get("complete") is True and _is_non_negative_int(
            summary_counts.get("inputs_count")
        ):
            inputs_count = int(summary_counts["inputs_count"])
            complete_summary_fields = (
                DELIVERY_ACCEPTANCE_GATE_SOURCE_SUMMARY_FIELDS.get(str(source))
                if _is_non_empty_string(source)
                else None
            )
            if complete_summary_fields is None:
                complete_summary_fields = (
                    DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_FIELDS
                    | DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_FIELDS
                )
            for key in sorted(
                DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_EQUAL_INPUTS
                & complete_summary_fields
            ):
                _validate_complete_summary_count(
                    errors,
                    summary_counts,
                    key,
                    inputs_count,
                )
            for key in sorted(
                DELIVERY_ACCEPTANCE_GATE_COMPLETE_SUMMARY_ZERO & complete_summary_fields
            ):
                _validate_complete_summary_count(errors, summary_counts, key, 0)
            scope_counts = DELIVERY_ACCEPTANCE_GATE_SCOPE_COMPLETE_SUMMARY_COUNTS.get(
                str(payload.get("verification_scope"))
            )
            if scope_counts is not None:
                for key in sorted(
                    scope_counts["equal_inputs"] & complete_summary_fields
                ):
                    _validate_complete_summary_count(
                        errors,
                        summary_counts,
                        key,
                        inputs_count,
                    )
                for key in sorted(scope_counts["zero"] & complete_summary_fields):
                    _validate_complete_summary_count(
                        errors,
                        summary_counts,
                        key,
                        0,
                    )
            if isinstance(requirements, Mapping):
                for (
                    summary_map_key,
                    requirement_map,
                ) in DELIVERY_ACCEPTANCE_GATE_REQUIREMENT_SUMMARY_MAP_COUNTS.items():
                    for requirement_key, count_key in requirement_map.items():
                        if requirements.get(requirement_key) is True:
                            _validate_complete_summary_map_count(
                                errors,
                                summary_counts,
                                summary_map_key,
                                count_key,
                                inputs_count,
                            )
        for key in sorted(DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_FIELDS):
            value = summary_counts.get(key)
            if value is None:
                continue
            if not isinstance(value, Mapping):
                errors.append(f"summary_counts.{key} must be an object")
                continue
            allowed_map_keys = DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_MAP_KEY_VALUES[
                key
            ]
            unknown_map_keys = sorted(
                str(count_key)
                for count_key in value
                if count_key not in allowed_map_keys
            )
            if unknown_map_keys:
                errors.append(
                    f"summary_counts.{key} contains unknown check keys: "
                    f"{', '.join(unknown_map_keys)}"
                )
            invalid = sorted(
                str(count_key)
                for count_key, count_value in value.items()
                if not _is_non_negative_int(count_value)
            )
            if invalid:
                errors.append(
                    f"summary_counts.{key} values must be non-negative integers: "
                    f"{', '.join(invalid)}"
                )
        for rule in DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_SUM_RULES:
            _validate_summary_count_sum(
                errors,
                summary_counts,
                rule["counts"],
                str(rule["total"]),
                str(rule["message"]),
            )
        for rule in DELIVERY_ACCEPTANCE_GATE_SUMMARY_COUNT_LTE_RULES:
            _validate_summary_count_lte(
                errors,
                summary_counts,
                str(rule["count"]),
                str(rule["max"]),
            )
        for rule in DELIVERY_ACCEPTANCE_GATE_SUMMARY_MAP_SUM_RULES:
            _validate_summary_map_sum(
                errors,
                summary_counts,
                str(rule["map"]),
                str(rule["total"]),
            )

    return errors


WORKFLOW_STEP_STATUSES = {"pending", "running", "completed", "failed", "skipped"}
WORKFLOW_EXECUTOR_MODES = {"mock", "real"}

WORKFLOW_STEP_ARTIFACT_REQUIRED_FIELDS = {
    "schema_version",
    "artifact_type",
    "workflow",
    "step",
    "executor",
    "action",
    "status",
    "mode",
    "inputs",
    "output",
    "artifact_index",
    "attempts",
    "duration_seconds",
    "created_at",
}

WORKFLOW_STEP_ARTIFACT_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "artifact_type": "workflow_step",
    "required": sorted(WORKFLOW_STEP_ARTIFACT_REQUIRED_FIELDS),
    "status_values": sorted(WORKFLOW_STEP_STATUSES),
    "mode_values": sorted(WORKFLOW_EXECUTOR_MODES),
}

ROBOT_CONFIG_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "mechanical_schema_version": ROBOT_MECHANICAL_SCHEMA_VERSION,
    "required": ["name", "parts", "connections"],
    "part_required": ["id", "type", "params"],
    "connection_required": ["from", "to", "joint_type"],
    "godot_dynamic_required": [
        "parts[].shape",
        "parts[].params.position",
        "connections[].origin",
        "connections[].axis",
        "connections[].limits",
        "connections[].motor",
    ],
}

PART_SPEC_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "required": ["id", "category", "name", "weight_kg", "cost_usd", "specs"],
}

OPTIMIZATION_RESULT_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "required": ["success", "iterations", "mass_distribution", "com_position"],
    "optional": ["com_error", "message", "parameters", "final_value"],
}

EXPORT_RESULT_SCHEMA = {
    "schema_version": WORKFLOW_CONTRACT_VERSION,
    "required": ["status", "action", "output_file", "format"],
    "format_values": ["urdf", "sdf", "mjcf"],
}


def to_jsonable(value: Any) -> Any:
    """Convert common executor return values into deterministic JSON payloads."""
    if is_dataclass(value):
        return to_jsonable(asdict(value))
    if isinstance(value, Mapping):
        return {str(key): to_jsonable(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [to_jsonable(item) for item in value]
    if hasattr(value, "tolist") and callable(value.tolist):
        return to_jsonable(value.tolist())
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def build_workflow_step_artifact(
    *,
    workflow: str,
    step: str,
    executor: str,
    action: str,
    status: str,
    mode: str,
    inputs: Mapping[str, Any],
    output: Mapping[str, Any],
    artifact_index: int,
    attempts: int,
    duration_seconds: float,
    created_at: str,
    error: str | None = None,
    error_type: str | None = None,
) -> dict[str, Any]:
    """Create the canonical JSON shape for a persisted workflow step."""
    return {
        "schema_version": WORKFLOW_CONTRACT_VERSION,
        "artifact_type": "workflow_step",
        "workflow": workflow,
        "step": step,
        "executor": executor,
        "action": action,
        "status": status,
        "mode": mode,
        "inputs": to_jsonable(dict(inputs)),
        "output": to_jsonable(dict(output)),
        "artifact_index": artifact_index,
        "attempts": attempts,
        "duration_seconds": round(float(duration_seconds), 6),
        "created_at": created_at,
        "error": error,
        "error_type": error_type,
    }


def validate_workflow_step_artifact(payload: Any) -> list[str]:
    """Return validation errors for a workflow-step artifact payload."""
    if not isinstance(payload, Mapping):
        return ["artifact must be an object"]

    errors: list[str] = []
    missing = sorted(WORKFLOW_STEP_ARTIFACT_REQUIRED_FIELDS - set(payload))
    if missing:
        errors.append(f"missing required fields: {', '.join(missing)}")

    if payload.get("schema_version") != WORKFLOW_CONTRACT_VERSION:
        errors.append(
            f"schema_version must be {WORKFLOW_CONTRACT_VERSION!r}, got {payload.get('schema_version')!r}"
        )
    if payload.get("artifact_type") != "workflow_step":
        errors.append("artifact_type must be 'workflow_step'")

    for key in [
        "workflow",
        "step",
        "executor",
        "action",
        "status",
        "mode",
        "created_at",
    ]:
        if key in payload and not _is_non_empty_string(payload.get(key)):
            errors.append(f"{key} must be a non-empty string")

    if payload.get("status") not in WORKFLOW_STEP_STATUSES:
        errors.append(f"status must be one of {sorted(WORKFLOW_STEP_STATUSES)}")
    if payload.get("mode") not in WORKFLOW_EXECUTOR_MODES:
        errors.append(f"mode must be one of {sorted(WORKFLOW_EXECUTOR_MODES)}")

    if "inputs" in payload and not isinstance(payload.get("inputs"), Mapping):
        errors.append("inputs must be an object")
    if "output" in payload and not isinstance(payload.get("output"), Mapping):
        errors.append("output must be an object")
    if "artifact_index" in payload and not _is_non_negative_int(
        payload.get("artifact_index")
    ):
        errors.append("artifact_index must be a non-negative integer")
    if "attempts" in payload and not _is_non_negative_int(payload.get("attempts")):
        errors.append("attempts must be a non-negative integer")
    if "duration_seconds" in payload and not _is_non_negative_number(
        payload.get("duration_seconds")
    ):
        errors.append("duration_seconds must be a non-negative number")

    return errors


def validate_workflow_definition(
    workflow_name: str,
    workflow: Any,
    executor_actions: Mapping[str, set[str]],
) -> list[str]:
    """Validate the built-in/custom workflow definition shape."""
    if not isinstance(workflow, Mapping):
        return [f"workflow {workflow_name!r} must be an object"]

    errors: list[str] = []
    if workflow.get("name") != workflow_name:
        errors.append(
            f"workflow name mismatch: expected {workflow_name!r}, got {workflow.get('name')!r}"
        )

    steps = workflow.get("steps")
    if not isinstance(steps, list) or not steps:
        return errors + ["workflow steps must be a non-empty list"]

    seen_steps: set[str] = set()
    previous_steps: set[str] = set()
    for index, step in enumerate(steps, start=1):
        prefix = f"steps[{index}]"
        if not isinstance(step, Mapping):
            errors.append(f"{prefix} must be an object")
            continue

        step_name = step.get("name")
        executor_name = step.get("skill_executor")
        action = step.get("action")
        inputs = step.get("inputs", {})

        if not _is_non_empty_string(step_name):
            errors.append(f"{prefix}.name must be a non-empty string")
        elif step_name in seen_steps:
            errors.append(f"{prefix}.name duplicates step {step_name!r}")
        else:
            seen_steps.add(step_name)

        if not _is_non_empty_string(executor_name):
            errors.append(f"{prefix}.skill_executor must be a non-empty string")
        elif executor_name not in executor_actions:
            errors.append(
                f"{prefix}.skill_executor {executor_name!r} is not registered"
            )

        if not _is_non_empty_string(action):
            errors.append(f"{prefix}.action must be a non-empty string")
        elif (
            _is_non_empty_string(executor_name)
            and executor_name in executor_actions
            and action not in executor_actions[executor_name]
        ):
            supported = ", ".join(sorted(executor_actions[executor_name]))
            errors.append(
                f"{prefix}.action {action!r} is not supported by {executor_name!r}; supported: {supported}"
            )

        if not isinstance(inputs, Mapping):
            errors.append(f"{prefix}.inputs must be an object")
        else:
            for ref in _iter_exact_references(inputs):
                if "." not in ref:
                    errors.append(
                        f"{prefix}.inputs reference {{{ref}}} must be step.key"
                    )
                    continue
                ref_step, ref_key = ref.split(".", 1)
                if not ref_step or not ref_key:
                    errors.append(
                        f"{prefix}.inputs reference {{{ref}}} must be step.key"
                    )
                elif ref_step not in previous_steps:
                    errors.append(
                        f"{prefix}.inputs reference {{{ref}}} points to a missing or future step"
                    )

        if _is_non_empty_string(step_name):
            previous_steps.add(step_name)

    return errors


def validate_robot_config(payload: Any) -> list[str]:
    """Validate the phase-one RobotConfig JSON contract."""
    if not isinstance(payload, Mapping):
        return ["robot config must be an object"]

    errors: list[str] = []
    if not _is_non_empty_string(payload.get("name")):
        errors.append("name must be a non-empty string")

    parts = payload.get("parts")
    if not isinstance(parts, list):
        errors.append("parts must be a list")
    else:
        for index, part in enumerate(parts, start=1):
            prefix = f"parts[{index}]"
            if not isinstance(part, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            if not _is_non_empty_string(part.get("id")):
                errors.append(f"{prefix}.id must be a non-empty string")
            if not _is_non_empty_string(part.get("type")):
                errors.append(f"{prefix}.type must be a non-empty string")
            if not isinstance(part.get("params"), Mapping):
                errors.append(f"{prefix}.params must be an object")

    connections = payload.get("connections")
    if not isinstance(connections, list):
        errors.append("connections must be a list")
    else:
        for index, connection in enumerate(connections, start=1):
            prefix = f"connections[{index}]"
            if not isinstance(connection, Mapping):
                errors.append(f"{prefix} must be an object")
                continue
            for field in ["from", "to", "joint_type"]:
                if not _is_non_empty_string(connection.get(field)):
                    errors.append(f"{prefix}.{field} must be a non-empty string")

    if "metadata" in payload and not isinstance(payload.get("metadata"), Mapping):
        errors.append("metadata must be an object when present")

    if not errors:
        normalized = normalize_robot_config_for_godot(payload)
        errors.extend(validate_godot_robot_config(normalized))
    return errors


def validate_delivery_acceptance_validation_summary(payload: Any) -> list[str]:
    """Return validation errors for a validation-summary artifact payload."""
    if not isinstance(payload, Mapping):
        return ["delivery_acceptance_gate validation summary must be an object"]

    errors: list[str] = []
    missing = sorted(
        DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_REQUIRED_FIELDS - set(payload)
    )
    if missing:
        errors.append(f"missing required summary fields: {', '.join(missing)}")

    if (
        payload.get("summary_version")
        != DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    ):
        errors.append(
            "summary_version must be "
            f"{DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION!r}, "
            f"got {payload.get('summary_version')!r}"
        )

    status = payload.get("status")
    if not _is_non_empty_string(status):
        errors.append("status must be a non-empty string")
    elif status not in DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_STATUSES:
        errors.append(
            "status must be one of: "
            f"{', '.join(sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_STATUSES))}"
        )

    for key in [
        "expanded_inputs_count",
        "inputs_count",
        "success_count",
        "skipped_count",
        "error_count",
    ]:
        if key in payload and not _is_non_negative_int(payload.get(key)):
            errors.append(f"{key} must be a non-negative integer")

    if (
        _is_non_negative_int(payload.get("expanded_inputs_count"))
        and _is_non_negative_int(payload.get("inputs_count"))
        and int(payload["expanded_inputs_count"]) < int(payload["inputs_count"])
    ):
        errors.append("expanded_inputs_count must be >= inputs_count")

    for field in sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELDS):
        if field in payload and not isinstance(payload.get(field), bool):
            errors.append(f"{field} must be a boolean when present")

    result_count_keys = ["success_count", "skipped_count", "error_count"]
    if _is_non_negative_int(payload.get("inputs_count")) and all(
        _is_non_negative_int(payload.get(key)) for key in result_count_keys
    ):
        result_total = sum(int(payload[key]) for key in result_count_keys)
        if result_total != int(payload["inputs_count"]):
            errors.append(
                "success_count + skipped_count + error_count must equal inputs_count"
            )

    summary_errors = payload.get("errors")
    if not isinstance(summary_errors, list):
        errors.append("errors must be a list")
    else:
        invalid_errors = [
            str(index)
            for index, error in enumerate(summary_errors, start=1)
            if not _is_non_empty_string(error)
        ]
        if invalid_errors:
            errors.append(
                "errors entries must be non-empty strings: "
                f"{', '.join(invalid_errors)}"
            )
        duplicate_errors = _duplicate_non_empty_strings(summary_errors)
        if duplicate_errors:
            errors.append(
                "errors entries must be unique: " f"{'; '.join(duplicate_errors)}"
            )

    _validate_optional_summary_metadata_preview(
        errors,
        payload,
        field="summary_versions",
        count_field="summary_versions_count",
        truncated_field="summary_versions_truncated",
        required_value=payload.get("summary_version"),
    )
    _validate_optional_summary_metadata_preview(
        errors,
        payload,
        field="validation_summary_statuses",
        count_field="validation_summary_statuses_count",
        truncated_field="validation_summary_statuses_truncated",
        required_value=payload.get("status"),
    )
    _validate_optional_node_tree_manifest_sidecar_summary(errors, payload)
    _validate_optional_summary_constraint_audit(
        errors,
        payload,
        observed_field="summary_versions",
        expected_field="expected_summary_versions",
        missing_expected_field="missing_expected_summary_versions",
        allowed_field="allowed_summary_versions",
        unexpected_field="unexpected_summary_versions",
        forbidden_field="forbidden_summary_versions",
        present_forbidden_field="present_forbidden_summary_versions",
    )
    _validate_optional_summary_constraint_audit(
        errors,
        payload,
        observed_field="validation_summary_statuses",
        expected_field="expected_validation_summary_statuses",
        missing_expected_field="missing_expected_validation_summary_statuses",
        allowed_field="allowed_validation_summary_statuses",
        unexpected_field="unexpected_validation_summary_statuses",
        forbidden_field="forbidden_validation_summary_statuses",
        present_forbidden_field="present_forbidden_validation_summary_statuses",
    )
    for field in [
        "expected_node_tree_manifest_sidecar_complete_count",
        "expected_node_tree_manifest_sidecar_count",
        "expected_node_tree_manifest_sidecar_incomplete_count",
        "expected_node_tree_manifest_sidecar_invalid_count",
        "expected_node_tree_manifest_sidecar_joint_path_count",
        "expected_node_tree_manifest_sidecar_joints_planned_count",
        "expected_node_tree_manifest_sidecar_part_path_count",
        "expected_node_tree_manifest_sidecar_parts_planned_count",
        "expected_node_tree_manifest_sidecar_path_incomplete_count",
        "expected_node_tree_manifest_sidecar_path_map_mismatch_count",
        "expected_node_tree_manifest_sidecar_valid_count",
        "expected_node_tree_manifest_sidecar_validation_error_count",
        "expected_summary_versions_count",
        "expected_validation_summary_statuses_count",
    ]:
        value = payload.get(field)
        if value is not None and not _is_non_negative_int(value):
            errors.append(
                f"{field} must be a non-negative integer or null when present"
            )
    _validate_non_negative_int_map(
        errors,
        payload.get(
            "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
        ),
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts",
    )

    return errors


def _validate_optional_node_tree_manifest_sidecar_summary(
    errors: list[str],
    payload: Mapping[str, Any],
) -> None:
    count_fields = [
        "node_tree_manifest_sidecar_count",
        "node_tree_manifest_sidecar_complete_count",
        "node_tree_manifest_sidecar_incomplete_count",
        "node_tree_manifest_sidecar_valid_count",
        "node_tree_manifest_sidecar_invalid_count",
        "node_tree_manifest_sidecar_path_incomplete_count",
        "node_tree_manifest_sidecar_path_map_mismatch_count",
        "node_tree_manifest_sidecar_parts_planned_count",
        "node_tree_manifest_sidecar_joints_planned_count",
        "node_tree_manifest_sidecar_part_path_count",
        "node_tree_manifest_sidecar_joint_path_count",
        "node_tree_manifest_sidecar_validation_error_count",
    ]
    if not any(field in payload for field in count_fields) and not any(
        field in payload
        for field in [
            "node_tree_manifest_sidecars",
            "node_tree_manifest_sidecars_truncated",
        ]
    ):
        return

    for field in count_fields:
        if not _is_non_negative_int(payload.get(field)):
            errors.append(f"{field} must be a non-negative integer when present")
    _validate_non_negative_int_map(
        errors,
        payload.get("node_tree_manifest_sidecar_path_map_mismatch_kind_counts"),
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts",
    )
    top_level_kind_counts = payload.get(
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts"
    )
    top_level_mismatch_count = payload.get(
        "node_tree_manifest_sidecar_path_map_mismatch_count"
    )
    if isinstance(top_level_kind_counts, Mapping) and _is_non_negative_int(
        top_level_mismatch_count
    ):
        kind_count_sum = sum(
            int(value)
            for value in top_level_kind_counts.values()
            if _is_non_negative_int(value)
        )
        if kind_count_sum != int(top_level_mismatch_count):
            errors.append(
                "node_tree_manifest_sidecar_path_map_mismatch_kind_counts "
                "must sum to node_tree_manifest_sidecar_path_map_mismatch_count"
            )

    sidecars = payload.get("node_tree_manifest_sidecars")
    if not isinstance(sidecars, list):
        errors.append("node_tree_manifest_sidecars must be a list when present")
        return
    invalid_sidecars = [
        str(index)
        for index, item in enumerate(sidecars, start=1)
        if not isinstance(item, Mapping)
    ]
    if invalid_sidecars:
        errors.append(
            "node_tree_manifest_sidecars entries must be objects: "
            f"{', '.join(invalid_sidecars)}"
        )

    truncated = payload.get("node_tree_manifest_sidecars_truncated")
    if not isinstance(truncated, bool):
        errors.append(
            "node_tree_manifest_sidecars_truncated must be a boolean when present"
        )

    count = payload.get("node_tree_manifest_sidecar_count")
    if _is_non_negative_int(count):
        if int(count) < len(sidecars):
            errors.append(
                "node_tree_manifest_sidecar_count must be >= "
                "node_tree_manifest_sidecars preview length"
            )
        if truncated is False and int(count) != len(sidecars):
            errors.append(
                "node_tree_manifest_sidecar_count must equal "
                "node_tree_manifest_sidecars preview length"
            )

    complete = payload.get("node_tree_manifest_sidecar_complete_count")
    incomplete = payload.get("node_tree_manifest_sidecar_incomplete_count")
    if all(_is_non_negative_int(value) for value in [count, complete, incomplete]):
        if int(complete) + int(incomplete) > int(count):
            errors.append(
                "node_tree_manifest_sidecar_complete_count + "
                "node_tree_manifest_sidecar_incomplete_count must be <= "
                "node_tree_manifest_sidecar_count"
            )
    valid = payload.get("node_tree_manifest_sidecar_valid_count")
    invalid = payload.get("node_tree_manifest_sidecar_invalid_count")
    if all(_is_non_negative_int(value) for value in [count, valid, invalid]):
        if int(valid) + int(invalid) > int(count):
            errors.append(
                "node_tree_manifest_sidecar_valid_count + "
                "node_tree_manifest_sidecar_invalid_count must be <= "
                "node_tree_manifest_sidecar_count"
            )
    path_incomplete = payload.get("node_tree_manifest_sidecar_path_incomplete_count")
    if all(_is_non_negative_int(value) for value in [count, path_incomplete]):
        if int(path_incomplete) > int(count):
            errors.append(
                "node_tree_manifest_sidecar_path_incomplete_count must be <= "
                "node_tree_manifest_sidecar_count"
            )

    parts_planned = payload.get("node_tree_manifest_sidecar_parts_planned_count")
    part_paths = payload.get("node_tree_manifest_sidecar_part_path_count")
    if all(_is_non_negative_int(value) for value in [parts_planned, part_paths]):
        if int(part_paths) > int(parts_planned):
            errors.append(
                "node_tree_manifest_sidecar_part_path_count must be <= "
                "node_tree_manifest_sidecar_parts_planned_count"
            )
    joints_planned = payload.get("node_tree_manifest_sidecar_joints_planned_count")
    joint_paths = payload.get("node_tree_manifest_sidecar_joint_path_count")
    if all(_is_non_negative_int(value) for value in [joints_planned, joint_paths]):
        if int(joint_paths) > int(joints_planned):
            errors.append(
                "node_tree_manifest_sidecar_joint_path_count must be <= "
                "node_tree_manifest_sidecar_joints_planned_count"
            )

    for index, item in enumerate(sidecars, start=1):
        if not isinstance(item, Mapping):
            continue
        prefix = f"node_tree_manifest_sidecars[{index}]"
        for field in [
            "parts_count",
            "joints_count",
            "part_node_path_count",
            "joint_node_path_count",
            "node_tree_manifest_validation_error_count",
        ]:
            if field in item and not _is_non_negative_int(item.get(field)):
                errors.append(f"{prefix}.{field} must be a non-negative integer")
        validation_error_count = item.get("node_tree_manifest_validation_error_count")
        validation_errors = item.get("node_tree_manifest_validation_errors")
        if "node_tree_manifest_validation_errors" in item:
            if not isinstance(validation_errors, list) or not all(
                isinstance(error, str) for error in validation_errors
            ):
                errors.append(
                    f"{prefix}.node_tree_manifest_validation_errors must be "
                    "a list of strings"
                )
            elif _is_non_negative_int(validation_error_count) and int(
                validation_error_count
            ) != len(validation_errors):
                errors.append(
                    f"{prefix}.node_tree_manifest_validation_error_count "
                    "must equal node_tree_manifest_validation_errors length"
                )
        if "path_maps_complete" in item and not isinstance(
            item.get("path_maps_complete"), bool
        ):
            errors.append(f"{prefix}.path_maps_complete must be a boolean")
        mismatch_count = item.get("node_tree_manifest_path_map_mismatch_count")
        mismatches = item.get("node_tree_manifest_path_map_mismatches")
        kind_counts = item.get("node_tree_manifest_path_map_mismatch_kind_counts")
        if "node_tree_manifest_path_map_mismatch_kind_counts" in item:
            _validate_non_negative_int_map(
                errors,
                kind_counts,
                f"{prefix}.node_tree_manifest_path_map_mismatch_kind_counts",
            )
            if isinstance(kind_counts, Mapping) and _is_non_negative_int(
                mismatch_count
            ):
                kind_count_sum = sum(
                    int(value)
                    for value in kind_counts.values()
                    if _is_non_negative_int(value)
                )
                if kind_count_sum != int(mismatch_count):
                    errors.append(
                        f"{prefix}.node_tree_manifest_path_map_mismatch_kind_counts "
                        "must sum to node_tree_manifest_path_map_mismatch_count"
                    )
        if (
            "node_tree_manifest_path_map_mismatch_count" in item
            and not _is_non_negative_int(mismatch_count)
        ):
            errors.append(
                f"{prefix}.node_tree_manifest_path_map_mismatch_count must be "
                "a non-negative integer"
            )
        if "node_tree_manifest_path_map_mismatches" in item:
            if not isinstance(mismatches, list):
                errors.append(
                    f"{prefix}.node_tree_manifest_path_map_mismatches must be a list"
                )
            else:
                invalid_mismatch_indexes = [
                    str(mismatch_index)
                    for mismatch_index, mismatch in enumerate(mismatches, start=1)
                    if not isinstance(mismatch, Mapping)
                ]
                if invalid_mismatch_indexes:
                    errors.append(
                        f"{prefix}.node_tree_manifest_path_map_mismatches "
                        "entries must be objects: "
                        f"{', '.join(invalid_mismatch_indexes)}"
                    )
                for mismatch_index, mismatch in enumerate(mismatches, start=1):
                    if isinstance(mismatch, Mapping):
                        _validate_sidecar_path_map_mismatch_entry(
                            errors,
                            mismatch,
                            f"{prefix}.node_tree_manifest_path_map_mismatches[{mismatch_index}]",
                        )
                if _is_non_negative_int(mismatch_count) and int(mismatch_count) < len(
                    mismatches
                ):
                    errors.append(
                        f"{prefix}.node_tree_manifest_path_map_mismatch_count "
                        "must be >= node_tree_manifest_path_map_mismatches "
                        "preview length"
                    )
                if (
                    truncated is False
                    and _is_non_negative_int(mismatch_count)
                    and int(mismatch_count) != len(mismatches)
                ):
                    errors.append(
                        f"{prefix}.node_tree_manifest_path_map_mismatch_count "
                        "must equal node_tree_manifest_path_map_mismatches "
                        "preview length when sidecar preview is not truncated"
                    )
        item_parts = item.get("parts_count")
        item_part_paths = item.get("part_node_path_count")
        if all(_is_non_negative_int(value) for value in [item_parts, item_part_paths]):
            if int(item_part_paths) > int(item_parts):
                errors.append(f"{prefix}.part_node_path_count must be <= parts_count")
        item_joints = item.get("joints_count")
        item_joint_paths = item.get("joint_node_path_count")
        if all(
            _is_non_negative_int(value) for value in [item_joints, item_joint_paths]
        ):
            if int(item_joint_paths) > int(item_joints):
                errors.append(f"{prefix}.joint_node_path_count must be <= joints_count")
        if isinstance(item.get("path_maps_complete"), bool) and all(
            _is_non_negative_int(value)
            for value in [
                item_parts,
                item_joints,
                item_part_paths,
                item_joint_paths,
            ]
        ):
            expected_path_maps_complete = int(item_part_paths) >= int(
                item_parts
            ) and int(item_joint_paths) >= int(item_joints)
            if item["path_maps_complete"] is not expected_path_maps_complete:
                errors.append(f"{prefix}.path_maps_complete must match path map counts")

    if truncated is False and not invalid_sidecars:
        _validate_sidecar_preview_count_sum(
            errors,
            payload,
            sidecars,
            item_field="parts_count",
            aggregate_field="node_tree_manifest_sidecar_parts_planned_count",
        )
        _validate_sidecar_preview_count_sum(
            errors,
            payload,
            sidecars,
            item_field="joints_count",
            aggregate_field="node_tree_manifest_sidecar_joints_planned_count",
        )
        _validate_sidecar_preview_count_sum(
            errors,
            payload,
            sidecars,
            item_field="part_node_path_count",
            aggregate_field="node_tree_manifest_sidecar_part_path_count",
        )
        _validate_sidecar_preview_count_sum(
            errors,
            payload,
            sidecars,
            item_field="joint_node_path_count",
            aggregate_field="node_tree_manifest_sidecar_joint_path_count",
        )
        _validate_sidecar_preview_boolean_counts(
            errors,
            payload,
            sidecars,
            item_field="complete",
            true_field="node_tree_manifest_sidecar_complete_count",
            false_field="node_tree_manifest_sidecar_incomplete_count",
        )
        _validate_sidecar_preview_boolean_counts(
            errors,
            payload,
            sidecars,
            item_field="node_tree_manifest_valid",
            true_field="node_tree_manifest_sidecar_valid_count",
            false_field="node_tree_manifest_sidecar_invalid_count",
        )
        _validate_sidecar_preview_path_incomplete_count(
            errors,
            payload,
            sidecars,
        )
        _validate_sidecar_preview_count_sum(
            errors,
            payload,
            sidecars,
            item_field="node_tree_manifest_path_map_mismatch_count",
            aggregate_field="node_tree_manifest_sidecar_path_map_mismatch_count",
        )
        _validate_sidecar_preview_count_sum(
            errors,
            payload,
            sidecars,
            item_field="node_tree_manifest_validation_error_count",
            aggregate_field="node_tree_manifest_sidecar_validation_error_count",
        )
        _validate_sidecar_preview_kind_count_sum(
            errors,
            payload,
            sidecars,
        )


def _validate_non_negative_int_map(
    errors: list[str],
    value: Any,
    prefix: str,
) -> None:
    if value is None:
        return
    if not isinstance(value, Mapping):
        errors.append(f"{prefix} must be an object when present")
        return
    invalid_keys = [str(key) for key in value if not _is_non_empty_string(key)]
    if invalid_keys:
        errors.append(f"{prefix} keys must be non-empty strings")
    invalid_values = [
        str(key) for key, item in value.items() if not _is_non_negative_int(item)
    ]
    if invalid_values:
        errors.append(
            f"{prefix} values must be non-negative integers: "
            f"{', '.join(invalid_values)}"
        )


def _validate_sidecar_preview_kind_count_sum(
    errors: list[str],
    payload: Mapping[str, Any],
    sidecars: list[Any],
) -> None:
    aggregate = payload.get("node_tree_manifest_sidecar_path_map_mismatch_kind_counts")
    if not isinstance(aggregate, Mapping):
        return
    summed: dict[str, int] = {}
    for item in sidecars:
        if not isinstance(item, Mapping):
            return
        kind_counts = item.get("node_tree_manifest_path_map_mismatch_kind_counts")
        if not isinstance(kind_counts, Mapping):
            return
        for kind, count in kind_counts.items():
            if not _is_non_empty_string(kind) or not _is_non_negative_int(count):
                return
            summed[str(kind)] = summed.get(str(kind), 0) + int(count)
    normalized_aggregate = {
        str(kind): int(count)
        for kind, count in aggregate.items()
        if _is_non_empty_string(kind) and _is_non_negative_int(count)
    }
    if normalized_aggregate != summed:
        errors.append(
            "node_tree_manifest_sidecar_path_map_mismatch_kind_counts must equal "
            "node_tree_manifest_sidecars kind-count sum when preview is not "
            "truncated"
        )


def _validate_sidecar_preview_count_sum(
    errors: list[str],
    payload: Mapping[str, Any],
    sidecars: list[Any],
    *,
    item_field: str,
    aggregate_field: str,
) -> None:
    if aggregate_field not in payload:
        return
    values = [item.get(item_field) for item in sidecars if isinstance(item, Mapping)]
    if len(values) != len(sidecars) or not all(
        _is_non_negative_int(value) for value in values
    ):
        return
    aggregate = payload.get(aggregate_field)
    if _is_non_negative_int(aggregate) and int(aggregate) != sum(
        int(value) for value in values
    ):
        errors.append(
            f"{aggregate_field} must equal node_tree_manifest_sidecars "
            f"{item_field} sum when preview is not truncated"
        )


def _validate_sidecar_path_map_mismatch_entry(
    errors: list[str],
    mismatch: Mapping[str, Any],
    prefix: str,
) -> None:
    for field in ["map", "key", "kind"]:
        if not _is_non_empty_string(mismatch.get(field)):
            errors.append(f"{prefix}.{field} must be a non-empty string")
    kind = mismatch.get("kind")
    if kind not in {
        "duplicate",
        "missing",
        "root_mismatch",
        "unexpected",
        "value_mismatch",
    }:
        errors.append(
            f"{prefix}.kind must be one of duplicate, missing, root_mismatch, "
            "unexpected, value_mismatch"
        )
    if kind in {"root_mismatch", "value_mismatch"}:
        if not _is_non_empty_string(mismatch.get("field")):
            errors.append(f"{prefix}.field must be a non-empty string")
        if not _is_non_empty_string(mismatch.get("expected")):
            errors.append(f"{prefix}.expected must be a non-empty string")
        if "actual" in mismatch and not isinstance(mismatch.get("actual"), str):
            errors.append(f"{prefix}.actual must be a string when present")


def _validate_sidecar_preview_boolean_counts(
    errors: list[str],
    payload: Mapping[str, Any],
    sidecars: list[Any],
    *,
    item_field: str,
    true_field: str,
    false_field: str,
) -> None:
    values = [item.get(item_field) for item in sidecars if isinstance(item, Mapping)]
    if len(values) != len(sidecars) or not all(
        isinstance(value, bool) for value in values
    ):
        return
    true_count = sum(1 for value in values if value is True)
    false_count = sum(1 for value in values if value is False)
    if (
        _is_non_negative_int(payload.get(true_field))
        and int(payload[true_field]) != true_count
    ):
        errors.append(
            f"{true_field} must equal node_tree_manifest_sidecars "
            f"{item_field}=true count when preview is not truncated"
        )
    if (
        _is_non_negative_int(payload.get(false_field))
        and int(payload[false_field]) != false_count
    ):
        errors.append(
            f"{false_field} must equal node_tree_manifest_sidecars "
            f"{item_field}=false count when preview is not truncated"
        )


def _validate_sidecar_preview_path_incomplete_count(
    errors: list[str],
    payload: Mapping[str, Any],
    sidecars: list[Any],
) -> None:
    aggregate_field = "node_tree_manifest_sidecar_path_incomplete_count"
    if aggregate_field not in payload:
        return
    incomplete_count = 0
    for item in sidecars:
        if not isinstance(item, Mapping):
            return
        if isinstance(item.get("path_maps_complete"), bool):
            incomplete_count += item["path_maps_complete"] is False
            continue
        values = [
            item.get("parts_count"),
            item.get("joints_count"),
            item.get("part_node_path_count"),
            item.get("joint_node_path_count"),
        ]
        if not all(_is_non_negative_int(value) for value in values):
            return
        incomplete_count += int(item["part_node_path_count"]) < int(
            item["parts_count"]
        ) or int(item["joint_node_path_count"]) < int(item["joints_count"])
    aggregate = payload.get(aggregate_field)
    if _is_non_negative_int(aggregate) and int(aggregate) != incomplete_count:
        errors.append(
            f"{aggregate_field} must equal node_tree_manifest_sidecars "
            "path-incomplete count when preview is not truncated"
        )


def validate_part_spec(payload: Any) -> list[str]:
    """Validate a normalized PartSpec payload."""
    if not isinstance(payload, Mapping):
        return ["part spec must be an object"]

    errors: list[str] = []
    for field in ["id", "category", "name"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    for field in ["weight_kg", "cost_usd"]:
        if not _is_non_negative_number(payload.get(field)):
            errors.append(f"{field} must be a non-negative number")
    if not isinstance(payload.get("specs"), Mapping):
        errors.append("specs must be an object")
    return errors


def validate_optimization_result(payload: Any) -> list[str]:
    """Validate mass-optimization result payloads persisted by real workflows."""
    if not isinstance(payload, Mapping):
        return ["optimization result must be an object"]

    errors: list[str] = []
    if not isinstance(payload.get("success"), bool):
        errors.append("success must be a boolean")
    if not _is_non_negative_int(payload.get("iterations")):
        errors.append("iterations must be a non-negative integer")
    if not isinstance(payload.get("mass_distribution"), Mapping):
        errors.append("mass_distribution must be an object")
    com_position = payload.get("com_position")
    if not (
        isinstance(com_position, Sequence)
        and not isinstance(com_position, (str, bytes))
        and len(com_position) == 3
        and all(_is_number(item) for item in com_position)
    ):
        errors.append("com_position must be a 3-number sequence")
    if "com_error" in payload and not _is_non_negative_number(payload.get("com_error")):
        errors.append("com_error must be a non-negative number when present")
    return errors


def validate_export_result(payload: Any) -> list[str]:
    """Validate URDF/SDF/MJCF export executor result payloads."""
    if not isinstance(payload, Mapping):
        return ["export result must be an object"]

    errors: list[str] = []
    for field in ["status", "action", "output_file", "format"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("format") not in set(EXPORT_RESULT_SCHEMA["format_values"]):
        errors.append(f"format must be one of {EXPORT_RESULT_SCHEMA['format_values']}")
    if "file_size" in payload and not _is_non_negative_int(payload.get("file_size")):
        errors.append("file_size must be a non-negative integer when present")
    if "output_generated" in payload and not isinstance(
        payload.get("output_generated"), bool
    ):
        errors.append("output_generated must be a boolean when present")
    return errors


def _iter_exact_references(value: Any):
    if isinstance(value, Mapping):
        for item in value.values():
            yield from _iter_exact_references(item)
    elif isinstance(value, list):
        for item in value:
            yield from _iter_exact_references(item)
    elif isinstance(value, str) and value.startswith("{") and value.endswith("}"):
        yield value[1:-1]


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _validate_allowed_text(
    errors: list[str],
    payload: Mapping[str, Any],
    key: str,
    allowed: set[str],
) -> None:
    value = payload.get(key)
    if _is_non_empty_string(value) and value not in allowed:
        errors.append(f"{key} must be one of: {', '.join(sorted(allowed))}")


def _validate_optional_summary_metadata_preview(
    errors: list[str],
    payload: Mapping[str, Any],
    *,
    field: str,
    count_field: str,
    truncated_field: str,
    required_value: Any,
) -> None:
    values = payload.get(field)
    count = payload.get(count_field)
    truncated = payload.get(truncated_field)
    if values is None and count is None and truncated is None:
        return
    if not isinstance(values, list):
        errors.append(f"{field} must be a list when present")
        return
    invalid_values = [
        str(index)
        for index, value in enumerate(values, start=1)
        if not _is_non_empty_string(value)
    ]
    if invalid_values:
        errors.append(
            f"{field} entries must be non-empty strings: "
            f"{', '.join(invalid_values)}"
        )
    duplicate_values = _duplicate_non_empty_strings(values)
    if duplicate_values:
        errors.append(
            f"{field} entries must be unique: " f"{'; '.join(duplicate_values)}"
        )
    if not _is_non_negative_int(count):
        errors.append(f"{count_field} must be a non-negative integer when present")
    else:
        if int(count) < len(values):
            errors.append(f"{count_field} must be >= {field} preview length")
        if truncated is False and int(count) != len(values):
            errors.append(f"{count_field} must equal {field} preview length")
    if not isinstance(truncated, bool):
        errors.append(f"{truncated_field} must be a boolean when present")
    if values and _is_non_empty_string(required_value) and required_value not in values:
        errors.append(f"{field} must include {required_value!r}")


def _validate_optional_summary_constraint_audit(
    errors: list[str],
    payload: Mapping[str, Any],
    *,
    observed_field: str,
    expected_field: str,
    missing_expected_field: str,
    allowed_field: str,
    unexpected_field: str,
    forbidden_field: str,
    present_forbidden_field: str,
) -> None:
    observed = _validate_optional_string_list_field(errors, payload, observed_field)
    expected = _validate_optional_string_list_field(errors, payload, expected_field)
    missing_expected = _validate_optional_string_list_field(
        errors, payload, missing_expected_field
    )
    allowed = _validate_optional_string_list_field(errors, payload, allowed_field)
    unexpected = _validate_optional_string_list_field(errors, payload, unexpected_field)
    forbidden = _validate_optional_string_list_field(errors, payload, forbidden_field)
    present_forbidden = _validate_optional_string_list_field(
        errors, payload, present_forbidden_field
    )

    if expected is not None and missing_expected is not None:
        extra = [value for value in missing_expected if value not in expected]
        if extra:
            errors.append(
                f"{missing_expected_field} must be a subset of {expected_field}: "
                f"{', '.join(extra)}"
            )
    if observed is not None and missing_expected is not None:
        found = [value for value in missing_expected if value in observed]
        if found:
            errors.append(
                f"{missing_expected_field} must not include values present in "
                f"{observed_field}: {', '.join(found)}"
            )
    if allowed is not None and unexpected is not None:
        allowed_unexpected = [value for value in unexpected if value in allowed]
        if allowed_unexpected:
            errors.append(
                f"{unexpected_field} must not include values present in "
                f"{allowed_field}: {', '.join(allowed_unexpected)}"
            )
    if forbidden is not None and present_forbidden is not None:
        extra = [value for value in present_forbidden if value not in forbidden]
        if extra:
            errors.append(
                f"{present_forbidden_field} must be a subset of "
                f"{forbidden_field}: {', '.join(extra)}"
            )
    if observed is not None and present_forbidden is not None:
        missing = [value for value in present_forbidden if value not in observed]
        if missing:
            errors.append(
                f"{present_forbidden_field} must only include values present in "
                f"{observed_field}: {', '.join(missing)}"
            )


def _validate_optional_string_list_field(
    errors: list[str],
    payload: Mapping[str, Any],
    field: str,
) -> list[str] | None:
    values = payload.get(field)
    if values is None:
        return None
    if not isinstance(values, list):
        errors.append(f"{field} must be a list when present")
        return None
    invalid_values = [
        str(index)
        for index, value in enumerate(values, start=1)
        if not _is_non_empty_string(value)
    ]
    if invalid_values:
        errors.append(
            f"{field} entries must be non-empty strings: "
            f"{', '.join(invalid_values)}"
        )
    duplicate_values = _duplicate_non_empty_strings(values)
    if duplicate_values:
        errors.append(
            f"{field} entries must be unique: " f"{'; '.join(duplicate_values)}"
        )
    return [value for value in values if _is_non_empty_string(value)]


def _validate_summary_count_sum(
    errors: list[str],
    summary_counts: Mapping[str, Any],
    count_keys: list[str],
    total_key: str,
    message: str,
) -> None:
    keys = [*count_keys, total_key]
    if not all(_is_non_negative_int(summary_counts.get(key)) for key in keys):
        return
    if sum(int(summary_counts[key]) for key in count_keys) != int(
        summary_counts[total_key]
    ):
        errors.append(message)


def _validate_summary_count_lte(
    errors: list[str],
    summary_counts: Mapping[str, Any],
    count_key: str,
    total_key: str,
) -> None:
    if not (
        _is_non_negative_int(summary_counts.get(count_key))
        and _is_non_negative_int(summary_counts.get(total_key))
    ):
        return
    if int(summary_counts[count_key]) > int(summary_counts[total_key]):
        errors.append(f"summary_counts.{count_key} must be <= {total_key}")


def _validate_complete_summary_count(
    errors: list[str],
    summary_counts: Mapping[str, Any],
    count_key: str,
    expected: int,
) -> None:
    if count_key not in summary_counts or not _is_non_negative_int(
        summary_counts.get(count_key)
    ):
        errors.append(
            f"summary_counts.{count_key} must be {expected} when complete is true"
        )
        return
    if int(summary_counts[count_key]) != expected:
        errors.append(
            f"summary_counts.{count_key} must be {expected} when complete is true"
        )


def _validate_complete_summary_map_count(
    errors: list[str],
    summary_counts: Mapping[str, Any],
    map_key: str,
    count_key: str,
    expected: int,
) -> None:
    value = summary_counts.get(map_key)
    if not isinstance(value, Mapping):
        errors.append(
            f"summary_counts.{map_key}.{count_key} must be {expected} when "
            "complete is true and the matching requirement is enabled"
        )
        return
    if not _is_non_negative_int(value.get(count_key)):
        errors.append(
            f"summary_counts.{map_key}.{count_key} must be {expected} when "
            "complete is true and the matching requirement is enabled"
        )
        return
    if int(value[count_key]) != expected:
        errors.append(
            f"summary_counts.{map_key}.{count_key} must be {expected} when "
            "complete is true and the matching requirement is enabled"
        )


def _validate_summary_map_sum(
    errors: list[str],
    summary_counts: Mapping[str, Any],
    map_key: str,
    total_key: str,
) -> None:
    value = summary_counts.get(map_key)
    if not isinstance(value, Mapping) or not _is_non_negative_int(
        summary_counts.get(total_key)
    ):
        return
    if not all(_is_non_negative_int(item) for item in value.values()):
        return
    if sum(int(item) for item in value.values()) != int(summary_counts[total_key]):
        errors.append(f"summary_counts.{map_key} values must sum to {total_key}")


def _duplicate_non_empty_strings(values: Sequence[Any]) -> list[str]:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for value in values:
        if not _is_non_empty_string(value):
            continue
        text = str(value)
        if text in seen:
            duplicates.add(text)
        seen.add(text)
    return sorted(duplicates)


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_non_negative_number(value: Any) -> bool:
    return _is_number(value) and value >= 0


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
