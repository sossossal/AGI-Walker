import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW_CONTRACTS = ROOT / "agi_walker" / "core" / "api" / "workflow_contracts.py"

spec = importlib.util.spec_from_file_location("workflow_contracts", WORKFLOW_CONTRACTS)
workflow_contracts = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(workflow_contracts)

DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION = (
    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION
)
DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION = (
    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
)
DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_STATUSES = (
    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_STATUSES
)
DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS = (
    workflow_contracts.DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS
)
DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_REQUIRED_FIELDS = (
    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_REQUIRED_FIELDS
)
DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELDS = (
    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELDS
)
DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELDS = (
    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELDS
)
DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELDS = (
    workflow_contracts.DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELDS
)
DELIVERY_ACCEPTANCE_GATE_SCHEMA = workflow_contracts.DELIVERY_ACCEPTANCE_GATE_SCHEMA
WORKFLOW_CONTRACT_VERSION = workflow_contracts.WORKFLOW_CONTRACT_VERSION
build_delivery_acceptance_requirements = (
    workflow_contracts.build_delivery_acceptance_requirements
)
build_workflow_step_artifact = workflow_contracts.build_workflow_step_artifact
validate_export_result = workflow_contracts.validate_export_result
validate_optimization_result = workflow_contracts.validate_optimization_result
validate_part_spec = workflow_contracts.validate_part_spec
validate_robot_config = workflow_contracts.validate_robot_config
validate_workflow_definition = workflow_contracts.validate_workflow_definition
validate_workflow_step_artifact = workflow_contracts.validate_workflow_step_artifact
validate_delivery_acceptance_gate = workflow_contracts.validate_delivery_acceptance_gate
validate_delivery_acceptance_validation_summary = (
    workflow_contracts.validate_delivery_acceptance_validation_summary
)


def test_delivery_acceptance_requirements_contract_is_explicit() -> None:
    requirements = build_delivery_acceptance_requirements(
        run_godot_smoke=True,
        full_mechanical_restoration_gate=True,
        node_tree_fixed_lock_match=True,
    )

    assert DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION == "delivery_acceptance_gate.v1"
    assert requirements == {
        **DELIVERY_ACCEPTANCE_REQUIREMENT_DEFAULTS,
        "run_godot_smoke": True,
        "full_mechanical_restoration_gate": True,
        "node_tree_fixed_lock_match": True,
    }


def test_delivery_acceptance_gate_schema_exposes_machine_readable_enums() -> None:
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["contract_version"] == (
        DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["validation_summary_version"] == (
        DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_required_fields"
    ] == sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_REQUIRED_FIELDS)
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_required_fields"
    ] == [
        "error_count",
        "errors",
        "expanded_inputs_count",
        "inputs_count",
        "skipped_count",
        "status",
        "success_count",
        "summary_version",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_status_values"
    ] == sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_STATUSES)
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_metadata_fields"
    ] == sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_METADATA_FIELDS)
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_metadata_fields"
    ] == [
        "node_tree_manifest_sidecar_complete_count",
        "node_tree_manifest_sidecar_count",
        "node_tree_manifest_sidecar_incomplete_count",
        "node_tree_manifest_sidecar_invalid_count",
        "node_tree_manifest_sidecar_joint_path_count",
        "node_tree_manifest_sidecar_joints_planned_count",
        "node_tree_manifest_sidecar_part_path_count",
        "node_tree_manifest_sidecar_parts_planned_count",
        "node_tree_manifest_sidecar_path_incomplete_count",
        "node_tree_manifest_sidecar_path_map_mismatch_count",
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts",
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
    ]
    assert sorted(
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_metadata_field_types"
        ]
    ) == DELIVERY_ACCEPTANCE_GATE_SCHEMA["validation_summary_metadata_fields"]
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_metadata_field_types"
        ]["summary_versions"]
        == "list[str]"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_metadata_field_types"
        ]["summary_versions_count"]
        == "integer"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_metadata_field_types"
        ]["node_tree_manifest_sidecar_path_map_mismatch_kind_counts"]
        == "object"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_metadata_field_types"
        ]["summary_versions_truncated"]
        == "boolean"
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_constraint_fields"
    ] == sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_CONSTRAINT_FIELDS)
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_constraint_fields"
    ] == [
        "allowed_summary_versions",
        "allowed_validation_summary_statuses",
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
        "expected_summary_versions",
        "expected_summary_versions_count",
        "expected_validation_summary_statuses",
        "expected_validation_summary_statuses_count",
        "forbidden_summary_versions",
        "forbidden_validation_summary_statuses",
        "missing_expected_summary_versions",
        "missing_expected_validation_summary_statuses",
        "present_forbidden_summary_versions",
        "present_forbidden_validation_summary_statuses",
        "unexpected_summary_versions",
        "unexpected_validation_summary_statuses",
    ]
    assert sorted(
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]
    ) == DELIVERY_ACCEPTANCE_GATE_SCHEMA["validation_summary_constraint_fields"]
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]["expected_summary_versions_count"]
        == "integer|null"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]["expected_node_tree_manifest_sidecar_path_map_mismatch_count"]
        == "integer|null"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]["expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts"]
        == "object"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]["expected_node_tree_manifest_sidecar_valid_count"]
        == "integer|null"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]["expected_node_tree_manifest_sidecar_validation_error_count"]
        == "integer|null"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]["expected_node_tree_manifest_sidecar_parts_planned_count"]
        == "integer|null"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_constraint_field_types"
        ]["missing_expected_validation_summary_statuses"]
        == "list[str]"
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_policy_fields"
    ] == sorted(DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_POLICY_FIELDS)
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "validation_summary_policy_fields"
    ] == [
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
    ]
    assert sorted(
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_policy_field_types"
        ]
    ) == DELIVERY_ACCEPTANCE_GATE_SCHEMA["validation_summary_policy_fields"]
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "validation_summary_policy_field_types"
        ]["fail_on_node_tree_manifest_sidecar_path_map_mismatch"]
        == "boolean"
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["source_values"] == [
        "dynamic_godot_report_cli",
        "web_godot_delivery",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["verification_scope_values"] == [
        "godot_load",
        "godot_smoke_motion",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["acceptance_profile_values"] == [
        "custom",
        "full_mechanical_restoration",
        "web_godot_load",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["level_values"] == [
        "godot_load_verified",
        "godot_verified",
        "incomplete",
        "static_only",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["source_scope_pairs"] == {
        "dynamic_godot_report_cli": "godot_smoke_motion",
        "web_godot_delivery": "godot_load",
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["source_profile_values"] == {
        "dynamic_godot_report_cli": ["custom", "full_mechanical_restoration"],
        "web_godot_delivery": ["web_godot_load"],
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["scope_level_values"] == {
        "godot_load": ["godot_load_verified", "incomplete"],
        "godot_smoke_motion": ["godot_verified", "incomplete", "static_only"],
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["complete_level_by_scope"] == {
        "godot_load": "godot_load_verified",
        "godot_smoke_motion": "godot_verified",
    }
    assert "reason_codes" in DELIVERY_ACCEPTANCE_GATE_SCHEMA["required"]
    assert "godot_load" in DELIVERY_ACCEPTANCE_GATE_SCHEMA["requirement_fields"]
    assert "run_godot_smoke" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["enabled_requirement_values_by_source"][
            "dynamic_godot_report_cli"
        ]
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["enabled_requirement_values_by_source"][
        "web_godot_delivery"
    ] == [
        "godot_load",
        "joint_parameter_readback",
        "mechanical_restoration_complete",
        "node_tree_fixed_lock_match",
    ]
    assert "missing_godot_smoke" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["reason_code_values"]
    )
    assert "godot_delivery_failed" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["reason_code_values"]
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["reason_detail_fields"] == [
        "code",
        "count",
        "inputs",
        "inputs_count",
        "inputs_truncated",
        "message",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["reason_code_values_by_source"][
        "dynamic_godot_report_cli"
    ] == [
        "control_readback_missing",
        "incomplete_delivery",
        "incomplete_joint_parameters",
        "invalid_godot_smoke_report",
        "missing_dynamic_generation",
        "missing_full_mechanical_restoration_gate",
        "missing_godot_smoke",
        "missing_godot_smoke_report",
        "missing_static_node_tree_manifest_output",
        "no_inputs",
        "node_tree_fixed_lock_mismatch",
        "robot_errors",
        "smoke_failure_reasons",
        "static_node_tree_incomplete",
        "static_only",
        "unknown_delivery_provenance",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["reason_code_values_by_source"][
        "web_godot_delivery"
    ] == [
        "fixed_lock_mismatch",
        "godot_delivery_failed",
        "incomplete_delivery",
        "incomplete_joint_parameters",
        "incomplete_joints",
        "incomplete_parts",
        "incomplete_restoration",
        "missing_godot_assembly_summary",
    ]
    assert "control_readback_missing_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "static_topology_complete_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "static_topology_cycle_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "static_node_tree_manifest_output_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "static_node_tree_manifest_error_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "static_node_tree_manifest_path_map_mismatch_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "mechanical_behavior_evidence_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "mechanical_behavior_complete_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "mechanical_behavior_residual_risk_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert "mechanical_behavior_step_trace_artifact_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_fields"]
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_map_fields"] == [
        "mechanical_gate_check_counts",
        "node_tree_gate_check_counts",
        "static_node_tree_manifest_path_map_mismatch_kind_counts",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_map_key_values"] == {
        "mechanical_gate_check_counts": [
            "control_parameter_readback",
            "full_node_tree_restoration",
            "joint_parameter_readback",
            "mechanical_restoration",
        ],
        "node_tree_gate_check_counts": [
            "class_mismatch",
            "fixed_lock_mismatch",
            "incomplete_node_tree",
            "missing_parameters",
            "physical_mismatch",
            "transform_mismatch",
        ],
        "static_node_tree_manifest_path_map_mismatch_kind_counts": [
            "duplicate",
            "missing",
            "root_mismatch",
            "unexpected",
            "value_mismatch",
        ],
    }
    assert "delivery_static_only_count" in DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "summary_value_paths"
    ]
    assert "node_tree_gate_check_counts.fixed_lock_mismatch" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths"]
    )
    assert "mechanical_gate_check_counts.mechanical_restoration" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths"]
    )
    assert "control_readback_missing_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths_by_source"][
            "dynamic_godot_report_cli"
        ]
    )
    assert "mechanical_behavior_evidence_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths_by_source"][
            "dynamic_godot_report_cli"
        ]
    )
    assert "control_readback_missing_count" not in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths_by_source"][
            "web_godot_delivery"
        ]
    )
    assert "mechanical_behavior_evidence_count" not in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths_by_source"][
            "web_godot_delivery"
        ]
    )
    assert "node_tree_gate_check_counts.fixed_lock_mismatch" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths_by_source"][
            "web_godot_delivery"
        ]
    )
    assert "mechanical_gate_check_counts.mechanical_restoration" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_paths_by_source"][
            "web_godot_delivery"
        ]
    )
    assert "expected_summary_value_sources" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_fields"]
    )
    assert "summary_value_source_matched_sources" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_fields"]
    )
    assert "summary_value_source_excluded_sources" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_fields"]
    )
    assert "unexpected_summary_value_source_matched_sources" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_fields"]
    )
    assert "present_forbidden_summary_value_source_excluded_sources" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_fields"]
    )
    assert sorted(
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_field_types"]
    ) == DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_fields"]
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_field_types"][
            "actual_expected_summary_values"
        ]
        == "object"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_field_types"][
            "summary_value_source_matched_count"
        ]
        == "integer|null"
    )
    assert (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_value_source_filter_field_types"][
            "summary_value_source_matched_sources"
        ]
        == "list[str]"
    )
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["requirement_summary_map_counts"] == {
        "mechanical_gate_check_counts": {
            "control_parameter_readback": "control_parameter_readback",
            "full_node_tree_restoration": "full_node_tree_restoration",
            "joint_parameter_readback": "joint_parameter_readback",
            "mechanical_restoration_complete": "mechanical_restoration",
        },
        "node_tree_gate_check_counts": {
            "node_tree_class_match": "class_mismatch",
            "node_tree_complete": "incomplete_node_tree",
            "node_tree_fixed_lock_match": "fixed_lock_mismatch",
            "node_tree_parameters_applied": "missing_parameters",
            "node_tree_physical_match": "physical_mismatch",
            "node_tree_transform_match": "transform_mismatch",
        },
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "requirement_summary_map_counts_by_source_scope"
    ] == {
        "dynamic_godot_report_cli": {
            "godot_smoke_motion": DELIVERY_ACCEPTANCE_GATE_SCHEMA[
                "requirement_summary_map_counts"
            ]
        },
        "web_godot_delivery": {"godot_load": {}},
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["complete_summary_counts"] == {
        "equal_inputs": [
            "delivery_complete_count",
            "delivery_dynamic_generation_count",
            "delivery_godot_verified_count",
            "static_node_tree_complete_count",
            "static_node_tree_endpoint_paths_complete_count",
            "static_node_tree_manifest_count",
            "static_node_tree_manifest_valid_count",
            "static_node_tree_parameters_complete_count",
            "static_topology_complete_count",
            "success_count",
        ],
        "zero": [
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
            "static_node_tree_endpoint_paths_incomplete_count",
            "static_node_tree_incomplete_count",
            "static_node_tree_manifest_error_count",
            "static_node_tree_manifest_invalid_count",
            "static_node_tree_manifest_path_map_mismatch_count",
            "static_node_tree_missing_endpoint_connections_count",
            "static_node_tree_missing_endpoint_parts_count",
            "static_node_tree_parameters_incomplete_count",
            "static_topology_cycle_count",
            "static_topology_disconnected_parts_count",
            "static_topology_duplicate_child_endpoint_count",
            "static_topology_incomplete_count",
            "static_topology_unreachable_parts_count",
        ],
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["complete_summary_counts_by_source"] == {
        "dynamic_godot_report_cli": DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "complete_summary_counts"
        ],
        "web_godot_delivery": {
            "equal_inputs": [
                "delivery_complete_count",
                "delivery_dynamic_generation_count",
                "delivery_godot_verified_count",
                "static_node_tree_complete_count",
                "static_node_tree_endpoint_paths_complete_count",
                "static_node_tree_manifest_count",
                "static_node_tree_manifest_valid_count",
                "static_node_tree_parameters_complete_count",
                "success_count",
            ],
            "zero": [
                "delivery_incomplete_count",
                "delivery_parameters_incomplete_count",
                "delivery_static_only_count",
                "delivery_unverified_count",
                "error_count",
                "failure_reasons_count",
                "fixed_lock_mismatch_count",
                "node_tree_fixed_lock_mismatch_count",
                "node_tree_fixed_locks_incomplete_count",
                "static_node_tree_endpoint_paths_incomplete_count",
                "static_node_tree_incomplete_count",
                "static_node_tree_manifest_error_count",
                "static_node_tree_manifest_invalid_count",
                "static_node_tree_manifest_path_map_mismatch_count",
                "static_node_tree_parameters_incomplete_count",
            ],
        },
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "complete_summary_counts_by_source_scope"
    ] == {
        "dynamic_godot_report_cli": {
            "godot_smoke_motion": {
                "equal_inputs": [
                    "delivery_complete_count",
                    "delivery_dynamic_generation_count",
                    "delivery_godot_verified_count",
                    "live_smoke_count",
                    "smoke_report_written_count",
                    "static_node_tree_complete_count",
                    "static_node_tree_endpoint_paths_complete_count",
                    "static_node_tree_manifest_count",
                    "static_node_tree_manifest_valid_count",
                    "static_node_tree_parameters_complete_count",
                    "static_topology_complete_count",
                    "success_count",
                ],
                "zero": [
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
                    "smoke_report_missing_count",
                    "smoke_report_read_error_count",
                    "static_node_tree_endpoint_paths_incomplete_count",
                    "static_node_tree_incomplete_count",
                    "static_node_tree_manifest_error_count",
                    "static_node_tree_manifest_invalid_count",
                    "static_node_tree_manifest_path_map_mismatch_count",
                    "static_node_tree_missing_endpoint_connections_count",
                    "static_node_tree_missing_endpoint_parts_count",
                    "static_node_tree_parameters_incomplete_count",
                    "static_topology_cycle_count",
                    "static_topology_disconnected_parts_count",
                    "static_topology_duplicate_child_endpoint_count",
                    "static_topology_incomplete_count",
                    "static_topology_unreachable_parts_count",
                ],
            }
        },
        "web_godot_delivery": {
            "godot_load": {
                "equal_inputs": [
                    "delivery_complete_count",
                    "delivery_dynamic_generation_count",
                    "delivery_godot_verified_count",
                    "static_node_tree_complete_count",
                    "static_node_tree_endpoint_paths_complete_count",
                    "static_node_tree_manifest_count",
                    "static_node_tree_manifest_valid_count",
                    "static_node_tree_parameters_complete_count",
                    "success_count",
                ],
                "zero": [
                    "delivery_incomplete_count",
                    "delivery_parameters_incomplete_count",
                    "delivery_static_only_count",
                    "delivery_unverified_count",
                    "error_count",
                    "failure_reasons_count",
                    "fixed_lock_mismatch_count",
                    "node_tree_fixed_lock_mismatch_count",
                    "node_tree_fixed_locks_incomplete_count",
                    "static_node_tree_endpoint_paths_incomplete_count",
                    "static_node_tree_incomplete_count",
                    "static_node_tree_manifest_error_count",
                    "static_node_tree_manifest_invalid_count",
                    "static_node_tree_manifest_path_map_mismatch_count",
                    "static_node_tree_parameters_incomplete_count",
                ],
            }
        },
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "complete_required_summary_fields_by_source_scope"
    ] == {
        source: {
            scope: sorted(set(counts["equal_inputs"]) | set(counts["zero"]))
            for scope, counts in scopes.items()
        }
        for source, scopes in DELIVERY_ACCEPTANCE_GATE_SCHEMA[
            "complete_summary_counts_by_source_scope"
        ].items()
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA[
        "complete_required_summary_fields_source_scopes"
    ] == [
        "dynamic_godot_report_cli/godot_smoke_motion",
        "web_godot_delivery/godot_load",
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["scope_complete_summary_counts"] == {
        "godot_smoke_motion": {
            "equal_inputs": [
                "live_smoke_count",
                "smoke_report_written_count",
            ],
            "zero": [
                "smoke_report_missing_count",
                "smoke_report_read_error_count",
            ],
        }
    }
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_sum_rules"] == [
        {
            "name": "result_counts",
            "counts": ["success_count", "error_count"],
            "total": "inputs_count",
            "message": "summary_counts.success_count + error_count must equal inputs_count",
        },
        {
            "name": "delivery_provenance_counts",
            "counts": [
                "delivery_godot_verified_count",
                "delivery_static_only_count",
                "delivery_unverified_count",
            ],
            "total": "inputs_count",
            "message": "summary_counts delivery provenance counts must equal inputs_count",
        },
        {
            "name": "delivery_completion_counts",
            "counts": ["delivery_complete_count", "delivery_incomplete_count"],
            "total": "inputs_count",
            "message": (
                "summary_counts.delivery_complete_count + delivery_incomplete_count "
                "must equal inputs_count"
            ),
        },
        {
            "name": "static_node_tree_manifest_validity_counts",
            "counts": [
                "static_node_tree_manifest_valid_count",
                "static_node_tree_manifest_invalid_count",
            ],
            "total": "static_node_tree_manifest_count",
            "message": (
                "summary_counts.static_node_tree_manifest_valid_count + "
                "static_node_tree_manifest_invalid_count must equal "
                "static_node_tree_manifest_count"
            ),
        },
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_count_lte_rules"] == [
        {"count": "live_smoke_count", "max": "inputs_count"}
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_map_sum_rules"] == [
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
    ]
    assert DELIVERY_ACCEPTANCE_GATE_SCHEMA["complete_result_requirements"] == {
        "passed": True,
        "exit_code": 0,
        "empty_lists": ["reasons", "reason_codes", "reason_details"],
    }
    assert "control_readback_missing_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_fields_by_source"][
            "dynamic_godot_report_cli"
        ]
    )
    assert "mechanical_behavior_evidence_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_fields_by_source"][
            "dynamic_godot_report_cli"
        ]
    )
    assert "control_readback_missing_count" not in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_fields_by_source"][
            "web_godot_delivery"
        ]
    )
    assert "mechanical_behavior_evidence_count" not in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_fields_by_source"][
            "web_godot_delivery"
        ]
    )
    assert "fixed_lock_mismatch_count" in (
        DELIVERY_ACCEPTANCE_GATE_SCHEMA["summary_fields_by_source"][
            "web_godot_delivery"
        ]
    )


def test_delivery_acceptance_requirements_reject_unknown_fields() -> None:
    try:
        build_delivery_acceptance_requirements(unknown_gate=True)
    except ValueError as exc:
        assert "unknown delivery acceptance requirement" in str(exc)
    else:
        raise AssertionError("unknown requirement was accepted")


def test_delivery_acceptance_gate_contract_rejects_source_specific_enabled_requirements() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "web_godot_delivery",
        "verification_scope": "godot_load",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "web_godot_load",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_load=True,
            run_godot_smoke=True,
        ),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["Godot delivery failed before load acceptance."],
        "reason_codes": ["godot_delivery_failed"],
        "reason_details": [
            {
                "code": "godot_delivery_failed",
                "count": 1,
                "message": "Godot delivery failed before load acceptance.",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "acceptance_requirements contains enabled fields not allowed for source "
        "'web_godot_delivery': run_godot_smoke" in error
        for error in errors
    )


def test_delivery_acceptance_validation_summary_contract_accepts_canonical_payload() -> None:
    summary = {
        "summary_version": DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION,
        "status": "success",
        "expanded_inputs_count": 2,
        "inputs_count": 2,
        "success_count": 1,
        "skipped_count": 1,
        "error_count": 0,
        "errors": [],
        "summary_versions": [DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION],
        "summary_versions_count": 1,
        "summary_versions_truncated": False,
        "validation_summary_statuses": ["success"],
        "validation_summary_statuses_count": 1,
        "validation_summary_statuses_truncated": False,
        "fail_on_control_readback_missing": False,
        "fail_on_full_mechanical_gate_false": False,
        "fail_on_full_mechanical_gate_unknown": False,
        "fail_on_invalid_node_tree_manifest_sidecar": False,
        "fail_on_node_tree_manifest_sidecar_incomplete": False,
        "fail_on_node_tree_manifest_sidecar_path_incomplete": False,
        "fail_on_node_tree_manifest_sidecar_path_map_mismatch": False,
        "fail_on_node_tree_manifest_sidecar_validation_error": False,
        "fail_on_smoke_report_missing": False,
        "fail_on_smoke_report_read_error": False,
        "node_tree_manifest_sidecar_count": 1,
        "node_tree_manifest_sidecar_complete_count": 1,
        "node_tree_manifest_sidecar_incomplete_count": 0,
        "node_tree_manifest_sidecar_valid_count": 1,
        "node_tree_manifest_sidecar_invalid_count": 0,
        "node_tree_manifest_sidecar_validation_error_count": 0,
        "node_tree_manifest_sidecar_path_incomplete_count": 0,
        "node_tree_manifest_sidecar_path_map_mismatch_count": 2,
        "node_tree_manifest_sidecar_path_map_mismatch_kind_counts": {
            "duplicate": 1,
            "root_mismatch": 1,
        },
        "node_tree_manifest_sidecar_parts_planned_count": 2,
        "node_tree_manifest_sidecar_joints_planned_count": 1,
        "node_tree_manifest_sidecar_part_path_count": 2,
        "node_tree_manifest_sidecar_joint_path_count": 1,
        "node_tree_manifest_sidecars": [
            {
                "input": "artifact.node_tree_manifest.json",
                "manifest_version": "godot_node_tree_manifest.v1",
                "parts_count": 2,
                "joints_count": 1,
                "part_node_path_count": 2,
                "joint_node_path_count": 1,
                "path_maps_complete": True,
                "node_tree_manifest_path_map_mismatch_count": 2,
                "node_tree_manifest_path_map_mismatches": [
                    {
                        "map": "part_node_paths",
                        "key": "base",
                        "field": "part_id",
                        "kind": "duplicate",
                    },
                    {
                        "map": "part_nodes",
                        "key": "base",
                        "field": "body_node",
                        "kind": "root_mismatch",
                        "expected": "robot/base",
                        "actual": "wrong/base",
                    },
                ],
                "node_tree_manifest_path_map_mismatch_kind_counts": {
                    "duplicate": 1,
                    "root_mismatch": 1,
                },
                "node_tree_manifest_validation_error_count": 0,
                "node_tree_manifest_validation_errors": [],
                "complete": True,
                "node_tree_manifest_valid": True,
            }
        ],
        "node_tree_manifest_sidecars_truncated": False,
        "expected_node_tree_manifest_sidecar_count": 1,
        "expected_node_tree_manifest_sidecar_complete_count": 1,
        "expected_node_tree_manifest_sidecar_incomplete_count": 0,
        "expected_node_tree_manifest_sidecar_valid_count": 1,
        "expected_node_tree_manifest_sidecar_invalid_count": 0,
        "expected_node_tree_manifest_sidecar_validation_error_count": 0,
        "expected_node_tree_manifest_sidecar_path_incomplete_count": 0,
        "expected_node_tree_manifest_sidecar_path_map_mismatch_count": 2,
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts": {
            "duplicate": 1,
            "root_mismatch": 1,
        },
        "expected_node_tree_manifest_sidecar_parts_planned_count": 2,
        "expected_node_tree_manifest_sidecar_joints_planned_count": 1,
        "expected_node_tree_manifest_sidecar_part_path_count": 2,
        "expected_node_tree_manifest_sidecar_joint_path_count": 1,
        "expected_summary_versions_count": None,
        "expected_validation_summary_statuses_count": None,
        "expected_summary_versions": [
            DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
        ],
        "missing_expected_summary_versions": [],
        "expected_validation_summary_statuses": ["success"],
        "missing_expected_validation_summary_statuses": [],
        "allowed_summary_versions": [
            DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
        ],
        "unexpected_summary_versions": [],
        "allowed_validation_summary_statuses": ["success"],
        "unexpected_validation_summary_statuses": [],
        "forbidden_summary_versions": ["old"],
        "present_forbidden_summary_versions": [],
        "forbidden_validation_summary_statuses": ["error"],
        "present_forbidden_validation_summary_statuses": [],
    }

    assert validate_delivery_acceptance_validation_summary(summary) == []


def test_delivery_acceptance_validation_summary_contract_rejects_invalid_shape() -> None:
    errors = validate_delivery_acceptance_validation_summary(
        {
            "summary_version": "old",
            "status": "partial",
            "expanded_inputs_count": 1,
            "inputs_count": 2,
            "success_count": 1,
            "skipped_count": 0,
            "error_count": 0,
            "errors": ["bad", "bad", ""],
            "summary_versions": ["old", "old", ""],
            "summary_versions_count": 1,
            "summary_versions_truncated": False,
            "validation_summary_statuses": ["success"],
            "validation_summary_statuses_count": "1",
            "validation_summary_statuses_truncated": "false",
            "fail_on_node_tree_manifest_sidecar_validation_error": "true",
            "fail_on_node_tree_manifest_sidecar_path_map_mismatch": "true",
            "node_tree_manifest_sidecar_count": 1,
            "node_tree_manifest_sidecar_complete_count": 2,
            "node_tree_manifest_sidecar_incomplete_count": 1,
            "node_tree_manifest_sidecar_valid_count": 2,
            "node_tree_manifest_sidecar_invalid_count": 1,
            "node_tree_manifest_sidecar_validation_error_count": -1,
            "node_tree_manifest_sidecar_path_incomplete_count": 2,
            "node_tree_manifest_sidecar_path_map_mismatch_count": -1,
            "node_tree_manifest_sidecar_parts_planned_count": -1,
            "node_tree_manifest_sidecar_joints_planned_count": "1",
            "node_tree_manifest_sidecar_part_path_count": -1,
            "node_tree_manifest_sidecar_joint_path_count": "1",
            "node_tree_manifest_sidecars": [
                {
                    "parts_count": 1,
                    "joints_count": 1,
                    "part_node_path_count": 2,
                    "joint_node_path_count": 2,
                    "path_maps_complete": True,
                    "node_tree_manifest_validation_error_count": 2,
                    "node_tree_manifest_validation_errors": ["bad"],
                    "node_tree_manifest_path_map_mismatch_count": "1",
                    "node_tree_manifest_path_map_mismatches": [
                        {
                            "map": "",
                            "key": "",
                            "kind": "value_mismatch",
                            "field": "",
                            "expected": "",
                            "actual": 1,
                        },
                        "bad",
                    ],
                },
                "bad",
            ],
            "node_tree_manifest_sidecars_truncated": False,
            "expected_node_tree_manifest_sidecar_count": -1,
            "expected_node_tree_manifest_sidecar_valid_count": "1",
            "expected_node_tree_manifest_sidecar_validation_error_count": -1,
            "expected_node_tree_manifest_sidecar_path_map_mismatch_count": -1,
            "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts": {
                "": 1,
                "root_mismatch": -1,
            },
            "expected_node_tree_manifest_sidecar_parts_planned_count": "2",
            "expected_node_tree_manifest_sidecar_joint_path_count": -1,
            "expected_summary_versions_count": "1",
            "expected_validation_summary_statuses_count": -1,
            "expected_summary_versions": ["old", "future"],
            "missing_expected_summary_versions": [
                "old",
                "unknown",
            ],
            "allowed_summary_versions": [
                DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
            ],
            "unexpected_summary_versions": [
                DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION
            ],
            "forbidden_summary_versions": ["old"],
            "present_forbidden_summary_versions": ["future"],
            "expected_validation_summary_statuses": ["error", "error"],
            "missing_expected_validation_summary_statuses": ["success"],
            "allowed_validation_summary_statuses": "success",
            "unexpected_validation_summary_statuses": ["error"],
            "forbidden_validation_summary_statuses": ["error"],
            "present_forbidden_validation_summary_statuses": ["success"],
        }
    )

    assert any("summary_version must be" in error for error in errors)
    assert any("status must be one of" in error for error in errors)
    assert "expanded_inputs_count must be >= inputs_count" in errors
    assert (
        "success_count + skipped_count + error_count must equal inputs_count"
        in errors
    )
    assert any("errors entries must be non-empty strings" in error for error in errors)
    assert any("errors entries must be unique" in error for error in errors)
    assert any(
        "summary_versions entries must be non-empty strings" in error
        for error in errors
    )
    assert any("summary_versions entries must be unique" in error for error in errors)
    assert "summary_versions_count must be >= summary_versions preview length" in errors
    assert "summary_versions_count must equal summary_versions preview length" in errors
    assert any(
        "validation_summary_statuses_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "validation_summary_statuses_truncated must be a boolean" in error
        for error in errors
    )
    assert any(
        "fail_on_node_tree_manifest_sidecar_path_map_mismatch must be a boolean when present"
        in error
        for error in errors
    )
    assert any(
        "fail_on_node_tree_manifest_sidecar_validation_error must be a boolean when present"
        in error
        for error in errors
    )
    assert "validation_summary_statuses must include 'partial'" in errors
    assert any(
        "node_tree_manifest_sidecar_parts_planned_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_joints_planned_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_part_path_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_joint_path_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_path_map_mismatch_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_validation_error_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars[1].node_tree_manifest_validation_error_count "
        "must equal node_tree_manifest_validation_errors length" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars entries must be objects: 2" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars[1].part_node_path_count must be <= parts_count"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars[1].joint_node_path_count must be <= joints_count"
        in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars[1].node_tree_manifest_path_map_mismatch_count "
        "must be a non-negative integer" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars[1].node_tree_manifest_path_map_mismatches "
        "entries must be objects: 2" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars[1].node_tree_manifest_path_map_mismatches[1].map "
        "must be a non-empty string" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecars[1].node_tree_manifest_path_map_mismatches[1].actual "
        "must be a string when present" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_complete_count + "
        "node_tree_manifest_sidecar_incomplete_count must be <= "
        "node_tree_manifest_sidecar_count" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_valid_count + "
        "node_tree_manifest_sidecar_invalid_count must be <= "
        "node_tree_manifest_sidecar_count" in error
        for error in errors
    )
    assert any(
        "node_tree_manifest_sidecar_path_incomplete_count must be <= "
        "node_tree_manifest_sidecar_count" in error
        for error in errors
    )
    assert any(
        "expected_summary_versions_count must be a non-negative integer or null"
        in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_path_map_mismatch_count must be "
        "a non-negative integer or null" in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts keys "
        "must be non-empty strings" in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_path_map_mismatch_kind_counts values "
        "must be non-negative integers: root_mismatch" in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_count must be "
        "a non-negative integer or null" in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_valid_count must be "
        "a non-negative integer or null" in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_validation_error_count must be "
        "a non-negative integer or null" in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_parts_planned_count must be "
        "a non-negative integer or null" in error
        for error in errors
    )
    assert any(
        "expected_node_tree_manifest_sidecar_joint_path_count must be "
        "a non-negative integer or null" in error
        for error in errors
    )
    assert any(
        "expected_validation_summary_statuses_count must be a non-negative "
        "integer or null" in error
        for error in errors
    )
    assert any(
        "missing_expected_summary_versions must be a subset of "
        "expected_summary_versions" in error
        for error in errors
    )
    assert any(
        "missing_expected_summary_versions must not include values present in "
        "summary_versions" in error
        for error in errors
    )
    assert any(
        "unexpected_summary_versions must not include values present in "
        "allowed_summary_versions" in error
        for error in errors
    )
    assert any(
        "present_forbidden_summary_versions must be a subset of "
        "forbidden_summary_versions" in error
        for error in errors
    )
    assert any(
        "present_forbidden_summary_versions must only include values present in "
        "summary_versions" in error
        for error in errors
    )
    assert any(
        "expected_validation_summary_statuses entries must be unique" in error
        for error in errors
    )
    assert "allowed_validation_summary_statuses must be a list when present" in errors
    assert any(
        "present_forbidden_validation_summary_statuses must be a subset of "
        "forbidden_validation_summary_statuses" in error
        for error in errors
    )


def test_delivery_acceptance_validation_summary_contract_rejects_sidecar_path_overcounts() -> None:
    summary = {
        "summary_version": DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION,
        "status": "success",
        "expanded_inputs_count": 1,
        "inputs_count": 1,
        "success_count": 0,
        "skipped_count": 1,
        "error_count": 0,
        "errors": [],
        "node_tree_manifest_sidecar_count": 1,
        "node_tree_manifest_sidecar_complete_count": 1,
        "node_tree_manifest_sidecar_incomplete_count": 0,
        "node_tree_manifest_sidecar_valid_count": 1,
        "node_tree_manifest_sidecar_invalid_count": 0,
        "node_tree_manifest_sidecar_validation_error_count": 0,
        "node_tree_manifest_sidecar_path_map_mismatch_count": 0,
        "node_tree_manifest_sidecar_parts_planned_count": 1,
        "node_tree_manifest_sidecar_joints_planned_count": 1,
        "node_tree_manifest_sidecar_part_path_count": 2,
        "node_tree_manifest_sidecar_joint_path_count": 2,
        "node_tree_manifest_sidecars": [
            {
                "input": "artifact.node_tree_manifest.json",
                "manifest_version": "godot_node_tree_manifest.v1",
                "parts_count": 1,
                "joints_count": 1,
                "part_node_path_count": 2,
                "joint_node_path_count": 2,
                "path_maps_complete": True,
                "node_tree_manifest_validation_error_count": 1,
                "node_tree_manifest_validation_errors": [],
                "node_tree_manifest_path_map_mismatch_count": 1,
                "node_tree_manifest_path_map_mismatches": [],
                "complete": True,
            }
        ],
        "node_tree_manifest_sidecars_truncated": False,
    }

    errors = validate_delivery_acceptance_validation_summary(summary)

    assert (
        "node_tree_manifest_sidecar_part_path_count must be <= "
        "node_tree_manifest_sidecar_parts_planned_count"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_joint_path_count must be <= "
        "node_tree_manifest_sidecar_joints_planned_count"
    ) in errors
    assert (
        "node_tree_manifest_sidecars[1].part_node_path_count must be <= parts_count"
        in errors
    )
    assert (
        "node_tree_manifest_sidecars[1].joint_node_path_count must be <= joints_count"
        in errors
    )
    assert (
        "node_tree_manifest_sidecars[1].node_tree_manifest_path_map_mismatch_count "
        "must equal node_tree_manifest_path_map_mismatches preview length when "
        "sidecar preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_path_map_mismatch_count must equal "
        "node_tree_manifest_sidecars node_tree_manifest_path_map_mismatch_count "
        "sum when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecars[1].node_tree_manifest_validation_error_count "
        "must equal node_tree_manifest_validation_errors length"
    ) in errors


def test_delivery_acceptance_validation_summary_contract_rejects_untruncated_sidecar_sum_mismatch() -> None:
    summary = {
        "summary_version": DELIVERY_ACCEPTANCE_GATE_VALIDATION_SUMMARY_VERSION,
        "status": "success",
        "expanded_inputs_count": 1,
        "inputs_count": 1,
        "success_count": 0,
        "skipped_count": 1,
        "error_count": 0,
        "errors": [],
        "node_tree_manifest_sidecar_count": 1,
        "node_tree_manifest_sidecar_complete_count": 0,
        "node_tree_manifest_sidecar_incomplete_count": 1,
        "node_tree_manifest_sidecar_valid_count": 0,
        "node_tree_manifest_sidecar_invalid_count": 1,
        "node_tree_manifest_sidecar_validation_error_count": 1,
        "node_tree_manifest_sidecar_path_incomplete_count": 0,
        "node_tree_manifest_sidecar_path_map_mismatch_count": 1,
        "node_tree_manifest_sidecar_parts_planned_count": 1,
        "node_tree_manifest_sidecar_joints_planned_count": 0,
        "node_tree_manifest_sidecar_part_path_count": 1,
        "node_tree_manifest_sidecar_joint_path_count": 0,
        "node_tree_manifest_sidecars": [
            {
                "input": "artifact.node_tree_manifest.json",
                "manifest_version": "godot_node_tree_manifest.v1",
                "parts_count": 2,
                "joints_count": 1,
                "part_node_path_count": 2,
                "joint_node_path_count": 1,
                "path_maps_complete": False,
                "complete": True,
                "node_tree_manifest_valid": True,
                "node_tree_manifest_validation_error_count": 0,
                "node_tree_manifest_validation_errors": [],
                "node_tree_manifest_path_map_mismatch_count": 0,
            }
        ],
        "node_tree_manifest_sidecars_truncated": False,
    }

    errors = validate_delivery_acceptance_validation_summary(summary)

    assert (
        "node_tree_manifest_sidecar_parts_planned_count must equal "
        "node_tree_manifest_sidecars parts_count sum when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_joints_planned_count must equal "
        "node_tree_manifest_sidecars joints_count sum when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_part_path_count must equal "
        "node_tree_manifest_sidecars part_node_path_count sum when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_joint_path_count must equal "
        "node_tree_manifest_sidecars joint_node_path_count sum when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_complete_count must equal "
        "node_tree_manifest_sidecars complete=true count when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_incomplete_count must equal "
        "node_tree_manifest_sidecars complete=false count when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_valid_count must equal "
        "node_tree_manifest_sidecars node_tree_manifest_valid=true count when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_invalid_count must equal "
        "node_tree_manifest_sidecars node_tree_manifest_valid=false count when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_path_incomplete_count must equal "
        "node_tree_manifest_sidecars path-incomplete count when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_path_map_mismatch_count must equal "
        "node_tree_manifest_sidecars node_tree_manifest_path_map_mismatch_count "
        "sum when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecar_validation_error_count must equal "
        "node_tree_manifest_sidecars node_tree_manifest_validation_error_count "
        "sum when preview is not truncated"
    ) in errors
    assert (
        "node_tree_manifest_sidecars[1].path_maps_complete must match path map counts"
        in errors
    )


def test_delivery_acceptance_gate_contract_accepts_canonical_payload() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_verified_acceptance=True
        ),
        "passed": False,
        "exit_code": 1,
        "level": "static_only",
        "complete": False,
        "reasons": ["1 robot(s) were not run through Godot smoke"],
        "reason_codes": ["missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "1 robot(s) were not run through Godot smoke",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {
            "inputs_count": 1,
            "control_configured_count": 1,
            "control_readback_checked_count": 0,
            "control_readback_missing_count": 1,
            "node_tree_gate_check_counts": {"fixed_lock_mismatch": 1},
            "mechanical_gate_check_counts": {"control_parameter_readback": 1},
        },
    }

    assert validate_delivery_acceptance_gate(gate) == []


def test_delivery_acceptance_gate_contract_reports_invalid_shape() -> None:
    errors = validate_delivery_acceptance_gate(
        {
            "contract_version": "old",
            "source": "",
            "acceptance_requirements": {"unknown_gate": True, "run_godot_smoke": 1},
            "reason_details": [{"code": "", "count": -1}],
            "summary_counts": [],
        }
    )

    assert any("missing required fields" in error for error in errors)
    assert any("contract_version must be" in error for error in errors)
    assert any("source must be a non-empty string" in error for error in errors)
    assert any("unknown fields" in error for error in errors)
    assert any("values must be booleans" in error for error in errors)
    assert any("reason_details[1].code" in error for error in errors)
    assert any("summary_counts must be an object" in error for error in errors)


def test_delivery_acceptance_gate_contract_rejects_invalid_summary_counts() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["control readback metadata missing"],
        "reason_codes": ["control_readback_missing"],
        "reason_details": [
            {
                "code": "control_readback_missing",
                "count": 1,
                "message": "control readback metadata missing",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {
            "inputs_count": -1,
            "control_readback_missing_count": "1",
            "mechanical_behavior_evidence_count": -1,
            "node_tree_gate_check_counts": {"fixed_lock_mismatch": -1},
            "mechanical_gate_check_counts": [],
        },
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "summary_counts.inputs_count must be a non-negative integer" in error
        for error in errors
    )
    assert any(
        "summary_counts.control_readback_missing_count must be a non-negative integer"
        in error
        for error in errors
    )
    assert any(
        "summary_counts.mechanical_behavior_evidence_count must be a "
        "non-negative integer" in error
        for error in errors
    )
    assert any(
        "summary_counts.node_tree_gate_check_counts values must be non-negative "
        "integers: fixed_lock_mismatch" in error
        for error in errors
    )
    assert any(
        "summary_counts.mechanical_gate_check_counts must be an object" in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_unknown_and_source_specific_summary_counts() -> None:
    cli_gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["Godot smoke did not run"],
        "reason_codes": ["missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {
            "inputs_count": 1,
            "unknown_counter": 1,
        },
    }
    web_gate = {
        **cli_gate,
        "source": "web_godot_delivery",
        "verification_scope": "godot_load",
        "acceptance_profile": "web_godot_load",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_load=True
        ),
        "reasons": ["Godot delivery failed before load acceptance."],
        "reason_codes": ["godot_delivery_failed"],
        "reason_details": [
            {
                "code": "godot_delivery_failed",
                "count": 1,
                "message": "Godot delivery failed before load acceptance.",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {
            "inputs_count": 1,
            "control_readback_missing_count": 1,
        },
    }

    cli_errors = validate_delivery_acceptance_gate(cli_gate)
    web_errors = validate_delivery_acceptance_gate(web_gate)

    assert any(
        "summary_counts contains unknown fields: unknown_counter" in error
        for error in cli_errors
    )
    assert any(
        "summary_counts contains fields not allowed for source "
        "'web_godot_delivery': control_readback_missing_count" in error
        for error in web_errors
    )


def test_delivery_acceptance_gate_contract_rejects_inconsistent_summary_count_totals() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["Godot smoke did not run"],
        "reason_codes": ["missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {
            "inputs_count": 2,
            "success_count": 2,
            "error_count": 1,
            "live_smoke_count": 3,
            "delivery_godot_verified_count": 1,
            "delivery_static_only_count": 0,
            "delivery_unverified_count": 0,
            "delivery_complete_count": 1,
            "delivery_incomplete_count": 0,
        },
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "summary_counts.success_count + error_count must equal inputs_count" in error
        for error in errors
    )
    assert any(
        "summary_counts delivery provenance counts must equal inputs_count" in error
        for error in errors
    )
    assert any(
        "summary_counts.delivery_complete_count + delivery_incomplete_count "
        "must equal inputs_count" in error
        for error in errors
    )
    assert any(
        "summary_counts.live_smoke_count must be <= inputs_count" in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_inconsistent_gate_check_counts() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["Godot smoke did not run"],
        "reason_codes": ["missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {
            "inputs_count": 1,
            "success_count": 0,
            "error_count": 1,
            "live_smoke_count": 0,
            "delivery_godot_verified_count": 0,
            "delivery_static_only_count": 0,
            "delivery_unverified_count": 1,
            "delivery_complete_count": 0,
            "delivery_incomplete_count": 1,
            "node_tree_gate_enabled_count": 3,
            "node_tree_gate_check_counts": {
                "incomplete_node_tree": 1,
                "class_mismatch": 1,
                "node_tree_typo": 0,
            },
            "mechanical_gate_enabled_count": 1,
            "mechanical_gate_check_counts": {
                "mechanical_restoration": 1,
                "joint_parameter_readback": 1,
                "readback_typo": 0,
            },
        },
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "summary_counts.node_tree_gate_check_counts values must sum to "
        "node_tree_gate_enabled_count" in error
        for error in errors
    )
    assert any(
        "summary_counts.node_tree_gate_check_counts contains unknown check keys: "
        "node_tree_typo" in error
        for error in errors
    )
    assert any(
        "summary_counts.mechanical_gate_check_counts values must sum to "
        "mechanical_gate_enabled_count" in error
        for error in errors
    )
    assert any(
        "summary_counts.mechanical_gate_check_counts contains unknown check keys: "
        "readback_typo" in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_reason_code_mismatch() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": [
            "control readback metadata missing",
            "smoke failure reason was reported",
        ],
        "reason_codes": ["control_readback_missing", ""],
        "reason_details": [
            {
                "code": "smoke_failure_reasons",
                "count": 1,
                "message": "smoke failure reason was reported",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "reason_codes entries must be non-empty strings: 2" in error
        for error in errors
    )
    assert any(
        "reason_codes missing matching reason_details: control_readback_missing"
        in error
        for error in errors
    )
    assert any(
        "reason_details codes missing from reason_codes: smoke_failure_reasons"
        in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_reason_message_mismatch() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": [
            "control readback metadata missing",
            "stale human reason",
        ],
        "reason_codes": ["control_readback_missing"],
        "reason_details": [
            {
                "code": "control_readback_missing",
                "count": 1,
                "message": "control readback metadata missing",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
            {
                "code": "smoke_failure_reasons",
                "count": 1,
                "message": "smoke failure reason was reported",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {"inputs_count": 1},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "reasons missing matching reason_details messages: stale human reason"
        in error
        for error in errors
    )
    assert any(
        "reason_details messages missing from reasons: "
        "smoke failure reason was reported" in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_reason_order_mismatch() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": [
            "smoke failure reason was reported",
            "control readback metadata missing",
        ],
        "reason_codes": ["smoke_failure_reasons", "control_readback_missing"],
        "reason_details": [
            {
                "code": "control_readback_missing",
                "count": 1,
                "message": "control readback metadata missing",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
            {
                "code": "smoke_failure_reasons",
                "count": 1,
                "message": "smoke failure reason was reported",
                "inputs": ["robot-b.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {"inputs_count": 2},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert "reason_codes must match reason_details codes in order" in errors
    assert "reasons must match reason_details messages in order" in errors


def test_delivery_acceptance_gate_contract_rejects_duplicate_reason_messages() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": [
            "Godot smoke did not run",
            "Godot smoke did not run",
        ],
        "reason_codes": ["missing_godot_smoke", "missing_godot_smoke_report"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
            {
                "code": "missing_godot_smoke_report",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot-b.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {"inputs_count": 2},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "reasons entries must be unique: Godot smoke did not run" in error
        for error in errors
    )
    assert any(
        "reason_details messages must be unique: Godot smoke did not run" in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_unknown_reason_codes() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["temporary blocker"],
        "reason_codes": ["temporary_blocker"],
        "reason_details": [
            {
                "code": "temporary_blocker",
                "count": 1,
                "message": "temporary blocker",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "reason_codes contains unknown values: temporary_blocker" in error
        for error in errors
    )
    assert any(
        "reason_details[1].code contains unknown value: temporary_blocker" in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_source_specific_reason_codes() -> None:
    cli_gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["Godot delivery failed before load acceptance."],
        "reason_codes": ["godot_delivery_failed"],
        "reason_details": [
            {
                "code": "godot_delivery_failed",
                "count": 1,
                "message": "Godot delivery failed before load acceptance.",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }
    web_gate = {
        **cli_gate,
        "source": "web_godot_delivery",
        "verification_scope": "godot_load",
        "acceptance_profile": "web_godot_load",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_load=True
        ),
        "reason_codes": ["missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
    }

    cli_errors = validate_delivery_acceptance_gate(cli_gate)
    web_errors = validate_delivery_acceptance_gate(web_gate)

    assert any(
        "reason_codes contains values not allowed for source "
        "'dynamic_godot_report_cli': godot_delivery_failed" in error
        for error in cli_errors
    )
    assert any(
        "reason_details[1].code is not allowed for source "
        "'dynamic_godot_report_cli': godot_delivery_failed" in error
        for error in cli_errors
    )
    assert any(
        "reason_codes contains values not allowed for source "
        "'web_godot_delivery': missing_godot_smoke" in error
        for error in web_errors
    )
    assert any(
        "reason_details[1].code is not allowed for source "
        "'web_godot_delivery': missing_godot_smoke" in error
        for error in web_errors
    )


def test_delivery_acceptance_gate_contract_rejects_duplicate_reason_codes() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": [
            "Godot smoke did not run",
            "Godot smoke still did not run",
        ],
        "reason_codes": ["missing_godot_smoke", "missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot-a.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke still did not run",
                "inputs": ["robot-b.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {"inputs_count": 2},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "reason_codes entries must be unique: missing_godot_smoke" in error
        for error in errors
    )
    assert any(
        "reason_details codes must be unique: missing_godot_smoke" in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_invalid_reason_detail_inputs() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": [
            "control readback metadata missing",
            "smoke failure reason was reported",
        ],
        "reason_codes": ["control_readback_missing", "smoke_failure_reasons"],
        "reason_details": [
            {
                "code": "control_readback_missing",
                "count": 0,
                "message": "control readback metadata missing",
                "inputs": ["robot.json", "", "robot.json"],
                "inputs_count": 1,
                "inputs_truncated": True,
                "transient_debug_note": "do not persist this",
            },
            {
                "code": "smoke_failure_reasons",
                "count": 2,
                "message": "smoke failure reason was reported",
                "inputs": ["robot-a.json"],
                "inputs_count": 2,
                "inputs_truncated": False,
            },
        ],
        "summary_counts": {"inputs_count": 2},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "reason_details[1] contains unknown fields: transient_debug_note" in error
        for error in errors
    )
    assert any(
        "reason_details[1].inputs entries must be non-empty strings: 2" in error
        for error in errors
    )
    assert any(
        "reason_details[1].inputs entries must be unique: robot.json" in error
        for error in errors
    )
    assert any(
        "reason_details[1].inputs_count must be greater than or equal to "
        "the inputs preview length" in error
        for error in errors
    )
    assert any(
        "reason_details[2].inputs_count must equal the inputs preview length "
        "when inputs_truncated is false" in error
        for error in errors
    )
    assert any(
        "reason_details[1].count must be greater than or equal to inputs_count"
        in error
        for error in errors
    )


def test_delivery_acceptance_gate_contract_rejects_result_inconsistency() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": True,
        "exit_code": 1,
        "level": "godot_verified",
        "complete": True,
        "reasons": ["control readback metadata missing", ""],
        "reason_codes": ["control_readback_missing"],
        "reason_details": [
            {
                "code": "control_readback_missing",
                "count": 1,
                "message": "control readback metadata missing",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert any(
        "passed must equal whether exit_code is zero" in error for error in errors
    )
    assert "exit_code must be zero when complete is true" in errors
    assert any("reasons entries must be non-empty strings: 2" in error for error in errors)
    assert any("reasons must be empty when complete is true" in error for error in errors)
    assert any(
        "reason_codes must be empty when complete is true" in error
        for error in errors
    )
    assert any(
        "reason_details must be empty when complete is true" in error
        for error in errors
    )

    failed_complete_gate = {
        **gate,
        "passed": False,
        "exit_code": 1,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
    }

    failed_complete_errors = validate_delivery_acceptance_gate(failed_complete_gate)

    assert "passed must be true when complete is true" in failed_complete_errors
    assert "exit_code must be zero when complete is true" in failed_complete_errors


def test_delivery_acceptance_gate_contract_rejects_complete_level_inconsistency() -> None:
    complete_static_gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": True,
        "exit_code": 0,
        "level": "static_only",
        "complete": True,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {"inputs_count": 1},
    }
    incomplete_verified_gate = {
        **complete_static_gate,
        "level": "godot_verified",
        "complete": False,
    }
    web_incomplete_verified_gate = {
        **complete_static_gate,
        "source": "web_godot_delivery",
        "verification_scope": "godot_load",
        "acceptance_profile": "web_godot_load",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_load=True
        ),
        "level": "godot_load_verified",
        "complete": False,
    }

    complete_errors = validate_delivery_acceptance_gate(complete_static_gate)
    incomplete_errors = validate_delivery_acceptance_gate(incomplete_verified_gate)
    web_errors = validate_delivery_acceptance_gate(web_incomplete_verified_gate)

    assert any(
        "level must be 'godot_verified' when complete is true for "
        "verification_scope 'godot_smoke_motion'" in error
        for error in complete_errors
    )
    assert "complete must be true when level is 'godot_verified'" in incomplete_errors
    assert "complete must be true when level is 'godot_load_verified'" in web_errors


def test_delivery_acceptance_gate_contract_rejects_complete_gate_without_inputs() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": True,
        "exit_code": 0,
        "level": "godot_verified",
        "complete": True,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {"inputs_count": 0},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert (
        "summary_counts.inputs_count must be greater than zero when complete is true"
        in errors
    )


def test_delivery_acceptance_gate_contract_rejects_complete_gate_missing_required_summary_counts() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": True,
        "exit_code": 0,
        "level": "godot_verified",
        "complete": True,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {"inputs_count": 1},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert "summary_counts.success_count must be 1 when complete is true" in errors
    assert (
        "summary_counts.delivery_complete_count must be 1 when complete is true"
        in errors
    )
    assert "summary_counts.error_count must be 0 when complete is true" in errors
    assert (
        "summary_counts.live_smoke_count must be 1 when complete is true" in errors
    )
    assert (
        "summary_counts.smoke_report_written_count must be 1 when complete is true"
        in errors
    )
    assert (
        "summary_counts.smoke_report_missing_count must be 0 when complete is true"
        in errors
    )


def test_delivery_acceptance_gate_contract_rejects_complete_gate_failure_counts() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": True,
        "exit_code": 0,
        "level": "godot_verified",
        "complete": True,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {
            "inputs_count": 2,
            "success_count": 1,
            "error_count": 1,
            "delivery_godot_verified_count": 1,
            "delivery_dynamic_generation_count": 1,
            "delivery_static_only_count": 1,
            "delivery_unverified_count": 0,
            "delivery_complete_count": 1,
            "delivery_incomplete_count": 1,
            "delivery_parameters_incomplete_count": 1,
            "control_readback_missing_count": 1,
            "failure_reasons_count": 1,
            "fixed_lock_mismatch_count": 1,
            "node_tree_fixed_lock_mismatch_count": 1,
            "node_tree_fixed_locks_incomplete_count": 1,
        },
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert "summary_counts.success_count must be 2 when complete is true" in errors
    assert "summary_counts.error_count must be 0 when complete is true" in errors
    assert (
        "summary_counts.delivery_complete_count must be 2 when complete is true"
        in errors
    )
    assert (
        "summary_counts.delivery_godot_verified_count must be 2 when complete is true"
        in errors
    )
    assert (
        "summary_counts.delivery_dynamic_generation_count must be 2 when complete is true"
        in errors
    )
    assert (
        "summary_counts.delivery_incomplete_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.delivery_parameters_incomplete_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.delivery_static_only_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.control_readback_missing_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.failure_reasons_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.fixed_lock_mismatch_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.node_tree_fixed_lock_mismatch_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.node_tree_fixed_locks_incomplete_count must be 0 when complete is true"
        in errors
    )


def test_delivery_acceptance_gate_contract_rejects_complete_cli_smoke_count_mismatch() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": True,
        "exit_code": 0,
        "level": "godot_verified",
        "complete": True,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {
            "inputs_count": 2,
            "success_count": 2,
            "error_count": 0,
            "delivery_godot_verified_count": 2,
            "delivery_dynamic_generation_count": 2,
            "delivery_static_only_count": 0,
            "delivery_unverified_count": 0,
            "delivery_complete_count": 2,
            "delivery_incomplete_count": 0,
            "live_smoke_count": 1,
            "smoke_report_written_count": 1,
            "smoke_report_missing_count": 1,
            "smoke_report_read_error_count": 1,
        },
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert "summary_counts.live_smoke_count must be 2 when complete is true" in errors
    assert (
        "summary_counts.smoke_report_written_count must be 2 when complete is true"
        in errors
    )
    assert (
        "summary_counts.smoke_report_missing_count must be 0 when complete is true"
        in errors
    )
    assert (
        "summary_counts.smoke_report_read_error_count must be 0 when complete is true"
        in errors
    )


def test_delivery_acceptance_gate_contract_rejects_complete_cli_missing_required_check_counts() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            node_tree_parameters_applied=True,
            joint_parameter_readback=True,
        ),
        "passed": True,
        "exit_code": 0,
        "level": "godot_verified",
        "complete": True,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {
            "inputs_count": 2,
            "success_count": 2,
            "error_count": 0,
            "delivery_godot_verified_count": 2,
            "delivery_dynamic_generation_count": 2,
            "delivery_static_only_count": 0,
            "delivery_unverified_count": 0,
            "delivery_complete_count": 2,
            "delivery_incomplete_count": 0,
            "delivery_parameters_incomplete_count": 0,
            "control_readback_missing_count": 0,
            "failure_reasons_count": 0,
            "fixed_lock_mismatch_count": 0,
            "node_tree_fixed_lock_mismatch_count": 0,
            "node_tree_fixed_locks_incomplete_count": 0,
            "live_smoke_count": 2,
            "smoke_report_written_count": 2,
            "smoke_report_missing_count": 0,
            "smoke_report_read_error_count": 0,
            "node_tree_gate_enabled_count": 1,
            "node_tree_gate_check_counts": {
                "incomplete_node_tree": 0,
                "class_mismatch": 0,
                "missing_parameters": 1,
                "transform_mismatch": 0,
                "physical_mismatch": 0,
                "fixed_lock_mismatch": 0,
            },
            "mechanical_gate_enabled_count": 1,
            "mechanical_gate_check_counts": {
                "mechanical_restoration": 0,
                "joint_parameter_readback": 1,
                "control_parameter_readback": 0,
                "full_node_tree_restoration": 0,
            },
        },
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert (
        "summary_counts.node_tree_gate_check_counts.missing_parameters must be 2 "
        "when complete is true and the matching requirement is enabled" in errors
    )
    assert (
        "summary_counts.mechanical_gate_check_counts.joint_parameter_readback must "
        "be 2 when complete is true and the matching requirement is enabled"
        in errors
    )


def test_delivery_acceptance_gate_contract_rejects_complete_web_missing_required_check_counts() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "web_godot_delivery",
        "verification_scope": "godot_load",
        "required": False,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "web_godot_load",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_load=True,
            mechanical_restoration_complete=True,
            joint_parameter_readback=True,
            node_tree_fixed_lock_match=True,
        ),
        "passed": True,
        "exit_code": 0,
        "level": "godot_load_verified",
        "complete": True,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {
            "inputs_count": 1,
            "success_count": 1,
            "error_count": 0,
            "live_smoke_count": 0,
            "delivery_godot_verified_count": 1,
            "delivery_static_only_count": 0,
            "delivery_unverified_count": 0,
            "delivery_dynamic_generation_count": 1,
            "delivery_complete_count": 1,
            "delivery_incomplete_count": 0,
            "delivery_parameters_incomplete_count": 0,
            "fixed_lock_checked_count": 0,
            "fixed_lock_mismatch_count": 0,
            "node_tree_fixed_lock_checked_count": 0,
            "node_tree_fixed_lock_mismatch_count": 0,
            "node_tree_fixed_locks_complete_count": 0,
            "node_tree_fixed_locks_incomplete_count": 0,
            "node_tree_gate_enabled_count": 1,
            "node_tree_full_restoration_required_count": 0,
            "node_tree_full_restoration_not_required_count": 1,
            "node_tree_gate_check_counts": {"fixed_lock_mismatch": 0},
            "mechanical_gate_enabled_count": 2,
            "full_mechanical_restoration_required_count": 0,
            "full_mechanical_restoration_not_required_count": 1,
            "mechanical_gate_check_counts": {
                "mechanical_restoration": 1,
                "joint_parameter_readback": 0,
            },
            "failure_reasons_count": 0,
        },
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert (
        "summary_counts.node_tree_gate_check_counts.fixed_lock_mismatch must be 1 "
        "when complete is true and the matching requirement is enabled" in errors
    )
    assert (
        "summary_counts.mechanical_gate_check_counts.joint_parameter_readback must "
        "be 1 when complete is true and the matching requirement is enabled"
        in errors
    )


def test_delivery_acceptance_gate_contract_rejects_failed_gate_without_reasons() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": [],
        "reason_codes": [],
        "reason_details": [],
        "summary_counts": {"inputs_count": 0},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert "reasons must not be empty when exit_code is non-zero" in errors
    assert "reason_codes must not be empty when exit_code is non-zero" in errors
    assert "reason_details must not be empty when exit_code is non-zero" in errors


def test_delivery_acceptance_gate_contract_rejects_required_incomplete_success() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": True,
        "acceptance_profile": "full_mechanical_restoration",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            full_mechanical_restoration_gate=True
        ),
        "passed": True,
        "exit_code": 0,
        "level": "incomplete",
        "complete": False,
        "reasons": ["full mechanical restoration gate missing"],
        "reason_codes": ["missing_full_mechanical_restoration_gate"],
        "reason_details": [
            {
                "code": "missing_full_mechanical_restoration_gate",
                "count": 1,
                "message": "full mechanical restoration gate missing",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }

    errors = validate_delivery_acceptance_gate(gate)

    assert "exit_code must be non-zero when required gate is incomplete" in errors
    assert (
        "exit_code must be non-zero when full mechanical restoration gate is "
        "incomplete" in errors
    )


def test_delivery_acceptance_gate_contract_rejects_requirement_inconsistency() -> None:
    full_mechanical_gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": True,
        "acceptance_profile": "full_mechanical_restoration",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "incomplete",
        "complete": False,
        "reasons": ["full mechanical restoration gate missing"],
        "reason_codes": ["missing_full_mechanical_restoration_gate"],
        "reason_details": [
            {
                "code": "missing_full_mechanical_restoration_gate",
                "count": 1,
                "message": "full mechanical restoration gate missing",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }
    godot_load_gate = {
        **full_mechanical_gate,
        "source": "web_godot_delivery",
        "verification_scope": "godot_load",
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "web_godot_load",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
    }
    profile_requires_mismatch_gate = {
        **full_mechanical_gate,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_requirements": build_delivery_acceptance_requirements(
            full_mechanical_restoration_gate=True
        ),
    }

    full_errors = validate_delivery_acceptance_gate(full_mechanical_gate)
    load_errors = validate_delivery_acceptance_gate(godot_load_gate)
    profile_errors = validate_delivery_acceptance_gate(profile_requires_mismatch_gate)

    assert any(
        "acceptance_requirements.full_mechanical_restoration_gate must be true "
        "when requires_full_mechanical_restoration_gate is true" in error
        for error in full_errors
    )
    assert any(
        "requires_full_mechanical_restoration_gate must be true when "
        "acceptance_profile is 'full_mechanical_restoration'" in error
        for error in profile_errors
    )
    assert any(
        "acceptance_requirements.godot_load must be true when verification_scope "
        "is 'godot_load'" in error
        for error in load_errors
    )


def test_delivery_acceptance_gate_contract_rejects_unknown_metadata_values() -> None:
    gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_load",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "experimental",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_load=True
        ),
        "passed": False,
        "exit_code": 1,
        "level": "partial",
        "complete": False,
        "reasons": ["Godot smoke did not run"],
        "reason_codes": ["missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }
    unknown_gate = {
        **gate,
        "source": "unknown_source",
        "verification_scope": "unknown_scope",
        "acceptance_profile": "unknown_profile",
        "level": "unknown_level",
    }

    mismatch_errors = validate_delivery_acceptance_gate(gate)
    unknown_errors = validate_delivery_acceptance_gate(unknown_gate)

    assert any(
        "verification_scope must be 'godot_smoke_motion' when source is "
        "'dynamic_godot_report_cli'" in error
        for error in mismatch_errors
    )
    assert any("acceptance_profile must be one of:" in error for error in mismatch_errors)
    assert any("level must be one of:" in error for error in mismatch_errors)
    assert any("source must be one of:" in error for error in unknown_errors)
    assert any("verification_scope must be one of:" in error for error in unknown_errors)
    assert any("acceptance_profile must be one of:" in error for error in unknown_errors)
    assert any("level must be one of:" in error for error in unknown_errors)


def test_delivery_acceptance_gate_contract_rejects_metadata_combination_errors() -> None:
    web_profile_on_cli_gate = {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": "dynamic_godot_report_cli",
        "verification_scope": "godot_smoke_motion",
        "required": True,
        "requires_full_mechanical_restoration_gate": False,
        "acceptance_profile": "web_godot_load",
        "acceptance_requirements": build_delivery_acceptance_requirements(),
        "passed": False,
        "exit_code": 1,
        "level": "godot_load_verified",
        "complete": False,
        "reasons": ["Godot smoke did not run"],
        "reason_codes": ["missing_godot_smoke"],
        "reason_details": [
            {
                "code": "missing_godot_smoke",
                "count": 1,
                "message": "Godot smoke did not run",
                "inputs": ["robot.json"],
                "inputs_count": 1,
                "inputs_truncated": False,
            }
        ],
        "summary_counts": {"inputs_count": 1},
    }
    full_profile_without_requirement = {
        **web_profile_on_cli_gate,
        "acceptance_profile": "full_mechanical_restoration",
        "level": "incomplete",
    }
    custom_profile_on_web_gate = {
        **web_profile_on_cli_gate,
        "source": "web_godot_delivery",
        "verification_scope": "godot_load",
        "acceptance_profile": "custom",
        "acceptance_requirements": build_delivery_acceptance_requirements(
            godot_load=True
        ),
        "level": "static_only",
    }

    cli_errors = validate_delivery_acceptance_gate(web_profile_on_cli_gate)
    full_errors = validate_delivery_acceptance_gate(full_profile_without_requirement)
    web_errors = validate_delivery_acceptance_gate(custom_profile_on_web_gate)

    assert any(
        "acceptance_profile must be one of custom, full_mechanical_restoration "
        "when source is 'dynamic_godot_report_cli'" in error
        for error in cli_errors
    )
    assert any(
        "level must be one of godot_verified, incomplete, static_only when "
        "verification_scope is 'godot_smoke_motion'" in error
        for error in cli_errors
    )
    assert any(
        "acceptance_requirements.full_mechanical_restoration_gate must be true "
        "when acceptance_profile is 'full_mechanical_restoration'" in error
        for error in full_errors
    )
    assert any(
        "acceptance_profile must be one of web_godot_load when source is "
        "'web_godot_delivery'" in error
        for error in web_errors
    )
    assert any(
        "level must be one of godot_load_verified, incomplete when "
        "verification_scope is 'godot_load'" in error
        for error in web_errors
    )


def test_workflow_step_artifact_contract_accepts_canonical_payload() -> None:
    artifact = build_workflow_step_artifact(
        workflow="robot_creation_pipeline",
        step="create_model",
        executor="robot_modeling",
        action="create_from_template",
        status="completed",
        mode="real",
        inputs={"template": "biped_basic"},
        output={"status": "success", "output_file": "robot.json"},
        artifact_index=1,
        attempts=1,
        duration_seconds=0.25,
        created_at="2026-04-11T00:00:00",
    )

    assert artifact["schema_version"] == WORKFLOW_CONTRACT_VERSION
    assert validate_workflow_step_artifact(artifact) == []


def test_workflow_step_artifact_contract_reports_missing_fields() -> None:
    errors = validate_workflow_step_artifact(
        {
            "schema_version": WORKFLOW_CONTRACT_VERSION,
            "artifact_type": "workflow_step",
            "workflow": "robot_creation_pipeline",
        }
    )

    assert any("missing required fields" in error for error in errors)
    assert any("status must be" in error for error in errors)


def test_workflow_definition_contract_rejects_invalid_references_and_actions() -> None:
    errors = validate_workflow_definition(
        "bad_workflow",
        {
            "name": "bad_workflow",
            "steps": [
                {
                    "name": "create_model",
                    "skill_executor": "robot_modeling",
                    "action": "fly",
                    "inputs": {"robot_config": "{future.output_file}"},
                }
            ],
        },
        {"robot_modeling": {"create_from_template", "load_config"}},
    )

    assert any("is not supported" in error for error in errors)
    assert any("missing or future step" in error for error in errors)


def test_robot_config_part_optimization_and_export_contracts() -> None:
    assert (
        validate_robot_config(
            {
                "name": "test_robot",
                "parts": [
                    {"id": "torso_1", "type": "torso", "params": {"mass": 8.0}},
                    {"id": "shin_1", "type": "leg", "params": {"mass": 2.0}},
                ],
                "connections": [
                    {
                        "from": "torso_1",
                        "to": "shin_1",
                        "joint_type": "revolute",
                    }
                ],
                "metadata": {},
            }
        )
        == []
    )
    assert (
        validate_part_spec(
            {
                "id": "go_m8010",
                "category": "actuators",
                "name": "Unitree Go-M8010 Style",
                "weight_kg": 0.53,
                "cost_usd": 350.0,
                "specs": {"max_torque_nm": 23.7},
            }
        )
        == []
    )
    assert (
        validate_optimization_result(
            {
                "success": True,
                "iterations": 4,
                "mass_distribution": {"torso_1": 7.5},
                "com_position": [0.0, 0.0, 0.4],
                "com_error": 0.01,
            }
        )
        == []
    )
    assert (
        validate_export_result(
            {
                "status": "success",
                "action": "export_to_format",
                "output_file": "robot.urdf",
                "format": "urdf",
                "file_size": 100,
                "output_generated": True,
            }
        )
        == []
    )
