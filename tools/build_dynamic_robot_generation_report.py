"""Build a diagnostic report for JSON-to-Godot dynamic robot generation."""

from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

_repo_root = Path(__file__).resolve().parents[1]
_workflow_contracts_path = (
    _repo_root / "agi_walker" / "core" / "api" / "workflow_contracts.py"
)
_workflow_contracts_spec = importlib.util.spec_from_file_location(
    "workflow_contracts", _workflow_contracts_path
)
if _workflow_contracts_spec is None or _workflow_contracts_spec.loader is None:
    raise RuntimeError(
        f"Unable to load workflow contracts from {_workflow_contracts_path}"
    )
_workflow_contracts = importlib.util.module_from_spec(_workflow_contracts_spec)
_workflow_contracts_spec.loader.exec_module(_workflow_contracts)
DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION = (
    _workflow_contracts.DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION
)
build_delivery_acceptance_requirements = (
    _workflow_contracts.build_delivery_acceptance_requirements
)
validate_delivery_acceptance_gate = _workflow_contracts.validate_delivery_acceptance_gate

DELIVERY_ACCEPTANCE_GATE_SOURCE = "dynamic_godot_report_cli"
DELIVERY_ACCEPTANCE_GATE_SCOPE = "godot_smoke_motion"


def _configure_stdio_for_json_output() -> None:
    """Keep CLI report output stable on non-UTF-8 Windows consoles."""
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(errors="replace")


def _load_robot_schema_module(repo_root: Path) -> Any:
    module_path = repo_root / "agi_walker" / "core" / "api" / "robot_schema.py"
    spec = importlib.util.spec_from_file_location("robot_schema", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load robot schema module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")


def _read_optional_json_object(path: Path) -> tuple[dict[str, Any], str | None]:
    if not path.exists():
        return {}, None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return {}, f"{exc.msg} at line {exc.lineno} column {exc.colno}"
    if not isinstance(payload, dict):
        return {}, "top-level JSON value is not an object"
    return payload, None


def _build_static_report(
    *,
    input_path: Path,
    normalized: dict[str, Any],
    errors: list[str],
    topology_summary: dict[str, Any],
    node_tree_manifest: dict[str, Any],
    node_tree_manifest_errors: list[str],
    node_tree_manifest_path_map_mismatches: list[dict[str, Any]],
) -> dict[str, Any]:
    parts = normalized.get("parts", [])
    connections = normalized.get("connections", [])
    shape_counts: dict[str, int] = {}
    joint_counts: dict[str, int] = {}
    for part in parts:
        if isinstance(part, dict):
            shape = str(part.get("shape", "unknown"))
            shape_counts[shape] = shape_counts.get(shape, 0) + 1
    for connection in connections:
        if isinstance(connection, dict):
            joint_type = str(connection.get("joint_type", "unknown"))
            joint_counts[joint_type] = joint_counts.get(joint_type, 0) + 1

    return {
        "input": str(input_path),
        "robot_name": normalized.get("name"),
        "schema_version": normalized.get("schema_version"),
        "status": "error" if errors else "success",
        "parts_count": len(parts) if isinstance(parts, list) else 0,
        "connections_count": len(connections) if isinstance(connections, list) else 0,
        "shape_counts": shape_counts,
        "joint_counts": joint_counts,
        "topology_summary": topology_summary,
        "node_tree_manifest": node_tree_manifest,
        "node_tree_manifest_errors": node_tree_manifest_errors,
        "node_tree_manifest_path_map_mismatch_count": len(
            node_tree_manifest_path_map_mismatches
        ),
        "node_tree_manifest_path_map_mismatch_kind_counts": (
            _node_tree_manifest_path_map_mismatch_kind_counts(
                node_tree_manifest_path_map_mismatches
            )
        ),
        "node_tree_manifest_path_map_mismatches": (
            node_tree_manifest_path_map_mismatches
        ),
        "errors": errors,
    }


def _build_delivery_contract_preview(static_report: dict[str, Any]) -> dict[str, Any]:
    return {
        "dynamic_robot_generation": static_report["status"] == "success",
        "complete": static_report["status"] == "success",
        "expected_parts": static_report["parts_count"],
        "parts_created": static_report["parts_count"],
        "parts_complete": static_report["status"] == "success",
        "expected_joints": static_report["connections_count"],
        "joints_created": static_report["connections_count"],
        "failed_joints": None,
        "joints_complete": static_report["status"] == "success",
        "parameterized_joints": None,
        "parameters_complete": None,
        "part_nodes_count": None,
        "joint_nodes_count": None,
        "failed_connections_count": None,
        "warnings_count": None,
        "source": "static_normalization",
    }


def _run_godot_smoke(
    *,
    repo_root: Path,
    normalized_output: Path,
    smoke_output: Path,
    godot_exe: str,
    port: int,
    timeout_seconds: float,
    max_endpoint_distance: float | None,
    max_relative_angle: float | None,
    min_body_displacement: float | None,
    max_linear_speed: float | None,
    min_joint_angle_delta: float | None,
    min_joint_angle_range: float | None,
    min_moving_joint_coverage: float | None,
    min_commanded_joint_response_coverage: float | None,
    joint_motion_epsilon: float,
    min_action_target_coverage: float | None,
    min_control_action_coverage: float | None,
    min_nonzero_action_targets: int | None,
    min_action_transitions: int | None,
    min_action_transition_delta: float | None,
    fail_on_joint_limit_violation: bool,
    fail_on_incomplete_restoration: bool,
    min_restoration_score: float | None,
    fail_on_parameter_mismatch: bool,
    fail_on_control_mismatch: bool,
    fail_on_full_mechanical_restoration: bool,
    fail_on_action_target_mismatch: bool,
    fail_on_action_sequence_target_mismatch: bool,
    fail_on_unknown_action_target: bool,
    fail_on_invalid_action_target: bool,
    fail_on_incomplete_node_tree: bool,
    fail_on_full_node_tree_restoration: bool,
    fail_on_node_tree_class_mismatch: bool,
    fail_on_node_tree_missing_parameters: bool,
    fail_on_node_tree_transform_mismatch: bool,
    fail_on_node_tree_physical_mismatch: bool,
    fail_on_node_tree_fixed_lock_mismatch: bool,
    node_tree_tolerance: float,
    parameter_tolerance: float,
    action_json: str,
    action_sequence_json: str | None,
    steps: int,
    step_delay_seconds: float,
    mechanical_trace_output: Path | None = None,
    live_profile: str = "local",
    live_artifact_root: Path | None = None,
    live_retention_days: int | None = None,
    flaky_retry_attempts: int | None = None,
) -> dict[str, Any]:
    command = [
        sys.executable,
        str(repo_root / "tools" / "run_dynamic_godot_robot_smoke.py"),
        str(normalized_output),
        "--godot-exe",
        godot_exe,
        "--live-profile",
        live_profile,
        "--port",
        str(port),
        "--timeout-seconds",
        str(timeout_seconds),
        "--action-json",
        action_json,
        "--steps",
        str(steps),
        "--step-delay-seconds",
        str(step_delay_seconds),
        "--output",
        str(smoke_output),
    ]
    if mechanical_trace_output is not None:
        command.extend(["--mechanical-trace-output", str(mechanical_trace_output)])
    if live_artifact_root is not None:
        command.extend(["--live-artifact-root", str(live_artifact_root)])
    if live_retention_days is not None:
        command.extend(["--live-retention-days", str(live_retention_days)])
    if flaky_retry_attempts is not None:
        command.extend(["--flaky-retry-attempts", str(flaky_retry_attempts)])
    if action_sequence_json is not None:
        command.extend(["--action-sequence-json", action_sequence_json])
    if max_endpoint_distance is not None:
        command.extend(["--max-endpoint-distance", str(max_endpoint_distance)])
    if max_relative_angle is not None:
        command.extend(["--max-relative-angle", str(max_relative_angle)])
    if min_body_displacement is not None:
        command.extend(["--min-body-displacement", str(min_body_displacement)])
    if max_linear_speed is not None:
        command.extend(["--max-linear-speed", str(max_linear_speed)])
    if min_joint_angle_delta is not None:
        command.extend(["--min-joint-angle-delta", str(min_joint_angle_delta)])
    if min_joint_angle_range is not None:
        command.extend(["--min-joint-angle-range", str(min_joint_angle_range)])
    if min_moving_joint_coverage is not None:
        command.extend(["--min-moving-joint-coverage", str(min_moving_joint_coverage)])
    if min_commanded_joint_response_coverage is not None:
        command.extend(
            [
                "--min-commanded-joint-response-coverage",
                str(min_commanded_joint_response_coverage),
            ]
        )
    command.extend(["--joint-motion-epsilon", str(joint_motion_epsilon)])
    if min_action_target_coverage is not None:
        command.extend(["--min-action-target-coverage", str(min_action_target_coverage)])
    if min_control_action_coverage is not None:
        command.extend(["--min-control-action-coverage", str(min_control_action_coverage)])
    if min_nonzero_action_targets is not None:
        command.extend(["--min-nonzero-action-targets", str(min_nonzero_action_targets)])
    if min_action_transitions is not None:
        command.extend(["--min-action-transitions", str(min_action_transitions)])
    if min_action_transition_delta is not None:
        command.extend(["--min-action-transition-delta", str(min_action_transition_delta)])
    if fail_on_joint_limit_violation:
        command.append("--fail-on-joint-limit-violation")
    if fail_on_incomplete_restoration:
        command.append("--fail-on-incomplete-restoration")
    if min_restoration_score is not None:
        command.extend(["--min-restoration-score", str(min_restoration_score)])
    if fail_on_parameter_mismatch:
        command.append("--fail-on-parameter-mismatch")
    if fail_on_control_mismatch:
        command.append("--fail-on-control-mismatch")
    if fail_on_full_mechanical_restoration:
        command.append("--fail-on-full-mechanical-restoration")
    if fail_on_action_target_mismatch:
        command.append("--fail-on-action-target-mismatch")
    if fail_on_action_sequence_target_mismatch:
        command.append("--fail-on-action-sequence-target-mismatch")
    if fail_on_unknown_action_target:
        command.append("--fail-on-unknown-action-target")
    if fail_on_invalid_action_target:
        command.append("--fail-on-invalid-action-target")
    if fail_on_incomplete_node_tree:
        command.append("--fail-on-incomplete-node-tree")
    if fail_on_full_node_tree_restoration:
        command.append("--fail-on-full-node-tree-restoration")
    if fail_on_node_tree_class_mismatch:
        command.append("--fail-on-node-tree-class-mismatch")
    if fail_on_node_tree_missing_parameters:
        command.append("--fail-on-node-tree-missing-parameters")
    if fail_on_node_tree_transform_mismatch:
        command.append("--fail-on-node-tree-transform-mismatch")
    if fail_on_node_tree_physical_mismatch:
        command.append("--fail-on-node-tree-physical-mismatch")
    if fail_on_node_tree_fixed_lock_mismatch:
        command.append("--fail-on-node-tree-fixed-lock-mismatch")
    command.extend(["--node-tree-tolerance", str(node_tree_tolerance)])
    command.extend(["--parameter-tolerance", str(parameter_tolerance)])
    if smoke_output.exists():
        smoke_output.unlink()
    result = subprocess.run(
        command,
        cwd=repo_root,
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    report_written = smoke_output.exists()
    smoke_report, report_read_error = _read_optional_json_object(smoke_output)
    stdout_lines = result.stdout.splitlines()
    stderr_lines = result.stderr.splitlines()
    execution_failure_reasons: list[str] = []
    if result.returncode != 0:
        execution_failure_reasons.append(
            f"godot smoke exited with returncode {result.returncode}"
        )
    if not report_written:
        execution_failure_reasons.append("godot smoke report was not written")
    if report_read_error:
        execution_failure_reasons.append(
            f"godot smoke report could not be read: {report_read_error}"
        )
    return {
        "command": command,
        "returncode": result.returncode,
        "report_written": report_written,
        "report_read_error": report_read_error,
        "execution_failure_reasons": execution_failure_reasons,
        "stdout_line_count": len(stdout_lines),
        "stderr_line_count": len(stderr_lines),
        "stdout_tail": stdout_lines[-20:],
        "stderr_tail": stderr_lines[-20:],
        "report_path": str(smoke_output),
        "report_summary": _build_smoke_report_summary(smoke_report),
        "live_verification": smoke_report.get("live_verification", {}),
    }


def _build_smoke_report_summary(smoke_report: dict[str, Any]) -> dict[str, Any]:
    load_result = smoke_report.get("load_result", {})
    mapping = smoke_report.get("mapping_summary", {})
    step = smoke_report.get("step_summary", {})
    node_tree_manifest = smoke_report.get("node_tree_manifest", {})
    return {
        "status": smoke_report.get("status"),
        "robot_name": smoke_report.get("robot_name"),
        "expected_parts": smoke_report.get("expected_parts"),
        "expected_joints": smoke_report.get("expected_joints"),
        "parts_created": load_result.get("parts_created"),
        "joints_created": load_result.get("joints_created"),
        "part_nodes_count": mapping.get("part_nodes"),
        "joint_nodes_count": mapping.get("joint_nodes"),
        "mechanical_gate_summary": smoke_report.get("mechanical_gate_summary", {}),
        "node_tree_gate_summary": smoke_report.get("node_tree_gate_summary", {}),
        "node_tree_manifest": node_tree_manifest,
        "live_verification": smoke_report.get("live_verification", {}),
        "mechanical_behavior_evidence": smoke_report.get(
            "mechanical_behavior_evidence", {}
        ),
        "node_tree_mismatch_preview": _build_node_tree_mismatch_preview(
            node_tree_manifest
        ),
        "step_body_count": step.get("body_count"),
        "step_joint_count": step.get("joint_count"),
        "step_body_states_count": step.get("body_states"),
        "step_joint_states_count": step.get("joint_states"),
        "first_body_state": step.get("first_body_state", {}),
        "first_joint_state": step.get("first_joint_state", {}),
        "joint_endpoint_summary": step.get("joint_endpoint_summary", {}),
        "joint_angle_summary": step.get("joint_angle_summary", {}),
        "joint_limit_summary": step.get("joint_limit_summary", {}),
        "joint_parameter_summary": step.get("joint_parameter_summary", {}),
        "joint_parameter_consistency_summary": step.get(
            "joint_parameter_consistency_summary", {}
        ),
        "joint_control_summary": step.get("joint_control_summary", {}),
        "joint_control_consistency_summary": step.get(
            "joint_control_consistency_summary", {}
        ),
        "action_target_consistency_summary": step.get(
            "action_target_consistency_summary", {}
        ),
        "action_sequence_target_consistency_summary": step.get(
            "action_sequence_target_consistency_summary", {}
        ),
        "action_target_coverage_summary": step.get(
            "action_target_coverage_summary", {}
        ),
        "control_action_coverage_summary": step.get(
            "control_action_coverage_summary", {}
        ),
        "simulation_summary": step.get("simulation_summary", {}),
        "joint_motion_summary": step.get("joint_motion_summary", {}),
        "action_sequence_summary": step.get("action_sequence_summary", {}),
        "mechanical_restoration_summary": step.get(
            "mechanical_restoration_summary", {}
        ),
        "action_sent": step.get("action_sent"),
        "first_action_sent": step.get("first_action_sent"),
        "last_action_sent": step.get("last_action_sent"),
        "steps_run": step.get("steps_run"),
        "errors": smoke_report.get("errors", []),
    }


def _build_node_tree_mismatch_preview(node_tree_manifest: Any) -> dict[str, Any]:
    if not isinstance(node_tree_manifest, dict):
        return {
            "class_mismatches": [],
            "transform_mismatches": [],
            "physical_mismatches": [],
            "fixed_lock_mismatches": [],
            "missing_parameter_connection_names": [],
        }
    return {
        "class_mismatches": _slice_list(
            node_tree_manifest.get("class_mismatches", []), 5
        ),
        "transform_mismatches": _slice_list(
            node_tree_manifest.get("transform_mismatches", []), 5
        ),
        "physical_mismatches": _slice_list(
            node_tree_manifest.get("physical_mismatches", []), 5
        ),
        "fixed_lock_mismatches": _slice_list(
            node_tree_manifest.get("fixed_lock_mismatches", []), 5
        ),
        "missing_parameter_connection_names": _slice_list(
            node_tree_manifest.get("missing_parameter_connection_names", []), 10
        ),
    }


def _slice_list(value: Any, limit: int) -> list[Any]:
    if not isinstance(value, list):
        return []
    return value[:limit]


def _unique_strings(values: Any) -> list[str]:
    result: list[str] = []
    for value in values:
        if isinstance(value, str) and value and value not in result:
            result.append(value)
    return result


def _node_tree_manifest_path_map_mismatch_kind_counts(
    mismatches: Any,
) -> dict[str, int]:
    counts: dict[str, int] = {}
    if not isinstance(mismatches, list):
        return counts
    for mismatch in mismatches:
        if not isinstance(mismatch, dict):
            continue
        kind = mismatch.get("kind")
        if not isinstance(kind, str) or not kind:
            continue
        counts[kind] = counts.get(kind, 0) + 1
    return counts


def _merge_count_maps(maps: Any) -> dict[str, int]:
    merged: dict[str, int] = {}
    for count_map in maps:
        if not isinstance(count_map, dict):
            continue
        for key, value in count_map.items():
            if isinstance(key, str) and key and isinstance(value, int):
                merged[key] = merged.get(key, 0) + value
    return merged


def _report_input(report: dict[str, Any]) -> str:
    static = report.get("static", {})
    if isinstance(static, dict) and static.get("input"):
        return str(static["input"])
    return ""


def _acceptance_reason_detail(
    *,
    code: str,
    count: int,
    message: str,
    inputs: list[str],
) -> dict[str, Any]:
    return {
        "code": code,
        "count": count,
        "message": message,
        "inputs": _slice_list(inputs, 10),
        "inputs_count": len(inputs),
        "inputs_truncated": len(inputs) > 10,
    }


def _build_live_delivery_summary(smoke_report: dict[str, Any]) -> dict[str, Any]:
    if not smoke_report:
        return {
            "dynamic_robot_generation": False,
            "complete": False,
            "expected_parts": None,
            "parts_created": None,
            "parts_complete": False,
            "expected_joints": None,
            "joints_created": None,
            "failed_joints": None,
            "joints_complete": False,
            "parameterized_joints": None,
            "parameters_complete": False,
            "part_nodes_count": None,
            "joint_nodes_count": None,
            "failed_connections_count": 0,
            "warnings_count": 0,
            "source": "missing_godot_smoke_report",
        }
    load_result = smoke_report.get("load_result", {})
    mapping = smoke_report.get("mapping_summary", {})
    load_result = load_result if isinstance(load_result, dict) else {}
    mapping = mapping if isinstance(mapping, dict) else {}
    return {
        "dynamic_robot_generation": smoke_report.get("status") == "success",
        "complete": load_result.get("complete") is True,
        "expected_parts": load_result.get("expected_parts"),
        "parts_created": load_result.get("parts_created"),
        "parts_complete": load_result.get("parts_complete") is True,
        "expected_joints": load_result.get("expected_joints"),
        "joints_created": load_result.get("joints_created"),
        "failed_joints": load_result.get("failed_joints"),
        "joints_complete": load_result.get("joints_complete") is True,
        "parameterized_joints": load_result.get("parameterized_joints"),
        "parameters_complete": load_result.get("parameters_complete") is True,
        "part_nodes_count": mapping.get("part_nodes"),
        "joint_nodes_count": mapping.get("joint_nodes"),
        "failed_connections_count": len(load_result.get("failed_connections", []))
        if isinstance(load_result.get("failed_connections"), list)
        else 0,
        "warnings_count": len(load_result.get("warnings", []))
        if isinstance(load_result.get("warnings"), list)
        else 0,
        "source": (
            "godot_smoke"
            if smoke_report.get("status") == "success"
            else "godot_smoke_failed"
        ),
    }


def _default_normalized_output(repo_root: Path, input_path: Path) -> Path:
    return repo_root / "test_env" / f"{input_path.stem}.godot.normalized.json"


def _default_smoke_output(repo_root: Path, input_path: Path) -> Path:
    return repo_root / "test_env" / f"{input_path.stem}.godot.smoke.json"


def _safe_artifact_stem(input_path: Path) -> str:
    stem = "".join(
        char if char.isalnum() or char in {"-", "_"} else "_"
        for char in input_path.stem
    ).strip("_")
    return stem or "robot"


def _static_node_tree_manifest_output_path(
    manifest_dir: Path,
    input_path: Path,
    index: int,
) -> Path:
    return (
        manifest_dir
        / f"{index:03d}_{_safe_artifact_stem(input_path)}.node_tree_manifest.json"
    )


def _write_static_node_tree_manifest_artifact(
    report: dict[str, Any],
    output_path: Path,
) -> None:
    static_report = report.get("static")
    node_tree_manifest = (
        static_report.get("node_tree_manifest") if isinstance(static_report, dict) else {}
    )
    if not isinstance(node_tree_manifest, dict):
        node_tree_manifest = {}
    _write_json(output_path, node_tree_manifest)
    report["static_node_tree_manifest_output"] = str(output_path)


def _build_report_for_input(
    *,
    repo_root: Path,
    robot_schema: Any,
    input_path: Path,
    normalized_output: Path,
    run_godot_smoke: bool,
    smoke_output: Path,
    godot_exe: str,
    port: int,
    timeout_seconds: float,
    max_endpoint_distance: float | None,
    max_relative_angle: float | None,
    min_body_displacement: float | None,
    max_linear_speed: float | None,
    min_joint_angle_delta: float | None,
    min_joint_angle_range: float | None,
    min_moving_joint_coverage: float | None,
    min_commanded_joint_response_coverage: float | None,
    joint_motion_epsilon: float,
    min_action_target_coverage: float | None,
    min_control_action_coverage: float | None,
    min_nonzero_action_targets: int | None,
    min_action_transitions: int | None,
    min_action_transition_delta: float | None,
    fail_on_joint_limit_violation: bool,
    fail_on_incomplete_restoration: bool,
    min_restoration_score: float | None,
    fail_on_parameter_mismatch: bool,
    fail_on_control_mismatch: bool,
    fail_on_full_mechanical_restoration: bool,
    fail_on_action_target_mismatch: bool,
    fail_on_action_sequence_target_mismatch: bool,
    fail_on_unknown_action_target: bool,
    fail_on_invalid_action_target: bool,
    fail_on_incomplete_node_tree: bool,
    fail_on_full_node_tree_restoration: bool,
    fail_on_node_tree_class_mismatch: bool,
    fail_on_node_tree_missing_parameters: bool,
    fail_on_node_tree_transform_mismatch: bool,
    fail_on_node_tree_physical_mismatch: bool,
    fail_on_node_tree_fixed_lock_mismatch: bool,
    node_tree_tolerance: float,
    parameter_tolerance: float,
    action_json: str,
    action_sequence_json: str | None,
    steps: int,
    step_delay_seconds: float,
    mechanical_trace_output: Path | None = None,
    live_profile: str = "local",
    live_artifact_root: Path | None = None,
    live_retention_days: int | None = None,
    flaky_retry_attempts: int | None = None,
) -> dict[str, Any]:
    payload = json.loads(input_path.read_text(encoding="utf-8"))
    normalized = robot_schema.normalize_robot_config_for_godot(payload)
    errors = robot_schema.validate_godot_robot_config(normalized)
    topology_summary = robot_schema.build_mechanical_topology_summary(normalized)
    node_tree_manifest = robot_schema.build_godot_node_tree_manifest(normalized)
    node_tree_manifest_errors = (
        robot_schema.validate_godot_node_tree_manifest(node_tree_manifest)
    )
    node_tree_manifest_path_map_mismatches = (
        robot_schema.build_godot_node_tree_manifest_path_map_mismatches(
            node_tree_manifest
        )
    )

    _write_json(normalized_output, normalized)

    static_report = _build_static_report(
        input_path=input_path,
        normalized=normalized,
        errors=[*errors, *node_tree_manifest_errors],
        topology_summary=topology_summary,
        node_tree_manifest=node_tree_manifest,
        node_tree_manifest_errors=node_tree_manifest_errors,
        node_tree_manifest_path_map_mismatches=(
            node_tree_manifest_path_map_mismatches
        ),
    )
    report: dict[str, Any] = {
        "status": static_report["status"],
        "static": static_report,
        "normalized_output": str(normalized_output),
        "delivery_contract_preview": _build_delivery_contract_preview(static_report),
        "godot_smoke": None,
        "godot_smoke_skipped_reason": None,
    }

    if run_godot_smoke and not errors:
        smoke_result = _run_godot_smoke(
            repo_root=repo_root,
            normalized_output=normalized_output,
            smoke_output=smoke_output,
            godot_exe=godot_exe,
            live_profile=live_profile,
            live_artifact_root=live_artifact_root,
            live_retention_days=live_retention_days,
            flaky_retry_attempts=flaky_retry_attempts,
            port=port,
            timeout_seconds=timeout_seconds,
            max_endpoint_distance=max_endpoint_distance,
            max_relative_angle=max_relative_angle,
            min_body_displacement=min_body_displacement,
            max_linear_speed=max_linear_speed,
            min_joint_angle_delta=min_joint_angle_delta,
            min_joint_angle_range=min_joint_angle_range,
            min_moving_joint_coverage=min_moving_joint_coverage,
            min_commanded_joint_response_coverage=min_commanded_joint_response_coverage,
            joint_motion_epsilon=joint_motion_epsilon,
            min_action_target_coverage=min_action_target_coverage,
            min_control_action_coverage=min_control_action_coverage,
            min_nonzero_action_targets=min_nonzero_action_targets,
            min_action_transitions=min_action_transitions,
            min_action_transition_delta=min_action_transition_delta,
            fail_on_joint_limit_violation=fail_on_joint_limit_violation,
            fail_on_incomplete_restoration=fail_on_incomplete_restoration,
            min_restoration_score=min_restoration_score,
            fail_on_parameter_mismatch=fail_on_parameter_mismatch,
            fail_on_control_mismatch=fail_on_control_mismatch,
            fail_on_full_mechanical_restoration=fail_on_full_mechanical_restoration,
            fail_on_action_target_mismatch=fail_on_action_target_mismatch,
            fail_on_action_sequence_target_mismatch=fail_on_action_sequence_target_mismatch,
            fail_on_unknown_action_target=fail_on_unknown_action_target,
            fail_on_invalid_action_target=fail_on_invalid_action_target,
            fail_on_incomplete_node_tree=fail_on_incomplete_node_tree,
            fail_on_full_node_tree_restoration=fail_on_full_node_tree_restoration,
            fail_on_node_tree_class_mismatch=fail_on_node_tree_class_mismatch,
            fail_on_node_tree_missing_parameters=fail_on_node_tree_missing_parameters,
            fail_on_node_tree_transform_mismatch=fail_on_node_tree_transform_mismatch,
            fail_on_node_tree_physical_mismatch=fail_on_node_tree_physical_mismatch,
            fail_on_node_tree_fixed_lock_mismatch=fail_on_node_tree_fixed_lock_mismatch,
            node_tree_tolerance=node_tree_tolerance,
            parameter_tolerance=parameter_tolerance,
            action_json=action_json,
            action_sequence_json=action_sequence_json,
            steps=steps,
            step_delay_seconds=step_delay_seconds,
            mechanical_trace_output=mechanical_trace_output,
        )
        report["godot_smoke"] = smoke_result
        smoke_report, _report_read_error = _read_optional_json_object(smoke_output)
        report["delivery_contract_preview"] = _build_live_delivery_summary(smoke_report)
        if smoke_result["returncode"] != 0:
            report["status"] = "error"
    elif run_godot_smoke and errors:
        report["godot_smoke_skipped_reason"] = (
            "validation_failed: Godot smoke was not launched because the normalized "
            "robot config does not satisfy the dynamic assembly contract."
        )

    return report


def _build_batch_summary(
    reports: list[dict[str, Any]],
    *,
    require_full_mechanical_restoration_gate: bool = False,
    acceptance_profile: str = "custom",
    acceptance_requirements: dict[str, bool] | None = None,
) -> dict[str, Any]:
    live_smoke_count = sum(1 for report in reports if report.get("godot_smoke"))
    robot_summaries = [_build_batch_robot_summary(report) for report in reports]
    delivery_summaries = [
        report.get("delivery_contract_preview", {})
        for report in reports
    ]
    parameter_summaries = [
        _batch_smoke_nested(report, ["joint_parameter_consistency_summary"])
        for report in reports
    ]
    control_summaries = [
        _batch_smoke_nested(report, ["joint_control_consistency_summary"])
        for report in reports
    ]
    runtime_control_summaries = [
        _batch_smoke_nested(report, ["joint_control_summary"])
        for report in reports
    ]
    action_target_summaries = [
        _batch_smoke_nested(report, ["action_target_consistency_summary"])
        for report in reports
    ]
    action_sequence_target_summaries = [
        _batch_smoke_nested(report, ["action_sequence_target_consistency_summary"])
        for report in reports
    ]
    action_coverage_summaries = [
        _batch_smoke_nested(report, ["action_target_coverage_summary"])
        for report in reports
    ]
    control_action_coverage_summaries = [
        _batch_smoke_nested(report, ["control_action_coverage_summary"])
        for report in reports
    ]
    joint_motion_summaries = [
        _batch_smoke_nested(report, ["joint_motion_summary"])
        for report in reports
    ]
    action_sequence_summaries = [
        _batch_smoke_nested(report, ["action_sequence_summary"])
        for report in reports
    ]
    restoration_summaries = [
        _batch_smoke_nested(report, ["mechanical_restoration_summary"])
        for report in reports
    ]
    mechanical_gate_summaries = [
        _batch_smoke_nested(report, ["mechanical_gate_summary"])
        for report in reports
    ]
    mechanical_behavior_summaries = [
        _batch_smoke_nested(report, ["mechanical_behavior_evidence"])
        for report in reports
    ]
    node_tree_manifests = [
        _batch_smoke_nested(report, ["node_tree_manifest"])
        for report in reports
    ]
    node_tree_gate_summaries = [
        _batch_smoke_nested(report, ["node_tree_gate_summary"])
        for report in reports
    ]
    static_topology_summaries = [
        report.get("static", {}).get("topology_summary")
        for report in reports
    ]
    static_node_tree_manifests = [
        report.get("static", {}).get("node_tree_manifest")
        for report in reports
    ]
    static_node_tree_manifest_mismatches = [
        mismatch
        for report in reports
        for mismatch in report.get("static", {}).get(
            "node_tree_manifest_path_map_mismatches", []
        )
        if isinstance(mismatch, dict)
    ]
    static_node_tree_manifest_outputs = _unique_strings(
        report.get("static_node_tree_manifest_output")
        for report in reports
        if isinstance(report.get("static_node_tree_manifest_output"), str)
    )
    return {
        "inputs_count": len(reports),
        "success_count": sum(1 for report in reports if report["status"] == "success"),
        "error_count": sum(1 for report in reports if report["status"] != "success"),
        "live_smoke_count": live_smoke_count,
        "smoke_report_written_count": sum(
            1
            for report in reports
            if isinstance(report.get("godot_smoke"), dict)
            and report["godot_smoke"].get("report_written") is True
        ),
        "smoke_report_missing_count": sum(
            1
            for report in reports
            if isinstance(report.get("godot_smoke"), dict)
            and report["godot_smoke"].get("report_written") is False
        ),
        "smoke_report_read_error_count": sum(
            1
            for report in reports
            if isinstance(report.get("godot_smoke"), dict)
            and bool(report["godot_smoke"].get("report_read_error"))
        ),
        "delivery_godot_verified_count": sum(
            1
            for summary in delivery_summaries
            if isinstance(summary, dict) and summary.get("source") == "godot_smoke"
        ),
        "delivery_static_only_count": sum(
            1
            for summary in delivery_summaries
            if isinstance(summary, dict)
            and summary.get("source") == "static_normalization"
        ),
        "delivery_unverified_count": sum(
            1
            for summary in delivery_summaries
            if not isinstance(summary, dict)
            or summary.get("source")
            not in {"godot_smoke", "static_normalization"}
        ),
        "delivery_dynamic_generation_count": sum(
            1
            for summary in delivery_summaries
            if isinstance(summary, dict)
            and summary.get("dynamic_robot_generation") is True
        ),
        "delivery_complete_count": sum(
            1
            for summary in delivery_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "delivery_incomplete_count": sum(
            1
            for summary in delivery_summaries
            if isinstance(summary, dict) and summary.get("complete") is False
        ),
        "delivery_parameters_incomplete_count": sum(
            1
            for summary in delivery_summaries
            if isinstance(summary, dict) and summary.get("parameters_complete") is False
        ),
        "parameter_mismatch_count": sum(
            int(summary.get("mismatch_count") or 0)
            for summary in parameter_summaries
            if isinstance(summary, dict)
        ),
        "fixed_lock_checked_count": sum(
            int(summary.get("fixed_lock_checked_count") or 0)
            for summary in parameter_summaries
            if isinstance(summary, dict)
        ),
        "fixed_lock_mismatch_count": sum(
            int(summary.get("fixed_lock_mismatch_count") or 0)
            for summary in parameter_summaries
            if isinstance(summary, dict)
        ),
        "parameter_consistency_complete_count": sum(
            1
            for summary in parameter_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "control_mismatch_count": sum(
            int(summary.get("mismatch_count") or 0)
            for summary in control_summaries
            if isinstance(summary, dict)
        ),
        "control_configured_count": sum(
            int(summary.get("configured_count") or 0)
            for summary in control_summaries
            if isinstance(summary, dict)
        ),
        "control_readback_checked_count": sum(
            int(summary.get("checked_count") or 0)
            for summary in control_summaries
            if isinstance(summary, dict)
        ),
        "control_readback_missing_count": sum(
            int(summary.get("missing_count") or 0)
            for summary in control_summaries
            if isinstance(summary, dict)
        ),
        "control_consistency_complete_count": sum(
            1
            for summary in control_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "nonzero_action_targets_under_min_count": sum(
            1
            for summary in runtime_control_summaries
            if isinstance(summary, dict) and summary.get("nonzero_targets_under_min") is True
        ),
        "action_target_mismatch_count": sum(
            int(summary.get("mismatch_count") or 0)
            for summary in action_target_summaries
            if isinstance(summary, dict)
        ),
        "unknown_action_target_count": sum(
            _effective_unknown_action_target_count(action_summary, sequence_summary)
            for action_summary, sequence_summary in zip(
                action_target_summaries,
                action_sequence_target_summaries,
            )
        ),
        "invalid_action_target_count": sum(
            _effective_invalid_action_target_count(action_summary, sequence_summary)
            for action_summary, sequence_summary in zip(
                action_target_summaries,
                action_sequence_target_summaries,
            )
        ),
        "action_target_consistency_complete_count": sum(
            1
            for summary in action_target_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "action_sequence_target_mismatch_count": sum(
            int(summary.get("mismatch_count") or 0)
            for summary in action_sequence_target_summaries
            if isinstance(summary, dict)
        ),
        "action_sequence_target_consistency_complete_count": sum(
            1
            for summary in action_sequence_target_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "action_target_coverage_under_min_count": sum(
            1
            for summary in action_coverage_summaries
            if isinstance(summary, dict) and summary.get("coverage_under_min") is True
        ),
        "action_target_coverage_complete_count": sum(
            1
            for summary in action_coverage_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "control_action_coverage_under_min_count": sum(
            1
            for summary in control_action_coverage_summaries
            if isinstance(summary, dict) and summary.get("coverage_under_min") is True
        ),
        "control_action_coverage_complete_count": sum(
            1
            for summary in control_action_coverage_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "joint_angle_delta_under_min_count": sum(
            1
            for summary in joint_motion_summaries
            if isinstance(summary, dict) and summary.get("angle_delta_under_min") is True
        ),
        "joint_angle_range_under_min_count": sum(
            1
            for summary in joint_motion_summaries
            if isinstance(summary, dict) and summary.get("angle_range_under_min") is True
        ),
        "moving_joint_coverage_under_min_count": sum(
            1
            for summary in joint_motion_summaries
            if isinstance(summary, dict) and summary.get("moving_joint_coverage_under_min") is True
        ),
        "commanded_joint_response_under_min_count": sum(
            1
            for summary in joint_motion_summaries
            if isinstance(summary, dict) and summary.get("commanded_joint_response_under_min") is True
        ),
        "commanded_static_joint_count": sum(
            len(summary.get("commanded_static_joints", []))
            for summary in joint_motion_summaries
            if isinstance(summary, dict)
        ),
        "action_transitions_under_min_count": sum(
            1
            for summary in action_sequence_summaries
            if isinstance(summary, dict) and summary.get("transitions_under_min") is True
        ),
        "action_transition_delta_under_min_count": sum(
            1
            for summary in action_sequence_summaries
            if isinstance(summary, dict) and summary.get("transition_delta_under_min") is True
        ),
        "restoration_complete_count": sum(
            1
            for summary in restoration_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "restoration_incomplete_count": sum(
            1
            for summary in restoration_summaries
            if isinstance(summary, dict) and summary.get("complete") is False
        ),
        "mechanical_gate_enabled_count": sum(
            int(summary.get("enabled_count") or 0)
            for summary in mechanical_gate_summaries
            if isinstance(summary, dict)
        ),
        "full_mechanical_restoration_required_count": sum(
            1
            for summary in mechanical_gate_summaries
            if isinstance(summary, dict)
            and summary.get("full_mechanical_restoration_required") is True
        ),
        "full_mechanical_restoration_not_required_count": sum(
            1
            for summary in mechanical_gate_summaries
            if isinstance(summary, dict)
            and summary.get("full_mechanical_restoration_required") is False
        ),
        "mechanical_gate_check_counts": _mechanical_gate_check_counts(
            mechanical_gate_summaries
        ),
        "mechanical_behavior_evidence_count": sum(
            1
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict) and bool(summary.get("evidence_version"))
        ),
        "mechanical_behavior_complete_count": sum(
            1
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict) and summary.get("complete") is True
        ),
        "mechanical_behavior_incomplete_count": sum(
            1
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict) and summary.get("complete") is False
        ),
        "mechanical_behavior_residual_risk_count": sum(
            len(summary.get("residual_risks", []))
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict) and isinstance(summary.get("residual_risks"), list)
        ),
        "mechanical_behavior_threshold_failure_count": sum(
            len(summary.get("threshold_failures", []))
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict)
            and isinstance(summary.get("threshold_failures"), list)
        ),
        "mechanical_behavior_center_of_mass_available_count": sum(
            1
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict)
            and isinstance(summary.get("center_of_mass_evidence"), dict)
            and summary["center_of_mass_evidence"].get("available") is True
        ),
        "mechanical_behavior_contact_state_available_count": sum(
            1
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict)
            and isinstance(summary.get("contact_state_evidence"), dict)
            and summary["contact_state_evidence"].get("available") is True
        ),
        "mechanical_behavior_step_trace_artifact_count": sum(
            1
            for summary in mechanical_behavior_summaries
            if isinstance(summary, dict)
            and isinstance(summary.get("step_trace_evidence"), dict)
            and (
                summary["step_trace_evidence"].get("artifact_written") is True
                or bool(summary["step_trace_evidence"].get("artifact_path"))
            )
        ),
        "node_tree_complete_count": sum(
            1
            for manifest in node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("complete") is True
        ),
        "node_tree_incomplete_count": sum(
            1
            for manifest in node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("complete") is False
        ),
        "node_tree_missing_parts_count": sum(
            len(manifest.get("missing_part_ids", []))
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_missing_joints_count": sum(
            len(manifest.get("missing_connection_names", []))
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_class_mismatch_count": sum(
            int(manifest.get("class_mismatch_count") or 0)
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_parameter_missing_count": sum(
            int(manifest.get("parameter_missing_count") or 0)
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_transform_mismatch_count": sum(
            int(manifest.get("transform_mismatch_count") or 0)
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_physical_mismatch_count": sum(
            int(manifest.get("physical_mismatch_count") or 0)
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_fixed_lock_checked_count": sum(
            int(manifest.get("fixed_lock_checked_count") or 0)
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_fixed_lock_mismatch_count": sum(
            int(manifest.get("fixed_lock_mismatch_count") or 0)
            for manifest in node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "node_tree_fixed_locks_complete_count": sum(
            1
            for manifest in node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("fixed_locks_complete") is True
        ),
        "node_tree_fixed_locks_incomplete_count": sum(
            1
            for manifest in node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("fixed_locks_complete") is False
        ),
        "node_tree_gate_enabled_count": sum(
            int(summary.get("enabled_count") or 0)
            for summary in node_tree_gate_summaries
            if isinstance(summary, dict)
        ),
        "node_tree_full_restoration_required_count": sum(
            1
            for summary in node_tree_gate_summaries
            if isinstance(summary, dict)
            and summary.get("full_node_tree_restoration_required") is True
        ),
        "node_tree_full_restoration_not_required_count": sum(
            1
            for summary in node_tree_gate_summaries
            if isinstance(summary, dict)
            and summary.get("full_node_tree_restoration_required") is False
        ),
        "node_tree_gate_check_counts": _node_tree_gate_check_counts(
            node_tree_gate_summaries
        ),
        "static_topology_complete_count": sum(
            1
            for summary in static_topology_summaries
            if isinstance(summary, dict) and summary.get("complete_tree") is True
        ),
        "static_topology_incomplete_count": sum(
            1
            for summary in static_topology_summaries
            if isinstance(summary, dict) and summary.get("complete_tree") is False
        ),
        "static_topology_disconnected_parts_count": sum(
            len(summary.get("disconnected_parts", []))
            for summary in static_topology_summaries
            if isinstance(summary, dict)
        ),
        "static_topology_unreachable_parts_count": sum(
            len(summary.get("unreachable_parts", []))
            for summary in static_topology_summaries
            if isinstance(summary, dict)
        ),
        "static_topology_duplicate_child_endpoint_count": sum(
            len(summary.get("duplicate_child_endpoints", []))
            for summary in static_topology_summaries
            if isinstance(summary, dict)
        ),
        "static_topology_cycle_count": sum(
            1
            for summary in static_topology_summaries
            if isinstance(summary, dict) and bool(summary.get("cycle"))
        ),
        "static_topology_root_parts": _unique_strings(
            summary.get("root_part")
            for summary in static_topology_summaries
            if isinstance(summary, dict)
        ),
        "static_node_tree_manifest_count": sum(
            1 for manifest in static_node_tree_manifests if isinstance(manifest, dict)
        ),
        "static_node_tree_manifest_valid_count": sum(
            1
            for report in reports
            if isinstance(report.get("static", {}).get("node_tree_manifest"), dict)
            and report.get("static", {}).get("node_tree_manifest_errors") == []
        ),
        "static_node_tree_manifest_invalid_count": sum(
            1
            for report in reports
            if isinstance(report.get("static", {}).get("node_tree_manifest"), dict)
            and bool(report.get("static", {}).get("node_tree_manifest_errors"))
        ),
        "static_node_tree_manifest_error_count": sum(
            len(report.get("static", {}).get("node_tree_manifest_errors", []))
            for report in reports
            if isinstance(
                report.get("static", {}).get("node_tree_manifest_errors"), list
            )
        ),
        "static_node_tree_manifest_output_count": len(
            static_node_tree_manifest_outputs
        ),
        "static_node_tree_manifest_outputs": static_node_tree_manifest_outputs,
        "static_node_tree_manifest_path_map_mismatch_count": len(
            static_node_tree_manifest_mismatches
        ),
        "static_node_tree_manifest_path_map_mismatch_kind_counts": (
            _node_tree_manifest_path_map_mismatch_kind_counts(
                static_node_tree_manifest_mismatches
            )
        ),
        "static_node_tree_manifest_path_map_mismatches": _slice_list(
            static_node_tree_manifest_mismatches, 20
        ),
        "static_node_tree_parts_planned_count": sum(
            int(manifest.get("parts_count") or 0)
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "static_node_tree_joints_planned_count": sum(
            int(manifest.get("joints_count") or 0)
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "static_node_tree_parameterized_joints_count": sum(
            int(manifest.get("parameterized_joints") or 0)
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "static_node_tree_complete_count": sum(
            1
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("complete") is True
        ),
        "static_node_tree_incomplete_count": sum(
            1
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("complete") is False
        ),
        "static_node_tree_endpoint_paths_complete_count": sum(
            1
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
            and manifest.get("endpoint_paths_complete") is True
        ),
        "static_node_tree_endpoint_paths_incomplete_count": sum(
            1
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
            and manifest.get("endpoint_paths_complete") is False
        ),
        "static_node_tree_missing_endpoint_parts_count": sum(
            len(manifest.get("missing_endpoint_part_ids", []))
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "static_node_tree_missing_endpoint_connections_count": sum(
            len(manifest.get("missing_endpoint_connection_names", []))
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
        ),
        "static_node_tree_missing_endpoint_part_ids": _unique_strings(
            part_id
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
            for part_id in manifest.get("missing_endpoint_part_ids", [])
        ),
        "static_node_tree_missing_endpoint_connection_names": _unique_strings(
            connection_name
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict)
            for connection_name in manifest.get(
                "missing_endpoint_connection_names", []
            )
        ),
        "static_node_tree_missing_endpoint_details": _slice_list(
            [
                detail
                for manifest in static_node_tree_manifests
                if isinstance(manifest, dict)
                for detail in manifest.get("missing_endpoint_details", [])
                if isinstance(detail, dict)
            ],
            20,
        ),
        "static_node_tree_parameters_complete_count": sum(
            1
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("parameters_complete") is True
        ),
        "static_node_tree_parameters_incomplete_count": sum(
            1
            for manifest in static_node_tree_manifests
            if isinstance(manifest, dict) and manifest.get("parameters_complete") is False
        ),
        "failure_reasons_count": sum(
            len(summary.get("failure_reasons", []))
            for summary in robot_summaries
        ),
        "require_full_mechanical_restoration_gate": (
            require_full_mechanical_restoration_gate
        ),
        "acceptance_profile": acceptance_profile,
        "acceptance_requirements": acceptance_requirements or {},
        **_build_delivery_acceptance_summary(
            reports,
            delivery_summaries,
            robot_summaries,
            live_smoke_count,
            require_full_mechanical_restoration_gate=require_full_mechanical_restoration_gate,
            require_static_node_tree_complete=bool(
                (acceptance_requirements or {}).get("static_node_tree_complete")
            ),
            require_static_node_tree_manifest_output=bool(
                (acceptance_requirements or {}).get(
                    "static_node_tree_manifest_output"
                )
            ),
        ),
        "delivery_failure_robots": _build_delivery_failure_robots(robot_summaries),
        "commanded_response_failure_robots": _build_commanded_response_failure_robots(
            robot_summaries
        ),
        "robots": robot_summaries,
    }


def _node_tree_gate_check_counts(
    gate_summaries: list[Any],
) -> dict[str, int]:
    counts = {
        "incomplete_node_tree": 0,
        "class_mismatch": 0,
        "missing_parameters": 0,
        "transform_mismatch": 0,
        "physical_mismatch": 0,
        "fixed_lock_mismatch": 0,
    }
    for summary in gate_summaries:
        if not isinstance(summary, dict):
            continue
        checks = summary.get("checks", {})
        if not isinstance(checks, dict):
            continue
        for key in counts:
            if checks.get(key) is True:
                counts[key] += 1
    return counts


def _mechanical_gate_check_counts(
    gate_summaries: list[Any],
) -> dict[str, int]:
    counts = {
        "mechanical_restoration": 0,
        "joint_parameter_readback": 0,
        "control_parameter_readback": 0,
        "full_node_tree_restoration": 0,
    }
    for summary in gate_summaries:
        if not isinstance(summary, dict):
            continue
        checks = summary.get("checks", {})
        if not isinstance(checks, dict):
            continue
        for key in counts:
            if checks.get(key) is True:
                counts[key] += 1
    return counts


def _build_delivery_acceptance_summary(
    reports: list[dict[str, Any]],
    delivery_summaries: list[Any],
    robot_summaries: list[dict[str, Any]],
    live_smoke_count: int,
    *,
    require_full_mechanical_restoration_gate: bool = False,
    require_static_node_tree_complete: bool = False,
    require_static_node_tree_manifest_output: bool = False,
) -> dict[str, Any]:
    inputs_count = len(reports)
    error_count = sum(1 for report in reports if report["status"] != "success")
    complete_count = sum(
        1
        for summary in delivery_summaries
        if isinstance(summary, dict) and summary.get("complete") is True
    )
    incomplete_count = sum(
        1
        for summary in delivery_summaries
        if isinstance(summary, dict) and summary.get("complete") is False
    )
    parameters_incomplete_count = sum(
        1
        for summary in delivery_summaries
        if isinstance(summary, dict) and summary.get("parameters_complete") is False
    )
    godot_verified_count = sum(
        1
        for summary in delivery_summaries
        if isinstance(summary, dict) and summary.get("source") == "godot_smoke"
    )
    static_only_count = sum(
        1
        for summary in delivery_summaries
        if isinstance(summary, dict) and summary.get("source") == "static_normalization"
    )
    unverified_count = sum(
        1
        for summary in delivery_summaries
        if not isinstance(summary, dict)
        or summary.get("source") not in {"godot_smoke", "static_normalization"}
    )
    dynamic_generation_count = sum(
        1
        for summary in delivery_summaries
        if isinstance(summary, dict)
        and summary.get("dynamic_robot_generation") is True
    )
    failure_reasons_count = sum(
        len(summary.get("failure_reasons", [])) for summary in robot_summaries
    )
    node_tree_fixed_lock_mismatch_count = sum(
        int(summary.get("node_tree_fixed_lock_mismatch_count") or 0)
        for summary in robot_summaries
    )
    control_readback_missing_count = sum(
        int(summary.get("control_readback_missing_count") or 0)
        for summary in robot_summaries
    )
    full_mechanical_gate_missing_count = (
        sum(
            1
            for summary in robot_summaries
            if summary.get("full_mechanical_restoration_required") is not True
        )
        if require_full_mechanical_restoration_gate
        else 0
    )
    static_node_tree_manifest_output_missing_count = (
        sum(
            1
            for summary in robot_summaries
            if not summary.get("static_node_tree_manifest_output")
        )
        if require_static_node_tree_manifest_output
        else 0
    )
    static_node_tree_incomplete_count = (
        sum(
            1
            for summary in robot_summaries
            if summary.get("static_node_tree_complete") is not True
        )
        if require_static_node_tree_complete
        else 0
    )
    smoke_report_missing_count = sum(
        1
        for report in reports
        if isinstance(report.get("godot_smoke"), dict)
        and report["godot_smoke"].get("report_written") is False
    )
    smoke_report_read_error_count = sum(
        1
        for report in reports
        if isinstance(report.get("godot_smoke"), dict)
        and bool(report["godot_smoke"].get("report_read_error"))
    )
    all_inputs = [_report_input(report) for report in reports]
    error_inputs = [
        _report_input(report) for report in reports if report["status"] != "success"
    ]
    missing_smoke_inputs = [
        _report_input(report) for report in reports if not report.get("godot_smoke")
    ]
    static_only_inputs = [
        _report_input(report)
        for report, summary in zip(reports, delivery_summaries)
        if isinstance(summary, dict) and summary.get("source") == "static_normalization"
    ]
    unverified_inputs = [
        _report_input(report)
        for report, summary in zip(reports, delivery_summaries)
        if not isinstance(summary, dict)
        or summary.get("source") not in {"godot_smoke", "static_normalization"}
    ]
    missing_dynamic_inputs = [
        _report_input(report)
        for report, summary in zip(reports, delivery_summaries)
        if not isinstance(summary, dict)
        or summary.get("dynamic_robot_generation") is not True
    ]
    incomplete_inputs = [
        _report_input(report)
        for report, summary in zip(reports, delivery_summaries)
        if isinstance(summary, dict) and summary.get("complete") is False
    ]
    parameters_incomplete_inputs = [
        _report_input(report)
        for report, summary in zip(reports, delivery_summaries)
        if isinstance(summary, dict) and summary.get("parameters_complete") is False
    ]
    failure_reason_inputs = [
        str(summary.get("input"))
        for summary in robot_summaries
        if summary.get("input") and summary.get("failure_reasons")
    ]
    node_tree_fixed_lock_mismatch_inputs = [
        str(summary.get("input"))
        for summary in robot_summaries
        if summary.get("input")
        and int(summary.get("node_tree_fixed_lock_mismatch_count") or 0) > 0
    ]
    control_readback_missing_inputs = [
        str(summary.get("input"))
        for summary in robot_summaries
        if summary.get("input")
        and int(summary.get("control_readback_missing_count") or 0) > 0
    ]
    full_mechanical_gate_missing_inputs = [
        str(summary.get("input"))
        for summary in robot_summaries
        if summary.get("input")
        and summary.get("full_mechanical_restoration_required") is not True
    ]
    static_node_tree_manifest_output_missing_inputs = [
        str(summary.get("input"))
        for summary in robot_summaries
        if summary.get("input") and not summary.get("static_node_tree_manifest_output")
    ]
    static_node_tree_incomplete_inputs = [
        str(summary.get("input"))
        for summary in robot_summaries
        if summary.get("input") and summary.get("static_node_tree_complete") is not True
    ]
    smoke_report_missing_inputs = [
        _report_input(report)
        for report in reports
        if isinstance(report.get("godot_smoke"), dict)
        and report["godot_smoke"].get("report_written") is False
    ]
    smoke_report_read_error_inputs = [
        _report_input(report)
        for report in reports
        if isinstance(report.get("godot_smoke"), dict)
        and bool(report["godot_smoke"].get("report_read_error"))
    ]

    acceptance_complete = (
        inputs_count > 0
        and error_count == 0
        and live_smoke_count == inputs_count
        and godot_verified_count == inputs_count
        and dynamic_generation_count == inputs_count
        and complete_count == inputs_count
        and incomplete_count == 0
        and parameters_incomplete_count == 0
        and node_tree_fixed_lock_mismatch_count == 0
        and control_readback_missing_count == 0
        and full_mechanical_gate_missing_count == 0
        and static_node_tree_manifest_output_missing_count == 0
        and static_node_tree_incomplete_count == 0
        and failure_reasons_count == 0
    )
    if acceptance_complete:
        level = "godot_verified"
    elif (
        inputs_count > 0
        and error_count == 0
        and static_only_count == inputs_count
        and dynamic_generation_count == inputs_count
        and complete_count == inputs_count
        and incomplete_count == 0
    ):
        level = "static_only"
    else:
        level = "incomplete"

    reasons: list[str] = []
    reason_details: list[dict[str, Any]] = []
    if inputs_count == 0:
        message = "no robot inputs were provided"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="no_inputs",
                count=inputs_count,
                message=message,
                inputs=all_inputs,
            )
        )
    if error_count:
        message = f"{error_count} robot(s) reported errors"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="robot_errors",
                count=error_count,
                message=message,
                inputs=error_inputs,
            )
        )
    if live_smoke_count < inputs_count:
        missing = inputs_count - live_smoke_count
        message = f"{missing} robot(s) were not run through Godot smoke"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="missing_godot_smoke",
                count=missing,
                message=message,
                inputs=missing_smoke_inputs,
            )
        )
    if smoke_report_missing_count:
        message = f"{smoke_report_missing_count} Godot smoke report(s) were not written"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="missing_godot_smoke_report",
                count=smoke_report_missing_count,
                message=message,
                inputs=smoke_report_missing_inputs,
            )
        )
    if smoke_report_read_error_count:
        message = (
            f"{smoke_report_read_error_count} Godot smoke report(s) could not be read"
        )
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="invalid_godot_smoke_report",
                count=smoke_report_read_error_count,
                message=message,
                inputs=smoke_report_read_error_inputs,
            )
        )
    if static_only_count:
        message = f"{static_only_count} robot(s) only passed static normalization"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="static_only",
                count=static_only_count,
                message=message,
                inputs=static_only_inputs,
            )
        )
    if unverified_count:
        message = f"{unverified_count} robot(s) have unknown delivery provenance"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="unknown_delivery_provenance",
                count=unverified_count,
                message=message,
                inputs=unverified_inputs,
            )
        )
    if dynamic_generation_count < inputs_count:
        missing = inputs_count - dynamic_generation_count
        message = f"{missing} robot(s) did not report dynamic robot generation"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="missing_dynamic_generation",
                count=missing,
                message=message,
                inputs=missing_dynamic_inputs,
            )
        )
    if incomplete_count:
        message = f"{incomplete_count} robot(s) have incomplete delivery"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="incomplete_delivery",
                count=incomplete_count,
                message=message,
                inputs=incomplete_inputs,
            )
        )
    if parameters_incomplete_count:
        message = (
            f"{parameters_incomplete_count} robot(s) have incomplete joint parameters"
        )
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="incomplete_joint_parameters",
                count=parameters_incomplete_count,
                message=message,
                inputs=parameters_incomplete_inputs,
            )
        )
    if node_tree_fixed_lock_mismatch_count:
        message = (
            f"{node_tree_fixed_lock_mismatch_count} node-tree fixed joint lock "
            "mismatch(es) were reported"
        )
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="node_tree_fixed_lock_mismatch",
                count=node_tree_fixed_lock_mismatch_count,
                message=message,
                inputs=node_tree_fixed_lock_mismatch_inputs,
            )
        )
    if control_readback_missing_count:
        message = (
            f"{control_readback_missing_count} control readback metadata item(s) "
            "were missing"
        )
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="control_readback_missing",
                count=control_readback_missing_count,
                message=message,
                inputs=control_readback_missing_inputs,
            )
        )
    if full_mechanical_gate_missing_count:
        message = (
            f"{full_mechanical_gate_missing_count} robot(s) did not require "
            "full mechanical restoration gate"
        )
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="missing_full_mechanical_restoration_gate",
                count=full_mechanical_gate_missing_count,
                message=message,
                inputs=full_mechanical_gate_missing_inputs,
            )
        )
    if static_node_tree_manifest_output_missing_count:
        message = (
            f"{static_node_tree_manifest_output_missing_count} robot(s) did not "
            "write a static Godot node-tree manifest artifact"
        )
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="missing_static_node_tree_manifest_output",
                count=static_node_tree_manifest_output_missing_count,
                message=message,
                inputs=static_node_tree_manifest_output_missing_inputs,
            )
        )
    if static_node_tree_incomplete_count:
        message = (
            f"{static_node_tree_incomplete_count} robot(s) have incomplete static "
            "Godot node-tree manifests"
        )
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="static_node_tree_incomplete",
                count=static_node_tree_incomplete_count,
                message=message,
                inputs=static_node_tree_incomplete_inputs,
            )
        )
    if failure_reasons_count:
        message = f"{failure_reasons_count} smoke failure reason(s) were reported"
        reasons.append(message)
        reason_details.append(
            _acceptance_reason_detail(
                code="smoke_failure_reasons",
                count=failure_reasons_count,
                message=message,
                inputs=failure_reason_inputs,
            )
        )

    return {
        "delivery_acceptance_complete": acceptance_complete,
        "delivery_acceptance_level": level,
        "delivery_acceptance_reasons": reasons,
        "delivery_acceptance_reason_codes": [
            detail["code"] for detail in reason_details
        ],
        "delivery_acceptance_reason_details": reason_details,
    }


def _build_delivery_acceptance_gate(
    *,
    required: bool,
    report_status: str,
    batch_summary: dict[str, Any],
) -> dict[str, Any]:
    acceptance_complete = batch_summary["delivery_acceptance_complete"]
    requires_full_mechanical_gate = batch_summary.get(
        "require_full_mechanical_restoration_gate", False
    )
    exit_code = 0
    if report_status != "success":
        exit_code = 1
    elif (required or requires_full_mechanical_gate) and not acceptance_complete:
        exit_code = 1
    elif (
        batch_summary.get("acceptance_requirements", {}).get(
            "static_node_tree_manifest_output"
        )
        and "missing_static_node_tree_manifest_output"
        in batch_summary.get("delivery_acceptance_reason_codes", [])
    ):
        exit_code = 1
    elif (
        batch_summary.get("acceptance_requirements", {}).get(
            "static_node_tree_complete"
        )
        and "static_node_tree_incomplete"
        in batch_summary.get("delivery_acceptance_reason_codes", [])
    ):
        exit_code = 1

    return {
        "contract_version": DELIVERY_ACCEPTANCE_GATE_CONTRACT_VERSION,
        "source": DELIVERY_ACCEPTANCE_GATE_SOURCE,
        "verification_scope": DELIVERY_ACCEPTANCE_GATE_SCOPE,
        "required": required,
        "requires_full_mechanical_restoration_gate": requires_full_mechanical_gate,
        "acceptance_profile": batch_summary.get("acceptance_profile", "custom"),
        "acceptance_requirements": batch_summary.get("acceptance_requirements", {}),
        "passed": exit_code == 0,
        "exit_code": exit_code,
        "level": batch_summary["delivery_acceptance_level"],
        "complete": acceptance_complete,
        "reasons": batch_summary["delivery_acceptance_reasons"],
        "reason_codes": batch_summary["delivery_acceptance_reason_codes"],
        "reason_details": batch_summary["delivery_acceptance_reason_details"],
        "summary_counts": {
            "inputs_count": batch_summary.get("inputs_count", 0),
            "success_count": batch_summary.get("success_count", 0),
            "error_count": batch_summary.get("error_count", 0),
            "live_smoke_count": batch_summary.get("live_smoke_count", 0),
            "smoke_report_written_count": batch_summary.get(
                "smoke_report_written_count", 0
            ),
            "smoke_report_missing_count": batch_summary.get(
                "smoke_report_missing_count", 0
            ),
            "smoke_report_read_error_count": batch_summary.get(
                "smoke_report_read_error_count", 0
            ),
            "delivery_godot_verified_count": batch_summary.get(
                "delivery_godot_verified_count", 0
            ),
            "delivery_static_only_count": batch_summary.get(
                "delivery_static_only_count", 0
            ),
            "delivery_unverified_count": batch_summary.get(
                "delivery_unverified_count", 0
            ),
            "delivery_dynamic_generation_count": batch_summary.get(
                "delivery_dynamic_generation_count", 0
            ),
            "delivery_complete_count": batch_summary.get(
                "delivery_complete_count", 0
            ),
            "delivery_incomplete_count": batch_summary.get(
                "delivery_incomplete_count", 0
            ),
            "delivery_parameters_incomplete_count": batch_summary.get(
                "delivery_parameters_incomplete_count", 0
            ),
            "fixed_lock_checked_count": batch_summary.get(
                "fixed_lock_checked_count", 0
            ),
            "fixed_lock_mismatch_count": batch_summary.get(
                "fixed_lock_mismatch_count", 0
            ),
            "control_configured_count": batch_summary.get(
                "control_configured_count", 0
            ),
            "control_readback_checked_count": batch_summary.get(
                "control_readback_checked_count", 0
            ),
            "control_readback_missing_count": batch_summary.get(
                "control_readback_missing_count", 0
            ),
            "node_tree_fixed_lock_checked_count": batch_summary.get(
                "node_tree_fixed_lock_checked_count", 0
            ),
            "node_tree_fixed_lock_mismatch_count": batch_summary.get(
                "node_tree_fixed_lock_mismatch_count", 0
            ),
            "node_tree_fixed_locks_complete_count": batch_summary.get(
                "node_tree_fixed_locks_complete_count", 0
            ),
            "node_tree_fixed_locks_incomplete_count": batch_summary.get(
                "node_tree_fixed_locks_incomplete_count", 0
            ),
            "node_tree_gate_enabled_count": batch_summary.get(
                "node_tree_gate_enabled_count", 0
            ),
            "node_tree_full_restoration_required_count": batch_summary.get(
                "node_tree_full_restoration_required_count", 0
            ),
            "node_tree_full_restoration_not_required_count": batch_summary.get(
                "node_tree_full_restoration_not_required_count", 0
            ),
            "node_tree_gate_check_counts": batch_summary.get(
                "node_tree_gate_check_counts", {}
            ),
            "static_topology_complete_count": batch_summary.get(
                "static_topology_complete_count", 0
            ),
            "static_topology_incomplete_count": batch_summary.get(
                "static_topology_incomplete_count", 0
            ),
            "static_topology_disconnected_parts_count": batch_summary.get(
                "static_topology_disconnected_parts_count", 0
            ),
            "static_topology_unreachable_parts_count": batch_summary.get(
                "static_topology_unreachable_parts_count", 0
            ),
            "static_topology_duplicate_child_endpoint_count": batch_summary.get(
                "static_topology_duplicate_child_endpoint_count", 0
            ),
            "static_topology_cycle_count": batch_summary.get(
                "static_topology_cycle_count", 0
            ),
            "static_node_tree_manifest_count": batch_summary.get(
                "static_node_tree_manifest_count", 0
            ),
            "static_node_tree_manifest_valid_count": batch_summary.get(
                "static_node_tree_manifest_valid_count", 0
            ),
            "static_node_tree_manifest_invalid_count": batch_summary.get(
                "static_node_tree_manifest_invalid_count", 0
            ),
            "static_node_tree_manifest_error_count": batch_summary.get(
                "static_node_tree_manifest_error_count", 0
            ),
            "static_node_tree_manifest_output_count": batch_summary.get(
                "static_node_tree_manifest_output_count", 0
            ),
            "static_node_tree_manifest_path_map_mismatch_count": batch_summary.get(
                "static_node_tree_manifest_path_map_mismatch_count", 0
            ),
            "static_node_tree_manifest_path_map_mismatch_kind_counts": (
                batch_summary.get(
                    "static_node_tree_manifest_path_map_mismatch_kind_counts",
                    {},
                )
            ),
            "static_node_tree_parts_planned_count": batch_summary.get(
                "static_node_tree_parts_planned_count", 0
            ),
            "static_node_tree_joints_planned_count": batch_summary.get(
                "static_node_tree_joints_planned_count", 0
            ),
            "static_node_tree_parameterized_joints_count": batch_summary.get(
                "static_node_tree_parameterized_joints_count", 0
            ),
            "static_node_tree_complete_count": batch_summary.get(
                "static_node_tree_complete_count", 0
            ),
            "static_node_tree_incomplete_count": batch_summary.get(
                "static_node_tree_incomplete_count", 0
            ),
            "static_node_tree_endpoint_paths_complete_count": batch_summary.get(
                "static_node_tree_endpoint_paths_complete_count", 0
            ),
            "static_node_tree_endpoint_paths_incomplete_count": batch_summary.get(
                "static_node_tree_endpoint_paths_incomplete_count", 0
            ),
            "static_node_tree_missing_endpoint_parts_count": batch_summary.get(
                "static_node_tree_missing_endpoint_parts_count", 0
            ),
            "static_node_tree_missing_endpoint_connections_count": batch_summary.get(
                "static_node_tree_missing_endpoint_connections_count", 0
            ),
            "static_node_tree_parameters_complete_count": batch_summary.get(
                "static_node_tree_parameters_complete_count", 0
            ),
            "static_node_tree_parameters_incomplete_count": batch_summary.get(
                "static_node_tree_parameters_incomplete_count", 0
            ),
            "mechanical_gate_enabled_count": batch_summary.get(
                "mechanical_gate_enabled_count", 0
            ),
            "full_mechanical_restoration_required_count": batch_summary.get(
                "full_mechanical_restoration_required_count", 0
            ),
            "full_mechanical_restoration_not_required_count": batch_summary.get(
                "full_mechanical_restoration_not_required_count", 0
            ),
            "mechanical_gate_check_counts": batch_summary.get(
                "mechanical_gate_check_counts", {}
            ),
            "mechanical_behavior_evidence_count": batch_summary.get(
                "mechanical_behavior_evidence_count", 0
            ),
            "mechanical_behavior_complete_count": batch_summary.get(
                "mechanical_behavior_complete_count", 0
            ),
            "mechanical_behavior_incomplete_count": batch_summary.get(
                "mechanical_behavior_incomplete_count", 0
            ),
            "mechanical_behavior_residual_risk_count": batch_summary.get(
                "mechanical_behavior_residual_risk_count", 0
            ),
            "mechanical_behavior_threshold_failure_count": batch_summary.get(
                "mechanical_behavior_threshold_failure_count", 0
            ),
            "mechanical_behavior_center_of_mass_available_count": batch_summary.get(
                "mechanical_behavior_center_of_mass_available_count", 0
            ),
            "mechanical_behavior_contact_state_available_count": batch_summary.get(
                "mechanical_behavior_contact_state_available_count", 0
            ),
            "mechanical_behavior_step_trace_artifact_count": batch_summary.get(
                "mechanical_behavior_step_trace_artifact_count", 0
            ),
            "failure_reasons_count": batch_summary.get("failure_reasons_count", 0),
        },
    }


def _format_delivery_acceptance_gate_log(
    gate: dict[str, Any],
    batch_summary: dict[str, Any] | None = None,
) -> str:
    codes = gate.get("reason_codes", [])
    code_text = ",".join(str(code) for code in codes) if codes else "none"
    requirements = gate.get("acceptance_requirements", {})
    if not isinstance(requirements, dict):
        requirements = {}
    requirement_text = ",".join(
        str(name) for name, enabled in requirements.items() if enabled is True
    )
    if not requirement_text:
        requirement_text = "none"
    counts = gate.get("summary_counts", {})
    if not isinstance(counts, dict):
        counts = {}
    count_text = (
        f"inputs:{counts.get('inputs_count', 0)},"
        f"errors:{counts.get('error_count', 0)},"
        f"live:{counts.get('live_smoke_count', 0)},"
        f"smokereports:{counts.get('smoke_report_written_count', 0)}/"
        f"{counts.get('smoke_report_missing_count', 0)}/"
        f"{counts.get('smoke_report_read_error_count', 0)},"
        f"verified:{counts.get('delivery_godot_verified_count', 0)},"
        f"static:{counts.get('delivery_static_only_count', 0)},"
        f"staticmanifest:{counts.get('static_node_tree_manifest_valid_count', 0)}/"
        f"{counts.get('static_node_tree_manifest_invalid_count', 0)},"
        f"fixed:{counts.get('fixed_lock_checked_count', 0)}/"
        f"{counts.get('fixed_lock_mismatch_count', 0)},"
        f"control:{counts.get('control_readback_checked_count', 0)}/"
        f"{counts.get('control_readback_missing_count', 0)}/"
        f"{counts.get('control_configured_count', 0)},"
        f"treefixed:{counts.get('node_tree_fixed_lock_checked_count', 0)}/"
        f"{counts.get('node_tree_fixed_lock_mismatch_count', 0)},"
        f"treefixedok:{counts.get('node_tree_fixed_locks_complete_count', 0)}/"
        f"{counts.get('node_tree_fixed_locks_incomplete_count', 0)},"
        f"treegate:{counts.get('node_tree_gate_enabled_count', 0)}/"
        f"{counts.get('node_tree_full_restoration_required_count', 0)},"
        f"mechgate:{counts.get('mechanical_gate_enabled_count', 0)}/"
        f"{counts.get('full_mechanical_restoration_required_count', 0)},"
        f"mechbehavior:{counts.get('mechanical_behavior_evidence_count', 0)}/"
        f"{counts.get('mechanical_behavior_complete_count', 0)},"
        f"failures:{counts.get('failure_reasons_count', 0)}"
    )
    check_text = _format_gate_check_counts(counts)
    topology_text = _format_static_topology_counts(batch_summary or {})
    inputs: list[str] = []
    for detail in gate.get("reason_details", []):
        if not isinstance(detail, dict):
            continue
        for input_path in detail.get("inputs", []):
            input_text = str(input_path)
            if input_text not in inputs:
                inputs.append(input_text)
    input_text = ",".join(inputs[:5]) if inputs else "none"
    if len(inputs) > 5:
        input_text = f"{input_text},..."
    return (
        "delivery_acceptance_gate failed "
        f"contract_version={gate.get('contract_version')} "
        f"source={gate.get('source')} "
        f"verification_scope={gate.get('verification_scope')} "
        f"acceptance_profile={gate.get('acceptance_profile', 'custom')} "
        f"requirements={requirement_text} "
        f"exit_code={gate.get('exit_code')} "
        f"required={str(gate.get('required')).lower()} "
        f"level={gate.get('level')} "
        f"complete={str(gate.get('complete')).lower()} "
        f"counts={count_text} "
        f"checks={check_text} "
        f"topology={topology_text} "
        f"reason_codes={code_text} "
        f"affected_inputs={input_text}"
    )


def _format_static_topology_counts(batch_summary: dict[str, Any]) -> str:
    roots = batch_summary.get("static_topology_root_parts", [])
    if isinstance(roots, list) and roots:
        root_text = ",".join(str(root) for root in roots[:5])
        if len(roots) > 5:
            root_text = f"{root_text},..."
    else:
        root_text = "none"
    return (
        f"complete:{batch_summary.get('static_topology_complete_count', 0)}/"
        f"{batch_summary.get('inputs_count', 0)},"
        f"incomplete:{batch_summary.get('static_topology_incomplete_count', 0)},"
        f"disconnected:{batch_summary.get('static_topology_disconnected_parts_count', 0)},"
        f"unreachable:{batch_summary.get('static_topology_unreachable_parts_count', 0)},"
        f"duplicates:{batch_summary.get('static_topology_duplicate_child_endpoint_count', 0)},"
        f"cycles:{batch_summary.get('static_topology_cycle_count', 0)},"
        f"roots:{root_text}"
    )


def _format_gate_check_counts(counts: dict[str, Any]) -> str:
    groups = [
        ("tree", counts.get("node_tree_gate_check_counts")),
        ("mech", counts.get("mechanical_gate_check_counts")),
    ]
    parts: list[str] = []
    for label, raw_counts in groups:
        if not isinstance(raw_counts, dict):
            continue
        entries = [
            f"{key}:{value}"
            for key, value in sorted(raw_counts.items())
            if _is_positive_count(value)
        ]
        if entries:
            parts.append(f"{label}[{','.join(entries)}]")
    return "|".join(parts) if parts else "none"


def _is_positive_count(value: Any) -> bool:
    try:
        return int(value) > 0
    except (TypeError, ValueError):
        return False


def _build_delivery_failure_robots(
    robot_summaries: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    failures: list[dict[str, Any]] = []
    for summary in robot_summaries:
        delivery_incomplete = summary.get("delivery_complete") is False
        parameters_incomplete = summary.get("delivery_parameters_complete") is False
        if not delivery_incomplete and not parameters_incomplete:
            continue

        failures.append(
            {
                "input": summary.get("input"),
                "robot_name": summary.get("robot_name"),
                "delivery_source": summary.get("delivery_source"),
                "delivery_dynamic_robot_generation": summary.get(
                    "delivery_dynamic_robot_generation"
                ),
                "delivery_complete": summary.get("delivery_complete"),
                "delivery_expected_parts": summary.get("delivery_expected_parts"),
                "delivery_parts_complete": summary.get("delivery_parts_complete"),
                "delivery_expected_joints": summary.get("delivery_expected_joints"),
                "delivery_failed_joints": summary.get("delivery_failed_joints"),
                "delivery_joints_complete": summary.get("delivery_joints_complete"),
                "delivery_parameterized_joints": summary.get(
                    "delivery_parameterized_joints"
                ),
                "delivery_parameters_complete": summary.get(
                    "delivery_parameters_complete"
                ),
                "delivery_part_nodes_count": summary.get(
                    "delivery_part_nodes_count"
                ),
                "delivery_joint_nodes_count": summary.get(
                    "delivery_joint_nodes_count"
                ),
                "failure_reasons": _slice_list(summary.get("failure_reasons", []), 5),
            }
        )

    return _slice_list(failures, 10)


def _build_commanded_response_failure_robots(
    robot_summaries: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    failures: list[dict[str, Any]] = []
    for summary in robot_summaries:
        if summary.get("commanded_joint_response_under_min") is not True:
            continue

        response_details = summary.get("commanded_joint_response_details", [])
        if isinstance(response_details, list):
            failed_details = [
                detail
                for detail in response_details
                if isinstance(detail, dict) and detail.get("responded") is False
            ]
        else:
            failed_details = []

        failures.append(
            {
                "input": summary.get("input"),
                "robot_name": summary.get("robot_name"),
                "commanded_joint_response_ratio": summary.get(
                    "commanded_joint_response_ratio"
                ),
                "commanded_static_joints": _slice_list(
                    summary.get("commanded_static_joints", []),
                    10,
                ),
                "commanded_joint_response_details": _slice_list(
                    failed_details or response_details,
                    10,
                ),
                "failure_reasons": _slice_list(summary.get("failure_reasons", []), 5),
            }
        )

    return _slice_list(failures, 10)


def _build_batch_robot_summary(report: dict[str, Any]) -> dict[str, Any]:
    godot_smoke = report.get("godot_smoke")
    delivery_summary = report.get("delivery_contract_preview", {})
    parameter_summary = _batch_smoke_nested(
        report,
        ["joint_parameter_consistency_summary"],
    )
    control_summary = _batch_smoke_nested(
        report,
        ["joint_control_consistency_summary"],
    )
    runtime_control_summary = _batch_smoke_nested(
        report,
        ["joint_control_summary"],
    )
    action_target_summary = _batch_smoke_nested(
        report,
        ["action_target_consistency_summary"],
    )
    action_sequence_target_summary = _batch_smoke_nested(
        report,
        ["action_sequence_target_consistency_summary"],
    )
    action_coverage_summary = _batch_smoke_nested(
        report,
        ["action_target_coverage_summary"],
    )
    control_action_coverage_summary = _batch_smoke_nested(
        report,
        ["control_action_coverage_summary"],
    )
    joint_motion_summary = _batch_smoke_nested(
        report,
        ["joint_motion_summary"],
    )
    action_sequence_summary = _batch_smoke_nested(
        report,
        ["action_sequence_summary"],
    )
    restoration_summary = _batch_smoke_nested(
        report,
        ["mechanical_restoration_summary"],
    )
    mechanical_gate_summary = _batch_smoke_nested(report, ["mechanical_gate_summary"])
    mechanical_behavior = _batch_smoke_nested(report, ["mechanical_behavior_evidence"])
    node_tree_gate_summary = _batch_smoke_nested(report, ["node_tree_gate_summary"])
    node_tree_manifest = _batch_smoke_nested(report, ["node_tree_manifest"])
    topology_summary = report.get("static", {}).get("topology_summary")
    if not isinstance(topology_summary, dict):
        topology_summary = {}
    static_node_tree_manifest = report.get("static", {}).get("node_tree_manifest")
    if not isinstance(static_node_tree_manifest, dict):
        static_node_tree_manifest = {}
    static_report = report.get("static", {})
    if not isinstance(static_report, dict):
        static_report = {}
    has_live_smoke = isinstance(godot_smoke, dict)
    return {
        "input": report["static"]["input"],
        "robot_name": report["static"].get("robot_name"),
        "status": report["status"],
        "parts_count": report["static"].get("parts_count"),
        "connections_count": report["static"].get("connections_count"),
        "topology_root_part": topology_summary.get("root_part"),
        "topology_complete_tree": topology_summary.get("complete_tree"),
        "topology_reachable_parts_count": topology_summary.get(
            "reachable_parts_count"
        ),
        "topology_disconnected_parts": _slice_list(
            topology_summary.get("disconnected_parts", []), 10
        ),
        "topology_unreachable_parts": _slice_list(
            topology_summary.get("unreachable_parts", []), 10
        ),
        "topology_duplicate_child_endpoints": _slice_list(
            topology_summary.get("duplicate_child_endpoints", []), 10
        ),
        "topology_cycle": _slice_list(topology_summary.get("cycle", []), 10),
        "static_node_tree_manifest_output": report.get(
            "static_node_tree_manifest_output"
        ),
        "static_node_tree_manifest_version": static_node_tree_manifest.get(
            "manifest_version"
        ),
        "static_node_tree_manifest_valid": (
            static_report.get("node_tree_manifest_errors") == []
            if isinstance(static_report, dict)
            and isinstance(static_node_tree_manifest, dict)
            else None
        ),
        "static_node_tree_manifest_errors": (
            _slice_list(static_report.get("node_tree_manifest_errors", []), 10)
            if isinstance(static_report, dict)
            else []
        ),
        "static_node_tree_manifest_error_count": (
            len(static_report.get("node_tree_manifest_errors", []))
            if isinstance(static_report.get("node_tree_manifest_errors"), list)
            else 0
        ),
        "static_node_tree_manifest_path_map_mismatch_count": (
            int(static_report.get("node_tree_manifest_path_map_mismatch_count") or 0)
            if isinstance(static_report, dict)
            else 0
        ),
        "static_node_tree_manifest_path_map_mismatch_kind_counts": (
            static_report.get("node_tree_manifest_path_map_mismatch_kind_counts", {})
            if isinstance(
                static_report.get("node_tree_manifest_path_map_mismatch_kind_counts"),
                dict,
            )
            else {}
        ),
        "static_node_tree_manifest_path_map_mismatches": (
            _slice_list(
                static_report.get("node_tree_manifest_path_map_mismatches", []),
                10,
            )
            if isinstance(static_report, dict)
            else []
        ),
        "static_node_tree_parts_planned_count": static_node_tree_manifest.get(
            "parts_count"
        ),
        "static_node_tree_joints_planned_count": static_node_tree_manifest.get(
            "joints_count"
        ),
        "static_node_tree_parameterized_joints_count": static_node_tree_manifest.get(
            "parameterized_joints"
        ),
        "static_node_tree_complete": static_node_tree_manifest.get("complete"),
        "static_node_tree_endpoint_paths_complete": static_node_tree_manifest.get(
            "endpoint_paths_complete"
        ),
        "static_node_tree_missing_endpoint_part_ids": _slice_list(
            static_node_tree_manifest.get("missing_endpoint_part_ids", []), 10
        ),
        "static_node_tree_missing_endpoint_connection_names": _slice_list(
            static_node_tree_manifest.get(
                "missing_endpoint_connection_names", []
            ),
            10,
        ),
        "static_node_tree_missing_endpoint_details": _slice_list(
            static_node_tree_manifest.get("missing_endpoint_details", []), 10
        ),
        "static_node_tree_parameters_complete": static_node_tree_manifest.get(
            "parameters_complete"
        ),
        "errors_count": len(report["static"].get("errors", [])),
        "delivery_source": (
            delivery_summary.get("source")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_dynamic_robot_generation": (
            delivery_summary.get("dynamic_robot_generation")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_complete": (
            delivery_summary.get("complete")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_expected_parts": (
            delivery_summary.get("expected_parts")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_parts_complete": (
            delivery_summary.get("parts_complete")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_expected_joints": (
            delivery_summary.get("expected_joints")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_failed_joints": (
            delivery_summary.get("failed_joints")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_joints_complete": (
            delivery_summary.get("joints_complete")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_parameterized_joints": (
            delivery_summary.get("parameterized_joints")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_parameters_complete": (
            delivery_summary.get("parameters_complete")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_part_nodes_count": (
            delivery_summary.get("part_nodes_count")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "delivery_joint_nodes_count": (
            delivery_summary.get("joint_nodes_count")
            if isinstance(delivery_summary, dict)
            else None
        ),
        "godot_smoke_returncode": (
            godot_smoke.get("returncode") if has_live_smoke else None
        ),
        "parameter_mismatch_count": (
            int(parameter_summary.get("mismatch_count") or 0)
            if has_live_smoke and isinstance(parameter_summary, dict)
            else None
        ),
        "parameter_consistency_complete": (
            parameter_summary.get("complete")
            if has_live_smoke and isinstance(parameter_summary, dict)
            else None
        ),
        "fixed_lock_checked_count": (
            int(parameter_summary.get("fixed_lock_checked_count") or 0)
            if has_live_smoke and isinstance(parameter_summary, dict)
            else None
        ),
        "fixed_lock_mismatch_count": (
            int(parameter_summary.get("fixed_lock_mismatch_count") or 0)
            if has_live_smoke and isinstance(parameter_summary, dict)
            else None
        ),
        "control_mismatch_count": (
            int(control_summary.get("mismatch_count") or 0)
            if has_live_smoke and isinstance(control_summary, dict)
            else None
        ),
        "control_configured_count": (
            int(control_summary.get("configured_count") or 0)
            if has_live_smoke and isinstance(control_summary, dict)
            else None
        ),
        "control_readback_checked_count": (
            int(control_summary.get("checked_count") or 0)
            if has_live_smoke and isinstance(control_summary, dict)
            else None
        ),
        "control_readback_missing_count": (
            int(control_summary.get("missing_count") or 0)
            if has_live_smoke and isinstance(control_summary, dict)
            else None
        ),
        "control_consistency_complete": (
            control_summary.get("complete")
            if has_live_smoke and isinstance(control_summary, dict)
            else None
        ),
        "nonzero_action_target_count": (
            int(runtime_control_summary.get("targeted_count") or 0)
            if has_live_smoke and isinstance(runtime_control_summary, dict)
            else None
        ),
        "nonzero_action_targets_under_min": (
            runtime_control_summary.get("nonzero_targets_under_min")
            if has_live_smoke and isinstance(runtime_control_summary, dict)
            else None
        ),
        "action_target_mismatch_count": (
            int(action_target_summary.get("mismatch_count") or 0)
            if has_live_smoke and isinstance(action_target_summary, dict)
            else None
        ),
        "unknown_action_target_count": (
            _effective_unknown_action_target_count(
                action_target_summary,
                action_sequence_target_summary,
            )
            if has_live_smoke
            and isinstance(action_target_summary, dict)
            and isinstance(action_sequence_target_summary, dict)
            else None
        ),
        "invalid_action_target_count": (
            _effective_invalid_action_target_count(
                action_target_summary,
                action_sequence_target_summary,
            )
            if has_live_smoke
            and isinstance(action_target_summary, dict)
            and isinstance(action_sequence_target_summary, dict)
            else None
        ),
        "action_target_consistency_complete": (
            action_target_summary.get("complete")
            if has_live_smoke and isinstance(action_target_summary, dict)
            else None
        ),
        "action_sequence_target_mismatch_count": (
            int(action_sequence_target_summary.get("mismatch_count") or 0)
            if has_live_smoke and isinstance(action_sequence_target_summary, dict)
            else None
        ),
        "action_sequence_target_consistency_complete": (
            action_sequence_target_summary.get("complete")
            if has_live_smoke and isinstance(action_sequence_target_summary, dict)
            else None
        ),
        "action_target_coverage_ratio": (
            action_coverage_summary.get("coverage_ratio")
            if has_live_smoke and isinstance(action_coverage_summary, dict)
            else None
        ),
        "action_target_coverage_under_min": (
            action_coverage_summary.get("coverage_under_min")
            if has_live_smoke and isinstance(action_coverage_summary, dict)
            else None
        ),
        "action_target_coverage_complete": (
            action_coverage_summary.get("complete")
            if has_live_smoke and isinstance(action_coverage_summary, dict)
            else None
        ),
        "control_action_coverage_ratio": (
            control_action_coverage_summary.get("coverage_ratio")
            if has_live_smoke and isinstance(control_action_coverage_summary, dict)
            else None
        ),
        "control_action_coverage_under_min": (
            control_action_coverage_summary.get("coverage_under_min")
            if has_live_smoke and isinstance(control_action_coverage_summary, dict)
            else None
        ),
        "control_action_coverage_complete": (
            control_action_coverage_summary.get("complete")
            if has_live_smoke and isinstance(control_action_coverage_summary, dict)
            else None
        ),
        "joint_angle_delta_max": (
            joint_motion_summary.get("max_abs_relative_angle_delta")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "joint_angle_delta_under_min": (
            joint_motion_summary.get("angle_delta_under_min")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "joint_angle_range_max": (
            joint_motion_summary.get("max_abs_relative_angle_range")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "joint_angle_range_under_min": (
            joint_motion_summary.get("angle_range_under_min")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "moving_joint_coverage_ratio": (
            joint_motion_summary.get("moving_joint_coverage_ratio")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "moving_joint_coverage_under_min": (
            joint_motion_summary.get("moving_joint_coverage_under_min")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "commanded_joint_response_ratio": (
            joint_motion_summary.get("commanded_joint_response_ratio")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "commanded_joint_response_under_min": (
            joint_motion_summary.get("commanded_joint_response_under_min")
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "commanded_static_joints": (
            _slice_list(joint_motion_summary.get("commanded_static_joints", []), 10)
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "commanded_joint_response_details": (
            _slice_list(
                joint_motion_summary.get("commanded_joint_response_details", []),
                10,
            )
            if has_live_smoke and isinstance(joint_motion_summary, dict)
            else None
        ),
        "action_transition_count": (
            int(action_sequence_summary.get("transition_count") or 0)
            if has_live_smoke and isinstance(action_sequence_summary, dict)
            else None
        ),
        "action_transitions_under_min": (
            action_sequence_summary.get("transitions_under_min")
            if has_live_smoke and isinstance(action_sequence_summary, dict)
            else None
        ),
        "action_transition_delta_max": (
            action_sequence_summary.get("max_numeric_transition_delta")
            if has_live_smoke and isinstance(action_sequence_summary, dict)
            else None
        ),
        "action_transition_delta_under_min": (
            action_sequence_summary.get("transition_delta_under_min")
            if has_live_smoke and isinstance(action_sequence_summary, dict)
            else None
        ),
        "restoration_complete": (
            restoration_summary.get("complete")
            if has_live_smoke and isinstance(restoration_summary, dict)
            else None
        ),
        "restoration_score": (
            restoration_summary.get("score")
            if has_live_smoke and isinstance(restoration_summary, dict)
            else None
        ),
        "mechanical_gate_enabled_count": (
            int(mechanical_gate_summary.get("enabled_count") or 0)
            if has_live_smoke and isinstance(mechanical_gate_summary, dict)
            else None
        ),
        "full_mechanical_restoration_required": (
            mechanical_gate_summary.get("full_mechanical_restoration_required")
            if has_live_smoke and isinstance(mechanical_gate_summary, dict)
            else None
        ),
        "mechanical_gate_enabled_checks": (
            _slice_list(mechanical_gate_summary.get("enabled_checks", []), 10)
            if has_live_smoke and isinstance(mechanical_gate_summary, dict)
            else None
        ),
        "mechanical_behavior_evidence_version": (
            mechanical_behavior.get("evidence_version")
            if has_live_smoke and isinstance(mechanical_behavior, dict)
            else None
        ),
        "mechanical_behavior_complete": (
            mechanical_behavior.get("complete")
            if has_live_smoke and isinstance(mechanical_behavior, dict)
            else None
        ),
        "mechanical_behavior_residual_risks": (
            _slice_list(mechanical_behavior.get("residual_risks", []), 10)
            if has_live_smoke and isinstance(mechanical_behavior, dict)
            else None
        ),
        "mechanical_behavior_threshold_failures": (
            _slice_list(mechanical_behavior.get("threshold_failures", []), 10)
            if has_live_smoke and isinstance(mechanical_behavior, dict)
            else None
        ),
        "mechanical_behavior_center_of_mass_available": (
            mechanical_behavior.get("center_of_mass_evidence", {}).get("available")
            if has_live_smoke
            and isinstance(mechanical_behavior, dict)
            and isinstance(mechanical_behavior.get("center_of_mass_evidence"), dict)
            else None
        ),
        "mechanical_behavior_contact_state_available": (
            mechanical_behavior.get("contact_state_evidence", {}).get("available")
            if has_live_smoke
            and isinstance(mechanical_behavior, dict)
            and isinstance(mechanical_behavior.get("contact_state_evidence"), dict)
            else None
        ),
        "mechanical_behavior_step_trace_artifact_path": (
            mechanical_behavior.get("step_trace_evidence", {}).get("artifact_path")
            if has_live_smoke
            and isinstance(mechanical_behavior, dict)
            and isinstance(mechanical_behavior.get("step_trace_evidence"), dict)
            else None
        ),
        "node_tree_complete": (
            node_tree_manifest.get("complete")
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_gate_enabled_count": (
            int(node_tree_gate_summary.get("enabled_count") or 0)
            if has_live_smoke and isinstance(node_tree_gate_summary, dict)
            else None
        ),
        "node_tree_full_restoration_required": (
            node_tree_gate_summary.get("full_node_tree_restoration_required")
            if has_live_smoke and isinstance(node_tree_gate_summary, dict)
            else None
        ),
        "node_tree_gate_enabled_checks": (
            _slice_list(node_tree_gate_summary.get("enabled_checks", []), 10)
            if has_live_smoke and isinstance(node_tree_gate_summary, dict)
            else None
        ),
        "node_tree_missing_parts_count": (
            len(node_tree_manifest.get("missing_part_ids", []))
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_missing_joints_count": (
            len(node_tree_manifest.get("missing_connection_names", []))
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_class_mismatch_count": (
            int(node_tree_manifest.get("class_mismatch_count") or 0)
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_parameter_missing_count": (
            int(node_tree_manifest.get("parameter_missing_count") or 0)
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_transform_mismatch_count": (
            int(node_tree_manifest.get("transform_mismatch_count") or 0)
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_physical_mismatch_count": (
            int(node_tree_manifest.get("physical_mismatch_count") or 0)
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_fixed_lock_checked_count": (
            int(node_tree_manifest.get("fixed_lock_checked_count") or 0)
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_fixed_lock_mismatch_count": (
            int(node_tree_manifest.get("fixed_lock_mismatch_count") or 0)
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "node_tree_fixed_locks_complete": (
            node_tree_manifest.get("fixed_locks_complete")
            if has_live_smoke and isinstance(node_tree_manifest, dict)
            else None
        ),
        "failure_reasons": _build_batch_failure_reasons(report),
    }


def _build_batch_failure_reasons(report: dict[str, Any]) -> list[str]:
    reasons: list[str] = []
    static_errors = report.get("static", {}).get("errors", [])
    if isinstance(static_errors, list):
        reasons.extend(str(error) for error in static_errors if error)

    skipped_reason = report.get("godot_smoke_skipped_reason")
    if skipped_reason:
        reasons.append(str(skipped_reason))

    smoke = report.get("godot_smoke")
    if isinstance(smoke, dict):
        execution_failure_reasons = smoke.get("execution_failure_reasons")
        if isinstance(execution_failure_reasons, list):
            reasons.extend(
                str(reason) for reason in execution_failure_reasons if reason
            )
        elif smoke.get("returncode") not in (None, 0):
            reasons.append(f"godot smoke exited with returncode {smoke.get('returncode')}")
        smoke_errors = (
            smoke.get("report_summary", {}).get("errors", [])
            if isinstance(smoke.get("report_summary"), dict)
            else []
        )
        if isinstance(smoke_errors, list):
            reasons.extend(str(error) for error in smoke_errors if error)

    return reasons


def _effective_unknown_action_target_count(
    action_target_summary: Any,
    action_sequence_target_summary: Any,
) -> int:
    if isinstance(action_sequence_target_summary, dict) and int(
        action_sequence_target_summary.get("steps") or 0
    ) > 0:
        return int(action_sequence_target_summary.get("unknown_target_count") or 0)
    if isinstance(action_target_summary, dict):
        return int(action_target_summary.get("unknown_target_count") or 0)
    return 0


def _effective_invalid_action_target_count(
    action_target_summary: Any,
    action_sequence_target_summary: Any,
) -> int:
    if isinstance(action_sequence_target_summary, dict) and int(
        action_sequence_target_summary.get("steps") or 0
    ) > 0:
        return int(action_sequence_target_summary.get("invalid_target_count") or 0)
    if isinstance(action_target_summary, dict):
        return int(action_target_summary.get("invalid_target_count") or 0)
    return 0


def _batch_smoke_nested(report: dict[str, Any], keys: list[str]) -> Any:
    smoke = report.get("godot_smoke")
    if not isinstance(smoke, dict):
        return {}
    current: Any = smoke.get("report_summary", {})
    for key in keys:
        if not isinstance(current, dict):
            return {}
        current = current.get(key, {})
    return current


def _build_acceptance_requirements(args: argparse.Namespace) -> dict[str, bool]:
    full_mechanical = bool(args.fail_on_full_mechanical_restoration)
    full_node_tree = bool(args.fail_on_full_node_tree_restoration) or full_mechanical
    return build_delivery_acceptance_requirements(
        run_godot_smoke=bool(args.run_godot_smoke),
        godot_verified_acceptance=bool(args.require_godot_verified_acceptance),
        full_mechanical_restoration_gate=bool(
            args.require_full_mechanical_restoration_gate
        ),
        full_mechanical_restoration_smoke_gate=full_mechanical,
        mechanical_restoration_complete=bool(args.fail_on_incomplete_restoration)
        or full_mechanical,
        joint_parameter_readback=bool(args.fail_on_parameter_mismatch)
        or full_mechanical,
        control_parameter_readback=bool(args.fail_on_control_mismatch)
        or full_mechanical,
        full_node_tree_restoration=full_node_tree,
        node_tree_complete=bool(args.fail_on_incomplete_node_tree)
        or full_node_tree,
        node_tree_class_match=bool(args.fail_on_node_tree_class_mismatch)
        or full_node_tree,
        node_tree_parameters_applied=bool(args.fail_on_node_tree_missing_parameters)
        or full_node_tree,
        node_tree_transform_match=bool(args.fail_on_node_tree_transform_mismatch)
        or full_node_tree,
        node_tree_physical_match=bool(args.fail_on_node_tree_physical_mismatch)
        or full_node_tree,
        node_tree_fixed_lock_match=bool(args.fail_on_node_tree_fixed_lock_mismatch)
        or full_node_tree,
        joint_limit_violation_gate=bool(args.fail_on_joint_limit_violation),
        body_motion_gate=args.min_body_displacement is not None
        or args.max_linear_speed is not None,
        joint_motion_delta_gate=args.min_joint_angle_delta is not None,
        joint_motion_range_gate=args.min_joint_angle_range is not None,
        moving_joint_coverage_gate=args.min_moving_joint_coverage is not None,
        commanded_joint_response_gate=args.min_commanded_joint_response_coverage
        is not None,
        action_target_consistency=bool(args.fail_on_action_target_mismatch),
        action_sequence_target_consistency=bool(
            args.fail_on_action_sequence_target_mismatch
        ),
        unknown_action_target_gate=bool(args.fail_on_unknown_action_target),
        invalid_action_target_gate=bool(args.fail_on_invalid_action_target),
        action_target_coverage_gate=args.min_action_target_coverage is not None,
        control_action_coverage_gate=args.min_control_action_coverage is not None,
        nonzero_action_targets_gate=args.min_nonzero_action_targets is not None,
        action_transition_count_gate=args.min_action_transitions is not None,
        action_transition_delta_gate=args.min_action_transition_delta is not None,
        restoration_score_gate=args.min_restoration_score is not None,
        static_node_tree_complete=bool(
            getattr(args, "fail_on_static_node_tree_incomplete", False)
        ),
        static_node_tree_manifest_output=bool(
            getattr(args, "require_static_node_tree_manifest_output", False)
        ),
    )


def main() -> int:
    _configure_stdio_for_json_output()
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Create a JSON diagnostic report for dynamic Godot robot generation."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="One or more robot JSON configs to inspect.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional report output path.",
    )
    parser.add_argument(
        "--gate-output",
        type=Path,
        help="Optional path for a compact delivery_acceptance_gate JSON artifact.",
    )
    parser.add_argument(
        "--static-node-tree-manifest-dir",
        type=Path,
        help=(
            "Optional directory for per-input static Godot node-tree manifest "
            "JSON artifacts."
        ),
    )
    parser.add_argument(
        "--require-static-node-tree-manifest-output",
        action="store_true",
        help=(
            "Return a non-zero exit code unless every input writes a standalone "
            "static Godot node-tree manifest artifact."
        ),
    )
    parser.add_argument(
        "--fail-on-static-node-tree-incomplete",
        action="store_true",
        help=(
            "Return a non-zero exit code when the static Godot node-tree "
            "manifest is incomplete before live smoke."
        ),
    )
    parser.add_argument(
        "--normalized-output",
        type=Path,
        help="Optional path for the normalized Godot-ready robot config.",
    )
    parser.add_argument(
        "--run-godot-smoke",
        action="store_true",
        help="Launch Godot headless and include live assembly results.",
    )
    parser.add_argument(
        "--mechanical-trace-output",
        type=Path,
        help=(
            "Optional JSON artifact path for the full mechanical step trace "
            "written by the live Godot smoke runner."
        ),
    )
    parser.add_argument(
        "--full-mechanical-restoration-acceptance",
        action="store_true",
        help=(
            "Shortcut for live CI acceptance: run Godot smoke, enable full "
            "mechanical restoration gates, require Godot-verified acceptance, "
            "and require full mechanical gate coverage."
        ),
    )
    parser.add_argument(
        "--require-godot-verified-acceptance",
        action="store_true",
        help=(
            "Return a non-zero exit code unless every input is dynamically generated, "
            "loaded through Godot smoke, fully delivered, fully parameterized, and free "
            "of smoke failure reasons."
        ),
    )
    parser.add_argument(
        "--require-full-mechanical-restoration-gate",
        action="store_true",
        help=(
            "Return a non-zero acceptance gate exit code unless every input reports "
            "full mechanical restoration gate coverage in live smoke."
        ),
    )
    parser.add_argument(
        "--smoke-output",
        type=Path,
        help="Optional path for the Godot smoke report.",
    )
    parser.add_argument(
        "--godot-exe",
        default=r"D:\迅雷下载\Godot\Godot.exe",
        help="Path to Godot executable when --run-godot-smoke is used.",
    )
    parser.add_argument(
        "--live-profile",
        choices=["local", "manual_ci", "scheduled_ci"],
        default="local",
        help="Reusable live Godot verification profile passed to the smoke runner.",
    )
    parser.add_argument(
        "--live-artifact-root",
        type=Path,
        default=None,
        help="Artifact root recorded by the live Godot smoke runner.",
    )
    parser.add_argument(
        "--live-retention-days",
        type=int,
        default=None,
        help="Retention days recorded by the live Godot smoke runner.",
    )
    parser.add_argument(
        "--flaky-retry-attempts",
        type=int,
        default=None,
        help="Retry budget recorded by the live Godot smoke runner.",
    )
    parser.add_argument("--port", type=int, default=19170)
    parser.add_argument("--timeout-seconds", type=float, default=8.0)
    parser.add_argument(
        "--max-endpoint-distance",
        type=float,
        default=None,
        help="Optional failure threshold for joint endpoint distance in live smoke.",
    )
    parser.add_argument(
        "--max-relative-angle",
        type=float,
        default=None,
        help="Optional failure threshold for absolute joint relative angle in radians.",
    )
    parser.add_argument(
        "--min-body-displacement",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for minimum body displacement.",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for max final body linear speed.",
    )
    parser.add_argument(
        "--min-joint-angle-delta",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for maximum joint relative_angle change.",
    )
    parser.add_argument(
        "--min-joint-angle-range",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for maximum joint relative_angle range.",
    )
    parser.add_argument(
        "--min-moving-joint-coverage",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for ratio of joints whose angle range exceeds --joint-motion-epsilon.",
    )
    parser.add_argument(
        "--min-commanded-joint-response-coverage",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for ratio of action-targeted joints whose angle range exceeds --joint-motion-epsilon.",
    )
    parser.add_argument(
        "--joint-motion-epsilon",
        type=float,
        default=0.0,
        help="Minimum relative_angle range required to count a joint as moving.",
    )
    parser.add_argument(
        "--min-action-target-coverage",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for action target joint coverage ratio.",
    )
    parser.add_argument(
        "--min-control-action-coverage",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for control.joints action coverage ratio.",
    )
    parser.add_argument(
        "--min-nonzero-action-targets",
        type=int,
        default=None,
        help="Optional live smoke failure threshold for nonzero runtime joint target velocity count.",
    )
    parser.add_argument(
        "--min-action-transitions",
        type=int,
        default=None,
        help="Optional live smoke failure threshold for adjacent action payload transitions.",
    )
    parser.add_argument(
        "--min-action-transition-delta",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for adjacent numeric action delta.",
    )
    parser.add_argument(
        "--fail-on-joint-limit-violation",
        action="store_true",
        help="Fail live smoke when generated joint telemetry exceeds JSON limits.",
    )
    parser.add_argument(
        "--fail-on-incomplete-restoration",
        action="store_true",
        help="Fail live smoke when mechanical restoration is incomplete.",
    )
    parser.add_argument(
        "--min-restoration-score",
        type=float,
        default=None,
        help="Optional live smoke failure threshold for restoration score.",
    )
    parser.add_argument(
        "--fail-on-parameter-mismatch",
        action="store_true",
        help="Fail live smoke when source JSON joint parameters mismatch runtime readback.",
    )
    parser.add_argument(
        "--fail-on-control-mismatch",
        action="store_true",
        help="Fail live smoke when source JSON control.joints parameters mismatch runtime readback.",
    )
    parser.add_argument(
        "--fail-on-full-mechanical-restoration",
        action="store_true",
        help=(
            "Fail live smoke when mechanical restoration completeness, JSON joint "
            "parameter readback, control.joints readback, or full node-tree "
            "restoration checks fail."
        ),
    )
    parser.add_argument(
        "--fail-on-action-target-mismatch",
        action="store_true",
        help="Fail live smoke when the last action payload mismatches runtime joint target velocity.",
    )
    parser.add_argument(
        "--fail-on-action-sequence-target-mismatch",
        action="store_true",
        help="Fail live smoke when any action sequence step mismatches runtime joint target velocity.",
    )
    parser.add_argument(
        "--fail-on-unknown-action-target",
        action="store_true",
        help="Fail live smoke when an action references a joint name or list index not present in telemetry.",
    )
    parser.add_argument(
        "--fail-on-invalid-action-target",
        action="store_true",
        help="Fail live smoke when an action target value is present but is not numeric.",
    )
    parser.add_argument(
        "--fail-on-incomplete-node-tree",
        action="store_true",
        help="Fail live smoke when node_tree_manifest reports missing parts or joints.",
    )
    parser.add_argument(
        "--fail-on-full-node-tree-restoration",
        action="store_true",
        help=(
            "Fail live smoke when any node_tree_manifest restoration check fails, "
            "including missing nodes, class mismatches, missing parameters, "
            "transform mismatches, physical mismatches, or fixed joint lock mismatches."
        ),
    )
    parser.add_argument(
        "--fail-on-node-tree-class-mismatch",
        action="store_true",
        help="Fail live smoke when node_tree_manifest reports unexpected generated node classes.",
    )
    parser.add_argument(
        "--fail-on-node-tree-missing-parameters",
        action="store_true",
        help="Fail live smoke when node_tree_manifest reports joints without applied_parameters.",
    )
    parser.add_argument(
        "--fail-on-node-tree-transform-mismatch",
        action="store_true",
        help="Fail live smoke when generated node transforms differ from JSON.",
    )
    parser.add_argument(
        "--fail-on-node-tree-physical-mismatch",
        action="store_true",
        help="Fail live smoke when generated mass, collision, or mesh parameters differ from JSON.",
    )
    parser.add_argument(
        "--fail-on-node-tree-fixed-lock-mismatch",
        action="store_true",
        help="Fail live smoke when node_tree_manifest reports fixed joint lock mismatches.",
    )
    parser.add_argument(
        "--node-tree-tolerance",
        type=float,
        default=1e-4,
        help="Absolute tolerance used by live smoke node tree transform checks.",
    )
    parser.add_argument(
        "--parameter-tolerance",
        type=float,
        default=1e-4,
        help="Absolute tolerance used by live smoke parameter consistency checks.",
    )
    parser.add_argument(
        "--action-json",
        default="[0.0]",
        help="JSON action payload sent to each live Godot smoke step.",
    )
    parser.add_argument(
        "--action-sequence-json",
        default=None,
        help="JSON array of action payloads sent across live Godot smoke steps.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=1,
        help="Number of Godot step commands to send in live smoke.",
    )
    parser.add_argument(
        "--step-delay-seconds",
        type=float,
        default=0.0,
        help="Optional delay between live smoke step commands.",
    )
    args = parser.parse_args()
    if args.parameter_tolerance < 0:
        parser.error("--parameter-tolerance must be greater than or equal to 0")
    if args.node_tree_tolerance < 0:
        parser.error("--node-tree-tolerance must be greater than or equal to 0")
    if args.min_joint_angle_delta is not None and args.min_joint_angle_delta < 0:
        parser.error("--min-joint-angle-delta must be greater than or equal to 0")
    if args.min_joint_angle_range is not None and args.min_joint_angle_range < 0:
        parser.error("--min-joint-angle-range must be greater than or equal to 0")
    if args.joint_motion_epsilon < 0:
        parser.error("--joint-motion-epsilon must be greater than or equal to 0")
    if args.min_moving_joint_coverage is not None and not (
        0.0 <= args.min_moving_joint_coverage <= 1.0
    ):
        parser.error("--min-moving-joint-coverage must be between 0 and 1")
    if args.min_commanded_joint_response_coverage is not None and not (
        0.0 <= args.min_commanded_joint_response_coverage <= 1.0
    ):
        parser.error("--min-commanded-joint-response-coverage must be between 0 and 1")
    if args.min_action_target_coverage is not None and not (
        0.0 <= args.min_action_target_coverage <= 1.0
    ):
        parser.error("--min-action-target-coverage must be between 0 and 1")
    if args.min_control_action_coverage is not None and not (
        0.0 <= args.min_control_action_coverage <= 1.0
    ):
        parser.error("--min-control-action-coverage must be between 0 and 1")
    if args.min_nonzero_action_targets is not None and args.min_nonzero_action_targets < 0:
        parser.error("--min-nonzero-action-targets must be greater than or equal to 0")
    if args.min_action_transitions is not None and args.min_action_transitions < 0:
        parser.error("--min-action-transitions must be greater than or equal to 0")
    if args.min_action_transition_delta is not None and args.min_action_transition_delta < 0:
        parser.error("--min-action-transition-delta must be greater than or equal to 0")
    if args.live_retention_days is not None and args.live_retention_days < 0:
        parser.error("--live-retention-days must be greater than or equal to 0")
    if args.flaky_retry_attempts is not None and args.flaky_retry_attempts < 0:
        parser.error("--flaky-retry-attempts must be greater than or equal to 0")

    if args.full_mechanical_restoration_acceptance:
        args.run_godot_smoke = True
        args.fail_on_full_mechanical_restoration = True
        args.require_godot_verified_acceptance = True
        args.require_full_mechanical_restoration_gate = True
    acceptance_profile = (
        "full_mechanical_restoration"
        if args.full_mechanical_restoration_acceptance
        else "custom"
    )
    acceptance_requirements = _build_acceptance_requirements(args)

    robot_schema = _load_robot_schema_module(repo_root)
    if len(args.inputs) > 1 and (
        args.normalized_output or args.smoke_output or args.mechanical_trace_output
    ):
        parser.error(
            "--normalized-output, --smoke-output, and --mechanical-trace-output "
            "are only valid for one input"
        )

    reports: list[dict[str, Any]] = []
    for index, input_path in enumerate(args.inputs):
        normalized_output = args.normalized_output or _default_normalized_output(
            repo_root, input_path
        )
        smoke_output = args.smoke_output or _default_smoke_output(repo_root, input_path)
        report_for_input = _build_report_for_input(
            repo_root=repo_root,
            robot_schema=robot_schema,
            input_path=input_path,
            normalized_output=normalized_output,
            run_godot_smoke=args.run_godot_smoke,
            smoke_output=smoke_output,
            godot_exe=args.godot_exe,
            live_profile=args.live_profile,
            live_artifact_root=args.live_artifact_root,
            live_retention_days=args.live_retention_days,
            flaky_retry_attempts=args.flaky_retry_attempts,
            port=args.port + index,
            timeout_seconds=args.timeout_seconds,
            max_endpoint_distance=args.max_endpoint_distance,
            max_relative_angle=args.max_relative_angle,
            min_body_displacement=args.min_body_displacement,
            max_linear_speed=args.max_linear_speed,
            min_joint_angle_delta=args.min_joint_angle_delta,
            min_joint_angle_range=args.min_joint_angle_range,
            min_moving_joint_coverage=args.min_moving_joint_coverage,
            min_commanded_joint_response_coverage=args.min_commanded_joint_response_coverage,
            joint_motion_epsilon=args.joint_motion_epsilon,
            min_action_target_coverage=args.min_action_target_coverage,
            min_control_action_coverage=args.min_control_action_coverage,
            min_nonzero_action_targets=args.min_nonzero_action_targets,
            min_action_transitions=args.min_action_transitions,
            min_action_transition_delta=args.min_action_transition_delta,
            fail_on_joint_limit_violation=args.fail_on_joint_limit_violation,
            fail_on_incomplete_restoration=args.fail_on_incomplete_restoration,
            min_restoration_score=args.min_restoration_score,
            fail_on_parameter_mismatch=args.fail_on_parameter_mismatch,
            fail_on_control_mismatch=args.fail_on_control_mismatch,
            fail_on_full_mechanical_restoration=args.fail_on_full_mechanical_restoration,
            fail_on_action_target_mismatch=args.fail_on_action_target_mismatch,
            fail_on_action_sequence_target_mismatch=args.fail_on_action_sequence_target_mismatch,
            fail_on_unknown_action_target=args.fail_on_unknown_action_target,
            fail_on_invalid_action_target=args.fail_on_invalid_action_target,
            fail_on_incomplete_node_tree=args.fail_on_incomplete_node_tree,
            fail_on_full_node_tree_restoration=args.fail_on_full_node_tree_restoration,
            fail_on_node_tree_class_mismatch=args.fail_on_node_tree_class_mismatch,
            fail_on_node_tree_missing_parameters=args.fail_on_node_tree_missing_parameters,
            fail_on_node_tree_transform_mismatch=args.fail_on_node_tree_transform_mismatch,
            fail_on_node_tree_physical_mismatch=args.fail_on_node_tree_physical_mismatch,
            fail_on_node_tree_fixed_lock_mismatch=args.fail_on_node_tree_fixed_lock_mismatch,
            node_tree_tolerance=args.node_tree_tolerance,
            parameter_tolerance=args.parameter_tolerance,
            action_json=args.action_json,
            action_sequence_json=args.action_sequence_json,
            steps=args.steps,
            step_delay_seconds=args.step_delay_seconds,
            mechanical_trace_output=args.mechanical_trace_output,
        )
        if args.static_node_tree_manifest_dir:
            manifest_output = _static_node_tree_manifest_output_path(
                args.static_node_tree_manifest_dir,
                input_path,
                index,
            )
            _write_static_node_tree_manifest_artifact(report_for_input, manifest_output)
        reports.append(report_for_input)

    batch_summary = _build_batch_summary(
        reports,
        require_full_mechanical_restoration_gate=(
            args.require_full_mechanical_restoration_gate
        ),
        acceptance_profile=acceptance_profile,
        acceptance_requirements=acceptance_requirements,
    )
    report_status = (
        reports[0]["status"]
        if len(reports) == 1
        else (
            "success"
            if all(item["status"] == "success" for item in reports)
            else "error"
        )
    )
    acceptance_gate = _build_delivery_acceptance_gate(
        required=args.require_godot_verified_acceptance,
        report_status=report_status,
        batch_summary=batch_summary,
    )
    gate_contract_errors = validate_delivery_acceptance_gate(acceptance_gate)
    if gate_contract_errors:
        print(
            "delivery_acceptance_gate contract invalid "
            f"errors={'; '.join(gate_contract_errors)}",
            file=sys.stderr,
        )
        return 2
    report = (
        {
            **reports[0],
            "delivery_acceptance_summary": {
                "delivery_acceptance_complete": batch_summary[
                    "delivery_acceptance_complete"
                ],
                "acceptance_profile": batch_summary["acceptance_profile"],
                "delivery_acceptance_level": batch_summary[
                    "delivery_acceptance_level"
                ],
                "delivery_acceptance_reasons": batch_summary[
                    "delivery_acceptance_reasons"
                ],
                "delivery_acceptance_reason_codes": batch_summary[
                    "delivery_acceptance_reason_codes"
                ],
                "delivery_acceptance_reason_details": batch_summary[
                    "delivery_acceptance_reason_details"
                ],
            },
            "delivery_acceptance_gate": acceptance_gate,
        }
        if len(reports) == 1
        else {
            "status": report_status,
            "delivery_acceptance_gate": acceptance_gate,
            "batch_summary": batch_summary,
            "reports": reports,
        }
    )

    if args.output:
        _write_json(args.output, report)
    if args.gate_output:
        _write_json(args.gate_output, acceptance_gate)

    print(json.dumps(report, indent=2, ensure_ascii=False))
    if acceptance_gate["exit_code"]:
        print(
            _format_delivery_acceptance_gate_log(acceptance_gate, batch_summary),
            file=sys.stderr,
        )
    return int(acceptance_gate["exit_code"])


if __name__ == "__main__":
    raise SystemExit(main())
