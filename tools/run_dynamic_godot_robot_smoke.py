"""Run a headless Godot smoke test for dynamic robot JSON assembly."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path
from typing import Any


def _load_robot_schema_module() -> Any:
    repo_root = Path(__file__).resolve().parents[1]
    module_path = repo_root / "agi_walker" / "core" / "api" / "robot_schema.py"
    spec = importlib.util.spec_from_file_location("robot_schema", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load robot schema module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


robot_schema = _load_robot_schema_module()

DEFAULT_GODOT_EXE = r"D:\迅雷下载\Godot\Godot.exe"
LIVE_VERIFICATION_PROFILE_VERSION = "dynamic_godot_live_verification_profile.v1"
MECHANICAL_BEHAVIOR_EVIDENCE_VERSION = "dynamic_godot_mechanical_behavior_evidence.v1"
MECHANICAL_BEHAVIOR_TRACE_VERSION = "dynamic_godot_mechanical_behavior_trace.v1"
MECHANICAL_BEHAVIOR_TRACE_INLINE_LIMIT = 20
LIVE_VERIFICATION_PROFILES: dict[str, dict[str, Any]] = {
    "local": {
        "environment_mode": "local",
        "ci_mode": False,
        "scheduled": False,
        "default_artifact_root": "test_env/dynamic_godot_live/local",
        "retention_days": 7,
        "flaky_retry_attempts": 0,
    },
    "manual_ci": {
        "environment_mode": "manual_ci",
        "ci_mode": True,
        "scheduled": False,
        "default_artifact_root": "test_env/dynamic_godot_live/manual_ci",
        "retention_days": 14,
        "flaky_retry_attempts": 1,
    },
    "scheduled_ci": {
        "environment_mode": "scheduled_ci",
        "ci_mode": True,
        "scheduled": True,
        "default_artifact_root": "test_env/dynamic_godot_live/scheduled_ci",
        "retention_days": 30,
        "flaky_retry_attempts": 1,
    },
}


def _mechanical_step_trace_entries(
    step_results: list[dict[str, Any]],
    actions: list[Any],
    *,
    limit: int | None = None,
) -> list[dict[str, Any]]:
    selected_steps = step_results[:limit] if limit is not None else step_results
    trace: list[dict[str, Any]] = []
    for index, step in enumerate(selected_steps):
        body_states = step.get("body_states", {})
        joint_states = step.get("joint_states", {})
        trace.append(
            {
                "step_index": index,
                "body_count": step.get("body_count"),
                "joint_count": step.get("joint_count"),
                "body_states_count": len(body_states) if isinstance(body_states, dict) else 0,
                "joint_states_count": len(joint_states) if isinstance(joint_states, dict) else 0,
                "reward": step.get("reward"),
                "done": step.get("done"),
                "action": actions[index] if index < len(actions) else None,
            }
        )
    return trace


def _step_trace_evidence(
    step_results: list[dict[str, Any]],
    actions: list[Any],
    *,
    inline_limit: int = MECHANICAL_BEHAVIOR_TRACE_INLINE_LIMIT,
    artifact_path: str | None = None,
) -> dict[str, Any]:
    trace = _mechanical_step_trace_entries(
        step_results,
        actions,
        limit=inline_limit,
    )
    return {
        "available": bool(step_results),
        "steps_run": len(step_results),
        "inline_limit": inline_limit,
        "inline_step_count": len(trace),
        "truncated": len(step_results) > inline_limit,
        "trace": trace,
        "artifact_path": artifact_path,
        "artifact_written": artifact_path is not None,
    }


def _mechanical_trace_artifact(
    *,
    robot_name: str | None,
    step_results: list[dict[str, Any]],
    actions: list[Any],
) -> dict[str, Any]:
    return {
        "artifact_version": MECHANICAL_BEHAVIOR_TRACE_VERSION,
        "evidence_version": MECHANICAL_BEHAVIOR_EVIDENCE_VERSION,
        "robot_name": robot_name,
        "steps_run": len(step_results),
        "trace": _mechanical_step_trace_entries(step_results, actions),
    }


def _write_mechanical_trace_artifact(
    *,
    robot_name: str | None,
    step_results: list[dict[str, Any]],
    actions: list[Any],
    output: Path | None,
) -> str | None:
    if output is None:
        return None
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            _mechanical_trace_artifact(
                robot_name=robot_name,
                step_results=step_results,
                actions=actions,
            ),
            indent=2,
            ensure_ascii=False,
        ),
        encoding="utf-8",
    )
    return str(output)


def _center_of_mass_for_body_states(body_states: Any) -> dict[str, Any] | None:
    if not isinstance(body_states, dict):
        return None
    weighted_position = [0.0, 0.0, 0.0]
    weighted_velocity = [0.0, 0.0, 0.0]
    total_mass = 0.0
    included_parts: list[str] = []
    missing_parts: list[str] = []
    for part_id, state in body_states.items():
        if not isinstance(state, dict):
            missing_parts.append(str(part_id))
            continue
        position = state.get("position")
        mass = state.get("mass")
        if not _is_vector3(position) or not _is_number(mass) or float(mass) <= 0.0:
            missing_parts.append(str(part_id))
            continue
        part_mass = float(mass)
        total_mass += part_mass
        included_parts.append(str(part_id))
        for axis in range(3):
            weighted_position[axis] += float(position[axis]) * part_mass
        velocity = state.get("linear_velocity")
        if _is_vector3(velocity):
            for axis in range(3):
                weighted_velocity[axis] += float(velocity[axis]) * part_mass
    if total_mass <= 0.0:
        return None
    return {
        "center_of_mass": [value / total_mass for value in weighted_position],
        "center_of_mass_velocity": [
            value / total_mass for value in weighted_velocity
        ],
        "total_mass": total_mass,
        "included_part_count": len(included_parts),
        "included_part_ids": included_parts[:20],
        "missing_part_count": len(missing_parts),
        "missing_part_ids": missing_parts[:20],
        "complete": not missing_parts,
    }


def _center_of_mass_evidence(step_results: list[dict[str, Any]]) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    for index, step in enumerate(step_results):
        sample = _center_of_mass_for_body_states(step.get("body_states", {}))
        if sample is None:
            continue
        sample["step_index"] = index
        samples.append(sample)
    if not samples:
        return {
            "available": False,
            "source": "not_reported_by_godot_smoke",
            "sample_count": 0,
            "current_center_of_mass": None,
            "current_center_of_mass_velocity": None,
            "stability_summary": {},
        }

    first = samples[0]
    current = samples[-1]
    first_position = first["center_of_mass"]
    current_position = current["center_of_mass"]
    horizontal_displacement = (
        (current_position[0] - first_position[0]) ** 2
        + (current_position[2] - first_position[2]) ** 2
    ) ** 0.5
    heights = [sample["center_of_mass"][1] for sample in samples]
    speed_values = [
        _vector_length(sample.get("center_of_mass_velocity"))
        for sample in samples
    ]
    speeds = [value for value in speed_values if value is not None]
    return {
        "available": True,
        "source": "body_states_mass_position",
        "sample_count": len(samples),
        "current_center_of_mass": current_position,
        "current_center_of_mass_velocity": current["center_of_mass_velocity"],
        "total_mass": current["total_mass"],
        "included_part_count": current["included_part_count"],
        "missing_part_count": current["missing_part_count"],
        "missing_part_ids": current["missing_part_ids"],
        "complete": all(sample["complete"] for sample in samples),
        "stability_summary": {
            "first_center_of_mass": first_position,
            "current_center_of_mass": current_position,
            "horizontal_displacement": horizontal_displacement,
            "min_height": min(heights),
            "max_height": max(heights),
            "height_range": max(heights) - min(heights),
            "max_center_of_mass_speed": max(speeds) if speeds else None,
        },
    }


def _contact_count_from_value(value: Any) -> int | None:
    if isinstance(value, bool):
        return 1 if value else 0
    if _is_number(value):
        return max(0, int(value))
    if isinstance(value, list):
        return len(value)
    if isinstance(value, dict):
        for key in [
            "count",
            "contact_count",
            "points_count",
            "contact_points_count",
        ]:
            candidate = value.get(key)
            if _is_number(candidate):
                return max(0, int(candidate))
        for key in [
            "active",
            "contact",
            "contacting",
            "in_contact",
            "on_floor",
            "ground_contact",
            "support_contact",
        ]:
            candidate = value.get(key)
            if isinstance(candidate, bool):
                return 1 if candidate else 0
        for key in ["points", "contact_points", "colliders", "contacts"]:
            candidate = value.get(key)
            if isinstance(candidate, list):
                return len(candidate)
    return None


def _contact_entries_from_mapping(mapping: Any) -> list[dict[str, Any]]:
    if not isinstance(mapping, dict):
        return []
    entries: list[dict[str, Any]] = []
    for part_id, state in mapping.items():
        if not isinstance(state, dict):
            contact_count = _contact_count_from_value(state)
            if contact_count is None:
                continue
            entries.append(
                {
                    "part_id": str(part_id),
                    "in_contact": contact_count > 0,
                    "contact_count": contact_count,
                    "source_fields": ["value"],
                }
            )
            continue
        counts: list[int] = []
        source_fields: list[str] = []
        for field in [
            "contact",
            "contacts",
            "contact_count",
            "contacting",
            "in_contact",
            "on_floor",
            "ground_contact",
            "support_contact",
            "contact_points",
        ]:
            if field not in state:
                continue
            contact_count = _contact_count_from_value(state.get(field))
            if contact_count is None:
                continue
            counts.append(contact_count)
            source_fields.append(field)
        if not source_fields:
            continue
        entries.append(
            {
                "part_id": str(part_id),
                "in_contact": any(count > 0 for count in counts),
                "contact_count": max(counts) if counts else 0,
                "source_fields": source_fields,
            }
        )
    return entries


def _contact_state_evidence(step_results: list[dict[str, Any]]) -> dict[str, Any]:
    samples: list[dict[str, Any]] = []
    for index, step in enumerate(step_results):
        entries = _contact_entries_from_mapping(step.get("body_states", {}))
        top_level_entries = _contact_entries_from_mapping(step.get("contacts", {}))
        if top_level_entries:
            entries.extend(top_level_entries)
        if not entries:
            continue
        support_parts = sorted(
            {entry["part_id"] for entry in entries if entry["in_contact"]}
        )
        samples.append(
            {
                "step_index": index,
                "contact_part_count": len(support_parts),
                "support_parts": support_parts[:20],
                "entries": entries[:20],
                "entries_truncated": len(entries) > 20,
            }
        )
    if not samples:
        return {
            "available": False,
            "source": "not_reported_by_godot_smoke",
            "sample_count": 0,
            "support_part_count": 0,
            "support_parts": [],
            "current_contacts": [],
        }
    current = samples[-1]
    return {
        "available": True,
        "source": "runtime_contact_telemetry",
        "sample_count": len(samples),
        "support_part_count": current["contact_part_count"],
        "support_parts": current["support_parts"],
        "current_contacts": current["entries"],
        "samples": samples[:20],
        "samples_truncated": len(samples) > 20,
    }


def _mechanical_behavior_evidence(
    *,
    limit_summary: dict[str, Any],
    control_summary: dict[str, Any],
    action_target_coverage_summary: dict[str, Any],
    control_action_coverage_summary: dict[str, Any],
    simulation_summary: dict[str, Any],
    joint_motion_summary: dict[str, Any],
    step_results: list[dict[str, Any]],
    actions: list[Any],
    trace_artifact_path: str | None = None,
) -> dict[str, Any]:
    center_of_mass_evidence = _center_of_mass_evidence(step_results)
    contact_state_evidence = _contact_state_evidence(step_results)
    missing_sections: list[str] = []
    residual_risks: list[str] = []
    if not center_of_mass_evidence["available"]:
        missing_sections.append("center_of_mass")
        residual_risks.append("center_of_mass_runtime_readback_missing")
    if not contact_state_evidence["available"]:
        missing_sections.append("contact_state")
        residual_risks.append("contact_state_runtime_readback_missing")
    threshold_failures: list[str] = []
    if int(limit_summary.get("violation_count") or 0) > 0:
        threshold_failures.append("joint_limit_violation")
    for field, code in [
        ("angle_delta_under_min", "joint_angle_delta_under_min"),
        ("angle_range_under_min", "joint_angle_range_under_min"),
        ("moving_joint_coverage_under_min", "moving_joint_coverage_under_min"),
        ("commanded_joint_response_under_min", "commanded_joint_response_under_min"),
    ]:
        if joint_motion_summary.get(field):
            threshold_failures.append(code)
    if action_target_coverage_summary.get("coverage_under_min"):
        threshold_failures.append("action_target_coverage_under_min")
    if control_action_coverage_summary.get("coverage_under_min"):
        threshold_failures.append("control_action_coverage_under_min")

    return {
        "evidence_version": MECHANICAL_BEHAVIOR_EVIDENCE_VERSION,
        "units": {
            "joint_angle": "radians",
            "linear_distance": "meters",
            "linear_speed": "meters_per_second",
        },
        "available_sections": [
            "joint_limits",
            "torque_velocity_response",
            *(
                ["center_of_mass"]
                if center_of_mass_evidence["available"]
                else []
            ),
            *(
                ["contact_state"]
                if contact_state_evidence["available"]
                else []
            ),
            "step_trace",
        ],
        "missing_sections": missing_sections,
        "residual_risks": residual_risks,
        "threshold_failures": threshold_failures,
        "complete": not residual_risks and not threshold_failures,
        "joint_limit_evidence": {
            "available": bool(limit_summary),
            "joint_count": limit_summary.get("joint_count", 0),
            "checked_count": limit_summary.get("checked_count", 0),
            "violation_count": limit_summary.get("violation_count", 0),
            "violations": limit_summary.get("violations", []),
        },
        "torque_velocity_response_evidence": {
            "available": bool(joint_motion_summary),
            "targeted_joint_count": control_summary.get("targeted_count", 0),
            "max_abs_target_velocity": control_summary.get("max_abs_target_velocity"),
            "commanded_joint_count": joint_motion_summary.get("commanded_joint_count", 0),
            "commanded_moving_joint_count": joint_motion_summary.get(
                "commanded_moving_joint_count", 0
            ),
            "commanded_joint_response_ratio": joint_motion_summary.get(
                "commanded_joint_response_ratio", 0.0
            ),
            "min_commanded_joint_response_coverage_threshold": joint_motion_summary.get(
                "min_commanded_joint_response_coverage_threshold"
            ),
            "commanded_joint_response_under_min": joint_motion_summary.get(
                "commanded_joint_response_under_min", False
            ),
            "action_target_coverage_ratio": action_target_coverage_summary.get(
                "coverage_ratio", 0.0
            ),
            "control_action_coverage_ratio": control_action_coverage_summary.get(
                "coverage_ratio", 0.0
            ),
            "joint_motion_epsilon": joint_motion_summary.get("joint_motion_epsilon"),
            "max_abs_relative_angle_range": joint_motion_summary.get(
                "max_abs_relative_angle_range"
            ),
            "commanded_joint_response_details": joint_motion_summary.get(
                "commanded_joint_response_details", []
            ),
        },
        "center_of_mass_evidence": {
            **center_of_mass_evidence,
            "motion_summary": simulation_summary,
        },
        "contact_state_evidence": contact_state_evidence,
        "step_trace_evidence": _step_trace_evidence(
            step_results,
            actions,
            artifact_path=trace_artifact_path,
        ),
    }


def _profile_contract(
    *,
    profile_name: str,
    artifact_root: str | None,
    retention_days: int | None,
    flaky_retry_attempts: int | None,
) -> dict[str, Any]:
    profile = LIVE_VERIFICATION_PROFILES[profile_name]
    resolved_artifact_root = artifact_root or str(profile["default_artifact_root"])
    resolved_retention_days = (
        int(retention_days)
        if retention_days is not None
        else int(profile["retention_days"])
    )
    resolved_retry_attempts = (
        int(flaky_retry_attempts)
        if flaky_retry_attempts is not None
        else int(profile["flaky_retry_attempts"])
    )
    return {
        "profile_version": LIVE_VERIFICATION_PROFILE_VERSION,
        "profile_name": profile_name,
        "environment_mode": profile["environment_mode"],
        "ci_mode": profile["ci_mode"],
        "scheduled": profile["scheduled"],
        "artifact_retention": {
            "artifact_root": resolved_artifact_root,
            "retention_days": resolved_retention_days,
            "required_artifacts": [
                "smoke_report",
                "generation_report",
                "delivery_gate",
                "release_readiness",
            ],
        },
        "flaky_policy": {
            "retry_attempts": resolved_retry_attempts,
            "classification": "not_retried",
            "attempts_recorded": 1,
        },
    }


def _resolve_godot_executable(godot_exe: str) -> dict[str, Any]:
    env_value = os.environ.get("GODOT_EXECUTABLE")
    requested = env_value if godot_exe == DEFAULT_GODOT_EXE and env_value else godot_exe
    source = "GODOT_EXECUTABLE" if requested == env_value and env_value else "argument"
    which_result = shutil.which(requested)
    candidate = Path(which_result) if which_result else Path(requested)
    exists = candidate.exists()
    is_file = candidate.is_file()
    resolved_path = str(candidate) if exists or which_result else None
    return {
        "requested": requested,
        "source": source if not which_result else "PATH",
        "resolved_path": resolved_path,
        "exists": exists,
        "is_file": is_file,
        "failure_category": None if exists and is_file else "missing_godot_executable",
    }


def _live_verification_metadata(
    args: argparse.Namespace,
    *,
    failure_category: str | None = None,
) -> dict[str, Any]:
    profile = _profile_contract(
        profile_name=args.live_profile,
        artifact_root=str(args.live_artifact_root) if args.live_artifact_root else None,
        retention_days=args.live_retention_days,
        flaky_retry_attempts=args.flaky_retry_attempts,
    )
    executable = _resolve_godot_executable(args.godot_exe)
    return {
        **profile,
        "godot_executable": executable,
        "failure_category": failure_category or executable.get("failure_category"),
        "dry_run": bool(args.dry_run_discovery),
    }


def _write_and_print_report(report: dict[str, Any], output: Path | None) -> None:
    if output:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(report, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
    print(json.dumps(report, indent=2, ensure_ascii=False))


def _build_discovery_report(
    *,
    robot_config: dict[str, Any],
    live_verification: dict[str, Any],
) -> dict[str, Any]:
    failure_category = live_verification.get("failure_category")
    return {
        "status": "blocked" if failure_category else "success",
        "robot_name": robot_config.get("name"),
        "live_verification": live_verification,
        "errors": (
            [f"live verification discovery failed: {failure_category}"]
            if failure_category
            else []
        ),
    }


def _build_runtime_failure_report(
    *,
    robot_config: dict[str, Any],
    live_verification: dict[str, Any],
    error: Exception,
    stdout: str = "",
    stderr: str = "",
) -> dict[str, Any]:
    failure_category = live_verification.get("failure_category") or "godot_runtime_failure"
    return {
        "status": "error",
        "robot_name": robot_config.get("name"),
        "live_verification": live_verification,
        "failure_detail": {
            "category": failure_category,
            "exception_type": type(error).__name__,
            "message": str(error),
        },
        "errors": [
            f"live verification failed: {failure_category}: {error}",
        ],
        "stdout_tail": stdout.splitlines()[-20:],
        "stderr_tail": stderr.splitlines()[-20:],
    }


def _send_command(port: int, payload: dict[str, Any]) -> dict[str, Any]:
    message = json.dumps(payload).encode("utf-8")
    with socket.create_connection(("127.0.0.1", port), timeout=5) as sock:
        sock.settimeout(5)
        sock.sendall(struct.pack("<I", len(message)) + message)
        stream = sock.makefile("rb")
        header = stream.read(4)
        if len(header) != 4:
            raise RuntimeError("Godot TCP server did not return a response header")
        size = struct.unpack("<I", header)[0]
        response = stream.read(size)
    return json.loads(response.decode("utf-8"))


def _send_command_with_retry(
    port: int,
    payload: dict[str, Any],
    timeout_seconds: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_seconds
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return _send_command(port, payload)
        except OSError as exc:
            last_error = exc
            time.sleep(0.1)
    raise TimeoutError(f"Godot TCP server did not respond on port {port}: {last_error}")


def _launch_godot(args: argparse.Namespace) -> subprocess.Popen[str]:
    executable = _resolve_godot_executable(args.godot_exe)
    godot_exe = executable.get("resolved_path") or executable.get("requested")
    command = [
        str(godot_exe),
        "--headless",
        "--path",
        str(args.project),
        "--quit-after",
        "0",
    ]
    if args.scene:
        command.extend(["--scene", args.scene])
    command.extend(["--", f"--tcp-port={args.port}"])
    return subprocess.Popen(
        command,
        cwd=args.project,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def _build_result(
    *,
    robot_config: dict[str, Any],
    load_result: dict[str, Any],
    schema_result: dict[str, Any],
    step_result: dict[str, Any],
    step_results: list[dict[str, Any]],
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
    fail_on_action_target_mismatch: bool,
    fail_on_action_sequence_target_mismatch: bool,
    fail_on_unknown_action_target: bool,
    fail_on_invalid_action_target: bool,
    fail_on_incomplete_node_tree: bool,
    fail_on_node_tree_class_mismatch: bool,
    fail_on_node_tree_missing_parameters: bool,
    fail_on_node_tree_transform_mismatch: bool,
    fail_on_node_tree_physical_mismatch: bool,
    fail_on_node_tree_fixed_lock_mismatch: bool,
    node_tree_tolerance: float,
    parameter_tolerance: float,
    action: Any,
    actions: list[Any],
    stdout: str,
    stderr: str,
    live_verification: dict[str, Any] | None = None,
    trace_artifact_path: str | None = None,
) -> dict[str, Any]:
    expected_parts = len(robot_config.get("parts", []))
    expected_joints = len(robot_config.get("connections", []))
    assembly_summary = schema_result.get("meta", {}).get("assembly_summary", {})
    part_nodes = load_result.get("part_nodes", assembly_summary.get("part_nodes", []))
    joint_nodes = load_result.get("joint_nodes", assembly_summary.get("joint_nodes", []))
    body_states = step_result.get("body_states", {})
    joint_states = step_result.get("joint_states", {})
    endpoint_summary = _joint_endpoint_summary(joint_states, max_endpoint_distance)
    angle_summary = _joint_angle_summary(joint_states, max_relative_angle)
    limit_summary = _joint_limit_summary(joint_states)
    parameter_summary = _joint_parameter_summary(joint_states)
    parameter_consistency_summary = _joint_parameter_consistency_summary(
        joint_states,
        tolerance=parameter_tolerance,
    )
    control_summary = _joint_control_summary(joint_states)
    control_summary["min_nonzero_target_threshold"] = min_nonzero_action_targets
    control_summary["nonzero_targets_under_min"] = (
        min_nonzero_action_targets is not None
        and int(control_summary.get("targeted_count") or 0) < min_nonzero_action_targets
    )
    control_consistency_summary = _joint_control_consistency_summary(
        robot_config,
        joint_states,
        tolerance=parameter_tolerance,
    )
    action_target_consistency_summary = _action_target_consistency_summary(
        joint_states,
        actions[-1] if actions else action,
        tolerance=parameter_tolerance,
    )
    action_sequence_target_consistency_summary = _action_sequence_target_consistency_summary(
        step_results,
        actions,
        fallback_action=action,
        tolerance=parameter_tolerance,
    )
    action_target_coverage_summary = _action_target_coverage_summary(
        joint_states,
        actions,
        fallback_action=action,
        min_coverage=min_action_target_coverage,
    )
    control_action_coverage_summary = _control_action_coverage_summary(
        robot_config,
        joint_states,
        actions,
        fallback_action=action,
        min_coverage=min_control_action_coverage,
    )
    simulation_summary = _simulation_summary(
        step_results,
        min_body_displacement=min_body_displacement,
        max_linear_speed=max_linear_speed,
    )
    joint_motion_summary = _joint_motion_summary(
        step_results,
        min_joint_angle_delta=min_joint_angle_delta,
        min_joint_angle_range=min_joint_angle_range,
        min_moving_joint_coverage=min_moving_joint_coverage,
        min_commanded_joint_response_coverage=min_commanded_joint_response_coverage,
        joint_motion_epsilon=joint_motion_epsilon,
        actions=actions,
    )
    action_sequence_summary = _action_sequence_summary(
        actions,
        min_transitions=min_action_transitions,
        min_transition_delta=min_action_transition_delta,
    )
    node_tree_manifest = _node_tree_manifest(
        robot_config=robot_config,
        part_nodes=part_nodes,
        joint_nodes=joint_nodes,
        tolerance=node_tree_tolerance,
    )
    static_node_tree_manifest = robot_schema.build_godot_node_tree_manifest(robot_config)
    static_manifest_comparison = (
        robot_schema.compare_godot_node_tree_manifest_to_runtime(
            static_node_tree_manifest,
            part_nodes,
            joint_nodes,
            tolerance=node_tree_tolerance,
        )
    )
    node_tree_manifest["static_manifest_version"] = static_node_tree_manifest.get(
        "manifest_version"
    )
    node_tree_manifest["static_manifest_comparison"] = static_manifest_comparison
    restoration_summary = _mechanical_restoration_summary(
        expected_parts=expected_parts,
        expected_joints=expected_joints,
        part_nodes=part_nodes,
        joint_nodes=joint_nodes,
        body_states=body_states,
        joint_states=joint_states,
        parameter_summary=parameter_summary,
        steps_run=len(step_results),
        min_restoration_score=min_restoration_score,
    )
    errors = []
    if load_result.get("status") != "success":
        errors.append("load_robot did not return success")
    if load_result.get("parts_created") != expected_parts:
        errors.append("parts_created does not match config parts count")
    if load_result.get("joints_created") != expected_joints:
        errors.append("joints_created does not match config connections count")
    if not schema_result.get("meta", {}).get("dynamic_robot_generation"):
        errors.append("schema did not report dynamic_robot_generation")
    if step_result.get("body_count") != expected_parts:
        errors.append("step telemetry body_count does not match config parts count")
    if len(body_states) != expected_parts:
        errors.append("step telemetry body_states does not match config parts count")
    if len(joint_states) != expected_joints:
        errors.append("step telemetry joint_states does not match config connections count")
    if joint_states and not _joint_endpoint_telemetry_is_complete(joint_states):
        errors.append("joint_states endpoint telemetry is incomplete")
    if endpoint_summary.get("threshold_exceeded"):
        errors.append(
            "joint endpoint distance threshold exceeded: "
            f"{endpoint_summary.get('max_endpoint_distance')} > "
            f"{endpoint_summary.get('max_endpoint_distance_threshold')}"
        )
    if angle_summary.get("threshold_exceeded"):
        errors.append(
            "joint relative angle threshold exceeded: "
            f"{angle_summary.get('max_abs_relative_angle')} > "
            f"{angle_summary.get('max_relative_angle_threshold')}"
        )
    if fail_on_joint_limit_violation and limit_summary.get("violation_count", 0) > 0:
        errors.append(
            "joint limit violation detected: "
            f"{limit_summary.get('violation_count')} joints exceeded configured limits"
        )
    if simulation_summary.get("displacement_under_min"):
        errors.append(
            "body displacement below minimum: "
            f"{simulation_summary.get('max_body_displacement')} < "
            f"{simulation_summary.get('min_body_displacement_threshold')}"
        )
    if simulation_summary.get("speed_threshold_exceeded"):
        errors.append(
            "linear speed threshold exceeded: "
            f"{simulation_summary.get('max_linear_speed')} > "
            f"{simulation_summary.get('max_linear_speed_threshold')}"
        )
    if joint_motion_summary.get("angle_delta_under_min"):
        errors.append(
            "joint angle delta below minimum: "
            f"{joint_motion_summary.get('max_abs_relative_angle_delta')} < "
            f"{joint_motion_summary.get('min_joint_angle_delta_threshold')}"
        )
    if joint_motion_summary.get("angle_range_under_min"):
        errors.append(
            "joint angle range below minimum: "
            f"{joint_motion_summary.get('max_abs_relative_angle_range')} < "
            f"{joint_motion_summary.get('min_joint_angle_range_threshold')}"
        )
    if joint_motion_summary.get("moving_joint_coverage_under_min"):
        errors.append(
            "moving joint coverage below minimum: "
            f"{joint_motion_summary.get('moving_joint_coverage_ratio')} < "
            f"{joint_motion_summary.get('min_moving_joint_coverage_threshold')}"
        )
    if joint_motion_summary.get("commanded_joint_response_under_min"):
        errors.append(
            "commanded joint response coverage below minimum: "
            f"{joint_motion_summary.get('commanded_joint_response_ratio')} < "
            f"{joint_motion_summary.get('min_commanded_joint_response_coverage_threshold')}"
        )
    if fail_on_incomplete_restoration and not restoration_summary.get("complete"):
        errors.append("mechanical restoration is incomplete")
    if restoration_summary.get("score_under_min"):
        errors.append(
            "mechanical restoration score below minimum: "
            f"{restoration_summary.get('score')} < "
            f"{restoration_summary.get('min_restoration_score_threshold')}"
        )
    if fail_on_parameter_mismatch and parameter_consistency_summary.get("mismatch_count", 0) > 0:
        errors.append(
            "joint parameter mismatch detected: "
            f"{parameter_consistency_summary.get('mismatch_count')} mismatches"
        )
    if fail_on_control_mismatch and control_consistency_summary.get("mismatch_count", 0) > 0:
        errors.append(
            "joint control parameter mismatch detected: "
            f"{control_consistency_summary.get('mismatch_count')} mismatches"
        )
    if (
        min_nonzero_action_targets is not None
        and int(control_summary.get("targeted_count") or 0) < min_nonzero_action_targets
    ):
        errors.append(
            "nonzero action targets below minimum: "
            f"{control_summary.get('targeted_count')} < {min_nonzero_action_targets}"
        )
    if action_sequence_summary.get("transitions_under_min"):
        errors.append(
            "action transitions below minimum: "
            f"{action_sequence_summary.get('transition_count')} < "
            f"{action_sequence_summary.get('min_transition_threshold')}"
        )
    if action_sequence_summary.get("transition_delta_under_min"):
        errors.append(
            "action transition delta below minimum: "
            f"{action_sequence_summary.get('max_numeric_transition_delta')} < "
            f"{action_sequence_summary.get('min_transition_delta_threshold')}"
        )
    if fail_on_action_target_mismatch and action_target_consistency_summary.get("mismatch_count", 0) > 0:
        errors.append(
            "action target velocity mismatch detected: "
            f"{action_target_consistency_summary.get('mismatch_count')} mismatches"
        )
    if (
        fail_on_action_sequence_target_mismatch
        and action_sequence_target_consistency_summary.get("mismatch_count", 0) > 0
    ):
        errors.append(
            "action sequence target velocity mismatch detected: "
            f"{action_sequence_target_consistency_summary.get('mismatch_count')} mismatches"
        )
    unknown_action_targets = _effective_unknown_action_target_count(
        action_target_consistency_summary,
        action_sequence_target_consistency_summary,
    )
    if fail_on_unknown_action_target and unknown_action_targets > 0:
        errors.append(
            "unknown action target detected: "
            f"{unknown_action_targets} targets do not map to generated joints"
        )
    invalid_action_targets = _effective_invalid_action_target_count(
        action_target_consistency_summary,
        action_sequence_target_consistency_summary,
    )
    if fail_on_invalid_action_target and invalid_action_targets > 0:
        errors.append(
            "invalid action target detected: "
            f"{invalid_action_targets} targets are not numeric"
        )
    if action_target_coverage_summary.get("coverage_under_min"):
        errors.append(
            "action target coverage below minimum: "
            f"{action_target_coverage_summary.get('coverage_ratio')} < "
            f"{action_target_coverage_summary.get('min_coverage_threshold')}"
        )
    if control_action_coverage_summary.get("coverage_under_min"):
        errors.append(
            "control action coverage below minimum: "
            f"{control_action_coverage_summary.get('coverage_ratio')} < "
            f"{control_action_coverage_summary.get('min_coverage_threshold')}"
        )
    if fail_on_incomplete_node_tree and not node_tree_manifest.get("complete"):
        errors.append(
            "node tree restoration is incomplete: "
            f"{len(node_tree_manifest.get('missing_part_ids', []))} missing parts, "
            f"{len(node_tree_manifest.get('missing_connection_names', []))} missing joints"
        )
    if fail_on_node_tree_class_mismatch and node_tree_manifest.get("class_mismatch_count", 0) > 0:
        errors.append(
            "node tree class mismatch detected: "
            f"{node_tree_manifest.get('class_mismatch_count')} mismatches"
        )
    if fail_on_node_tree_missing_parameters and node_tree_manifest.get("parameter_missing_count", 0) > 0:
        errors.append(
            "node tree joint parameters missing: "
            f"{node_tree_manifest.get('parameter_missing_count')} joints"
        )
    if fail_on_node_tree_transform_mismatch and node_tree_manifest.get("transform_mismatch_count", 0) > 0:
        errors.append(
            "node tree transform mismatch detected: "
            f"{node_tree_manifest.get('transform_mismatch_count')} mismatches"
        )
    if fail_on_node_tree_physical_mismatch and node_tree_manifest.get("physical_mismatch_count", 0) > 0:
        errors.append(
            "node tree physical parameter mismatch detected: "
            f"{node_tree_manifest.get('physical_mismatch_count')} mismatches"
        )
    if fail_on_node_tree_fixed_lock_mismatch and node_tree_manifest.get("fixed_lock_mismatch_count", 0) > 0:
        errors.append(
            "node tree fixed joint lock mismatch detected: "
            f"{node_tree_manifest.get('fixed_lock_mismatch_count')} mismatches"
        )
    if len(part_nodes) != expected_parts:
        errors.append("part_nodes mapping count does not match config parts count")
    if len(joint_nodes) != expected_joints:
        errors.append("joint_nodes mapping count does not match config connections count")
    node_tree_gate_summary = _node_tree_gate_summary(
        fail_on_incomplete_node_tree=fail_on_incomplete_node_tree,
        fail_on_node_tree_class_mismatch=fail_on_node_tree_class_mismatch,
        fail_on_node_tree_missing_parameters=fail_on_node_tree_missing_parameters,
        fail_on_node_tree_transform_mismatch=fail_on_node_tree_transform_mismatch,
        fail_on_node_tree_physical_mismatch=fail_on_node_tree_physical_mismatch,
        fail_on_node_tree_fixed_lock_mismatch=fail_on_node_tree_fixed_lock_mismatch,
    )
    mechanical_gate_summary = _mechanical_gate_summary(
        fail_on_incomplete_restoration=fail_on_incomplete_restoration,
        fail_on_parameter_mismatch=fail_on_parameter_mismatch,
        fail_on_control_mismatch=fail_on_control_mismatch,
        node_tree_gate_summary=node_tree_gate_summary,
    )
    mechanical_behavior_evidence = _mechanical_behavior_evidence(
        limit_summary=limit_summary,
        control_summary=control_summary,
        action_target_coverage_summary=action_target_coverage_summary,
        control_action_coverage_summary=control_action_coverage_summary,
        simulation_summary=simulation_summary,
        joint_motion_summary=joint_motion_summary,
        step_results=step_results,
        actions=actions,
        trace_artifact_path=trace_artifact_path,
    )
    return {
        "status": "error" if errors else "success",
        "robot_name": robot_config.get("name"),
        "expected_parts": expected_parts,
        "expected_joints": expected_joints,
        "load_result": load_result,
        "schema_meta": schema_result.get("meta", {}),
        "mapping_summary": {
            "part_nodes": len(part_nodes),
            "joint_nodes": len(joint_nodes),
            "first_part": part_nodes[0] if part_nodes else {},
            "first_joint": joint_nodes[0] if joint_nodes else {},
        },
        "mechanical_gate_summary": mechanical_gate_summary,
        "node_tree_gate_summary": node_tree_gate_summary,
        "live_verification": live_verification or {},
        "node_tree_manifest": node_tree_manifest,
        "mechanical_behavior_evidence": mechanical_behavior_evidence,
        "step_summary": {
            "body_count": step_result.get("body_count"),
            "joint_count": step_result.get("joint_count"),
            "body_states": len(body_states),
            "joint_states": len(joint_states),
            "first_body_state": _first_mapping_value(body_states),
            "first_joint_state": _first_mapping_value(joint_states),
            "joint_endpoint_summary": endpoint_summary,
            "joint_angle_summary": angle_summary,
            "joint_limit_summary": limit_summary,
            "joint_parameter_summary": parameter_summary,
            "joint_parameter_consistency_summary": parameter_consistency_summary,
            "joint_control_summary": control_summary,
            "joint_control_consistency_summary": control_consistency_summary,
            "action_target_consistency_summary": action_target_consistency_summary,
            "action_sequence_target_consistency_summary": action_sequence_target_consistency_summary,
            "action_target_coverage_summary": action_target_coverage_summary,
            "control_action_coverage_summary": control_action_coverage_summary,
            "simulation_summary": simulation_summary,
            "joint_motion_summary": joint_motion_summary,
            "action_sequence_summary": action_sequence_summary,
            "mechanical_restoration_summary": restoration_summary,
            "action_sent": action,
            "first_action_sent": actions[0] if actions else action,
            "last_action_sent": actions[-1] if actions else action,
            "steps_run": len(step_results),
            "reward": step_result.get("reward"),
            "done": step_result.get("done"),
        },
        "errors": errors,
        "stdout_tail": stdout.splitlines()[-20:],
        "stderr_tail": stderr.splitlines()[-20:],
    }


def _joint_parameter_consistency_summary(
    joint_states: Any,
    *,
    tolerance: float = 1e-4,
) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "joint_count": 0,
            "checked_count": 0,
            "mismatch_count": 0,
            "mismatches": [],
            "tolerance": tolerance,
            "complete": False,
        }

    mismatches: list[dict[str, Any]] = []
    checked_count = 0
    fixed_lock_checked_count = 0
    for joint_name, state in joint_states.items():
        if not isinstance(state, dict):
            continue
        applied = state.get("applied_parameters")
        if not isinstance(applied, dict):
            continue
        source = applied.get("source")
        runtime = applied.get("runtime")
        if not isinstance(source, dict) or not isinstance(runtime, dict):
            continue
        checked_count += 1
        _compare_joint_parameter(
            mismatches,
            joint_name=str(joint_name),
            field="limits.lower",
            expected=_nested_get(source, ["limits", "lower"]),
            actual=runtime.get("limit_lower"),
            tolerance=tolerance,
        )
        _compare_joint_parameter(
            mismatches,
            joint_name=str(joint_name),
            field="limits.upper",
            expected=_nested_get(source, ["limits", "upper"]),
            actual=runtime.get("limit_upper"),
            tolerance=tolerance,
        )
        _compare_joint_parameter(
            mismatches,
            joint_name=str(joint_name),
            field="motor.enabled",
            expected=_nested_get(source, ["motor", "enabled"]),
            actual=runtime.get("motor_enabled"),
            tolerance=tolerance,
        )
        _compare_joint_parameter(
            mismatches,
            joint_name=str(joint_name),
            field="motor.target_velocity",
            expected=_nested_get(source, ["motor", "target_velocity"]),
            actual=runtime.get("motor_target_velocity"),
            compare_runtime_target=False,
            tolerance=tolerance,
        )
        _compare_joint_parameter(
            mismatches,
            joint_name=str(joint_name),
            field="motor.max_impulse",
            expected=_nested_get(source, ["motor", "max_impulse"]),
            actual=runtime.get("motor_max_impulse"),
            tolerance=tolerance,
        )
        if runtime.get("fixed_approximation") is True or runtime.get("fixed_lock_applied") is not None:
            _compare_fixed_joint_lock(
                mismatches,
                joint_name=str(joint_name),
                runtime=runtime,
                tolerance=tolerance,
            )
            fixed_lock_checked_count += 1

    return {
        "joint_count": len(joint_states),
        "checked_count": checked_count,
        "fixed_lock_checked_count": fixed_lock_checked_count,
        "fixed_lock_mismatch_count": sum(
            1 for mismatch in mismatches if str(mismatch.get("field", "")).startswith("fixed.")
        ),
        "mismatch_count": len(mismatches),
        "mismatches": mismatches[:20],
        "tolerance": tolerance,
        "complete": checked_count == len(joint_states) and not mismatches,
    }


def _node_tree_gate_summary(
    *,
    fail_on_incomplete_node_tree: bool,
    fail_on_node_tree_class_mismatch: bool,
    fail_on_node_tree_missing_parameters: bool,
    fail_on_node_tree_transform_mismatch: bool,
    fail_on_node_tree_physical_mismatch: bool,
    fail_on_node_tree_fixed_lock_mismatch: bool,
) -> dict[str, Any]:
    checks = {
        "incomplete_node_tree": fail_on_incomplete_node_tree,
        "class_mismatch": fail_on_node_tree_class_mismatch,
        "missing_parameters": fail_on_node_tree_missing_parameters,
        "transform_mismatch": fail_on_node_tree_transform_mismatch,
        "physical_mismatch": fail_on_node_tree_physical_mismatch,
        "fixed_lock_mismatch": fail_on_node_tree_fixed_lock_mismatch,
    }
    enabled_checks = [name for name, enabled in checks.items() if enabled]
    return {
        "checks": checks,
        "enabled_checks": enabled_checks,
        "enabled_count": len(enabled_checks),
        "full_node_tree_restoration_required": len(enabled_checks) == len(checks),
    }


def _mechanical_gate_summary(
    *,
    fail_on_incomplete_restoration: bool,
    fail_on_parameter_mismatch: bool,
    fail_on_control_mismatch: bool,
    node_tree_gate_summary: dict[str, Any],
) -> dict[str, Any]:
    checks = {
        "mechanical_restoration": fail_on_incomplete_restoration,
        "joint_parameter_readback": fail_on_parameter_mismatch,
        "control_parameter_readback": fail_on_control_mismatch,
        "full_node_tree_restoration": (
            node_tree_gate_summary.get("full_node_tree_restoration_required") is True
        ),
    }
    enabled_checks = [name for name, enabled in checks.items() if enabled]
    return {
        "checks": checks,
        "enabled_checks": enabled_checks,
        "enabled_count": len(enabled_checks),
        "full_mechanical_restoration_required": len(enabled_checks) == len(checks),
    }


def _compare_fixed_joint_lock(
    mismatches: list[dict[str, Any]],
    *,
    joint_name: str,
    runtime: dict[str, Any],
    tolerance: float,
) -> None:
    _compare_required_joint_parameter(
        mismatches,
        joint_name=joint_name,
        field="fixed.lock_applied",
        expected=True,
        actual=runtime.get("fixed_lock_applied"),
        tolerance=tolerance,
    )
    for group_name in ["linear_limit_enabled", "angular_limit_enabled"]:
        group = runtime.get(group_name)
        for axis in ["x", "y", "z"]:
            _compare_required_joint_parameter(
                mismatches,
                joint_name=joint_name,
                field=f"fixed.{group_name}.{axis}",
                expected=True,
                actual=group.get(axis) if isinstance(group, dict) else None,
                tolerance=tolerance,
            )
    for group_name in ["linear_lower", "linear_upper", "angular_lower", "angular_upper"]:
        group = runtime.get(group_name)
        for axis in ["x", "y", "z"]:
            _compare_required_joint_parameter(
                mismatches,
                joint_name=joint_name,
                field=f"fixed.{group_name}.{axis}",
                expected=0.0,
                actual=group.get(axis) if isinstance(group, dict) else None,
                tolerance=tolerance,
            )


def _compare_required_joint_parameter(
    mismatches: list[dict[str, Any]],
    *,
    joint_name: str,
    field: str,
    expected: Any,
    actual: Any,
    tolerance: float,
) -> None:
    if expected is None or actual is None:
        mismatches.append(
            {"joint": joint_name, "field": field, "expected": expected, "actual": actual}
        )
        return
    _compare_joint_parameter(
        mismatches,
        joint_name=joint_name,
        field=field,
        expected=expected,
        actual=actual,
        tolerance=tolerance,
    )


def _compare_joint_parameter(
    mismatches: list[dict[str, Any]],
    *,
    joint_name: str,
    field: str,
    expected: Any,
    actual: Any,
    compare_runtime_target: bool = True,
    tolerance: float = 1e-4,
) -> None:
    if expected is None or actual is None:
        return
    if not compare_runtime_target:
        return
    if isinstance(expected, bool) or isinstance(actual, bool):
        if bool(expected) != bool(actual):
            mismatches.append(
                {"joint": joint_name, "field": field, "expected": expected, "actual": actual}
            )
        return
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        if abs(float(expected) - float(actual)) > tolerance:
            mismatches.append(
                {"joint": joint_name, "field": field, "expected": expected, "actual": actual}
            )
        return
    if expected != actual:
        mismatches.append(
            {"joint": joint_name, "field": field, "expected": expected, "actual": actual}
        )


def _nested_get(value: Any, keys: list[str]) -> Any:
    current = value
    for key in keys:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def _node_tree_manifest(
    *,
    robot_config: dict[str, Any],
    part_nodes: Any,
    joint_nodes: Any,
    tolerance: float = 1e-4,
) -> dict[str, Any]:
    parts = robot_config.get("parts", [])
    connections = robot_config.get("connections", [])
    part_mappings = {
        str(item.get("part_id")): item
        for item in part_nodes
        if isinstance(item, dict) and item.get("part_id") is not None
    } if isinstance(part_nodes, list) else {}
    joint_mappings = {
        str(item.get("connection_name")): item
        for item in joint_nodes
        if isinstance(item, dict) and item.get("connection_name") is not None
    } if isinstance(joint_nodes, list) else {}

    manifest_parts = []
    missing_part_ids = []
    class_mismatches: list[dict[str, Any]] = []
    transform_mismatches: list[dict[str, Any]] = []
    physical_mismatches: list[dict[str, Any]] = []
    if isinstance(parts, list):
        for part in parts:
            if not isinstance(part, dict):
                continue
            part_id = str(part.get("id", ""))
            mapping = part_mappings.get(part_id, {})
            restored = bool(mapping)
            if not restored:
                missing_part_ids.append(part_id)
            expected_collision_shape = _expected_collision_shape(part.get("shape"))
            expected_mesh_type = _expected_mesh_type(part.get("shape"))
            actual_collision_shape = mapping.get("collision_shape")
            actual_mesh_type = mapping.get("mesh_type")
            if restored and actual_collision_shape != expected_collision_shape:
                class_mismatches.append(
                    {
                        "kind": "part",
                        "name": part_id,
                        "field": "collision_shape",
                        "expected": expected_collision_shape,
                        "actual": actual_collision_shape,
                    }
                )
            if restored and actual_mesh_type != expected_mesh_type:
                class_mismatches.append(
                    {
                        "kind": "part",
                        "name": part_id,
                        "field": "mesh_type",
                        "expected": expected_mesh_type,
                        "actual": actual_mesh_type,
                    }
                )
            params = part.get("params", {}) if isinstance(part.get("params"), dict) else {}
            if restored:
                expected_physical_parameters = _expected_part_physical_parameters(part)
                _compare_node_tree_value(
                    physical_mismatches,
                    kind="part",
                    name=part_id,
                    field="mass",
                    expected=expected_physical_parameters.get("mass"),
                    actual=mapping.get("mass"),
                    tolerance=tolerance,
                )
                _compare_node_tree_mapping(
                    physical_mismatches,
                    kind="part",
                    name=part_id,
                    prefix="collision_parameters",
                    expected=expected_physical_parameters.get("collision_parameters", {}),
                    actual=mapping.get("collision_parameters", {}),
                    tolerance=tolerance,
                )
                _compare_node_tree_mapping(
                    physical_mismatches,
                    kind="part",
                    name=part_id,
                    prefix="mesh_parameters",
                    expected=expected_physical_parameters.get("mesh_parameters", {}),
                    actual=mapping.get("mesh_parameters", {}),
                    tolerance=tolerance,
                )
                _compare_node_tree_vector(
                    transform_mismatches,
                    kind="part",
                    name=part_id,
                    field="position",
                    expected=params.get("position"),
                    actual=mapping.get("position"),
                    tolerance=tolerance,
                )
                _compare_node_tree_vector(
                    transform_mismatches,
                    kind="part",
                    name=part_id,
                    field="rotation",
                    expected=params.get("rotation"),
                    actual=mapping.get("rotation"),
                    tolerance=tolerance,
                )
            manifest_parts.append(
                {
                    "part_id": part_id,
                    "part_type": part.get("type"),
                    "shape": part.get("shape"),
                    "restored": restored,
                    "body_node": mapping.get("body_node"),
                    "body_class": mapping.get("body_class"),
                    "expected_collision_shape": expected_collision_shape,
                    "collision_shape": actual_collision_shape,
                    "expected_mesh_type": expected_mesh_type,
                    "mesh_type": actual_mesh_type,
                    "expected_physical_parameters": _expected_part_physical_parameters(part),
                    "collision_parameters": mapping.get("collision_parameters"),
                    "mesh_parameters": mapping.get("mesh_parameters"),
                    "mass": mapping.get("mass"),
                    "position": mapping.get("position"),
                    "rotation": mapping.get("rotation"),
                }
            )

    manifest_joints = []
    missing_connection_names = []
    missing_parameter_connection_names = []
    fixed_lock_checked_count = 0
    fixed_lock_mismatches: list[dict[str, Any]] = []
    if isinstance(connections, list):
        for index, connection in enumerate(connections):
            if not isinstance(connection, dict):
                continue
            connection_name = str(
                connection.get(
                    "name",
                    f"{connection.get('from')}_to_{connection.get('to')}_{index}",
                )
            )
            mapping = joint_mappings.get(connection_name, {})
            restored = bool(mapping)
            if not restored:
                missing_connection_names.append(connection_name)
            expected_joint_class = _expected_joint_class(connection.get("joint_type"))
            actual_joint_class = mapping.get("joint_class")
            if restored and actual_joint_class != expected_joint_class:
                class_mismatches.append(
                    {
                        "kind": "joint",
                        "name": connection_name,
                        "field": "joint_class",
                        "expected": expected_joint_class,
                        "actual": actual_joint_class,
                    }
                )
            has_applied_parameters = bool(mapping.get("applied_parameters"))
            if restored and not has_applied_parameters:
                missing_parameter_connection_names.append(connection_name)
            fixed_lock_checked = False
            fixed_lock_complete = None
            fixed_lock_mismatch_count = 0
            if restored:
                applied = mapping.get("applied_parameters")
                runtime = applied.get("runtime") if isinstance(applied, dict) else {}
                if not isinstance(runtime, dict):
                    runtime = {}
                fixed_lock_checked = (
                    connection.get("joint_type") == "fixed"
                    or runtime.get("fixed_approximation") is True
                    or runtime.get("fixed_lock_applied") is not None
                )
                if fixed_lock_checked:
                    fixed_lock_checked_count += 1
                    before_fixed_mismatches = len(fixed_lock_mismatches)
                    _compare_fixed_joint_lock(
                        fixed_lock_mismatches,
                        joint_name=connection_name,
                        runtime=runtime,
                        tolerance=tolerance,
                    )
                    fixed_lock_mismatch_count = (
                        len(fixed_lock_mismatches) - before_fixed_mismatches
                    )
                    fixed_lock_complete = fixed_lock_mismatch_count == 0
            if restored:
                _compare_node_tree_vector(
                    transform_mismatches,
                    kind="joint",
                    name=connection_name,
                    field="origin",
                    expected=connection.get("origin"),
                    actual=mapping.get("origin"),
                    tolerance=tolerance,
                )
                _compare_node_tree_vector(
                    transform_mismatches,
                    kind="joint",
                    name=connection_name,
                    field="axis",
                    expected=connection.get("axis"),
                    actual=mapping.get("axis"),
                    tolerance=tolerance,
                )
            manifest_joints.append(
                {
                    "connection_name": connection_name,
                    "joint_type": connection.get("joint_type"),
                    "from": connection.get("from"),
                    "to": connection.get("to"),
                    "restored": restored,
                    "joint_node": mapping.get("joint_node"),
                    "expected_joint_class": expected_joint_class,
                    "joint_class": actual_joint_class,
                    "node_a": mapping.get("node_a"),
                    "node_b": mapping.get("node_b"),
                    "origin": mapping.get("origin"),
                    "axis": mapping.get("axis"),
                    "has_applied_parameters": has_applied_parameters,
                    "fixed_lock_checked": fixed_lock_checked,
                    "fixed_lock_complete": fixed_lock_complete,
                    "fixed_lock_mismatch_count": fixed_lock_mismatch_count,
                }
            )

    return {
        "robot_name": robot_config.get("name"),
        "parts": manifest_parts,
        "joints": manifest_joints,
        "missing_part_ids": missing_part_ids,
        "missing_connection_names": missing_connection_names,
        "missing_parameter_connection_names": missing_parameter_connection_names,
        "parameter_missing_count": len(missing_parameter_connection_names),
        "parameters_complete": not missing_parameter_connection_names,
        "fixed_lock_checked_count": fixed_lock_checked_count,
        "fixed_lock_mismatch_count": len(fixed_lock_mismatches),
        "fixed_lock_mismatches": fixed_lock_mismatches[:20],
        "fixed_locks_complete": not fixed_lock_mismatches,
        "class_mismatch_count": len(class_mismatches),
        "class_mismatches": class_mismatches[:20],
        "classes_complete": not class_mismatches,
        "transform_mismatch_count": len(transform_mismatches),
        "transform_mismatches": transform_mismatches[:20],
        "transforms_complete": not transform_mismatches,
        "physical_mismatch_count": len(physical_mismatches),
        "physical_mismatches": physical_mismatches[:20],
        "physical_complete": not physical_mismatches,
        "tolerance": tolerance,
        "parts_complete": not missing_part_ids and len(manifest_parts) == len(parts),
        "joints_complete": (
            not missing_connection_names and len(manifest_joints) == len(connections)
        ),
        "complete": (
            not missing_part_ids
            and not missing_connection_names
            and len(manifest_parts) == len(parts)
            and len(manifest_joints) == len(connections)
            and not class_mismatches
            and not missing_parameter_connection_names
            and not fixed_lock_mismatches
            and not transform_mismatches
            and not physical_mismatches
        ),
    }


def _expected_part_physical_parameters(part: dict[str, Any]) -> dict[str, Any]:
    params = part.get("params", {}) if isinstance(part.get("params"), dict) else {}
    shape = str(part.get("shape", "box"))
    mass = float(params.get("mass", 1.0))
    if shape == "capsule":
        radius = float(params.get("radius", 0.04))
        height = float(params.get("length", 0.3))
        return {
            "mass": mass,
            "collision_parameters": {"radius": radius, "height": height},
            "mesh_parameters": {"radius": radius, "height": height},
        }
    if shape == "cylinder":
        radius = float(params.get("radius", 0.04))
        height = float(params.get("length", 0.3))
        return {
            "mass": mass,
            "collision_parameters": {"radius": radius, "height": height},
            "mesh_parameters": {
                "top_radius": radius,
                "bottom_radius": radius,
                "height": height,
            },
        }
    if shape == "sphere":
        radius = float(params.get("radius", 0.05))
        return {
            "mass": mass,
            "collision_parameters": {"radius": radius},
            "mesh_parameters": {"radius": radius, "height": radius * 2.0},
        }
    size = params.get("size", [0.2, 0.2, 0.2])
    return {
        "mass": mass,
        "collision_parameters": {"size": size},
        "mesh_parameters": {"size": size},
    }


def _compare_node_tree_mapping(
    mismatches: list[dict[str, Any]],
    *,
    kind: str,
    name: str,
    prefix: str,
    expected: Any,
    actual: Any,
    tolerance: float,
) -> None:
    if not isinstance(expected, dict) or not isinstance(actual, dict):
        _compare_node_tree_value(
            mismatches,
            kind=kind,
            name=name,
            field=prefix,
            expected=expected,
            actual=actual,
            tolerance=tolerance,
        )
        return
    for key, expected_value in expected.items():
        _compare_node_tree_value(
            mismatches,
            kind=kind,
            name=name,
            field=f"{prefix}.{key}",
            expected=expected_value,
            actual=actual.get(key),
            tolerance=tolerance,
        )


def _compare_node_tree_value(
    mismatches: list[dict[str, Any]],
    *,
    kind: str,
    name: str,
    field: str,
    expected: Any,
    actual: Any,
    tolerance: float,
) -> None:
    if isinstance(expected, list) or isinstance(actual, list):
        _compare_node_tree_vector(
            mismatches,
            kind=kind,
            name=name,
            field=field,
            expected=expected,
            actual=actual,
            tolerance=tolerance,
        )
        return
    if expected is None or actual is None:
        if expected != actual:
            mismatches.append(
                {
                    "kind": kind,
                    "name": name,
                    "field": field,
                    "expected": expected,
                    "actual": actual,
                    "max_delta": None,
                }
            )
        return
    if isinstance(expected, (int, float)) and isinstance(actual, (int, float)):
        delta = abs(float(expected) - float(actual))
        if delta > tolerance:
            mismatches.append(
                {
                    "kind": kind,
                    "name": name,
                    "field": field,
                    "expected": expected,
                    "actual": actual,
                    "max_delta": delta,
                }
            )
        return
    if expected != actual:
        mismatches.append(
            {
                "kind": kind,
                "name": name,
                "field": field,
                "expected": expected,
                "actual": actual,
                "max_delta": None,
            }
        )


def _compare_node_tree_vector(
    mismatches: list[dict[str, Any]],
    *,
    kind: str,
    name: str,
    field: str,
    expected: Any,
    actual: Any,
    tolerance: float,
) -> None:
    if expected is None or actual is None:
        return
    if not isinstance(expected, list) or not isinstance(actual, list):
        if expected != actual:
            mismatches.append(
                {
                    "kind": kind,
                    "name": name,
                    "field": field,
                    "expected": expected,
                    "actual": actual,
                    "max_delta": None,
                }
            )
        return
    if len(expected) != len(actual):
        mismatches.append(
            {
                "kind": kind,
                "name": name,
                "field": field,
                "expected": expected,
                "actual": actual,
                "max_delta": None,
            }
        )
        return
    deltas = [
        abs(float(expected_item) - float(actual_item))
        for expected_item, actual_item in zip(expected, actual)
        if isinstance(expected_item, (int, float)) and isinstance(actual_item, (int, float))
    ]
    if len(deltas) != len(expected):
        mismatches.append(
            {
                "kind": kind,
                "name": name,
                "field": field,
                "expected": expected,
                "actual": actual,
                "max_delta": None,
            }
        )
        return
    max_delta = max(deltas, default=0.0)
    if max_delta > tolerance:
        mismatches.append(
            {
                "kind": kind,
                "name": name,
                "field": field,
                "expected": expected,
                "actual": actual,
                "max_delta": max_delta,
            }
        )


def _expected_collision_shape(shape: Any) -> str:
    shape_name = str(shape or "box")
    return {
        "capsule": "CapsuleShape3D",
        "cylinder": "CylinderShape3D",
        "sphere": "SphereShape3D",
    }.get(shape_name, "BoxShape3D")


def _expected_mesh_type(shape: Any) -> str:
    shape_name = str(shape or "box")
    return {
        "capsule": "CapsuleMesh",
        "cylinder": "CylinderMesh",
        "sphere": "SphereMesh",
    }.get(shape_name, "BoxMesh")


def _expected_joint_class(joint_type: Any) -> str:
    joint_name = str(joint_type or "fixed")
    if joint_name in {"hinge", "revolute"}:
        return "HingeJoint3D"
    if joint_name in {"slider", "prismatic"}:
        return "SliderJoint3D"
    return "Generic6DOFJoint3D"


def _mechanical_restoration_summary(
    *,
    expected_parts: int,
    expected_joints: int,
    part_nodes: Any,
    joint_nodes: Any,
    body_states: Any,
    joint_states: Any,
    parameter_summary: dict[str, Any],
    steps_run: int,
    min_restoration_score: float | None,
) -> dict[str, Any]:
    restored_parts = len(part_nodes) if isinstance(part_nodes, list) else 0
    restored_joints = len(joint_nodes) if isinstance(joint_nodes, list) else 0
    body_telemetry_count = len(body_states) if isinstance(body_states, dict) else 0
    joint_telemetry_count = len(joint_states) if isinstance(joint_states, dict) else 0
    parameterized_count = int(parameter_summary.get("parameterized_count") or 0)

    checks = {
        "parts_mapped": restored_parts == expected_parts,
        "joints_mapped": restored_joints == expected_joints,
        "body_telemetry_complete": body_telemetry_count == expected_parts,
        "joint_telemetry_complete": joint_telemetry_count == expected_joints,
        "joint_parameters_restored": parameterized_count == expected_joints,
        "simulation_sampled": steps_run > 0,
    }
    passed = sum(1 for value in checks.values() if value)
    total = len(checks)
    return {
        "expected_parts": expected_parts,
        "restored_parts": restored_parts,
        "expected_joints": expected_joints,
        "restored_joints": restored_joints,
        "body_telemetry_count": body_telemetry_count,
        "joint_telemetry_count": joint_telemetry_count,
        "parameterized_joints": parameterized_count,
        "steps_run": steps_run,
        "checks": checks,
        "score": passed / total if total else 0.0,
        "min_restoration_score_threshold": min_restoration_score,
        "score_under_min": (
            min_restoration_score is not None
            and (passed / total if total else 0.0) < min_restoration_score
        ),
        "complete": passed == total,
    }


def _action_sequence_summary(
    actions: list[Any],
    *,
    min_transitions: int | None = None,
    min_transition_delta: float | None = None,
) -> dict[str, Any]:
    if not actions:
        return {
            "steps": 0,
            "unique_action_count": 0,
            "transition_count": 0,
            "min_transition_threshold": min_transitions,
            "transitions_under_min": min_transitions is not None and min_transitions > 0,
            "max_numeric_transition_delta": 0.0,
            "average_numeric_transition_delta": 0.0,
            "min_transition_delta_threshold": min_transition_delta,
            "transition_delta_under_min": (
                min_transition_delta is not None and min_transition_delta > 0.0
            ),
            "first_action": None,
            "last_action": None,
        }
    encoded = [_stable_json(action) for action in actions]
    transition_count = sum(
        1 for index in range(1, len(encoded)) if encoded[index] != encoded[index - 1]
    )
    numeric_deltas = [
        _numeric_action_delta(actions[index - 1], actions[index])
        for index in range(1, len(actions))
    ]
    max_numeric_delta = max(numeric_deltas, default=0.0)
    average_numeric_delta = (
        sum(numeric_deltas) / len(numeric_deltas) if numeric_deltas else 0.0
    )
    return {
        "steps": len(actions),
        "unique_action_count": len(set(encoded)),
        "transition_count": transition_count,
        "min_transition_threshold": min_transitions,
        "transitions_under_min": (
            min_transitions is not None and transition_count < min_transitions
        ),
        "max_numeric_transition_delta": max_numeric_delta,
        "average_numeric_transition_delta": average_numeric_delta,
        "min_transition_delta_threshold": min_transition_delta,
        "transition_delta_under_min": (
            min_transition_delta is not None
            and max_numeric_delta < min_transition_delta
        ),
        "first_action": actions[0],
        "last_action": actions[-1],
    }


def _numeric_action_delta(left: Any, right: Any) -> float:
    left_targets = _numeric_action_targets_for_delta(left)
    right_targets = _numeric_action_targets_for_delta(right)
    names = set(left_targets) | set(right_targets)
    if not names:
        return 0.0
    return max(abs(right_targets.get(name, 0.0) - left_targets.get(name, 0.0)) for name in names)


def _numeric_action_targets_for_delta(action: Any) -> dict[str, float]:
    if isinstance(action, dict):
        return {
            str(name): float(value)
            for name, value in action.items()
            if isinstance(value, int | float) and not isinstance(value, bool)
        }
    if isinstance(action, list):
        return {
            str(index): float(value)
            for index, value in enumerate(action)
            if isinstance(value, int | float) and not isinstance(value, bool)
        }
    return {}


def _stable_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def _simulation_summary(
    step_results: list[dict[str, Any]],
    *,
    min_body_displacement: float | None,
    max_linear_speed: float | None,
) -> dict[str, Any]:
    if not step_results:
        return {
            "steps_run": 0,
            "max_body_displacement": None,
            "average_body_displacement": None,
            "max_linear_speed": None,
            "min_body_displacement_threshold": min_body_displacement,
            "max_linear_speed_threshold": max_linear_speed,
            "displacement_under_min": False,
            "speed_threshold_exceeded": False,
            "fastest_body": None,
            "most_displaced_body": None,
        }

    first_bodies = step_results[0].get("body_states", {})
    last_bodies = step_results[-1].get("body_states", {})
    if not isinstance(first_bodies, dict) or not isinstance(last_bodies, dict):
        return {
            "steps_run": len(step_results),
            "max_body_displacement": None,
            "average_body_displacement": None,
            "max_linear_speed": None,
            "min_body_displacement_threshold": min_body_displacement,
            "max_linear_speed_threshold": max_linear_speed,
            "displacement_under_min": False,
            "speed_threshold_exceeded": False,
            "fastest_body": None,
            "most_displaced_body": None,
        }

    displacements: list[tuple[str, float]] = []
    speeds: list[tuple[str, float]] = []
    for body_name, last_state in last_bodies.items():
        if not isinstance(last_state, dict):
            continue
        first_state = first_bodies.get(body_name)
        if isinstance(first_state, dict):
            distance = _vector_distance(first_state.get("position"), last_state.get("position"))
            if distance is not None:
                displacements.append((str(body_name), distance))
        speed = _vector_length(last_state.get("linear_velocity"))
        if speed is not None:
            speeds.append((str(body_name), speed))

    most_displaced = max(displacements, key=lambda item: item[1]) if displacements else None
    fastest = max(speeds, key=lambda item: item[1]) if speeds else None
    average_displacement = (
        sum(distance for _name, distance in displacements) / len(displacements)
        if displacements
        else None
    )
    return {
        "steps_run": len(step_results),
        "max_body_displacement": most_displaced[1] if most_displaced else None,
        "average_body_displacement": average_displacement,
        "max_linear_speed": fastest[1] if fastest else None,
        "min_body_displacement_threshold": min_body_displacement,
        "max_linear_speed_threshold": max_linear_speed,
        "displacement_under_min": (
            bool(most_displaced)
            and min_body_displacement is not None
            and most_displaced[1] < min_body_displacement
        ),
        "speed_threshold_exceeded": (
            bool(fastest)
            and max_linear_speed is not None
            and fastest[1] > max_linear_speed
        ),
        "fastest_body": fastest[0] if fastest else None,
        "most_displaced_body": most_displaced[0] if most_displaced else None,
    }


def _joint_motion_summary(
    step_results: list[dict[str, Any]],
    *,
    min_joint_angle_delta: float | None,
    min_joint_angle_range: float | None,
    min_moving_joint_coverage: float | None,
    min_commanded_joint_response_coverage: float | None,
    joint_motion_epsilon: float,
    actions: list[Any],
) -> dict[str, Any]:
    if not step_results:
        return {
            "steps_run": 0,
            "joint_count": 0,
            "measured_joint_count": 0,
            "moving_joint_count": 0,
            "moving_joint_coverage_ratio": 0.0,
            "commanded_joint_count": 0,
            "commanded_moving_joint_count": 0,
            "commanded_joint_response_ratio": 0.0,
            "commanded_static_joints": [],
            "commanded_joint_response_details": [],
            "joint_motion_epsilon": joint_motion_epsilon,
            "max_abs_relative_angle_delta": 0.0,
            "average_abs_relative_angle_delta": 0.0,
            "max_abs_relative_angle_range": 0.0,
            "average_abs_relative_angle_range": 0.0,
            "largest_moving_joint": None,
            "largest_range_joint": None,
            "min_joint_angle_delta_threshold": min_joint_angle_delta,
            "min_joint_angle_range_threshold": min_joint_angle_range,
            "min_moving_joint_coverage_threshold": min_moving_joint_coverage,
            "min_commanded_joint_response_coverage_threshold": min_commanded_joint_response_coverage,
            "angle_delta_under_min": (
                min_joint_angle_delta is not None and min_joint_angle_delta > 0.0
            ),
            "angle_range_under_min": (
                min_joint_angle_range is not None and min_joint_angle_range > 0.0
            ),
            "moving_joint_coverage_under_min": (
                min_moving_joint_coverage is not None and min_moving_joint_coverage > 0.0
            ),
            "commanded_joint_response_under_min": (
                min_commanded_joint_response_coverage is not None
                and min_commanded_joint_response_coverage > 0.0
            ),
        }

    first_joints = step_results[0].get("joint_states", {})
    last_joints = step_results[-1].get("joint_states", {})
    if not isinstance(first_joints, dict) or not isinstance(last_joints, dict):
        return {
            "steps_run": len(step_results),
            "joint_count": 0,
            "measured_joint_count": 0,
            "moving_joint_count": 0,
            "moving_joint_coverage_ratio": 0.0,
            "commanded_joint_count": 0,
            "commanded_moving_joint_count": 0,
            "commanded_joint_response_ratio": 0.0,
            "commanded_static_joints": [],
            "commanded_joint_response_details": [],
            "joint_motion_epsilon": joint_motion_epsilon,
            "max_abs_relative_angle_delta": 0.0,
            "average_abs_relative_angle_delta": 0.0,
            "max_abs_relative_angle_range": 0.0,
            "average_abs_relative_angle_range": 0.0,
            "largest_moving_joint": None,
            "largest_range_joint": None,
            "min_joint_angle_delta_threshold": min_joint_angle_delta,
            "min_joint_angle_range_threshold": min_joint_angle_range,
            "min_moving_joint_coverage_threshold": min_moving_joint_coverage,
            "min_commanded_joint_response_coverage_threshold": min_commanded_joint_response_coverage,
            "angle_delta_under_min": (
                min_joint_angle_delta is not None and min_joint_angle_delta > 0.0
            ),
            "angle_range_under_min": (
                min_joint_angle_range is not None and min_joint_angle_range > 0.0
            ),
            "moving_joint_coverage_under_min": (
                min_moving_joint_coverage is not None and min_moving_joint_coverage > 0.0
            ),
            "commanded_joint_response_under_min": (
                min_commanded_joint_response_coverage is not None
                and min_commanded_joint_response_coverage > 0.0
            ),
        }

    deltas: list[tuple[str, float]] = []
    angle_samples: dict[str, list[float]] = {}
    for step in step_results:
        step_joints = step.get("joint_states", {})
        if not isinstance(step_joints, dict):
            continue
        for joint_name, state in step_joints.items():
            if not isinstance(state, dict):
                continue
            angle = state.get("relative_angle")
            if _is_number(angle):
                angle_samples.setdefault(str(joint_name), []).append(float(angle))
    for joint_name, last_state in last_joints.items():
        if not isinstance(last_state, dict):
            continue
        first_state = first_joints.get(joint_name)
        if not isinstance(first_state, dict):
            continue
        first_angle = first_state.get("relative_angle")
        last_angle = last_state.get("relative_angle")
        if not _is_number(first_angle) or not _is_number(last_angle):
            continue
        deltas.append((str(joint_name), abs(float(last_angle) - float(first_angle))))

    largest = max(deltas, key=lambda item: item[1]) if deltas else None
    max_delta = largest[1] if largest else 0.0
    average_delta = sum(delta for _name, delta in deltas) / len(deltas) if deltas else 0.0
    ranges = [
        (joint_name, max(samples) - min(samples))
        for joint_name, samples in angle_samples.items()
        if samples
    ]
    largest_range = max(ranges, key=lambda item: item[1]) if ranges else None
    max_range = largest_range[1] if largest_range else 0.0
    largest_range_joint = largest_range[0] if largest_range and max_range > 0.0 else None
    average_range = sum(value for _name, value in ranges) / len(ranges) if ranges else 0.0
    moving_joint_count = sum(1 for _name, value in ranges if value > joint_motion_epsilon)
    measured_joint_count = len(deltas)
    moving_joint_coverage_ratio = (
        moving_joint_count / measured_joint_count if measured_joint_count else 0.0
    )
    largest_moving_joint = (
        largest_range[0] if largest_range and max_range > joint_motion_epsilon else None
    )
    range_by_joint = dict(ranges)
    commanded_targets = _commanded_action_targets(last_joints, actions)
    commanded_joints = sorted(commanded_targets)
    commanded_moving_joints = [
        joint_name
        for joint_name in commanded_joints
        if range_by_joint.get(joint_name, 0.0) > joint_motion_epsilon
    ]
    commanded_static_joints = [
        joint_name
        for joint_name in commanded_joints
        if range_by_joint.get(joint_name, 0.0) <= joint_motion_epsilon
    ]
    commanded_response_ratio = (
        len(commanded_moving_joints) / len(commanded_joints) if commanded_joints else 0.0
    )
    commanded_response_details = [
        {
            "joint": joint_name,
            "max_abs_target": commanded_targets[joint_name],
            "angle_range": range_by_joint.get(joint_name, 0.0),
            "responded": range_by_joint.get(joint_name, 0.0) > joint_motion_epsilon,
        }
        for joint_name in commanded_joints
    ]
    return {
        "steps_run": len(step_results),
        "joint_count": len(last_joints),
        "measured_joint_count": measured_joint_count,
        "moving_joint_count": moving_joint_count,
        "moving_joint_coverage_ratio": moving_joint_coverage_ratio,
        "commanded_joint_count": len(commanded_joints),
        "commanded_moving_joint_count": len(commanded_moving_joints),
        "commanded_joint_response_ratio": commanded_response_ratio,
        "commanded_static_joints": commanded_static_joints[:20],
        "commanded_joint_response_details": commanded_response_details[:20],
        "joint_motion_epsilon": joint_motion_epsilon,
        "max_abs_relative_angle_delta": max_delta,
        "average_abs_relative_angle_delta": average_delta,
        "max_abs_relative_angle_range": max_range,
        "average_abs_relative_angle_range": average_range,
        "largest_moving_joint": largest_moving_joint,
        "largest_range_joint": largest_range_joint,
        "min_joint_angle_delta_threshold": min_joint_angle_delta,
        "min_joint_angle_range_threshold": min_joint_angle_range,
        "min_moving_joint_coverage_threshold": min_moving_joint_coverage,
        "min_commanded_joint_response_coverage_threshold": min_commanded_joint_response_coverage,
        "angle_delta_under_min": (
            min_joint_angle_delta is not None and max_delta < min_joint_angle_delta
        ),
        "angle_range_under_min": (
            min_joint_angle_range is not None and max_range < min_joint_angle_range
        ),
        "moving_joint_coverage_under_min": (
            min_moving_joint_coverage is not None
            and moving_joint_coverage_ratio < min_moving_joint_coverage
        ),
        "commanded_joint_response_under_min": (
            min_commanded_joint_response_coverage is not None
            and commanded_response_ratio < min_commanded_joint_response_coverage
        ),
    }


def _commanded_action_targets(joint_states: dict[str, Any], actions: list[Any]) -> dict[str, float]:
    commanded: dict[str, float] = {}
    for action in actions:
        plan = _action_target_plan(joint_states, action)
        for joint_name, target in plan["expected_targets"].items():
            commanded[joint_name] = max(abs(float(target)), commanded.get(joint_name, 0.0))
    return commanded


def _vector_distance(left: Any, right: Any) -> float | None:
    if not (_is_vector3(left) and _is_vector3(right)):
        return None
    return sum((float(left[index]) - float(right[index])) ** 2 for index in range(3)) ** 0.5


def _vector_length(value: Any) -> float | None:
    if not _is_vector3(value):
        return None
    return sum(float(item) ** 2 for item in value) ** 0.5


def _is_vector3(value: Any) -> bool:
    return isinstance(value, list) and len(value) == 3 and all(
        isinstance(item, (int, float)) for item in value
    )


def _is_number(value: Any) -> bool:
    return isinstance(value, int | float) and not isinstance(value, bool)


def _joint_control_summary(joint_states: Any) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "joint_count": 0,
            "targeted_count": 0,
            "max_abs_target_velocity": None,
            "average_abs_target_velocity": None,
            "largest_target_joint": None,
        }

    targets: list[tuple[str, float]] = []
    for joint_name, state in joint_states.items():
        if not isinstance(state, dict):
            continue
        target = state.get("target_velocity")
        if isinstance(target, (int, float)):
            targets.append((str(joint_name), abs(float(target))))

    largest = max(targets, key=lambda item: item[1]) if targets else None
    average = (
        sum(target for _name, target in targets) / len(targets) if targets else None
    )
    return {
        "joint_count": len(joint_states),
        "targeted_count": sum(1 for _name, target in targets if target > 0.0),
        "max_abs_target_velocity": largest[1] if largest else None,
        "average_abs_target_velocity": average,
        "largest_target_joint": largest[0] if largest else None,
    }


def _joint_control_consistency_summary(
    robot_config: dict[str, Any],
    joint_states: Any,
    *,
    tolerance: float = 1e-4,
) -> dict[str, Any]:
    configured = _configured_joint_controls(robot_config)
    if not isinstance(joint_states, dict):
        return {
            "configured_count": len(configured),
            "checked_count": 0,
            "missing_count": len(configured),
            "mismatch_count": 0,
            "mismatches": [],
            "tolerance": tolerance,
            "complete": not configured,
        }

    mismatches: list[dict[str, Any]] = []
    checked_count = 0
    missing_count = 0
    for joint_name, expected_control in configured.items():
        state = joint_states.get(joint_name)
        if not isinstance(state, dict):
            missing_count += 1
            mismatches.append(
                {
                    "joint": joint_name,
                    "field": "control_parameters",
                    "expected": expected_control,
                    "actual": None,
                }
            )
            continue
        actual_control = state.get("control_parameters")
        if not isinstance(actual_control, dict):
            missing_count += 1
            mismatches.append(
                {
                    "joint": joint_name,
                    "field": "control_parameters",
                    "expected": expected_control,
                    "actual": actual_control,
                }
            )
            continue
        checked_count += 1
        for field, expected_value in expected_control.items():
            _compare_joint_parameter(
                mismatches,
                joint_name=joint_name,
                field=f"control.{field}",
                expected=expected_value,
                actual=actual_control.get(field),
                tolerance=tolerance,
            )

    return {
        "configured_count": len(configured),
        "checked_count": checked_count,
        "missing_count": missing_count,
        "mismatch_count": len(mismatches),
        "mismatches": mismatches[:20],
        "tolerance": tolerance,
        "complete": checked_count == len(configured) and not mismatches,
    }


def _configured_joint_controls(robot_config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    control = robot_config.get("control", {})
    if not isinstance(control, dict):
        return {}
    joints = control.get("joints", {})
    if not isinstance(joints, dict):
        return {}
    return {
        str(joint_name): control_params
        for joint_name, control_params in joints.items()
        if isinstance(control_params, dict)
    }


def _action_target_consistency_summary(
    joint_states: Any,
    action: Any,
    *,
    tolerance: float = 1e-4,
) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "expected_count": 0,
            "checked_count": 0,
            "mismatch_count": 0,
            "mismatches": [],
            "unknown_target_count": 0,
            "unknown_targets": [],
            "invalid_target_count": 0,
            "invalid_targets": [],
            "tolerance": tolerance,
            "complete": False,
        }

    plan = _action_target_plan(joint_states, action)
    expected_targets = plan["expected_targets"]
    mismatches: list[dict[str, Any]] = []
    checked_count = 0
    for joint_name, expected_target in expected_targets.items():
        state = joint_states.get(joint_name)
        actual_target = state.get("target_velocity") if isinstance(state, dict) else None
        if isinstance(actual_target, (int, float)):
            checked_count += 1
        _compare_joint_parameter(
            mismatches,
            joint_name=joint_name,
            field="target_velocity",
            expected=expected_target,
            actual=actual_target,
            tolerance=tolerance,
        )

    return {
        "expected_count": len(expected_targets),
        "checked_count": checked_count,
        "mismatch_count": len(mismatches),
        "mismatches": mismatches[:20],
        "unknown_target_count": len(plan["unknown_targets"]),
        "unknown_targets": plan["unknown_targets"][:20],
        "invalid_target_count": len(plan["invalid_targets"]),
        "invalid_targets": plan["invalid_targets"][:20],
        "tolerance": tolerance,
        "complete": (
            checked_count == len(expected_targets)
            and not mismatches
            and not plan["unknown_targets"]
            and not plan["invalid_targets"]
        ),
    }


def _effective_unknown_action_target_count(
    action_target_summary: dict[str, Any],
    action_sequence_target_summary: dict[str, Any],
) -> int:
    if int(action_sequence_target_summary.get("steps") or 0) > 0:
        return int(action_sequence_target_summary.get("unknown_target_count") or 0)
    return int(action_target_summary.get("unknown_target_count") or 0)


def _effective_invalid_action_target_count(
    action_target_summary: dict[str, Any],
    action_sequence_target_summary: dict[str, Any],
) -> int:
    if int(action_sequence_target_summary.get("steps") or 0) > 0:
        return int(action_sequence_target_summary.get("invalid_target_count") or 0)
    return int(action_target_summary.get("invalid_target_count") or 0)


def _action_target_coverage_summary(
    joint_states: Any,
    actions: list[Any],
    *,
    fallback_action: Any,
    min_coverage: float | None,
) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "joint_count": 0,
            "covered_joint_count": 0,
            "missing_target_count": 0,
            "missing_target_joints": [],
            "target_command_count": 0,
            "unknown_target_count": 0,
            "invalid_target_count": 0,
            "coverage_ratio": None,
            "min_coverage_threshold": min_coverage,
            "coverage_under_min": False,
            "complete": False,
        }

    action_items = actions if actions else [fallback_action]
    covered: set[str] = set()
    unknown_target_count = 0
    invalid_target_count = 0
    target_command_count = 0
    for action in action_items:
        plan = _action_target_plan(joint_states, action)
        covered.update(plan["expected_targets"].keys())
        target_command_count += len(plan["expected_targets"])
        unknown_target_count += len(plan["unknown_targets"])
        invalid_target_count += len(plan["invalid_targets"])

    joint_names = [str(joint_name) for joint_name in joint_states.keys()]
    missing = [joint_name for joint_name in joint_names if joint_name not in covered]
    coverage_ratio = (len(covered) / len(joint_names)) if joint_names else None
    coverage_under_min = (
        coverage_ratio is not None
        and min_coverage is not None
        and coverage_ratio < min_coverage
    )
    return {
        "joint_count": len(joint_names),
        "covered_joint_count": len(covered),
        "missing_target_count": len(missing),
        "missing_target_joints": missing[:20],
        "target_command_count": target_command_count,
        "unknown_target_count": unknown_target_count,
        "invalid_target_count": invalid_target_count,
        "coverage_ratio": coverage_ratio,
        "min_coverage_threshold": min_coverage,
        "coverage_under_min": coverage_under_min,
        "complete": (
            len(covered) == len(joint_names)
            and unknown_target_count == 0
            and invalid_target_count == 0
        ),
    }


def _control_action_coverage_summary(
    robot_config: dict[str, Any],
    joint_states: Any,
    actions: list[Any],
    *,
    fallback_action: Any,
    min_coverage: float | None,
) -> dict[str, Any]:
    configured_controls = _configured_joint_controls(robot_config)
    if not isinstance(joint_states, dict):
        return {
            "configured_count": len(configured_controls),
            "generated_configured_count": 0,
            "covered_control_count": 0,
            "missing_action_count": 0,
            "missing_action_joints": [],
            "missing_generated_count": len(configured_controls),
            "missing_generated_joints": list(configured_controls.keys())[:20],
            "coverage_ratio": None,
            "min_coverage_threshold": min_coverage,
            "coverage_under_min": False,
            "complete": False,
        }

    configured_names = list(configured_controls.keys())
    generated_configured = [
        joint_name for joint_name in configured_names if joint_name in joint_states
    ]
    action_items = actions if actions else [fallback_action]
    covered: set[str] = set()
    for action in action_items:
        plan = _action_target_plan(joint_states, action)
        covered.update(
            joint_name
            for joint_name in plan["expected_targets"].keys()
            if joint_name in configured_controls
        )

    missing_action = [
        joint_name for joint_name in generated_configured if joint_name not in covered
    ]
    missing_generated = [
        joint_name for joint_name in configured_names if joint_name not in joint_states
    ]
    coverage_ratio = (
        len(covered) / len(generated_configured) if generated_configured else None
    )
    coverage_under_min = (
        coverage_ratio is not None
        and min_coverage is not None
        and coverage_ratio < min_coverage
    )
    return {
        "configured_count": len(configured_names),
        "generated_configured_count": len(generated_configured),
        "covered_control_count": len(covered),
        "missing_action_count": len(missing_action),
        "missing_action_joints": missing_action[:20],
        "missing_generated_count": len(missing_generated),
        "missing_generated_joints": missing_generated[:20],
        "coverage_ratio": coverage_ratio,
        "min_coverage_threshold": min_coverage,
        "coverage_under_min": coverage_under_min,
        "complete": (
            len(covered) == len(generated_configured)
            and not missing_generated
            and bool(configured_names)
        ),
    }


def _action_sequence_target_consistency_summary(
    step_results: list[dict[str, Any]],
    actions: list[Any],
    *,
    fallback_action: Any,
    tolerance: float = 1e-4,
) -> dict[str, Any]:
    if not step_results:
        return {
            "steps": 0,
            "expected_count": 0,
            "checked_count": 0,
            "mismatch_count": 0,
            "mismatches": [],
            "unknown_target_count": 0,
            "unknown_targets": [],
            "invalid_target_count": 0,
            "invalid_targets": [],
            "tolerance": tolerance,
            "complete": False,
        }

    mismatches: list[dict[str, Any]] = []
    unknown_targets: list[dict[str, Any]] = []
    invalid_targets: list[dict[str, Any]] = []
    expected_count = 0
    checked_count = 0
    for step_index, step_result in enumerate(step_results):
        joint_states = step_result.get("joint_states", {})
        if not isinstance(joint_states, dict):
            continue
        action = actions[step_index] if step_index < len(actions) else fallback_action
        plan = _action_target_plan(joint_states, action)
        expected_targets = plan["expected_targets"]
        for unknown_target in plan["unknown_targets"]:
            item = dict(unknown_target)
            item["step_index"] = step_index
            unknown_targets.append(item)
        for invalid_target in plan["invalid_targets"]:
            item = dict(invalid_target)
            item["step_index"] = step_index
            invalid_targets.append(item)
        expected_count += len(expected_targets)
        for joint_name, expected_target in expected_targets.items():
            state = joint_states.get(joint_name)
            actual_target = state.get("target_velocity") if isinstance(state, dict) else None
            if isinstance(actual_target, (int, float)):
                checked_count += 1
            before = len(mismatches)
            _compare_joint_parameter(
                mismatches,
                joint_name=joint_name,
                field="target_velocity",
                expected=expected_target,
                actual=actual_target,
                tolerance=tolerance,
            )
            for mismatch in mismatches[before:]:
                mismatch["step_index"] = step_index

    return {
        "steps": len(step_results),
        "expected_count": expected_count,
        "checked_count": checked_count,
        "mismatch_count": len(mismatches),
        "mismatches": mismatches[:20],
        "unknown_target_count": len(unknown_targets),
        "unknown_targets": unknown_targets[:20],
        "invalid_target_count": len(invalid_targets),
        "invalid_targets": invalid_targets[:20],
        "tolerance": tolerance,
        "complete": (
            checked_count == expected_count
            and not mismatches
            and not unknown_targets
            and not invalid_targets
        ),
    }


def _expected_action_targets(joint_states: dict[str, Any], action: Any) -> dict[str, float]:
    return _action_target_plan(joint_states, action)["expected_targets"]


def _action_target_plan(joint_states: dict[str, Any], action: Any) -> dict[str, Any]:
    joint_names = list(joint_states.keys())
    if isinstance(action, dict):
        expected_targets: dict[str, float] = {}
        unknown_targets: list[dict[str, Any]] = []
        invalid_targets: list[dict[str, Any]] = []
        for joint_name, target in action.items():
            name = str(joint_name)
            if name in joint_states and isinstance(target, (int, float)):
                expected_targets[name] = float(target)
            elif name not in joint_states:
                unknown_targets.append(
                    {"kind": "named_joint", "joint": name, "target": target}
                )
            else:
                invalid_targets.append(
                    {
                        "kind": "named_joint",
                        "joint": name,
                        "target": target,
                        "target_type": type(target).__name__,
                    }
                )
        return {
            "expected_targets": expected_targets,
            "unknown_targets": unknown_targets,
            "invalid_targets": invalid_targets,
        }
    if isinstance(action, list):
        targets: dict[str, float] = {}
        unknown_targets = []
        invalid_targets = []
        for index, target in enumerate(action):
            if index >= len(joint_names):
                unknown_targets.append(
                    {"kind": "list_index", "index": index, "target": target}
                )
                continue
            if isinstance(target, (int, float)):
                targets[str(joint_names[index])] = float(target)
            else:
                invalid_targets.append(
                    {
                        "kind": "list_index",
                        "index": index,
                        "joint": str(joint_names[index]),
                        "target": target,
                        "target_type": type(target).__name__,
                    }
                )
        return {
            "expected_targets": targets,
            "unknown_targets": unknown_targets,
            "invalid_targets": invalid_targets,
        }
    return {
        "expected_targets": {},
        "unknown_targets": [],
        "invalid_targets": (
            [
                {
                    "kind": "action_payload",
                    "target": action,
                    "target_type": type(action).__name__,
                }
            ]
            if action is not None
            else []
        ),
    }


def _joint_parameter_summary(joint_states: Any) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "joint_count": 0,
            "parameterized_count": 0,
            "limits_applied_count": 0,
            "motor_supported_count": 0,
            "motor_enabled_count": 0,
            "dynamics_configured_count": 0,
            "damping_applied_count": 0,
        }

    summary = {
        "joint_count": len(joint_states),
        "parameterized_count": 0,
        "limits_applied_count": 0,
        "motor_supported_count": 0,
        "motor_enabled_count": 0,
        "dynamics_configured_count": 0,
        "damping_applied_count": 0,
    }
    for state in joint_states.values():
        if not isinstance(state, dict):
            continue
        applied = state.get("applied_parameters")
        if not isinstance(applied, dict):
            continue
        runtime = applied.get("runtime")
        if not isinstance(runtime, dict):
            continue
        summary["parameterized_count"] += 1
        if runtime.get("limit_enabled") is True:
            summary["limits_applied_count"] += 1
        if runtime.get("motor_supported") is True:
            summary["motor_supported_count"] += 1
        if runtime.get("motor_enabled") is True:
            summary["motor_enabled_count"] += 1
        if runtime.get("dynamics_configured") is True:
            summary["dynamics_configured_count"] += 1
        if runtime.get("damping_applied") is True:
            summary["damping_applied_count"] += 1
    return summary


def _joint_limit_summary(joint_states: Any) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "joint_count": 0,
            "configured_count": 0,
            "violation_count": 0,
            "min_margin": None,
            "worst_joint": None,
        }

    margins: list[tuple[str, float]] = []
    violation_count = 0
    configured_count = 0
    for joint_name, state in joint_states.items():
        if not isinstance(state, dict):
            continue
        limits = state.get("limits")
        if not isinstance(limits, dict) or not limits.get("configured"):
            continue
        configured_count += 1
        if limits.get("violation") is True:
            violation_count += 1
        margin = limits.get("min_margin")
        if isinstance(margin, (int, float)):
            margins.append((str(joint_name), float(margin)))

    worst = min(margins, key=lambda item: item[1]) if margins else None
    return {
        "joint_count": len(joint_states),
        "configured_count": configured_count,
        "violation_count": violation_count,
        "min_margin": worst[1] if worst else None,
        "worst_joint": worst[0] if worst else None,
    }


def _joint_angle_summary(
    joint_states: Any,
    max_relative_angle: float | None,
) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "joint_count": 0,
            "max_abs_relative_angle": None,
            "average_abs_relative_angle": None,
            "max_relative_angle_threshold": max_relative_angle,
            "threshold_exceeded": False,
            "largest_angle_joint": None,
        }

    angles: list[tuple[str, float]] = []
    for joint_name, state in joint_states.items():
        if not isinstance(state, dict):
            continue
        angle = state.get("relative_angle")
        if isinstance(angle, (int, float)):
            angles.append((str(joint_name), abs(float(angle))))

    largest = max(angles, key=lambda item: item[1]) if angles else None
    average = (
        sum(angle for _name, angle in angles) / len(angles) if angles else None
    )
    return {
        "joint_count": len(joint_states),
        "max_abs_relative_angle": largest[1] if largest else None,
        "average_abs_relative_angle": average,
        "max_relative_angle_threshold": max_relative_angle,
        "threshold_exceeded": (
            bool(largest)
            and max_relative_angle is not None
            and largest[1] > max_relative_angle
        ),
        "largest_angle_joint": largest[0] if largest else None,
    }


def _joint_endpoint_summary(
    joint_states: Any,
    max_endpoint_distance: float | None,
) -> dict[str, Any]:
    if not isinstance(joint_states, dict):
        return {
            "joint_count": 0,
            "max_endpoint_distance": None,
            "average_endpoint_distance": None,
            "max_endpoint_distance_threshold": max_endpoint_distance,
            "threshold_exceeded": False,
            "missing_endpoint_count": 0,
            "farthest_joint": None,
        }

    distances: list[tuple[str, float]] = []
    missing_endpoint_count = 0
    for joint_name, state in joint_states.items():
        if not isinstance(state, dict):
            missing_endpoint_count += 1
            continue
        for endpoint_key in ["body_a", "body_b"]:
            endpoint = state.get(endpoint_key)
            if not isinstance(endpoint, dict) or endpoint.get("missing"):
                missing_endpoint_count += 1
        distance = state.get("endpoint_distance")
        if isinstance(distance, (int, float)) and distance >= 0:
            distances.append((str(joint_name), float(distance)))

    farthest = max(distances, key=lambda item: item[1]) if distances else None
    average = (
        sum(distance for _name, distance in distances) / len(distances)
        if distances
        else None
    )
    return {
        "joint_count": len(joint_states),
        "max_endpoint_distance": farthest[1] if farthest else None,
        "average_endpoint_distance": average,
        "max_endpoint_distance_threshold": max_endpoint_distance,
        "threshold_exceeded": (
            bool(farthest)
            and max_endpoint_distance is not None
            and farthest[1] > max_endpoint_distance
        ),
        "missing_endpoint_count": missing_endpoint_count,
        "farthest_joint": farthest[0] if farthest else None,
    }


def _first_mapping_value(value: Any) -> dict[str, Any]:
    if isinstance(value, dict) and value:
        first_key = next(iter(value))
        first_value = value[first_key]
        return first_value if isinstance(first_value, dict) else {}
    return {}


def _joint_endpoint_telemetry_is_complete(joint_states: Any) -> bool:
    if not isinstance(joint_states, dict):
        return False
    for state in joint_states.values():
        if not isinstance(state, dict):
            return False
        if "endpoint_distance" not in state or "axis" not in state:
            return False
        if not isinstance(state.get("relative_angle"), (int, float)):
            return False
        limits = state.get("limits")
        if not isinstance(limits, dict) or "configured" not in limits:
            return False
        applied = state.get("applied_parameters")
        if not isinstance(applied, dict) or not isinstance(applied.get("runtime"), dict):
            return False
        for endpoint_key in ["body_a", "body_b"]:
            endpoint = state.get(endpoint_key)
            if not isinstance(endpoint, dict) or endpoint.get("missing"):
                return False
            if "position" not in endpoint or "rotation" not in endpoint:
                return False
    return True


def _action_for_step(action: Any, action_sequence: list[Any] | None, index: int) -> Any:
    if not action_sequence:
        return action
    if index < len(action_sequence):
        return action_sequence[index]
    return action_sequence[-1]


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Launch Godot headless and verify dynamic robot JSON assembly."
    )
    parser.add_argument("robot_config", help="Robot JSON to load into Godot.")
    parser.add_argument(
        "--godot-exe",
        default=DEFAULT_GODOT_EXE,
        help="Path to Godot executable.",
    )
    parser.add_argument(
        "--live-profile",
        choices=sorted(LIVE_VERIFICATION_PROFILES),
        default="local",
        help="Reusable live verification profile for local/manual/scheduled runs.",
    )
    parser.add_argument(
        "--live-artifact-root",
        type=Path,
        default=None,
        help="Artifact root recorded in live verification metadata.",
    )
    parser.add_argument(
        "--live-retention-days",
        type=int,
        default=None,
        help="Retention days recorded for live verification artifacts.",
    )
    parser.add_argument(
        "--flaky-retry-attempts",
        type=int,
        default=None,
        help="Retry budget recorded in live verification flaky policy.",
    )
    parser.add_argument(
        "--dry-run-discovery",
        action="store_true",
        help="Write profile and Godot executable discovery metadata without launching Godot.",
    )
    parser.add_argument("--project", type=Path, default=repo_root / "godot_project")
    parser.add_argument(
        "--scene",
        default="res://run_rl_server.tscn",
        help="Optional scene path. Empty uses the project default main scene.",
    )
    parser.add_argument("--port", type=int, default=19090)
    parser.add_argument("--timeout-seconds", type=float, default=8.0)
    parser.add_argument(
        "--max-endpoint-distance",
        type=float,
        default=None,
        help="Optional failure threshold for joint endpoint distance.",
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
        help="Optional failure threshold requiring at least this max body displacement.",
    )
    parser.add_argument(
        "--max-linear-speed",
        type=float,
        default=None,
        help="Optional failure threshold for max final body linear speed.",
    )
    parser.add_argument(
        "--min-joint-angle-delta",
        type=float,
        default=None,
        help="Optional failure threshold for maximum joint relative_angle change across sampled telemetry.",
    )
    parser.add_argument(
        "--min-joint-angle-range",
        type=float,
        default=None,
        help="Optional failure threshold for maximum joint relative_angle range across sampled telemetry.",
    )
    parser.add_argument(
        "--min-moving-joint-coverage",
        type=float,
        default=None,
        help="Optional failure threshold for ratio of measured joints whose angle range exceeds --joint-motion-epsilon.",
    )
    parser.add_argument(
        "--min-commanded-joint-response-coverage",
        type=float,
        default=None,
        help="Optional failure threshold for ratio of valid action-targeted joints whose angle range exceeds --joint-motion-epsilon.",
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
        help="Optional failure threshold for the ratio of generated joints targeted by the action set.",
    )
    parser.add_argument(
        "--min-control-action-coverage",
        type=float,
        default=None,
        help="Optional failure threshold for the ratio of generated control.joints targeted by the action set.",
    )
    parser.add_argument(
        "--min-nonzero-action-targets",
        type=int,
        default=None,
        help="Optional failure threshold for nonzero runtime joint target_velocity count.",
    )
    parser.add_argument(
        "--min-action-transitions",
        type=int,
        default=None,
        help="Optional failure threshold for changed adjacent action payload count.",
    )
    parser.add_argument(
        "--min-action-transition-delta",
        type=float,
        default=None,
        help="Optional failure threshold for the largest numeric delta between adjacent action payloads.",
    )
    parser.add_argument(
        "--fail-on-joint-limit-violation",
        action="store_true",
        help="Fail when any generated joint reports a JSON limit violation.",
    )
    parser.add_argument(
        "--fail-on-incomplete-restoration",
        action="store_true",
        help="Fail when mechanical_restoration_summary.complete is false.",
    )
    parser.add_argument(
        "--min-restoration-score",
        type=float,
        default=None,
        help="Optional failure threshold for mechanical restoration score.",
    )
    parser.add_argument(
        "--fail-on-parameter-mismatch",
        action="store_true",
        help="Fail when source JSON joint parameters do not match runtime readback.",
    )
    parser.add_argument(
        "--fail-on-control-mismatch",
        action="store_true",
        help="Fail when source JSON control.joints parameters do not match runtime readback.",
    )
    parser.add_argument(
        "--fail-on-full-mechanical-restoration",
        action="store_true",
        help=(
            "Fail when any full mechanical restoration check fails, including "
            "mechanical_restoration_summary completeness, JSON joint parameter "
            "readback, control.joints readback, or full node-tree restoration."
        ),
    )
    parser.add_argument(
        "--fail-on-action-target-mismatch",
        action="store_true",
        help="Fail when the last action payload does not match runtime joint target_velocity readback.",
    )
    parser.add_argument(
        "--fail-on-action-sequence-target-mismatch",
        action="store_true",
        help="Fail when any action-sequence step does not match runtime joint target_velocity readback.",
    )
    parser.add_argument(
        "--fail-on-unknown-action-target",
        action="store_true",
        help="Fail when an action references a joint name or list index that is not present in telemetry.",
    )
    parser.add_argument(
        "--fail-on-invalid-action-target",
        action="store_true",
        help="Fail when an action target value is present but is not numeric.",
    )
    parser.add_argument(
        "--fail-on-incomplete-node-tree",
        action="store_true",
        help="Fail when node_tree_manifest reports missing parts or joints.",
    )
    parser.add_argument(
        "--fail-on-full-node-tree-restoration",
        action="store_true",
        help=(
            "Fail when any node_tree_manifest restoration check fails, including "
            "missing nodes, class mismatches, missing parameters, transform "
            "mismatches, physical mismatches, or fixed joint lock mismatches."
        ),
    )
    parser.add_argument(
        "--fail-on-node-tree-class-mismatch",
        action="store_true",
        help="Fail when node_tree_manifest reports unexpected generated node classes.",
    )
    parser.add_argument(
        "--fail-on-node-tree-missing-parameters",
        action="store_true",
        help="Fail when node_tree_manifest reports joints without applied_parameters.",
    )
    parser.add_argument(
        "--fail-on-node-tree-transform-mismatch",
        action="store_true",
        help="Fail when node_tree_manifest reports position, rotation, origin, or axis mismatches.",
    )
    parser.add_argument(
        "--fail-on-node-tree-physical-mismatch",
        action="store_true",
        help="Fail when node_tree_manifest reports mass, collision, or mesh parameter mismatches.",
    )
    parser.add_argument(
        "--fail-on-node-tree-fixed-lock-mismatch",
        action="store_true",
        help="Fail when node_tree_manifest reports fixed joint lock mismatches.",
    )
    parser.add_argument(
        "--node-tree-tolerance",
        type=float,
        default=1e-4,
        help="Absolute tolerance used when comparing JSON transforms with generated node mappings.",
    )
    parser.add_argument(
        "--parameter-tolerance",
        type=float,
        default=1e-4,
        help="Absolute tolerance used when comparing JSON joint parameters with runtime readback.",
    )
    parser.add_argument(
        "--action-json",
        default="[0.0]",
        help="JSON action payload sent to the Godot step command.",
    )
    parser.add_argument(
        "--action-sequence-json",
        default=None,
        help="JSON array of action payloads sent across step commands.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=1,
        help="Number of Godot step commands to send after loading the robot.",
    )
    parser.add_argument(
        "--step-delay-seconds",
        type=float,
        default=0.0,
        help="Optional delay between step commands to let physics advance.",
    )
    parser.add_argument(
        "--mechanical-trace-output",
        type=Path,
        help="Optional JSON artifact path for the full mechanical step trace.",
    )
    parser.add_argument("--output", type=Path, help="Optional JSON report path.")
    args = parser.parse_args()

    robot_config = json.loads(Path(args.robot_config).read_text(encoding="utf-8"))
    action = json.loads(args.action_json)
    action_sequence = (
        json.loads(args.action_sequence_json)
        if args.action_sequence_json is not None
        else None
    )
    if action_sequence is not None and not isinstance(action_sequence, list):
        raise ValueError("--action-sequence-json must decode to a JSON array")
    if args.parameter_tolerance < 0:
        raise ValueError("--parameter-tolerance must be greater than or equal to 0")
    if args.node_tree_tolerance < 0:
        raise ValueError("--node-tree-tolerance must be greater than or equal to 0")
    if args.min_joint_angle_delta is not None and args.min_joint_angle_delta < 0:
        raise ValueError("--min-joint-angle-delta must be greater than or equal to 0")
    if args.min_joint_angle_range is not None and args.min_joint_angle_range < 0:
        raise ValueError("--min-joint-angle-range must be greater than or equal to 0")
    if args.joint_motion_epsilon < 0:
        raise ValueError("--joint-motion-epsilon must be greater than or equal to 0")
    if args.min_moving_joint_coverage is not None and not (
        0.0 <= args.min_moving_joint_coverage <= 1.0
    ):
        raise ValueError("--min-moving-joint-coverage must be between 0 and 1")
    if args.min_commanded_joint_response_coverage is not None and not (
        0.0 <= args.min_commanded_joint_response_coverage <= 1.0
    ):
        raise ValueError("--min-commanded-joint-response-coverage must be between 0 and 1")
    if args.min_action_target_coverage is not None and not (
        0.0 <= args.min_action_target_coverage <= 1.0
    ):
        raise ValueError("--min-action-target-coverage must be between 0 and 1")
    if args.min_control_action_coverage is not None and not (
        0.0 <= args.min_control_action_coverage <= 1.0
    ):
        raise ValueError("--min-control-action-coverage must be between 0 and 1")
    if args.min_nonzero_action_targets is not None and args.min_nonzero_action_targets < 0:
        raise ValueError("--min-nonzero-action-targets must be greater than or equal to 0")
    if args.min_action_transitions is not None and args.min_action_transitions < 0:
        raise ValueError("--min-action-transitions must be greater than or equal to 0")
    if args.min_action_transition_delta is not None and args.min_action_transition_delta < 0:
        raise ValueError("--min-action-transition-delta must be greater than or equal to 0")
    if args.live_retention_days is not None and args.live_retention_days < 0:
        raise ValueError("--live-retention-days must be greater than or equal to 0")
    if args.flaky_retry_attempts is not None and args.flaky_retry_attempts < 0:
        raise ValueError("--flaky-retry-attempts must be greater than or equal to 0")
    live_verification = _live_verification_metadata(args)
    if args.dry_run_discovery:
        report = _build_discovery_report(
            robot_config=robot_config,
            live_verification=live_verification,
        )
        _write_and_print_report(report, args.output)
        return 0
    if live_verification.get("failure_category") == "missing_godot_executable":
        report = {
            "status": "error",
            "robot_name": robot_config.get("name"),
            "live_verification": live_verification,
            "errors": ["Godot executable was not found"],
            "stdout_tail": [],
            "stderr_tail": [],
        }
        _write_and_print_report(report, args.output)
        return 1
    try:
        process = _launch_godot(args)
    except OSError as exc:
        report = _build_runtime_failure_report(
            robot_config=robot_config,
            live_verification=_live_verification_metadata(
                args,
                failure_category="godot_launch_failure",
            ),
            error=exc,
        )
        _write_and_print_report(report, args.output)
        return 1

    stdout = ""
    stderr = ""
    try:
        try:
            load_result = _send_command_with_retry(
                args.port,
                {"type": "load_robot", "robot_config": robot_config},
                args.timeout_seconds,
            )
            schema_result = _send_command(args.port, {"type": "get_schema"})
            step_results = []
            actions_sent = []
            for index in range(max(args.steps, 1)):
                if index > 0 and args.step_delay_seconds > 0:
                    time.sleep(args.step_delay_seconds)
                step_action = _action_for_step(action, action_sequence, index)
                actions_sent.append(step_action)
                step_results.append(
                    _send_command(args.port, {"type": "step", "action": step_action})
                )
            step_result = step_results[-1]
        except TimeoutError as exc:
            live_verification = _live_verification_metadata(
                args,
                failure_category="godot_tcp_timeout",
            )
            report = _build_runtime_failure_report(
                robot_config=robot_config,
                live_verification=live_verification,
                error=exc,
                stdout=stdout,
                stderr=stderr,
            )
            _write_and_print_report(report, args.output)
            return 1
        except Exception as exc:
            live_verification = _live_verification_metadata(
                args,
                failure_category="godot_runtime_failure",
            )
            report = _build_runtime_failure_report(
                robot_config=robot_config,
                live_verification=live_verification,
                error=exc,
                stdout=stdout,
                stderr=stderr,
            )
            _write_and_print_report(report, args.output)
            return 1
    finally:
        process.terminate()
        try:
            process.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=3)

    fail_on_incomplete_restoration = (
        args.fail_on_incomplete_restoration
        or args.fail_on_full_mechanical_restoration
    )
    fail_on_parameter_mismatch = (
        args.fail_on_parameter_mismatch
        or args.fail_on_full_mechanical_restoration
    )
    fail_on_control_mismatch = (
        args.fail_on_control_mismatch
        or args.fail_on_full_mechanical_restoration
    )
    fail_on_full_node_tree_restoration = (
        args.fail_on_full_node_tree_restoration
        or args.fail_on_full_mechanical_restoration
    )
    fail_on_incomplete_node_tree = (
        args.fail_on_incomplete_node_tree or fail_on_full_node_tree_restoration
    )
    fail_on_node_tree_class_mismatch = (
        args.fail_on_node_tree_class_mismatch
        or fail_on_full_node_tree_restoration
    )
    fail_on_node_tree_missing_parameters = (
        args.fail_on_node_tree_missing_parameters
        or fail_on_full_node_tree_restoration
    )
    fail_on_node_tree_transform_mismatch = (
        args.fail_on_node_tree_transform_mismatch
        or fail_on_full_node_tree_restoration
    )
    fail_on_node_tree_physical_mismatch = (
        args.fail_on_node_tree_physical_mismatch
        or fail_on_full_node_tree_restoration
    )
    fail_on_node_tree_fixed_lock_mismatch = (
        args.fail_on_node_tree_fixed_lock_mismatch
        or fail_on_full_node_tree_restoration
    )

    report = _build_result(
        robot_config=robot_config,
        load_result=load_result,
        schema_result=schema_result,
        step_result=step_result,
        step_results=step_results,
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
        fail_on_incomplete_restoration=fail_on_incomplete_restoration,
        min_restoration_score=args.min_restoration_score,
        fail_on_parameter_mismatch=fail_on_parameter_mismatch,
        fail_on_control_mismatch=fail_on_control_mismatch,
        fail_on_action_target_mismatch=args.fail_on_action_target_mismatch,
        fail_on_action_sequence_target_mismatch=args.fail_on_action_sequence_target_mismatch,
        fail_on_unknown_action_target=args.fail_on_unknown_action_target,
        fail_on_invalid_action_target=args.fail_on_invalid_action_target,
        fail_on_incomplete_node_tree=fail_on_incomplete_node_tree,
        fail_on_node_tree_class_mismatch=fail_on_node_tree_class_mismatch,
        fail_on_node_tree_missing_parameters=fail_on_node_tree_missing_parameters,
        fail_on_node_tree_transform_mismatch=fail_on_node_tree_transform_mismatch,
        fail_on_node_tree_physical_mismatch=fail_on_node_tree_physical_mismatch,
        fail_on_node_tree_fixed_lock_mismatch=fail_on_node_tree_fixed_lock_mismatch,
        node_tree_tolerance=args.node_tree_tolerance,
        parameter_tolerance=args.parameter_tolerance,
        action=actions_sent[-1] if actions_sent else action,
        actions=actions_sent,
        stdout=stdout,
        stderr=stderr,
        live_verification=live_verification,
        trace_artifact_path=(
            str(args.mechanical_trace_output)
            if args.mechanical_trace_output is not None
            else None
        ),
    )
    _write_mechanical_trace_artifact(
        robot_name=robot_config.get("name"),
        step_results=step_results,
        actions=actions_sent,
        output=args.mechanical_trace_output,
    )
    _write_and_print_report(report, args.output)
    return 1 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
