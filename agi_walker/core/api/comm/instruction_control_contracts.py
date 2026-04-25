"""
Canonical contracts for Godot ecosystem product extensions.

This module closes three previously separate surfaces into one managed contract:

- Godot extension instruction-set control
- simulated circuit communication parameters
- compatibility projection back into the legacy Godot parameter plane
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, List

from agi_walker.core.api.godot_robot_env.hardware_controller import IMC22Controller

ROS2_INSTRUCTION_SET_SCHEMA_VERSION = "1.0"
SIMULATED_CIRCUIT_SCHEMA_VERSION = "1.0"
SUPPORTED_INSTRUCTION_KINDS = {
    "set_velocity",
    "set_pid",
    "set_joint_targets",
    "set_gait",
    "set_pose",
    "emergency_stop",
}
DEFAULT_JOINT_ORDER = [
    "hip_left",
    "knee_left",
    "hip_right",
    "knee_right",
]


def default_simulated_circuit_config() -> Dict[str, Any]:
    return {
        "schema_version": SIMULATED_CIRCUIT_SCHEMA_VERSION,
        "transport": "imc22_can_fd",
        "channel": "simulated-can0",
        "bustype": "virtual",
        "bitrate": 1_000_000,
        "control_freq_hz": 100,
        "status_rate_hz": 200,
        "command_base_id": IMC22Controller.ID_COMMAND_BASE,
        "status_base_id": IMC22Controller.ID_STATUS_BASE,
        "config_base_id": IMC22Controller.ID_CONFIG_BASE,
        "default_compliance": 0.5,
        "joint_order": list(DEFAULT_JOINT_ORDER),
    }


def validate_simulated_circuit_config(config: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(config, dict):
        return ["simulated circuit config must be a dict"]
    if config.get("schema_version") != SIMULATED_CIRCUIT_SCHEMA_VERSION:
        errors.append(
            f"simulated_circuit.schema_version must be {SIMULATED_CIRCUIT_SCHEMA_VERSION!r}"
        )
    if not isinstance(config.get("transport"), str) or not config["transport"]:
        errors.append("simulated_circuit.transport must be a non-empty string")
    if not isinstance(config.get("channel"), str) or not config["channel"]:
        errors.append("simulated_circuit.channel must be a non-empty string")
    if not isinstance(config.get("bustype"), str) or not config["bustype"]:
        errors.append("simulated_circuit.bustype must be a non-empty string")
    for key in [
        "bitrate",
        "control_freq_hz",
        "status_rate_hz",
        "command_base_id",
        "status_base_id",
        "config_base_id",
    ]:
        value = config.get(key)
        if not isinstance(value, int) or value <= 0:
            errors.append(f"simulated_circuit.{key} must be a positive int")
    compliance = config.get("default_compliance")
    if not isinstance(compliance, (int, float)) or not 0.0 <= float(compliance) <= 1.0:
        errors.append("simulated_circuit.default_compliance must be within [0.0, 1.0]")
    joint_order = config.get("joint_order")
    if not isinstance(joint_order, list) or not joint_order:
        errors.append("simulated_circuit.joint_order must be a non-empty list")
    elif not all(isinstance(joint_name, str) and joint_name for joint_name in joint_order):
        errors.append("simulated_circuit.joint_order entries must be non-empty strings")
    return errors


def normalize_simulated_circuit_config(
    config: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    normalized = default_simulated_circuit_config()
    if config:
        normalized.update(config)
    return normalized


def _validate_numeric_step_field(
    step: Dict[str, Any], index: int, key: str, errors: List[str]
) -> None:
    if not isinstance(step.get(key), (int, float)):
        errors.append(f"steps[{index}].{key} must be numeric")


def validate_instruction_set_payload(payload: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(payload, dict):
        return ["instruction_set payload must be a dict"]
    if payload.get("schema_version") != ROS2_INSTRUCTION_SET_SCHEMA_VERSION:
        errors.append(
            f"schema_version must be {ROS2_INSTRUCTION_SET_SCHEMA_VERSION!r}"
        )
    if not isinstance(payload.get("sequence_name"), str) or not payload["sequence_name"]:
        errors.append("sequence_name must be a non-empty string")
    steps = payload.get("steps")
    if not isinstance(steps, list) or not steps:
        errors.append("steps must be a non-empty list")
    else:
        for index, step in enumerate(steps):
            if not isinstance(step, dict):
                errors.append(f"steps[{index}] must be a dict")
                continue
            kind = step.get("kind")
            if kind not in SUPPORTED_INSTRUCTION_KINDS:
                errors.append(
                    f"steps[{index}].kind must be one of {sorted(SUPPORTED_INSTRUCTION_KINDS)!r}"
                )
                continue
            if kind == "set_velocity":
                for key in ["linear_x", "linear_y", "angular_z"]:
                    _validate_numeric_step_field(step, index, key, errors)
            elif kind == "set_pid":
                for key in ["pid_kp", "pid_ki", "pid_kd"]:
                    _validate_numeric_step_field(step, index, key, errors)
            elif kind == "set_joint_targets":
                joint_targets = step.get("joint_targets")
                if not isinstance(joint_targets, dict) or not joint_targets:
                    errors.append(
                        f"steps[{index}].joint_targets must be a non-empty dict"
                    )
                else:
                    for joint_name, value in joint_targets.items():
                        if not isinstance(joint_name, str) or not joint_name:
                            errors.append(
                                f"steps[{index}].joint_targets keys must be non-empty strings"
                            )
                        if not isinstance(value, (int, float)):
                            errors.append(
                                f"steps[{index}].joint_targets[{joint_name!r}] must be numeric"
                            )
                compliance = step.get("compliance")
                if compliance is not None and (
                    not isinstance(compliance, (int, float))
                    or not 0.0 <= float(compliance) <= 1.0
                ):
                    errors.append(
                        f"steps[{index}].compliance must be within [0.0, 1.0]"
                    )
            elif kind == "set_gait":
                if not isinstance(step.get("gait"), str) or not step["gait"]:
                    errors.append(f"steps[{index}].gait must be a non-empty string")
                for key in ["cadence_hz", "stride_scale"]:
                    _validate_numeric_step_field(step, index, key, errors)
            elif kind == "set_pose":
                if not isinstance(step.get("posture"), str) or not step["posture"]:
                    errors.append(f"steps[{index}].posture must be a non-empty string")
                _validate_numeric_step_field(step, index, "balance_gain", errors)
    simulated_circuit = payload.get("simulated_circuit")
    if simulated_circuit is not None:
        errors.extend(validate_simulated_circuit_config(simulated_circuit))
    return errors


def instruction_step_to_godot_params(step: Dict[str, Any]) -> Dict[str, Any]:
    kind = step["kind"]
    if kind == "set_velocity":
        return {
            "cmd_linear_x": float(step["linear_x"]),
            "cmd_linear_y": float(step["linear_y"]),
            "cmd_angular_z": float(step["angular_z"]),
        }
    if kind == "set_pid":
        return {
            "pid_kp": float(step["pid_kp"]),
            "pid_ki": float(step["pid_ki"]),
            "pid_kd": float(step["pid_kd"]),
        }
    if kind == "set_joint_targets":
        return {
            "joint_targets": {
                joint_name: float(target)
                for joint_name, target in step["joint_targets"].items()
            },
            "joint_target_compliance": float(step.get("compliance", 0.5)),
        }
    if kind == "set_gait":
        return {
            "gait_name": step["gait"],
            "gait_cadence_hz": float(step["cadence_hz"]),
            "gait_stride_scale": float(step["stride_scale"]),
        }
    if kind == "set_pose":
        return {
            "posture_name": step["posture"],
            "balance_gain": float(step["balance_gain"]),
        }
    if kind == "emergency_stop":
        return {"emergency_stop": True, "motor_power_multiplier": 0.0}
    raise ValueError(f"Unsupported instruction kind: {kind}")


def instruction_set_to_compatibility_params(payload: Dict[str, Any]) -> Dict[str, Any]:
    params: Dict[str, Any] = {}
    for step in payload["steps"]:
        params.update(instruction_step_to_godot_params(step))
    return params


def instruction_set_to_circuit_command_batch(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
    config = normalize_simulated_circuit_config(payload.get("simulated_circuit"))
    joint_order = config["joint_order"]
    commands: List[Dict[str, Any]] = []
    for step in payload["steps"]:
        if step["kind"] != "set_joint_targets":
            continue
        compliance = float(step.get("compliance", config["default_compliance"]))
        for index, joint_name in enumerate(joint_order, start=1):
            if joint_name not in step["joint_targets"]:
                continue
            commands.append(
                {
                    "node_id": index,
                    "joint_name": joint_name,
                    "target_angle": float(step["joint_targets"][joint_name]),
                    "compliance": compliance,
                    "command_id": config["command_base_id"] + index,
                }
            )
    return commands


def build_instruction_runtime_contract(payload: Dict[str, Any]) -> Dict[str, Any]:
    errors = validate_instruction_set_payload(payload)
    if errors:
        raise ValueError("; ".join(errors))
    normalized_payload = deepcopy(payload)
    normalized_payload["simulated_circuit"] = normalize_simulated_circuit_config(
        payload.get("simulated_circuit")
    )
    return {
        "instruction_set": normalized_payload,
        "compatibility_params": instruction_set_to_compatibility_params(
            normalized_payload
        ),
        "simulated_circuit": normalized_payload["simulated_circuit"],
        "simulated_circuit_command_batch": instruction_set_to_circuit_command_batch(
            normalized_payload
        ),
    }
