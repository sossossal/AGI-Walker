from __future__ import annotations

import pytest

from agi_walker.core.api.comm.instruction_control_contracts import (
    build_instruction_runtime_contract,
    default_simulated_circuit_config,
    instruction_set_to_circuit_command_batch,
    instruction_set_to_compatibility_params,
    validate_instruction_set_payload,
    validate_simulated_circuit_config,
)


def test_default_simulated_circuit_config_uses_canonical_parameters() -> None:
    config = default_simulated_circuit_config()

    assert config["transport"] == "imc22_can_fd"
    assert config["channel"] == "simulated-can0"
    assert config["bustype"] == "virtual"
    assert config["bitrate"] == 1_000_000
    assert config["control_freq_hz"] == 100
    assert config["status_rate_hz"] == 200
    assert config["command_base_id"] == 0x200
    assert config["status_base_id"] == 0x100
    assert config["config_base_id"] == 0x300
    assert config["joint_order"] == [
        "hip_left",
        "knee_left",
        "hip_right",
        "knee_right",
    ]


def test_simulated_circuit_config_validation_rejects_invalid_values() -> None:
    errors = validate_simulated_circuit_config(
        {
            "schema_version": "0.0",
            "transport": "",
            "channel": "",
            "bustype": "",
            "bitrate": 0,
            "control_freq_hz": -1,
            "status_rate_hz": 0,
            "command_base_id": 0,
            "status_base_id": 0,
            "config_base_id": 0,
            "default_compliance": 2.0,
            "joint_order": [],
        }
    )

    assert "simulated_circuit.schema_version must be '1.0'" in errors
    assert "simulated_circuit.transport must be a non-empty string" in errors
    assert "simulated_circuit.joint_order must be a non-empty list" in errors


def test_instruction_payload_builds_compatibility_and_circuit_batches() -> None:
    payload = {
        "schema_version": "1.0",
        "sequence_name": "demo-walk",
        "steps": [
            {"kind": "set_velocity", "linear_x": 0.3, "linear_y": 0.0, "angular_z": 0.1},
            {"kind": "set_pid", "pid_kp": 1.2, "pid_ki": 0.1, "pid_kd": 0.05},
            {
                "kind": "set_joint_targets",
                "joint_targets": {"hip_left": 0.25, "knee_right": -0.5},
                "compliance": 0.4,
            },
            {"kind": "set_gait", "gait": "trot", "cadence_hz": 2.0, "stride_scale": 1.1},
            {"kind": "emergency_stop"},
        ],
    }

    assert validate_instruction_set_payload(payload) == []

    compatibility = instruction_set_to_compatibility_params(payload)
    assert compatibility["cmd_linear_x"] == pytest.approx(0.3)
    assert compatibility["pid_kp"] == pytest.approx(1.2)
    assert compatibility["joint_targets"] == {"hip_left": 0.25, "knee_right": -0.5}
    assert compatibility["gait_name"] == "trot"
    assert compatibility["emergency_stop"] is True
    assert compatibility["motor_power_multiplier"] == 0.0

    circuit_batch = instruction_set_to_circuit_command_batch(payload)
    assert circuit_batch == [
        {
            "node_id": 1,
            "joint_name": "hip_left",
            "target_angle": 0.25,
            "compliance": 0.4,
            "command_id": 0x201,
        },
        {
            "node_id": 4,
            "joint_name": "knee_right",
            "target_angle": -0.5,
            "compliance": 0.4,
            "command_id": 0x204,
        },
    ]

    runtime = build_instruction_runtime_contract(payload)
    assert runtime["compatibility_params"] == compatibility
    assert runtime["simulated_circuit_command_batch"] == circuit_batch


def test_instruction_payload_validation_rejects_bad_step_shapes() -> None:
    errors = validate_instruction_set_payload(
        {
            "schema_version": "1.0",
            "sequence_name": "",
            "steps": [
                {"kind": "set_velocity", "linear_x": "fast", "linear_y": 0.0, "angular_z": 0.1},
                {"kind": "set_joint_targets", "joint_targets": {"hip_left": "bad"}},
            ],
        }
    )

    assert "sequence_name must be a non-empty string" in errors
    assert "steps[0].linear_x must be numeric" in errors
    assert "steps[1].joint_targets['hip_left'] must be numeric" in errors
