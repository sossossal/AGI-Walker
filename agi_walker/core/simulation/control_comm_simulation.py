"""Deterministic control/communication simulation evidence contracts."""

from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Any

CONTROL_COMM_SIMULATION_REPORT_VERSION = "control_comm_simulation_report.v1"
CONTROL_MESSAGE_ENVELOPE_VERSION = "control_message_envelope.v1"
GODOT_CONTROL_COMM_SIMULATION_LOG_VERSION = "godot_control_comm_simulation_log.v1"
ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION = "zenoh_openneuro_topic_mapping.v1"
ETHERCAT_MODEL_TRACE_VERSION = "ethercat_model_trace.v1"
MOTOR_JOINT_RESPONSE_TRACE_VERSION = "motor_joint_response_trace.v1"
SIMULATOR_ADAPTER_BOUNDARY_VERSION = "simulator_adapter_boundary.v1"
LIVE_HARDWARE_MIGRATION_GATE_VERSION = "live_hardware_migration_gate.v1"

EVIDENCE_SOURCE_PYTHON_VIRTUAL_CLOCK = "python_virtual_clock"
EVIDENCE_SOURCE_GODOT_SCRIPT = "godot_script"
EVIDENCE_SOURCE_GODOT_HEADLESS = "godot_headless"
GODOT_LOG_NOT_RUN_STATUS = "not_run"

TRANSPORT_MODE_ASYNCIO = "asyncio"
TRANSPORT_MODE_ZENOH_SIMULATED = "zenoh_simulated"
TRANSPORT_MODE_ETHERCAT_MODEL = "ethercat_model"
CLOCK_MODE_VIRTUAL = "virtual"
LOCAL_ASYNCIO_BUS_MODE = "local_asyncio_virtual"
ETHERCAT_CYCLE_MODEL_MODE = "ethercat_cycle_model"
MOTOR_JOINT_MODEL_MODE = "motor_joint_model"
SUPPORTED_SIMULATOR_ADAPTERS = ("gazebo", "mujoco", "isaac_sim")
SUPPORTED_LIVE_HARDWARE_TRANSPORTS = ("can", "ethercat", "tsn")


def build_control_message_envelope(
    *,
    topic: str,
    sequence: int,
    timestamp_ns: int,
    source: str,
    target: str,
    payload_type: str,
    payload: dict[str, Any],
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        "schema_version": CONTROL_MESSAGE_ENVELOPE_VERSION,
        "topic": topic,
        "sequence": sequence,
        "timestamp_ns": timestamp_ns,
        "source": source,
        "target": target,
        "payload_type": payload_type,
        "payload": payload,
        "metadata": metadata or {},
    }


def validate_control_message_envelope(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["control message envelope must be an object"]

    errors: list[str] = []
    if payload.get("schema_version") != CONTROL_MESSAGE_ENVELOPE_VERSION:
        errors.append(
            f"schema_version must be {CONTROL_MESSAGE_ENVELOPE_VERSION!r}"
        )
    for field in ["topic", "source", "target", "payload_type"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if not _is_non_negative_int(payload.get("sequence")):
        errors.append("sequence must be a non-negative integer")
    if not _is_non_negative_int(payload.get("timestamp_ns")):
        errors.append("timestamp_ns must be a non-negative integer")
    if not isinstance(payload.get("payload"), dict):
        errors.append("payload must be an object")
    if not isinstance(payload.get("metadata"), dict):
        errors.append("metadata must be an object")
    return errors


def build_default_control_comm_scenario(
    *,
    scenario_id: str = "default_joint_command_stream",
    cycle_count: int = 4,
    cycle_period_ns: int = 10_000_000,
) -> dict[str, Any]:
    _require_positive_int(cycle_count, "cycle_count")
    _require_positive_int(cycle_period_ns, "cycle_period_ns")
    return {
        "schema_version": "control_comm_simulation_scenario.v1",
        "scenario_id": scenario_id,
        "cycle_count": cycle_count,
        "cycle_period_ns": cycle_period_ns,
        "jitter_budget_ns": 0,
        "bus": {
            "mode": LOCAL_ASYNCIO_BUS_MODE,
            "latency_ns": 0,
            "jitter_by_cycle_ns": [],
            "drop_sequences": [],
            "duplicate_sequences": [],
        },
        "ethercat": {
            "mode": ETHERCAT_CYCLE_MODEL_MODE,
            "watchdog_timeout_cycles": 2,
            "deadline_budget_ns": 0,
            "pdo_input_map": ["target_velocity_rad_s", "target_torque_nm"],
            "pdo_output_map": ["position_rad", "velocity_rad_s", "torque_nm"],
        },
        "motor_joint": {
            "mode": MOTOR_JOINT_MODEL_MODE,
            "joint_name": "left_hip",
            "position_lower_rad": -1.0,
            "position_upper_rad": 1.0,
            "velocity_limit_rad_s": 1.0,
            "torque_limit_nm": 2.0,
            "friction_nm": 0.0,
            "backlash_rad": 0.0,
        },
        "source": "controller",
        "target": "joint_endpoint.left_hip",
        "topic": "agi/control/joint/left_hip/command",
        "payload_type": "joint_velocity_command",
        "command": {
            "joint_name": "left_hip",
            "target_velocity_rad_s": 0.25,
            "target_torque_nm": 1.5,
        },
    }


def run_deterministic_control_comm_simulation(
    scenario: dict[str, Any],
    *,
    output_root: Path | None = None,
    include_godot_log_contract: bool = True,
    include_zenoh_openneuro_simulation: bool = True,
    include_ethercat_cycle_model: bool = True,
    include_motor_joint_model: bool = True,
    include_simulator_adapter_boundary: bool = True,
    include_live_hardware_migration_gate: bool = True,
) -> dict[str, Any]:
    errors = validate_control_comm_scenario(scenario)
    if errors:
        raise ValueError("; ".join(errors))

    cycle_count = int(scenario["cycle_count"])
    cycle_period_ns = int(scenario["cycle_period_ns"])
    jitter_budget_ns = int(scenario.get("jitter_budget_ns", 0))
    envelopes: list[dict[str, Any]] = []
    for cycle_index in range(cycle_count):
        timestamp_ns = cycle_index * cycle_period_ns
        envelope = build_control_message_envelope(
            topic=scenario["topic"],
            sequence=cycle_index,
            timestamp_ns=timestamp_ns,
            source=scenario["source"],
            target=scenario["target"],
            payload_type=scenario["payload_type"],
            payload=scenario["command"],
            metadata={"scenario_id": scenario["scenario_id"], "cycle_index": cycle_index},
        )
        envelopes.append(envelope)
    bus_result = run_local_asyncio_bus(
        envelopes=envelopes,
        bus_config=scenario.get("bus", {}),
        jitter_budget_ns=jitter_budget_ns,
    )
    timing_trace = bus_result["timing_trace"]
    message_trace = bus_result["message_trace"]
    zenoh_mapping = build_zenoh_openneuro_topic_mapping(scenario)
    zenoh_trace = build_zenoh_simulated_trace(
        mapping=zenoh_mapping,
        message_trace=message_trace,
    )
    ethercat_trace = build_ethercat_model_trace(
        scenario=scenario,
        timing_trace=timing_trace,
        message_trace=message_trace,
    )
    motor_joint_trace = build_motor_joint_response_trace(
        scenario=scenario,
        ethercat_trace=ethercat_trace,
    )
    simulator_adapter_boundary = build_simulator_adapter_boundary(
        scenario=scenario,
        artifact_paths={
            "timing_trace": "timing_trace.json",
            "message_trace": "message_trace.json",
            "ethercat_model_trace": "ethercat_model_trace.json",
            "motor_joint_response_trace": "motor_joint_response_trace.json",
        },
    )
    live_hardware_migration_gate = build_live_hardware_migration_gate(
        scenario=scenario,
        artifact_paths={
            "timing_trace": "timing_trace.json",
            "message_trace": "message_trace.json",
            "zenoh_simulated_trace": "zenoh_simulated_trace.json",
            "ethercat_model_trace": "ethercat_model_trace.json",
            "motor_joint_response_trace": "motor_joint_response_trace.json",
            "simulator_adapter_boundary": "simulator_adapter_boundary.json",
        },
    )

    artifact_paths: dict[str, str] = {}
    godot_log_contract = build_godot_log_contract_preview(
        scenario=scenario,
        envelopes=envelopes,
    )
    if output_root is not None:
        output_root.mkdir(parents=True, exist_ok=True)
        artifact_paths["timing_trace"] = _write_json(
            output_root / "timing_trace.json", timing_trace
        )
        artifact_paths["message_trace"] = _write_json(
            output_root / "message_trace.json", message_trace
        )
        artifact_paths["godot_replay_scenario"] = _write_json(
            output_root / "godot_replay_scenario.json", scenario
        )
        if include_godot_log_contract:
            artifact_paths["godot_log_contract_preview"] = _write_json(
                output_root / "godot_control_comm_simulation_log.json",
                godot_log_contract,
            )
        if include_zenoh_openneuro_simulation:
            artifact_paths["zenoh_openneuro_topic_mapping"] = _write_json(
                output_root / "zenoh_openneuro_topic_mapping.json",
                zenoh_mapping,
            )
            artifact_paths["zenoh_simulated_trace"] = _write_json(
                output_root / "zenoh_simulated_trace.json",
                zenoh_trace,
            )
        if include_ethercat_cycle_model:
            artifact_paths["ethercat_model_trace"] = _write_json(
                output_root / "ethercat_model_trace.json",
                ethercat_trace,
            )
        if include_motor_joint_model:
            artifact_paths["motor_joint_response_trace"] = _write_json(
                output_root / "motor_joint_response_trace.json",
                motor_joint_trace,
            )
        if include_simulator_adapter_boundary:
            artifact_paths["simulator_adapter_boundary"] = _write_json(
                output_root / "simulator_adapter_boundary.json",
                simulator_adapter_boundary,
            )
        if include_live_hardware_migration_gate:
            artifact_paths["live_hardware_migration_gate"] = _write_json(
                output_root / "live_hardware_migration_gate.json",
                live_hardware_migration_gate,
            )

    report = {
        "report_version": CONTROL_COMM_SIMULATION_REPORT_VERSION,
        "status": "success",
        "scenario_id": scenario["scenario_id"],
        "transport_mode": TRANSPORT_MODE_ASYNCIO,
        "simulated_transport_modes": _simulated_transport_modes(
            include_zenoh_openneuro_simulation=include_zenoh_openneuro_simulation,
            include_ethercat_cycle_model=include_ethercat_cycle_model,
            include_motor_joint_model=include_motor_joint_model,
        ),
        "clock_mode": CLOCK_MODE_VIRTUAL,
        "evidence_sources": [EVIDENCE_SOURCE_PYTHON_VIRTUAL_CLOCK],
        "cycle_period_ns": cycle_period_ns,
        "cycle_count": cycle_count,
        "jitter_budget_ns": jitter_budget_ns,
        "local_asyncio_bus": bus_result["bus_summary"],
        "timing_metrics": {
            "cycles": cycle_count,
            "max_abs_jitter_ns": bus_result["max_abs_jitter_ns"],
            "deadline_miss_count": bus_result["deadline_miss_count"],
        },
        "message_integrity": bus_result["message_integrity"],
        "artifact_paths": artifact_paths,
        "godot_log_evidence": {
            "available": False,
            "status": GODOT_LOG_NOT_RUN_STATUS,
            "contract_version": GODOT_CONTROL_COMM_SIMULATION_LOG_VERSION,
            "artifact_path": artifact_paths.get("godot_log_contract_preview"),
        },
        "zenoh_openneuro_simulation": {
            "available": include_zenoh_openneuro_simulation,
            "status": "simulated" if include_zenoh_openneuro_simulation else "not_run",
            "transport_mode": TRANSPORT_MODE_ZENOH_SIMULATED,
            "mapping_version": ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION,
            "topic_mapping_artifact_path": artifact_paths.get(
                "zenoh_openneuro_topic_mapping"
            ),
            "trace_artifact_path": artifact_paths.get("zenoh_simulated_trace"),
            "event_count": len(zenoh_trace["events"])
            if include_zenoh_openneuro_simulation
            else 0,
            "compatibility_claim": "simulation_only",
            "residual_risks": [
                "real_zenoh_session_not_run",
                "openneuro_compatibility_not_claimed",
            ],
        },
        "ethercat_cycle_model": {
            "available": include_ethercat_cycle_model,
            "status": "simulated" if include_ethercat_cycle_model else "not_run",
            "transport_mode": TRANSPORT_MODE_ETHERCAT_MODEL,
            "trace_version": ETHERCAT_MODEL_TRACE_VERSION,
            "trace_artifact_path": artifact_paths.get("ethercat_model_trace"),
            "cycle_count": ethercat_trace["cycle_count"]
            if include_ethercat_cycle_model
            else 0,
            "deadline_miss_count": ethercat_trace["summary"]["deadline_miss_count"]
            if include_ethercat_cycle_model
            else 0,
            "watchdog_trip_count": ethercat_trace["summary"]["watchdog_trip_count"]
            if include_ethercat_cycle_model
            else 0,
            "compatibility_claim": "simulation_only",
            "residual_risks": [
                "real_ethercat_master_not_run",
                "real_fieldbus_hardware_not_run",
            ],
        },
        "motor_joint_model": {
            "available": include_motor_joint_model,
            "status": "simulated" if include_motor_joint_model else "not_run",
            "trace_version": MOTOR_JOINT_RESPONSE_TRACE_VERSION,
            "trace_artifact_path": artifact_paths.get("motor_joint_response_trace"),
            "step_count": motor_joint_trace["step_count"]
            if include_motor_joint_model
            else 0,
            "saturation_count": motor_joint_trace["summary"]["saturation_count"]
            if include_motor_joint_model
            else 0,
            "limit_violation_count": motor_joint_trace["summary"][
                "limit_violation_count"
            ]
            if include_motor_joint_model
            else 0,
            "fault_count": motor_joint_trace["summary"]["fault_count"]
            if include_motor_joint_model
            else 0,
            "compatibility_claim": "simulation_only",
            "residual_risks": [
                "real_motor_driver_not_run",
                "physical_joint_dynamics_simplified",
            ],
        },
        "simulator_adapter_boundary": {
            "available": include_simulator_adapter_boundary,
            "status": "planned" if include_simulator_adapter_boundary else "not_run",
            "boundary_version": SIMULATOR_ADAPTER_BOUNDARY_VERSION,
            "artifact_path": artifact_paths.get("simulator_adapter_boundary"),
            "supported_adapters": list(SUPPORTED_SIMULATOR_ADAPTERS),
            "runtime_dependency_required": False,
            "compatibility_claim": "adapter_contract_only",
            "residual_risks": [
                "gazebo_runtime_not_run",
                "mujoco_runtime_not_run",
                "isaac_sim_runtime_not_run",
            ],
        },
        "live_hardware_migration_gate": {
            "available": include_live_hardware_migration_gate,
            "status": "blocked"
            if include_live_hardware_migration_gate
            else "not_run",
            "gate_version": LIVE_HARDWARE_MIGRATION_GATE_VERSION,
            "artifact_path": artifact_paths.get("live_hardware_migration_gate"),
            "required_transports": list(SUPPORTED_LIVE_HARDWARE_TRANSPORTS),
            "external_evidence_required": True,
            "simulation_substitute_allowed": False,
            "release_gate_status": "blocked",
            "compatibility_claim": "migration_gate_only",
            "residual_risks": [
                "real_can_transport_not_run",
                "real_ethercat_transport_not_run",
                "real_tsn_transport_not_run",
            ],
        },
        "residual_risks": [
            "godot_replay_not_run",
            "real_zenoh_session_not_run",
            "real_ethercat_master_not_run",
            "real_motor_driver_not_run",
            "external_simulator_runtime_not_run",
            "real_hardware_transport_not_run",
            "live_hardware_migration_gate_blocked",
        ],
        "errors": [],
    }
    return report


def validate_control_comm_scenario(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["control communication scenario must be an object"]

    errors: list[str] = []
    if payload.get("schema_version") != "control_comm_simulation_scenario.v1":
        errors.append("schema_version must be 'control_comm_simulation_scenario.v1'")
    for field in ["scenario_id", "source", "target", "topic", "payload_type"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    for field in ["cycle_count", "cycle_period_ns"]:
        if not _is_positive_int(payload.get(field)):
            errors.append(f"{field} must be a positive integer")
    if not _is_non_negative_int(payload.get("jitter_budget_ns")):
        errors.append("jitter_budget_ns must be a non-negative integer")
    if not isinstance(payload.get("command"), dict):
        errors.append("command must be an object")
    bus_errors = validate_local_asyncio_bus_config(payload.get("bus", {}))
    errors.extend(bus_errors)
    ethercat_errors = validate_ethercat_model_config(
        payload.get("ethercat", _default_ethercat_model_config())
    )
    errors.extend(ethercat_errors)
    motor_joint_errors = validate_motor_joint_model_config(
        payload.get("motor_joint", _default_motor_joint_model_config())
    )
    errors.extend(motor_joint_errors)
    return errors


def validate_local_asyncio_bus_config(payload: Any) -> list[str]:
    if payload is None:
        return []
    if not isinstance(payload, dict):
        return ["bus must be an object"]

    errors: list[str] = []
    mode = payload.get("mode", LOCAL_ASYNCIO_BUS_MODE)
    if mode != LOCAL_ASYNCIO_BUS_MODE:
        errors.append(f"bus.mode must be {LOCAL_ASYNCIO_BUS_MODE!r}")
    if not _is_non_negative_int(payload.get("latency_ns", 0)):
        errors.append("bus.latency_ns must be a non-negative integer")
    jitter_values = payload.get("jitter_by_cycle_ns", [])
    if not isinstance(jitter_values, list) or not all(
        _is_int(value) for value in jitter_values
    ):
        errors.append("bus.jitter_by_cycle_ns must be a list of integers")
    for field in ["drop_sequences", "duplicate_sequences"]:
        values = payload.get(field, [])
        if not isinstance(values, list) or not all(
            _is_non_negative_int(value) for value in values
        ):
            errors.append(f"bus.{field} must be a list of non-negative integers")
    return errors


def _default_ethercat_model_config() -> dict[str, Any]:
    return {
        "mode": ETHERCAT_CYCLE_MODEL_MODE,
        "watchdog_timeout_cycles": 2,
        "deadline_budget_ns": 0,
        "pdo_input_map": ["target_velocity_rad_s", "target_torque_nm"],
        "pdo_output_map": ["position_rad", "velocity_rad_s", "torque_nm"],
    }


def validate_ethercat_model_config(payload: Any) -> list[str]:
    if payload is None:
        return []
    if not isinstance(payload, dict):
        return ["ethercat must be an object"]

    errors: list[str] = []
    mode = payload.get("mode", ETHERCAT_CYCLE_MODEL_MODE)
    if mode != ETHERCAT_CYCLE_MODEL_MODE:
        errors.append(f"ethercat.mode must be {ETHERCAT_CYCLE_MODEL_MODE!r}")
    if not _is_positive_int(payload.get("watchdog_timeout_cycles", 2)):
        errors.append("ethercat.watchdog_timeout_cycles must be a positive integer")
    if not _is_non_negative_int(payload.get("deadline_budget_ns", 0)):
        errors.append("ethercat.deadline_budget_ns must be a non-negative integer")
    for field in ["pdo_input_map", "pdo_output_map"]:
        values = payload.get(field, [])
        if not isinstance(values, list) or not all(
            _is_non_empty_string(value) for value in values
        ):
            errors.append(f"ethercat.{field} must be a list of non-empty strings")
    return errors


def normalize_ethercat_model_config(payload: dict[str, Any] | None) -> dict[str, Any]:
    config = _default_ethercat_model_config()
    if payload:
        config.update(payload)
    errors = validate_ethercat_model_config(config)
    if errors:
        raise ValueError("; ".join(errors))
    return config


def build_ethercat_model_trace(
    *,
    scenario: dict[str, Any],
    timing_trace: list[dict[str, Any]],
    message_trace: list[dict[str, Any]],
) -> dict[str, Any]:
    scenario_errors = validate_control_comm_scenario(scenario)
    if scenario_errors:
        raise ValueError("; ".join(scenario_errors))
    config = normalize_ethercat_model_config(scenario.get("ethercat"))
    messages_by_cycle = _first_message_by_cycle(message_trace)

    cycles: list[dict[str, Any]] = []
    consecutive_misses = 0
    watchdog_trip_count = 0
    for timing in timing_trace:
        if not isinstance(timing, dict):
            raise ValueError("timing_trace entries must be objects")
        cycle_index = int(timing["cycle_index"])
        message = messages_by_cycle.get(cycle_index)
        envelope = message.get("envelope") if message else None
        envelope_errors = validate_control_message_envelope(envelope)
        if envelope_errors:
            raise ValueError(
                "; ".join(
                    f"cycle[{cycle_index}].envelope.{error}"
                    for error in envelope_errors
                )
            )

        expected_timestamp_ns = int(timing["expected_timestamp_ns"])
        actual_timestamp_ns = int(timing["actual_timestamp_ns"])
        cycle_jitter_ns = actual_timestamp_ns - expected_timestamp_ns
        deadline_missed = cycle_jitter_ns > int(config["deadline_budget_ns"])
        delivery_outcome = str(message.get("delivery_outcome", "missing"))
        frame_missing = delivery_outcome == "dropped"
        if deadline_missed or frame_missing:
            consecutive_misses += 1
        else:
            consecutive_misses = 0
        watchdog_tripped = consecutive_misses >= int(config["watchdog_timeout_cycles"])
        if watchdog_tripped:
            watchdog_trip_count += 1

        pdo_inputs = _build_pdo_inputs(envelope, config["pdo_input_map"])
        pdo_outputs = _build_pdo_outputs(
            pdo_inputs=pdo_inputs,
            pdo_output_map=config["pdo_output_map"],
            watchdog_tripped=watchdog_tripped,
            frame_missing=frame_missing,
        )
        cycles.append(
            {
                "cycle_index": cycle_index,
                "expected_timestamp_ns": expected_timestamp_ns,
                "actual_timestamp_ns": actual_timestamp_ns,
                "cycle_jitter_ns": cycle_jitter_ns,
                "deadline_budget_ns": config["deadline_budget_ns"],
                "deadline_missed": deadline_missed,
                "pdo_inputs": pdo_inputs,
                "pdo_outputs": pdo_outputs,
                "frame_status": "missing" if frame_missing else "received",
                "watchdog_state": "tripped" if watchdog_tripped else "armed",
                "fault_class": _ethercat_fault_class(
                    deadline_missed=deadline_missed,
                    frame_missing=frame_missing,
                    watchdog_tripped=watchdog_tripped,
                ),
                "envelope": envelope,
            }
        )

    deadline_miss_count = sum(1 for cycle in cycles if cycle["deadline_missed"])
    frame_missing_count = sum(1 for cycle in cycles if cycle["frame_status"] == "missing")
    return {
        "trace_version": ETHERCAT_MODEL_TRACE_VERSION,
        "transport_mode": TRANSPORT_MODE_ETHERCAT_MODEL,
        "mode": ETHERCAT_CYCLE_MODEL_MODE,
        "scenario_id": scenario["scenario_id"],
        "cycle_period_ns": scenario["cycle_period_ns"],
        "cycle_count": len(cycles),
        "watchdog_timeout_cycles": config["watchdog_timeout_cycles"],
        "pdo_input_map": config["pdo_input_map"],
        "pdo_output_map": config["pdo_output_map"],
        "cycles": cycles,
        "summary": {
            "deadline_miss_count": deadline_miss_count,
            "frame_missing_count": frame_missing_count,
            "watchdog_trip_count": watchdog_trip_count,
        },
        "compatibility_claim": "simulation_only",
        "residual_risks": [
            "real_ethercat_master_not_run",
            "real_fieldbus_hardware_not_run",
        ],
    }


def validate_ethercat_model_trace(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["ethercat model trace must be an object"]

    errors: list[str] = []
    if payload.get("trace_version") != ETHERCAT_MODEL_TRACE_VERSION:
        errors.append(f"trace_version must be {ETHERCAT_MODEL_TRACE_VERSION!r}")
    if payload.get("transport_mode") != TRANSPORT_MODE_ETHERCAT_MODEL:
        errors.append(f"transport_mode must be {TRANSPORT_MODE_ETHERCAT_MODEL!r}")
    if payload.get("mode") != ETHERCAT_CYCLE_MODEL_MODE:
        errors.append(f"mode must be {ETHERCAT_CYCLE_MODEL_MODE!r}")
    for field in ["scenario_id", "compatibility_claim"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("compatibility_claim") != "simulation_only":
        errors.append("compatibility_claim must be 'simulation_only'")
    for field in ["cycle_period_ns", "cycle_count", "watchdog_timeout_cycles"]:
        if not _is_positive_int(payload.get(field)):
            errors.append(f"{field} must be a positive integer")
    for field in ["pdo_input_map", "pdo_output_map", "residual_risks"]:
        if not isinstance(payload.get(field), list):
            errors.append(f"{field} must be a list")
    if not isinstance(payload.get("summary"), dict):
        errors.append("summary must be an object")
    cycles = payload.get("cycles")
    if not isinstance(cycles, list):
        errors.append("cycles must be a list")
        return errors
    if _is_positive_int(payload.get("cycle_count")) and int(
        payload["cycle_count"]
    ) != len(cycles):
        errors.append("cycle_count must equal len(cycles)")
    for index, cycle in enumerate(cycles):
        if not isinstance(cycle, dict):
            errors.append(f"cycles[{index}] must be an object")
            continue
        for field in [
            "cycle_index",
            "expected_timestamp_ns",
            "actual_timestamp_ns",
            "cycle_jitter_ns",
            "deadline_budget_ns",
        ]:
            if not _is_int(cycle.get(field)):
                errors.append(f"cycles[{index}].{field} must be an integer")
        if not isinstance(cycle.get("deadline_missed"), bool):
            errors.append(f"cycles[{index}].deadline_missed must be a boolean")
        for field in ["pdo_inputs", "pdo_outputs"]:
            if not isinstance(cycle.get(field), dict):
                errors.append(f"cycles[{index}].{field} must be an object")
        for field in ["frame_status", "watchdog_state", "fault_class"]:
            if not _is_non_empty_string(cycle.get(field)):
                errors.append(f"cycles[{index}].{field} must be a non-empty string")
        envelope_errors = validate_control_message_envelope(cycle.get("envelope"))
        errors.extend(
            f"cycles[{index}].envelope.{error}" for error in envelope_errors
        )
    if isinstance(payload.get("summary"), dict):
        for field in [
            "deadline_miss_count",
            "frame_missing_count",
            "watchdog_trip_count",
        ]:
            if not _is_non_negative_int(payload["summary"].get(field)):
                errors.append(f"summary.{field} must be a non-negative integer")
    return errors


def _default_motor_joint_model_config() -> dict[str, Any]:
    return {
        "mode": MOTOR_JOINT_MODEL_MODE,
        "joint_name": "left_hip",
        "position_lower_rad": -1.0,
        "position_upper_rad": 1.0,
        "velocity_limit_rad_s": 1.0,
        "torque_limit_nm": 2.0,
        "friction_nm": 0.0,
        "backlash_rad": 0.0,
    }


def validate_motor_joint_model_config(payload: Any) -> list[str]:
    if payload is None:
        return []
    if not isinstance(payload, dict):
        return ["motor_joint must be an object"]

    errors: list[str] = []
    if payload.get("mode", MOTOR_JOINT_MODEL_MODE) != MOTOR_JOINT_MODEL_MODE:
        errors.append(f"motor_joint.mode must be {MOTOR_JOINT_MODEL_MODE!r}")
    if not _is_non_empty_string(payload.get("joint_name", "left_hip")):
        errors.append("motor_joint.joint_name must be a non-empty string")
    for field in [
        "position_lower_rad",
        "position_upper_rad",
        "velocity_limit_rad_s",
        "torque_limit_nm",
        "friction_nm",
        "backlash_rad",
    ]:
        if not _is_number(payload.get(field, _default_motor_joint_model_config()[field])):
            errors.append(f"motor_joint.{field} must be a number")
    lower = payload.get("position_lower_rad", -1.0)
    upper = payload.get("position_upper_rad", 1.0)
    if _is_number(lower) and _is_number(upper) and float(lower) >= float(upper):
        errors.append("motor_joint.position_lower_rad must be less than position_upper_rad")
    for field in ["velocity_limit_rad_s", "torque_limit_nm"]:
        value = payload.get(field, _default_motor_joint_model_config()[field])
        if _is_number(value) and float(value) <= 0:
            errors.append(f"motor_joint.{field} must be greater than zero")
    for field in ["friction_nm", "backlash_rad"]:
        value = payload.get(field, _default_motor_joint_model_config()[field])
        if _is_number(value) and float(value) < 0:
            errors.append(f"motor_joint.{field} must be non-negative")
    return errors


def normalize_motor_joint_model_config(payload: dict[str, Any] | None) -> dict[str, Any]:
    config = _default_motor_joint_model_config()
    if payload:
        config.update(payload)
    errors = validate_motor_joint_model_config(config)
    if errors:
        raise ValueError("; ".join(errors))
    return config


def build_motor_joint_response_trace(
    *,
    scenario: dict[str, Any],
    ethercat_trace: dict[str, Any],
) -> dict[str, Any]:
    config = normalize_motor_joint_model_config(scenario.get("motor_joint"))
    trace_errors = validate_ethercat_model_trace(ethercat_trace)
    if trace_errors:
        raise ValueError("; ".join(trace_errors))

    cycle_period_s = float(scenario["cycle_period_ns"]) / 1_000_000_000.0
    lower = float(config["position_lower_rad"])
    upper = float(config["position_upper_rad"])
    velocity_limit = float(config["velocity_limit_rad_s"])
    torque_limit = float(config["torque_limit_nm"])
    friction = float(config["friction_nm"])
    position = 0.0
    steps: list[dict[str, Any]] = []

    for cycle in ethercat_trace["cycles"]:
        pdo_outputs = cycle["pdo_outputs"]
        commanded_velocity = float(pdo_outputs.get("velocity_rad_s", 0) or 0)
        commanded_torque = float(pdo_outputs.get("torque_nm", 0) or 0)
        velocity = _clamp(commanded_velocity, -velocity_limit, velocity_limit)
        torque_after_friction = _apply_friction(commanded_torque, friction)
        torque = _clamp(torque_after_friction, -torque_limit, torque_limit)
        saturation = (
            velocity != commanded_velocity
            or torque != torque_after_friction
            or torque_after_friction != commanded_torque
        )
        next_position = position + velocity * cycle_period_s
        limit_state = "within_limits"
        limit_violation = False
        if next_position < lower:
            next_position = lower
            limit_state = "lower_limit_clamped"
            limit_violation = True
        elif next_position > upper:
            next_position = upper
            limit_state = "upper_limit_clamped"
            limit_violation = True
        fault_state = (
            "nominal"
            if cycle["fault_class"] == "none"
            else f"fieldbus_{cycle['fault_class']}"
        )
        steps.append(
            {
                "step_index": int(cycle["cycle_index"]),
                "joint_name": config["joint_name"],
                "command": {
                    "target_velocity_rad_s": commanded_velocity,
                    "target_torque_nm": commanded_torque,
                },
                "position_rad": next_position,
                "velocity_rad_s": velocity,
                "torque_nm": torque,
                "limits": {
                    "position_lower_rad": lower,
                    "position_upper_rad": upper,
                    "velocity_limit_rad_s": velocity_limit,
                    "torque_limit_nm": torque_limit,
                },
                "saturation": saturation,
                "limit_state": limit_state,
                "limit_violation": limit_violation,
                "fault_state": fault_state,
                "friction_nm": friction,
                "backlash_rad": float(config["backlash_rad"]),
                "source_cycle_fault_class": cycle["fault_class"],
            }
        )
        position = next_position

    return {
        "trace_version": MOTOR_JOINT_RESPONSE_TRACE_VERSION,
        "mode": MOTOR_JOINT_MODEL_MODE,
        "scenario_id": scenario["scenario_id"],
        "joint_name": config["joint_name"],
        "step_count": len(steps),
        "steps": steps,
        "schema_mapping": {
            "actuator_fields": ["torque_limit_nm", "velocity_limit_rad_s"],
            "joint_limit_fields": ["position_lower_rad", "position_upper_rad"],
            "controller_fields": ["target_velocity_rad_s", "target_torque_nm"],
        },
        "summary": {
            "saturation_count": sum(1 for step in steps if step["saturation"]),
            "limit_violation_count": sum(
                1 for step in steps if step["limit_violation"]
            ),
            "fault_count": sum(1 for step in steps if step["fault_state"] != "nominal"),
        },
        "compatibility_claim": "simulation_only",
        "residual_risks": [
            "real_motor_driver_not_run",
            "physical_joint_dynamics_simplified",
        ],
    }


def validate_motor_joint_response_trace(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["motor joint response trace must be an object"]

    errors: list[str] = []
    if payload.get("trace_version") != MOTOR_JOINT_RESPONSE_TRACE_VERSION:
        errors.append(f"trace_version must be {MOTOR_JOINT_RESPONSE_TRACE_VERSION!r}")
    if payload.get("mode") != MOTOR_JOINT_MODEL_MODE:
        errors.append(f"mode must be {MOTOR_JOINT_MODEL_MODE!r}")
    for field in ["scenario_id", "joint_name", "compatibility_claim"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("compatibility_claim") != "simulation_only":
        errors.append("compatibility_claim must be 'simulation_only'")
    if not _is_non_negative_int(payload.get("step_count")):
        errors.append("step_count must be a non-negative integer")
    for field in ["schema_mapping", "summary"]:
        if not isinstance(payload.get(field), dict):
            errors.append(f"{field} must be an object")
    if not isinstance(payload.get("residual_risks"), list):
        errors.append("residual_risks must be a list")
    steps = payload.get("steps")
    if not isinstance(steps, list):
        errors.append("steps must be a list")
        return errors
    if _is_non_negative_int(payload.get("step_count")) and int(
        payload["step_count"]
    ) != len(steps):
        errors.append("step_count must equal len(steps)")
    for index, step in enumerate(steps):
        if not isinstance(step, dict):
            errors.append(f"steps[{index}] must be an object")
            continue
        if not _is_non_negative_int(step.get("step_index")):
            errors.append(f"steps[{index}].step_index must be a non-negative integer")
        if not _is_non_empty_string(step.get("joint_name")):
            errors.append(f"steps[{index}].joint_name must be a non-empty string")
        for field in ["command", "limits"]:
            if not isinstance(step.get(field), dict):
                errors.append(f"steps[{index}].{field} must be an object")
        for field in ["position_rad", "velocity_rad_s", "torque_nm", "friction_nm", "backlash_rad"]:
            if not _is_number(step.get(field)):
                errors.append(f"steps[{index}].{field} must be a number")
        for field in ["saturation", "limit_violation"]:
            if not isinstance(step.get(field), bool):
                errors.append(f"steps[{index}].{field} must be a boolean")
        for field in ["limit_state", "fault_state", "source_cycle_fault_class"]:
            if not _is_non_empty_string(step.get(field)):
                errors.append(f"steps[{index}].{field} must be a non-empty string")
    if isinstance(payload.get("summary"), dict):
        for field in ["saturation_count", "limit_violation_count", "fault_count"]:
            if not _is_non_negative_int(payload["summary"].get(field)):
                errors.append(f"summary.{field} must be a non-negative integer")
    return errors


def build_simulator_adapter_boundary(
    *,
    scenario: dict[str, Any],
    artifact_paths: dict[str, str],
) -> dict[str, Any]:
    scenario_errors = validate_control_comm_scenario(scenario)
    if scenario_errors:
        raise ValueError("; ".join(scenario_errors))
    adapters = [
        {
            "adapter": "gazebo",
            "runtime_status": "not_run",
            "runtime_dependency_required": False,
            "input_contracts": _simulator_adapter_input_contracts(artifact_paths),
            "output_contracts": ["simulator_step_trace.v1", "simulator_contact_trace.v1"],
            "launch_profile": None,
            "residual_risks": ["gazebo_runtime_not_run"],
        },
        {
            "adapter": "mujoco",
            "runtime_status": "not_run",
            "runtime_dependency_required": False,
            "input_contracts": _simulator_adapter_input_contracts(artifact_paths),
            "output_contracts": ["simulator_step_trace.v1", "simulator_contact_trace.v1"],
            "launch_profile": None,
            "residual_risks": ["mujoco_runtime_not_run"],
        },
        {
            "adapter": "isaac_sim",
            "runtime_status": "not_run",
            "runtime_dependency_required": False,
            "input_contracts": _simulator_adapter_input_contracts(artifact_paths),
            "output_contracts": ["simulator_step_trace.v1", "simulator_contact_trace.v1"],
            "launch_profile": None,
            "residual_risks": ["isaac_sim_runtime_not_run"],
        },
    ]
    return {
        "boundary_version": SIMULATOR_ADAPTER_BOUNDARY_VERSION,
        "scenario_id": scenario["scenario_id"],
        "status": "planned",
        "core_runtime_dependency": "none",
        "supported_adapters": list(SUPPORTED_SIMULATOR_ADAPTERS),
        "adapter_count": len(adapters),
        "adapters": adapters,
        "canonical_contracts": [
            CONTROL_MESSAGE_ENVELOPE_VERSION,
            ETHERCAT_MODEL_TRACE_VERSION,
            MOTOR_JOINT_RESPONSE_TRACE_VERSION,
        ],
        "runtime_dependency_required": False,
        "compatibility_claim": "adapter_contract_only",
        "residual_risks": [
            "gazebo_runtime_not_run",
            "mujoco_runtime_not_run",
            "isaac_sim_runtime_not_run",
        ],
    }


def validate_simulator_adapter_boundary(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["simulator adapter boundary must be an object"]

    errors: list[str] = []
    if payload.get("boundary_version") != SIMULATOR_ADAPTER_BOUNDARY_VERSION:
        errors.append(
            f"boundary_version must be {SIMULATOR_ADAPTER_BOUNDARY_VERSION!r}"
        )
    for field in ["scenario_id", "status", "core_runtime_dependency", "compatibility_claim"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("status") != "planned":
        errors.append("status must be 'planned'")
    if payload.get("core_runtime_dependency") != "none":
        errors.append("core_runtime_dependency must be 'none'")
    if payload.get("compatibility_claim") != "adapter_contract_only":
        errors.append("compatibility_claim must be 'adapter_contract_only'")
    if not isinstance(payload.get("runtime_dependency_required"), bool):
        errors.append("runtime_dependency_required must be a boolean")
    elif payload["runtime_dependency_required"]:
        errors.append("runtime_dependency_required must be false for boundary-only evidence")
    for field in ["supported_adapters", "canonical_contracts", "residual_risks"]:
        if not isinstance(payload.get(field), list):
            errors.append(f"{field} must be a list")
    if isinstance(payload.get("supported_adapters"), list):
        missing = set(SUPPORTED_SIMULATOR_ADAPTERS) - set(payload["supported_adapters"])
        if missing:
            errors.append(
                "supported_adapters must include " + ", ".join(sorted(missing))
            )
    if not _is_non_negative_int(payload.get("adapter_count")):
        errors.append("adapter_count must be a non-negative integer")
    adapters = payload.get("adapters")
    if not isinstance(adapters, list):
        errors.append("adapters must be a list")
        return errors
    if _is_non_negative_int(payload.get("adapter_count")) and int(
        payload["adapter_count"]
    ) != len(adapters):
        errors.append("adapter_count must equal len(adapters)")
    for index, adapter in enumerate(adapters):
        if not isinstance(adapter, dict):
            errors.append(f"adapters[{index}] must be an object")
            continue
        if adapter.get("adapter") not in SUPPORTED_SIMULATOR_ADAPTERS:
            errors.append(f"adapters[{index}].adapter must be a supported adapter")
        if adapter.get("runtime_status") != "not_run":
            errors.append(f"adapters[{index}].runtime_status must be 'not_run'")
        if adapter.get("runtime_dependency_required") is not False:
            errors.append(
                f"adapters[{index}].runtime_dependency_required must be false"
            )
        for field in ["input_contracts", "output_contracts", "residual_risks"]:
            if not isinstance(adapter.get(field), list):
                errors.append(f"adapters[{index}].{field} must be a list")
    return errors


def build_live_hardware_migration_gate(
    *,
    scenario: dict[str, Any],
    artifact_paths: dict[str, str],
) -> dict[str, Any]:
    scenario_errors = validate_control_comm_scenario(scenario)
    if scenario_errors:
        raise ValueError("; ".join(scenario_errors))
    transport_requirements = {
        "can": [
            "can_bus_capture",
            "hardware_closeout_report",
            "operator_confirmation",
        ],
        "ethercat": [
            "ethercat_master_capture",
            "fieldbus_watchdog_report",
            "hardware_closeout_report",
            "operator_confirmation",
        ],
        "tsn": [
            "tsn_schedule_capture",
            "clock_sync_report",
            "hardware_closeout_report",
            "operator_confirmation",
        ],
    }
    transport_gates = [
        {
            "transport": transport,
            "status": "blocked",
            "hardware_role_required": True,
            "operator_confirmation_required": True,
            "external_evidence_required": True,
            "simulation_substitute_allowed": False,
            "required_external_evidence": transport_requirements[transport],
            "residual_risks": [f"real_{transport}_transport_not_run"],
        }
        for transport in SUPPORTED_LIVE_HARDWARE_TRANSPORTS
    ]
    simulation_evidence_inputs = [
        {
            "artifact_key": artifact_key,
            "artifact_path": artifact_paths.get(artifact_key),
            "accepted_for_live": False,
        }
        for artifact_key in [
            "timing_trace",
            "message_trace",
            "zenoh_simulated_trace",
            "ethercat_model_trace",
            "motor_joint_response_trace",
            "simulator_adapter_boundary",
        ]
    ]
    return {
        "gate_version": LIVE_HARDWARE_MIGRATION_GATE_VERSION,
        "scenario_id": scenario["scenario_id"],
        "status": "blocked",
        "required_transports": list(SUPPORTED_LIVE_HARDWARE_TRANSPORTS),
        "transport_gate_count": len(transport_gates),
        "transport_gates": transport_gates,
        "hardware_role_required": True,
        "operator_confirmation_required": True,
        "external_evidence_required": True,
        "simulation_substitute_allowed": False,
        "simulation_evidence_inputs": simulation_evidence_inputs,
        "release_gate": {
            "status": "blocked",
            "reason": "external_live_hardware_evidence_missing",
        },
        "compatibility_claim": "migration_gate_only",
        "residual_risks": [
            "real_can_transport_not_run",
            "real_ethercat_transport_not_run",
            "real_tsn_transport_not_run",
        ],
    }


def validate_live_hardware_migration_gate(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["live hardware migration gate must be an object"]

    errors: list[str] = []
    if payload.get("gate_version") != LIVE_HARDWARE_MIGRATION_GATE_VERSION:
        errors.append(
            f"gate_version must be {LIVE_HARDWARE_MIGRATION_GATE_VERSION!r}"
        )
    for field in ["scenario_id", "status", "compatibility_claim"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("status") != "blocked":
        errors.append("status must be 'blocked'")
    if payload.get("compatibility_claim") != "migration_gate_only":
        errors.append("compatibility_claim must be 'migration_gate_only'")
    for field in [
        "hardware_role_required",
        "operator_confirmation_required",
        "external_evidence_required",
    ]:
        if payload.get(field) is not True:
            errors.append(f"{field} must be true")
    if payload.get("simulation_substitute_allowed") is not False:
        errors.append("simulation_substitute_allowed must be false")
    for field in [
        "required_transports",
        "transport_gates",
        "simulation_evidence_inputs",
        "residual_risks",
    ]:
        if not isinstance(payload.get(field), list):
            errors.append(f"{field} must be a list")
    if isinstance(payload.get("required_transports"), list):
        missing = set(SUPPORTED_LIVE_HARDWARE_TRANSPORTS) - set(
            payload["required_transports"]
        )
        if missing:
            errors.append(
                "required_transports must include " + ", ".join(sorted(missing))
            )
    if not _is_non_negative_int(payload.get("transport_gate_count")):
        errors.append("transport_gate_count must be a non-negative integer")
    gates = payload.get("transport_gates")
    if isinstance(gates, list):
        if _is_non_negative_int(payload.get("transport_gate_count")) and int(
            payload["transport_gate_count"]
        ) != len(gates):
            errors.append("transport_gate_count must equal len(transport_gates)")
        seen_transports: set[str] = set()
        for index, gate in enumerate(gates):
            if not isinstance(gate, dict):
                errors.append(f"transport_gates[{index}] must be an object")
                continue
            transport = gate.get("transport")
            if transport not in SUPPORTED_LIVE_HARDWARE_TRANSPORTS:
                errors.append(
                    f"transport_gates[{index}].transport must be a supported transport"
                )
            elif transport in seen_transports:
                errors.append(f"transport_gates[{index}].transport must be unique")
            elif isinstance(transport, str):
                seen_transports.add(transport)
            if gate.get("status") != "blocked":
                errors.append(f"transport_gates[{index}].status must be 'blocked'")
            for field in [
                "hardware_role_required",
                "operator_confirmation_required",
                "external_evidence_required",
            ]:
                if gate.get(field) is not True:
                    errors.append(f"transport_gates[{index}].{field} must be true")
            if gate.get("simulation_substitute_allowed") is not False:
                errors.append(
                    f"transport_gates[{index}].simulation_substitute_allowed must be false"
                )
            for field in ["required_external_evidence", "residual_risks"]:
                values = gate.get(field)
                if not isinstance(values, list) or not all(
                    _is_non_empty_string(value) for value in values
                ):
                    errors.append(
                        f"transport_gates[{index}].{field} must be a list of non-empty strings"
                    )
        missing_gate_transports = set(SUPPORTED_LIVE_HARDWARE_TRANSPORTS) - seen_transports
        if missing_gate_transports:
            errors.append(
                "transport_gates must include "
                + ", ".join(sorted(missing_gate_transports))
            )
    evidence_inputs = payload.get("simulation_evidence_inputs")
    if isinstance(evidence_inputs, list):
        for index, evidence in enumerate(evidence_inputs):
            if not isinstance(evidence, dict):
                errors.append(f"simulation_evidence_inputs[{index}] must be an object")
                continue
            if not _is_non_empty_string(evidence.get("artifact_key")):
                errors.append(
                    f"simulation_evidence_inputs[{index}].artifact_key must be a non-empty string"
                )
            artifact_path = evidence.get("artifact_path")
            if artifact_path is not None and not _is_non_empty_string(artifact_path):
                errors.append(
                    f"simulation_evidence_inputs[{index}].artifact_path must be null or a non-empty string"
                )
            if evidence.get("accepted_for_live") is not False:
                errors.append(
                    f"simulation_evidence_inputs[{index}].accepted_for_live must be false"
                )
    release_gate = payload.get("release_gate")
    if not isinstance(release_gate, dict):
        errors.append("release_gate must be an object")
    elif release_gate.get("status") != "blocked":
        errors.append("release_gate.status must be 'blocked'")
    return errors


def build_zenoh_openneuro_topic_mapping(
    scenario: dict[str, Any],
) -> dict[str, Any]:
    errors = validate_control_comm_scenario(scenario)
    if errors:
        raise ValueError("; ".join(errors))

    canonical_topic = str(scenario["topic"])
    keyexpr_suffix = _topic_to_zenoh_keyexpr_suffix(canonical_topic)
    return {
        "mapping_version": ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION,
        "transport_mode": TRANSPORT_MODE_ZENOH_SIMULATED,
        "scenario_id": scenario["scenario_id"],
        "canonical_topic": canonical_topic,
        "zenoh_keyexpr": f"agi_walker/{keyexpr_suffix}",
        "openneuro_like_topic": f"openneuro_like/{keyexpr_suffix}",
        "envelope_schema_version": CONTROL_MESSAGE_ENVELOPE_VERSION,
        "payload_type": scenario["payload_type"],
        "source": scenario["source"],
        "target": scenario["target"],
        "compatibility_claim": "simulation_only",
        "residual_risks": [
            "real_zenoh_session_not_run",
            "openneuro_compatibility_not_claimed",
        ],
    }


def validate_zenoh_openneuro_topic_mapping(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["zenoh/openneuro topic mapping must be an object"]

    errors: list[str] = []
    if payload.get("mapping_version") != ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION:
        errors.append(
            f"mapping_version must be {ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION!r}"
        )
    if payload.get("transport_mode") != TRANSPORT_MODE_ZENOH_SIMULATED:
        errors.append(f"transport_mode must be {TRANSPORT_MODE_ZENOH_SIMULATED!r}")
    for field in [
        "scenario_id",
        "canonical_topic",
        "zenoh_keyexpr",
        "openneuro_like_topic",
        "envelope_schema_version",
        "payload_type",
        "source",
        "target",
        "compatibility_claim",
    ]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("envelope_schema_version") != CONTROL_MESSAGE_ENVELOPE_VERSION:
        errors.append(
            f"envelope_schema_version must be {CONTROL_MESSAGE_ENVELOPE_VERSION!r}"
        )
    if payload.get("compatibility_claim") != "simulation_only":
        errors.append("compatibility_claim must be 'simulation_only'")
    if not isinstance(payload.get("residual_risks"), list):
        errors.append("residual_risks must be a list")
    return errors


def build_zenoh_simulated_trace(
    *,
    mapping: dict[str, Any],
    message_trace: list[dict[str, Any]],
) -> dict[str, Any]:
    mapping_errors = validate_zenoh_openneuro_topic_mapping(mapping)
    if mapping_errors:
        raise ValueError("; ".join(mapping_errors))

    events: list[dict[str, Any]] = []
    for index, message in enumerate(message_trace):
        if not isinstance(message, dict):
            raise ValueError(f"message_trace[{index}] must be an object")
        envelope = message.get("envelope")
        envelope_errors = validate_control_message_envelope(envelope)
        if envelope_errors:
            raise ValueError(
                "; ".join(
                    f"message_trace[{index}].envelope.{error}"
                    for error in envelope_errors
                )
            )
        events.append(
            {
                "event_index": index,
                "evidence_source": TRANSPORT_MODE_ZENOH_SIMULATED,
                "transport_mode": TRANSPORT_MODE_ZENOH_SIMULATED,
                "keyexpr": mapping["zenoh_keyexpr"],
                "openneuro_like_topic": mapping["openneuro_like_topic"],
                "canonical_topic": envelope["topic"],
                "sequence": envelope["sequence"],
                "timestamp_ns": envelope["timestamp_ns"],
                "delivery_timestamp_ns": message.get("delivery_timestamp_ns"),
                "delivery_outcome": message.get("delivery_outcome"),
                "duplicate": bool(message.get("duplicate", False)),
                "fault_injection": message.get("fault_injection", "none"),
                "envelope": envelope,
            }
        )

    return {
        "trace_version": "zenoh_simulated_trace.v1",
        "transport_mode": TRANSPORT_MODE_ZENOH_SIMULATED,
        "mapping_version": ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION,
        "scenario_id": mapping["scenario_id"],
        "event_count": len(events),
        "events": events,
        "compatibility_claim": "simulation_only",
        "residual_risks": [
            "real_zenoh_session_not_run",
            "openneuro_compatibility_not_claimed",
        ],
    }


def validate_zenoh_simulated_trace(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["zenoh simulated trace must be an object"]

    errors: list[str] = []
    if payload.get("trace_version") != "zenoh_simulated_trace.v1":
        errors.append("trace_version must be 'zenoh_simulated_trace.v1'")
    if payload.get("transport_mode") != TRANSPORT_MODE_ZENOH_SIMULATED:
        errors.append(f"transport_mode must be {TRANSPORT_MODE_ZENOH_SIMULATED!r}")
    if payload.get("mapping_version") != ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION:
        errors.append(
            f"mapping_version must be {ZENOH_OPENNEURO_TOPIC_MAPPING_VERSION!r}"
        )
    for field in ["scenario_id", "compatibility_claim"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    if payload.get("compatibility_claim") != "simulation_only":
        errors.append("compatibility_claim must be 'simulation_only'")
    if not _is_non_negative_int(payload.get("event_count")):
        errors.append("event_count must be a non-negative integer")
    if not isinstance(payload.get("residual_risks"), list):
        errors.append("residual_risks must be a list")
    events = payload.get("events")
    if not isinstance(events, list):
        errors.append("events must be a list")
        return errors
    if _is_non_negative_int(payload.get("event_count")) and int(
        payload["event_count"]
    ) != len(events):
        errors.append("event_count must equal len(events)")
    for index, event in enumerate(events):
        if not isinstance(event, dict):
            errors.append(f"events[{index}] must be an object")
            continue
        if not _is_non_negative_int(event.get("event_index")):
            errors.append(f"events[{index}].event_index must be a non-negative integer")
        for field in [
            "evidence_source",
            "transport_mode",
            "keyexpr",
            "openneuro_like_topic",
            "canonical_topic",
            "delivery_outcome",
            "fault_injection",
        ]:
            if not _is_non_empty_string(event.get(field)):
                errors.append(f"events[{index}].{field} must be a non-empty string")
        if event.get("transport_mode") != TRANSPORT_MODE_ZENOH_SIMULATED:
            errors.append(
                f"events[{index}].transport_mode must be "
                f"{TRANSPORT_MODE_ZENOH_SIMULATED!r}"
            )
        if not _is_non_negative_int(event.get("sequence")):
            errors.append(f"events[{index}].sequence must be a non-negative integer")
        if not _is_non_negative_int(event.get("timestamp_ns")):
            errors.append(
                f"events[{index}].timestamp_ns must be a non-negative integer"
            )
        if not isinstance(event.get("duplicate"), bool):
            errors.append(f"events[{index}].duplicate must be a boolean")
        envelope_errors = validate_control_message_envelope(event.get("envelope"))
        errors.extend(
            f"events[{index}].envelope.{error}" for error in envelope_errors
        )
    return errors


def run_local_asyncio_bus(
    *,
    envelopes: list[dict[str, Any]],
    bus_config: dict[str, Any] | None = None,
    jitter_budget_ns: int = 0,
) -> dict[str, Any]:
    return asyncio.run(
        simulate_local_asyncio_bus(
            envelopes=envelopes,
            bus_config=bus_config,
            jitter_budget_ns=jitter_budget_ns,
        )
    )


async def simulate_local_asyncio_bus(
    *,
    envelopes: list[dict[str, Any]],
    bus_config: dict[str, Any] | None = None,
    jitter_budget_ns: int = 0,
) -> dict[str, Any]:
    config = normalize_local_asyncio_bus_config(bus_config)
    dropped = set(config["drop_sequences"])
    duplicated = set(config["duplicate_sequences"])
    jitter_by_cycle = config["jitter_by_cycle_ns"]

    timing_trace: list[dict[str, Any]] = []
    message_trace: list[dict[str, Any]] = []
    delivery_events: list[dict[str, Any]] = []
    for envelope in envelopes:
        sequence = int(envelope["sequence"])
        expected_timestamp_ns = int(envelope["timestamp_ns"])
        jitter_ns = (
            int(jitter_by_cycle[sequence]) if sequence < len(jitter_by_cycle) else 0
        )
        actual_timestamp_ns = expected_timestamp_ns + config["latency_ns"] + jitter_ns
        deadline_missed = actual_timestamp_ns - expected_timestamp_ns > jitter_budget_ns
        is_dropped = sequence in dropped
        is_duplicated = sequence in duplicated and not is_dropped
        outcome = "dropped" if is_dropped else "delivered"
        fault_injection = _fault_injection_label(
            dropped=is_dropped,
            duplicated=is_duplicated,
            jitter_ns=jitter_ns,
            latency_ns=config["latency_ns"],
        )
        timing_trace.append(
            {
                "cycle_index": sequence,
                "expected_timestamp_ns": expected_timestamp_ns,
                "actual_timestamp_ns": actual_timestamp_ns,
                "latency_ns": config["latency_ns"],
                "jitter_ns": actual_timestamp_ns - expected_timestamp_ns,
                "deadline_missed": deadline_missed,
                "task_outcome": outcome,
            }
        )
        message_trace.append(
            {
                "cycle_index": sequence,
                "envelope": envelope,
                "delivery_outcome": outcome,
                "fault_injection": fault_injection,
                "delivery_timestamp_ns": None if is_dropped else actual_timestamp_ns,
                "duplicate": False,
            }
        )
        if not is_dropped:
            delivery_events.append(
                {
                    "sequence": sequence,
                    "delivery_timestamp_ns": actual_timestamp_ns,
                    "duplicate": False,
                }
            )
        if is_duplicated:
            duplicate_timestamp_ns = actual_timestamp_ns + 1
            message_trace.append(
                {
                    "cycle_index": sequence,
                    "envelope": envelope,
                    "delivery_outcome": "duplicated",
                    "fault_injection": "duplicate",
                    "delivery_timestamp_ns": duplicate_timestamp_ns,
                    "duplicate": True,
                }
            )
            delivery_events.append(
                {
                    "sequence": sequence,
                    "delivery_timestamp_ns": duplicate_timestamp_ns,
                    "duplicate": True,
                }
            )

    await asyncio.sleep(0)
    delivery_events.sort(
        key=lambda item: (int(item["delivery_timestamp_ns"]), int(item["sequence"]))
    )
    delivered_sequences = [int(item["sequence"]) for item in delivery_events]
    dropped_count = sum(1 for entry in message_trace if entry["delivery_outcome"] == "dropped")
    duplicated_count = sum(
        1 for entry in message_trace if entry["delivery_outcome"] == "duplicated"
    )
    deadline_miss_count = sum(1 for entry in timing_trace if entry["deadline_missed"])
    max_abs_jitter_ns = max(
        (abs(int(entry["jitter_ns"])) for entry in timing_trace),
        default=0,
    )
    return {
        "timing_trace": timing_trace,
        "message_trace": message_trace,
        "bus_summary": {
            "mode": LOCAL_ASYNCIO_BUS_MODE,
            "latency_ns": config["latency_ns"],
            "jitter_by_cycle_ns": jitter_by_cycle,
            "drop_sequences": config["drop_sequences"],
            "duplicate_sequences": config["duplicate_sequences"],
            "delivery_order": delivered_sequences,
        },
        "message_integrity": {
            "messages_sent": len(envelopes),
            "messages_delivered": len(delivery_events),
            "messages_dropped": dropped_count,
            "messages_duplicated": duplicated_count,
            "sequence_gap_count": dropped_count,
        },
        "deadline_miss_count": deadline_miss_count,
        "max_abs_jitter_ns": max_abs_jitter_ns,
    }


def normalize_local_asyncio_bus_config(payload: dict[str, Any] | None) -> dict[str, Any]:
    config = {
        "mode": LOCAL_ASYNCIO_BUS_MODE,
        "latency_ns": 0,
        "jitter_by_cycle_ns": [],
        "drop_sequences": [],
        "duplicate_sequences": [],
    }
    if payload:
        config.update(payload)
    errors = validate_local_asyncio_bus_config(config)
    if errors:
        raise ValueError("; ".join(errors))
    return config


def build_godot_log_contract_preview(
    *,
    scenario: dict[str, Any],
    envelopes: list[dict[str, Any]],
) -> dict[str, Any]:
    events = [
        {
            "cycle_index": index,
            "event_type": "message_delivered",
            "evidence_source": EVIDENCE_SOURCE_GODOT_SCRIPT,
            "envelope": envelope,
            "godot_node_path": None,
            "script_source": "res://scripts/control_comm_replay.gd",
        }
        for index, envelope in enumerate(envelopes)
    ]
    return {
        "log_version": GODOT_CONTROL_COMM_SIMULATION_LOG_VERSION,
        "status": GODOT_LOG_NOT_RUN_STATUS,
        "evidence_source": EVIDENCE_SOURCE_GODOT_SCRIPT,
        "script_name": "control_comm_replay.gd",
        "godot_profile": {
            "mode": "not_run",
            "executable": None,
            "project": "godot_project",
        },
        "scenario_id": scenario["scenario_id"],
        "cycle_count": len(events),
        "message_event_count": len(events),
        "events": events,
        "artifact_paths": {},
        "residual_risks": ["godot_executable_not_run"],
        "errors": [],
    }


def validate_godot_control_comm_simulation_log(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["godot control communication simulation log must be an object"]

    errors: list[str] = []
    if payload.get("log_version") != GODOT_CONTROL_COMM_SIMULATION_LOG_VERSION:
        errors.append(
            f"log_version must be {GODOT_CONTROL_COMM_SIMULATION_LOG_VERSION!r}"
        )
    if payload.get("evidence_source") not in {
        EVIDENCE_SOURCE_GODOT_SCRIPT,
        EVIDENCE_SOURCE_GODOT_HEADLESS,
    }:
        errors.append("evidence_source must be 'godot_script' or 'godot_headless'")
    for field in ["status", "script_name", "scenario_id"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    for field in ["cycle_count", "message_event_count"]:
        if not _is_non_negative_int(payload.get(field)):
            errors.append(f"{field} must be a non-negative integer")
    if not isinstance(payload.get("godot_profile"), dict):
        errors.append("godot_profile must be an object")
    if not isinstance(payload.get("artifact_paths"), dict):
        errors.append("artifact_paths must be an object")
    if not isinstance(payload.get("residual_risks"), list):
        errors.append("residual_risks must be a list")
    if not isinstance(payload.get("errors"), list):
        errors.append("errors must be a list")
    events = payload.get("events")
    if not isinstance(events, list):
        errors.append("events must be a list")
        return errors
    if _is_non_negative_int(payload.get("message_event_count")) and int(
        payload["message_event_count"]
    ) != len(events):
        errors.append("message_event_count must equal len(events)")
    for index, event in enumerate(events):
        if not isinstance(event, dict):
            errors.append(f"events[{index}] must be an object")
            continue
        if not _is_non_negative_int(event.get("cycle_index")):
            errors.append(f"events[{index}].cycle_index must be a non-negative integer")
        if not _is_non_empty_string(event.get("event_type")):
            errors.append(f"events[{index}].event_type must be a non-empty string")
        if event.get("evidence_source") not in {
            EVIDENCE_SOURCE_GODOT_SCRIPT,
            EVIDENCE_SOURCE_GODOT_HEADLESS,
        }:
            errors.append(
                f"events[{index}].evidence_source must be 'godot_script' "
                "or 'godot_headless'"
            )
        envelope_errors = validate_control_message_envelope(event.get("envelope"))
        errors.extend(
            f"events[{index}].envelope.{error}" for error in envelope_errors
        )
    return errors


def validate_control_comm_simulation_report(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["control communication simulation report must be an object"]

    errors: list[str] = []
    if payload.get("report_version") != CONTROL_COMM_SIMULATION_REPORT_VERSION:
        errors.append(
            f"report_version must be {CONTROL_COMM_SIMULATION_REPORT_VERSION!r}"
        )
    for field in ["status", "scenario_id", "transport_mode", "clock_mode"]:
        if not _is_non_empty_string(payload.get(field)):
            errors.append(f"{field} must be a non-empty string")
    for field in ["cycle_period_ns", "cycle_count", "jitter_budget_ns"]:
        if not _is_non_negative_int(payload.get(field)):
            errors.append(f"{field} must be a non-negative integer")
    if not isinstance(payload.get("evidence_sources"), list):
        errors.append("evidence_sources must be a list")
    if not isinstance(payload.get("simulated_transport_modes"), list):
        errors.append("simulated_transport_modes must be a list")
    if not isinstance(payload.get("timing_metrics"), dict):
        errors.append("timing_metrics must be an object")
    if not isinstance(payload.get("message_integrity"), dict):
        errors.append("message_integrity must be an object")
    if not isinstance(payload.get("local_asyncio_bus"), dict):
        errors.append("local_asyncio_bus must be an object")
    if not isinstance(payload.get("artifact_paths"), dict):
        errors.append("artifact_paths must be an object")
    if not isinstance(payload.get("godot_log_evidence"), dict):
        errors.append("godot_log_evidence must be an object")
    if not isinstance(payload.get("zenoh_openneuro_simulation"), dict):
        errors.append("zenoh_openneuro_simulation must be an object")
    if not isinstance(payload.get("ethercat_cycle_model"), dict):
        errors.append("ethercat_cycle_model must be an object")
    if not isinstance(payload.get("motor_joint_model"), dict):
        errors.append("motor_joint_model must be an object")
    if not isinstance(payload.get("simulator_adapter_boundary"), dict):
        errors.append("simulator_adapter_boundary must be an object")
    if not isinstance(payload.get("live_hardware_migration_gate"), dict):
        errors.append("live_hardware_migration_gate must be an object")
    if not isinstance(payload.get("residual_risks"), list):
        errors.append("residual_risks must be a list")
    if not isinstance(payload.get("errors"), list):
        errors.append("errors must be a list")
    return errors


def _write_json(path: Path, payload: Any) -> str:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return str(path)


def _require_positive_int(value: int, field: str) -> None:
    if not _is_positive_int(value):
        raise ValueError(f"{field} must be a positive integer")


def _fault_injection_label(
    *,
    dropped: bool,
    duplicated: bool,
    jitter_ns: int,
    latency_ns: int,
) -> str:
    labels: list[str] = []
    if dropped:
        labels.append("drop")
    if duplicated:
        labels.append("duplicate")
    if jitter_ns:
        labels.append("jitter")
    if latency_ns:
        labels.append("latency")
    return "+".join(labels) if labels else "none"


def _topic_to_zenoh_keyexpr_suffix(topic: str) -> str:
    segments = [
        "".join(
            character
            if character.isalnum() or character in {"_", "-"}
            else "_"
            for character in segment.strip()
        )
        for segment in topic.strip("/").split("/")
        if segment.strip()
    ]
    return "/".join(segments) if segments else "unnamed"


def _simulated_transport_modes(
    *,
    include_zenoh_openneuro_simulation: bool,
    include_ethercat_cycle_model: bool,
    include_motor_joint_model: bool,
) -> list[str]:
    modes = [TRANSPORT_MODE_ASYNCIO]
    if include_zenoh_openneuro_simulation:
        modes.append(TRANSPORT_MODE_ZENOH_SIMULATED)
    if include_ethercat_cycle_model:
        modes.append(TRANSPORT_MODE_ETHERCAT_MODEL)
    if include_motor_joint_model:
        modes.append(MOTOR_JOINT_MODEL_MODE)
    return modes


def _simulator_adapter_input_contracts(
    artifact_paths: dict[str, str],
) -> list[dict[str, Any]]:
    return [
        {
            "contract": "timing_trace.v1",
            "artifact_key": "timing_trace",
            "artifact_path": artifact_paths.get("timing_trace"),
            "required": True,
        },
        {
            "contract": "message_trace.v1",
            "artifact_key": "message_trace",
            "artifact_path": artifact_paths.get("message_trace"),
            "required": True,
        },
        {
            "contract": ETHERCAT_MODEL_TRACE_VERSION,
            "artifact_key": "ethercat_model_trace",
            "artifact_path": artifact_paths.get("ethercat_model_trace"),
            "required": True,
        },
        {
            "contract": MOTOR_JOINT_RESPONSE_TRACE_VERSION,
            "artifact_key": "motor_joint_response_trace",
            "artifact_path": artifact_paths.get("motor_joint_response_trace"),
            "required": True,
        },
    ]


def _first_message_by_cycle(
    message_trace: list[dict[str, Any]],
) -> dict[int, dict[str, Any]]:
    messages_by_cycle: dict[int, dict[str, Any]] = {}
    for message in message_trace:
        if not isinstance(message, dict):
            continue
        cycle_index = message.get("cycle_index")
        if _is_non_negative_int(cycle_index) and int(cycle_index) not in messages_by_cycle:
            messages_by_cycle[int(cycle_index)] = message
    return messages_by_cycle


def _build_pdo_inputs(
    envelope: dict[str, Any],
    pdo_input_map: list[str],
) -> dict[str, Any]:
    payload = envelope.get("payload", {})
    return {
        name: payload.get(name)
        for name in pdo_input_map
    }


def _build_pdo_outputs(
    *,
    pdo_inputs: dict[str, Any],
    pdo_output_map: list[str],
    watchdog_tripped: bool,
    frame_missing: bool,
) -> dict[str, Any]:
    if watchdog_tripped or frame_missing:
        return {name: 0 for name in pdo_output_map}
    output_values = {
        "position_rad": 0.0,
        "velocity_rad_s": pdo_inputs.get("target_velocity_rad_s", 0),
        "torque_nm": pdo_inputs.get("target_torque_nm", 0),
    }
    return {name: output_values.get(name, 0) for name in pdo_output_map}


def _ethercat_fault_class(
    *,
    deadline_missed: bool,
    frame_missing: bool,
    watchdog_tripped: bool,
) -> str:
    if watchdog_tripped:
        return "watchdog_tripped"
    if frame_missing:
        return "pdo_frame_missing"
    if deadline_missed:
        return "cycle_deadline_miss"
    return "none"


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _apply_friction(commanded_torque: float, friction_nm: float) -> float:
    if commanded_torque > 0:
        return max(0.0, commanded_torque - friction_nm)
    if commanded_torque < 0:
        return min(0.0, commanded_torque + friction_nm)
    return 0.0


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _is_non_negative_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _is_positive_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)
