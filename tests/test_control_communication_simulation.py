import importlib.util
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "agi_walker/core/simulation/control_comm_simulation.py"
TOOL_PATH = ROOT / "tools/run_control_comm_simulation.py"
CLOSEOUT_TOOL_PATH = ROOT / "tools/build_control_comm_simulation_closeout.py"

spec = importlib.util.spec_from_file_location("control_comm_simulation", MODULE_PATH)
control_comm_simulation = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(control_comm_simulation)

closeout_spec = importlib.util.spec_from_file_location(
    "build_control_comm_simulation_closeout", CLOSEOUT_TOOL_PATH
)
control_comm_closeout = importlib.util.module_from_spec(closeout_spec)
assert closeout_spec.loader is not None
closeout_spec.loader.exec_module(control_comm_closeout)


def test_canonical_envelope_contract_accepts_valid_payload() -> None:
    envelope = control_comm_simulation.build_control_message_envelope(
        topic="agi/control/joint/left_hip/command",
        sequence=1,
        timestamp_ns=10_000_000,
        source="controller",
        target="joint_endpoint.left_hip",
        payload_type="joint_velocity_command",
        payload={"joint_name": "left_hip", "target_velocity_rad_s": 0.25},
        metadata={"scenario_id": "demo"},
    )

    assert envelope["schema_version"] == "control_message_envelope.v1"
    assert control_comm_simulation.validate_control_message_envelope(envelope) == []


def test_canonical_envelope_contract_rejects_malformed_payload() -> None:
    errors = control_comm_simulation.validate_control_message_envelope(
        {
            "schema_version": "wrong",
            "topic": "",
            "sequence": -1,
            "timestamp_ns": "now",
            "source": "controller",
            "target": "",
            "payload_type": "joint_velocity_command",
            "payload": [],
            "metadata": [],
        }
    )

    assert "schema_version must be 'control_message_envelope.v1'" in errors
    assert "topic must be a non-empty string" in errors
    assert "sequence must be a non-negative integer" in errors
    assert "timestamp_ns must be a non-negative integer" in errors
    assert "payload must be an object" in errors


def test_deterministic_simulation_writes_report_and_trace_artifacts(
    tmp_path: Path,
) -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario(
        cycle_count=3,
        cycle_period_ns=5_000_000,
    )
    report = control_comm_simulation.run_deterministic_control_comm_simulation(
        scenario,
        output_root=tmp_path,
    )

    assert report["report_version"] == "control_comm_simulation_report.v1"
    assert report["status"] == "success"
    assert report["clock_mode"] == "virtual"
    assert report["message_integrity"]["messages_delivered"] == 3
    assert report["timing_metrics"]["deadline_miss_count"] == 0
    assert report["zenoh_openneuro_simulation"]["status"] == "simulated"
    assert report["ethercat_cycle_model"]["status"] == "simulated"
    assert report["motor_joint_model"]["status"] == "simulated"
    assert report["simulator_adapter_boundary"]["status"] == "planned"
    assert report["live_hardware_migration_gate"]["status"] == "blocked"
    assert (
        report["live_hardware_migration_gate"]["simulation_substitute_allowed"]
        is False
    )
    assert control_comm_simulation.validate_control_comm_simulation_report(report) == []

    timing_trace = json.loads(
        Path(report["artifact_paths"]["timing_trace"]).read_text(encoding="utf-8")
    )
    message_trace = json.loads(
        Path(report["artifact_paths"]["message_trace"]).read_text(encoding="utf-8")
    )
    godot_log = json.loads(
        Path(report["artifact_paths"]["godot_log_contract_preview"]).read_text(
            encoding="utf-8"
        )
    )
    zenoh_mapping = json.loads(
        Path(report["artifact_paths"]["zenoh_openneuro_topic_mapping"]).read_text(
            encoding="utf-8"
        )
    )
    zenoh_trace = json.loads(
        Path(report["artifact_paths"]["zenoh_simulated_trace"]).read_text(
            encoding="utf-8"
        )
    )
    ethercat_trace = json.loads(
        Path(report["artifact_paths"]["ethercat_model_trace"]).read_text(
            encoding="utf-8"
        )
    )
    motor_joint_trace = json.loads(
        Path(report["artifact_paths"]["motor_joint_response_trace"]).read_text(
            encoding="utf-8"
        )
    )
    simulator_adapter_boundary = json.loads(
        Path(report["artifact_paths"]["simulator_adapter_boundary"]).read_text(
            encoding="utf-8"
        )
    )
    live_hardware_migration_gate = json.loads(
        Path(report["artifact_paths"]["live_hardware_migration_gate"]).read_text(
            encoding="utf-8"
        )
    )

    assert [entry["actual_timestamp_ns"] for entry in timing_trace] == [
        0,
        5_000_000,
        10_000_000,
    ]
    assert [entry["envelope"]["sequence"] for entry in message_trace] == [0, 1, 2]
    assert (
        control_comm_simulation.validate_godot_control_comm_simulation_log(godot_log)
        == []
    )
    assert godot_log["status"] == "not_run"
    assert godot_log["message_event_count"] == 3
    assert (
        control_comm_simulation.validate_zenoh_openneuro_topic_mapping(zenoh_mapping)
        == []
    )
    assert control_comm_simulation.validate_zenoh_simulated_trace(zenoh_trace) == []
    assert zenoh_mapping["compatibility_claim"] == "simulation_only"
    assert zenoh_trace["event_count"] == 3
    assert control_comm_simulation.validate_ethercat_model_trace(ethercat_trace) == []
    assert ethercat_trace["summary"]["deadline_miss_count"] == 0
    assert ethercat_trace["summary"]["watchdog_trip_count"] == 0
    assert (
        control_comm_simulation.validate_motor_joint_response_trace(motor_joint_trace)
        == []
    )
    assert motor_joint_trace["summary"] == {
        "saturation_count": 0,
        "limit_violation_count": 0,
        "fault_count": 0,
    }
    assert (
        control_comm_simulation.validate_simulator_adapter_boundary(
            simulator_adapter_boundary
        )
        == []
    )
    assert simulator_adapter_boundary["supported_adapters"] == [
        "gazebo",
        "mujoco",
        "isaac_sim",
    ]
    assert simulator_adapter_boundary["runtime_dependency_required"] is False
    assert (
        control_comm_simulation.validate_live_hardware_migration_gate(
            live_hardware_migration_gate
        )
        == []
    )
    assert live_hardware_migration_gate["required_transports"] == [
        "can",
        "ethercat",
        "tsn",
    ]
    assert all(
        gate["status"] == "blocked"
        and gate["simulation_substitute_allowed"] is False
        for gate in live_hardware_migration_gate["transport_gates"]
    )


def test_local_asyncio_bus_models_latency_jitter_drop_and_duplicate(
    tmp_path: Path,
) -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario(
        cycle_count=4,
        cycle_period_ns=100,
    )
    scenario["jitter_budget_ns"] = 5
    scenario["bus"] = {
        "mode": "local_asyncio_virtual",
        "latency_ns": 3,
        "jitter_by_cycle_ns": [0, 10, -2, 0],
        "drop_sequences": [2],
        "duplicate_sequences": [1],
    }
    scenario["ethercat"] = {
        "mode": "ethercat_cycle_model",
        "watchdog_timeout_cycles": 3,
        "deadline_budget_ns": 5,
        "pdo_input_map": ["target_velocity_rad_s", "target_torque_nm"],
        "pdo_output_map": ["position_rad", "velocity_rad_s", "torque_nm"],
    }

    report = control_comm_simulation.run_deterministic_control_comm_simulation(
        scenario,
        output_root=tmp_path,
    )
    timing_trace = json.loads(
        Path(report["artifact_paths"]["timing_trace"]).read_text(encoding="utf-8")
    )
    message_trace = json.loads(
        Path(report["artifact_paths"]["message_trace"]).read_text(encoding="utf-8")
    )

    assert report["local_asyncio_bus"]["delivery_order"] == [0, 1, 1, 3]
    assert report["timing_metrics"]["max_abs_jitter_ns"] == 13
    assert report["timing_metrics"]["deadline_miss_count"] == 1
    assert report["message_integrity"] == {
        "messages_sent": 4,
        "messages_delivered": 4,
        "messages_dropped": 1,
        "messages_duplicated": 1,
        "sequence_gap_count": 1,
    }
    assert timing_trace[1]["deadline_missed"] is True
    assert timing_trace[2]["task_outcome"] == "dropped"
    assert [entry["delivery_outcome"] for entry in message_trace] == [
        "delivered",
        "delivered",
        "duplicated",
        "dropped",
        "delivered",
    ]

    zenoh_trace = json.loads(
        Path(report["artifact_paths"]["zenoh_simulated_trace"]).read_text(
            encoding="utf-8"
        )
    )
    assert [entry["delivery_outcome"] for entry in zenoh_trace["events"]] == [
        "delivered",
        "delivered",
        "duplicated",
        "dropped",
        "delivered",
    ]
    assert zenoh_trace["events"][2]["duplicate"] is True
    assert zenoh_trace["events"][3]["delivery_timestamp_ns"] is None

    ethercat_trace = json.loads(
        Path(report["artifact_paths"]["ethercat_model_trace"]).read_text(
            encoding="utf-8"
        )
    )
    assert ethercat_trace["summary"] == {
        "deadline_miss_count": 1,
        "frame_missing_count": 1,
        "watchdog_trip_count": 0,
    }
    assert ethercat_trace["cycles"][1]["fault_class"] == "cycle_deadline_miss"
    assert ethercat_trace["cycles"][2]["fault_class"] == "pdo_frame_missing"
    assert ethercat_trace["cycles"][2]["pdo_outputs"] == {
        "position_rad": 0,
        "velocity_rad_s": 0,
        "torque_nm": 0,
    }

    motor_joint_trace = json.loads(
        Path(report["artifact_paths"]["motor_joint_response_trace"]).read_text(
            encoding="utf-8"
        )
    )
    assert motor_joint_trace["summary"]["fault_count"] == 2
    assert motor_joint_trace["steps"][1]["fault_state"] == (
        "fieldbus_cycle_deadline_miss"
    )
    assert motor_joint_trace["steps"][2]["fault_state"] == "fieldbus_pdo_frame_missing"


def test_local_asyncio_bus_config_rejects_invalid_fault_lists() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario()
    scenario["bus"] = {
        "mode": "local_asyncio_virtual",
        "latency_ns": -1,
        "jitter_by_cycle_ns": ["fast"],
        "drop_sequences": [-1],
        "duplicate_sequences": ["one"],
    }

    errors = control_comm_simulation.validate_control_comm_scenario(scenario)

    assert "bus.latency_ns must be a non-negative integer" in errors
    assert "bus.jitter_by_cycle_ns must be a list of integers" in errors
    assert "bus.drop_sequences must be a list of non-negative integers" in errors
    assert "bus.duplicate_sequences must be a list of non-negative integers" in errors


def test_ethercat_model_config_rejects_invalid_watchdog_and_pdo_maps() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario()
    scenario["ethercat"] = {
        "mode": "live_ethercat",
        "watchdog_timeout_cycles": 0,
        "deadline_budget_ns": -1,
        "pdo_input_map": ["target_velocity_rad_s", ""],
        "pdo_output_map": "velocity",
    }

    errors = control_comm_simulation.validate_control_comm_scenario(scenario)

    assert "ethercat.mode must be 'ethercat_cycle_model'" in errors
    assert "ethercat.watchdog_timeout_cycles must be a positive integer" in errors
    assert "ethercat.deadline_budget_ns must be a non-negative integer" in errors
    assert "ethercat.pdo_input_map must be a list of non-empty strings" in errors
    assert "ethercat.pdo_output_map must be a list of non-empty strings" in errors


def test_motor_joint_model_config_rejects_invalid_limits() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario()
    scenario["motor_joint"] = {
        "mode": "live_motor",
        "joint_name": "",
        "position_lower_rad": 1.0,
        "position_upper_rad": -1.0,
        "velocity_limit_rad_s": 0,
        "torque_limit_nm": -1,
        "friction_nm": -0.1,
        "backlash_rad": -0.1,
    }

    errors = control_comm_simulation.validate_control_comm_scenario(scenario)

    assert "motor_joint.mode must be 'motor_joint_model'" in errors
    assert "motor_joint.joint_name must be a non-empty string" in errors
    assert (
        "motor_joint.position_lower_rad must be less than position_upper_rad"
        in errors
    )
    assert "motor_joint.velocity_limit_rad_s must be greater than zero" in errors
    assert "motor_joint.torque_limit_nm must be greater than zero" in errors
    assert "motor_joint.friction_nm must be non-negative" in errors
    assert "motor_joint.backlash_rad must be non-negative" in errors


def test_godot_log_contract_rejects_invalid_event_count() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario(
        cycle_count=1
    )
    envelope = control_comm_simulation.build_control_message_envelope(
        topic=scenario["topic"],
        sequence=0,
        timestamp_ns=0,
        source=scenario["source"],
        target=scenario["target"],
        payload_type=scenario["payload_type"],
        payload=scenario["command"],
    )
    log = control_comm_simulation.build_godot_log_contract_preview(
        scenario=scenario,
        envelopes=[envelope],
    )
    log["message_event_count"] = 2

    errors = control_comm_simulation.validate_godot_control_comm_simulation_log(log)

    assert "message_event_count must equal len(events)" in errors


def test_zenoh_openneuro_mapping_preserves_canonical_envelope_boundary() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario()

    mapping = control_comm_simulation.build_zenoh_openneuro_topic_mapping(scenario)

    assert mapping["mapping_version"] == "zenoh_openneuro_topic_mapping.v1"
    assert mapping["transport_mode"] == "zenoh_simulated"
    assert mapping["canonical_topic"] == scenario["topic"]
    assert mapping["zenoh_keyexpr"] == "agi_walker/agi/control/joint/left_hip/command"
    assert (
        mapping["openneuro_like_topic"]
        == "openneuro_like/agi/control/joint/left_hip/command"
    )
    assert mapping["compatibility_claim"] == "simulation_only"
    assert "real_zenoh_session_not_run" in mapping["residual_risks"]
    assert control_comm_simulation.validate_zenoh_openneuro_topic_mapping(mapping) == []


def test_zenoh_openneuro_mapping_rejects_live_compatibility_claim() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario()
    mapping = control_comm_simulation.build_zenoh_openneuro_topic_mapping(scenario)
    mapping["compatibility_claim"] = "openneuro_compatible"

    errors = control_comm_simulation.validate_zenoh_openneuro_topic_mapping(mapping)

    assert "compatibility_claim must be 'simulation_only'" in errors


def test_ethercat_model_watchdog_trips_after_consecutive_faults(
    tmp_path: Path,
) -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario(
        cycle_count=3,
        cycle_period_ns=100,
    )
    scenario["bus"] = {
        "mode": "local_asyncio_virtual",
        "latency_ns": 0,
        "jitter_by_cycle_ns": [0, 10, 10],
        "drop_sequences": [],
        "duplicate_sequences": [],
    }
    scenario["ethercat"] = {
        "mode": "ethercat_cycle_model",
        "watchdog_timeout_cycles": 2,
        "deadline_budget_ns": 5,
        "pdo_input_map": ["target_velocity_rad_s", "target_torque_nm"],
        "pdo_output_map": ["position_rad", "velocity_rad_s", "torque_nm"],
    }

    report = control_comm_simulation.run_deterministic_control_comm_simulation(
        scenario,
        output_root=tmp_path,
    )
    ethercat_trace = json.loads(
        Path(report["artifact_paths"]["ethercat_model_trace"]).read_text(
            encoding="utf-8"
        )
    )

    assert ethercat_trace["summary"]["deadline_miss_count"] == 2
    assert ethercat_trace["summary"]["watchdog_trip_count"] == 1
    assert ethercat_trace["cycles"][2]["watchdog_state"] == "tripped"
    assert ethercat_trace["cycles"][2]["fault_class"] == "watchdog_tripped"
    assert control_comm_simulation.validate_ethercat_model_trace(ethercat_trace) == []


def test_motor_joint_model_saturates_and_clamps_position(
    tmp_path: Path,
) -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario(
        cycle_count=3,
        cycle_period_ns=1_000_000_000,
    )
    scenario["command"] = {
        "joint_name": "left_hip",
        "target_velocity_rad_s": 2.0,
        "target_torque_nm": 5.0,
    }
    scenario["motor_joint"] = {
        "mode": "motor_joint_model",
        "joint_name": "left_hip",
        "position_lower_rad": -0.5,
        "position_upper_rad": 0.5,
        "velocity_limit_rad_s": 0.25,
        "torque_limit_nm": 1.0,
        "friction_nm": 0.1,
        "backlash_rad": 0.01,
    }

    report = control_comm_simulation.run_deterministic_control_comm_simulation(
        scenario,
        output_root=tmp_path,
    )
    motor_joint_trace = json.loads(
        Path(report["artifact_paths"]["motor_joint_response_trace"]).read_text(
            encoding="utf-8"
        )
    )

    assert motor_joint_trace["summary"]["saturation_count"] == 3
    assert motor_joint_trace["summary"]["limit_violation_count"] == 1
    assert motor_joint_trace["steps"][0]["velocity_rad_s"] == 0.25
    assert motor_joint_trace["steps"][0]["torque_nm"] == 1.0
    assert motor_joint_trace["steps"][2]["position_rad"] == 0.5
    assert motor_joint_trace["steps"][2]["limit_state"] == "upper_limit_clamped"
    assert (
        control_comm_simulation.validate_motor_joint_response_trace(motor_joint_trace)
        == []
    )


def test_simulator_adapter_boundary_rejects_runtime_dependency_claim() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario()
    boundary = control_comm_simulation.build_simulator_adapter_boundary(
        scenario=scenario,
        artifact_paths={
            "timing_trace": "timing_trace.json",
            "message_trace": "message_trace.json",
            "ethercat_model_trace": "ethercat_model_trace.json",
            "motor_joint_response_trace": "motor_joint_response_trace.json",
        },
    )
    boundary["runtime_dependency_required"] = True
    boundary["adapters"][0]["runtime_status"] = "passed"

    errors = control_comm_simulation.validate_simulator_adapter_boundary(boundary)

    assert (
        "runtime_dependency_required must be false for boundary-only evidence"
        in errors
    )
    assert "adapters[0].runtime_status must be 'not_run'" in errors


def test_live_hardware_migration_gate_rejects_unproven_live_claim() -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario()
    gate = control_comm_simulation.build_live_hardware_migration_gate(
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
    gate["status"] = "passed"
    gate["simulation_substitute_allowed"] = True
    gate["transport_gates"][0]["status"] = "passed"
    gate["transport_gates"][0]["simulation_substitute_allowed"] = True
    gate["release_gate"]["status"] = "passed"

    errors = control_comm_simulation.validate_live_hardware_migration_gate(gate)

    assert "status must be 'blocked'" in errors
    assert "simulation_substitute_allowed must be false" in errors
    assert "transport_gates[0].status must be 'blocked'" in errors
    assert (
        "transport_gates[0].simulation_substitute_allowed must be false"
        in errors
    )
    assert "release_gate.status must be 'blocked'" in errors


def test_cli_writes_self_validated_artifacts(tmp_path: Path) -> None:
    result = subprocess.run(
        [
            sys.executable,
            str(TOOL_PATH),
            "--cycles",
            "2",
            "--cycle-period-ns",
            "100",
            "--jitter-budget-ns",
            "5",
            "--bus-latency-ns",
            "3",
            "--bus-jitter-by-cycle-ns",
            "[0, 10]",
            "--bus-duplicate-sequences",
            "[1]",
            "--output-root",
            str(tmp_path),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    report = json.loads(
        (tmp_path / "control_comm_simulation_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert report["validation"]["control_comm_simulation_report_errors"] == []
    assert report["validation"]["godot_control_comm_simulation_log_errors"] == []
    assert report["validation"]["zenoh_openneuro_topic_mapping_errors"] == []
    assert report["validation"]["zenoh_simulated_trace_errors"] == []
    assert report["validation"]["ethercat_model_trace_errors"] == []
    assert report["validation"]["motor_joint_response_trace_errors"] == []
    assert report["validation"]["simulator_adapter_boundary_errors"] == []
    assert report["validation"]["live_hardware_migration_gate_errors"] == []
    assert report["local_asyncio_bus"]["latency_ns"] == 3
    assert report["message_integrity"]["messages_duplicated"] == 1
    assert report["artifact_paths"]["godot_replay_scenario"].endswith(
        "godot_replay_scenario.json"
    )
    assert report["artifact_paths"]["zenoh_openneuro_topic_mapping"].endswith(
        "zenoh_openneuro_topic_mapping.json"
    )
    assert report["artifact_paths"]["zenoh_simulated_trace"].endswith(
        "zenoh_simulated_trace.json"
    )
    assert report["artifact_paths"]["ethercat_model_trace"].endswith(
        "ethercat_model_trace.json"
    )
    assert report["artifact_paths"]["motor_joint_response_trace"].endswith(
        "motor_joint_response_trace.json"
    )
    assert report["artifact_paths"]["simulator_adapter_boundary"].endswith(
        "simulator_adapter_boundary.json"
    )
    assert report["artifact_paths"]["live_hardware_migration_gate"].endswith(
        "live_hardware_migration_gate.json"
    )


def test_closeout_cli_accepts_generated_non_live_evidence(tmp_path: Path) -> None:
    simulation_result = subprocess.run(
        [
            sys.executable,
            str(TOOL_PATH),
            "--cycles",
            "2",
            "--cycle-period-ns",
            "100",
            "--output-root",
            str(tmp_path),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert simulation_result.returncode == 0, simulation_result.stderr

    closeout_result = subprocess.run(
        [
            sys.executable,
            str(CLOSEOUT_TOOL_PATH),
            "--report",
            str(tmp_path / "control_comm_simulation_report.json"),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert closeout_result.returncode == 0, closeout_result.stderr
    closeout = json.loads(
        (tmp_path / "control_comm_simulation_closeout.json").read_text(
            encoding="utf-8"
        )
    )
    assert closeout["closeout_version"] == "control_comm_simulation_closeout.v1"
    assert closeout["status"] == "accepted_with_documented_external_blockers"
    assert closeout["evidence_level"] == "non_live_simulation"
    assert closeout["artifact_error_count"] == 0
    assert closeout["live_hardware_release_gate_status"] == "blocked"
    assert "real_can_transport_not_run" in closeout["external_blockers"]
    assert all(not item["validation_errors"] for item in closeout["artifact_results"])
    assert all(item["size_bytes"] > 0 for item in closeout["artifact_results"])
    assert all(len(item["sha256"]) == 64 for item in closeout["artifact_results"])
    assert closeout["closeout_validation_errors"] == []
    assert (
        control_comm_closeout.validate_control_comm_simulation_closeout(closeout)
        == []
    )


def test_closeout_cli_blocks_malformed_report(tmp_path: Path) -> None:
    report_path = tmp_path / "control_comm_simulation_report.json"
    report_path.write_text(
        json.dumps(
            {
                "report_version": "wrong",
                "status": "success",
                "artifact_paths": {},
            }
        ),
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            sys.executable,
            str(CLOSEOUT_TOOL_PATH),
            "--report",
            str(report_path),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 1
    closeout = json.loads(
        (tmp_path / "control_comm_simulation_closeout.json").read_text(
            encoding="utf-8"
        )
    )
    assert closeout["status"] == "blocked"
    assert "report_version must be 'control_comm_simulation_report.v1'" in closeout[
        "errors"
    ]
    assert closeout["artifact_error_count"] > 0


def test_closeout_contract_rejects_malformed_payload() -> None:
    errors = control_comm_closeout.validate_control_comm_simulation_closeout(
        {
            "closeout_version": "wrong",
            "status": "accepted_with_documented_external_blockers",
            "evidence_level": "live_hardware",
            "scenario_id": "",
            "source_report": "",
            "report_status": "",
            "clock_mode": "",
            "transport_mode": "",
            "simulated_transport_modes": {},
            "cycle_count": -1,
            "timing_metrics": [],
            "message_integrity": [],
            "artifact_results": [
                {
                    "artifact_key": "",
                    "artifact_path": [],
                    "present": "yes",
                    "size_bytes": None,
                    "sha256": None,
                    "validation_errors": ["bad"],
                }
            ],
            "artifact_error_count": 0,
            "live_hardware_release_gate_status": "passed",
            "external_blockers": {},
            "residual_risks": {},
            "errors": [],
        }
    )

    assert "closeout_version must be 'control_comm_simulation_closeout.v1'" in errors
    assert "evidence_level must be 'non_live_simulation'" in errors
    assert "scenario_id must be a non-empty string" in errors
    assert "artifact_results[0].present must be a boolean" in errors
    assert "live_hardware_release_gate_status must be 'blocked'" in errors
    assert "artifact_error_count must equal total artifact validation errors" in errors
    assert "accepted closeout must not contain artifact validation errors" in errors


def test_closeout_contract_rejects_bad_artifact_integrity_metadata() -> None:
    errors = control_comm_closeout.validate_control_comm_simulation_closeout(
        {
            "closeout_version": "control_comm_simulation_closeout.v1",
            "status": "accepted_with_documented_external_blockers",
            "evidence_level": "non_live_simulation",
            "scenario_id": "demo",
            "source_report": "report.json",
            "report_status": "success",
            "clock_mode": "virtual",
            "transport_mode": "asyncio",
            "simulated_transport_modes": ["asyncio"],
            "cycle_count": 1,
            "timing_metrics": {},
            "message_integrity": {},
            "artifact_results": [
                {
                    "artifact_key": "timing_trace",
                    "artifact_path": "timing_trace.json",
                    "present": True,
                    "size_bytes": -1,
                    "sha256": "not-a-sha",
                    "validation_errors": [],
                },
                {
                    "artifact_key": "missing",
                    "artifact_path": "missing.json",
                    "present": False,
                    "size_bytes": 10,
                    "sha256": "0" * 64,
                    "validation_errors": [],
                },
            ],
            "artifact_error_count": 0,
            "live_hardware_release_gate_status": "blocked",
            "external_blockers": [],
            "residual_risks": [],
            "errors": [],
        }
    )

    assert (
        "artifact_results[0].size_bytes must be a non-negative integer" in errors
    )
    assert "artifact_results[0].sha256 must be a SHA-256 hex digest" in errors
    assert "artifact_results[1].size_bytes must be null" in errors
    assert "artifact_results[1].sha256 must be null" in errors
