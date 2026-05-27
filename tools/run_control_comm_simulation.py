"""Build deterministic control/communication simulation evidence artifacts."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from agi_walker.core.simulation.control_comm_simulation import (  # noqa: E402
    build_default_control_comm_scenario,
    run_deterministic_control_comm_simulation,
    validate_control_comm_simulation_report,
    validate_ethercat_model_trace,
    validate_godot_control_comm_simulation_log,
    validate_live_hardware_migration_gate,
    validate_motor_joint_response_trace,
    validate_simulator_adapter_boundary,
    validate_zenoh_openneuro_topic_mapping,
    validate_zenoh_simulated_trace,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate deterministic control/communication simulation evidence."
    )
    parser.add_argument("--scenario", type=Path, help="Optional scenario JSON path.")
    parser.add_argument(
        "--scenario-id",
        default="default_joint_command_stream",
        help="Scenario id used when --scenario is omitted.",
    )
    parser.add_argument("--cycles", type=int, default=4)
    parser.add_argument("--cycle-period-ns", type=int, default=10_000_000)
    parser.add_argument("--jitter-budget-ns", type=int, default=0)
    parser.add_argument("--bus-latency-ns", type=int, default=0)
    parser.add_argument(
        "--bus-jitter-by-cycle-ns",
        default="[]",
        help="JSON array of per-cycle virtual jitter values in nanoseconds.",
    )
    parser.add_argument(
        "--bus-drop-sequences",
        default="[]",
        help="JSON array of non-negative message sequence numbers to drop.",
    )
    parser.add_argument(
        "--bus-duplicate-sequences",
        default="[]",
        help="JSON array of non-negative message sequence numbers to duplicate.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=ROOT / "test_env" / "control_comm_simulation",
    )
    parser.add_argument(
        "--no-godot-log-contract-preview",
        action="store_true",
        help="Do not write the non-live Godot log contract preview artifact.",
    )
    parser.add_argument(
        "--no-zenoh-openneuro-simulation",
        action="store_true",
        help="Do not write Zenoh/OpenNeuro-like simulated mapping and trace artifacts.",
    )
    parser.add_argument(
        "--no-ethercat-cycle-model",
        action="store_true",
        help="Do not write the non-live EtherCAT cycle/PDO/watchdog model trace.",
    )
    parser.add_argument(
        "--ethercat-watchdog-timeout-cycles",
        type=int,
        default=2,
        help="Consecutive EtherCAT model misses before the watchdog trips.",
    )
    parser.add_argument(
        "--ethercat-deadline-budget-ns",
        type=int,
        default=None,
        help="EtherCAT model deadline budget. Defaults to --jitter-budget-ns.",
    )
    parser.add_argument(
        "--no-motor-joint-model",
        action="store_true",
        help="Do not write the non-live motor/joint response model trace.",
    )
    parser.add_argument("--motor-velocity-limit-rad-s", type=float, default=1.0)
    parser.add_argument("--motor-torque-limit-nm", type=float, default=2.0)
    parser.add_argument("--joint-position-lower-rad", type=float, default=-1.0)
    parser.add_argument("--joint-position-upper-rad", type=float, default=1.0)
    parser.add_argument(
        "--no-simulator-adapter-boundary",
        action="store_true",
        help="Do not write the Gazebo/MuJoCo/Isaac Sim adapter boundary artifact.",
    )
    parser.add_argument(
        "--no-live-hardware-migration-gate",
        action="store_true",
        help="Do not write the fail-closed CAN/EtherCAT/TSN migration gate artifact.",
    )
    args = parser.parse_args()

    scenario = (
        json.loads(args.scenario.read_text(encoding="utf-8"))
        if args.scenario
        else build_default_control_comm_scenario(
            scenario_id=args.scenario_id,
            cycle_count=args.cycles,
            cycle_period_ns=args.cycle_period_ns,
        )
    )
    if not args.scenario:
        scenario["jitter_budget_ns"] = args.jitter_budget_ns
        scenario["bus"] = {
            "mode": "local_asyncio_virtual",
            "latency_ns": args.bus_latency_ns,
            "jitter_by_cycle_ns": json.loads(args.bus_jitter_by_cycle_ns),
            "drop_sequences": json.loads(args.bus_drop_sequences),
            "duplicate_sequences": json.loads(args.bus_duplicate_sequences),
        }
        scenario["ethercat"] = {
            "mode": "ethercat_cycle_model",
            "watchdog_timeout_cycles": args.ethercat_watchdog_timeout_cycles,
            "deadline_budget_ns": (
                args.jitter_budget_ns
                if args.ethercat_deadline_budget_ns is None
                else args.ethercat_deadline_budget_ns
            ),
            "pdo_input_map": ["target_velocity_rad_s", "target_torque_nm"],
            "pdo_output_map": ["position_rad", "velocity_rad_s", "torque_nm"],
        }
        scenario["motor_joint"] = {
            "mode": "motor_joint_model",
            "joint_name": scenario["command"].get("joint_name", "left_hip"),
            "position_lower_rad": args.joint_position_lower_rad,
            "position_upper_rad": args.joint_position_upper_rad,
            "velocity_limit_rad_s": args.motor_velocity_limit_rad_s,
            "torque_limit_nm": args.motor_torque_limit_nm,
            "friction_nm": 0.0,
            "backlash_rad": 0.0,
        }
    report = run_deterministic_control_comm_simulation(
        scenario,
        output_root=args.output_root,
        include_godot_log_contract=not args.no_godot_log_contract_preview,
        include_zenoh_openneuro_simulation=not args.no_zenoh_openneuro_simulation,
        include_ethercat_cycle_model=not args.no_ethercat_cycle_model,
        include_motor_joint_model=not args.no_motor_joint_model,
        include_simulator_adapter_boundary=not args.no_simulator_adapter_boundary,
        include_live_hardware_migration_gate=not args.no_live_hardware_migration_gate,
    )
    report_errors = validate_control_comm_simulation_report(report)
    godot_log_errors: list[str] = []
    godot_log_path = report["artifact_paths"].get("godot_log_contract_preview")
    if godot_log_path:
        godot_log_errors = validate_godot_control_comm_simulation_log(
            json.loads(Path(godot_log_path).read_text(encoding="utf-8"))
        )
    zenoh_mapping_errors: list[str] = []
    zenoh_trace_errors: list[str] = []
    zenoh_mapping_path = report["artifact_paths"].get("zenoh_openneuro_topic_mapping")
    zenoh_trace_path = report["artifact_paths"].get("zenoh_simulated_trace")
    if zenoh_mapping_path:
        zenoh_mapping_errors = validate_zenoh_openneuro_topic_mapping(
            json.loads(Path(zenoh_mapping_path).read_text(encoding="utf-8"))
        )
    if zenoh_trace_path:
        zenoh_trace_errors = validate_zenoh_simulated_trace(
            json.loads(Path(zenoh_trace_path).read_text(encoding="utf-8"))
        )
    ethercat_trace_errors: list[str] = []
    ethercat_trace_path = report["artifact_paths"].get("ethercat_model_trace")
    if ethercat_trace_path:
        ethercat_trace_errors = validate_ethercat_model_trace(
            json.loads(Path(ethercat_trace_path).read_text(encoding="utf-8"))
        )
    motor_joint_trace_errors: list[str] = []
    motor_joint_trace_path = report["artifact_paths"].get("motor_joint_response_trace")
    if motor_joint_trace_path:
        motor_joint_trace_errors = validate_motor_joint_response_trace(
            json.loads(Path(motor_joint_trace_path).read_text(encoding="utf-8"))
        )
    simulator_adapter_boundary_errors: list[str] = []
    simulator_adapter_boundary_path = report["artifact_paths"].get(
        "simulator_adapter_boundary"
    )
    if simulator_adapter_boundary_path:
        simulator_adapter_boundary_errors = validate_simulator_adapter_boundary(
            json.loads(
                Path(simulator_adapter_boundary_path).read_text(encoding="utf-8")
            )
        )
    live_hardware_migration_gate_errors: list[str] = []
    live_hardware_migration_gate_path = report["artifact_paths"].get(
        "live_hardware_migration_gate"
    )
    if live_hardware_migration_gate_path:
        live_hardware_migration_gate_errors = validate_live_hardware_migration_gate(
            json.loads(
                Path(live_hardware_migration_gate_path).read_text(encoding="utf-8")
            )
        )
    report["validation"] = {
        "control_comm_simulation_report_errors": report_errors,
        "godot_control_comm_simulation_log_errors": godot_log_errors,
        "zenoh_openneuro_topic_mapping_errors": zenoh_mapping_errors,
        "zenoh_simulated_trace_errors": zenoh_trace_errors,
        "ethercat_model_trace_errors": ethercat_trace_errors,
        "motor_joint_response_trace_errors": motor_joint_trace_errors,
        "simulator_adapter_boundary_errors": simulator_adapter_boundary_errors,
        "live_hardware_migration_gate_errors": live_hardware_migration_gate_errors,
    }
    if (
        report_errors
        or godot_log_errors
        or zenoh_mapping_errors
        or zenoh_trace_errors
        or ethercat_trace_errors
        or motor_joint_trace_errors
        or simulator_adapter_boundary_errors
        or live_hardware_migration_gate_errors
    ):
        report["status"] = "error"
        report["errors"] = [
            *report_errors,
            *godot_log_errors,
            *zenoh_mapping_errors,
            *zenoh_trace_errors,
            *ethercat_trace_errors,
            *motor_joint_trace_errors,
            *simulator_adapter_boundary_errors,
            *live_hardware_migration_gate_errors,
        ]

    report_path = args.output_root / "control_comm_simulation_report.json"
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"control_comm_simulation_report={report_path}")
    print(f"control_comm_simulation_status={report['status']}")
    if report["errors"]:
        print("control_comm_simulation_errors=" + "; ".join(report["errors"]))
    return 1 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
