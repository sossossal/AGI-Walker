from __future__ import annotations

import argparse
import importlib
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GODOT_READINESS = (
    "test_env/mountain_biped/live_godot_mountain_readiness.json"
)
DEFAULT_OUTPUT = (
    "test_env/hardwareless_acceptance/hardwareless_acceptance_report.json"
)
DEFAULT_HARDWARE_LIVE_CLOSEOUT = (
    "test_env/hardware_live/hardware_live_closeout_report.json"
)
DEFAULT_ROS2_BRIDGE_SMOKE = (
    "test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json"
)
SCHEMA_VERSION = "hardwareless_acceptance_report.v1"
SOURCE_PATH_FIELDS = (
    "godot_readiness",
    "hardware_live_closeout",
    "ros2_bridge_smoke",
)
ROS2_RUNTIME_MODULES = (
    "rclpy",
    "tf2_ros",
    "sensor_msgs.msg",
    "geometry_msgs.msg",
    "std_srvs.srv",
)


HARDWARELESS_SAFETY_SCENARIOS = [
    {
        "id": "command_limit_clamping",
        "risk_reduced": "unsafe target angle or compliance command",
        "evidence_command_ids": ["imc22_controller_mock_replay"],
        "covered_by_tests": [
            "tests/test_hardware_controller.py::TestIMC22Controller::test_send_command_clamps_by_safety_profile",
            "tests/test_hardware_controller.py::TestIMC22Controller::test_command_angle_bounds",
            "tests/test_hardware_controller.py::TestIMC22Controller::test_command_compliance_bounds",
        ],
    },
    {
        "id": "watchdog_hold_fallback",
        "risk_reduced": "stale command or controller communication pause",
        "evidence_command_ids": ["imc22_controller_mock_replay"],
        "covered_by_tests": [
            "tests/test_hardware_controller.py::TestIMC22Controller::test_watchdog_sends_hold_commands_for_known_nodes",
            "tests/test_hardware_controller.py::TestIMC22Controller::test_clear_faults_resets_watchdog_state",
        ],
    },
    {
        "id": "fault_class_recovery_policy",
        "risk_reduced": "undifferentiated overload, overcurrent, sensor or communication fault handling",
        "evidence_command_ids": ["imc22_controller_mock_replay", "ros2_fake_runtime"],
        "covered_by_tests": [
            "tests/test_hardware_controller.py::TestIMC22Controller::test_fault_summary_groups_node_errors",
            "tests/test_hardware_controller.py::TestIMC22Controller::test_recover_by_fault_class_executes_fault_specific_actions",
            "tests/test_ros2_bridge_runtime.py::test_ros2_bridge_hardware_recovery_services",
        ],
    },
    {
        "id": "serial_protocol_replay",
        "risk_reduced": "driver packet encoding or replay drift before serial hardware is available",
        "evidence_command_ids": ["serial_driver_mock_replay"],
        "covered_by_tests": [
            "tests/test_real_robot_driver.py::test_real_robot_driver_from_replay_updates_state_and_records_command",
            "tests/test_real_robot_driver.py::test_real_robot_driver_send_motor_config_updates_state_and_records_packet",
        ],
    },
    {
        "id": "hardware_recovery_permission_gate",
        "risk_reduced": "unauthorized recover or clear-fault operation through Web APIs",
        "evidence_command_ids": ["web_hardware_recovery_mock"],
        "covered_by_tests": [
            "tests/test_web_hardware_operation_permissions_static.py",
            "tests/test_web_hardware_role_policy.py",
            "tests/test_web_godot_session_bridge.py::test_hardware_recovery_routes",
        ],
    },
    {
        "id": "live_godot_mountain_mechanical_gate",
        "risk_reduced": "robot generation, terrain setup or mechanical telemetry drift outside Python replay",
        "evidence_command_ids": ["live_godot_mountain_biped"],
        "covered_by_tests": [
            "tests/test_dynamic_godot_robot_generation.py",
            "tests/test_mountain_biped_simulation.py",
        ],
    },
]


SUBSTITUTE_EVIDENCE_COMMANDS = [
    {
        "id": "serial_driver_mock_replay",
        "scope": "real_robot_driver protocol and SysID mock path",
        "command": "py -3.12 -m pytest tests\\test_real_robot_driver.py -q -rs",
        "hardware_required": False,
    },
    {
        "id": "imc22_controller_mock_replay",
        "scope": "IMC-22 encoding, replay bus, safety and recovery logic",
        "command": "py -3.12 -m pytest tests\\test_hardware_controller.py -q -rs",
        "hardware_required": False,
    },
    {
        "id": "ros2_fake_runtime",
        "scope": "ROS2 bridge payload mapping with in-process fake runtime",
        "command": "py -3.12 -m pytest tests\\test_ros2_bridge_runtime.py tests\\test_ros2_workspace.py -q -rs",
        "hardware_required": False,
    },
    {
        "id": "web_hardware_recovery_mock",
        "scope": "Web hardware recovery permission and mock session bridge",
        "command": "py -3.12 -m pytest tests\\test_web_godot_session_bridge.py tests\\test_web_hardware_operation_permissions_static.py tests\\test_web_hardware_role_policy.py -q -rs",
        "hardware_required": False,
    },
    {
        "id": "live_godot_mountain_biped",
        "scope": "Live Godot humanoid biped on generated mountain terrain",
        "command": (
            "py -3.12 tools\\build_dynamic_robot_generation_report.py "
            "configs\\mountain_humanoid_biped.json --terrain-json "
            "configs\\mountain_terrain.json --full-mechanical-restoration-acceptance"
        ),
        "hardware_required": False,
    },
    {
        "id": "production_compose_mock_workflow",
        "scope": "Authenticated Web workflow through supported Docker Compose stack",
        "command": (
            "$env:AGI_WALKER_ENABLE_PROD_COMPOSE_SMOKE='1'; "
            "py -3.12 -m pytest tests\\test_prod_compose_smoke.py -q -rs"
        ),
        "hardware_required": False,
    },
    {
        "id": "distributed_profile_smoke",
        "scope": "Distributed Zenoh monitor and actor discovery profile",
        "command": (
            "py -3.12 tests\\run_distributed_smoke.py --build --stop-after "
            "--report-file test_env\\distributed_smoke\\distributed_smoke_report.json"
        ),
        "hardware_required": False,
        "live_external_service": True,
    },
]


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a fail-explicit no-hardware acceptance report."
    )
    parser.add_argument(
        "--no-hardware",
        action="store_true",
        help="Confirm that this report is for a no-hardware validation pass.",
    )
    parser.add_argument("--godot-readiness", default=DEFAULT_GODOT_READINESS)
    parser.add_argument("--hardware-live-closeout", default=DEFAULT_HARDWARE_LIVE_CLOSEOUT)
    parser.add_argument("--ros2-bridge-smoke", default=DEFAULT_ROS2_BRIDGE_SMOKE)
    parser.add_argument(
        "--require-external-evidence",
        action="store_true",
        help="Fail the report unless real hardware and ROS2 live evidence are ready.",
    )
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _resolve_relative_path(
    value: str | Path | None,
    *,
    base_dir: Path,
) -> tuple[bool, Path | None, str | None]:
    text = _text(str(value)) if value is not None else ""
    if not text:
        return False, None, "empty"
    path = Path(text)
    if path.is_absolute():
        return False, None, "absolute"
    if ".." in path.parts:
        return False, None, "parent_directory"
    base_relative = base_dir / path
    if base_relative.exists():
        return True, base_relative, None
    return True, PROJECT_ROOT / path, None


def _path_status(value: str | Path | None, *, base_dir: Path) -> dict[str, Any]:
    valid, resolved_path, error = _resolve_relative_path(value, base_dir=base_dir)
    return {
        "path": _text(str(value)) if value is not None else "",
        "path_valid": valid,
        "path_error": error,
        "resolved_path": str(resolved_path) if resolved_path is not None else None,
        "exists": bool(resolved_path and resolved_path.exists()),
    }


def _source_path_blockers(statuses: dict[str, dict[str, Any]]) -> list[dict[str, str]]:
    blockers: list[dict[str, str]] = []
    for field in SOURCE_PATH_FIELDS:
        status = statuses[field]
        if not status["path_valid"]:
            blockers.append({"field": field, "reason": str(status["path_error"])})
    return blockers


def _load_json_if_exists(status: dict[str, Any]) -> dict[str, Any] | None:
    if not status["path_valid"] or not status["exists"] or not status["resolved_path"]:
        return None
    json_path = Path(status["resolved_path"])
    return json.loads(json_path.read_text(encoding="utf-8"))


def _probe_ros2_runtime(
    importer: Callable[[str], Any] | None = None,
) -> dict[str, Any]:
    import_module = importer or importlib.import_module
    modules: dict[str, dict[str, Any]] = {}
    missing_modules: list[str] = []
    for module_name in ROS2_RUNTIME_MODULES:
        try:
            import_module(module_name)
            modules[module_name] = {"available": True, "error": None}
        except Exception as exc:
            modules[module_name] = {"available": False, "error": str(exc)}
            missing_modules.append(module_name)
    return {
        "available": not missing_modules,
        "modules": modules,
        "missing_modules": missing_modules,
    }


def _probe_module(module_name: str) -> dict[str, Any]:
    try:
        importlib.import_module(module_name)
        return {"available": True, "module": module_name, "error": None}
    except Exception as exc:
        return {"available": False, "module": module_name, "error": str(exc)}


def _godot_evidence(path: str, payload: dict[str, Any] | None) -> dict[str, Any]:
    status = None if payload is None else payload.get("status")
    residual_risks = [] if payload is None else payload.get("residual_risks", [])
    ready = payload is not None and status == "ready" and not residual_risks
    return {
        "id": "live_godot_mountain_readiness",
        "path": path,
        "status": "ready" if ready else "missing_or_not_ready",
        "actual_status": status,
        "residual_risk_count": len(residual_risks)
        if isinstance(residual_risks, list)
        else None,
        "required_for_hardwareless_acceptance": True,
    }


def _external_evidence(
    evidence_id: str,
    path: str,
    payload: dict[str, Any] | None,
    *,
    expected_status: str,
) -> dict[str, Any]:
    actual_status = None if payload is None else payload.get("status")
    status = "ready" if actual_status == expected_status else "missing_or_not_ready"
    return {
        "id": evidence_id,
        "path": path,
        "status": status,
        "actual_status": actual_status,
        "expected_status": expected_status,
    }


def _external_blockers(
    *,
    ros2_probe: dict[str, Any],
    hardware_live_evidence: dict[str, Any],
    ros2_live_evidence: dict[str, Any],
) -> list[dict[str, Any]]:
    blockers = []
    if hardware_live_evidence["status"] != "ready":
        blockers.extend(
            [
                {
                    "id": "real_robot_hardware",
                    "status": "blocked_external_prerequisite",
                    "reason": "physical robot, motors, encoders and power chain are not available",
                    "required_evidence_id": hardware_live_evidence["id"],
                    "unblocks": "customer-site hardware motion, actuator load and emergency-stop validation",
                },
                {
                    "id": "real_serial_or_can_transport",
                    "status": "blocked_external_prerequisite",
                    "reason": "real serial/CAN adapter and configured channel are not available",
                    "required_evidence_id": hardware_live_evidence["id"],
                    "unblocks": "live IMC-22 transport diagnostics and fault telemetry closeout",
                },
            ]
        )
    if ros2_live_evidence["status"] != "ready" and not ros2_probe["available"]:
        blockers.append(
            {
                "id": "ros2_humble_python_runtime",
                "status": "blocked_external_prerequisite",
                "reason": "ROS2 Python runtime modules are missing",
                "required_evidence_id": ros2_live_evidence["id"],
                "missing_modules": ros2_probe["missing_modules"],
                "unblocks": "real ROS2 bridge live smoke",
            }
        )
    elif ros2_live_evidence["status"] != "ready":
        blockers.append(
            {
                "id": "ros2_bridge_live_smoke",
                "status": "blocked_external_prerequisite",
                "reason": "ROS2 runtime is available but passed live bridge smoke evidence was not supplied",
                "required_evidence_id": ros2_live_evidence["id"],
                "unblocks": "real ROS2 bridge live smoke",
            }
        )
    return blockers


def _safety_scenario_entries() -> list[dict[str, Any]]:
    command_ids = {item["id"] for item in SUBSTITUTE_EVIDENCE_COMMANDS}
    entries = []
    for scenario in HARDWARELESS_SAFETY_SCENARIOS:
        missing = [
            command_id
            for command_id in scenario["evidence_command_ids"]
            if command_id not in command_ids
        ]
        entries.append(
            {
                **scenario,
                "status": "covered_by_substitute_evidence"
                if not missing
                else "evidence_command_missing",
                "missing_evidence_command_ids": missing,
            }
        )
    return entries


def _release_gate(
    *,
    external_blockers: list[dict[str, Any]],
    blockers: list[str],
    require_external_evidence: bool,
) -> dict[str, Any]:
    gate_blockers = list(blockers)
    if external_blockers:
        gate_blockers.append("external_evidence_missing_or_not_ready")
    return {
        "status": "blocked" if gate_blockers else "ready",
        "strict_mode": require_external_evidence,
        "blockers": gate_blockers,
        "external_blocker_ids": [item["id"] for item in external_blockers],
    }


def build_hardwareless_acceptance_report(
    *,
    no_hardware: bool,
    godot_readiness_path: str,
    godot_readiness: dict[str, Any] | None,
    hardware_live_closeout_path: str = DEFAULT_HARDWARE_LIVE_CLOSEOUT,
    hardware_live_closeout: dict[str, Any] | None = None,
    ros2_bridge_smoke_path: str = DEFAULT_ROS2_BRIDGE_SMOKE,
    ros2_bridge_smoke: dict[str, Any] | None = None,
    require_external_evidence: bool = False,
    ros2_probe: dict[str, Any] | None = None,
    source_path_statuses: dict[str, dict[str, Any]] | None = None,
    source_path_blockers: list[dict[str, str]] | None = None,
) -> dict[str, Any]:
    path_statuses = source_path_statuses or {}
    path_blockers = source_path_blockers or []
    ros2_runtime = ros2_probe or _probe_ros2_runtime()
    zenoh_runtime = _probe_module("zenoh")
    godot = _godot_evidence(godot_readiness_path, godot_readiness)
    hardware_live = _external_evidence(
        "hardware_live_closeout",
        hardware_live_closeout_path,
        hardware_live_closeout,
        expected_status="ready",
    )
    ros2_live = _external_evidence(
        "ros2_bridge_live_smoke",
        ros2_bridge_smoke_path,
        ros2_bridge_smoke,
        expected_status="passed",
    )
    external_blockers = _external_blockers(
        ros2_probe=ros2_runtime,
        hardware_live_evidence=hardware_live,
        ros2_live_evidence=ros2_live,
    )
    safety_scenarios = _safety_scenario_entries()
    residual_risks: list[dict[str, Any]] = []
    blockers: list[str] = []
    if not no_hardware:
        blockers.append("no_hardware_confirmation_missing")
    if godot["status"] != "ready":
        blockers.append("live_godot_mountain_readiness_missing_or_not_ready")
    if any(item["status"] != "covered_by_substitute_evidence" for item in safety_scenarios):
        blockers.append("hardwareless_safety_scenario_evidence_incomplete")
    if require_external_evidence and external_blockers:
        blockers.append("required_external_evidence_missing_or_not_ready")
    if path_blockers:
        blockers.append("source_path_validation")

    if blockers:
        status = "blocked"
    elif external_blockers:
        status = "accepted_with_documented_external_blockers"
    else:
        status = "accepted"
    release_gate = _release_gate(
        external_blockers=external_blockers,
        blockers=blockers,
        require_external_evidence=require_external_evidence,
    )

    return {
        "schema_version": SCHEMA_VERSION,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": status,
        "hardware_available": False if no_hardware else None,
        "external_evidence_required": require_external_evidence,
        "release_gate": release_gate,
        "enterprise_acceptance_verdict": (
            "Blocked"
            if release_gate["status"] == "blocked"
            else "Accepted"
        ),
        "summary": {
            "substitute_evidence_command_count": len(SUBSTITUTE_EVIDENCE_COMMANDS),
            "hardwareless_safety_scenario_count": len(safety_scenarios),
            "hardwareless_safety_scenario_covered_count": sum(
                1
                for item in safety_scenarios
                if item["status"] == "covered_by_substitute_evidence"
            ),
            "external_blocker_count": len(external_blockers),
            "residual_risk_count": len(residual_risks),
            "godot_readiness_status": godot["status"],
            "hardware_live_closeout_status": hardware_live["status"],
            "ros2_bridge_live_smoke_status": ros2_live["status"],
            "ros2_runtime_available": ros2_runtime["available"],
            "zenoh_python_runtime_available": zenoh_runtime["available"],
        },
        "sources": {
            "godot_readiness": godot_readiness_path,
            "hardware_live_closeout": hardware_live_closeout_path,
            "ros2_bridge_smoke": ros2_bridge_smoke_path,
        },
        "source_path_statuses": path_statuses,
        "source_path_blockers": path_blockers,
        "required_local_evidence": [godot],
        "required_external_evidence": [hardware_live, ros2_live],
        "substitute_evidence_commands": SUBSTITUTE_EVIDENCE_COMMANDS,
        "hardwareless_safety_scenarios": safety_scenarios,
        "ros2_runtime_probe": ros2_runtime,
        "zenoh_runtime_probe": zenoh_runtime,
        "external_blockers": external_blockers,
        "residual_risks": residual_risks,
        "blockers": blockers,
        "next_actions": _next_actions(status, blockers),
    }


def _next_actions(status: str, blockers: list[str]) -> list[str]:
    if status == "blocked":
        return [
            f"Resolve hardwareless acceptance blockers: {', '.join(blockers)}."
        ]
    return [
        "Keep this report with the targeted test output as no-hardware acceptance evidence.",
        "Before real deployment, run hardware live closeout and ROS2 bridge live smoke in the target environment.",
    ]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output_dir = Path(args.output).resolve().parent
    source_path_statuses = {
        "godot_readiness": _path_status(args.godot_readiness, base_dir=output_dir),
        "hardware_live_closeout": _path_status(
            args.hardware_live_closeout,
            base_dir=output_dir,
        ),
        "ros2_bridge_smoke": _path_status(
            args.ros2_bridge_smoke,
            base_dir=output_dir,
        ),
    }
    source_path_blockers = _source_path_blockers(source_path_statuses)
    report = build_hardwareless_acceptance_report(
        no_hardware=args.no_hardware,
        godot_readiness_path=args.godot_readiness,
        godot_readiness=_load_json_if_exists(source_path_statuses["godot_readiness"]),
        hardware_live_closeout_path=args.hardware_live_closeout,
        hardware_live_closeout=_load_json_if_exists(
            source_path_statuses["hardware_live_closeout"]
        ),
        ros2_bridge_smoke_path=args.ros2_bridge_smoke,
        ros2_bridge_smoke=_load_json_if_exists(
            source_path_statuses["ros2_bridge_smoke"]
        ),
        require_external_evidence=args.require_external_evidence,
        source_path_statuses=source_path_statuses,
        source_path_blockers=source_path_blockers,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if report["status"] != "blocked" else 1


if __name__ == "__main__":
    raise SystemExit(main())
