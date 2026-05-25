from __future__ import annotations

import json
from pathlib import Path

from tools.build_hardwareless_acceptance_report import (
    SCHEMA_VERSION,
    build_hardwareless_acceptance_report,
    main,
)
from tools.validate_hardwareless_release_gate import (
    main as validate_release_gate_main,
    validate_hardwareless_release_gate,
)


ROOT = Path(__file__).resolve().parents[1]
HARDWARE_GUIDE = ROOT / "docs/hardware/HARDWARE_INTEGRATION_GUIDE.md"
PROJECT_PLAN = ROOT / "PROJECT_PLAN.md"
MODULE_PLAN = ROOT / "plans/modules/hardwareless-acceptance.md"


def _ready_godot_readiness() -> dict[str, object]:
    return {
        "schema_version": "dynamic_godot_release_readiness_summary.v1",
        "status": "ready",
        "residual_risks": [],
    }


def _missing_ros2_probe() -> dict[str, object]:
    return {
        "available": False,
        "modules": {
            "rclpy": {"available": False, "error": "missing"},
            "tf2_ros": {"available": False, "error": "missing"},
        },
        "missing_modules": ["rclpy", "tf2_ros"],
    }


def _ready_hardware_closeout() -> dict[str, object]:
    return {"schema_version": "1.0", "status": "ready"}


def _passed_ros2_smoke() -> dict[str, object]:
    return {"test_name": "test_ros2_bridge_humble_smoke", "status": "passed"}


def test_hardwareless_acceptance_records_external_blockers() -> None:
    report = build_hardwareless_acceptance_report(
        no_hardware=True,
        godot_readiness_path="test_env/mountain_biped/live_godot_mountain_readiness.json",
        godot_readiness=_ready_godot_readiness(),
        ros2_probe=_missing_ros2_probe(),
    )

    assert report["schema_version"] == SCHEMA_VERSION
    assert report["status"] == "accepted_with_documented_external_blockers"
    assert report["release_gate"]["status"] == "blocked"
    assert report["enterprise_acceptance_verdict"] == "Blocked"
    assert "external_evidence_missing_or_not_ready" in report["release_gate"]["blockers"]
    assert report["hardware_available"] is False
    assert report["summary"]["godot_readiness_status"] == "ready"
    assert report["summary"]["residual_risk_count"] == 0
    assert report["summary"]["hardware_live_closeout_status"] == "missing_or_not_ready"
    assert report["summary"]["ros2_bridge_live_smoke_status"] == "missing_or_not_ready"
    assert report["summary"]["hardwareless_safety_scenario_count"] >= 6
    assert (
        report["summary"]["hardwareless_safety_scenario_covered_count"]
        == report["summary"]["hardwareless_safety_scenario_count"]
    )
    blocker_ids = {item["id"] for item in report["external_blockers"]}
    assert "real_robot_hardware" in blocker_ids
    assert "real_serial_or_can_transport" in blocker_ids
    assert "ros2_humble_python_runtime" in blocker_ids
    external_evidence = {item["id"]: item for item in report["required_external_evidence"]}
    assert external_evidence["hardware_live_closeout"]["status"] == "missing_or_not_ready"
    assert external_evidence["ros2_bridge_live_smoke"]["status"] == "missing_or_not_ready"
    command_ids = {item["id"] for item in report["substitute_evidence_commands"]}
    assert "serial_driver_mock_replay" in command_ids
    assert "imc22_controller_mock_replay" in command_ids
    assert "ros2_fake_runtime" in command_ids
    scenario_ids = {item["id"] for item in report["hardwareless_safety_scenarios"]}
    assert "command_limit_clamping" in scenario_ids
    assert "watchdog_hold_fallback" in scenario_ids
    assert "fault_class_recovery_policy" in scenario_ids
    assert all(
        item["status"] == "covered_by_substitute_evidence"
        for item in report["hardwareless_safety_scenarios"]
    )
    assert report["residual_risks"] == []


def test_hardwareless_acceptance_resolves_external_blockers_with_live_evidence() -> None:
    report = build_hardwareless_acceptance_report(
        no_hardware=True,
        godot_readiness_path="readiness.json",
        godot_readiness=_ready_godot_readiness(),
        hardware_live_closeout_path="hardware_closeout.json",
        hardware_live_closeout=_ready_hardware_closeout(),
        ros2_bridge_smoke_path="ros2_smoke.json",
        ros2_bridge_smoke=_passed_ros2_smoke(),
        require_external_evidence=True,
        ros2_probe=_missing_ros2_probe(),
    )

    assert report["status"] == "accepted"
    assert report["release_gate"]["status"] == "ready"
    assert report["enterprise_acceptance_verdict"] == "Accepted"
    assert report["external_evidence_required"] is True
    assert report["external_blockers"] == []
    assert report["summary"]["external_blocker_count"] == 0
    external_evidence = {item["id"]: item for item in report["required_external_evidence"]}
    assert external_evidence["hardware_live_closeout"]["status"] == "ready"
    assert external_evidence["ros2_bridge_live_smoke"]["status"] == "ready"


def test_hardwareless_acceptance_strict_mode_blocks_missing_external_evidence() -> None:
    report = build_hardwareless_acceptance_report(
        no_hardware=True,
        godot_readiness_path="readiness.json",
        godot_readiness=_ready_godot_readiness(),
        require_external_evidence=True,
        ros2_probe=_missing_ros2_probe(),
    )

    assert report["status"] == "blocked"
    assert report["release_gate"]["status"] == "blocked"
    assert report["external_evidence_required"] is True
    assert "required_external_evidence_missing_or_not_ready" in report["blockers"]
    assert report["external_blockers"]


def test_hardwareless_acceptance_requires_explicit_no_hardware_confirmation(
    tmp_path: Path,
) -> None:
    readiness = tmp_path / "readiness.json"
    output = tmp_path / "report.json"
    readiness.write_text(json.dumps(_ready_godot_readiness()), encoding="utf-8")

    exit_code = main(
        [
            "--godot-readiness",
            str(readiness),
            "--output",
            str(output),
        ]
    )

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["no_hardware_confirmation_missing"]


def test_hardwareless_acceptance_cli_writes_report(tmp_path: Path) -> None:
    readiness = tmp_path / "readiness.json"
    output = tmp_path / "report.json"
    readiness.write_text(json.dumps(_ready_godot_readiness()), encoding="utf-8")

    exit_code = main(
        [
            "--no-hardware",
            "--godot-readiness",
            str(readiness),
            "--output",
            str(output),
        ]
    )

    assert exit_code in {0, 1}
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["hardware_available"] is False
    assert payload["external_evidence_required"] is False
    assert payload["release_gate"]["status"] == "blocked"
    assert payload["required_local_evidence"][0]["status"] == "ready"
    assert "required_external_evidence" in payload
    assert "zenoh_runtime_probe" in payload


def test_hardwareless_acceptance_cli_strict_mode_exits_blocked(tmp_path: Path) -> None:
    readiness = tmp_path / "readiness.json"
    output = tmp_path / "strict-report.json"
    readiness.write_text(json.dumps(_ready_godot_readiness()), encoding="utf-8")

    exit_code = main(
        [
            "--no-hardware",
            "--require-external-evidence",
            "--godot-readiness",
            str(readiness),
            "--output",
            str(output),
        ]
    )

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["release_gate"]["status"] == "blocked"
    assert "required_external_evidence_missing_or_not_ready" in payload["blockers"]


def test_hardwareless_release_gate_validator_accepts_ready_report() -> None:
    report = build_hardwareless_acceptance_report(
        no_hardware=True,
        godot_readiness_path="readiness.json",
        godot_readiness=_ready_godot_readiness(),
        hardware_live_closeout=_ready_hardware_closeout(),
        ros2_bridge_smoke=_passed_ros2_smoke(),
        require_external_evidence=True,
        ros2_probe=_missing_ros2_probe(),
    )

    assert validate_hardwareless_release_gate(report) == []


def test_hardwareless_release_gate_validator_rejects_blocked_report(
    tmp_path: Path,
) -> None:
    report = build_hardwareless_acceptance_report(
        no_hardware=True,
        godot_readiness_path="readiness.json",
        godot_readiness=_ready_godot_readiness(),
        ros2_probe=_missing_ros2_probe(),
    )
    report_path = tmp_path / "blocked-report.json"
    report_path.write_text(json.dumps(report), encoding="utf-8")

    errors = validate_hardwareless_release_gate(report)
    exit_code = validate_release_gate_main([str(report_path)])

    assert "release_gate.status must be 'ready'; got 'blocked'" in errors
    assert exit_code == 1


def test_hardwareless_release_gate_validator_writes_output_artifact(
    tmp_path: Path,
) -> None:
    report = build_hardwareless_acceptance_report(
        no_hardware=True,
        godot_readiness_path="readiness.json",
        godot_readiness=_ready_godot_readiness(),
        ros2_probe=_missing_ros2_probe(),
    )
    report_path = tmp_path / "blocked-report.json"
    output_path = tmp_path / "validation-report.json"
    report_path.write_text(json.dumps(report), encoding="utf-8")

    exit_code = validate_release_gate_main(
        [
            str(report_path),
            "--output",
            str(output_path),
        ]
    )

    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert exit_code == 1
    assert payload["status"] == "failed"
    assert payload["actual_status"] == "blocked"
    assert payload["errors"]


def test_hardwareless_acceptance_docs_are_linked() -> None:
    tool = "tools/build_hardwareless_acceptance_report.py"
    validator = "tools/validate_hardwareless_release_gate.py"
    report = "test_env/hardwareless_acceptance/hardwareless_acceptance_report.json"

    assert tool in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert validator in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert report in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert "hardwareless-acceptance" in PROJECT_PLAN.read_text(encoding="utf-8")
    assert tool in MODULE_PLAN.read_text(encoding="utf-8")
    assert validator in MODULE_PLAN.read_text(encoding="utf-8")
