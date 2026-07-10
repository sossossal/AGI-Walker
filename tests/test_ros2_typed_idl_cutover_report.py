from __future__ import annotations

import json
from pathlib import Path

from tools.build_ros2_typed_idl_cutover_report import main


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
MIGRATION_RUNBOOK = ROOT / "docs/ros2/ROS2_TYPED_IDL_MIGRATION.md"
TEMPLATE = ROOT / "deployment/ros2_typed_idl_cutover.template.json"


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _ready_payload(tmp_path: Path) -> dict[str, object]:
    smoke = tmp_path / "ros2_bridge_smoke_report.json"
    inventory = tmp_path / "typed_inventory.json"
    rollback = tmp_path / "rollback.md"
    _write_json(smoke, {"schema_version": "1.0", "status": "passed"})
    _write_json(inventory, {"schema_version": "1.0", "typed_surfaces": ["instruction_set"]})
    rollback.write_text("# rollback\n", encoding="utf-8")
    return {
        "schema_version": "1.0",
        "cutover_id": "cutover-test",
        "target_environment": "ros2-humble-prod",
        "operator": "ros2-operator",
        "rollback_owner": "rollback-owner",
        "launch_profile": "hardware/ros2_ws/src/agi_walker_ros2/config/profiles/live.yaml",
        "json_writers_disabled": True,
        "typed_surfaces_verified": [
            {
                "name": "instruction_set",
                "legacy_surface": "/instruction_set/json",
                "typed_surface": "/instruction_set",
                "status": "passed",
            },
            {
                "name": "hardware_recovery",
                "legacy_surface": "/hardware/recover_by_fault_class",
                "typed_surface": "/hardware/recovery",
                "status": "ready",
            },
        ],
        "evidence": {
            "live_smoke_report": smoke.name,
            "typed_inventory": inventory.name,
            "rollback_plan": rollback.name,
        },
    }


def test_ros2_typed_idl_cutover_report_ready(tmp_path: Path) -> None:
    input_path = tmp_path / "cutover.json"
    output_path = tmp_path / "cutover_report.json"
    _write_json(input_path, _ready_payload(tmp_path))

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output_path),
            "--require-evidence-files",
        ]
    )

    assert exit_code == 0
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["blockers"] == []
    assert payload["summary"]["verified_surface_count"] == 2
    assert payload["summary"]["live_smoke_status"] == "passed"
    assert payload["summary"]["evidence_path_validation_error_count"] == 0
    assert payload["evidence"]["path_statuses"]["typed_inventory"]["exists"] is True


def test_ros2_typed_idl_cutover_blocks_template_defaults(tmp_path: Path) -> None:
    template_payload = json.loads(TEMPLATE.read_text(encoding="utf-8"))
    template_payload["evidence"] = {
        "live_smoke_report": "missing_smoke.json",
        "typed_inventory": "missing_inventory.json",
        "rollback_plan": "missing_rollback.md",
    }
    input_path = tmp_path / "template.json"
    output_path = tmp_path / "cutover_report.json"
    _write_json(input_path, template_payload)

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output_path),
        ]
    )

    assert exit_code == 1
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "target_environment" in payload["blockers"]
    assert "json_writers_disabled" in payload["blockers"]
    assert "instruction_set" in payload["blockers"]
    assert payload["summary"]["verified_surface_count"] == 0
    assert payload["summary"]["blocked_surface_count"] == 4
    assert payload["summary"]["evidence_path_validation_error_count"] == 0


def test_ros2_typed_idl_cutover_blocks_nonpassing_surface(tmp_path: Path) -> None:
    input_path = tmp_path / "cutover.json"
    output_path = tmp_path / "cutover_report.json"
    payload = _ready_payload(tmp_path)
    payload["typed_surfaces_verified"][0]["status"] = "failed"
    _write_json(input_path, payload)

    exit_code = main(["--input", str(input_path), "--output", str(output_path)])

    assert exit_code == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["blockers"] == ["instruction_set"]
    assert report["typed_surfaces"][0]["status"] == "blocked"


def test_ros2_typed_idl_cutover_accepts_inventory_verified_pending_surfaces(
    tmp_path: Path,
) -> None:
    input_path = tmp_path / "cutover.json"
    output_path = tmp_path / "cutover_report.json"
    payload = _ready_payload(tmp_path)
    payload["typed_surfaces_verified"][0]["status"] = "pending"
    payload["typed_surfaces_verified"][1]["status"] = "pending"
    inventory_path = tmp_path / str(payload["evidence"]["typed_inventory"])
    _write_json(
        inventory_path,
        {
            "schema_version": "1.0",
            "typed_surfaces": [
                {"name": "instruction_set", "status": "ready"},
                {"name": "hardware_recovery", "status": "passed"},
            ],
        },
    )
    _write_json(input_path, payload)

    exit_code = main(["--input", str(input_path), "--output", str(output_path)])

    assert exit_code == 0
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["summary"]["verified_surface_count"] == 2
    assert report["typed_surfaces"][0]["inventory_status"] == "ready"
    assert report["typed_surfaces"][1]["inventory_status"] == "passed"


def test_ros2_typed_idl_cutover_blocks_unsafe_evidence_paths(
    tmp_path: Path,
) -> None:
    input_path = tmp_path / "cutover.json"
    output_path = tmp_path / "cutover_report.json"
    payload = _ready_payload(tmp_path)
    payload["evidence"] = {
        "live_smoke_report": "../ros2_bridge_smoke_report.json",
        "typed_inventory": str((tmp_path / "typed_inventory.json").resolve()),
        "rollback_plan": "rollback.md",
    }
    _write_json(input_path, payload)

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output_path),
            "--require-evidence-files",
        ]
    )

    assert exit_code == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["status"] == "blocked"
    assert report["summary"]["evidence_path_validation_error_count"] == 2
    assert (
        report["evidence"]["path_statuses"]["live_smoke_report"]["path_error"]
        == "parent_directory"
    )
    assert report["evidence"]["path_statuses"]["typed_inventory"]["path_error"] == "absolute"
    assert "live_smoke_report" in report["blockers"]
    assert "typed_inventory" in report["blockers"]


def test_ros2_typed_idl_cutover_docs_are_linked() -> None:
    tool = "tools/build_ros2_typed_idl_cutover_report.py"
    template = "deployment/ros2_typed_idl_cutover.template.json"
    report = "test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json"

    assert tool in README.read_text(encoding="utf-8")
    assert template in README.read_text(encoding="utf-8")
    assert tool in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert template in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert tool in MIGRATION_RUNBOOK.read_text(encoding="utf-8")
    assert report in MIGRATION_RUNBOOK.read_text(encoding="utf-8")
