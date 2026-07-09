import json
import subprocess
import sys
from pathlib import Path

from tools.build_next_stage_external_evidence_checklist import (
    EXECUTION_PREREQUISITES,
    SCHEMA_VERSION,
    build_next_stage_external_evidence_checklist,
    main,
    validate_execution_prerequisites,
    validate_next_stage_external_evidence_checklist,
    validate_next_stage_external_evidence_checklist_handoff,
)
from tools.build_next_stage_readiness_report import build_next_stage_readiness_report


def test_next_stage_external_evidence_checklist_tracks_current_blockers() -> None:
    readiness = build_next_stage_readiness_report()

    checklist = build_next_stage_external_evidence_checklist(readiness)

    assert checklist["schema_version"] == SCHEMA_VERSION
    assert checklist["status"] == "blocked"
    assert checklist["readiness_status"] == readiness["status"]
    assert checklist["execution_prerequisites"] == EXECUTION_PREREQUISITES
    assert checklist["execution_prerequisites"]["python"]["recommended"] == "3.12"
    assert "placeholders" in checklist["execution_prerequisites"]["evidence_policy"]
    assert checklist["readiness_validation_errors"] == []
    assert checklist["handoff_validation_errors"] == []
    assert checklist["validation_errors"] == []
    assert checklist["summary"]["item_count"] == len(readiness["blockers"])
    assert checklist["summary"]["unresolved_item_count"] == len(readiness["blockers"])
    assert checklist["summary"]["code_or_config_item_count"] == 0
    assert checklist["summary"]["external_input_item_count"] == len(readiness["blockers"])
    assert checklist["summary"]["handoff_validation_error_count"] == 0
    assert checklist["summary"]["execution_prerequisite_validation_error_count"] == 0
    assert checklist["unresolved_items"] == readiness["blockers"]
    assert checklist["non_external_items"] == []
    hardware = checklist["items"][0]
    assert hardware["artifact_id"] == "hardware_live_closeout"
    assert hardware["requires_real_input"] is True
    assert hardware["target_status"] == "ready"
    assert hardware["issues"]
    assert hardware["acceptance_evidence"].endswith("reaches target status ready")
    assert hardware["evidence_commands"] == [
        "python tools/build_hardware_live_closeout_report.py --output test_env/hardware_live/hardware_live_closeout_report.json"
    ]
    assert "deployment/hardware/imc22_live_transport.template.json" in hardware["input_templates"]
    assert "docs/hardware/HARDWARE_INTEGRATION_GUIDE.md" in hardware["guide_paths"]


def test_next_stage_external_evidence_checklist_includes_browser_handoff() -> None:
    readiness = build_next_stage_readiness_report()

    checklist = build_next_stage_external_evidence_checklist(readiness)

    browser = next(
        item
        for item in checklist["items"]
        if item["artifact_id"] == "web_browser_evidence_pack"
    )
    assert browser["evidence_commands"] == [
        "python tools/build_web_browser_manual_validation_report.py --output test_env/web_browser_manual_validation/web_browser_manual_validation_report.json",
        "python tools/build_web_browser_validation_closeout.py --output test_env/web_browser_manual_validation/web_browser_validation_closeout.json",
        "python tools/build_web_browser_validation_evidence_pack.py --output test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json",
    ]
    assert browser["input_templates"] == [
        "deployment/web_browser_manual_validation.template.json"
    ]
    assert browser["guide_paths"] == [
        "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md",
        "docs/guides/WEB_PANEL_GUIDE.md",
    ]


def test_next_stage_external_evidence_checklist_passes_ready_readiness() -> None:
    readiness = build_next_stage_readiness_report()
    readiness["status"] = "ready"
    readiness["blockers"] = []
    readiness["blocker_details"] = []
    readiness["action_plan"] = []
    readiness["artifacts"] = [
        {**artifact, "status": "ready", "actual_status": artifact["expected_statuses"][0]}
        for artifact in readiness["artifacts"]
    ]
    readiness["summary"].update(
        {
            "ready_artifact_count": len(readiness["artifacts"]),
            "blocked_artifact_count": 0,
            "blocker_detail_count": 0,
            "external_input_action_count": 0,
            "code_or_config_action_count": 0,
        }
    )
    readiness["validation_errors"] = []

    checklist = build_next_stage_external_evidence_checklist(readiness)

    assert checklist["status"] == "ready"
    assert checklist["summary"]["item_count"] == 0
    assert checklist["next_actions"] == ["All next-stage external evidence items are ready."]


def test_next_stage_external_evidence_checklist_blocks_invalid_readiness() -> None:
    readiness = build_next_stage_readiness_report()
    readiness["summary"]["artifact_count"] += 1

    checklist = build_next_stage_external_evidence_checklist(readiness)

    assert checklist["status"] == "blocked"
    assert checklist["summary"]["readiness_validation_error_count"] == 1
    assert checklist["readiness_validation_errors"]
    assert checklist["next_actions"] == [
        "Fix next-stage readiness report validation errors before acting."
    ]


def test_next_stage_external_evidence_checklist_validates_artifact_shape() -> None:
    readiness = build_next_stage_readiness_report()
    checklist = build_next_stage_external_evidence_checklist(readiness)
    checklist["summary"]["unresolved_item_count"] += 1
    checklist["status"] = "ready"
    checklist["unresolved_items"] = []

    errors = validate_next_stage_external_evidence_checklist(checklist)

    assert "summary.unresolved_item_count must equal 0" in errors
    assert "unresolved_items must match items with issue_count > 0" in errors
    assert "status must be blocked" in errors


def test_next_stage_external_evidence_checklist_validates_item_shape() -> None:
    readiness = build_next_stage_readiness_report()
    checklist = build_next_stage_external_evidence_checklist(readiness)
    item = checklist["items"][0]
    item["artifact_id"] = ""
    item["artifact_path"] = ""
    item["actual_status"] = 123
    item["target_status"] = ""
    item["execution_scope"] = "live_magic"
    item["requires_real_input"] = "yes"
    item["issue_count"] = "bad"
    item["issues"] = ["evidence_missing", 42]
    item["warnings"] = "none"
    item["primary_next_action"] = ""
    item["next_actions"] = [None]
    item["acceptance_evidence"] = ""
    item["evidence_commands"] = "python tool.py"
    item["input_templates"] = [None]
    item["guide_paths"] = [123]

    errors = validate_next_stage_external_evidence_checklist(checklist)

    assert "items[0].artifact_id must be a non-empty string" in errors
    assert "items[0].artifact_path must be a non-empty string" in errors
    assert "items[0].actual_status must be a string or null" in errors
    assert "items[0].target_status must be a non-empty string" in errors
    assert "items[0].execution_scope must be a known scope" in errors
    assert "items[0].requires_real_input must be a boolean" in errors
    assert "items[0].issue_count must be a non-negative integer" in errors
    assert "items[0].issues entries must be strings" in errors
    assert "items[0].warnings must be a list" in errors
    assert "items[0].primary_next_action must be a non-empty string" in errors
    assert "items[0].next_actions entries must be strings" in errors
    assert "items[0].acceptance_evidence must be a non-empty string" in errors
    assert "items[0].evidence_commands must be a list" in errors
    assert "items[0].input_templates entries must be strings" in errors
    assert "items[0].guide_paths entries must be strings" in errors


def test_next_stage_external_evidence_checklist_rejects_non_repo_artifact_path() -> None:
    readiness = build_next_stage_readiness_report()
    checklist = build_next_stage_external_evidence_checklist(readiness)
    checklist["items"][0]["artifact_path"] = "../outside.json"
    checklist["items"][1]["artifact_path"] = str(Path.cwd() / "outside.json")

    errors = validate_next_stage_external_evidence_checklist(checklist)

    assert "hardware_live_closeout.artifact_path must be a repository-relative path" in errors
    assert (
        "ros2_typed_idl_cutover.artifact_path must be a repository-relative path"
        in errors
    )


def test_next_stage_external_evidence_checklist_validates_handoff_paths() -> None:
    readiness = build_next_stage_readiness_report()
    checklist = build_next_stage_external_evidence_checklist(readiness)
    items = [dict(item) for item in checklist["items"]]
    items[0]["input_templates"] = ["deployment/missing-template.json"]
    items[1]["guide_paths"] = ["../outside.md"]
    items[2]["evidence_commands"] = []

    errors = validate_next_stage_external_evidence_checklist_handoff(items)

    assert (
        "hardware_live_closeout.input_templates[0] does not exist: "
        "deployment/missing-template.json"
    ) in errors
    assert (
        "ros2_typed_idl_cutover.guide_paths[0] must be a repository-relative path"
        in errors
    )
    assert "operator_delivery_checklist.evidence_commands must not be empty" in errors


def test_next_stage_external_evidence_checklist_blocks_invalid_handoff() -> None:
    readiness = build_next_stage_readiness_report()
    readiness["blocker_details"][0]["id"] = "unknown_external_artifact"
    readiness["blockers"][0] = "unknown_external_artifact"
    readiness["action_plan"][0]["artifact_id"] = "unknown_external_artifact"
    readiness["artifacts"][0]["id"] = "unknown_external_artifact"

    checklist = build_next_stage_external_evidence_checklist(readiness)

    assert checklist["status"] == "blocked"
    assert checklist["summary"]["handoff_validation_error_count"] == 3
    assert checklist["handoff_validation_errors"] == [
        "unknown_external_artifact.evidence_commands must not be empty",
        "unknown_external_artifact.input_templates must not be empty",
        "unknown_external_artifact.guide_paths must not be empty",
    ]
    assert checklist["next_actions"] == [
        "Fix next-stage external evidence checklist handoff validation errors before acting."
    ]


def test_next_stage_external_evidence_checklist_validates_execution_prerequisites() -> None:
    errors = validate_execution_prerequisites(
        {
            "python": {"required": ">=3.10", "recommended": "", "command_policy": "  "},
            "evidence_policy": "",
        }
    )

    assert errors == [
        "execution_prerequisites.python.recommended must be a non-empty string",
        "execution_prerequisites.python.command_policy must be a non-empty string",
        "execution_prerequisites.evidence_policy must be a non-empty string",
    ]


def test_next_stage_external_evidence_checklist_cli_writes_report(
    tmp_path: Path, capsys,
) -> None:
    readiness = build_next_stage_readiness_report()
    readiness_path = tmp_path / "next_stage_readiness_report.json"
    output = tmp_path / "next_stage_external_evidence_checklist.json"
    readiness_path.write_text(json.dumps(readiness), encoding="utf-8")

    exit_code = main(["--readiness-report", str(readiness_path), "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["status"] == "blocked"
    assert payload["handoff_validation_errors"] == []
    assert payload["execution_prerequisite_validation_errors"] == []
    assert payload["validation_errors"] == []
    stdout = capsys.readouterr().out
    assert "next_stage_external_evidence_checklist_written=" in stdout
    assert "next_stage_external_evidence_checklist_status=blocked" in stdout
    assert "next_stage_external_evidence_checklist_items=unresolved:" in stdout
    assert "handoff_errors:0" in stdout
    assert "prerequisite_errors:0" in stdout
    assert "validation_errors:0" in stdout


def test_next_stage_external_evidence_checklist_script_runs_directly(
    tmp_path: Path,
) -> None:
    readiness = build_next_stage_readiness_report()
    readiness_path = tmp_path / "next_stage_readiness_report.json"
    output = tmp_path / "next_stage_external_evidence_checklist.json"
    readiness_path.write_text(json.dumps(readiness), encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_next_stage_external_evidence_checklist.py",
            "--readiness-report",
            str(readiness_path),
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 1
    assert output.is_file()
    assert "next_stage_external_evidence_checklist_status=blocked" in result.stdout
