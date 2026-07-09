import json
import subprocess
import sys
from pathlib import Path

from tools.build_next_stage_external_evidence_checklist import (
    build_next_stage_external_evidence_checklist,
)
from tools.build_next_stage_external_evidence_status_report import (
    SCHEMA_VERSION,
    build_next_stage_external_evidence_status_report,
    main,
    validate_next_stage_external_evidence_status_report,
)
from tools.build_next_stage_readiness_report import build_next_stage_readiness_report


def test_next_stage_external_evidence_status_report_tracks_current_blockers() -> None:
    checklist = build_next_stage_external_evidence_checklist(
        build_next_stage_readiness_report()
    )

    report = build_next_stage_external_evidence_status_report(checklist)

    assert report["schema_version"] == SCHEMA_VERSION
    assert report["status"] == "blocked"
    assert report["checklist_status"] == checklist["status"]
    assert report["checklist_validation_errors"] == []
    assert report["validation_errors"] == []
    assert report["summary"]["item_count"] == len(checklist["items"])
    assert report["summary"]["blocked_item_count"] == len(checklist["items"])
    assert report["summary"]["ready_item_count"] == 0
    assert report["blocked_items"] == checklist["unresolved_items"]
    first = report["items"][0]
    assert first["artifact_id"] == checklist["items"][0]["artifact_id"]
    assert first["artifact_path"] == checklist["items"][0]["artifact_path"]
    assert first["target_status"] == checklist["items"][0]["target_status"]
    assert first["ready"] is False
    assert first["remaining_issues"] == checklist["items"][0]["issues"]


def test_next_stage_external_evidence_status_report_marks_ready_artifact(
    tmp_path: Path, monkeypatch,
) -> None:
    artifact = tmp_path / "ready_artifact.json"
    artifact.write_text(json.dumps({"status": "ready"}), encoding="utf-8")
    checklist = {
        "schema_version": "next_stage_external_evidence_checklist.v1",
        "generated_at": "2026-07-09T00:00:00+00:00",
        "status": "blocked",
        "summary": {
            "item_count": 1,
            "unresolved_item_count": 1,
            "external_input_item_count": 1,
            "code_or_config_item_count": 0,
            "non_external_item_count": 0,
            "readiness_validation_error_count": 0,
            "handoff_validation_error_count": 0,
            "execution_prerequisite_validation_error_count": 0,
        },
        "unresolved_items": ["sample"],
        "non_external_items": [],
        "items": [
            {
                "artifact_id": "sample",
                "artifact_path": "ready_artifact.json",
                "target_status": "ready",
                "execution_scope": "external_input",
                "requires_real_input": True,
                "issue_count": 1,
                "issues": ["evidence_missing"],
            }
        ],
        "readiness_validation_errors": [],
        "handoff_validation_errors": [],
        "execution_prerequisite_validation_errors": [],
    }
    monkeypatch.setattr(
        "tools.build_next_stage_external_evidence_status_report.PROJECT_ROOT",
        tmp_path,
    )

    report = build_next_stage_external_evidence_status_report(checklist)

    assert report["status"] == "ready"
    assert report["summary"]["ready_item_count"] == 1
    assert report["summary"]["blocked_item_count"] == 0
    assert report["blocked_items"] == []
    assert report["items"][0]["ready"] is True
    assert report["items"][0]["remaining_issues"] == []


def test_next_stage_external_evidence_status_report_validates_shape() -> None:
    checklist = build_next_stage_external_evidence_checklist(
        build_next_stage_readiness_report()
    )
    report = build_next_stage_external_evidence_status_report(checklist)
    report["summary"]["blocked_item_count"] += 1
    report["blocked_items"] = []
    report["status"] = "ready"

    errors = validate_next_stage_external_evidence_status_report(report)

    assert "summary.blocked_item_count must equal 0" in errors
    assert "blocked_items must match non-ready items" in errors
    assert "status must be blocked" in errors


def test_next_stage_external_evidence_status_report_rejects_non_repo_paths() -> None:
    checklist = build_next_stage_external_evidence_checklist(
        build_next_stage_readiness_report()
    )
    checklist["items"][0]["artifact_path"] = "../outside.json"
    checklist["items"][1]["artifact_path"] = str(Path.cwd() / "outside.json")

    report = build_next_stage_external_evidence_status_report(checklist)

    assert report["status"] == "blocked"
    assert report["items"][0]["artifact_path_valid"] is False
    assert report["items"][1]["artifact_path_valid"] is False
    assert (
        "items must use repository-relative artifact_path values: "
        "hardware_live_closeout, ros2_typed_idl_cutover"
    ) in report["validation_errors"]


def test_next_stage_external_evidence_status_report_cli_writes_blocked_snapshot(
    tmp_path: Path, capsys,
) -> None:
    checklist = build_next_stage_external_evidence_checklist(
        build_next_stage_readiness_report()
    )
    checklist_path = tmp_path / "checklist.json"
    output = tmp_path / "status_report.json"
    checklist_path.write_text(json.dumps(checklist), encoding="utf-8")

    exit_code = main(
        [
            "--checklist",
            str(checklist_path),
            "--output",
            str(output),
            "--expected-status",
            "blocked",
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["status"] == "blocked"
    assert payload["validation_errors"] == []
    stdout = capsys.readouterr().out
    assert "next_stage_external_evidence_status_report_status=blocked" in stdout
    assert "validation_errors:0" in stdout


def test_next_stage_external_evidence_status_report_script_runs_directly(
    tmp_path: Path,
) -> None:
    checklist = build_next_stage_external_evidence_checklist(
        build_next_stage_readiness_report()
    )
    checklist_path = tmp_path / "checklist.json"
    output = tmp_path / "status_report.json"
    checklist_path.write_text(json.dumps(checklist), encoding="utf-8")

    result = subprocess.run(
        [
            sys.executable,
            "tools/build_next_stage_external_evidence_status_report.py",
            "--checklist",
            str(checklist_path),
            "--output",
            str(output),
            "--expected-status",
            "blocked",
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    assert result.returncode == 0
    assert output.is_file()
    assert "next_stage_external_evidence_status_report_status=blocked" in result.stdout
