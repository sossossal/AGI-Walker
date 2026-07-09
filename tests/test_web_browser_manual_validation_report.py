from __future__ import annotations

import json
from pathlib import Path

from tools.build_web_browser_manual_validation_report import main


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def _completed_payload() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "browser_name": "Chromium",
        "browser_version": "124.0.0",
        "web_panel_start_command": "python -m web_panel.server",
        "session_id": "browser-validation-20260426",
        "live_hardware_used": False,
        "instruction_console": {
            "instruction_set_sent": True,
            "simulated_circuit_sent": True,
            "recovery_plan_built": True,
            "recover_cancel_confirmed": True,
            "recover_executed": True,
            "clear_faults_executed": True,
            "node_status_table_checked": True,
            "failure_drilldown_checked": True,
        },
        "operator_history": {
            "session_search_checked": True,
            "operator_filter_checked": True,
            "tag_filter_checked": True,
            "note_filter_checked": True,
            "note_exact_checked": True,
            "sort_checked": True,
            "json_export_checked": True,
            "csv_export_checked": True,
            "replay_checked": True,
        },
        "operator_timeline": {
            "session_search_checked": True,
            "filters_checked": True,
            "time_range_checked": True,
            "compare_checked": True,
            "clear_compare_checked": True,
            "json_export_checked": True,
            "csv_export_checked": True,
        },
        "responsive": {
            "desktop_layout_checked": True,
            "narrow_layout_checked": True,
            "page_refresh_checked": True,
            "console_errors_checked": True,
        },
        "evidence": {
            "screenshots": ["instruction-console.png"],
            "exports": ["operator-history.json"],
            "console_error_summary": "No blocking JavaScript errors.",
        },
    }


def test_web_browser_manual_validation_report_passes_completed_payload(
    tmp_path: Path,
) -> None:
    input_path = tmp_path / "browser_validation.json"
    output_path = tmp_path / "report.json"
    (tmp_path / "instruction-console.png").write_text("png", encoding="utf-8")
    (tmp_path / "operator-history.json").write_text("{}", encoding="utf-8")
    _write_json(input_path, _completed_payload())

    exit_code = main(["--input", str(input_path), "--output", str(output_path)])

    assert exit_code == 0
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["summary"]["blocked_section_count"] == 0
    assert payload["summary"]["screenshot_count"] == 1
    assert payload["summary"]["screenshot_missing_files"] == []
    assert payload["summary"]["export_missing_files"] == []
    assert payload["next_actions"] == [
        "Archive this browser validation report with Web Panel release evidence."
    ]


def test_web_browser_manual_validation_report_blocks_incomplete_payload(
    tmp_path: Path,
) -> None:
    input_path = tmp_path / "browser_validation.json"
    output_path = tmp_path / "report.json"
    payload = _completed_payload()
    payload["browser_version"] = ""
    payload["instruction_console"] = {"instruction_set_sent": True}
    payload["evidence"] = {"screenshots": [], "exports": [], "console_error_summary": ""}
    _write_json(input_path, payload)

    exit_code = main(["--input", str(input_path), "--output", str(output_path)])

    assert exit_code == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["status"] == "blocked"
    assert "browser_version_missing" in report["blockers"]
    assert "instruction_console_incomplete" in report["blockers"]
    assert "evidence_screenshots_missing" in report["blockers"]
    assert "evidence_exports_missing" in report["blockers"]
    assert report["sections"]["instruction_console"]["missing_steps"] == [
        "simulated_circuit_sent",
        "recovery_plan_built",
        "recover_cancel_confirmed",
        "recover_executed",
        "clear_faults_executed",
        "node_status_table_checked",
        "failure_drilldown_checked",
    ]


def test_web_browser_manual_validation_report_blocks_bad_evidence_paths(
    tmp_path: Path,
) -> None:
    input_path = tmp_path / "browser_validation.json"
    output_path = tmp_path / "report.json"
    payload = _completed_payload()
    payload["evidence"] = {
        "screenshots": ["../outside.png", "missing-screenshot.png"],
        "exports": [str(tmp_path / "absolute-export.json"), "missing-export.json"],
        "console_error_summary": "No blocking JavaScript errors.",
    }
    _write_json(input_path, payload)

    exit_code = main(["--input", str(input_path), "--output", str(output_path)])

    assert exit_code == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["status"] == "blocked"
    assert "evidence_screenshot_paths_invalid" in report["blockers"]
    assert "evidence_screenshot_files_missing" in report["blockers"]
    assert "evidence_export_paths_invalid" in report["blockers"]
    assert "evidence_export_files_missing" in report["blockers"]
    assert report["summary"]["screenshot_invalid_paths"] == ["../outside.png"]
    assert report["summary"]["screenshot_missing_files"] == ["missing-screenshot.png"]
    assert report["summary"]["export_invalid_paths"] == [
        str(tmp_path / "absolute-export.json")
    ]
    assert report["summary"]["export_missing_files"] == ["missing-export.json"]
