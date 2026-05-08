from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

DEFAULT_INPUT = "deployment/web_browser_manual_validation.template.json"
DEFAULT_OUTPUT = "test_env/web_browser_manual_validation/web_browser_manual_validation_report.json"
SCHEMA_VERSION = "1.0"
REQUIRED_SECTIONS = {
    "instruction_console": [
        "instruction_set_sent",
        "simulated_circuit_sent",
        "recovery_plan_built",
        "recover_cancel_confirmed",
        "recover_executed",
        "clear_faults_executed",
        "node_status_table_checked",
        "failure_drilldown_checked",
    ],
    "operator_history": [
        "session_search_checked",
        "operator_filter_checked",
        "tag_filter_checked",
        "note_filter_checked",
        "note_exact_checked",
        "sort_checked",
        "json_export_checked",
        "csv_export_checked",
        "replay_checked",
    ],
    "operator_timeline": [
        "session_search_checked",
        "filters_checked",
        "time_range_checked",
        "compare_checked",
        "clear_compare_checked",
        "json_export_checked",
        "csv_export_checked",
    ],
    "responsive": [
        "desktop_layout_checked",
        "narrow_layout_checked",
        "page_refresh_checked",
        "console_errors_checked",
    ],
}


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a machine-checkable report for Web browser manual validation evidence."
    )
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _load_json(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _is_non_empty_text(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _section(payload: dict[str, Any], section_name: str) -> dict[str, Any]:
    value = payload.get(section_name)
    return value if isinstance(value, dict) else {}


def build_web_browser_manual_validation_report(
    *, payload: dict[str, Any], input_path: str
) -> dict[str, Any]:
    blockers: list[str] = []
    warnings: list[str] = []
    section_reports: dict[str, dict[str, Any]] = {}

    if payload.get("schema_version") != SCHEMA_VERSION:
        blockers.append("schema_version_invalid")
    if not _is_non_empty_text(payload.get("browser_name")):
        blockers.append("browser_name_missing")
    if not _is_non_empty_text(payload.get("browser_version")):
        blockers.append("browser_version_missing")
    if not _is_non_empty_text(payload.get("web_panel_start_command")):
        blockers.append("web_panel_start_command_missing")
    if not _is_non_empty_text(payload.get("session_id")):
        blockers.append("session_id_missing")
    if payload.get("live_hardware_used") is True:
        warnings.append("live_hardware_used_requires_separate_site_approval")

    for section_name, required_steps in REQUIRED_SECTIONS.items():
        section_payload = _section(payload, section_name)
        missing_steps = [
            step_name for step_name in required_steps if section_payload.get(step_name) is not True
        ]
        if missing_steps:
            blockers.append(f"{section_name}_incomplete")
        section_reports[section_name] = {
            "status": "passed" if not missing_steps else "blocked",
            "required_step_count": len(required_steps),
            "passed_step_count": len(required_steps) - len(missing_steps),
            "missing_steps": missing_steps,
        }

    evidence = payload.get("evidence")
    if not isinstance(evidence, dict):
        blockers.append("evidence_missing")
        evidence_summary = {"screenshot_count": 0, "export_count": 0, "notes_present": False}
    else:
        screenshots = evidence.get("screenshots") if isinstance(evidence.get("screenshots"), list) else []
        exports = evidence.get("exports") if isinstance(evidence.get("exports"), list) else []
        notes_present = _is_non_empty_text(evidence.get("console_error_summary"))
        if not screenshots:
            blockers.append("evidence_screenshots_missing")
        if not exports:
            blockers.append("evidence_exports_missing")
        if not notes_present:
            blockers.append("evidence_console_summary_missing")
        evidence_summary = {
            "screenshot_count": len(screenshots),
            "export_count": len(exports),
            "notes_present": notes_present,
        }

    status = "passed" if not blockers else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "blockers": sorted(set(blockers)),
        "warnings": warnings,
        "source": input_path,
        "summary": {
            "browser_name": payload.get("browser_name"),
            "browser_version": payload.get("browser_version"),
            "session_id": payload.get("session_id"),
            "live_hardware_used": payload.get("live_hardware_used", False),
            "section_count": len(REQUIRED_SECTIONS),
            "blocked_section_count": sum(
                1 for report in section_reports.values() if report["status"] == "blocked"
            ),
            **evidence_summary,
        },
        "sections": section_reports,
        "next_actions": _next_actions(status=status, blockers=blockers),
    }


def _next_actions(*, status: str, blockers: list[str]) -> list[str]:
    if status == "passed":
        return ["Archive this browser validation report with Web Panel release evidence."]
    return [f"Complete browser validation blockers: {', '.join(sorted(set(blockers)))}."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    report = build_web_browser_manual_validation_report(
        payload=_load_json(args.input),
        input_path=args.input,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
