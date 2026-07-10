from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANUAL_REPORT = (
    "test_env/web_browser_manual_validation/web_browser_manual_validation_report.json"
)
DEFAULT_PLAYWRIGHT_REPORT = (
    "test_env/web_browser_manual_validation/playwright_smoke_report.json"
)
DEFAULT_OUTPUT = "test_env/web_browser_manual_validation/web_browser_validation_closeout.json"
SCHEMA_VERSION = "1.0"
SOURCE_PATH_FIELDS = ("manual_report", "playwright_report")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a closeout verdict for Web browser validation evidence."
    )
    parser.add_argument("--manual-report", default=DEFAULT_MANUAL_REPORT)
    parser.add_argument("--playwright-report", default=DEFAULT_PLAYWRIGHT_REPORT)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _resolve_relative_path(value: str | Path, *, base_dir: Path) -> tuple[bool, Path | None, str | None]:
    text = _text(str(value))
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


def _path_status(value: str | Path, *, base_dir: Path) -> dict[str, Any]:
    valid, resolved_path, error = _resolve_relative_path(value, base_dir=base_dir)
    return {
        "path": _text(str(value)),
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


def build_web_browser_validation_closeout(
    *,
    manual_report: dict[str, Any] | None,
    playwright_report: dict[str, Any] | None,
    manual_report_path: str,
    playwright_report_path: str,
    source_path_statuses: dict[str, dict[str, Any]],
    source_path_blockers: list[dict[str, str]],
) -> dict[str, Any]:
    blockers: list[str] = []
    warnings: list[str] = []

    manual_status = None if manual_report is None else manual_report.get("status")
    playwright_status = (
        None if playwright_report is None else playwright_report.get("status")
    )

    if manual_report is None:
        blockers.append("manual_report_missing")
    elif manual_report.get("schema_version") != SCHEMA_VERSION:
        blockers.append("manual_report_schema_invalid")
    elif manual_status != "passed":
        blockers.append("manual_report_not_passed")

    if playwright_report is None:
        warnings.append("playwright_report_missing")
    elif playwright_report.get("schema_version") != SCHEMA_VERSION:
        warnings.append("playwright_report_schema_invalid")
    elif playwright_status != "passed":
        warnings.append("playwright_report_not_passed")

    if source_path_blockers:
        blockers.append("source_path_validation")

    status = "passed" if not blockers else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "blockers": blockers,
        "warnings": warnings,
        "source_path_statuses": source_path_statuses,
        "source_path_blockers": source_path_blockers,
        "sources": {
            "manual_report": manual_report_path,
            "playwright_report": playwright_report_path,
        },
        "summary": {
            "manual_report_status": manual_status,
            "playwright_report_status": playwright_status,
            "manual_report_present": manual_report is not None,
            "playwright_report_present": playwright_report is not None,
            "manual_session_id": None
            if manual_report is None
            else manual_report.get("summary", {}).get("session_id"),
            "playwright_failed_page_count": None
            if playwright_report is None
            else playwright_report.get("summary", {}).get("failed_page_count"),
        },
        "next_actions": _next_actions(status=status, blockers=blockers, warnings=warnings),
    }


def _next_actions(
    *, status: str, blockers: list[str], warnings: list[str]
) -> list[str]:
    if status == "blocked":
        return [f"Resolve browser validation blockers: {', '.join(blockers)}."]
    if warnings:
        return [
            "Browser manual validation is closed; review optional Playwright warnings before final release evidence archival."
        ]
    return ["Browser validation is closed and ready for release evidence archival."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output_dir = Path(args.output).resolve().parent
    source_path_statuses = {
        "manual_report": _path_status(args.manual_report, base_dir=output_dir),
        "playwright_report": _path_status(args.playwright_report, base_dir=output_dir),
    }
    source_path_blockers = _source_path_blockers(source_path_statuses)
    closeout = build_web_browser_validation_closeout(
        manual_report=_load_json_if_exists(source_path_statuses["manual_report"]),
        playwright_report=_load_json_if_exists(source_path_statuses["playwright_report"]),
        manual_report_path=args.manual_report,
        playwright_report_path=args.playwright_report,
        source_path_statuses=source_path_statuses,
        source_path_blockers=source_path_blockers,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(closeout, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if closeout["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
