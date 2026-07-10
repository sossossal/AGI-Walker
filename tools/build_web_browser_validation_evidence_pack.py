from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_MANUAL_REPORT = (
    "test_env/web_browser_manual_validation/web_browser_manual_validation_report.json"
)
DEFAULT_CLOSEOUT = "test_env/web_browser_manual_validation/web_browser_validation_closeout.json"
DEFAULT_PLAYWRIGHT_REPORT = (
    "test_env/web_browser_manual_validation/playwright_smoke_report.json"
)
DEFAULT_OUTPUT = "test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json"
SCHEMA_VERSION = "1.0"
SOURCE_PATH_FIELDS = ("manual_report", "closeout", "playwright_report")


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a release evidence pack for Web browser validation artifacts."
    )
    parser.add_argument("--manual-report", default=DEFAULT_MANUAL_REPORT)
    parser.add_argument("--closeout", default=DEFAULT_CLOSEOUT)
    parser.add_argument("--playwright-report", default=DEFAULT_PLAYWRIGHT_REPORT)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--require-playwright", action="store_true")
    return parser.parse_args(argv)


def _text(value: Any) -> str:
    return value.strip() if isinstance(value, str) else ""


def _resolve_relative_path(value: str | Path | None, *, base_dir: Path) -> tuple[bool, Path | None, str | None]:
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


def _status(payload: dict[str, Any] | None) -> Any:
    if payload is None:
        return None
    return payload.get("status") or payload.get("summary", {}).get("status")


def _manual_evidence_counts(manual_report: dict[str, Any] | None) -> dict[str, Any]:
    if manual_report is None:
        return {"screenshot_count": 0, "export_count": 0, "notes_present": False}
    summary = manual_report.get("summary")
    if isinstance(summary, dict):
        return {
            "screenshot_count": summary.get("screenshot_count", 0),
            "export_count": summary.get("export_count", 0),
            "notes_present": summary.get("notes_present", False),
        }
    return {"screenshot_count": 0, "export_count": 0, "notes_present": False}


def build_web_browser_validation_evidence_pack(
    *,
    manual_report: dict[str, Any] | None,
    closeout: dict[str, Any] | None,
    playwright_report: dict[str, Any] | None,
    sources: dict[str, str],
    source_path_statuses: dict[str, dict[str, Any]],
    source_path_blockers: list[dict[str, str]],
    require_playwright: bool = False,
) -> dict[str, Any]:
    blockers: list[str] = []
    warnings: list[str] = []
    manual_status = _status(manual_report)
    closeout_status = _status(closeout)
    playwright_status = _status(playwright_report)
    if manual_status != "passed":
        blockers.append("manual_report")
    if closeout_status != "passed":
        blockers.append("validation_closeout")
    if require_playwright and playwright_status != "passed":
        blockers.append("playwright_report")
    elif playwright_status != "passed":
        warnings.append("playwright_report")

    evidence_counts = _manual_evidence_counts(manual_report)
    if evidence_counts["screenshot_count"] <= 0:
        blockers.append("screenshots")
    if evidence_counts["export_count"] <= 0:
        blockers.append("exports")
    if evidence_counts["notes_present"] is not True:
        blockers.append("console_error_summary")
    if source_path_blockers:
        blockers.append("source_path_validation")

    status = "ready" if not blockers else "blocked"
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "sources": sources,
        "source_path_statuses": source_path_statuses,
        "source_path_blockers": source_path_blockers,
        "summary": {
            "manual_report_status": manual_status,
            "closeout_status": closeout_status,
            "playwright_status": playwright_status,
            "require_playwright": require_playwright,
            **evidence_counts,
        },
        "blockers": sorted(set(blockers)),
        "warnings": sorted(set(warnings)),
        "next_actions": _next_actions(status=status, blockers=blockers, warnings=warnings),
    }


def _next_actions(
    *, status: str, blockers: list[str], warnings: list[str]
) -> list[str]:
    if status == "blocked":
        return [f"Resolve Web browser evidence pack blockers: {', '.join(sorted(set(blockers)))}."]
    if warnings:
        return ["Web browser evidence pack is ready; review optional Playwright warning."]
    return ["Web browser evidence pack is ready for release archive."]


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    output_dir = Path(args.output).resolve().parent
    source_path_statuses = {
        "manual_report": _path_status(args.manual_report, base_dir=output_dir),
        "closeout": _path_status(args.closeout, base_dir=output_dir),
        "playwright_report": _path_status(args.playwright_report, base_dir=output_dir),
    }
    source_path_blockers = _source_path_blockers(source_path_statuses)
    sources = {
        "manual_report": args.manual_report,
        "closeout": args.closeout,
        "playwright_report": args.playwright_report,
    }
    pack = build_web_browser_validation_evidence_pack(
        manual_report=_load_json_if_exists(source_path_statuses["manual_report"]),
        closeout=_load_json_if_exists(source_path_statuses["closeout"]),
        playwright_report=_load_json_if_exists(source_path_statuses["playwright_report"]),
        sources=sources,
        source_path_statuses=source_path_statuses,
        source_path_blockers=source_path_blockers,
        require_playwright=args.require_playwright,
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(pack, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return 0 if pack["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
