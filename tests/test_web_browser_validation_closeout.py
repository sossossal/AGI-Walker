from __future__ import annotations

import json
from pathlib import Path

from tools.build_web_browser_validation_closeout import main


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def _manual_report(status: str = "passed") -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "status": status,
        "summary": {"session_id": "browser-validation-20260426"},
    }


def _playwright_report(status: str = "passed") -> dict[str, object]:
    failed_page_count = 0 if status == "passed" else 3
    return {
        "schema_version": "1.0",
        "status": status,
        "summary": {"failed_page_count": failed_page_count},
    }


def test_browser_validation_closeout_passes_with_manual_and_playwright(
    tmp_path: Path,
) -> None:
    manual = tmp_path / "manual.json"
    playwright = tmp_path / "playwright.json"
    output = tmp_path / "closeout.json"
    _write_json(manual, _manual_report())
    _write_json(playwright, _playwright_report())

    exit_code = main(
        [
            "--manual-report",
            manual.name,
            "--playwright-report",
            playwright.name,
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["warnings"] == []
    assert payload["summary"]["manual_session_id"] == "browser-validation-20260426"


def test_browser_validation_closeout_warns_on_blocked_playwright(
    tmp_path: Path,
) -> None:
    manual = tmp_path / "manual.json"
    playwright = tmp_path / "playwright.json"
    output = tmp_path / "closeout.json"
    _write_json(manual, _manual_report())
    _write_json(playwright, _playwright_report("blocked"))

    exit_code = main(
        [
            "--manual-report",
            manual.name,
            "--playwright-report",
            playwright.name,
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["warnings"] == ["playwright_report_not_passed"]
    assert payload["summary"]["playwright_failed_page_count"] == 3


def test_browser_validation_closeout_blocks_without_manual_report(
    tmp_path: Path,
) -> None:
    output = tmp_path / "closeout.json"

    exit_code = main(
        [
            "--manual-report",
            "missing-manual.json",
            "--playwright-report",
            "missing-playwright.json",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["manual_report_missing"]
    assert payload["warnings"] == ["playwright_report_missing"]


def test_browser_validation_closeout_blocks_unsafe_source_path(
    tmp_path: Path,
) -> None:
    output = tmp_path / "closeout.json"

    exit_code = main(
        [
            "--manual-report",
            "../manual.json",
            "--playwright-report",
            "missing-playwright.json",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "source_path_validation" in payload["blockers"]
    assert payload["source_path_statuses"]["manual_report"]["path_valid"] is False
    assert payload["source_path_blockers"] == [
        {"field": "manual_report", "reason": "parent_directory"}
    ]
