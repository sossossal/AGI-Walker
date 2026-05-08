from __future__ import annotations

import json
from pathlib import Path

from tools.run_web_browser_playwright_smoke import (
    build_playwright_smoke_report,
    main,
)


def test_playwright_smoke_report_passes_all_pages() -> None:
    report = build_playwright_smoke_report(
        base_url="http://127.0.0.1:8000",
        page_results=[
            {
                "path": "/static/instruction-control.html",
                "status": "passed",
                "missing_selectors": [],
            },
            {
                "path": "/static/operator-history.html",
                "status": "passed",
                "missing_selectors": [],
            },
        ],
    )

    assert report["status"] == "passed"
    assert report["summary"]["passed_page_count"] == 2
    assert report["blockers"] == []


def test_playwright_smoke_report_blocks_failed_pages() -> None:
    report = build_playwright_smoke_report(
        base_url="http://127.0.0.1:8000",
        page_results=[
            {
                "path": "/static/instruction-control.html",
                "status": "blocked",
                "missing_selectors": ["#recover-faults-button"],
            }
        ],
    )

    assert report["status"] == "blocked"
    assert report["blockers"] == ["page_selector_check_failed"]
    assert report["summary"]["failed_page_count"] == 1


def test_playwright_smoke_writes_blocked_report_when_dependency_missing(
    tmp_path: Path,
    monkeypatch,
) -> None:
    output = tmp_path / "playwright_smoke.json"
    monkeypatch.setattr(
        "tools.run_web_browser_playwright_smoke.importlib.util.find_spec",
        lambda name: None if name == "playwright" else object(),
    )

    exit_code = main(["--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["playwright_missing"]
    assert payload["summary"]["failed_page_count"] == 3


def test_playwright_smoke_writes_blocked_report_when_runtime_crashes(
    tmp_path: Path,
    monkeypatch,
) -> None:
    output = tmp_path / "playwright_smoke.json"
    monkeypatch.setattr(
        "tools.run_web_browser_playwright_smoke.importlib.util.find_spec",
        lambda name: object(),
    )

    class Completed:
        returncode = -1073741819
        stderr = "greenlet runtime crash"

    monkeypatch.setattr(
        "tools.run_web_browser_playwright_smoke.subprocess.run",
        lambda *args, **kwargs: Completed(),
    )

    exit_code = main(["--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["playwright_runtime_failed"]
    assert "greenlet runtime crash" in payload["detail"]
