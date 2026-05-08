from __future__ import annotations

import argparse
import importlib.util
import json
import subprocess
import sys
from pathlib import Path
from typing import Any, Sequence

DEFAULT_BASE_URL = "http://127.0.0.1:8000"
DEFAULT_OUTPUT = "test_env/web_browser_manual_validation/playwright_smoke_report.json"
SCHEMA_VERSION = "1.0"

PAGE_CHECKS = [
    {
        "path": "/static/instruction-control.html",
        "selectors": [
            "#build-recovery-plan-button",
            "#recover-faults-button",
            "#clear-faults-button",
            "#hardware-recovery-timeline",
        ],
    },
    {
        "path": "/static/operator-history.html",
        "selectors": [
            "#history-session-query-filter",
            "#history-note-exact-filter",
            "#export-history-json-button",
            "#export-history-csv-button",
        ],
    },
    {
        "path": "/static/operator-history-timeline.html",
        "selectors": [
            "#timeline-note-exact-filter",
            "#timeline-clear-compare-button",
            "#timeline-export-json-button",
            "#timeline-export-csv-button",
        ],
    },
]


def _parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run an optional Playwright browser smoke for Web Panel static pages."
    )
    parser.add_argument("--base-url", default=DEFAULT_BASE_URL)
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout-ms", type=int, default=5000)
    parser.add_argument("--_child-run", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def _write_report(path: str | Path, report: dict[str, Any]) -> None:
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _blocked_report(*, base_url: str, blocker: str, detail: str) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "blocked",
        "blockers": [blocker],
        "base_url": base_url,
        "pages": [],
        "summary": {
            "page_count": len(PAGE_CHECKS),
            "passed_page_count": 0,
            "failed_page_count": len(PAGE_CHECKS),
        },
        "detail": detail,
        "next_actions": [_next_action_for_blocker(blocker)],
    }


def _next_action_for_blocker(blocker: str) -> str:
    if blocker == "playwright_missing":
        return "Install Playwright and browser binaries before running browser smoke."
    if blocker == "playwright_runtime_failed":
        return "Run Playwright smoke in a supported Python/browser runtime and archive the report."
    if blocker == "browser_smoke_failed":
        return "Start Web Panel and rerun the Playwright browser smoke."
    return "Resolve browser smoke blocker and rerun."


def build_playwright_smoke_report(
    *, base_url: str, page_results: list[dict[str, Any]]
) -> dict[str, Any]:
    failed_pages = [
        page for page in page_results if page.get("status") != "passed"
    ]
    status = "passed" if not failed_pages else "blocked"
    blockers = ["page_selector_check_failed"] if failed_pages else []
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "blockers": blockers,
        "base_url": base_url,
        "pages": page_results,
        "summary": {
            "page_count": len(page_results),
            "passed_page_count": len(page_results) - len(failed_pages),
            "failed_page_count": len(failed_pages),
        },
        "next_actions": [
            "Use this smoke report as supporting evidence for browser manual validation."
        ]
        if status == "passed"
        else ["Fix failed page selector checks and rerun Playwright smoke."],
    }


def _run_playwright_smoke(*, base_url: str, timeout_ms: int) -> dict[str, Any]:
    from playwright.sync_api import sync_playwright

    page_results: list[dict[str, Any]] = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        try:
            page = browser.new_page()
            page.set_default_timeout(timeout_ms)
            for page_check in PAGE_CHECKS:
                url = base_url.rstrip("/") + page_check["path"]
                missing_selectors: list[str] = []
                page.goto(url, wait_until="domcontentloaded")
                for selector in page_check["selectors"]:
                    if page.locator(selector).count() == 0:
                        missing_selectors.append(selector)
                page_results.append(
                    {
                        "path": page_check["path"],
                        "url": url,
                        "status": "passed" if not missing_selectors else "blocked",
                        "missing_selectors": missing_selectors,
                    }
                )
        finally:
            browser.close()
    return build_playwright_smoke_report(base_url=base_url, page_results=page_results)


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(argv)
    if importlib.util.find_spec("playwright") is None:
        report = _blocked_report(
            base_url=args.base_url,
            blocker="playwright_missing",
            detail="Python package 'playwright' is not importable.",
        )
        _write_report(args.output, report)
        return 1
    if not args._child_run:
        child_output = str(Path(args.output).with_suffix(".child.json"))
        child_args = [
            sys.executable,
            __file__,
            "--base-url",
            args.base_url,
            "--output",
            child_output,
            "--timeout-ms",
            str(args.timeout_ms),
            "--_child-run",
        ]
        completed = subprocess.run(
            child_args,
            capture_output=True,
            text=True,
            check=False,
        )
        child_path = Path(child_output)
        if child_path.exists():
            Path(args.output).parent.mkdir(parents=True, exist_ok=True)
            child_path.replace(args.output)
            try:
                payload = json.loads(Path(args.output).read_text(encoding="utf-8"))
            except json.JSONDecodeError:
                return 1
            return 0 if payload.get("status") == "passed" else 1
        report = _blocked_report(
            base_url=args.base_url,
            blocker="playwright_runtime_failed",
            detail=(
                f"Playwright child process exited with code {completed.returncode}. "
                f"stderr={completed.stderr.strip()!r}"
            ),
        )
        _write_report(args.output, report)
        return 1
    try:
        report = _run_playwright_smoke(
            base_url=args.base_url,
            timeout_ms=args.timeout_ms,
        )
    except Exception as exc:  # pragma: no cover - depends on local browser runtime.
        report = _blocked_report(
            base_url=args.base_url,
            blocker="browser_smoke_failed",
            detail=str(exc),
        )
    _write_report(args.output, report)
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
