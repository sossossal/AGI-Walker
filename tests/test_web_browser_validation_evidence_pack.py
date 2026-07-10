from __future__ import annotations

import json
from pathlib import Path

from tools.build_web_browser_validation_evidence_pack import main


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
CHECKLIST = ROOT / "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _write_ready_inputs(tmp_path: Path) -> dict[str, Path]:
    paths = {
        "manual": tmp_path / "manual.json",
        "closeout": tmp_path / "closeout.json",
        "playwright": tmp_path / "playwright.json",
        "output": tmp_path / "pack.json",
    }
    _write_json(
        paths["manual"],
        {
            "schema_version": "1.0",
            "status": "passed",
            "summary": {
                "screenshot_count": 2,
                "export_count": 2,
                "notes_present": True,
                "session_id": "browser-validation",
            },
        },
    )
    _write_json(paths["closeout"], {"schema_version": "1.0", "status": "passed"})
    _write_json(paths["playwright"], {"schema_version": "1.0", "status": "blocked"})
    return paths


def test_web_browser_validation_evidence_pack_ready_with_playwright_warning(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)

    exit_code = main(
        [
            "--manual-report",
            paths["manual"].name,
            "--closeout",
            paths["closeout"].name,
            "--playwright-report",
            paths["playwright"].name,
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 0
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["blockers"] == []
    assert payload["warnings"] == ["playwright_report"]
    assert payload["summary"]["screenshot_count"] == 2


def test_web_browser_validation_evidence_pack_can_require_playwright(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)

    exit_code = main(
        [
            "--manual-report",
            paths["manual"].name,
            "--closeout",
            paths["closeout"].name,
            "--playwright-report",
            paths["playwright"].name,
            "--output",
            str(paths["output"]),
            "--require-playwright",
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["playwright_report"]


def test_web_browser_validation_evidence_pack_blocks_missing_manual_evidence(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)
    _write_json(
        paths["manual"],
        {
            "schema_version": "1.0",
            "status": "passed",
            "summary": {
                "screenshot_count": 0,
                "export_count": 0,
                "notes_present": False,
            },
        },
    )

    exit_code = main(
        [
            "--manual-report",
            paths["manual"].name,
            "--closeout",
            paths["closeout"].name,
            "--playwright-report",
            paths["playwright"].name,
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["blockers"] == [
        "console_error_summary",
        "exports",
        "screenshots",
    ]


def test_web_browser_validation_evidence_pack_blocks_unsafe_source_path(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)

    exit_code = main(
        [
            "--manual-report",
            str(paths["manual"]),
            "--closeout",
            paths["closeout"].name,
            "--playwright-report",
            paths["playwright"].name,
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "source_path_validation" in payload["blockers"]
    assert payload["source_path_statuses"]["manual_report"]["path_valid"] is False
    assert payload["source_path_blockers"] == [
        {"field": "manual_report", "reason": "absolute"}
    ]


def test_web_browser_validation_evidence_pack_docs_are_linked() -> None:
    tool = "tools/build_web_browser_validation_evidence_pack.py"
    report = "test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json"

    assert tool in README.read_text(encoding="utf-8")
    assert tool in CHECKLIST.read_text(encoding="utf-8")
    assert report in CHECKLIST.read_text(encoding="utf-8")
    assert tool in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
