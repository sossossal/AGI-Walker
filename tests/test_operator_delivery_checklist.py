from __future__ import annotations

import json
from pathlib import Path

from tools.build_operator_delivery_checklist import main


ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / "deployment/operator_delivery_checklist.template.json"
GUIDE = ROOT / "docs/guides/OPERATOR_DELIVERY_CHECKLIST_AUTOMATION_20260427.md"
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
SLO_VIEW = ROOT / "docs/guides/SLA_SLO_VIEW_20260427.md"


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _template(tmp_path: Path) -> Path:
    payload = {
        "schema_version": "1.0",
        "checklist_id": "test_operator_delivery",
        "delivery_window": {"window_id": "window-1"},
        "required_items": [
            {
                "id": "system_status",
                "owner": "operator",
                "evidence_path": "evidence/system_status.json",
                "status_path": "status",
                "expected_statuses": ["running"],
            },
            {
                "id": "browser_validation_closeout",
                "owner": "operator",
                "evidence_path": "evidence/browser_closeout.json",
                "status_path": "status",
                "expected_statuses": ["passed"],
            },
            {
                "id": "ros2_smoke",
                "owner": "delivery_engineer",
                "evidence_path": "evidence/ros2_smoke.json",
                "status_path": "status",
                "expected_statuses": ["passed"],
                "required": False,
            },
        ],
    }
    path = tmp_path / "template.json"
    _write_json(path, payload)
    return path


def test_operator_delivery_checklist_ready_with_optional_warning(
    tmp_path: Path, monkeypatch,
) -> None:
    monkeypatch.setattr("tools.build_operator_delivery_checklist.PROJECT_ROOT", tmp_path)
    template = _template(tmp_path)
    output = tmp_path / "operator_delivery_checklist.json"
    evidence = tmp_path / "evidence"
    _write_json(evidence / "system_status.json", {"status": "running"})
    _write_json(evidence / "browser_closeout.json", {"status": "passed"})

    exit_code = main(["--template", str(template), "--output", str(output)])

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["summary"]["blocked_item_count"] == 0
    assert payload["summary"]["warning_item_count"] == 1
    assert payload["warnings"] == ["ros2_smoke"]


def test_operator_delivery_checklist_blocks_missing_required_item(
    tmp_path: Path, monkeypatch,
) -> None:
    monkeypatch.setattr("tools.build_operator_delivery_checklist.PROJECT_ROOT", tmp_path)
    template = _template(tmp_path)
    output = tmp_path / "operator_delivery_checklist.json"
    evidence = tmp_path / "evidence"
    _write_json(evidence / "system_status.json", {"status": "running"})

    exit_code = main(["--template", str(template), "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["browser_validation_closeout"]
    blocked = {
        item["id"]: item for item in payload["items"] if item["status"] == "blocked"
    }
    assert blocked["browser_validation_closeout"]["reason"] == "evidence_missing"


def test_operator_delivery_checklist_supports_evidence_override(
    tmp_path: Path, monkeypatch,
) -> None:
    monkeypatch.setattr("tools.build_operator_delivery_checklist.PROJECT_ROOT", tmp_path)
    template = _template(tmp_path)
    output = tmp_path / "operator_delivery_checklist.json"
    evidence = tmp_path / "evidence"
    override = tmp_path / "custom" / "browser_closeout.json"
    _write_json(evidence / "system_status.json", {"status": "running"})
    _write_json(override, {"status": "passed"})

    exit_code = main(
        [
            "--template",
            str(template),
            "--output",
            str(output),
            "--set-evidence",
            "browser_validation_closeout=custom/browser_closeout.json",
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    browser_item = next(
        item for item in payload["items"] if item["id"] == "browser_validation_closeout"
    )
    assert browser_item["evidence_path"] == "custom/browser_closeout.json"
    assert browser_item["evidence_path_valid"] is True
    assert browser_item["status"] == "ready"


def test_operator_delivery_checklist_blocks_unsafe_evidence_paths(
    tmp_path: Path, monkeypatch,
) -> None:
    monkeypatch.setattr("tools.build_operator_delivery_checklist.PROJECT_ROOT", tmp_path)
    template = _template(tmp_path)
    output = tmp_path / "operator_delivery_checklist.json"

    exit_code = main(
        [
            "--template",
            str(template),
            "--output",
            str(output),
            "--set-evidence",
            "system_status=../outside.json",
            "--set-evidence",
            f"browser_validation_closeout={tmp_path / 'absolute.json'}",
        ]
    )

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    items = {item["id"]: item for item in payload["items"]}
    assert payload["blockers"] == ["system_status", "browser_validation_closeout"]
    assert items["system_status"]["reason"] == "evidence_path_invalid"
    assert items["system_status"]["evidence_path_valid"] is False
    assert items["browser_validation_closeout"]["reason"] == "evidence_path_invalid"
    assert items["browser_validation_closeout"]["evidence_path_valid"] is False


def test_operator_delivery_checklist_docs_and_template_are_linked() -> None:
    template_content = TEMPLATE.read_text(encoding="utf-8")
    guide_content = GUIDE.read_text(encoding="utf-8")
    path = "docs/guides/OPERATOR_DELIVERY_CHECKLIST_AUTOMATION_20260427.md"

    assert "tools/build_operator_delivery_checklist.py" in template_content
    assert "deployment/operator_delivery_checklist.template.json" in guide_content
    assert "tools/build_operator_delivery_checklist.py" in guide_content
    assert "tools/build_customer_site_live_smoke_report.py" in guide_content
    assert "--require-customer-site-smoke" in guide_content
    assert path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert "deployment/operator_delivery_checklist.template.json" in README.read_text(
        encoding="utf-8"
    )
    slo_content = SLO_VIEW.read_text(encoding="utf-8")
    assert "tools/build_operator_delivery_checklist.py" in slo_content
    assert "tools/build_customer_site_live_smoke_report.py" in slo_content
    assert "--require-customer-site-smoke" in slo_content
