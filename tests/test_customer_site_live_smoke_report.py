from __future__ import annotations

import json
from pathlib import Path

from tools.build_customer_site_live_smoke_report import main


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
GUIDE = ROOT / "docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md"
SLO_VIEW = ROOT / "docs/guides/SLA_SLO_VIEW_20260427.md"
ARCHIVE_GUIDE = ROOT / "docs/guides/INDUSTRIAL_LIVE_EVIDENCE_ARCHIVE_20260427.md"
TEMPLATE = ROOT / "deployment/customer_site_live_smoke.template.json"


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _ready_payload(tmp_path: Path) -> dict[str, object]:
    evidence_root = tmp_path / "evidence"
    return {
        "schema_version": "1.0",
        "smoke_id": "customer-site-smoke-test",
        "site": {
            "engagement_id": "eng-001",
            "site_id": "site-line-1",
            "target_environment": "customer-prod-line-1",
            "operator": "field-operator",
            "started_at": "2026-04-27T08:00:00Z",
            "finished_at": "2026-04-27T08:10:00Z",
        },
        "device": {
            "vendor": "imc22_reflex",
            "device_id": "imc22-001",
            "transport": "serial_bridge",
            "endpoint": "COM5",
            "firmware_version": "1.2.3",
        },
        "safety_precheck": {
            "emergency_stop_verified": True,
            "power_verified": True,
            "mechanical_clearance_verified": True,
            "operator_has_hardware_recovery_role": True,
        },
        "checks": [
            {
                "id": "transport_connect",
                "status": "passed",
                "evidence_path": str(evidence_root / "transport_connect.json"),
            },
            {
                "id": "telemetry_read",
                "status": "ready",
                "evidence_path": str(evidence_root / "telemetry_read.json"),
            },
        ],
        "archive": {
            "closure_archive_root": "archive://customer/site-line-1/window-001",
            "report_output": "test_env/customer_site_live_smoke/customer_site_live_smoke_report.json",
        },
    }


def test_customer_site_live_smoke_report_passes_ready_payload(tmp_path: Path) -> None:
    input_path = tmp_path / "customer_site_live_smoke.json"
    output_path = tmp_path / "customer_site_live_smoke_report.json"
    _write_json(input_path, _ready_payload(tmp_path))

    exit_code = main(["--input", str(input_path), "--output", str(output_path)])

    assert exit_code == 0
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["blockers"] == []
    assert payload["summary"]["site_id"] == "site-line-1"
    assert payload["summary"]["passed_check_count"] == 2


def test_customer_site_live_smoke_report_blocks_placeholders_and_safety(
    tmp_path: Path,
) -> None:
    input_path = tmp_path / "customer_site_live_smoke.json"
    output_path = tmp_path / "customer_site_live_smoke_report.json"
    payload = _ready_payload(tmp_path)
    payload["site"]["target_environment"] = "<customer-production-environment-id>"
    payload["safety_precheck"]["emergency_stop_verified"] = False
    _write_json(input_path, payload)

    exit_code = main(["--input", str(input_path), "--output", str(output_path)])

    assert exit_code == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["status"] == "blocked"
    assert "site.target_environment" in report["missing_fields"]
    assert "safety_precheck.emergency_stop_verified" in report["safety_blockers"]


def test_customer_site_live_smoke_report_can_require_evidence_files(
    tmp_path: Path,
) -> None:
    input_path = tmp_path / "customer_site_live_smoke.json"
    output_path = tmp_path / "customer_site_live_smoke_report.json"
    payload = _ready_payload(tmp_path)
    _write_json(input_path, payload)

    exit_code = main(
        [
            "--input",
            str(input_path),
            "--output",
            str(output_path),
            "--require-evidence-files",
        ]
    )

    assert exit_code == 1
    report = json.loads(output_path.read_text(encoding="utf-8"))
    assert report["summary"]["require_evidence_files"] is True
    assert report["blockers"] == ["transport_connect", "telemetry_read"]


def test_customer_site_live_smoke_docs_and_template_are_linked() -> None:
    path = "docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md"
    tool = "tools/build_customer_site_live_smoke_report.py"
    template = "deployment/customer_site_live_smoke.template.json"

    assert tool in README.read_text(encoding="utf-8")
    assert path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert template in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert tool in GUIDE.read_text(encoding="utf-8")
    assert template in GUIDE.read_text(encoding="utf-8")
    assert tool in SLO_VIEW.read_text(encoding="utf-8")
    assert "customer_site_smoke" in ARCHIVE_GUIDE.read_text(encoding="utf-8")
    assert "clear_faults_or_recover" in TEMPLATE.read_text(encoding="utf-8")
