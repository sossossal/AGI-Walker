from __future__ import annotations

import json
from pathlib import Path

from tools.build_hardware_live_closeout_report import main


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
HARDWARE_GUIDE = ROOT / "docs/hardware/HARDWARE_INTEGRATION_GUIDE.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
CUSTOMER_SMOKE_GUIDE = ROOT / "docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md"


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _write_ready_required_evidence(tmp_path: Path) -> dict[str, Path]:
    paths = {
        "checklist": tmp_path / "live_diagnostics_checklist.json",
        "diagnostics": tmp_path / "hardware_transport_diagnostics_report.json",
        "telemetry": tmp_path / "hardware_fault_telemetry_report.json",
        "customer_site_smoke": tmp_path / "customer_site_live_smoke_report.json",
        "output": tmp_path / "hardware_live_closeout_report.json",
    }
    _write_json(
        paths["checklist"],
        {"schema_version": "1.0", "status": "ready_to_run", "transport": "serial_bridge"},
    )
    _write_json(paths["diagnostics"], {"schema_version": "1.0", "status": "ready"})
    _write_json(
        paths["telemetry"],
        {
            "schema_version": "1.0",
            "status": "ready",
            "entries": [{"node_id": 1, "raw_error_value": 45, "fault_class": "overcurrent"}],
        },
    )
    _write_json(paths["customer_site_smoke"], {"schema_version": "1.0", "status": "passed"})
    return paths


def test_hardware_live_closeout_report_ready(tmp_path: Path) -> None:
    paths = _write_ready_required_evidence(tmp_path)

    exit_code = main(
        [
            "--checklist",
            str(paths["checklist"]),
            "--diagnostics",
            str(paths["diagnostics"]),
            "--telemetry",
            str(paths["telemetry"]),
            "--customer-site-smoke",
            str(paths["customer_site_smoke"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 0
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["blockers"] == []
    assert payload["summary"]["telemetry_entry_count"] == 1
    assert payload["summary"]["customer_site_smoke_status"] == "passed"


def test_hardware_live_closeout_blocks_empty_telemetry(tmp_path: Path) -> None:
    paths = _write_ready_required_evidence(tmp_path)
    _write_json(paths["telemetry"], {"schema_version": "1.0", "status": "ready", "entries": []})

    exit_code = main(
        [
            "--checklist",
            str(paths["checklist"]),
            "--diagnostics",
            str(paths["diagnostics"]),
            "--telemetry",
            str(paths["telemetry"]),
            "--customer-site-smoke",
            str(paths["customer_site_smoke"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["fault_telemetry"]
    telemetry_item = next(item for item in payload["evidence"] if item["id"] == "fault_telemetry")
    assert telemetry_item["reason"] == "telemetry_entries_missing"


def test_hardware_live_closeout_blocks_missing_customer_site_smoke(
    tmp_path: Path,
) -> None:
    paths = _write_ready_required_evidence(tmp_path)
    paths["customer_site_smoke"].unlink()

    exit_code = main(
        [
            "--checklist",
            str(paths["checklist"]),
            "--diagnostics",
            str(paths["diagnostics"]),
            "--telemetry",
            str(paths["telemetry"]),
            "--customer-site-smoke",
            str(paths["customer_site_smoke"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["blockers"] == ["customer_site_smoke"]


def test_hardware_live_closeout_docs_are_linked() -> None:
    tool = "tools/build_hardware_live_closeout_report.py"
    report = "test_env/hardware_live/hardware_live_closeout_report.json"

    assert tool in README.read_text(encoding="utf-8")
    assert tool in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert report in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert tool in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert report in CUSTOMER_SMOKE_GUIDE.read_text(encoding="utf-8")
