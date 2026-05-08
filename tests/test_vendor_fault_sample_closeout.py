from __future__ import annotations

import json
from pathlib import Path

from tools.build_vendor_fault_sample_closeout import main


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
HARDWARE_GUIDE = ROOT / "docs/hardware/HARDWARE_INTEGRATION_GUIDE.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload), encoding="utf-8")


def _fault_table() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "vendor": "imc22_reflex",
        "data_version": "2026.04.27-r1",
        "exact_codes": {"45": "overcurrent"},
        "ranges": [],
        "fallback_fault_class": "unknown_fault",
        "change_log": [{"version": "2026.04.27-r1"}],
    }


def _recovery_policy() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "vendor": "imc22_reflex",
        "data_version": "2026.04.27-r1",
        "watchdog_action": {"action": "recover_hold_position"},
        "fault_actions": {"overcurrent": {"action": "recover_relaxed_hold"}},
        "change_log": [{"version": "2026.04.27-r1"}],
    }


def _sample_archive() -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "vendor": "imc22_reflex",
        "data_version": "2026.04.27-r1",
        "change_request": "FIELD-FAULT-DATA-20260427-001",
        "review_owner": "hardware-integration-owner",
        "samples": [
            {
                "node_id": 1,
                "raw_error_value": 45,
                "fault_class": "overcurrent",
                "source_evidence": "test_env/hardware_live/hardware_fault_telemetry_report.json",
                "captured_at": "2026-04-27T08:00:00Z",
                "captured_by": "field-operator",
            }
        ],
        "change_log": [{"version": "2026.04.27-r1"}],
    }


def _write_ready_inputs(tmp_path: Path) -> dict[str, Path]:
    paths = {
        "samples": tmp_path / "samples.json",
        "fault_table": tmp_path / "fault_table.json",
        "recovery_policy": tmp_path / "recovery_policy.json",
        "output": tmp_path / "closeout.json",
    }
    _write_json(paths["samples"], _sample_archive())
    _write_json(paths["fault_table"], _fault_table())
    _write_json(paths["recovery_policy"], _recovery_policy())
    return paths


def test_vendor_fault_sample_closeout_ready(tmp_path: Path) -> None:
    paths = _write_ready_inputs(tmp_path)

    exit_code = main(
        [
            "--sample-archive-file",
            str(paths["samples"]),
            "--fault-table-file",
            str(paths["fault_table"]),
            "--recovery-policy-file",
            str(paths["recovery_policy"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 0
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["blockers"] == []
    assert payload["summary"]["sample_count"] == 1
    assert payload["summary"]["sample_fault_classes"] == ["overcurrent"]


def test_vendor_fault_sample_closeout_blocks_template_placeholders(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)
    sample_archive = _sample_archive()
    sample_archive["change_request"] = "FIELD-FAULT-DATA-YYYYMMDD-001"
    sample_archive["samples"][0]["notes"] = "Replace with real live hardware evidence before promotion."
    sample_archive["samples"][0]["source_evidence"] = ""
    _write_json(paths["samples"], sample_archive)

    exit_code = main(
        [
            "--sample-archive-file",
            str(paths["samples"]),
            "--fault-table-file",
            str(paths["fault_table"]),
            "--recovery-policy-file",
            str(paths["recovery_policy"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert "change_request" in payload["blockers"]
    assert "sample_records" in payload["blockers"]
    assert payload["invalid_samples"][0]["blockers"] == ["source_evidence"]


def test_vendor_fault_sample_closeout_blocks_missing_recovery_policy_class(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)
    recovery_policy = _recovery_policy()
    recovery_policy["fault_actions"] = {}
    _write_json(paths["recovery_policy"], recovery_policy)

    exit_code = main(
        [
            "--sample-archive-file",
            str(paths["samples"]),
            "--fault-table-file",
            str(paths["fault_table"]),
            "--recovery-policy-file",
            str(paths["recovery_policy"]),
            "--output",
            str(paths["output"]),
        ]
    )

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["blockers"] == ["sample_records"]
    assert payload["invalid_samples"][0]["blockers"] == [
        "fault_class_not_in_recovery_policy"
    ]


def test_vendor_fault_sample_closeout_docs_are_linked() -> None:
    tool = "tools/build_vendor_fault_sample_closeout.py"
    report = "test_env/hardware_live/vendor_fault_sample_closeout.json"

    assert tool in README.read_text(encoding="utf-8")
    assert tool in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert report in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert tool in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
