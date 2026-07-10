from __future__ import annotations

import json
from pathlib import Path

from tools.build_vendor_data_promotion_checklist import main


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def _versioned_payload(kind: str) -> dict[str, object]:
    return {
        "schema_version": "1.0",
        "vendor": "imc22_reflex",
        "data_version": "2026.04.26-r1",
        kind: {},
        "change_log": [
            {
                "version": "2026.04.26-r1",
                "date": "2026-04-26",
                "summary": f"{kind} baseline",
            }
        ],
    }


def _write_ready_inputs(tmp_path: Path) -> dict[str, Path]:
    paths = {
        "fault_table": tmp_path / "fault_table.json",
        "recovery_policy": tmp_path / "recovery_policy.json",
        "telemetry_fields": tmp_path / "telemetry_fields.json",
        "sample_archive": tmp_path / "sample_archive.json",
        "vendor_review": tmp_path / "vendor_review.json",
        "output": tmp_path / "promotion.json",
    }
    _write_json(paths["fault_table"], _versioned_payload("exact_codes"))
    _write_json(paths["recovery_policy"], _versioned_payload("fault_actions"))
    _write_json(paths["telemetry_fields"], _versioned_payload("fields"))
    _write_json(
        paths["sample_archive"],
        {
            "schema_version": "1.0",
            "vendor": "imc22_reflex",
            "data_version": "2026.04.26-r1",
            "change_request": "FIELD-FAULT-DATA-20260426-001",
            "samples": [
                {
                    "node_id": 1,
                    "raw_error_value": 45.0,
                    "fault_class": "overcurrent",
                    "source_evidence": "test_env/hardware_live/faults.json",
                    "captured_at": "2026-04-26T00:10:00Z",
                    "captured_by": "field-operator",
                }
            ],
            "change_log": [
                {
                    "version": "2026.04.26-r1",
                    "date": "2026-04-26",
                    "summary": "sample archive baseline",
                }
            ],
        },
    )
    _write_json(
        paths["vendor_review"],
        {
            "schema_version": "1.0",
            "status": "passed",
            "summary": {"sample_archive_present": True},
        },
    )
    return paths


def _ready_args(paths: dict[str, Path]) -> list[str]:
    return [
        "--fault-table-file",
        paths["fault_table"].name,
        "--recovery-policy-file",
        paths["recovery_policy"].name,
        "--telemetry-fields-file",
        paths["telemetry_fields"].name,
        "--sample-archive-file",
        paths["sample_archive"].name,
        "--vendor-review-file",
        paths["vendor_review"].name,
        "--output",
        str(paths["output"]),
    ]


def test_vendor_data_promotion_checklist_ready(tmp_path: Path) -> None:
    paths = _write_ready_inputs(tmp_path)

    exit_code = main(_ready_args(paths))

    assert exit_code == 0
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "ready"
    assert payload["summary"]["change_request"] == "FIELD-FAULT-DATA-20260426-001"
    assert payload["summary"]["blocked_step_count"] == 0
    assert payload["summary"]["source_file_path_validation_error_count"] == 0
    assert payload["summary"]["sample_source_evidence_path_validation_error_count"] == 0
    assert payload["next_actions"] == [
        "Promote reviewed vendor data and archive this checklist with live evidence."
    ]


def test_vendor_data_promotion_checklist_blocks_without_review_archive(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)
    _write_json(
        paths["vendor_review"],
        {
            "schema_version": "1.0",
            "status": "passed",
            "summary": {"sample_archive_present": False},
        },
    )

    exit_code = main(_ready_args(paths))

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    blocked_steps = {
        step["id"] for step in payload["steps"] if step["status"] == "blocked"
    }
    assert blocked_steps == {"vendor_review"}


def test_vendor_data_promotion_checklist_blocks_placeholder_change_request(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)
    sample_archive = json.loads(paths["sample_archive"].read_text(encoding="utf-8"))
    sample_archive["change_request"] = "FIELD-FAULT-DATA-YYYYMMDD-001"
    _write_json(paths["sample_archive"], sample_archive)

    exit_code = main(_ready_args(paths))

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    blocked_steps = {
        step["id"] for step in payload["steps"] if step["status"] == "blocked"
    }
    assert blocked_steps == {"change_request"}


def test_vendor_data_promotion_checklist_blocks_unsafe_source_file_path(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)
    args = _ready_args(paths)
    args[args.index("--fault-table-file") + 1] = "../fault_table.json"

    exit_code = main(args)

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    blocked_steps = {
        step["id"] for step in payload["steps"] if step["status"] == "blocked"
    }
    assert "source_paths" in blocked_steps
    assert payload["source_file_statuses"]["fault_table_file"]["path_valid"] is False
    assert payload["summary"]["source_file_path_validation_error_count"] == 1


def test_vendor_data_promotion_checklist_blocks_unsafe_sample_source_evidence(
    tmp_path: Path,
) -> None:
    paths = _write_ready_inputs(tmp_path)
    sample_archive = json.loads(paths["sample_archive"].read_text(encoding="utf-8"))
    sample_archive["samples"][0]["source_evidence"] = "../faults.json"
    _write_json(paths["sample_archive"], sample_archive)

    exit_code = main(_ready_args(paths))

    assert exit_code == 1
    payload = json.loads(paths["output"].read_text(encoding="utf-8"))
    blocked_steps = {
        step["id"] for step in payload["steps"] if step["status"] == "blocked"
    }
    assert "sample_source_evidence_paths" in blocked_steps
    assert payload["sample_source_evidence_statuses"][0]["path_valid"] is False
    assert (
        payload["summary"]["sample_source_evidence_path_validation_error_count"] == 1
    )
