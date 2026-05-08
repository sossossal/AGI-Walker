from __future__ import annotations

import json
from pathlib import Path

from tools.build_vendor_fault_data_review import main


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_vendor_fault_data_review_passes_matching_telemetry(tmp_path: Path) -> None:
    telemetry = tmp_path / "fault_telemetry.json"
    output = tmp_path / "review.json"
    _write_json(
        telemetry,
        {
            "schema_version": "1.0",
            "entries": [
                {"node_id": 1, "raw_error_value": 45.0, "fault_class": "overcurrent"},
                {
                    "node_id": 2,
                    "raw_error_value": 95.0,
                    "fault_class": "communication_fault",
                },
            ],
        },
    )

    exit_code = main(["--telemetry-report", str(telemetry), "--output", str(output)])

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "passed"
    assert payload["summary"]["entry_count"] == 2
    assert payload["summary"]["mismatch_count"] == 0
    assert payload["summary"]["missing_required_field_count"] == 0
    assert payload["summary"]["required_telemetry_fields"] == [
        "fault_class",
        "node_id",
        "raw_error_value",
    ]
    assert payload["sources"]["telemetry_fields_file"].endswith(
        "deployment/hardware/imc22_fault_telemetry_fields.json"
    )
    assert payload["summary"]["fault_class_counts"]["overcurrent"] == 1
    assert payload["summary"]["fault_class_counts"]["communication_fault"] == 1
    assert payload["next_actions"] == [
        "Attach a reviewed sample archive before promoting new vendor data."
    ]


def test_vendor_fault_data_review_accepts_sample_archive(tmp_path: Path) -> None:
    telemetry = tmp_path / "fault_telemetry.json"
    sample_archive = tmp_path / "fault_samples.json"
    output = tmp_path / "review.json"
    _write_json(
        telemetry,
        {
            "schema_version": "1.0",
            "entries": [
                {"node_id": 1, "raw_error_value": 45.0, "fault_class": "overcurrent"}
            ],
        },
    )
    _write_json(
        sample_archive,
        {
            "schema_version": "1.0",
            "vendor": "imc22_reflex",
            "change_request": "FIELD-FAULT-DATA-20260426-001",
            "samples": [
                {
                    "node_id": 1,
                    "raw_error_value": 45.0,
                    "fault_class": "overcurrent",
                    "source_evidence": str(telemetry),
                    "captured_at": "2026-04-26T00:10:00Z",
                    "captured_by": "field-operator",
                }
            ],
        },
    )

    exit_code = main(
        [
            "--telemetry-report",
            str(telemetry),
            "--sample-archive-file",
            str(sample_archive),
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["summary"]["sample_archive_present"] is True
    assert payload["summary"]["sample_archive_mismatch_count"] == 0
    assert payload["sample_archive_entries"][0]["matches_fault_table"] is True
    assert payload["next_actions"] == [
        "Archive this review with live evidence and use it as vendor data baseline."
    ]


def test_vendor_fault_data_review_blocks_mismatched_telemetry(tmp_path: Path) -> None:
    telemetry = tmp_path / "fault_telemetry.json"
    output = tmp_path / "review.json"
    _write_json(
        telemetry,
        {
            "schema_version": "1.0",
            "entries": [
                {"node_id": 1, "raw_error_value": 45.0, "fault_class": "sensor_fault"}
            ],
        },
    )

    exit_code = main(["--telemetry-report", str(telemetry), "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "telemetry_fault_class_mismatch" in payload["blockers"]
    assert payload["entries"][0]["expected_fault_class"] == "overcurrent"
    assert payload["entries"][0]["matches_fault_table"] is False


def test_vendor_fault_data_review_blocks_missing_required_fields(
    tmp_path: Path,
) -> None:
    telemetry = tmp_path / "fault_telemetry.json"
    output = tmp_path / "review.json"
    _write_json(
        telemetry,
        {
            "schema_version": "1.0",
            "entries": [{"node_id": 1, "raw_error_value": 45.0}],
        },
    )

    exit_code = main(["--telemetry-report", str(telemetry), "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "telemetry_required_fields_missing" in payload["blockers"]
    assert payload["summary"]["missing_required_field_count"] == 1
    assert payload["entries"][0]["missing_required_fields"] == ["fault_class"]


def test_vendor_fault_data_review_blocks_missing_entries(tmp_path: Path) -> None:
    telemetry = tmp_path / "fault_telemetry.json"
    output = tmp_path / "review.json"
    _write_json(telemetry, {"schema_version": "1.0", "entries": []})

    exit_code = main(["--telemetry-report", str(telemetry), "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "telemetry_entries_missing" in payload["blockers"]


def test_vendor_fault_data_review_writes_blocked_report_when_telemetry_missing(
    tmp_path: Path,
) -> None:
    telemetry = tmp_path / "missing_fault_telemetry.json"
    output = tmp_path / "review.json"

    exit_code = main(["--telemetry-report", str(telemetry), "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert "telemetry_report_missing" in payload["blockers"]
    assert "telemetry_entries_missing" in payload["blockers"]
    assert payload["summary"]["telemetry_report_present"] is False
