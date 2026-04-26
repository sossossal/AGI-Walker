from __future__ import annotations

import json
from pathlib import Path

from tools.run_hardware_transport_diagnostics import main


IMC22_REPLAY_FIXTURE = Path(__file__).with_name("fixtures") / "imc22_status_replay.json"
IMC22_FAULT_TABLE_FIXTURE = (
    Path(__file__).with_name("fixtures") / "imc22_reflex_fault_table.json"
)
IMC22_RECOVERY_POLICY_FIXTURE = (
    Path(__file__).with_name("fixtures") / "imc22_reflex_recovery_policy.json"
)


def test_hardware_transport_diagnostics_runner_writes_ready_report(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "hardware_transport_diagnostics_report.json"

    exit_code = main(
        [
            "--transport",
            "replay",
            "--replay-source",
            str(IMC22_REPLAY_FIXTURE),
            "--attempt-connect",
            "--output",
            str(report_path),
        ]
    )

    assert exit_code == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "ready"
    assert report["transport_profile"]["transport"] == "replay"
    assert report["fault_telemetry_report"]["entries"]


def test_hardware_transport_diagnostics_runner_exports_fault_telemetry(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "hardware_transport_diagnostics_report.json"
    telemetry_path = tmp_path / "fault_telemetry_report.json"

    exit_code = main(
        [
            "--transport",
            "replay",
            "--replay-source",
            str(IMC22_REPLAY_FIXTURE),
            "--fault-table-file",
            str(IMC22_FAULT_TABLE_FIXTURE),
            "--attempt-connect",
            "--output",
            str(report_path),
            "--telemetry-output",
            str(telemetry_path),
        ]
    )

    assert exit_code == 0
    telemetry_report = json.loads(telemetry_path.read_text(encoding="utf-8"))
    assert telemetry_report["fault_table_source"] == str(IMC22_FAULT_TABLE_FIXTURE)
    assert telemetry_report["entries"]


def test_hardware_transport_diagnostics_runner_accepts_recovery_policy(
    tmp_path: Path,
) -> None:
    report_path = tmp_path / "hardware_transport_diagnostics_report.json"

    exit_code = main(
        [
            "--transport",
            "replay",
            "--replay-source",
            str(IMC22_REPLAY_FIXTURE),
            "--recovery-policy-file",
            str(IMC22_RECOVERY_POLICY_FIXTURE),
            "--output",
            str(report_path),
        ]
    )

    assert exit_code == 0
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert (
        report["transport_profile"]["recovery_policy_source"]
        == str(IMC22_RECOVERY_POLICY_FIXTURE)
    )


def test_hardware_transport_diagnostics_runner_blocks_invalid_profile(
    tmp_path: Path,
) -> None:
    profile_path = tmp_path / "profile.json"
    report_path = tmp_path / "hardware_transport_diagnostics_report.json"
    profile_path.write_text(
        json.dumps({"schema_version": "0.0", "transport": "socketcan"}),
        encoding="utf-8",
    )

    exit_code = main(
        [
            "--profile-file",
            str(profile_path),
            "--output",
            str(report_path),
        ]
    )

    assert exit_code == 1
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "blocked"
    assert report["checks"][0]["name"] == "profile_validation"
