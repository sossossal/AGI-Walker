from __future__ import annotations

import json
from pathlib import Path

from tools.build_hardware_live_diagnostics_checklist import main


def test_hardware_live_checklist_blocks_missing_serial_inputs(tmp_path: Path) -> None:
    output = tmp_path / "live_checklist.json"

    exit_code = main(["--transport", "serial_bridge", "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["transport_inputs_missing"]
    assert payload["missing_inputs"] == ["serial_port", "baudrate"]
    assert payload["checklist"][1]["status"] == "blocked"


def test_hardware_live_checklist_builds_serial_command(tmp_path: Path) -> None:
    output = tmp_path / "live_checklist.json"

    exit_code = main(
        [
            "--transport",
            "serial_bridge",
            "--serial-port",
            "COM5",
            "--baudrate",
            "921600",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    command = payload["diagnostics_command"]
    assert payload["status"] == "ready_to_run"
    assert command[:4] == [
        "python",
        "tools/run_hardware_transport_diagnostics.py",
        "--transport",
        "serial_bridge",
    ]
    assert "--serial-port" in command
    assert "COM5" in command
    assert "--baudrate" in command
    assert "921600" in command
    assert payload["evidence"]["fault_telemetry_report"].endswith(
        "hardware_fault_telemetry_report.json"
    )


def test_hardware_live_checklist_requires_can_channel(tmp_path: Path) -> None:
    output = tmp_path / "live_checklist.json"

    exit_code = main(["--transport", "socketcan", "--output", str(output)])

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["missing_inputs"] == ["channel"]


def test_hardware_live_checklist_accepts_profile_file(tmp_path: Path) -> None:
    output = tmp_path / "live_checklist.json"
    profile = tmp_path / "profile.json"
    profile.write_text(
        json.dumps(
            {
                "serial_port": "COM8",
                "baudrate": 460800,
                "bitrate": 500000,
                "fault_table_file": "deployment/hardware/imc22_reflex_fault_table.json",
                "recovery_policy_file": "deployment/hardware/imc22_reflex_recovery_policy.json",
            }
        ),
        encoding="utf-8",
    )

    exit_code = main(
        [
            "--transport",
            "serial_bridge",
            "--profile-file",
            profile.name,
            "--output",
            str(output),
        ]
    )

    assert exit_code == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["inputs"]["profile_file"] == profile.name
    assert payload["profile_file_status"]["path_valid"] is True
    assert payload["inputs"]["serial_port"] == "COM8"
    assert payload["inputs"]["baudrate"] == 460800
    assert payload["inputs"]["bitrate"] == 500000
    assert "COM8" in payload["diagnostics_command"]
    assert "500000" in payload["diagnostics_command"]


def test_hardware_live_checklist_blocks_unsafe_profile_file_path(
    tmp_path: Path,
) -> None:
    output = tmp_path / "live_checklist.json"

    exit_code = main(
        [
            "--transport",
            "serial_bridge",
            "--profile-file",
            "../profile.json",
            "--serial-port",
            "COM8",
            "--baudrate",
            "460800",
            "--output",
            str(output),
        ]
    )

    assert exit_code == 1
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "blocked"
    assert payload["blockers"] == ["profile_file_path_invalid"]
    assert payload["profile_file_status"]["path_valid"] is False
    assert payload["profile_file_status"]["path_error"] == "parent_directory"
