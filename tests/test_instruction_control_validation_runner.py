from __future__ import annotations

import json
from pathlib import Path

from tools.run_instruction_control_validation import main


def _write_json(path: Path, payload: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def test_instruction_control_validation_report_passes_when_expected_reports_exist(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "test_env" / "instruction_control_validation"
    _write_json(output_root / "smoke_report.json", {"status": "passed"})
    _write_json(
        output_root / "godot_instruction_smoke" / "godot_instruction_smoke_report.json",
        {"status": "passed"},
    )
    _write_json(
        output_root / "ros2_instruction_smoke" / "ros2_instruction_smoke_report.json",
        {"event": "instruction_set_applied", "latest_result": {"status": "applied"}},
    )
    _write_json(
        output_root
        / "simulated_circuit_smoke"
        / "simulated_circuit_replay_smoke_report.json",
        {"status": "passed"},
    )

    exit_code = main(["--output-root", str(output_root), "--skip-smoke-run"])

    assert exit_code == 0
    report = json.loads(
        (output_root / "instruction_control_validation_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert report["status"] == "passed"
    assert report["missing_reports"] == []
    assert report["specialized_reports"]["godot_instruction_smoke"]["status"] == "passed"


def test_instruction_control_validation_report_blocks_when_specialized_report_missing(
    tmp_path: Path,
) -> None:
    output_root = tmp_path / "test_env" / "instruction_control_validation"
    _write_json(output_root / "smoke_report.json", {"status": "passed"})
    _write_json(
        output_root / "godot_instruction_smoke" / "godot_instruction_smoke_report.json",
        {"status": "passed"},
    )
    _write_json(
        output_root / "ros2_instruction_smoke" / "ros2_instruction_smoke_report.json",
        {"status": "passed"},
    )

    exit_code = main(["--output-root", str(output_root), "--skip-smoke-run"])

    assert exit_code == 1
    report = json.loads(
        (output_root / "instruction_control_validation_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert report["status"] == "blocked"
    assert report["missing_reports"] == [
        "simulated_circuit_smoke/simulated_circuit_replay_smoke_report.json"
    ]
