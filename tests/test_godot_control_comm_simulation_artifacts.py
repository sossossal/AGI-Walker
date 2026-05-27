import importlib.util
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "agi_walker/core/simulation/control_comm_simulation.py"
TOOL_PATH = ROOT / "tools/run_godot_control_comm_replay.py"
GODOT_REPLAY_SCRIPT = ROOT / "godot_project/scripts/control_comm_replay.gd"

spec = importlib.util.spec_from_file_location("control_comm_simulation", MODULE_PATH)
control_comm_simulation = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(control_comm_simulation)

tool_spec = importlib.util.spec_from_file_location(
    "run_godot_control_comm_replay", TOOL_PATH
)
godot_replay_tool = importlib.util.module_from_spec(tool_spec)
assert tool_spec.loader is not None
tool_spec.loader.exec_module(godot_replay_tool)


def test_godot_replay_script_exposes_expected_log_contract_names() -> None:
    script = GODOT_REPLAY_SCRIPT.read_text(encoding="utf-8")

    assert "godot_control_comm_simulation_log.v1" in script
    assert "control_message_envelope.v1" in script
    assert "--control-comm-scenario" in script
    assert "--control-comm-log" in script
    assert "JSON.stringify" in script


def test_retained_godot_log_contract_preview_uses_canonical_envelopes(
    tmp_path: Path,
) -> None:
    scenario = control_comm_simulation.build_default_control_comm_scenario(
        cycle_count=2,
        cycle_period_ns=1_000,
    )
    report = control_comm_simulation.run_deterministic_control_comm_simulation(
        scenario,
        output_root=tmp_path,
    )
    log_path = Path(report["artifact_paths"]["godot_log_contract_preview"])
    log_payload = json.loads(log_path.read_text(encoding="utf-8"))

    assert log_payload["log_version"] == "godot_control_comm_simulation_log.v1"
    assert log_payload["status"] == "not_run"
    assert log_payload["events"][0]["envelope"]["schema_version"] == (
        "control_message_envelope.v1"
    )
    assert (
        control_comm_simulation.validate_godot_control_comm_simulation_log(log_payload)
        == []
    )


def test_godot_replay_runner_dry_run_blocks_without_executable(
    monkeypatch,
    tmp_path: Path,
) -> None:
    monkeypatch.delenv("GODOT_EXECUTABLE", raising=False)
    monkeypatch.delenv("GODOT", raising=False)
    monkeypatch.delenv("GODOT_EXE", raising=False)
    monkeypatch.delenv("GODOT_PATH", raising=False)
    monkeypatch.setattr(godot_replay_tool.shutil, "which", lambda _name: None)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "run_godot_control_comm_replay.py",
            "--dry-run-discovery",
            "--output-root",
            str(tmp_path),
        ],
    )

    assert godot_replay_tool.main() == 0
    report = json.loads(
        (tmp_path / "godot_control_comm_replay_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert report["report_version"] == "godot_control_comm_replay_report.v1"
    assert report["status"] == "blocked"
    assert report["failure_category"] == "missing_godot_executable"


def test_godot_replay_runner_archives_and_validates_log(
    monkeypatch,
    tmp_path: Path,
) -> None:
    fake_godot = tmp_path / "Godot.exe"
    fake_godot.write_text("fake", encoding="utf-8")

    def fake_run(command, **_kwargs):
        scenario_path = Path(command[command.index("--control-comm-scenario") + 1])
        log_path = Path(command[command.index("--control-comm-log") + 1])
        scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
        envelopes = [
            control_comm_simulation.build_control_message_envelope(
                topic=scenario["topic"],
                sequence=index,
                timestamp_ns=index * int(scenario["cycle_period_ns"]),
                source=scenario["source"],
                target=scenario["target"],
                payload_type=scenario["payload_type"],
                payload=scenario["command"],
                metadata={
                    "scenario_id": scenario["scenario_id"],
                    "cycle_index": index,
                },
            )
            for index in range(int(scenario["cycle_count"]))
        ]
        log_payload = control_comm_simulation.build_godot_log_contract_preview(
            scenario=scenario,
            envelopes=envelopes,
        )
        log_payload["status"] = "success"
        log_payload["godot_profile"]["mode"] = "headless"
        log_path.write_text(json.dumps(log_payload), encoding="utf-8")
        return subprocess.CompletedProcess(command, 0, stdout="ok", stderr="")

    monkeypatch.setattr(godot_replay_tool.subprocess, "run", fake_run)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "run_godot_control_comm_replay.py",
            "--godot-exe",
            str(fake_godot),
            "--output-root",
            str(tmp_path),
        ],
    )

    assert godot_replay_tool.main() == 0
    report = json.loads(
        (tmp_path / "godot_control_comm_replay_report.json").read_text(
            encoding="utf-8"
        )
    )
    assert report["status"] == "success"
    assert report["failure_category"] is None
    assert report["log_validation_errors"] == []
    assert report["godot_log_summary"]["status"] == "success"
    assert Path(report["artifact_paths"]["godot_control_comm_simulation_log"]).exists()
