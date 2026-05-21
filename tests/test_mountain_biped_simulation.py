import json
from pathlib import Path

from agi_walker.core.api.robot_schema import validate_godot_robot_config
from examples.mountain_biped_simulation import (
    RunConfig,
    TerrainConfig,
    load_robot_config,
    simulate_mountain_run,
)

REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_CONFIG = REPO_ROOT / "configs" / "mountain_humanoid_biped.json"
TERRAIN_CONFIG = REPO_ROOT / "configs" / "mountain_terrain.json"
TCP_SERVER = REPO_ROOT / "godot_project" / "scripts" / "tcp_server.gd"
SMOKE_RUNNER = REPO_ROOT / "tools" / "run_dynamic_godot_robot_smoke.py"
REPORT_BUILDER = REPO_ROOT / "tools" / "build_dynamic_robot_generation_report.py"


def test_mountain_humanoid_biped_config_validates() -> None:
    payload = json.loads(ROBOT_CONFIG.read_text(encoding="utf-8"))

    assert validate_godot_robot_config(payload) == []


def test_mountain_biped_simulation_is_deterministic() -> None:
    robot = load_robot_config(ROBOT_CONFIG)
    terrain = TerrainConfig(seed=7, length_m=8.0)
    run = RunConfig(steps=180)

    first_report, first_trace = simulate_mountain_run(robot, terrain, run)
    second_report, second_trace = simulate_mountain_run(robot, terrain, run)

    assert first_report == second_report
    assert first_trace == second_trace
    assert first_report["report_version"] == "dynamic_mountain_biped_simulation.v1"


def test_mountain_biped_simulation_reports_completed_run() -> None:
    robot = load_robot_config(ROBOT_CONFIG)
    report, trace = simulate_mountain_run(
        robot,
        TerrainConfig(seed=42, length_m=8.0),
        RunConfig(steps=260),
    )

    status = report["run_status"]
    assert status["status"] == "completed"
    assert status["fall_detected"] is False
    assert status["distance_m"] >= 6.0
    assert status["min_stability_margin_m"] > -0.04
    assert trace
    assert set(trace[0]) == {
        "step",
        "time_s",
        "base_position_m",
        "terrain_height_m",
        "terrain_pitch_deg",
        "stability_margin_m",
        "contacts",
        "joint_targets_rad",
    }


def test_mountain_terrain_config_is_godot_ready() -> None:
    payload = json.loads(TERRAIN_CONFIG.read_text(encoding="utf-8"))

    assert payload["terrain_version"] == "dynamic_godot_mountain_terrain.v1"
    assert payload["type"] == "mountain"
    assert payload["map_size"] >= 4
    assert payload["ridge_height_m"] > 0


def test_live_godot_smoke_supports_mountain_terrain_contract() -> None:
    tcp_content = TCP_SERVER.read_text(encoding="utf-8")
    smoke_content = SMOKE_RUNNER.read_text(encoding="utf-8")
    report_content = REPORT_BUILDER.read_text(encoding="utf-8")

    assert 'cmd.type == "configure_terrain"' in tcp_content
    assert "HeightMapShape3D.new()" in tcp_content
    assert '"terrain": terrain_summary' in tcp_content
    assert "--terrain-json" in smoke_content
    assert '{"type": "configure_terrain", "terrain": terrain_config}' in smoke_content
    assert "--terrain-json" in report_content
