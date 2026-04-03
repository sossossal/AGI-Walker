import pytest

from python_api.comm.godot_client import GodotSimulationClient


def test_start_simulation_merges_physics_overrides(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = GodotSimulationClient()
    recorded = {}

    monkeypatch.setattr(
        client,
        "send_command",
        lambda command, data=None: recorded.update(
            {
                "command": command,
                "data": data,
            }
        )
        or True,
    )

    result = client.start_simulation(
        {"parts": [{"id": "torso"}]},
        {"gravity": 1.62},
    )

    assert result is True
    assert recorded == {
        "command": "start_sim",
        "data": {
            "robot": {"parts": [{"id": "torso"}]},
            "physics": {"gravity": 1.62, "timestep": 0.01},
        },
    }
