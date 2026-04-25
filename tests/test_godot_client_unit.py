import pytest

from agi_walker.core.api.comm.godot_client import GodotSimulationClient


def test_start_simulation_merges_physics_overrides(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = GodotSimulationClient()
    recorded = {}

    monkeypatch.setattr(
        client,
        "send_command",
        lambda command, data=None: (
            recorded.update(
                {
                    "command": command,
                    "data": data,
                }
            )
            or True
        ),
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


def test_instruction_and_circuit_helpers_forward_to_send_command(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = GodotSimulationClient()
    recorded = []

    monkeypatch.setattr(
        client,
        "send_command",
        lambda command, data=None: recorded.append((command, data)) or True,
    )

    instruction_result = client.send_instruction_set({"sequence_name": "demo"})
    circuit_result = client.configure_simulated_circuit({"transport": "imc22_can_fd"})

    assert instruction_result is True
    assert circuit_result is True
    assert recorded == [
        ("instruction_set", {"sequence_name": "demo"}),
        ("configure_simulated_circuit", {"transport": "imc22_can_fd"}),
    ]
