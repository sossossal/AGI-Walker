from __future__ import annotations

from types import SimpleNamespace

from web_panel.godot_controller import GodotController


def test_godot_controller_forwards_instruction_and_circuit_payloads() -> None:
    controller = GodotController()
    session_id = "controller-extension-test"
    client = SimpleNamespace(
        send_instruction_set=lambda payload: {"instruction": payload},
        configure_simulated_circuit=lambda payload: {"circuit": payload},
    )

    controller.clients[session_id] = client
    try:
        instruction_result = controller.send_instruction_set(
            {"sequence_name": "demo"}, session_id=session_id
        )
        circuit_result = controller.configure_simulated_circuit(
            {"transport": "imc22_can_fd"}, session_id=session_id
        )
    finally:
        controller.release_session(session_id)

    assert instruction_result == {"instruction": {"sequence_name": "demo"}}
    assert circuit_result == {"circuit": {"transport": "imc22_can_fd"}}
