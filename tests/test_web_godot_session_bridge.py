import uuid

import pytest

pytest.importorskip("fastapi")

from fastapi.testclient import TestClient

import web_panel.server
from web_panel.ws_protocol import MessageType, WsMessage


@pytest.fixture()
def client() -> TestClient:
    return TestClient(web_panel.server.app)


def test_godot_capabilities_exposes_both_modes(client: TestClient) -> None:
    response = client.get("/api/godot/capabilities")

    assert response.status_code == 200
    payload = response.json()
    assert payload["default_session_id"] == "default"
    assert "legacy_controller" in payload["modes"]
    assert "session_bridge" in payload["modes"]
    assert payload["modes"]["session_bridge"]["tcp_commands"] == [
        "reset",
        "step",
        "get_schema",
    ]


def test_session_bridge_status_defaults_to_disconnected(client: TestClient) -> None:
    session_id = f"status-{uuid.uuid4().hex}"

    response = client.get(f"/api/godot/{session_id}/status")

    assert response.status_code == 200
    assert response.json() == {
        "engine_running": False,
        "tcp_connected": False,
        "last_sensor": {},
    }


def test_session_bridge_launch_without_godot_exe_returns_real_error(
    client: TestClient,
) -> None:
    session_id = f"launch-{uuid.uuid4().hex}"

    response = client.post(f"/api/godot/{session_id}/launch", json={})

    assert response.status_code == 200
    assert response.json()["status"] == "error"
    assert "Godot" in response.json()["message"]


def test_session_bridge_control_without_session_returns_error(
    client: TestClient,
) -> None:
    session_id = f"control-{uuid.uuid4().hex}"

    response = client.post(
        f"/api/godot/{session_id}/control",
        json={"action": [0.1, -0.1]},
    )

    assert response.status_code == 200
    assert response.json()["status"] == "error"
    assert "Session" in response.json()["message"]


def test_websocket_ping_round_trip(client: TestClient) -> None:
    session_id = f"ws-{uuid.uuid4().hex}"

    with client.websocket_connect(f"/ws/{session_id}") as websocket:
        websocket.send_text(WsMessage(type=MessageType.PING.value).to_json())
        response = websocket.receive_json()

    assert response["type"] == MessageType.PONG.value
    assert response["status"] == "success"
    assert "timestamp" in response["payload"]


def test_websocket_commands_return_error_when_godot_not_connected(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: False)
    session_id = f"ws-{uuid.uuid4().hex}"

    with client.websocket_connect(f"/ws/{session_id}") as websocket:
        websocket.send_text(
            WsMessage(
                type=MessageType.SIMULATION_START.value,
                payload={"physics": {"gravity": 9.81}},
            ).to_json()
        )
        start_response = websocket.receive_json()

        websocket.send_text(
            WsMessage(
                type=MessageType.CONFIG_LOAD_ROBOT.value,
                payload={"robot_config": {"parts": [], "connections": []}},
            ).to_json()
        )
        load_response = websocket.receive_json()

    assert start_response["status"] == "error"
    assert start_response["payload"]["error"] == "Godot is not connected"
    assert load_response["status"] == "error"
    assert load_response["payload"]["error"] == "Godot is not connected"
