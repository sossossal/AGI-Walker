import uuid

import pytest

pytest.importorskip("fastapi")

from fastapi.testclient import TestClient

import web_panel.server
from web_panel.godot_session_bridge import GodotBridge
from web_panel.godot_session_bridge import GODOT_PROJECT_DIR
from web_panel.ws_protocol import MessageType, WsMessage


@pytest.fixture()
def client() -> TestClient:
    return TestClient(web_panel.server.app)


def test_godot_capabilities_exposes_both_modes(client: TestClient) -> None:
    response = client.get("/api/godot/capabilities")

    assert response.status_code == 200
    payload = response.json()
    assert payload["default_session_id"] == "default"
    assert payload["preferred_mode"] == "session_bridge"
    assert "legacy_controller" in payload["modes"]
    assert "session_bridge" in payload["modes"]
    assert payload["modes"]["legacy_controller"]["status"] == "compatibility_only"
    assert payload["modes"]["legacy_controller"]["session_query_param"] == "session_id"
    assert payload["modes"]["legacy_controller"]["push_messages"] == [
        "telemetry.update",
        "simulation.status",
        "simulation.error",
        "connection.status",
    ]
    assert payload["modes"]["session_bridge"]["tcp_commands"] == [
        "reset",
        "step",
        "get_schema",
        "load_robot",
    ]
    assert payload["modes"]["session_bridge"]["status"] == "preferred"
    assert (
        payload["modes"]["workflow_bridge"]["preferred_transport_mode"]
        == "session_bridge"
    )
    assert "canonical websocket pushes" in payload["note"]


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
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_id = f"launch-{uuid.uuid4().hex}"
    monkeypatch.setattr(GodotBridge, "_find_godot_exe", lambda self: "")

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


def test_godot_bridge_launch_uses_scene_flag(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bridge = GodotBridge("launch-scene-flag", 9010)
    observed: dict[str, object] = {}

    def fake_create_task(coro):
        observed["task_created"] = True
        coro.close()
        return None

    def fake_launch_windows_headless(cmd):
        observed["cmd"] = cmd
        bridge._detached_pid = 4242
        return {
            "status": "launched",
            "pid": 4242,
            "scene": "run_rl_server.tscn",
            "exe": r"C:\Godot\Godot.exe",
        }

    monkeypatch.setattr(bridge, "_find_godot_exe", lambda: r"C:\Godot\Godot.exe")
    monkeypatch.setattr(
        bridge, "_launch_windows_headless", fake_launch_windows_headless
    )
    monkeypatch.setattr("asyncio.create_task", fake_create_task)

    result = bridge.launch(scene="run_rl_server.tscn", headless=True)

    assert result["status"] == "launched"
    assert observed["cmd"] == [
        r"C:\Godot\Godot.exe",
        "--headless",
        "--path",
        GODOT_PROJECT_DIR,
        "--log-file",
        bridge._log_file_path,
        "--scene",
        "run_rl_server.tscn",
        "--",
        "--tcp-port=9010",
    ]
    assert observed["task_created"] is True
    assert bridge.get_pid() == 4242
    assert bridge._log_file_path.endswith("launch-scene-flag_run_rl_server.log")


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
    monkeypatch.setattr(
        web_panel.server.godot_controller, "is_connected", lambda: False
    )
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


def test_websocket_legacy_commands_bind_controller_session(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_id = f"legacy-bind-{uuid.uuid4().hex}"
    recorded = {}

    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: True)
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "start_simulation",
        lambda physics, session_id=None: (
            recorded.update({"physics": physics, "session_id": session_id}) or True
        ),
    )

    with client.websocket_connect(f"/ws/{session_id}") as websocket:
        websocket.send_text(
            WsMessage(
                type=MessageType.SIMULATION_START.value,
                payload={"physics": {"gravity": 1.62}},
            ).to_json()
        )
        response = websocket.receive_json()

    assert response["status"] == "success"
    assert recorded == {
        "physics": {"gravity": 1.62},
        "session_id": session_id,
    }


@pytest.mark.asyncio
async def test_session_bridge_get_sensors_polls_with_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    bridge = GodotBridge("session-test", 9000)
    observed: dict[str, object] = {}

    async def fake_send_recv(payload):
        observed["payload"] = payload
        return {"vector": [0.1, 0.2]}

    monkeypatch.setattr(bridge, "_send_recv", fake_send_recv)

    response = await bridge.get_sensors()

    assert observed["payload"] == {"type": "step", "action": []}
    assert response == {"vector": [0.1, 0.2]}
    assert bridge.last_sensor == {"vector": [0.1, 0.2]}
