import asyncio
import json
import struct
import uuid

import pytest

pytest.importorskip("fastapi")

from fastapi.testclient import TestClient

import web_panel.server
from web_panel.godot_session_bridge import (
    GODOT_PROJECT_DIR,
    GODOT_SESSION_STATUS_SCHEMA_VERSION,
    GodotBridge,
)
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
    assert payload["modes"]["session_bridge"]["status_schema_version"] == "1.0"
    assert payload["modes"]["session_bridge"]["session_states"] == [
        "disconnected",
        "launching",
        "connected",
        "schema_ready",
        "running",
        "failed",
    ]
    assert (
        payload["modes"]["workflow_bridge"]["preferred_transport_mode"]
        == "session_bridge"
    )
    assert payload["modes"]["workflow_bridge"]["artifact_contract_version"] == "1.0"
    assert "canonical websocket pushes" in payload["note"]


def test_session_bridge_status_defaults_to_disconnected(client: TestClient) -> None:
    session_id = f"status-{uuid.uuid4().hex}"

    response = client.get(f"/api/godot/{session_id}/status")

    assert response.status_code == 200
    payload = response.json()
    assert payload["schema_version"] == GODOT_SESSION_STATUS_SCHEMA_VERSION
    assert payload["session_id"] == session_id
    assert payload["session_state"] == "disconnected"
    assert payload["engine_running"] is False
    assert payload["tcp_connected"] is False
    assert payload["schema_available"] is False
    assert payload["last_sensor"] == {}


def test_session_bridge_launch_without_godot_exe_returns_real_error(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    session_id = f"launch-{uuid.uuid4().hex}"
    monkeypatch.setattr(GodotBridge, "_find_godot_exe", lambda self: "")

    response = client.post(f"/api/godot/{session_id}/launch", json={})

    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "error"
    assert payload["session_state"] == "failed"
    assert payload["session"]["failure_stage"] == "launch"
    assert "Godot" in payload["message"]


def test_session_bridge_control_without_session_returns_error(
    client: TestClient,
) -> None:
    session_id = f"control-{uuid.uuid4().hex}"

    response = client.post(
        f"/api/godot/{session_id}/control",
        json={"action": [0.1, -0.1]},
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "error"
    assert payload["session_state"] == "disconnected"
    assert payload["session"]["session_state"] == "disconnected"
    assert "Session" in payload["message"]


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
    assert bridge.get_status_payload()["session_state"] == "launching"


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


async def _run_session_bridge_get_sensors_polls_with_step(
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
    assert bridge.get_status_payload()["session_state"] == "running"


async def _run_session_bridge_schema_state(monkeypatch: pytest.MonkeyPatch) -> None:
    bridge = GodotBridge("schema-test", 9001)

    async def fake_send_recv(payload):
        if payload["type"] == "get_schema":
            return {"sensors": {}, "actuators": {}}
        return {}

    monkeypatch.setattr(bridge, "_send_recv", fake_send_recv)

    schema = await bridge.wait_until_schema(timeout_seconds=0.1)

    assert schema == {"sensors": {}, "actuators": {}}
    status = bridge.get_status_payload()
    assert status["session_state"] == "schema_ready"
    assert status["schema_available"] is True
    assert sorted(status["schema"]) == ["actuators", "sensors"]


def test_session_bridge_get_sensors_polls_with_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    asyncio.run(_run_session_bridge_get_sensors_polls_with_step(monkeypatch))


def test_session_bridge_schema_state(monkeypatch: pytest.MonkeyPatch) -> None:
    asyncio.run(_run_session_bridge_schema_state(monkeypatch))


async def _run_session_bridge_send_recv_uses_little_endian() -> None:
    bridge = GodotBridge("framing-test", 9002)
    request = {"type": "get_schema"}
    response = {"sensors": {}, "actuators": {}}
    response_body = json.dumps(response).encode("utf-8")

    class FakeWriter:
        def __init__(self) -> None:
            self.buffer = b""

        def write(self, data: bytes) -> None:
            self.buffer += data

        async def drain(self) -> None:
            return None

        def is_closing(self) -> bool:
            return False

    class FakeReader:
        def __init__(self) -> None:
            self.parts = [
                struct.pack("<I", len(response_body)),
                response_body,
            ]

        async def readexactly(self, size: int) -> bytes:
            chunk = self.parts.pop(0)
            assert len(chunk) == size
            return chunk

    bridge.writer = FakeWriter()
    bridge.reader = FakeReader()

    payload = await bridge._send_recv(request)

    request_body = json.dumps(request).encode("utf-8")
    assert bridge.writer.buffer[:4] == struct.pack("<I", len(request_body))
    assert bridge.writer.buffer[4:] == request_body
    assert payload == response


def test_session_bridge_send_recv_uses_little_endian() -> None:
    asyncio.run(_run_session_bridge_send_recv_uses_little_endian())
