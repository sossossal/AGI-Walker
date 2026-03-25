import uuid

import pytest
from fastapi.testclient import TestClient

import web_panel.server
from web_panel.command_parser import CommandParser


@pytest.fixture()
def client() -> TestClient:
    return TestClient(web_panel.server.app)


@pytest.fixture(autouse=True)
def reset_server_state():
    web_panel.server.tasks_db.clear()
    web_panel.server.active_connections.clear()
    yield
    web_panel.server.tasks_db.clear()
    web_panel.server.active_connections.clear()


def test_tasks_crud_round_trip(client: TestClient):
    task_name = f"task-{uuid.uuid4().hex[:8]}"

    create_response = client.post(
        "/api/tasks",
        json={
            "name": task_name,
            "description": "integration test task",
            "priority": "high",
        },
    )
    assert create_response.status_code == 200
    created = create_response.json()
    assert created["status"] == "success"
    task_id = created["task_id"]

    get_response = client.get(f"/api/tasks/{task_id}")
    assert get_response.status_code == 200
    assert get_response.json()["task"]["name"] == task_name

    update_response = client.put(
        f"/api/tasks/{task_id}",
        json={"status": "running", "description": "updated"},
    )
    assert update_response.status_code == 200
    updated = update_response.json()
    assert updated["status"] == "success"
    assert updated["task"]["status"] == "running"
    assert updated["task"]["description"] == "updated"

    delete_response = client.delete(f"/api/tasks/{task_id}")
    assert delete_response.status_code == 200
    assert delete_response.json()["status"] == "success"

    missing_response = client.get(f"/api/tasks/{task_id}")
    assert missing_response.status_code == 404


def test_parse_command_endpoint_uses_regex_fallback(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(CommandParser, "_llm_parse", lambda self, prompt: None)

    response = client.post(
        "/api/agent/parse-command",
        json={"command": "创建一个双足机器人 高度0.62 质量7.5 叫walkerbot"},
    )

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["config"]["metadata"]["type"] == "biped"
    assert data["config"]["metadata"]["parser"] == "Regex"
    assert data["config"]["skills_params"]["torso_height"] == 0.62
    assert data["config"]["skills_params"]["torso_mass"] == 7.5


def test_legacy_godot_routes_reflect_mocked_controller_state(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(web_panel.server.godot_controller, "connect", lambda host, port: True)
    monkeypatch.setattr(web_panel.server.godot_controller, "disconnect", lambda: None)
    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: True)
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "load_robot",
        lambda parts, connections: True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "start_simulation",
        lambda physics: True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "stop_simulation",
        lambda: True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "update_params",
        lambda params: True,
    )
    monkeypatch.setattr(web_panel.server.godot_controller.client, "running", True)

    connect_response = client.post("/api/godot/connect", json={"host": "127.0.0.1", "port": 9999})
    assert connect_response.status_code == 200
    assert connect_response.json()["status"] == "connected"

    status_response = client.get("/api/godot/status")
    assert status_response.status_code == 200
    assert status_response.json() == {"connected": True, "client_running": True}

    load_response = client.post("/api/godot/load-robot", json={"parts": [], "connections": []})
    assert load_response.status_code == 200
    assert load_response.json()["status"] == "success"

    start_response = client.post("/api/godot/start", json={"physics": {"gravity": 9.81}})
    assert start_response.status_code == 200
    assert start_response.json()["status"] == "started"

    update_response = client.post("/api/godot/update-params", json={"params": {"mass": 7.5}})
    assert update_response.status_code == 200
    assert update_response.json()["status"] == "updated"

    stop_response = client.post("/api/godot/stop")
    assert stop_response.status_code == 200
    assert stop_response.json()["status"] == "stopped"

    disconnect_response = client.post("/api/godot/disconnect")
    assert disconnect_response.status_code == 200
    assert disconnect_response.json()["status"] == "disconnected"


def test_legacy_godot_load_robot_requires_connection(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: False)

    response = client.post("/api/godot/load-robot", json={"parts": [], "connections": []})

    assert response.status_code == 400
    assert response.json()["detail"] == "Godot not connected"
