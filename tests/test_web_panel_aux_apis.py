from fastapi.testclient import TestClient

from web_panel.server import app


client = TestClient(app)


def test_core_panel_routes_smoke():
    response = client.get("/")
    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"]

    response = client.get("/api/system/status")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "running"
    assert "tasks_count" in data
    assert "active_connections" in data

    response = client.get("/api/distributed/status")
    assert response.status_code == 200
    data = response.json()
    assert "actors" in data

    response = client.get("/api/godot/capabilities")
    assert response.status_code == 200
    data = response.json()
    assert "legacy_controller" in data["modes"]
    assert "session_bridge" in data["modes"]


def test_tasks_api_smoke():
    response = client.get("/api/tasks")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert isinstance(data["tasks"], list)


def test_services_api_smoke():
    response = client.get("/api/skills/list")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"


def test_sim2real_api_smoke():
    response = client.post("/api/sim2real/analyze", json={"mock": True})
    assert response.status_code == 200
    data = response.json()
    assert data["status"] in {"success", "error"}


def test_parts_market_api_smoke():
    response = client.get("/api/parts/market")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert isinstance(data["parts"], list)


def test_godot_skills_endpoints_smoke():
    response = client.get("/api/godot_skills/list")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert any(skill.get("id") == "biped_3d_robot" for skill in data["skills"])

    response = client.post("/api/godot_skills/apply", json={"skill_id": "biped_3d_robot"})
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["data"]["id"] == "biped_3d_robot"
