from pathlib import Path

from fastapi.testclient import TestClient

import web_panel.server
import web_panel.services_api as services_api


def test_generate_robot_route_returns_created_files() -> None:
    client = TestClient(web_panel.server.app)
    robot_name = "web_test_bot"
    config_path = Path("configs/generated") / f"{robot_name}.json"
    urdf_path = Path("exports") / f"{robot_name}.urdf"

    for path in (config_path, urdf_path):
        if path.exists():
            path.unlink()

    response = client.post(
        "/api/generate_robot",
        json={
            "name": robot_name,
            "type": "quadruped",
            "scenario": "custom",
            "height": 0.35,
            "mass": 4.5,
            "material": "carbon_fiber",
        },
    )

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["config_path"] == str(config_path).replace("\\", "/")
    assert data["urdf_path"] == str(urdf_path).replace("\\", "/")
    assert config_path.exists()
    assert urdf_path.exists()


def test_generate_robot_route_returns_http_500_on_missing_script(
    monkeypatch,
) -> None:
    client = TestClient(web_panel.server.app)
    missing = Path("tools/does-not-exist.py")

    monkeypatch.setattr(services_api, "_quick_design_script_path", lambda: missing)

    response = client.post(
        "/api/generate_robot",
        json={"name": "broken_web_test_bot", "type": "quadruped"},
    )

    assert response.status_code == 500
    data = response.json()
    assert data["status"] == "error"
    assert data["message"] == "quick_design 脚本不存在"
    assert data["error"] == str(missing)
