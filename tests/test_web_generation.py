import json
import xml.etree.ElementTree as ET
from pathlib import Path

from fastapi.testclient import TestClient

import web_panel.server
import web_panel.services_api as services_api

EXPECTED_LINKS = [
    "torso_1",
    "thigh_left_2",
    "shin_left_3",
    "thigh_right_4",
    "shin_right_5",
]
EXPECTED_JOINTS = ["hip_left", "knee_left", "hip_right", "knee_right"]


def _cleanup_generated_files(robot_name: str) -> tuple[Path, Path]:
    config_path = Path("configs/generated") / f"{robot_name}.json"
    urdf_path = Path("exports") / f"{robot_name}.urdf"
    for path in (config_path, urdf_path):
        if path.exists():
            path.unlink()
    return config_path, urdf_path


def _assert_generated_robot_artifacts(config_path: Path, urdf_path: Path) -> None:
    assert config_path.exists()
    assert urdf_path.exists()

    config = json.loads(config_path.read_text(encoding="utf-8"))
    metadata = config.get("metadata", {})
    advanced_dynamics = metadata.get("advanced_dynamics", {})

    assert config["name"] == config_path.stem
    assert metadata["is_real_world_config"] is True
    assert "estimated_cost_usd" in metadata
    assert "bom" in metadata and len(metadata["bom"]) >= 1
    assert advanced_dynamics["stiffness"] > 0
    assert advanced_dynamics["backlash"] >= 0
    assert "thermal_resistance" in advanced_dynamics

    root = ET.parse(urdf_path).getroot()
    links = [link.attrib["name"] for link in root.findall("link")]
    joints = [joint.attrib["name"] for joint in root.findall("joint")]
    cylinders = root.findall(".//cylinder")

    for expected_link in EXPECTED_LINKS:
        assert any(expected_link in found for found in links)
    for expected_joint in EXPECTED_JOINTS:
        assert any(expected_joint in found for found in joints)
    assert len(cylinders) >= 8


def test_generate_robot_route_returns_created_files() -> None:
    client = TestClient(web_panel.server.app)
    robot_name = "web_generation_route_tmp"
    config_path, urdf_path = _cleanup_generated_files(robot_name)

    try:
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
        _assert_generated_robot_artifacts(config_path, urdf_path)
    finally:
        _cleanup_generated_files(robot_name)


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
