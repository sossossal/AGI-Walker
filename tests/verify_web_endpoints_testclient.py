
from fastapi.testclient import TestClient
import sys
import os
from unittest.mock import MagicMock

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Mock godot_controller BEFORE importing server
import web_panel.server
from web_panel.godot_controller import GodotController

# Mock the client inside GodotController
mock_client = MagicMock()
mock_client.connect.return_value = True
mock_client.start_simulation.return_value = True
mock_client.stop_simulation.return_value = True
mock_client.is_connected.return_value = True # Simulate connected state for control tests
web_panel.godot_controller.godot_controller.client = mock_client

client = TestClient(web_panel.server.app)

def test_endpoints():
    print("Testing Godot Endpoints...")
    
    # 1. Connect
    print("1. Testing /api/godot/connect...")
    resp = client.post("/api/godot/connect", json={"host": "127.0.0.1", "port": 9999})
    print(f"   Status: {resp.status_code}")
    print(f"   Response: {resp.json()}")
    assert resp.status_code == 200
    assert resp.json()["status"] == "connected"

    # 2. Status
    print("2. Testing /api/godot/status...")
    resp = client.get("/api/godot/status")
    print(f"   Response: {resp.json()}")
    assert resp.status_code == 200
    assert resp.json()["connected"] == True

    # 3. Start
    print("3. Testing /api/godot/start...")
    resp = client.post("/api/godot/start", json={"physics": {"gravity": 9.8}})
    print(f"   Status: {resp.status_code}")
    assert resp.status_code == 200
    assert resp.json()["status"] == "started"

    # 4. Stop
    print("4. Testing /api/godot/stop...")
    resp = client.post("/api/godot/stop")
    print(f"   Status: {resp.status_code}")
    assert resp.status_code == 200

    print("\n✅ All endpoints verified reachable and defined.")

if __name__ == "__main__":
    test_endpoints()
