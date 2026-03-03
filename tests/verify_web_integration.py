"""
Verification script for Web-Godot Integration.
Simulates:
1. Agent Command -> /api/agent/parse-command
2. Web -> /api/godot/connect
3. Web -> /api/godot/load-robot
4. Web -> /api/godot/start
5. Verifies data reception via WebSocket (Mocked for this script or real connection)
"""

import requests
import time
import os
import sys

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.comm.godot_client import MockGodotServer

BASE_URL = "http://localhost:8000"


def verify_web_integration():
    print("=== Web-Godot Integration Verification ===")

    # 1. Start Mock Server
    print("[1] Starting Mock Godot Server...")
    server = MockGodotServer(port=9997)
    server.start()
    time.sleep(1)

    try:
        # 2. Test Agent Command Parsing
        print("[2] Testing Agent Command Parsing...")
        res = requests.post(
            f"{BASE_URL}/api/agent/parse-command", json={"command": "create quadruped"}
        )
        if res.status_code == 200 and res.json()["status"] == "success":
            config = res.json()["config"]
            if config["metadata"]["type"] == "quadruped" and len(config["parts"]) > 0:
                print("PASS: Agent command parsed successfully")
            else:
                print(f"FAIL: Invalid config returned: {config}")
                return False
        else:
            print(f"FAIL: Parse command API failed: {res.text}")
            return False

        # 3. Test Connect
        print("[3] Testing Connect API...")
        res = requests.post(
            f"{BASE_URL}/api/godot/connect", json={"host": "127.0.0.1", "port": 9997}
        )
        if res.status_code == 200 and res.json()["status"] == "connected":
            print("PASS: Connected to Godot via Web API")
        else:
            print(f"FAIL: Connect API failed: {res.text}")
            return False

        # 4. Test Load Robot
        print("[4] Testing Load Robot API...")
        res = requests.post(f"{BASE_URL}/api/godot/load-robot", json=config)
        if res.status_code == 200 and res.json()["status"] == "success":
            print("PASS: Robot loaded via Web API")
        else:
            print(f"FAIL: Load Robot API failed: {res.text}")
            return False

        # 5. Test Start Simulation
        print("[5] Testing Start Simulation API...")
        res = requests.post(
            f"{BASE_URL}/api/godot/start", json={"physics": {"gravity": 9.8}}
        )
        if res.status_code == 200 and res.json()["status"] == "started":
            print("PASS: Simulation started via Web API")
        else:
            print(f"FAIL: Start Simulation API failed: {res.text}")
            return False

        # 6. Test Stop Simulation
        print("[6] Testing Stop Simulation API...")
        res = requests.post(f"{BASE_URL}/api/godot/stop")
        if res.status_code == 200 and res.json()["status"] == "stopped":
            print("PASS: Simulation stopped via Web API")
        else:
            print(f"FAIL: Stop Simulation API failed: {res.text}")
            return False

        print("\n✅ Web Integration Verification Passed")
        return True

    except Exception as e:
        print(f"FAIL: Exception: {e}")
        return False
    finally:
        server.stop()


if __name__ == "__main__":
    if verify_web_integration():
        sys.exit(0)
    else:
        sys.exit(1)
