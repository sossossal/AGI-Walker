import requests
import time
import threading
import sys
import os

# Import mock server from godot_client
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from python_api.comm.godot_client import MockGodotServer

API_BASE = "http://localhost:8000/api/godot"

def test_web_godot_integration():
    print("🚀 Starting Web-Godot Integration Test...")
    
    # 1. Start Mock Godot Server
    mock_server = MockGodotServer(port=9999)
    if not mock_server.start():
        print("Failed to start mock server")
        return
    
    try:
        time.sleep(1)
        
        # 2. Test Connect
        print("\n[1] Testing Connect...")
        res = requests.post(f"{API_BASE}/connect", json={"host": "127.0.0.1", "port": 9999})
        print(f"Response: {res.json()}")
        assert res.status_code == 200 and res.json()['status'] == 'connected'
        
        # 3. Test Status
        print("\n[2] Testing Status...")
        res = requests.get(f"{API_BASE}/status")
        print(f"Response: {res.json()}")
        assert res.json()['connected'] == True
        
        # 4. Test Start Sim
        print("\n[3] Testing Start Simulation...")
        res = requests.post(f"{API_BASE}/start", json={"physics": {"gravity": 5.0}})
        print(f"Response: {res.json()}")
        assert res.json()['status'] == 'started'
        
        # 5. Test Update Params
        print("\n[4] Testing Update Params...")
        res = requests.post(f"{API_BASE}/update-params", json={"motor_power": 2.0})
        print(f"Response: {res.json()}")
        assert res.json()['status'] == 'updated'
        
        # 6. Test Stop Sim
        print("\n[5] Testing Stop Simulation...")
        res = requests.post(f"{API_BASE}/stop")
        print(f"Response: {res.json()}")
        assert res.json()['status'] == 'stopped'
        
        # 7. Test Disconnect
        print("\n[6] Testing Disconnect...")
        res = requests.post(f"{API_BASE}/disconnect")
        print(f"Response: {res.json()}")
        assert res.json()['status'] == 'disconnected'
        
        print("\n✅ All Godot Web APIs verified successfully!")
        
    except Exception as e:
        print(f"\n❌ Test Failed: {e}")
    finally:
        mock_server.stop()

if __name__ == "__main__":
    test_web_godot_integration()
