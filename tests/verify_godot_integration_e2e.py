"""
Automated Verification Script for Godot Integration DoD.
Implements the "Section IV" automated verification suggestion.

Scenario:
1. Start MockGodotServer (Background Thread)
2. Initialize GodotSimulationClient
3. Connect to Server
4. Send Robot Configuration (load_robot)
5. Start Simulation (start_sim)
6. Update Parameters (update_params)
7. Verify Data Reception (via callback)
8. Stop Simulation (stop_sim)
9. Disconnect
"""
import sys
import os
import time
import threading
import queue
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from python_api.godot_client import GodotSimulationClient, MockGodotServer

def verify_integration():
    print("=== Godot Integration DoD Verification ===")
    
    # 1. Start Mock Server
    print("[1] Starting Mock Godot Server...")
    server = MockGodotServer(port=9998) # Use different port to avoid conflict
    try:
        if not server.start():
            print("FAIL: Could not start mock server")
            return False
    except Exception as e:
        print(f"FAIL: Server start exception: {e}")
        return False
        
    time.sleep(1) # Wait for server
    
    # 2. Initialize Client
    print("[2] Initializing Client...")
    client = GodotSimulationClient(port=9998)
    
    # 3. Connect
    print("[3] Connecting to Server...")
    if not client.connect():
        print("FAIL: Could not connect to server")
        server.stop()
        return False
    print("PASS: Connected")
    
    # Data reception queue
    data_queue = queue.Queue()
    def on_data(data):
        data_queue.put(data)
    client.set_data_callback(on_data)
    
    success = True
    
    try:
        # 4. Load Robot
        print("[4] Loading Robot Config...")
        parts = [{'id': 'motor_1', 'type': 'motor'}]
        connections = []
        if client.load_robot_config(parts, connections):
            print("PASS: load_robot command sent")
        else:
            print("FAIL: load_robot command failed")
            success = False
            
        time.sleep(0.5)
        
        # 5. Start Simulation
        print("[5] Starting Simulation...")
        robot_config = {'parts': parts, 'connections': connections}
        if client.start_simulation(robot_config):
            print("PASS: start_sim command sent")
        else:
            print("FAIL: start_sim command failed")
            success = False
            
        # 6. Verify Data Reception (wait up to 2s)
        print("[6] Verifying Data stream...")
        try:
            # We expect multiple frames
            frames_received = 0
            for _ in range(5):
                data = data_queue.get(timeout=2.0)
                if data.get('type') == 'simulation_data':
                    frames_received += 1
            
            if frames_received > 0:
                print(f"PASS: Received {frames_received}+ data frames")
            else:
                print("FAIL: No simulation_data received")
                success = False
        except queue.Empty:
            print("FAIL: Data reception timeout")
            success = False
            
        # 7. Update Parameters
        print("[7] Updating Parameters...")
        if client.update_parameters({'motor_power': 1.0}):
            print("PASS: update_params command sent")
        else:
            print("FAIL: update_params command failed")
            success = False
            
        time.sleep(0.5)
        
        # 8. Stop Simulation
        print("[8] Stopping Simulation...")
        if client.stop_simulation():
            print("PASS: stop_sim command sent")
        else:
            print("FAIL: stop_sim command failed")
            success = False
            
    except Exception as e:
        print(f"FAIL: Exception during verification: {e}")
        success = False
    finally:
        # 9. Disconnect
        print("[9] Disconnecting...")
        client.disconnect()
        server.stop()
        
    print("\n=== Verification Result ===")
    if success:
        print("✅ ALL CHECKS PASSED")
    else:
        print("❌ VERIFICATION FAILED")
        
    return success

if __name__ == "__main__":
    if verify_integration():
        sys.exit(0)
    else:
        sys.exit(1)
