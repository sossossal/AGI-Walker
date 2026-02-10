import numpy as np
import requests
import time
import json

class SimulatorInterface:
    def set_params(self, friction, damping):
        raise NotImplementedError
        
    def run_trajectory(self, duration=1.0):
        raise NotImplementedError

class MockSimulator(SimulatorInterface):
    def __init__(self):
        self.friction = 0.5
        self.damping = 0.1
        self.target_friction = 1.2
        self.target_damping = 0.5
        
    def set_params(self, friction, damping):
        self.friction = friction
        self.damping = damping
        
    def run_trajectory(self, duration=1.0):
        # Simulation Logic matchin optimizer.py mock
        steps = int(duration * 50) # 50Hz
        data = {"measurements": []}
        for i in range(steps):
             t = i * 0.02
             ideal_pos = np.sin(t)
             
             # Friction influence: deviations based on parameter error
             distortion = (self.friction - self.target_friction) * 0.5 + (self.damping - self.target_damping) * 0.2
             sim_pos = ideal_pos + distortion
             
             data["measurements"].append({
                 "state": {"joints": {"hip_left": sim_pos}}
             })
        return data

class RemoteSimulator(SimulatorInterface):
    """
    Connects to Godot via Web Panel API.
    """
    def __init__(self, api_url="http://localhost:8000"):
        self.api_url = api_url
        
    def set_params(self, friction, damping):
        payload = {
            "friction": friction,
            "damping": damping
        }
        try:
            # Requires /api/godot/update_physics endpoint (to be implemented)
            res = requests.post(f"{self.api_url}/api/godot/update_physics", json=payload)
            if res.status_code != 200:
                print(f"⚠️ Failed to update physics: {res.text}")
        except Exception as e:
            print(f"⚠️ Connection error: {e}")
            
    def run_trajectory(self, duration=1.0):
        # 1. Reset Simulation
        requests.post(f"{self.api_url}/api/godot/reset")
        time.sleep(0.5)
        
        # 2. Start Recording via DataCollector logic 
        # (This class might need to coordinate with DataCollector)
        # For now, return empty or mock structure
        return {"measurements": []}
