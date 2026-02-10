import time
import numpy as np
import json
import os
from typing import List, Dict, Tuple

class DataCollector:
    """
    Collects trajectory data for System Identification.
    Executes excitation trajectories and records state.
    """
    def __init__(self, robot_interface, save_dir="sysid/data"):
        self.robot = robot_interface
        self.save_dir = save_dir
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            
    def generate_excitation_trajectory(self, duration: float = 5.0, dt: float = 0.02):
        """Generates a multi-sine excitation trajectory for all joints."""
        steps = int(duration / dt)
        t = np.linspace(0, duration, steps)
        
        # Simple sine wave for now: A * sin(omega * t)
        # In real SysID, we'd use diverse frequencies.
        trajectory = []
        for i in range(steps):
            cmd = {
                "hip_left": 0.5 * np.sin(2.0 * t[i]),
                "hip_right": 0.5 * np.sin(2.0 * t[i] + np.pi), # Antiphase
                "knee_left": 0.3 * np.sin(3.0 * t[i]),
                "knee_right": 0.3 * np.sin(3.0 * t[i])
            }
            trajectory.append(cmd)
        return trajectory

    def collect_data(self, duration: float = 5.0) -> str:
        """
        Executes trajectory and records data.
        Returns path to saved data file.
        """
        print(f"📡 Starting Data Collection ({duration}s)...")
        traj = self.generate_excitation_trajectory(duration)
        
        recorded_data = {
            "metadata": {"duration": duration, "timestamp": time.time()},
            "measurements": []
        }
        
        # Reset Robot
        self.robot.reset()
        time.sleep(1.0) # Settle
        
        start_time = time.time()
        for cmd in traj:
            # Send Command
            self.robot.send_command(cmd)
            
            # Read State
            state = self.robot.get_state()
            
            # Record
            recorded_data["measurements"].append({
                "time": time.time() - start_time,
                "command": cmd,
                "state": state
            })
            
            # Sync (approximate)
            time.sleep(0.02)
            
        # Save
        filename = f"sysid_log_{int(time.time())}.json"
        filepath = os.path.join(self.save_dir, filename)
        with open(filepath, 'w') as f:
            json.dump(recorded_data, f, indent=2)
            
        print(f"✅ Data saved to {filepath}")
        return filepath

# Mock Robot Interface for testing
class MockRobotInterface:
    def reset(self):
        print("Robot Reset")
        
    def send_command(self, cmd):
        pass
        
    def get_state(self):
        return {
            "joints": {"hip_left": 0.0, "hip_right": 0.0},
            "velocity": {"hip_left": 0.0, "hip_right": 0.0}
        }

if __name__ == "__main__":
    collector = DataCollector(MockRobotInterface())
    collector.collect_data(2.0)
