"""
Virtual Ganglion (Zone Controller) Simulator
Version: v1.0 (MVP)

This script simulates a Zone Controller (Ganglion) running on a PC.
It implements the full OpenNeuro communication stack:
- Zenoh connection
- Zone-specific Topics
- Mock Sensor Data
- Mock IMC-22 Inference
"""

import zenoh
import time
import json
import threading
import random
import math
import sys

class VirtualGanglion:
    def __init__(self, zone_id, zone_name):
        self.zone_id = zone_id
        self.zone_name = zone_name
        self.running = False
        
        # OpenNeuro Topics (no leading slash for Zenoh compatibility)
        self.base_topic = f"zone/{self.zone_id}"
        self.cmd_topic = f"{self.base_topic}/cmd"
        self.state_topic = f"{self.base_topic}/state"
        self.imc_topic = f"{self.base_topic}/imc22/output"
        
        # Internal State
        self.joints = [0.0] * 6  # 6 joints per zone (typical)
        self.target_joints = [0.0] * 6
        self.temperature = 35.0
        self.voltage = 24.0
        
        # Zenoh Session
        print(f"[{self.zone_name}] Initializing Zenoh...")
        self.session = zenoh.open(zenoh.Config())
        
        # Publishers
        self.pub_state = self.session.declare_publisher(self.state_topic)
        self.pub_imc = self.session.declare_publisher(self.imc_topic)
        
        # Subscribers
        self.sub_cmd = self.session.declare_subscriber(self.cmd_topic, self.on_cmd)
        
        print(f"[{self.zone_name}] Online! ID: {self.zone_id}")

    def on_cmd(self, sample):
        """Handle control commands"""
        try:
            payload_bytes = bytes(sample.payload)
            payload = json.loads(payload_bytes.decode('utf-8'))
            print(f"[{self.zone_name}] Received CMD: {payload}")
            
            if 'targets' in payload:
                self.target_joints = payload['targets']
                
        except Exception as e:
            print(f"[{self.zone_name}] CMD Error: {e}")

    def simulate_physics(self, dt):
        """Simulate motor physics and sensor noise"""
        for i in range(len(self.joints)):
            # Simple P-Controller simulation
            error = self.target_joints[i] - self.joints[i]
            self.joints[i] += error * 0.1
            
            # Add noise
            self.joints[i] += random.uniform(-0.001, 0.001)

    def simulate_imc22(self):
        """Simulate local AI inference (IMC-22)"""
        # In a real scenario, this would talk to the NPU
        # Here we simulate a "Health Check" AI model output
        inference_result = {
            "anomaly_score": random.random() * 0.1,
            "prediction": "NORMAL" if random.random() > 0.01 else "WARNING",
            "latency_ms": 1.5 + random.random()
        }
        return inference_result

    def run(self):
        self.running = True
        print(f"[{self.zone_name}] Loop started.")
        
        try:
            while self.running:
                start_time = time.time()
                
                # 1. Simulate Physics
                self.simulate_physics(0.01)
                
                # 2. Publish State (100Hz)
                state_msg = {
                    "timestamp": time.time(),
                    "joints": self.joints,
                    "metrics": {
                        "temp": self.temperature,
                        "batt": self.voltage
                    }
                }
                self.pub_state.put(json.dumps(state_msg))
                
                # 3. IMC-22 Inference (Simulated 10Hz)
                if random.random() < 0.1:
                    imc_msg = self.simulate_imc22()
                    self.pub_imc.put(json.dumps(imc_msg))
                
                # Sleep to maintain ~100Hz loop
                elapsed = time.time() - start_time
                sleep_time = max(0, 0.01 - elapsed)
                time.sleep(sleep_time)
                
        except KeyboardInterrupt:
            print(f"[{self.zone_name}] Stopping...")
        finally:
            self.session.close()

if __name__ == "__main__":
    # Create a Virtual Left Arm Controller (Zone 1)
    ganglion = VirtualGanglion(zone_id=1, zone_name="LeftArm")
    ganglion.run()
