import gymnasium as gym
import numpy as np
import socket
import json
import struct
import time
from gymnasium import spaces


class DreamerEnv(gym.Env):
    """
    OpenAI Gym/Gymnasium compatible environment for AGI-Walker.
    Connects to Godot via TCP.
    """

    def __init__(self, host="127.0.0.1", port=9000, img_size=(64, 64)):
        super().__init__()
        self.host = host
        self.port = port
        self.img_size = img_size
        self.sock = None
        self.frame_count = 0

        # Define Observation Space
        # Image: (C, H, W) or (H, W, C) - Dreamer usually expects channels first or handles it?
        # Let's stick to (H, W, 3) uint8 for standard Gym
        self.observation_space = spaces.Dict(
            {
                "image": spaces.Box(
                    low=0, high=255, shape=(img_size[0], img_size[1], 3), dtype=np.uint8
                ),
                "vector": spaces.Box(
                    low=-np.inf, high=np.inf, shape=(24,), dtype=np.float32
                ),  # example size
            }
        )

        # Define Action Space (Continuous Torques/Positions)
        # Assuming 12 DOF robot
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(12,), dtype=np.float32
        )

    def connect(self):
        if self.sock:
            return

        print(f"🔌 Connecting to Godot at {self.host}:{self.port}...")
        retry_count = 0
        while retry_count < 5:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self.sock.setsockopt(
                    socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
                )  # Low latency
                print("✅ Connected to Godot Sim.")
                return
            except ConnectionRefusedError:
                print("   Connection refused, retrying in 1s...")
                time.sleep(1.0)
                retry_count += 1

        raise ConnectionError("Could not connect to Godot Simulator.")

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.connect()

        # Send Reset Command
        self.send_command({"type": "reset"})

        # Receive Initial Observation
        obs = self.receive_obs()
        return obs, {}

    def step(self, action):
        start_time = time.time()

        # Send Action
        # Convert numpy to list
        cmd = {"type": "step", "action": action.tolist()}
        self.send_command(cmd)

        # Receive Observation
        obs = self.receive_obs()

        # Reward & Done usually come from Sim or calculated here
        reward = obs.pop("reward", 0.0)
        done = obs.pop("done", False)
        truncated = False

        info = {
            "latency": (time.time() - start_time) * 1000.0  # ms
        }

        return obs, reward, done, truncated, info

    def send_command(self, cmd_dict):
        # Protocol: Length Prefix (4 bytes) + JSON
        data = json.dumps(cmd_dict).encode("utf-8")
        length = struct.pack("<I", len(data))
        self.sock.sendall(length + data)

    def receive_obs(self):
        # Protocol: Length Prefix (4 bytes) + JSON
        # Read Length
        len_bytes = self._recv_all(4)
        if not len_bytes:
            raise ConnectionError("Socket closed during read.")

        length = struct.unpack("<I", len_bytes)[0]

        # Read Data
        data = self._recv_all(length)
        obs_json = json.loads(data.decode("utf-8"))

        # Parse into Gym format
        # Mocking Image for now if not sent
        if "image" not in obs_json:
            obs_json["image"] = np.zeros(
                (self.img_size[0], self.img_size[1], 3), dtype=np.uint8
            )
        else:
            # Decode Base64 or raw bytes if needed
            # For this MVP, we assume it's missing or mock
            pass

        if "vector" not in obs_json:
            obs_json["vector"] = np.zeros(24, dtype=np.float32)
        else:
            obs_json["vector"] = np.array(obs_json["vector"], dtype=np.float32)

        return obs_json

    def _recv_all(self, n):
        data = b""
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
