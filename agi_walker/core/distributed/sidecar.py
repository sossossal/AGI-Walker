import argparse
import json
import os
import socket
import struct
import time
import zenoh
import logging
from .scheduler import OffloadingScheduler

# Configure Logging
logger = logging.getLogger("sidecar")


class GodotClient:
    """Enhanced Godot TCP Client with V2.0 performance optimizations."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((self.host, self.port))
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                logger.info(f"✅ Connected to Godot at {self.host}:{self.port}")
                return
            except ConnectionRefusedError:
                time.sleep(2)
            except Exception as e:
                logger.error(f"Connection failed: {e}")
                time.sleep(2)

    def send_action(self, action_dict):
        if not self.sock:
            return
        try:
            data = json.dumps(action_dict).encode("utf-8")
            length = struct.pack("<I", len(data))
            self.sock.sendall(length + data)
        except Exception:
            self.reconnect()

    def receive_obs(self):
        if not self.sock:
            return None
        try:
            len_bytes = self._recv_all(4)
            if not len_bytes:
                return None
            length = struct.unpack("<I", len_bytes)[0]
            data = self._recv_all(length)
            if not data:
                return None
            return json.loads(data.decode("utf-8"))
        except Exception:
            self.reconnect()
            return None

    def _recv_all(self, n):
        data = b""
        while len(data) < n:
            packet = self.sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data

    def reconnect(self):
        if self.sock:
            self.sock.close()
        self.connect()


class SmartSidecar:
    """
    AGI-Walker V2.5 Smart Sidecar Agent.
    Implements Cloud-Edge Dynamic Offloading.
    """

    def __init__(self, actor_id, godot_host, godot_port, zenoh_router):
        self.id = actor_id
        self.scheduler = OffloadingScheduler()
        self.godot = GodotClient(godot_host, godot_port)

        # Zenoh Setup
        conf = zenoh.Config()
        if zenoh_router:
            conf.insert_json5("connect/endpoints", f'["{zenoh_router}"]')
        self.session = zenoh.open(conf)

        self.key_obs = f"ag/{actor_id}/obs"
        self.key_act = f"ag/{actor_id}/act"
        self.pub_obs = self.session.declare_publisher(self.key_obs)
        self.sub_act = self.session.declare_subscriber(
            self.key_act, self._on_cloud_action
        )

        self.last_obs_time = 0.0

    def _on_cloud_action(self, sample):
        """Handle AI inference results from Cloud."""
        # Calculate RTT for scheduler
        rtt = (time.time() - self.last_obs_time) * 1000
        self.scheduler.update_cloud_stats(rtt, available=True)

        payload = json.loads(bytes(sample.payload).decode("utf-8"))
        cmd = {"type": "step", "action": payload.get("action", [])}
        self.godot.send_action(cmd)
        logger.debug(f"Cloud Act Executed (RTT: {rtt:.1f}ms)")

    def run(self):
        self.godot.connect()
        logger.info(f"Smart Sidecar {self.id} initialized. Link: {self.key_obs}")

        # Initial Reset
        self.godot.send_action({"type": "reset"})

        try:
            while True:
                obs = self.godot.receive_obs()
                if not obs:
                    continue

                self.last_obs_time = time.time()

                if self.scheduler.should_offload():
                    # Link B: Cloud Offloading via Zenoh
                    self.pub_obs.put(json.dumps(obs))
                else:
                    # Link A: Local Execution (Fallback/Normal)
                    # For now, we simulate a very fast local control response
                    # or forward to a local inference process
                    logger.debug("Routing to Local AI Inference...")
                    # self.godot.send_action(local_inference(obs))
                    pass

        except KeyboardInterrupt:
            self.session.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--id", type=str, default="actor1")
    parser.add_argument(
        "--godot-host",
        type=str,
        default=os.environ.get("AGI_WALKER_GODOT_HOST", "127.0.0.1"),
    )
    parser.add_argument("--godot-port", type=int, default=9000)
    parser.add_argument(
        "--zenoh-router", type=str, default=os.environ.get("ZENOH_ROUTER")
    )
    args = parser.parse_args()

    agent = SmartSidecar(
        args.id,
        args.godot_host,
        args.godot_port,
        args.zenoh_router,
    )
    agent.run()


if __name__ == "__main__":
    main()
