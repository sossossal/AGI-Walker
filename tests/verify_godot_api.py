"""
Low-level Godot TCP API verification.

Validates the framed JSON protocol directly against MockGodotServer without
going through the higher-level GodotSimulationClient wrapper.
"""

import json
import logging
import socket
import struct
import sys
import time
from pathlib import Path


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.comm.godot_client import MockGodotServer


def _recv_exactly(sock: socket.socket, size: int) -> bytes:
    data = b""
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise ConnectionError("Socket closed while receiving framed payload")
        data += chunk
    return data


def _send_message(sock: socket.socket, payload: dict) -> None:
    body = json.dumps(payload).encode("utf-8")
    sock.sendall(struct.pack("<I", len(body)))
    sock.sendall(body)


def _recv_message(sock: socket.socket) -> dict:
    length = struct.unpack("<I", _recv_exactly(sock, 4))[0]
    body = _recv_exactly(sock, length)
    return json.loads(body.decode("utf-8"))


def verify_api(host: str = "127.0.0.1", port: int = 0) -> bool:
    logger.info("=== Godot TCP API Verification ===")
    server = MockGodotServer(port=port)
    if not server.start():
        logger.error("FAIL: Could not start mock server")
        return False
    port = server.port

    sock: socket.socket | None = None
    try:
        time.sleep(0.5)
        logger.info("Connecting to Mock Godot at %s:%s...", host, port)
        sock = socket.create_connection((host, port), timeout=2.0)
        logger.info("PASS: TCP connection established")

        logger.info("[1/3] Sending load_robot command...")
        _send_message(
            sock,
            {
                "command": "load_robot",
                "data": {
                    "parts": [{"id": "motor_1", "type": "motor"}],
                    "connections": [],
                },
                "timestamp": time.time(),
            },
        )
        logger.info("PASS: load_robot command sent")

        logger.info("[2/3] Sending start_sim command...")
        _send_message(
            sock,
            {
                "command": "start_sim",
                "data": {
                    "robot": {
                        "parts": [{"id": "motor_1", "type": "motor"}],
                        "connections": [],
                    },
                    "physics": {"gravity": 9.81, "timestep": 0.01},
                },
                "timestamp": time.time(),
            },
        )
        logger.info("PASS: start_sim command sent")

        logger.info("[3/3] Waiting for simulation_data frame...")
        payload = _recv_message(sock)
        if payload.get("type") != "simulation_data":
            raise AssertionError(f"Unexpected payload type: {payload}")
        if "position" not in payload or "velocity" not in payload:
            raise AssertionError(f"Incomplete telemetry payload: {payload}")
        logger.info(
            "PASS: Received simulation_data frame with position=%s velocity=%s",
            payload["position"],
            payload["velocity"],
        )

        logger.info("PASS: Godot TCP API verification completed")
        return True
    except Exception as exc:
        logger.exception("FAIL: %s", exc)
        return False
    finally:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        server.stop()


if __name__ == "__main__":
    raise SystemExit(0 if verify_api() else 1)
