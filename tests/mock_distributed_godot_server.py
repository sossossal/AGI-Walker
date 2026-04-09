"""
Minimal mock Godot TCP server for distributed smoke tests.

Implements the small length-prefixed JSON protocol expected by
distributed/sidecar.py and emits observation payloads after reset/step.
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import struct
from typing import Any


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [MockGodot] %(message)s",
)
logger = logging.getLogger("mock_godot")


def _recv_exactly(client: socket.socket, size: int) -> bytes | None:
    data = b""
    while len(data) < size:
        chunk = client.recv(size - len(data))
        if not chunk:
            return None
        data += chunk
    return data


def _send_observation(
    client: socket.socket,
    *,
    actor_id: str,
    reset_count: int,
    step_count: int,
    last_message_type: str,
    last_action: list[float],
) -> None:
    observation: dict[str, Any] = {
        "robot": {
            "actor_id": actor_id,
            "ready": True,
        },
        "telemetry": {
            "reset_count": reset_count,
            "step_count": step_count,
            "last_message_type": last_message_type,
            "last_action_size": len(last_action),
        },
        "sensors": {
            "imu": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "battery": 99.5,
        },
    }
    payload = json.dumps(observation).encode("utf-8")
    client.sendall(struct.pack("<I", len(payload)))
    client.sendall(payload)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a mock Godot TCP server.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--actor-id", default="actor_docker_1")
    args = parser.parse_args()

    reset_count = 0
    step_count = 0
    last_action: list[float] = []

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((args.host, args.port))
        server_socket.listen(1)
        logger.info(
            "listening on %s:%s for actor %s", args.host, args.port, args.actor_id
        )

        client, address = server_socket.accept()
        logger.info("client connected from %s:%s", address[0], address[1])
        with client:
            while True:
                length_bytes = _recv_exactly(client, 4)
                if not length_bytes:
                    logger.info("client disconnected")
                    return 0

                payload_length = struct.unpack("<I", length_bytes)[0]
                payload_bytes = _recv_exactly(client, payload_length)
                if not payload_bytes:
                    logger.info("client disconnected during payload read")
                    return 0

                message = json.loads(payload_bytes.decode("utf-8"))
                message_type = str(message.get("type", "unknown"))

                if message_type == "reset":
                    reset_count += 1
                    logger.info("received reset #%s", reset_count)
                elif message_type == "step":
                    step_count += 1
                    last_action = list(message.get("action", []))
                    logger.info(
                        "received step #%s with %s actions",
                        step_count,
                        len(last_action),
                    )
                else:
                    logger.info("received message type %s", message_type)

                _send_observation(
                    client,
                    actor_id=args.actor_id,
                    reset_count=reset_count,
                    step_count=step_count,
                    last_message_type=message_type,
                    last_action=last_action,
                )


if __name__ == "__main__":
    raise SystemExit(main())
