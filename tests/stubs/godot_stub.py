#!/usr/bin/env python
"""
Lightweight Godot-like stub used for local headless smoke verification when
the real Godot binary is unavailable. It mimics the minimal TCP protocol
expected by `GodotBridge` so the headless smoke test can run end-to-end.
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import struct
import sys
import threading
import time
from typing import Any, Dict, Optional

logger = logging.getLogger("godot_stub")
logging.basicConfig(level=logging.INFO, format="[%H:%M:%S] %(message)s")


def _parse_tcp_port(args: list[str]) -> Optional[int]:
    for arg in args:
        if arg.startswith("--tcp-port="):
            try:
                return int(arg.split("=", 1)[1])
            except ValueError:
                continue
    return None


def _recv_exact(sock: socket.socket, length: int) -> Optional[bytes]:
    data = bytearray()
    while len(data) < length:
        chunk = sock.recv(length - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


def _send_message(sock: socket.socket, payload: Dict[str, Any]) -> None:
    encoded = json.dumps(payload).encode("utf-8")
    sock.sendall(struct.pack("<I", len(encoded)))
    sock.sendall(encoded)


def _handle_client(conn: socket.socket) -> None:
    schema = {
        "sensors": {
            "pose": {"type": "vector3"},
            "imu": {"type": "vector3"},
            "instruction_runtime": {"type": "dict"},
            "simulated_circuit": {"type": "dict"},
        },
        "actuators": {
            "motors": {"size": 12},
            "instruction_set": {"type": "dict"},
            "configure_simulated_circuit": {"type": "dict"},
        },
        "meta": {
            "supported_commands": [
                "get_schema",
                "load_robot",
                "step",
                "instruction_set",
                "configure_simulated_circuit",
            ]
        },
    }
    sensor_step = 0
    latest_instruction: Dict[str, Any] = {}
    latest_circuit: Dict[str, Any] = {}
    try:
        while True:
            header = _recv_exact(conn, 4)
            if not header:
                logger.info("client disconnected")
                break
            length = struct.unpack("<I", header)[0]
            body = _recv_exact(conn, length)
            if not body:
                break
            payload = json.loads(body.decode("utf-8"))
            payload_type = payload.get("type", "")
            logger.info("received %s", payload_type)

            if payload_type == "get_schema":
                schema["meta"]["last_instruction_sequence"] = latest_instruction.get(
                    "sequence_name", ""
                )
                schema["meta"]["simulated_circuit_transport"] = latest_circuit.get(
                    "transport", ""
                )
                response = schema
            elif payload_type == "load_robot":
                response = {"status": "success", "message": "robot loaded"}
            elif payload_type == "instruction_set":
                latest_instruction = payload.get("instruction_set", {})
                response = {
                    "status": "success",
                    "sequence_name": latest_instruction.get("sequence_name", ""),
                    "instruction_step_count": len(latest_instruction.get("steps", [])),
                    "simulated_circuit_command_batch": payload.get(
                        "simulated_circuit_command_batch", []
                    ),
                    "compatibility_params": payload.get("compatibility_params", {}),
                }
            elif payload_type == "configure_simulated_circuit":
                latest_circuit = payload.get("simulated_circuit", {})
                response = {
                    "status": "success",
                    "simulated_circuit": latest_circuit,
                }
            elif payload_type == "step":
                sensor_step += 1
                response = {
                    "status": "ok",
                    "step": sensor_step,
                    "sensors": {
                        "pose": [0.0, 0.0, 0.0],
                        "imu": [0.0, 0.0, 0.0],
                        "timestamp": time.time(),
                    },
                    "instruction_runtime": {
                        "sequence_name": latest_instruction.get("sequence_name", ""),
                        "step_count": len(latest_instruction.get("steps", [])),
                    },
                    "simulated_circuit": {
                        "configured": bool(latest_circuit),
                        "transport": latest_circuit.get("transport", ""),
                    },
                }
            else:
                response = {"status": "ok"}

            _send_message(conn, response)
    except Exception as exc:  # pragma: no cover - best-effort stub
        logger.warning("client handler error: %s", exc)
    finally:
        conn.close()


def _start_server(port: int) -> None:
    logger.info("godot stub listening on tcp port %s", port)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", port))
        listener.listen(1)
        conn, addr = listener.accept()
        logger.info("accepted headless connection from %s", addr)
        _handle_client(conn)


def main() -> None:
    parser = argparse.ArgumentParser(description="Stub Godot headless server")
    parser.add_argument(
        "--version", action="store_true", help="Print emulated Godot version"
    )
    args, unknown = parser.parse_known_args()
    if args.version:
        print("Godot Engine 4.2.2-stable (stub)")
        return

    tcp_port = _parse_tcp_port(unknown) or _parse_tcp_port(sys.argv) or 9000
    server_thread = threading.Thread(
        target=_start_server, args=(tcp_port,), daemon=True
    )
    server_thread.start()

    try:
        while server_thread.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("stub interrupted, shutting down")


if __name__ == "__main__":
    main()
