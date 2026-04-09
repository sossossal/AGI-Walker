from __future__ import annotations

import json
import math
import socket
import threading
import time
from typing import Any, Dict, List


def reserve_free_port() -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(("127.0.0.1", 0))
    _, port = sock.getsockname()
    sock.close()
    return port


class JsonLineGodotServer:
    """Small newline-delimited JSON server for legacy tcp_client tests."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int | None = None,
        send_interval: float = 0.02,
    ) -> None:
        self.host = host
        self.port = port or reserve_free_port()
        self.send_interval = send_interval
        self.running = False
        self.server_socket: socket.socket | None = None
        self.accept_thread: threading.Thread | None = None
        self.client_threads: List[threading.Thread] = []
        self.client_sockets: List[socket.socket] = []
        self.received_commands: List[Dict[str, Any]] = []
        self._lock = threading.Lock()

    def start(self) -> None:
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.server_socket.settimeout(0.1)
        self.running = True
        self.accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self.accept_thread.start()

    def stop(self) -> None:
        self.running = False
        if self.server_socket is not None:
            try:
                self.server_socket.close()
            except OSError:
                pass
            self.server_socket = None

        for client in list(self.client_sockets):
            try:
                client.close()
            except OSError:
                pass

        if self.accept_thread and self.accept_thread.is_alive():
            self.accept_thread.join(timeout=1.0)

        for thread in list(self.client_threads):
            if thread.is_alive():
                thread.join(timeout=1.0)

    def _accept_loop(self) -> None:
        while self.running and self.server_socket is not None:
            try:
                client, _ = self.server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            client.settimeout(0.02)
            self.client_sockets.append(client)
            thread = threading.Thread(
                target=self._handle_client, args=(client,), daemon=True
            )
            self.client_threads.append(thread)
            thread.start()

    def _handle_client(self, client: socket.socket) -> None:
        buffer = ""
        frame_index = 0
        last_send = 0.0

        try:
            while self.running:
                now = time.time()
                if now - last_send >= self.send_interval:
                    payload = self._build_sensor_payload(frame_index)
                    client.sendall((json.dumps(payload) + "\n").encode("utf-8"))
                    frame_index += 1
                    last_send = now

                try:
                    data = client.recv(4096)
                except socket.timeout:
                    continue

                if not data:
                    break

                buffer += data.decode("utf-8")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        command = json.loads(line)
                    except json.JSONDecodeError:
                        continue
                    with self._lock:
                        self.received_commands.append(command)
        except OSError:
            pass
        finally:
            try:
                client.close()
            except OSError:
                pass

    def _build_sensor_payload(self, frame_index: int) -> Dict[str, Any]:
        roll = round(math.sin(frame_index / 8.0) * 2.5, 3)
        pitch = round(math.cos(frame_index / 10.0) * 1.5, 3)
        return {
            "timestamp": time.time(),
            "torso_height": 1.02,
            "sensors": {
                "imu": {"orient": [roll, pitch, 0.0]},
                "joints": {
                    "hip_left": {"angle": -roll, "velocity": 0.0},
                    "hip_right": {"angle": roll, "velocity": 0.0},
                },
            },
        }
