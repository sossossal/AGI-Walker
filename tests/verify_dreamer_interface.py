from __future__ import annotations

import json
import logging
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.algorithms.dreamer_env import DreamerEnv


def _configure_runtime() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")


class DreamerMockServer:
    def __init__(self, host: str = "127.0.0.1", max_steps: int = 8):
        self.host = host
        self.max_steps = max_steps
        self.port = 0
        self._server_socket: socket.socket | None = None
        self._client_socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._step_index = 0

    def start(self) -> bool:
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.host, 0))
        self._server_socket.listen(1)
        self._server_socket.settimeout(0.2)
        self.port = self._server_socket.getsockname()[1]

        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        for sock in (self._client_socket, self._server_socket):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _serve(self) -> None:
        assert self._server_socket is not None
        while not self._stop_event.is_set():
            try:
                client_socket, _ = self._server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            self._client_socket = client_socket
            client_socket.settimeout(0.2)
            try:
                self._handle_client(client_socket)
            finally:
                try:
                    client_socket.close()
                except OSError:
                    pass
                self._client_socket = None

    def _handle_client(self, client_socket: socket.socket) -> None:
        while not self._stop_event.is_set():
            command = self._recv_message(client_socket)
            if command is None:
                return
            response = self._build_response(command)
            self._send_message(client_socket, response)

    def _build_response(self, command: dict) -> dict:
        command_type = command.get("type")

        if command_type == "reset":
            self._step_index = 0
            vector = np.zeros(24, dtype=np.float32)
            return {
                "vector": vector.tolist(),
                "reward": 0.0,
                "done": False,
            }

        if command_type == "step":
            self._step_index += 1
            action = np.asarray(command.get("action", []), dtype=np.float32)
            if action.shape != (12,):
                action = np.zeros(12, dtype=np.float32)

            padded = np.concatenate([action, action])
            reward = float(np.mean(action))
            done = self._step_index >= self.max_steps
            return {
                "vector": padded.tolist(),
                "reward": reward,
                "done": done,
            }

        return {
            "vector": np.zeros(24, dtype=np.float32).tolist(),
            "reward": 0.0,
            "done": False,
        }

    def _send_message(self, client_socket: socket.socket, payload: dict) -> None:
        data = json.dumps(payload).encode("utf-8")
        client_socket.sendall(struct.pack("<I", len(data)) + data)

    def _recv_message(self, client_socket: socket.socket) -> dict | None:
        header = self._recv_all(client_socket, 4)
        if not header:
            return None

        length = struct.unpack("<I", header)[0]
        body = self._recv_all(client_socket, length)
        if body is None:
            return None
        return json.loads(body.decode("utf-8"))

    def _recv_all(self, client_socket: socket.socket, size: int) -> bytes | None:
        chunks = bytearray()
        while len(chunks) < size and not self._stop_event.is_set():
            try:
                packet = client_socket.recv(size - len(chunks))
            except socket.timeout:
                continue
            except OSError:
                return None
            if not packet:
                return None
            chunks.extend(packet)
        return bytes(chunks)


def verify_dreamer_interface(max_steps: int = 8) -> bool:
    logger.info("=== Dreamer Interface Verification ===")

    server = DreamerMockServer(max_steps=max_steps)
    env: DreamerEnv | None = None

    try:
        logger.info("[1/4] Starting mock Dreamer simulator...")
        server.start()

        logger.info("[2/4] Resetting DreamerEnv...")
        env = DreamerEnv(port=server.port)
        obs, _ = env.reset()

        assert obs["vector"].shape == (24,)
        assert obs["image"].shape == (64, 64, 3)

        logger.info("[3/4] Running step loop...")
        latencies: list[float] = []
        reward_trace: list[float] = []
        done = False

        for step in range(max_steps):
            action = np.linspace(-1.0, 1.0, 12, dtype=np.float32)
            obs, reward, done, truncated, info = env.step(action)
            assert obs["vector"].shape == (24,)
            assert truncated is False
            latencies.append(float(info["latency"]))
            reward_trace.append(float(reward))

            if done:
                break

        logger.info("[4/4] Validating observations and timing...")
        if not done:
            raise AssertionError("Dreamer mock server never reported done=True")
        if len(latencies) != max_steps:
            raise AssertionError(f"Expected {max_steps} steps, got {len(latencies)}")
        if not all(latency >= 0.0 for latency in latencies):
            raise AssertionError(f"Latency trace contains negative values: {latencies}")
        if float(np.mean(latencies)) >= 100.0:
            raise AssertionError(f"Average latency is unexpectedly high: {latencies}")
        if any(abs(reward) > 1e-6 for reward in reward_trace):
            raise AssertionError(f"Unexpected reward trace: {reward_trace}")

        logger.info("PASS: Dreamer interface verification completed")
        return True
    except Exception as exc:
        logger.exception("FAIL: %s", exc)
        return False
    finally:
        if env is not None:
            env.close()
        server.stop()


if __name__ == "__main__":
    _configure_runtime()
    raise SystemExit(0 if verify_dreamer_interface() else 1)
