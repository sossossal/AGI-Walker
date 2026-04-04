"""
TCP <-> Zenoh bridge for backward-compatible communication with Godot clients.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from typing import Optional
import logging

from agi_walker.core.api.comm.zenoh_interface import ZenohConfig, ZenohInterface

logger = logging.getLogger(__name__)


def setup_logging():
    """Configure basic logging for demonstration purposes"""
    if not logger.handlers:
        handler = logging.StreamHandler()
        formatter = logging.Formatter("%(levelname)s: %(message)s")
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)


class TcpZenohBridge:
    """
    Bridge TCP traffic from Godot clients into Zenoh topics and forward Zenoh
    commands back to the connected TCP client.
    """

    def __init__(
        self,
        tcp_host: str = "127.0.0.1",
        tcp_port: int = 9090,
        zenoh_config: Optional[ZenohConfig] = None,
    ):
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self.zenoh = ZenohInterface(zenoh_config or ZenohConfig())

        self.tcp_server = self._create_server_socket()
        self.tcp_client: Optional[socket.socket] = None

        self.running = False
        self.tcp_thread: Optional[threading.Thread] = None
        self.zenoh_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

        logger.info("TCP-Zenoh bridge initialized")

    def _create_server_socket(self) -> socket.socket:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        return server

    def start(self) -> None:
        """Start the bridge server and background accept loop."""
        if self.running:
            return

        self.running = True

        if (
            self.tcp_server is None
            or getattr(self.tcp_server, "fileno", lambda: 0)() == -1
        ):
            self.tcp_server = self._create_server_socket()

        self.tcp_server.bind((self.tcp_host, self.tcp_port))
        self.tcp_server.listen(5)
        try:
            self.tcp_server.settimeout(1.0)
        except OSError:
            pass

        self.zenoh.declare_subscriber("rt/python/cmd", self._on_zenoh_cmd)

        self.tcp_thread = threading.Thread(
            target=self._accept_connections,
            name="tcp-zenoh-accept",
            daemon=True,
        )
        self.tcp_thread.start()

        logger.info(f"TCP server listening on {self.tcp_host}:{self.tcp_port}")

    def _accept_connections(self) -> None:
        """Accept TCP clients and process messages until stopped."""
        while self.running:
            try:
                client, _addr = self.tcp_server.accept()
            except socket.timeout:
                continue
            except OSError:
                if self.running:
                    time.sleep(0.1)
                continue

            with self._lock:
                if self.tcp_client is not None:
                    try:
                        self.tcp_client.close()
                    except OSError:
                        pass
                self.tcp_client = client

            try:
                client.settimeout(1.0)
            except OSError:
                pass

            self._client_loop(client)

    def _client_loop(self, client: socket.socket) -> None:
        """Handle data from one TCP client."""
        while self.running and client is self.tcp_client:
            try:
                data = client.recv(4096)
            except socket.timeout:
                continue
            except (ConnectionError, OSError):
                break

            if not data:
                break

            for payload in self._split_messages(data):
                try:
                    message = json.loads(payload.decode("utf-8"))
                except json.JSONDecodeError:
                    continue

                self.zenoh.publish("rt/godot/state", message)

        if client is self.tcp_client:
            with self._lock:
                self.tcp_client = None
        try:
            client.close()
        except OSError:
            pass

    def _split_messages(self, data: bytes) -> list[bytes]:
        """Split newline-delimited payloads while still supporting single JSON blobs."""
        chunks = [chunk.strip() for chunk in data.splitlines() if chunk.strip()]
        return chunks or [data]

    def _on_zenoh_cmd(self, data) -> None:
        """Forward Zenoh command payloads to the active TCP client."""
        client = self.tcp_client
        if client is None:
            return

        payload = json.dumps(data).encode("utf-8") + b"\n"
        client.send(payload)

    def stop(self) -> None:
        """Stop the bridge and release all sockets/resources."""
        self.running = False

        client = self.tcp_client
        self.tcp_client = None
        if client is not None:
            try:
                client.close()
            except OSError:
                pass

        if self.tcp_server is not None:
            try:
                self.tcp_server.close()
            except OSError:
                pass

        self.zenoh.close()

        if self.tcp_thread and self.tcp_thread.is_alive():
            self.tcp_thread.join(timeout=1.0)

        logger.info("TCP-Zenoh bridge stopped")


if __name__ == "__main__":
    setup_logging()

    logger.info("=" * 60)
    logger.info("TCP-Zenoh bridge")
    logger.info("=" * 60)

    bridge = TcpZenohBridge()
    bridge.start()

    try:
        logger.info("Press Ctrl+C to stop...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("Stopping...")
        bridge.stop()
