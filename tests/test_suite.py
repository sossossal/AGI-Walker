"""
Legacy TCP 系统测试套件
使用自包含 mock Godot 服务器验证基础通信、稳定性与控制循环。
"""

from __future__ import annotations

import statistics
import time
from typing import Dict, List

import pytest

from agi_walker.core.controllers.tcp_client import GodotClient
from tests.tcp_json_mock_server import JsonLineGodotServer


pytestmark = pytest.mark.integration


@pytest.fixture
def connected_client(jsonline_godot_server):
    client = GodotClient(port=jsonline_godot_server.port)
    assert client.connect(timeout=1.0), "应能连接到 mock Godot 服务器"
    first_sensor = client.wait_for_sensors(timeout=0.5)
    assert first_sensor is not None, "连接后应收到首个传感器包"
    try:
        yield client, jsonline_godot_server
    finally:
        client.close()


def _collect_tilts(client: GodotClient, duration: float = 1.0) -> List[float]:
    tilts: List[float] = []
    deadline = time.time() + duration
    while time.time() < deadline:
        sensor = client.wait_for_sensors(timeout=0.1)
        if not sensor:
            continue
        orient = sensor["sensors"]["imu"]["orient"]
        tilts.append(abs(float(orient[0])) + abs(float(orient[1])))
    return tilts


def test_tcp_connection_and_stats(connected_client) -> None:
    client, _ = connected_client
    stats = client.get_stats()
    assert stats["connected"] is True
    assert stats["packets_received"] >= 1


def test_tcp_latency_round_trip(connected_client) -> None:
    client, server = connected_client
    latencies = []

    start_time = time.time()
    while time.time() - start_time < 1.0:
        t0 = time.perf_counter()
        assert client.send_motor_commands({"test": time.time()}) is True
        sensor = client.wait_for_sensors(timeout=0.1)
        assert sensor is not None
        latencies.append((time.perf_counter() - t0) * 1000.0)
        time.sleep(0.01)

    assert latencies
    assert statistics.mean(latencies) < 50.0
    assert len(server.received_commands) >= len(latencies)


def test_tcp_data_integrity(connected_client) -> None:
    client, _ = connected_client
    samples: List[Dict] = []

    while len(samples) < 10:
        sensor = client.wait_for_sensors(timeout=0.1)
        assert sensor is not None
        samples.append(sensor)

    for sensor in samples:
        assert "timestamp" in sensor
        assert sensor["torso_height"] > 0.5
        assert "sensors" in sensor
        assert "imu" in sensor["sensors"]
        assert len(sensor["sensors"]["imu"]["orient"]) == 3


def test_standing_stability(connected_client) -> None:
    client, _ = connected_client
    tilts = _collect_tilts(client, duration=1.0)

    assert tilts
    assert max(tilts) < 10.0
    assert statistics.mean(tilts) < 8.0


def test_control_frequency(connected_client) -> None:
    client, server = connected_client
    start_time = time.time()
    loop_count = 0

    while time.time() - start_time < 1.0:
        sensor = client.wait_for_sensors(timeout=0.1)
        assert sensor is not None
        assert client.send_motor_commands({"motors": {"hip_left": 0, "hip_right": 0}})
        loop_count += 1
        time.sleep(0.01)

    elapsed = time.time() - start_time
    frequency = loop_count / elapsed if elapsed > 0 else 0.0
    assert frequency >= 20.0
    assert any("motors" in command for command in server.received_commands)


def run_full_test_suite() -> None:
    server = JsonLineGodotServer()
    server.start()
    client = GodotClient(port=server.port)
    try:
        if not client.connect(timeout=1.0):
            raise RuntimeError("无法连接到 mock Godot 服务器")
        if client.wait_for_sensors(timeout=0.5) is None:
            raise RuntimeError("未收到首个传感器包")
        assert client.send_motor_commands({"test": time.time()}) is True
        assert client.wait_for_sensors(timeout=0.1) is not None
    finally:
        client.close()
        server.stop()


if __name__ == "__main__":
    run_full_test_suite()
