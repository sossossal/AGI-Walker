"""
通信延迟测试
使用仓库内 mock TCP 服务器验证 legacy GodotClient 的往返延迟。
"""

import statistics
import time

import pytest

from agi_walker.core.controllers.tcp_client import GodotClient


pytestmark = pytest.mark.integration


def test_latency(jsonline_godot_server) -> None:
    client = GodotClient(port=jsonline_godot_server.port)
    assert client.connect(timeout=1.0), "应能连接到 mock Godot 服务器"

    latencies = []
    try:
        first_sensor = client.wait_for_sensors(timeout=0.5)
        assert first_sensor is not None, "连接后应能收到首个传感器包"

        start_time = time.time()
        while time.time() - start_time < 1.0:
            t0 = time.perf_counter()
            assert client.send_motor_commands({"test": time.time()}) is True
            sensor = client.wait_for_sensors(timeout=0.1)
            assert sensor is not None, "发送命令后应在超时内收到传感器数据"
            latencies.append((time.perf_counter() - t0) * 1000.0)
            time.sleep(0.01)
    finally:
        client.close()

    assert latencies, "应采集到至少一次往返延迟"
    avg_latency = statistics.mean(latencies)
    assert avg_latency < 50.0
