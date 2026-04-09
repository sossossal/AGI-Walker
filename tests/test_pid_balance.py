"""
PID 平衡控制专项测试
使用 mock TCP 传感器流验证基础姿态统计。
"""

import statistics
import time

import pytest

from agi_walker.core.controllers.tcp_client import GodotClient


pytestmark = pytest.mark.integration


class TestPIDBalance:
    def test_pid_default_config(self, jsonline_godot_server) -> None:
        client = GodotClient(port=jsonline_godot_server.port)
        assert client.connect(timeout=1.0), "应能连接到 mock Godot 服务器"

        tilts = []
        try:
            deadline = time.time() + 1.0
            while time.time() < deadline:
                sensor = client.wait_for_sensors(timeout=0.1)
                assert sensor is not None, "应能持续收到传感器数据"
                orient = sensor["sensors"]["imu"]["orient"]
                tilts.append(abs(float(orient[0])) + abs(float(orient[1])))
        finally:
            client.close()

        assert len(tilts) >= 10
        assert statistics.mean(tilts) < 8.0


def run_pid_tests_manual() -> None:
    server = pytest.importorskip("tests.tcp_json_mock_server").JsonLineGodotServer()
    server.start()
    client = GodotClient(port=server.port)
    try:
        if not client.connect(timeout=1.0):
            raise RuntimeError("无法连接到 mock Godot 服务器")

        start_time = time.time()
        while time.time() - start_time < 1.0:
            sensor = client.wait_for_sensors(timeout=0.1)
            if sensor:
                print(f"当前姿态: {sensor['sensors']['imu']['orient']}")
    finally:
        client.close()
        server.stop()


if __name__ == "__main__":
    run_pid_tests_manual()
