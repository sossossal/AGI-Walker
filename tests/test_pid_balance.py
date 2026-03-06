"""
PID平衡控制专项测试
测试不同PID参数下的平衡效果
"""

import time
import json
import statistics
from typing import List, Tuple, Dict
import pytest

pytestmark = pytest.mark.integration

try:
    from python_controller.tcp_client import GodotClient
    CLIENT_AVAILABLE = True
except ImportError:
    CLIENT_AVAILABLE = False


def check_client_available():
    if not CLIENT_AVAILABLE:
        pytest.skip("python_controller.tcp_client 不可用")


class TestPIDBalance:
    """PID平衡控制测试类"""

    def setup_method(self):
        self.client = GodotClient()

    def teardown_method(self):
        if hasattr(self, 'client'):
            self.client.close()

    def test_pid_default_config(self):
        """测试默认PID配置"""
        check_client_available()
        
        # 在 CI 环境中自动跳过，除非能连接上
        if not self.client.connect(timeout=0.1):
            pytest.skip("无法连接到仿真器 (Godot 未运行)")

        duration = 5.0  # 测试时间缩短以适应 CI
        start_time = time.time()
        tilts = []

        while time.time() - start_time < duration:
            sensor = self.client.get_latest_sensors()
            if sensor:
                orient = sensor["sensors"]["imu"]["orient"]
                tilt = abs(orient[0]) + abs(orient[1])
                tilts.append(tilt)
            time.sleep(0.05)

        assert len(tilts) > 0, "未收集到传感器数据"
        avg_tilt = statistics.mean(tilts)
        print(f"平均倾斜: {avg_tilt:.2f}")


def run_pid_tests_manual():
    """手动运行PID测试 (兼容脚本模式)"""
    print("=" * 60)
    print("🧪 PID平衡控制测试 (手动模式)")
    print("=" * 60)

    if not CLIENT_AVAILABLE:
        print("❌ python_controller.tcp_client 不可用")
        return

    client = GodotClient()
    if not client.connect():
        print("❌ 无法连接到仿真器")
        return

    print("开始 5 秒监控...")
    try:
        start_time = time.time()
        while time.time() - start_time < 5:
            sensor = client.get_latest_sensors()
            if sensor:
                print(f"当前姿态: {sensor['sensors']['imu']['orient']}")
            time.sleep(1)
    finally:
        client.close()
    print("✅ 测试完成")


if __name__ == "__main__":
    run_pid_tests_manual()
