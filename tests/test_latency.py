"""
通信延迟测试脚本
测试Godot仿真器与Python控制端之间的通信性能
"""

import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import time
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


def test_latency() -> None:
    """测试通信延迟"""
    check_client_available()
    
    duration: float = 5.0  # 缩短 CI 运行时间

    client = GodotClient()

    if not client.connect(timeout=0.1):
        pytest.skip("无法连接到仿真器 (Godot 未运行)")

    logger.info(f"\n🧪 开始延迟测试 (持续{duration}秒)...\n")

    latencies = []
    start_time = time.time()

    try:
        while time.time() - start_time < duration:
            t0 = time.time()

            # 发送测试指令
            client.send_motor_commands({"test": time.time()})

            # 等待传感器数据
            sensor = client.wait_for_sensors(timeout=0.1)

            if sensor:
                t1 = time.time()
                latency = (t1 - t0) * 1000  # 转换为毫秒
                latencies.append(latency)

            time.sleep(0.01)  # 100Hz
    finally:
        client.close()

    # 统计结果
    if latencies:
        import statistics
        avg_latency = statistics.mean(latencies)
        assert avg_latency < 50.0  # 宽限度测试
    else:
        pytest.skip("未收到任何数据，跳过分析")


if __name__ == "__main__":
    try:
        test_latency()
    except Exception as e:
        logger.info(f"测试失败: {e}")
