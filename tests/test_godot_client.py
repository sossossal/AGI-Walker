"""
测试Godot通信客户端

验证TCP连接、命令发送和数据接收功能
"""

import logging

import unittest
import socket
import time
import pytest
from agi_walker.core.api.comm.godot_client import GodotSimulationClient, MockGodotServer

# 需要 TCP 服务器，不适合默认单元测试路径

logger = logging.getLogger(__name__)
pytestmark = pytest.mark.integration


def _find_unused_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class TestGodotClient(unittest.TestCase):
    """Godot客户端测试"""

    @classmethod
    def setUpClass(cls):
        """启动模拟服务器"""
        cls.server = MockGodotServer(port=0)
        assert cls.server.start()
        cls.port = cls.server.port
        time.sleep(0.5)  # 等待服务器启动

    @classmethod
    def tearDownClass(cls):
        """关闭服务器"""
        cls.server.stop()

    def test_connection(self) -> None:
        """测试连接"""
        client = GodotSimulationClient(port=self.port)

        # 连接应该成功
        self.assertTrue(client.connect())
        self.assertTrue(client.is_connected())

        # 断开
        client.disconnect()
        self.assertFalse(client.is_connected())

    def test_connection_timeout(self) -> None:
        """测试连接超时"""
        # 连接到不存在的服务器
        client = GodotSimulationClient(port=_find_unused_port())

        # 应该失败
        self.assertFalse(client.connect(timeout=1.0))

    def test_send_commands(self) -> None:
        """测试发送命令"""
        client = GodotSimulationClient(port=self.port)

        if client.connect():
            # 加载配置
            success = client.load_robot_config(
                parts=[{"id": "motor_1", "type": "motor"}], connections=[]
            )
            self.assertTrue(success)

            # 启动仿真
            success = client.start_simulation({"test": "config"})
            self.assertTrue(success)

            # 更新参数
            success = client.update_parameters({"power": 1.5})
            self.assertTrue(success)

            # 停止
            success = client.stop_simulation()
            self.assertTrue(success)

            client.disconnect()
        else:
            self.fail("无法连接到服务器")

    def test_data_callback(self) -> None:
        """测试数据回调"""
        client = GodotSimulationClient(port=self.port)

        received_data = []

        def callback(data):
            received_data.append(data)

        client.set_data_callback(callback)

        if client.connect():
            # 启动仿真以触发数据发送
            client.start_simulation({})

            # 等待接收数据
            time.sleep(2)

            # 应该接收到数据
            self.assertGreater(len(received_data), 0)

            # 验证数据格式
            if received_data:
                data = received_data[0]
                self.assertIn("type", data)
                self.assertEqual(data["type"], "simulation_data")

            client.disconnect()
        else:
            self.fail("无法连接到服务器")


if __name__ == "__main__":
    logger.info("=== Godot客户端单元测试 ===\n")
    unittest.main(verbosity=2)
