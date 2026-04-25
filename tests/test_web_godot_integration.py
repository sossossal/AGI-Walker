"""
Web-Godot 集成测试套件

测试 Web 面板与 Godot 仿真器之间的完整通信流程。

运行方式:
    pytest tests/test_web_godot_integration.py -v

或使用 pytest-asyncio:
    pytest tests/test_web_godot_integration.py -v --asyncio-mode=auto
"""

import logging

import pytest
import json
import time
from datetime import datetime

# 导入被测试的模块
from web_panel.ws_protocol import (
    WsMessage,
    WebSocketProtocolHandler,
    MessageType,
    get_protocol_handler,
)

logger = logging.getLogger(__name__)


class TestWebSocketProtocol:
    """WebSocket 协议处理器测试"""

    def test_message_serialization(self) -> None:
        """测试消息序列化"""
        msg = WsMessage(
            type=MessageType.SIMULATION_START.value,
            payload={"physics": {"gravity": 9.81}},
        )
        json_str = msg.to_json()
        assert isinstance(json_str, str)
        assert "simulation.start" in json_str

    def test_message_deserialization(self) -> None:
        """测试消息反序列化"""
        json_str = '{"type": "ping", "id": "123", "payload": {}}'
        msg = WsMessage.from_json(json_str)
        assert msg.type == "ping"
        assert msg.id == "123"

    def test_message_id_matching(self) -> None:
        """测试消息 ID 匹配"""
        msg1 = WsMessage(type="request", payload={})
        msg2 = WsMessage(type="response", payload={})

        # 验证 ID 是唯一的
        assert msg1.id != msg2.id

    def test_protocol_handler_initialization(self) -> None:
        """测试协议处理器初始化"""
        handler = WebSocketProtocolHandler()

        assert handler is not None
        assert len(handler.handlers) >= 4
        assert MessageType.PING.value in [h for h in handler.handlers.keys()]

    def test_protocol_handler_factory_returns_fresh_instances(self) -> None:
        """测试 handler 工厂不共享全局实例"""
        handler_a = get_protocol_handler()
        handler_b = get_protocol_handler()

        assert handler_a is not handler_b
        handler_a.on_start_simulation = lambda _physics: None
        assert handler_b.on_start_simulation is None

    def test_ping_pong(self) -> None:
        """测试 ping-pong 消息"""
        handler = WebSocketProtocolHandler()
        msg = WsMessage(type=MessageType.PING.value, payload={})
        response = handler.route_message(msg)

        assert response.status == "success"
        assert response.type == MessageType.PONG.value

    def test_command_routing(self) -> None:
        """测试命令路由"""
        handler = WebSocketProtocolHandler()

        # 设置回调
        called = {"start": False}

        def mock_start(physics):
            called["start"] = True

        handler.on_start_simulation = mock_start

        # 发送命令
        msg = WsMessage(
            type=MessageType.SIMULATION_START.value,
            payload={"physics": {"gravity": 9.81}},
        )
        response = handler.route_message(msg)

        # 验证
        assert response.status == "success"
        assert called["start"] is True

    def test_instruction_set_and_simulated_circuit_routing(self) -> None:
        handler = WebSocketProtocolHandler()
        recorded = {}

        handler.on_instruction_set_apply = lambda payload: recorded.setdefault(
            "instruction_set", payload
        ) or {"accepted": True}
        handler.on_simulated_circuit_configure = lambda payload: recorded.setdefault(
            "simulated_circuit", payload
        ) or {"configured": True}

        instruction_response = handler.route_message(
            WsMessage(
                type=MessageType.INSTRUCTION_SET_APPLY.value,
                payload={
                    "instruction_set": {
                        "schema_version": "1.0",
                        "sequence_name": "demo",
                        "steps": [
                            {
                                "kind": "set_velocity",
                                "linear_x": 0.2,
                                "linear_y": 0.0,
                                "angular_z": 0.1,
                            }
                        ],
                    }
                },
            )
        )
        circuit_response = handler.route_message(
            WsMessage(
                type=MessageType.SIMULATED_CIRCUIT_CONFIGURE.value,
                payload={"simulated_circuit": {"transport": "imc22_can_fd"}},
            )
        )

        assert instruction_response.status == "success"
        assert instruction_response.payload["status"] == "instruction_set_applied"
        assert circuit_response.status == "success"
        assert (
            circuit_response.payload["status"] == "simulated_circuit_configured"
        )
        assert recorded["instruction_set"]["sequence_name"] == "demo"
        assert recorded["simulated_circuit"]["transport"] == "imc22_can_fd"

    def test_telemetry_push(self) -> None:
        """测试遥测数据推送"""
        handler = WebSocketProtocolHandler()

        telemetry_data = {"sensors": {"imu": [1, 2, 3]}, "position": [0, 0, 1]}

        msg = handler.push_telemetry(telemetry_data)

        assert msg.type == MessageType.TELEMETRY_UPDATE.value
        assert msg.status == "push"
        assert msg.payload["data"] == telemetry_data

    def test_error_push(self) -> None:
        """测试错误消息推送"""
        handler = WebSocketProtocolHandler()

        msg = handler.push_error("Test error", "test")

        assert msg.type == MessageType.SIMULATION_ERROR.value
        assert msg.payload["error"] == "Test error"
        assert msg.payload["error_type"] == "test"

    def test_unknown_command(self) -> None:
        """测试未知命令处理"""
        handler = WebSocketProtocolHandler()

        msg = WsMessage(type="unknown_command", payload={})
        response = handler.route_message(msg)

        assert response.status == "error"
        assert "Unknown" in response.payload.get("error", "")


class TestMessageFlow:
    """测试完整的消息流"""

    def test_simulation_lifecycle(self) -> None:
        """测试完整的仿真生命周期"""
        handler = WebSocketProtocolHandler()

        # 跟踪调用
        calls = []

        handler.on_start_simulation = lambda p: calls.append(("start", p))
        handler.on_stop_simulation = lambda: calls.append(("stop", None))

        # 启动仿真
        start_msg = WsMessage(
            type=MessageType.SIMULATION_START.value,
            payload={"physics": {"gravity": 9.81}},
        )
        handler.route_message(start_msg)

        # 停止仿真
        stop_msg = WsMessage(type=MessageType.SIMULATION_STOP.value, payload={})
        handler.route_message(stop_msg)

        # 验证调用顺序
        assert len(calls) == 2
        assert calls[0][0] == "start"
        assert calls[1][0] == "stop"

    def test_parameter_update_sequence(self) -> None:
        """测试参数更新序列"""
        handler = WebSocketProtocolHandler()

        updates = []
        handler.on_update_params = lambda p: updates.append(p)

        # 发送多个参数更新
        params_list = [{"speed": 1.0}, {"speed": 1.5}, {"speed": 2.0}]

        for params in params_list:
            msg = WsMessage(
                type=MessageType.PARAMS_UPDATE.value, payload={"params": params}
            )
            handler.route_message(msg)

        assert len(updates) == 3
        assert updates[0]["speed"] == 1.0
        assert updates[2]["speed"] == 2.0

    def test_telemetry_stream(self) -> None:
        """测试遥测数据流"""
        handler = WebSocketProtocolHandler()

        # 生成多个遥测消息
        telemetry_stream = []
        for i in range(10):
            data = {
                "timestamp": i * 0.01,
                "position": [i * 0.1, 0, 0],
                "velocity": [1.0, 0, 0],
            }
            msg = handler.push_telemetry(data)
            telemetry_stream.append(msg)

        # 验证
        assert len(telemetry_stream) == 10
        for msg in telemetry_stream:
            assert msg.type == MessageType.TELEMETRY_UPDATE.value
            assert msg.status == "push"


class TestProtocolCompliance:
    """协议合规性测试"""

    def test_message_has_required_fields(self) -> None:
        """测试消息包含必需字段"""
        msg = WsMessage(type="test", payload={"data": "test"})

        msg_dict = msg.to_dict()

        assert "type" in msg_dict
        assert "id" in msg_dict
        assert "timestamp" in msg_dict
        assert "payload" in msg_dict

    def test_timestamp_is_iso_format(self) -> None:
        """测试时间戳为 ISO 格式"""
        msg = WsMessage(type="test", payload={})
        msg_dict = msg.to_dict()

        # 验证 ISO 时间戳格式
        timestamp = msg_dict["timestamp"]
        try:
            datetime.fromisoformat(timestamp.replace("Z", "+00:00"))
            assert True
        except ValueError:
            assert False, f"Invalid ISO timestamp: {timestamp}"

    def test_payload_is_dict(self) -> None:
        """测试 payload 是字典"""
        msg = WsMessage(type="test", payload={"key": "value"})
        msg_dict = msg.to_dict()

        assert isinstance(msg_dict["payload"], dict)

    def test_status_field_values(self) -> None:
        """测试 status 字段值"""
        valid_statuses = ["success", "error", "push"]

        for status in valid_statuses:
            msg = WsMessage(type="test", payload={}, status=status)
            msg_dict = msg.to_dict()
            assert msg_dict["status"] == status


class TestErrorHandling:
    """错误处理测试"""

    def test_invalid_json_handling(self) -> None:
        """测试无效 JSON 处理"""
        WebSocketProtocolHandler()

        # 这应该在实际应用中由 WebSocket 层处理
        # 这里测试协议处理器的鲁棒性
        try:
            WsMessage.from_json("invalid json")
            assert False, "Should have raised an error"
        except json.JSONDecodeError:
            assert True

    def test_missing_required_fields(self) -> None:
        """测试缺少必需字段"""
        # 最少需要 type 和 payload
        msg = WsMessage(type="", payload={})

        # 应该仍然能够创建消息
        assert msg is not None

    def test_callback_exception_handling(self) -> None:
        """测试回调异常处理"""
        handler = WebSocketProtocolHandler()

        # 设置会抛出异常的回调
        def bad_callback(params):
            raise ValueError("Test error")

        handler.on_update_params = bad_callback

        # 这不应该导致崩溃
        msg = WsMessage(
            type=MessageType.PARAMS_UPDATE.value, payload={"params": {"test": "value"}}
        )

        try:
            response = handler.route_message(msg)
            # 应该返回错误响应
            assert response.status == "error"
        except Exception as e:
            # 或者可能会抛出异常（取决于实现）
            assert isinstance(e, ValueError)


class TestPerformance:
    """性能测试"""

    def test_message_creation_speed(self) -> None:
        """测试消息创建速度"""
        start = time.time()

        for i in range(1000):
            WsMessage(type="test", payload={"index": i})

        elapsed = time.time() - start

        # 应该在 100ms 内创建 1000 个消息
        assert elapsed < 0.1

    def test_routing_speed(self) -> None:
        """测试消息路由速度"""
        handler = WebSocketProtocolHandler()

        start = time.time()

        for i in range(1000):
            msg = WsMessage(type=MessageType.PING.value, payload={})
            handler.route_message(msg)

        elapsed = time.time() - start

        # 应该在 500ms 内路由 1000 个消息
        assert elapsed < 0.5

    def test_telemetry_push_volume(self) -> None:
        """测试遥测推送容量"""
        handler = WebSocketProtocolHandler()

        telemetry_size = 100  # 字节
        telemetry_count = 100  # 每秒 100 条消息

        start = time.time()

        for i in range(telemetry_count):
            data = {"index": i, "data": "x" * telemetry_size}
            handler.push_telemetry(data)

        elapsed = time.time() - start

        # 应该能处理高容量的遥测数据
        assert elapsed < 0.5


# 测试套件
if __name__ == "__main__":
    pytest.main([__file__, "-v"])
