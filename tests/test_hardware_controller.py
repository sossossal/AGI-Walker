"""
硬件控制器测试
使用 fake CAN runtime 验证 IMC22Controller 与 HardwareEnvironment。
"""

from __future__ import annotations

import struct
import types
from pathlib import Path
from unittest.mock import MagicMock

import pytest

from agi_walker.core.api.godot_robot_env import hardware_controller as hw

REPLAY_FIXTURE = Path(__file__).with_name("fixtures") / "imc22_status_replay.json"


class FakeMessage:
    def __init__(self, arbitration_id, data, is_extended_id):
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


@pytest.fixture
def fake_can_runtime(monkeypatch):
    bus = MagicMock()
    bus.shutdown = MagicMock()
    bus.recv = MagicMock(return_value=None)
    bus_factory = MagicMock(return_value=bus)
    fake_can = types.SimpleNamespace(
        interface=types.SimpleNamespace(Bus=bus_factory),
        Message=FakeMessage,
    )
    monkeypatch.setattr(hw, "can", fake_can)
    return {"bus": bus, "bus_factory": bus_factory}


class TestIMC22Controller:
    def test_controller_initialization(self, fake_can_runtime) -> None:
        controller = hw.IMC22Controller(
            channel="virtual0", bustype="virtual", bitrate=500000
        )

        fake_can_runtime["bus_factory"].assert_called_once_with(
            channel="virtual0", bustype="virtual", bitrate=500000
        )
        assert controller.bus is fake_can_runtime["bus"]

    def test_send_command(self, fake_can_runtime) -> None:
        controller = hw.IMC22Controller()
        controller.send_command(node_id=5, target_angle=12.34, compliance=0.6)

        sent_message = fake_can_runtime["bus"].send.call_args.args[0]
        assert sent_message.arbitration_id == controller.ID_COMMAND_BASE + 5
        angle_raw, compliance_raw = struct.unpack("<hB", sent_message.data)
        assert angle_raw == 1234
        assert compliance_raw == 153

    def test_command_angle_bounds(self, fake_can_runtime) -> None:
        controller = hw.IMC22Controller()

        controller.send_command(node_id=1, target_angle=500.0, compliance=0.5)
        high_message = fake_can_runtime["bus"].send.call_args.args[0]
        high_angle, _ = struct.unpack("<hB", high_message.data)
        assert high_angle == 32767

        controller.send_command(node_id=1, target_angle=-500.0, compliance=0.5)
        low_message = fake_can_runtime["bus"].send.call_args.args[0]
        low_angle, _ = struct.unpack("<hB", low_message.data)
        assert low_angle == -32768

    def test_command_compliance_bounds(self, fake_can_runtime) -> None:
        controller = hw.IMC22Controller()

        controller.send_command(node_id=1, target_angle=0.0, compliance=2.0)
        high_message = fake_can_runtime["bus"].send.call_args.args[0]
        _, high_compliance = struct.unpack("<hB", high_message.data)
        assert high_compliance == 255

        controller.send_command(node_id=1, target_angle=0.0, compliance=-1.0)
        low_message = fake_can_runtime["bus"].send.call_args.args[0]
        _, low_compliance = struct.unpack("<hB", low_message.data)
        assert low_compliance == 0

    def test_read_status(self, fake_can_runtime) -> None:
        controller = hw.IMC22Controller()
        fake_can_runtime["bus"].recv.return_value = types.SimpleNamespace(
            arbitration_id=controller.ID_STATUS_BASE + 3,
            data=struct.pack("<hhH", 1234, 250, 12),
        )

        status = controller.read_status(timeout=0.01)

        assert status == {
            "node_id": 3,
            "angle": 12.34,
            "current": 0.25,
            "error": 0.12,
        }
        assert controller.node_states[3] == status

    def test_discover_nodes(self, fake_can_runtime) -> None:
        controller = hw.IMC22Controller()
        statuses = iter(
            [
                {"node_id": 3, "angle": 0.0, "current": 0.0, "error": 0.0},
                {"node_id": 1, "angle": 0.0, "current": 0.0, "error": 0.0},
                {"node_id": 3, "angle": 0.0, "current": 0.0, "error": 0.0},
                None,
                None,
            ]
        )

        controller.read_status = lambda timeout=0.1: next(statuses, None)
        nodes = controller.discover_nodes(timeout=0.01)
        assert nodes == [1, 3]

    def test_invalid_replay_payload_rejected(self) -> None:
        errors = hw.validate_imc22_replay_payload(
            {"schema_version": "0.0", "frames": [{"node_id": 0}]}
        )

        assert "schema_version must be '1.0'" in errors
        assert "frames[0].node_id must be a positive int" in errors
        assert "frames[0].angle must be numeric" in errors

    def test_controller_from_replay_fixture(self) -> None:
        controller = hw.IMC22Controller.from_replay(REPLAY_FIXTURE)
        nodes = controller.discover_nodes(timeout=0.01, expected_count=2)

        controller.send_command(node_id=1, target_angle=10.0, compliance=0.4)
        sent_message = controller.bus.sent_messages[0]

        assert nodes == [1, 2]
        assert sent_message.arbitration_id == controller.ID_COMMAND_BASE + 1
        angle_raw, compliance_raw = struct.unpack("<hB", sent_message.data)
        assert angle_raw == 1000
        assert compliance_raw == 102

        controller.close()
        assert controller.bus.closed is True

    def test_command_batch_projects_to_replay_feedback(self) -> None:
        command_batch = [
            {
                "node_id": 1,
                "joint_name": "hip_left",
                "target_angle": 0.3,
                "compliance": 0.4,
                "command_id": 0x201,
            },
            {
                "node_id": 4,
                "joint_name": "knee_right",
                "target_angle": -0.2,
                "compliance": 0.4,
                "command_id": 0x204,
            },
        ]

        replay_payload = hw.command_batch_to_imc22_replay_payload(command_batch)
        feedback = hw.simulate_imc22_command_batch_feedback(command_batch)

        assert replay_payload["schema_version"] == "1.0"
        assert replay_payload["frames"] == [
            {"node_id": 1, "angle": 0.3, "current": 0.106, "error": 0.0},
            {"node_id": 4, "angle": -0.2, "current": 0.104, "error": 0.0},
        ]
        assert feedback["node_ids"] == [1, 4]
        assert feedback["states"][1]["angle"] == 0.3
        assert feedback["states"][4]["angle"] == -0.2

    @pytest.mark.hardware
    def test_real_hardware_connection(self) -> None:
        pytest.skip("需要真实硬件")


class TestHardwareEnvironment:
    def test_hardware_env_creation(self, monkeypatch) -> None:
        controller = MagicMock()
        controller.discover_nodes.return_value = [1, 2, 3]
        monkeypatch.setattr(hw, "IMC22Controller", MagicMock(return_value=controller))

        env = hw.HardwareEnvironment(num_joints=3, control_freq_hz=50)

        assert env.controller is controller
        assert env.node_ids == [1, 2, 3]
        assert env.control_period == pytest.approx(0.02)

    def test_hardware_reset(self, monkeypatch) -> None:
        controller = MagicMock()
        controller.discover_nodes.return_value = [1, 2]
        controller.get_all_states.return_value = {
            1: {"angle": 1.0, "current": 0.1, "error": 0.0},
            2: {"angle": -1.0, "current": 0.2, "error": 0.0},
        }
        monkeypatch.setattr(hw, "IMC22Controller", MagicMock(return_value=controller))
        monkeypatch.setattr(hw.time, "sleep", lambda *_args, **_kwargs: None)

        env = hw.HardwareEnvironment(num_joints=2, control_freq_hz=100)
        obs = env.reset()

        assert obs == [1.0, 0.1, 0.0, -1.0, 0.2, 0.0]
        controller.send_command.assert_any_call(1, target_angle=0.0, compliance=0.5)
        controller.send_command.assert_any_call(2, target_angle=0.0, compliance=0.5)

    def test_hardware_step(self, monkeypatch) -> None:
        controller = MagicMock()
        controller.discover_nodes.return_value = [1, 2, 3]
        controller.get_all_states.return_value = {
            1: {"angle": 1.0, "current": 0.1, "error": 0.0},
            2: {"angle": 2.0, "current": 0.2, "error": 0.0},
            3: {"angle": 3.0, "current": 0.3, "error": 0.0},
        }
        monkeypatch.setattr(hw, "IMC22Controller", MagicMock(return_value=controller))
        monkeypatch.setattr(hw.time, "sleep", lambda *_args, **_kwargs: None)

        env = hw.HardwareEnvironment(num_joints=3, control_freq_hz=100)
        obs, reward, terminated, truncated, info = env.step([10.0, 20.0, 30.0])

        assert obs == [1.0, 0.1, 0.0, 2.0, 0.2, 0.0, 3.0, 0.3, 0.0]
        assert reward == 0.0
        assert terminated is False
        assert truncated is False
        assert info["states"][2]["angle"] == 2.0
        controller.send_command.assert_any_call(1, 10.0, compliance=0.5)
        controller.send_command.assert_any_call(2, 20.0, compliance=0.5)
        controller.send_command.assert_any_call(3, 30.0, compliance=0.5)

    def test_hardware_env_supports_replay_controller(self, monkeypatch) -> None:
        monkeypatch.setattr(hw.time, "sleep", lambda *_args, **_kwargs: None)
        controller = hw.IMC22Controller.from_replay(REPLAY_FIXTURE)

        env = hw.HardwareEnvironment(
            num_joints=2,
            control_freq_hz=50,
            controller=controller,
        )
        reset_obs = env.reset()
        step_obs, reward, terminated, truncated, info = env.step([5.0, -5.0])

        assert env.node_ids == [1, 2]
        assert reset_obs == [1.0, 0.11, 0.0, -1.0, 0.12, 0.0]
        assert step_obs == [1.5, 0.2, 0.01, -1.5, 0.25, 0.02]
        assert reward == 0.0
        assert terminated is False
        assert truncated is False
        assert info["states"][1]["angle"] == 1.5
        assert len(controller.bus.sent_messages) == 4


class TestCANProtocol:
    def test_message_id_calculation(self) -> None:
        assert hw.IMC22Controller.ID_COMMAND_BASE + 7 == 0x207
        assert hw.IMC22Controller.ID_STATUS_BASE + 7 == 0x107

    def test_angle_encoding(self) -> None:
        encoded = struct.pack("<h", int(12.34 * 100))
        assert struct.unpack("<h", encoded)[0] == 1234

    def test_angle_decoding(self) -> None:
        data = struct.pack("<h", -567)
        decoded = struct.unpack("<h", data)[0] * 0.01
        assert decoded == -5.67

    def test_compliance_encoding(self) -> None:
        encoded = int(0.5 * 255)
        assert encoded == 127
