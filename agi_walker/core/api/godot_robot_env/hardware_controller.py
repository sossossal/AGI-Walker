"""
AGI-Walker 硬件控制器接口
用于与 IMC-22 Reflex 控制器通信
"""

import json
import logging
import struct
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

try:
    import can
except ImportError:
    can = None


IMC22_REPLAY_SCHEMA_VERSION = "1.0"


class ReplayCANMessage:
    def __init__(self, arbitration_id: int, data: bytes, is_extended_id: bool) -> None:
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = is_extended_id


def encode_command_payload(target_angle: float, compliance: float = 0.5) -> bytes:
    angle_int16 = max(-32768, min(32767, int(target_angle * 100)))
    compliance_u8 = max(0, min(255, int(compliance * 255)))
    return struct.pack("<hB", angle_int16, compliance_u8)


def encode_status_payload(angle: float, current: float, error: float) -> bytes:
    angle_raw = max(-32768, min(32767, int(angle * 100)))
    current_raw = max(-32768, min(32767, int(current * 1000)))
    error_raw = max(0, min(65535, int(error * 100)))
    return struct.pack("<hhH", angle_raw, current_raw, error_raw)


def decode_status_payload(data: bytes) -> Dict[str, float]:
    angle_raw, current_raw, error_raw = struct.unpack("<hhH", data[:6])
    return {
        "angle": angle_raw * 0.01,
        "current": current_raw * 0.001,
        "error": error_raw * 0.01,
    }


def validate_imc22_replay_payload(payload: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(payload, dict):
        return ["replay payload must be a dict"]
    if payload.get("schema_version") != IMC22_REPLAY_SCHEMA_VERSION:
        errors.append(f"schema_version must be {IMC22_REPLAY_SCHEMA_VERSION!r}")
    frames = payload.get("frames")
    if not isinstance(frames, list) or not frames:
        errors.append("frames must be a non-empty list")
        return errors
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            errors.append(f"frames[{index}] must be a dict")
            continue
        node_id = frame.get("node_id")
        if not isinstance(node_id, int) or node_id <= 0:
            errors.append(f"frames[{index}].node_id must be a positive int")
        for key in ["angle", "current", "error"]:
            value = frame.get(key)
            if not isinstance(value, (int, float)):
                errors.append(f"frames[{index}].{key} must be numeric")
    return errors


def load_imc22_replay_payload(source: str | Path | Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(source, dict):
        payload = source
    else:
        payload = json.loads(Path(source).read_text(encoding="utf-8"))
    errors = validate_imc22_replay_payload(payload)
    if errors:
        raise ValueError("; ".join(errors))
    return payload


class ReplayCANBus:
    is_replay = True

    def __init__(
        self,
        frames: List[Dict[str, Any]],
        *,
        status_id_base: int,
        message_factory=ReplayCANMessage,
    ) -> None:
        self._frames = list(frames)
        self._cursor = 0
        self._status_id_base = status_id_base
        self._message_factory = message_factory
        self.sent_messages: List[Any] = []
        self.closed = False

    @classmethod
    def from_payload(
        cls,
        payload: Dict[str, Any],
        *,
        status_id_base: int,
        message_factory=ReplayCANMessage,
    ) -> "ReplayCANBus":
        return cls(
            payload["frames"],
            status_id_base=status_id_base,
            message_factory=message_factory,
        )

    def has_pending_frames(self) -> bool:
        return self._cursor < len(self._frames)

    def send(self, message: Any) -> None:
        self.sent_messages.append(message)

    def recv(self, timeout: float = 0.1) -> Optional[Any]:
        if not self.has_pending_frames():
            return None
        frame = self._frames[self._cursor]
        self._cursor += 1
        return self._message_factory(
            arbitration_id=self._status_id_base + frame["node_id"],
            data=encode_status_payload(
                angle=frame["angle"],
                current=frame["current"],
                error=frame["error"],
            ),
            is_extended_id=False,
        )

    def shutdown(self) -> None:
        self.closed = True


class IMC22Controller:
    """IMC-22 硬件控制器接口"""

    # CAN ID 定义
    ID_SYNC = 0x000
    ID_STATUS_BASE = 0x100
    ID_COMMAND_BASE = 0x200
    ID_CONFIG_BASE = 0x300
    ID_HANDSHAKE = 0x7FF

    def __init__(
        self,
        channel="can0",
        bustype="socketcan",
        bitrate=1000000,
        *,
        bus=None,
        message_factory=None,
    ):
        """
        初始化硬件控制器

        Args:
            channel: CAN 通道 (Linux: 'can0', Windows: 'PCAN_USBBUS1')
            bustype: 总线类型 (Linux: 'socketcan', Windows: 'pcan')
            bitrate: 波特率 (默认 1 Mbps)
        """
        self.message_factory = message_factory or getattr(
            can, "Message", ReplayCANMessage
        )
        if bus is not None:
            self.bus = bus
        else:
            if can is None:
                raise ImportError(
                    "python-can library is required for hardware_controller. "
                    "Install it with: pip install python-can"
                )
            try:
                self.bus = can.interface.Bus(
                    channel=channel, bustype=bustype, bitrate=bitrate
                )
                logger.info(f"CAN 总线已连接: {channel} @ {bitrate} bps")
            except Exception as e:
                logger.error(f"无法连接 CAN 总线: {e}")
                raise

        self.node_states = {}  # 存储各节点状态

    @classmethod
    def from_replay(
        cls, replay_source: str | Path | Dict[str, Any]
    ) -> "IMC22Controller":
        payload = load_imc22_replay_payload(replay_source)
        return cls(
            channel="replay",
            bustype="replay",
            bus=ReplayCANBus.from_payload(
                payload,
                status_id_base=cls.ID_STATUS_BASE,
                message_factory=ReplayCANMessage,
            ),
            message_factory=ReplayCANMessage,
        )

    def send_command(self, node_id: int, target_angle: float, compliance: float = 0.5):
        """
        发送控制命令到指定节点

        Args:
            node_id: 节点 ID (1-255)
            target_angle: 目标角度 (度)
            compliance: 柔顺系数 (0.0 = 刚性, 1.0 = 柔性)
        """
        msg = self.message_factory(
            arbitration_id=self.ID_COMMAND_BASE + node_id,
            data=encode_command_payload(target_angle, compliance),
            is_extended_id=False,
        )

        try:
            self.bus.send(msg)
        except Exception as e:
            logger.error(f"发送命令失败 (节点 {node_id}): {e}")

    def read_status(self, timeout: float = 0.1) -> Optional[Dict]:
        """
        读取节点状态

        Args:
            timeout: 超时时间 (秒)

        Returns:
            状态字典 {'node_id': int, 'angle': float, 'current': float, 'error': float}
            或 None (如果超时)
        """
        msg = self.bus.recv(timeout=timeout)

        if not msg:
            return None

        # 检查是否为状态消息
        if (
            msg.arbitration_id >= self.ID_STATUS_BASE
            and msg.arbitration_id < self.ID_COMMAND_BASE
        ):
            node_id = msg.arbitration_id - self.ID_STATUS_BASE

            status = {"node_id": node_id, **decode_status_payload(msg.data)}

            # 缓存状态
            self.node_states[node_id] = status

            return status

        return None

    def get_all_states(self, num_nodes: int, timeout: float = 0.5) -> Dict[int, Dict]:
        """
        获取所有节点的状态

        Args:
            num_nodes: 节点数量
            timeout: 总超时时间

        Returns:
            {node_id: {'angle': float, 'current': float, 'error': float}}
        """
        start_time = time.time()
        states = {}

        while len(states) < num_nodes and (time.time() - start_time) < timeout:
            status = self.read_status(timeout=0.01)
            if status:
                states[status["node_id"]] = status
                continue
            if (
                getattr(self.bus, "is_replay", False)
                and not self.bus.has_pending_frames()
            ):
                break

        return states

    def set_config(self, node_id: int, max_torque: float, kp: float, ki: float):
        """
        配置节点参数

        Args:
            node_id: 节点 ID
            max_torque: 最大力矩 (N·m)
            kp: PID 比例系数
            ki: PID 积分系数
        """
        data = struct.pack("<fff", max_torque, kp, ki)

        msg = self.message_factory(
            arbitration_id=self.ID_CONFIG_BASE + node_id,
            data=data,
            is_extended_id=False,
        )

        self.bus.send(msg)
        logger.info(f"节点 {node_id} 配置已更新")

    def discover_nodes(
        self, timeout: float = 2.0, expected_count: Optional[int] = None
    ) -> List[int]:
        """
        发现总线上的所有节点

        Args:
            timeout: 扫描超时时间

        Returns:
            节点 ID 列表
        """
        logger.info("扫描 CAN 总线上的节点...")

        discovered = set()
        start_time = time.time()

        while (time.time() - start_time) < timeout:
            status = self.read_status(timeout=0.1)
            if status:
                discovered.add(status["node_id"])
                if expected_count and len(discovered) >= expected_count:
                    break
                continue
            if (
                getattr(self.bus, "is_replay", False)
                and not self.bus.has_pending_frames()
            ):
                break

        nodes = sorted(list(discovered))
        logger.info(f"发现 {len(nodes)} 个节点: {nodes}")

        return nodes

    def close(self):
        """关闭 CAN 总线"""
        if self.bus:
            self.bus.shutdown()
            logger.info("CAN 总线已关闭")


class HardwareEnvironment:
    """
    硬件环境包装器，兼容 Gymnasium 接口
    用于在真实硬件上测试策略
    """

    def __init__(
        self,
        num_joints: int = 12,
        control_freq_hz: int = 100,
        *,
        controller: Optional[IMC22Controller] = None,
    ):
        """
        Args:
            num_joints: 关节数量
            control_freq_hz: 控制频率 (Hz)
        """
        self.controller = controller or IMC22Controller()
        self.num_joints = num_joints
        self.control_period = 1.0 / control_freq_hz

        # 发现节点
        self.node_ids = self.controller.discover_nodes(expected_count=num_joints)
        if len(self.node_ids) != num_joints:
            logger.warning(
                f"期望 {num_joints} 个节点，实际发现 {len(self.node_ids)} 个"
            )

    def reset(self):
        """重置到初始状态"""
        # 所有关节归零
        for node_id in self.node_ids:
            self.controller.send_command(node_id, target_angle=0.0, compliance=0.5)

        time.sleep(1.0)  # 等待稳定

        # 读取初始状态
        states = self.controller.get_all_states(self.num_joints)
        return self._build_observation(states)

    def step(self, action):
        """
        执行一步控制

        Args:
            action: 动作数组 (每个关节的目标角度)
        """
        # 发送命令
        for i, node_id in enumerate(self.node_ids):
            if i < len(action):
                self.controller.send_command(node_id, action[i], compliance=0.5)

        # 等待控制周期
        time.sleep(self.control_period)

        # 读取新状态
        states = self.controller.get_all_states(self.num_joints)
        obs = self._build_observation(states)

        # 简化的奖励（实际需要根据任务计算）
        reward = 0.0
        terminated = False
        truncated = False
        info = {"states": states}

        return obs, reward, terminated, truncated, info

    def _build_observation(self, states: Dict) -> List[float]:
        """从节点状态构建观察"""
        obs = []
        for node_id in sorted(self.node_ids):
            if node_id in states:
                obs.extend(
                    [
                        states[node_id]["angle"],
                        states[node_id]["current"],
                        states[node_id]["error"],
                    ]
                )
            else:
                obs.extend([0.0, 0.0, 0.0])
        return obs

    def close(self):
        """关闭环境"""
        self.controller.close()


if __name__ == "__main__":
    # 简单测试
    try:
        controller = IMC22Controller()

        # 发现节点
        nodes = controller.discover_nodes()

        # 发送测试命令
        if nodes:
            controller.send_command(nodes[0], target_angle=45.0, compliance=0.5)
            time.sleep(0.1)

            # 读取状态
            status = controller.read_status()
            if status:
                print(f"节点状态: {status}")

        controller.close()

    except Exception as e:
        print(f"测试失败: {e}")
        print("提示: 确保 CAN 适配器已连接并配置正确")
