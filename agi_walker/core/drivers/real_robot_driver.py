import json
import logging
import struct
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

try:
    import serial
except ImportError:
    serial = None  # Mock 模式下不需要 pyserial

# 通信协议:
# Head (2B): 0xAA 0x55
# Cmd (1B): 0x01 (电机指令), 0x02 (传感器请求), 0x03 (SysID 数据)
# Len (1B): Payload 长度
# Payload (N)
# CRC (1B): 校验和

PACKET_HEAD = b"\xaa\x55"
CMD_MOTOR_COMMAND = 0x01
CMD_SENSOR_STATE = 0x02
CMD_SYSID_DATA = 0x03
REAL_ROBOT_REPLAY_SCHEMA_VERSION = "1.0"


def build_packet(cmd: int, payload: bytes) -> bytes:
    frame = bytearray(PACKET_HEAD)
    frame.extend([cmd, len(payload)])
    frame.extend(payload)
    frame.append(sum(frame) % 256)
    return bytes(frame)


def parse_packet(frame: bytes) -> tuple[int, bytes]:
    if len(frame) < 5:
        raise ValueError("frame is too short")
    if bytes(frame[:2]) != PACKET_HEAD:
        raise ValueError("frame head mismatch")
    payload_length = frame[3]
    expected_length = 5 + payload_length
    if len(frame) != expected_length:
        raise ValueError("frame length mismatch")
    if frame[-1] != sum(frame[:-1]) % 256:
        raise ValueError("frame checksum mismatch")
    return frame[2], bytes(frame[4:-1])


def encode_motor_command_payload(commands: Dict[str, float]) -> bytes:
    payload = bytearray([len(commands)])
    for motor_name, pos in commands.items():
        motor_id = int(motor_name.split("_")[-1]) if "_" in motor_name else 0
        payload.append(motor_id)
        payload.extend(struct.pack("<f", pos))
    return bytes(payload)


def decode_motor_command_payload(payload: bytes) -> Dict[str, float]:
    if not payload:
        return {}
    count = payload[0]
    offset = 1
    commands: Dict[str, float] = {}
    for _ in range(count):
        motor_id = payload[offset]
        offset += 1
        (position,) = struct.unpack_from("<f", payload, offset)
        offset += 4
        commands[f"motor_{motor_id}"] = position
    return commands


def encode_motor_state_payload(motors: Dict[str, Dict[str, float]]) -> bytes:
    payload = bytearray([len(motors)])
    for motor_name, state in motors.items():
        motor_id = int(motor_name.split("_")[-1]) if "_" in motor_name else 0
        payload.append(motor_id)
        payload.extend(
            struct.pack(
                "<fff",
                float(state.get("pos", 0.0)),
                float(state.get("vel", 0.0)),
                float(state.get("torque", 0.0)),
            )
        )
    return bytes(payload)


def decode_motor_state_payload(payload: bytes) -> Dict[str, Dict[str, float]]:
    if not payload:
        return {}
    count = payload[0]
    offset = 1
    motors: Dict[str, Dict[str, float]] = {}
    for _ in range(count):
        motor_id = payload[offset]
        offset += 1
        pos, vel, torque = struct.unpack_from("<fff", payload, offset)
        offset += 12
        motors[f"motor_{motor_id}"] = {
            "pos": pos,
            "vel": vel,
            "torque": torque,
        }
    return motors


def validate_real_robot_replay_payload(payload: Dict[str, Any]) -> List[str]:
    errors: List[str] = []
    if not isinstance(payload, dict):
        return ["replay payload must be a dict"]
    if payload.get("schema_version") != REAL_ROBOT_REPLAY_SCHEMA_VERSION:
        errors.append(f"schema_version must be {REAL_ROBOT_REPLAY_SCHEMA_VERSION!r}")
    frames = payload.get("frames")
    if not isinstance(frames, list) or not frames:
        errors.append("frames must be a non-empty list")
        return errors
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict):
            errors.append(f"frames[{index}] must be a dict")
            continue
        if frame.get("cmd") != "sensor_state":
            errors.append(f"frames[{index}].cmd must be 'sensor_state'")
        motors = frame.get("motors")
        if not isinstance(motors, dict) or not motors:
            errors.append(f"frames[{index}].motors must be a non-empty dict")
            continue
        for motor_name, motor_state in motors.items():
            if not isinstance(motor_state, dict):
                errors.append(f"frames[{index}].motors[{motor_name!r}] must be a dict")
                continue
            for key in ["pos", "vel", "torque"]:
                if not isinstance(motor_state.get(key), (int, float)):
                    errors.append(
                        f"frames[{index}].motors[{motor_name!r}].{key} must be numeric"
                    )
    return errors


def load_real_robot_replay_payload(
    source: str | Path | Dict[str, Any],
) -> Dict[str, Any]:
    if isinstance(source, dict):
        payload = source
    else:
        payload = json.loads(Path(source).read_text(encoding="utf-8"))
    errors = validate_real_robot_replay_payload(payload)
    if errors:
        raise ValueError("; ".join(errors))
    return payload


class ReplaySerialTransport:
    is_replay = True

    def __init__(self, packets: List[bytes]) -> None:
        self._packets = list(packets)
        self.writes: List[bytes] = []
        self.closed = False

    @classmethod
    def from_payload(cls, payload: Dict[str, Any]) -> "ReplaySerialTransport":
        packets = [
            build_packet(CMD_SENSOR_STATE, encode_motor_state_payload(frame["motors"]))
            for frame in payload["frames"]
        ]
        return cls(packets)

    def pop_next_packet(self) -> Optional[bytes]:
        if not self._packets:
            return None
        return self._packets.pop(0)

    def has_pending_packets(self) -> bool:
        return bool(self._packets)

    def write(self, data: bytes) -> None:
        self.writes.append(bytes(data))

    def close(self) -> None:
        self.closed = True


class RealRobotDriver:
    """真实硬件驱动 — 通过串口/TCP 控制机器人"""

    def __init__(
        self,
        port: str = "COM3",
        baudrate: int = 115200,
        mock: bool = False,
        *,
        transport: Optional[Any] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.mock = mock
        self.transport = transport
        self.ser: Optional[Any] = None
        self.running = False
        self.lock = threading.Lock()

        # Robot State
        self.motor_states: Dict[str, Dict] = {}  # {id: {pos, vel, torque, temp}}
        self.imu_data = {"rpy": [0, 0, 0], "acc": [0, 0, 0], "gyro": [0, 0, 0]}

        self.logger = logging.getLogger("RealRobotDriver")

    @classmethod
    def from_replay(
        cls, replay_source: str | Path | Dict[str, Any]
    ) -> "RealRobotDriver":
        payload = load_real_robot_replay_payload(replay_source)
        return cls(
            port="replay",
            baudrate=0,
            transport=ReplaySerialTransport.from_payload(payload),
        )

    def connect(self) -> bool:
        if self.mock:
            self.logger.info(f"[Mock] Connected to {self.port}")
            self.running = True
            return True

        if self.transport is not None:
            self.ser = self.transport
            self.running = True
            self.logger.info(f"[Replay] Connected to {self.port}")
            return True

        if serial is None:
            self.logger.error(
                "pyserial is required for real hardware mode. Install it with: pip install pyserial"
            )
            return False

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.running = True
            # Start reader thread
            threading.Thread(target=self._read_loop, daemon=True).start()
            self.logger.info(f"Connected to real hardware at {self.port}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.ser:
            self.ser.close()

    def send_motor_commands(self, commands: Dict[str, float]) -> bool:
        """
        Send motor position/torque commands.
        commands: {'motor_1': 1.57, 'motor_2': -0.5} (radians)
        """
        if self.mock:
            # Update mock state directly
            with self.lock:
                for mid, val in commands.items():
                    if mid not in self.motor_states:
                        self.motor_states[mid] = {}
                    self.motor_states[mid]["pos"] = val
                    self.motor_states[mid]["vel"] = 0.0
                    self.motor_states[mid]["torque"] = 0.0
            return True

        # Pack data (Example: 0x01 + Count + [ID, Pos_High, Pos_Low]...)
        # Simplified for demo: Just sending text JSON for ESP32/Arduino parsing for now in MVP
        # Or binary struct for performance. Let's use simple binary.
        # [Cmd, NumMotors, ID1, Pos1(float), ID2, Pos2(float)...]
        try:
            self._send_packet(CMD_MOTOR_COMMAND, encode_motor_command_payload(commands))
            return True
        except Exception as e:
            self.logger.error(f"Send failed: {e}")
            return False

    def _send_packet(self, cmd: int, payload: bytes):
        if not self.ser:
            return
        self.ser.write(build_packet(cmd, payload))

    def _read_packet(self) -> Optional[bytes]:
        if not self.ser:
            return None
        if hasattr(self.ser, "pop_next_packet"):
            return self.ser.pop_next_packet()
        if getattr(self.ser, "in_waiting", 0) < 5:
            return None
        head = self.ser.read(2)
        if head != PACKET_HEAD:
            return None
        header = self.ser.read(2)
        if len(header) < 2:
            return None
        cmd = header[0]
        payload_length = header[1]
        payload = self.ser.read(payload_length)
        if len(payload) < payload_length:
            return None
        checksum = self.ser.read(1)
        if len(checksum) < 1:
            return None
        return head + bytes([cmd, payload_length]) + payload + checksum

    def poll_once(self) -> bool:
        packet = self._read_packet()
        if not packet:
            return False
        try:
            cmd, payload = parse_packet(packet)
        except ValueError as exc:
            self.logger.warning(f"Ignoring invalid frame: {exc}")
            return False

        if cmd == CMD_SENSOR_STATE:
            with self.lock:
                self.motor_states.update(decode_motor_state_payload(payload))
            return True
        return False

    def _read_loop(self):
        while self.running and self.ser:
            if not self.poll_once():
                if (
                    getattr(self.ser, "is_replay", False)
                    and not self.ser.has_pending_packets()
                ):
                    break
                time.sleep(0.001)

    def get_state(self):
        with self.lock:
            return {"motors": self.motor_states.copy(), "imu": self.imu_data.copy()}
