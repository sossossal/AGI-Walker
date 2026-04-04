import socket
import struct
import logging
from typing import Optional, List
from .proto import robot_protocol_pb2

logger = logging.getLogger(__name__)

class BinaryGodotClient:
    """
    AGI-Walker V2.0 High-Performance Binary Client using Protobuf.
    Replaces JSON-based communication to reduce CPU overhead and latency.
    """
    
    def __init__(self, host: str = "127.0.0.1", port: int = 4242):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.is_connected = False

    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self.sock.setblocking(False)
            self.is_connected = True
            logger.info(f"Connected to Godot (Binary) at {self.host}:{self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False

    def send_message(self, msg: robot_protocol_pb2.RobotMessage) -> bool:
        if not self.is_connected or not self.sock:
            return False
        
        try:
            # 序列化 Protobuf 消息
            data = msg.SerializeToString()
            # 添加 4 字节大端长度前缀 (Length-prefix)
            header = struct.pack(">I", len(data))
            self.sock.sendall(header + data)
            return True
        except Exception as e:
            logger.error(f"Send error: {e}")
            self.is_connected = False
            return False

    def receive_message(self) -> Optional[robot_protocol_pb2.RobotMessage]:
        """
        Non-blocking receive. Returns RobotMessage if a full packet is available.
        """
        if not self.is_connected or not self.sock:
            return None

        try:
            # 1. 读取 4 字节头部获取消息长度
            header = self.sock.recv(4)
            if not header:
                return None
            
            msg_len = struct.unpack(">I", header)[0]
            
            # 2. 读取完整的消息体
            chunks = []
            bytes_recvd = 0
            while bytes_recvd < msg_len:
                chunk = self.sock.recv(min(msg_len - bytes_recvd, 4096))
                if not chunk:
                    raise ConnectionError("Socket closed during read")
                chunks.append(chunk)
                bytes_recvd += len(chunk)
            
            full_data = b"".join(chunks)
            
            # 3. 解析 Protobuf
            msg = robot_protocol_pb2.RobotMessage()
            msg.ParseFromString(full_data)
            return msg
            
        except BlockingIOError:
            # 无数据可用
            return None
        except Exception as e:
            logger.error(f"Receive error: {e}")
            self.is_connected = False
            return None

    def send_motor_commands(self, commands: List[robot_protocol_pb2.MotorCommand]):
        """Helper to send a batch of motor commands."""
        msg = robot_protocol_pb2.RobotMessage()
        msg.type = robot_protocol_pb2.RobotMessage.COMMAND
        msg.commands.extend(commands)
        return self.send_message(msg)

    def close(self):
        if self.sock:
            self.sock.close()
        self.is_connected = False
