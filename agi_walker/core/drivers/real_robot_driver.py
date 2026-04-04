try:
    import serial
except ImportError:
    serial = None  # Mock 模式下不需要 pyserial

import time
import struct
import logging
import threading
from typing import Any, Dict, List, Optional, Tuple

# 通信协议:
# Head (2B): 0xAA 0x55
# Cmd (1B): 0x01 (电机指令), 0x02 (传感器请求), 0x03 (SysID 数据)
# Len (1B): Payload 长度
# Payload (N)
# CRC (1B): 校验和

class RealRobotDriver:
    """真实硬件驱动 — 通过串口/TCP 控制机器人"""

    def __init__(self, port: str = "COM3", baudrate: int = 115200, mock: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.mock = mock
        self.ser: Optional[Any] = None
        self.running = False
        self.lock = threading.Lock()
        
        # Robot State
        self.motor_states: Dict[str, Dict] = {} # {id: {pos, vel, torque, temp}}
        self.imu_data = {'rpy': [0,0,0], 'acc': [0,0,0], 'gyro': [0,0,0]}
        
        self.logger = logging.getLogger("RealRobotDriver")
        
    def connect(self) -> bool:
        if self.mock:
            self.logger.info(f"[Mock] Connected to {self.port}")
            self.running = True
            return True
            
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
                    if mid not in self.motor_states: self.motor_states[mid] = {}
                    self.motor_states[mid]['pos'] = val
                    self.motor_states[mid]['vel'] = 0.0
                    self.motor_states[mid]['torque'] = 0.0
            return True

        # Pack data (Example: 0x01 + Count + [ID, Pos_High, Pos_Low]...)
        # Simplified for demo: Just sending text JSON for ESP32/Arduino parsing for now in MVP
        # Or binary struct for performance. Let's use simple binary.
        # [Cmd, NumMotors, ID1, Pos1(float), ID2, Pos2(float)...]
        try:
            payload = bytearray()
            payload.append(len(commands))
            for mid, pos in commands.items():
                # Assuming ID is int suffix of 'motor_X'
                motor_id = int(mid.split('_')[-1]) if '_' in mid else 0
                payload.append(motor_id)
                payload.extend(struct.pack('f', pos))
            
            self._send_packet(0x01, payload)
            return True
        except Exception as e:
            self.logger.error(f"Send failed: {e}")
            return False

    def _send_packet(self, cmd: int, payload: bytes):
        if not self.ser: return
        
        frame = bytearray([0xAA, 0x55, cmd, len(payload)])
        frame.extend(payload)
        checksum = sum(frame) % 256
        frame.append(checksum)
        self.ser.write(frame)

    def _read_loop(self):
        while self.running and self.ser:
            # Implement reading logic (Head sync -> Read Lenovo -> Read Payload -> CRC)
            # For now, just a placeholder
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                # Parse data...
                pass
            time.sleep(0.001)

    def get_state(self):
        with self.lock:
            return {
                'motors': self.motor_states.copy(),
                'imu': self.imu_data.copy()
            }
