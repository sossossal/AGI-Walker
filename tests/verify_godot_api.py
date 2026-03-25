"""
验证 Godot API 接口
"""

import logging
logger = logging.getLogger(__name__)
import socket
import json
import time
import sys


def verify_api(host="127.0.0.1", port=9999):
    logger.info(f"Connecting to Godot at {host}:{port}...")

    try:
        # 创建 Socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        sock.connect((host, port))

        # 测试 1: 发送 Handshake
        handshake = {"type": "handshake", "client": "verify_script", "version": "1.0"}
        sock.sendall(json.dumps(handshake).encode())

        # 等待回复
        data = sock.recv(1024)
        if data:
            response = json.loads(data.decode())
            logger.info(f"✅ Handshake Success: {response}")
        else:
            logger.info("❌ No response from Godot")
            return False

        # 测试 2: 发送 Motor Commands
        motor_cmd = {
            "type": "control",
            "motors": {"hip_left": 1.0, "hip_right": -1.0},
            "timestamp": time.time(),
        }
        sock.sendall(json.dumps(motor_cmd).encode())
        logger.info("✅ Motor Command Sent")

        # 测试 3: 获取传感器数据
        data = sock.recv(4096)
        if data:
            sensors = json.loads(data.decode())
            logger.info(f"✅ Received Sensor Data: {sensors.keys()}")
            if "imu" in sensors.get("sensors", {}):
                logger.info(f"   IMU Orientation: {sensors['sensors']['imu']['orient']}")
        else:
            logger.info("❌ No sensor data received")

        sock.close()
        logger.info("\n🎉 Godot API Verification Finished Successfully!")
        return True

    except ConnectionRefusedError:
        logger.info("❌ Connection Refused: Ensure Godot is running and TCP server is started.")
        return False
    except socket.timeout:
        logger.info("❌ Connection Timeout")
        return False
    except Exception as e:
        logger.error(f"❌ Unexpected Error: {e}")
        return False


if __name__ == "__main__":
    success = verify_api()
    sys.exit(0 if success else 1)
