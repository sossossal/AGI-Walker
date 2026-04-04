import time
import logging
import threading
from typing import Optional
from ..api.comm.zenoh_interface import ZenohInterface, ZenohConfig
from ..api.comm.proto import robot_protocol_pb2

logger = logging.getLogger(__name__)

class SidecarLite:
    """
    AGI-Walker V2.0 Sidecar-Lite.
    A resource-efficient agent for edge devices to sync critical robot state.
    """
    def __init__(self, robot_id: str, zenoh_endpoint: str = "tcp/127.0.0.1:7447"):
        self.robot_id = robot_id
        self.zenoh = ZenohInterface(ZenohConfig(connect=zenoh_endpoint, mode="client"))
        self.running = False
        
        # 资源键
        self.state_key = f"rt/robot/{robot_id}/state"
        self.cmd_key = f"rt/robot/{robot_id}/cmd"

    def start(self):
        self.running = True
        logger.info(f"Sidecar-Lite started for Robot: {self.robot_id}")
        
        # 1. 订阅控制指令 (Protobuf)
        self.zenoh.declare_subscriber(
            self.cmd_key, 
            self._on_command_received,
            pb_class=robot_protocol_pb2.RobotMessage
        )
        
        # 2. 维持心跳 (1Hz)
        while self.running:
            self._send_heartbeat()
            time.sleep(1.0)

    def _send_heartbeat(self):
        """发送最小化的机器人状态"""
        msg = robot_protocol_pb2.RobotMessage()
        msg.type = robot_protocol_pb2.RobotMessage.PING
        msg.msg_id = int(time.time())
        msg.diagnostic_info = f"Robot {self.robot_id} is alive (Lite Mode)"
        
        self.zenoh.publish(self.state_key, msg)

    def _on_command_received(self, msg: robot_protocol_pb2.RobotMessage):
        """处理来自云端或中控的指令"""
        if msg.type == robot_protocol_pb2.RobotMessage.COMMAND:
            logger.info(f"Sidecar-Lite received {len(msg.commands)} motor commands.")
            # 在实际部署中，这里会将指令转发给本地 TCP 控制器端口

    def stop(self):
        self.running = False
        self.zenoh.close()
        logger.info("Sidecar-Lite shutdown.")

if __name__ == "__main__":
    sidecar = SidecarLite(robot_id="walker-01")
    try:
        sidecar.start()
    except KeyboardInterrupt:
        sidecar.stop()
