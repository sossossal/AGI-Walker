"""
AGI-Walker 硬件控制器接口
用于与 IMC-22 Reflex 控制器通信
"""

import can
import struct
import time
from typing import Dict, List, Optional, Tuple
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class IMC22Controller:
    """IMC-22 硬件控制器接口"""
    
    # CAN ID 定义
    ID_SYNC = 0x000
    ID_STATUS_BASE = 0x100
    ID_COMMAND_BASE = 0x200
    ID_CONFIG_BASE = 0x300
    ID_HANDSHAKE = 0x7FF
    
    def __init__(self, channel='can0', bustype='socketcan', bitrate=1000000):
        """
        初始化硬件控制器
        
        Args:
            channel: CAN 通道 (Linux: 'can0', Windows: 'PCAN_USBBUS1')
            bustype: 总线类型 (Linux: 'socketcan', Windows: 'pcan')
            bitrate: 波特率 (默认 1 Mbps)
        """
        try:
            self.bus = can.interface.Bus(
                channel=channel,
                bustype=bustype,
                bitrate=bitrate
            )
            logger.info(f"CAN 总线已连接: {channel} @ {bitrate} bps")
        except Exception as e:
            logger.error(f"无法连接 CAN 总线: {e}")
            raise
        
        self.node_states = {}  # 存储各节点状态
        
    def send_command(self, node_id: int, target_angle: float, compliance: float = 0.5):
        """
        发送控制命令到指定节点
        
        Args:
            node_id: 节点 ID (1-255)
            target_angle: 目标角度 (度)
            compliance: 柔顺系数 (0.0 = 刚性, 1.0 = 柔性)
        """
        # 角度转换为 int16 (单位: 0.01度)
        angle_int16 = int(target_angle * 100)
        angle_int16 = max(-32768, min(32767, angle_int16))  # 限幅
        
        # 柔顺系数转换为 uint8
        compliance_u8 = int(compliance * 255)
        compliance_u8 = max(0, min(255, compliance_u8))
        
        # 构造消息
        data = struct.pack('<hB', angle_int16, compliance_u8)
        
        msg = can.Message(
            arbitration_id=self.ID_COMMAND_BASE + node_id,
            data=data,
            is_extended_id=False
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
        if msg.arbitration_id >= self.ID_STATUS_BASE and \
           msg.arbitration_id < self.ID_COMMAND_BASE:
            
            node_id = msg.arbitration_id - self.ID_STATUS_BASE
            
            # 解析数据 (假设格式: angle:int16, current:uint16, error:int16)
            angle_raw, current_raw, error_raw = struct.unpack('<hhH', msg.data[:6])
            
            status = {
                'node_id': node_id,
                'angle': angle_raw * 0.01,      # 转换为度
                'current': current_raw * 0.001,  # 转换为安培
                'error': error_raw * 0.01       # 转换为度
            }
            
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
                states[status['node_id']] = status
        
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
        data = struct.pack('<fff', max_torque, kp, ki)
        
        msg = can.Message(
            arbitration_id=self.ID_CONFIG_BASE + node_id,
            data=data,
            is_extended_id=False
        )
        
        self.bus.send(msg)
        logger.info(f"节点 {node_id} 配置已更新")
    
    def discover_nodes(self, timeout: float = 2.0) -> List[int]:
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
                discovered.add(status['node_id'])
        
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
    
    def __init__(self, num_joints: int = 12, control_freq_hz: int = 100):
        """
        Args:
            num_joints: 关节数量
            control_freq_hz: 控制频率 (Hz)
        """
        self.controller = IMC22Controller()
        self.num_joints = num_joints
        self.control_period = 1.0 / control_freq_hz
        
        # 发现节点
        self.node_ids = self.controller.discover_nodes()
        if len(self.node_ids) != num_joints:
            logger.warning(f"期望 {num_joints} 个节点，实际发现 {len(self.node_ids)} 个")
    
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
        info = {'states': states}
        
        return obs, reward, terminated, truncated, info
   
    def _build_observation(self, states: Dict) -> List[float]:
        """从节点状态构建观察"""
        obs = []
        for node_id in sorted(self.node_ids):
            if node_id in states:
                obs.extend([
                    states[node_id]['angle'],
                    states[node_id]['current'],
                    states[node_id]['error']
                ])
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
