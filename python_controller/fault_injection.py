"""
故障注入测试框架
用于训练和测试系统鲁棒性
"""

import time
import random
import numpy as np
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum


class FaultType(Enum):
    """故障类型"""
    SENSOR_NOISE = "sensor_noise"           # 传感器噪声
    SENSOR_DROPOUT = "sensor_dropout"       # 传感器丢失
    SENSOR_BIAS = "sensor_bias"             # 传感器偏移
    ACTUATOR_DELAY = "actuator_delay"       # 执行器延迟
    ACTUATOR_FAILURE = "actuator_failure"   # 执行器故障
    COMMUNICATION_LAG = "communication_lag" # 通信延迟
    SUDDEN_DISTURBANCE = "sudden_disturbance"  # 突然扰动
    STUCK_SENSOR = "stuck_sensor"           # 传感器卡死


@dataclass
class FaultConfig:
    """故障配置"""
    fault_type: FaultType
    intensity: float = 1.0      # 强度 0-1
    probability: float = 1.0    # 触发概率 0-1
    duration: float = 0.0       # 持续时间（0表示单次）
    start_time: float = 0.0     # 开始时间
    target: str = ""            # 目标（如传感器名称）


class FaultInjector:
    """
    故障注入器
    
    支持多种故障类型的注入，用于测试系统鲁棒性
    """
    
    def __init__(self, seed: Optional[int] = None):
        if seed is not None:
            random.seed(seed)
            np.random.seed(seed)
        
        # 当前活跃故障
        self.active_faults: List[FaultConfig] = []
        
        # 故障历史
        self.fault_history: List[Dict] = []
        
        # 状态
        self.enabled = False
        self.start_time = 0.0
        
        # 延迟队列（用于模拟通信延迟）
        self.delay_queue: List[tuple] = []
        
        # 统计
        self.injections_count = 0
        self.data_modified_count = 0
    
    def enable(self):
        """启用故障注入"""
        self.enabled = True
        self.start_time = time.time()
        print("⚠️ 故障注入已启用")
    
    def disable(self):
        """禁用故障注入"""
        self.enabled = False
        self.active_faults.clear()
        print("✅ 故障注入已禁用")
    
    def add_fault(self, config: FaultConfig):
        """添加故障配置"""
        self.active_faults.append(config)
        self.fault_history.append({
            "timestamp": time.time(),
            "action": "add",
            "fault": config.fault_type.value
        })
    
    def remove_fault(self, fault_type: FaultType):
        """移除指定类型的故障"""
        self.active_faults = [
            f for f in self.active_faults
            if f.fault_type != fault_type
        ]
    
    def inject(self, data: dict, data_type: str = "sensor") -> dict:
        """
        注入故障
        
        Args:
            data: 要注入故障的数据
            data_type: 数据类型 ("sensor" 或 "action")
        
        Returns:
            注入故障后的数据
        """
        if not self.enabled:
            return data
        
        # 深拷贝避免修改原数据
        import copy
        data = copy.deepcopy(data)
        
        current_time = time.time() - self.start_time
        modified = False
        
        for fault in self.active_faults:
            # 检查概率
            if random.random() > fault.probability:
                continue
            
            # 检查时间窗口
            if fault.duration > 0:
                if current_time < fault.start_time:
                    continue
                if current_time > fault.start_time + fault.duration:
                    continue
            
            # 应用故障
            if data_type == "sensor":
                data, was_modified = self._inject_sensor_fault(data, fault)
            elif data_type == "action":
                data, was_modified = self._inject_action_fault(data, fault)
            else:
                was_modified = False
            
            if was_modified:
                modified = True
                self.injections_count += 1
        
        if modified:
            self.data_modified_count += 1
        
        return data
    
    def _inject_sensor_fault(
        self,
        data: dict,
        fault: FaultConfig
    ) -> tuple:
        """注入传感器故障"""
        modified = False
        
        if fault.fault_type == FaultType.SENSOR_NOISE:
            data = self._inject_sensor_noise(data, fault.intensity)
            modified = True
            
        elif fault.fault_type == FaultType.SENSOR_DROPOUT:
            data = self._inject_sensor_dropout(data, fault.intensity, fault.target)
            modified = True
            
        elif fault.fault_type == FaultType.SENSOR_BIAS:
            data = self._inject_sensor_bias(data, fault.intensity)
            modified = True
            
        elif fault.fault_type == FaultType.STUCK_SENSOR:
            data = self._inject_stuck_sensor(data, fault.target)
            modified = True
            
        elif fault.fault_type == FaultType.SUDDEN_DISTURBANCE:
            data = self._inject_disturbance(data, fault.intensity)
            modified = True
        
        return data, modified
    
    def _inject_action_fault(
        self,
        data: dict,
        fault: FaultConfig
    ) -> tuple:
        """注入动作故障"""
        modified = False
        
        if fault.fault_type == FaultType.ACTUATOR_DELAY:
            # 延迟处理在外部
            modified = True
            
        elif fault.fault_type == FaultType.ACTUATOR_FAILURE:
            data = self._inject_actuator_failure(data, fault.target)
            modified = True
        
        return data, modified
    
    def _inject_sensor_noise(self, data: dict, intensity: float) -> dict:
        """注入传感器噪声"""
        sensors = data.get('sensors', {})
        
        # IMU噪声
        if 'imu' in sensors:
            imu = sensors['imu']
            
            # 姿态噪声
            if 'orient' in imu:
                noise = np.random.normal(0, intensity * 5, 3)  # 标准差=5*intensity度
                imu['orient'] = [o + n for o, n in zip(imu['orient'], noise)]
            
            # 陀螺仪噪声
            if 'gyro' in imu:
                noise = np.random.normal(0, intensity * 0.5, 3)
                imu['gyro'] = [g + n for g, n in zip(imu['gyro'], noise)]
            
            # 加速度计噪声
            if 'accel' in imu:
                noise = np.random.normal(0, intensity * 0.3, 3)
                imu['accel'] = [a + n for a, n in zip(imu['accel'], noise)]
        
        # 关节噪声
        if 'joints' in sensors:
            for joint_name, joint_data in sensors['joints'].items():
                if 'angle' in joint_data:
                    noise = np.random.normal(0, intensity * 2)
                    joint_data['angle'] += noise
        
        # 高度噪声
        if 'torso_height' in data:
            noise = np.random.normal(0, intensity * 0.02)
            data['torso_height'] += noise
        
        return data
    
    def _inject_sensor_dropout(
        self,
        data: dict,
        intensity: float,
        target: str
    ) -> dict:
        """注入传感器丢失"""
        sensors = data.get('sensors', {})
        
        if not target or target == "random":
            # 随机选择要丢失的传感器
            dropout_prob = intensity * 0.3
            
            if random.random() < dropout_prob and 'imu' in sensors:
                sensors['imu']['orient'] = [0, 0, 0]
                sensors['imu']['_dropout'] = True
            
            if random.random() < dropout_prob and 'joints' in sensors:
                for joint_name in sensors['joints']:
                    if random.random() < dropout_prob:
                        sensors['joints'][joint_name]['angle'] = 0
                        sensors['joints'][joint_name]['_dropout'] = True
        else:
            # 指定传感器丢失
            if target == "imu" and 'imu' in sensors:
                sensors['imu']['orient'] = [0, 0, 0]
                sensors['imu']['_dropout'] = True
            elif target in sensors.get('joints', {}):
                sensors['joints'][target]['angle'] = 0
                sensors['joints'][target]['_dropout'] = True
        
        return data
    
    def _inject_sensor_bias(self, data: dict, intensity: float) -> dict:
        """注入传感器偏移"""
        sensors = data.get('sensors', {})
        
        # 固定偏移（在会话中保持一致）
        bias_roll = intensity * 3
        bias_pitch = -intensity * 2
        
        if 'imu' in sensors and 'orient' in sensors['imu']:
            sensors['imu']['orient'][0] += bias_roll
            sensors['imu']['orient'][1] += bias_pitch
        
        return data
    
    def _inject_stuck_sensor(self, data: dict, target: str) -> dict:
        """注入传感器卡死"""
        sensors = data.get('sensors', {})
        
        # 使用固定值
        if target == "imu" and 'imu' in sensors:
            sensors['imu']['orient'] = [5.0, -3.0, 0.0]  # 卡在某个值
            sensors['imu']['_stuck'] = True
        elif target in sensors.get('joints', {}):
            sensors['joints'][target]['angle'] = 10.0
            sensors['joints'][target]['_stuck'] = True
        
        return data
    
    def _inject_disturbance(self, data: dict, intensity: float) -> dict:
        """注入突然扰动（模拟外力）"""
        sensors = data.get('sensors', {})
        
        if 'imu' in sensors and 'orient' in sensors['imu']:
            # 随机方向的冲击
            direction = random.choice(['roll', 'pitch', 'both'])
            magnitude = intensity * 15  # 最大15度冲击
            
            if direction in ('roll', 'both'):
                sensors['imu']['orient'][0] += random.uniform(-magnitude, magnitude)
            if direction in ('pitch', 'both'):
                sensors['imu']['orient'][1] += random.uniform(-magnitude, magnitude)
            
            sensors['imu']['_disturbance'] = True
        
        return data
    
    def _inject_actuator_failure(self, data: dict, target: str) -> dict:
        """注入执行器故障"""
        motors = data.get('motors', {})
        
        if not target or target == "random":
            target = random.choice(list(motors.keys())) if motors else None
        
        if target and target in motors:
            motors[target] = 0  # 执行器锁死在0位置
            data['_actuator_failure'] = target
        
        return data
    
    def get_delayed_action(
        self,
        action: dict,
        delay_ms: float
    ) -> Optional[dict]:
        """
        处理延迟动作
        
        添加动作到延迟队列，返回应该执行的动作
        """
        current_time = time.time()
        
        # 添加当前动作到队列
        self.delay_queue.append((current_time + delay_ms / 1000, action))
        
        # 返回已到期的动作
        ready_actions = [
            (t, a) for t, a in self.delay_queue
            if t <= current_time
        ]
        
        if ready_actions:
            # 移除已到期的
            self.delay_queue = [
                (t, a) for t, a in self.delay_queue
                if t > current_time
            ]
            # 返回最新的到期动作
            return ready_actions[-1][1]
        
        return None
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "enabled": self.enabled,
            "active_faults": [f.fault_type.value for f in self.active_faults],
            "injections_count": self.injections_count,
            "data_modified_count": self.data_modified_count,
            "delay_queue_size": len(self.delay_queue)
        }
    
    def reset(self):
        """重置状态"""
        self.active_faults.clear()
        self.fault_history.clear()
        self.delay_queue.clear()
        self.injections_count = 0
        self.data_modified_count = 0


def create_robustness_test_suite() -> List[FaultConfig]:
    """创建鲁棒性测试套件"""
    return [
        # 轻度噪声（始终开启）
        FaultConfig(
            fault_type=FaultType.SENSOR_NOISE,
            intensity=0.3,
            probability=1.0
        ),
        
        # 偶发传感器丢失
        FaultConfig(
            fault_type=FaultType.SENSOR_DROPOUT,
            intensity=0.5,
            probability=0.05
        ),
        
        # 偶发突然扰动
        FaultConfig(
            fault_type=FaultType.SUDDEN_DISTURBANCE,
            intensity=0.5,
            probability=0.02
        ),
        
        # 传感器偏移
        FaultConfig(
            fault_type=FaultType.SENSOR_BIAS,
            intensity=0.5,
            probability=1.0,
            duration=10.0,
            start_time=30.0
        ),
    ]


# 测试代码
if __name__ == "__main__":
    import json
    
    print("故障注入测试框架测试\n")
    
    # 创建注入器
    injector = FaultInjector(seed=42)
    
    # 正常传感器数据
    sensor_data = {
        "sensors": {
            "imu": {"orient": [5.0, -3.0, 0.0], "gyro": [0.1, 0.0, 0.0], "accel": [0, 0, -9.8]},
            "joints": {
                "hip_left": {"angle": 10.0, "velocity": 0.5},
                "hip_right": {"angle": -8.0, "velocity": -0.3}
            }
        },
        "torso_height": 1.45
    }
    
    print("=== 原始数据 ===")
    print(f"IMU姿态: {sensor_data['sensors']['imu']['orient']}")
    
    # 启用故障注入
    injector.enable()
    
    # 添加噪声故障
    print("\n=== 测试传感器噪声 ===")
    injector.add_fault(FaultConfig(
        fault_type=FaultType.SENSOR_NOISE,
        intensity=0.5,
        probability=1.0
    ))
    
    for i in range(3):
        injected = injector.inject(sensor_data, "sensor")
        print(f"注入后姿态: {[f'{x:.2f}' for x in injected['sensors']['imu']['orient']]}")
    
    # 清除并测试丢失
    injector.active_faults.clear()
    
    print("\n=== 测试传感器丢失 ===")
    injector.add_fault(FaultConfig(
        fault_type=FaultType.SENSOR_DROPOUT,
        intensity=1.0,
        probability=1.0,
        target="hip_left"
    ))
    
    injected = injector.inject(sensor_data, "sensor")
    print(f"hip_left角度: {injected['sensors']['joints']['hip_left']['angle']}")
    print(f"dropout标记: {injected['sensors']['joints']['hip_left'].get('_dropout', False)}")
    
    # 测试扰动
    injector.active_faults.clear()
    
    print("\n=== 测试突然扰动 ===")
    injector.add_fault(FaultConfig(
        fault_type=FaultType.SUDDEN_DISTURBANCE,
        intensity=1.0,
        probability=1.0
    ))
    
    injected = injector.inject(sensor_data, "sensor")
    print(f"扰动后姿态: {[f'{x:.2f}' for x in injected['sensors']['imu']['orient']]}")
    
    # 统计
    print("\n=== 统计信息 ===")
    print(json.dumps(injector.get_stats(), indent=2))
    
    # 禁用
    injector.disable()
