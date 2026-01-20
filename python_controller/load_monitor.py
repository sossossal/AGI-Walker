"""
负载监控器（Load Monitor）
边缘层负载监控脚本，实现动态负载均衡和PID fallback
"""

import time
from typing import Dict, Optional, Callable, Deque
from collections import deque
from dataclasses import dataclass
from enum import Enum


class ControlMode(Enum):
    """控制模式"""
    AI = "ai"           # AI模型控制
    PID = "pid"         # PID控制（fallback）
    HYBRID = "hybrid"   # 混合模式


@dataclass
class LoadMonitorConfig:
    """负载监控配置"""
    latency_threshold_ms: float = 20.0      # 延迟阈值（毫秒）
    history_size: int = 100                  # 历史记录大小
    fallback_trigger_count: int = 5         # 触发fallback的连续超标次数
    recovery_trigger_count: int = 20        # 触发恢复的连续正常次数
    smoothing_factor: float = 0.3           # 指数移动平均平滑因子
    hybrid_blend_factor: float = 0.7        # 混合模式中AI权重


class LoadMonitor:
    """
    负载监控与动态切换
    
    功能：
    1. 监控AI推理延迟
    2. 当延迟超过阈值时自动切换到PID控制
    3. 延迟恢复正常时自动切回AI控制
    4. 支持混合模式平滑过渡
    """
    
    def __init__(
        self,
        pid_controller,
        config: Optional[LoadMonitorConfig] = None
    ):
        """
        初始化负载监控器
        
        Args:
            pid_controller: PID控制器实例
            config: 配置对象
        """
        self.pid_controller = pid_controller
        self.config = config or LoadMonitorConfig()
        
        # 延迟历史
        self.latency_history: Deque[float] = deque(maxlen=self.config.history_size)
        self.ema_latency: float = 0.0  # 指数移动平均延迟
        
        # 状态
        self.current_mode = ControlMode.AI
        self.consecutive_over_threshold = 0
        self.consecutive_under_threshold = 0
        
        # 统计
        self.mode_switches = 0
        self.total_samples = 0
        self.over_threshold_count = 0
        
        # 回调
        self.on_mode_change: Optional[Callable[[ControlMode, ControlMode], None]] = None
    
    def record_latency(self, latency_ms: float):
        """
        记录推理延迟
        
        Args:
            latency_ms: 延迟（毫秒）
        """
        self.latency_history.append(latency_ms)
        self.total_samples += 1
        
        # 更新EMA
        if self.ema_latency == 0:
            self.ema_latency = latency_ms
        else:
            self.ema_latency = (
                self.config.smoothing_factor * latency_ms +
                (1 - self.config.smoothing_factor) * self.ema_latency
            )
        
        # 检查是否超标
        if latency_ms > self.config.latency_threshold_ms:
            self.over_threshold_count += 1
            self.consecutive_over_threshold += 1
            self.consecutive_under_threshold = 0
        else:
            self.consecutive_under_threshold += 1
            self.consecutive_over_threshold = 0
        
        # 检查是否需要切换模式
        self._check_mode_switch()
    
    def _check_mode_switch(self):
        """检查是否需要切换控制模式"""
        old_mode = self.current_mode
        
        if self.current_mode == ControlMode.AI:
            # AI模式下，延迟超标切换到PID
            if self.consecutive_over_threshold >= self.config.fallback_trigger_count:
                self.current_mode = ControlMode.PID
                self.mode_switches += 1
                print(f"⚠️ 延迟超标({self.ema_latency:.1f}ms)，切换到PID控制")
        
        elif self.current_mode == ControlMode.PID:
            # PID模式下，延迟恢复正常切回AI
            if self.consecutive_under_threshold >= self.config.recovery_trigger_count:
                self.current_mode = ControlMode.HYBRID  # 先进入混合模式过渡
                print(f"✅ 延迟恢复正常({self.ema_latency:.1f}ms)，进入混合模式")
        
        elif self.current_mode == ControlMode.HYBRID:
            # 混合模式下，持续正常则切回AI
            if self.consecutive_under_threshold >= self.config.recovery_trigger_count * 2:
                self.current_mode = ControlMode.AI
                self.mode_switches += 1
                print(f"✅ 延迟稳定正常，切回AI控制")
        
        # 触发回调
        if old_mode != self.current_mode and self.on_mode_change:
            self.on_mode_change(old_mode, self.current_mode)
    
    def should_fallback(self) -> bool:
        """
        判断是否需要切换到fallback模式
        
        Returns:
            是否应使用PID控制
        """
        return self.current_mode in (ControlMode.PID, ControlMode.HYBRID)
    
    def get_control_action(
        self,
        sensor_data: dict,
        ai_action: Optional[dict] = None
    ) -> dict:
        """
        获取控制动作
        
        Args:
            sensor_data: 传感器数据
            ai_action: AI模型的动作（如果已计算）
        
        Returns:
            控制动作字典
        """
        if self.current_mode == ControlMode.AI:
            if ai_action is not None:
                return ai_action
            return {"use_ai": True}
        
        elif self.current_mode == ControlMode.PID:
            pid_action = self._compute_pid_action(sensor_data)
            pid_action['_control_mode'] = 'pid'
            return pid_action
        
        elif self.current_mode == ControlMode.HYBRID:
            # 混合模式：融合AI和PID
            pid_action = self._compute_pid_action(sensor_data)
            
            if ai_action is not None:
                blended = self._blend_actions(ai_action, pid_action)
                blended['_control_mode'] = 'hybrid'
                return blended
            
            pid_action['_control_mode'] = 'hybrid_pid_only'
            return pid_action
    
    def _compute_pid_action(self, sensor_data: dict) -> dict:
        """计算PID控制动作"""
        orient = sensor_data['sensors']['imu']['orient']
        roll = orient[0]
        pitch = orient[1]
        
        # 使用PID控制器计算
        # 这里假设pid_controller有compute方法
        if hasattr(self.pid_controller, 'compute'):
            hip_left, hip_right = self.pid_controller.compute(roll, pitch)
        else:
            # 简单PID计算（备用）
            kp = 2.0
            hip_left = -kp * pitch - kp * roll
            hip_right = -kp * pitch + kp * roll
        
        return {
            "motors": {
                "hip_left": float(hip_left),
                "hip_right": float(hip_right)
            },
            "confidence": 0.8
        }
    
    def _blend_actions(self, ai_action: dict, pid_action: dict) -> dict:
        """融合AI和PID动作"""
        blend = self.config.hybrid_blend_factor
        
        ai_motors = ai_action.get('motors', {})
        pid_motors = pid_action.get('motors', {})
        
        blended_motors = {}
        for key in ['hip_left', 'hip_right']:
            ai_val = ai_motors.get(key, 0.0)
            pid_val = pid_motors.get(key, 0.0)
            blended_motors[key] = blend * ai_val + (1 - blend) * pid_val
        
        return {
            "motors": blended_motors,
            "confidence": blend * ai_action.get('confidence', 0.5) + (1 - blend) * 0.8
        }
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        avg_latency = (
            sum(self.latency_history) / len(self.latency_history)
            if self.latency_history else 0
        )
        
        return {
            "current_mode": self.current_mode.value,
            "ema_latency_ms": self.ema_latency,
            "avg_latency_ms": avg_latency,
            "total_samples": self.total_samples,
            "over_threshold_count": self.over_threshold_count,
            "over_threshold_rate": (
                self.over_threshold_count / self.total_samples
                if self.total_samples > 0 else 0
            ),
            "mode_switches": self.mode_switches,
            "consecutive_over": self.consecutive_over_threshold,
            "consecutive_under": self.consecutive_under_threshold
        }
    
    def reset(self):
        """重置状态"""
        self.latency_history.clear()
        self.ema_latency = 0.0
        self.current_mode = ControlMode.AI
        self.consecutive_over_threshold = 0
        self.consecutive_under_threshold = 0
        self.mode_switches = 0
        self.total_samples = 0
        self.over_threshold_count = 0


class SimplePIDController:
    """简单PID控制器（用于fallback）"""
    
    def __init__(
        self,
        kp: float = 2.0,
        ki: float = 0.1,
        kd: float = 0.5,
        max_output: float = 45.0
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        # 积分项
        self.integral_roll = 0.0
        self.integral_pitch = 0.0
        
        # 上一次误差
        self.last_roll = 0.0
        self.last_pitch = 0.0
        
        # 积分限幅
        self.integral_limit = 10.0
    
    def compute(self, roll: float, pitch: float, dt: float = 0.033) -> tuple:
        """
        计算PID输出
        
        Args:
            roll: Roll角度
            pitch: Pitch角度
            dt: 时间步长
        
        Returns:
            (hip_left, hip_right) 目标角度
        """
        # 目标是保持直立（roll=0, pitch=0）
        error_roll = 0 - roll
        error_pitch = 0 - pitch
        
        # 积分项
        self.integral_roll += error_roll * dt
        self.integral_pitch += error_pitch * dt
        
        # 积分限幅
        self.integral_roll = max(-self.integral_limit, min(self.integral_limit, self.integral_roll))
        self.integral_pitch = max(-self.integral_limit, min(self.integral_limit, self.integral_pitch))
        
        # 微分项
        d_roll = (error_roll - self.last_roll) / dt if dt > 0 else 0
        d_pitch = (error_pitch - self.last_pitch) / dt if dt > 0 else 0
        
        # PID输出
        output_roll = self.kp * error_roll + self.ki * self.integral_roll + self.kd * d_roll
        output_pitch = self.kp * error_pitch + self.ki * self.integral_pitch + self.kd * d_pitch
        
        # 更新上一次误差
        self.last_roll = error_roll
        self.last_pitch = error_pitch
        
        # 计算髋关节角度
        hip_left = output_pitch - output_roll
        hip_right = output_pitch + output_roll
        
        # 输出限幅
        hip_left = max(-self.max_output, min(self.max_output, hip_left))
        hip_right = max(-self.max_output, min(self.max_output, hip_right))
        
        return hip_left, hip_right
    
    def reset(self):
        """重置积分项"""
        self.integral_roll = 0.0
        self.integral_pitch = 0.0
        self.last_roll = 0.0
        self.last_pitch = 0.0


# 测试代码
if __name__ == "__main__":
    import json
    import random
    
    print("负载监控器测试\n")
    
    # 创建PID控制器
    pid = SimplePIDController(kp=2.0, ki=0.1, kd=0.5)
    
    # 创建负载监控器
    config = LoadMonitorConfig(
        latency_threshold_ms=20.0,
        fallback_trigger_count=5,
        recovery_trigger_count=10
    )
    monitor = LoadMonitor(pid, config)
    
    # 模拟传感器数据
    sensor_data = {
        "sensors": {
            "imu": {"orient": [5.0, -3.0, 0.0]},
            "joints": {
                "hip_left": {"angle": 0.0},
                "hip_right": {"angle": 0.0}
            }
        }
    }
    
    print("=== 模拟正常延迟 ===")
    for i in range(15):
        latency = random.uniform(10, 18)  # 正常延迟
        monitor.record_latency(latency)
        print(f"延迟: {latency:.1f}ms, 模式: {monitor.current_mode.value}")
    
    print("\n=== 模拟延迟超标 ===")
    for i in range(10):
        latency = random.uniform(25, 40)  # 超标延迟
        monitor.record_latency(latency)
        print(f"延迟: {latency:.1f}ms, 模式: {monitor.current_mode.value}")
    
    # 测试控制动作
    print("\n=== 测试PID控制 ===")
    action = monitor.get_control_action(sensor_data)
    print(f"控制动作: {json.dumps(action, indent=2)}")
    
    print("\n=== 模拟延迟恢复 ===")
    for i in range(25):
        latency = random.uniform(8, 15)  # 恢复正常
        monitor.record_latency(latency)
        print(f"延迟: {latency:.1f}ms, 模式: {monitor.current_mode.value}")
    
    # 统计
    print("\n=== 统计信息 ===")
    stats = monitor.get_stats()
    print(json.dumps(stats, indent=2))
