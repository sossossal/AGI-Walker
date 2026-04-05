import psutil
import time
import logging
import platform
import os
from typing import Dict, Any, Optional
from enum import Enum

logger = logging.getLogger(__name__)

class ControlMode(Enum):
    """控制模式"""
    OFF = "off"
    PID = "pid"
    AI = "ai"
    SAFETY = "safety"

class SimplePIDController:
    """简单的PID控制器用于安全回退"""
    def __init__(self, kp=1.0, ki=0.0, kd=0.1):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.last_error = 0
        self.integral = 0

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class SystemMonitor:
    """
    AGI-Walker V2.0 Hardware Load Monitor.
    Tracks CPU, Memory, Temperature and Power on edge devices.
    """
    def __init__(self, cpu_threshold: float = 90.0, temp_threshold: float = 80.0):
        self.cpu_threshold = cpu_threshold
        self.temp_threshold = temp_threshold
        self.os_type = platform.system()

    def get_hw_stats(self) -> Dict[str, Any]:
        stats = {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "memory_percent": psutil.virtual_memory().percent,
            "load_avg": self._get_load_avg(),
            "temperature": self._get_temperature(),
            "timestamp": time.time()
        }
        return stats

    def _get_load_avg(self):
        if hasattr(os, 'getloadavg'):
            return os.getloadavg()
        return [0.0, 0.0, 0.0]

    def _get_temperature(self) -> Optional[float]:
        if self.os_type != "Linux":
            return None
        try:
            temps = psutil.sensors_temperatures()
            for name in ['coretemp', 'cpu-thermal', 'soc_thermal']:
                if name in temps:
                    return temps[name][0].current
            return None
        except Exception:
            return None

    def is_overloaded(self) -> bool:
        stats = self.get_hw_stats()
        if stats["cpu_percent"] > self.cpu_threshold:
            return True
        if stats["temperature"] and stats["temperature"] > self.temp_threshold:
            return True
        return False

    def report(self):
        stats = self.get_hw_stats()
        temp_str = f"{stats['temperature']:.1f}C" if stats['temperature'] else "N/A"
        logger.info(f"Hardware: CPU={stats['cpu_percent']}% | MEM={stats['memory_percent']}% | TEMP={temp_str}")

class LoadMonitor:
    """AGI-Walker V2.0 负载监控器 (集成系统感知与控制模式切换)"""
    def __init__(self, pid_controller: Optional[SimplePIDController] = None):
        self.system_monitor = SystemMonitor()
        self.pid = pid_controller or SimplePIDController()
        self.on_mode_change = None

    def record_latency(self, ms: float):
        if ms > 100 and self.on_mode_change:
            logger.warning(f"Extreme Inference Latency: {ms:.1f}ms - Triggering Safety Fallback.")
            self.on_mode_change(ControlMode.SAFETY)

    def check_system(self):
        if self.system_monitor.is_overloaded() and self.on_mode_change:
            self.on_mode_change(ControlMode.SAFETY)
        self.system_monitor.report()
