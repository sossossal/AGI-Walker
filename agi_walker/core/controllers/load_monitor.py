import psutil
import time
import logging
import platform
from typing import Dict, Any, Optional

logger = logging.getLogger(__name__)

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
        """
        获取实时硬件统计数据。
        """
        stats = {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "memory_percent": psutil.virtual_memory().percent,
            "load_avg": self._get_load_avg(),
            "temperature": self._get_temperature(),
            "timestamp": time.time()
        }
        return stats

    def _get_load_avg(self):
        """跨平台获取负载均值"""
        if hasattr(os, 'getloadavg'):
            return os.getloadavg()
        return [0.0, 0.0, 0.0]

    def _get_temperature(self) -> Optional[float]:
        """
        获取 CPU 温度。
        在 Linux/Jetson 上通过 psutil.sensors_temperatures() 获取。
        """
        if self.os_type != "Linux":
            return None # Windows/macOS 下通常需要特殊驱动，这里返回 None
        
        try:
            temps = psutil.sensors_temperatures()
            if not temps:
                return None
            
            # 寻找主要的 CPU 温度传感器 (常见名称: coretemp, cpu-thermal)
            for name in ['coretemp', 'cpu-thermal', 'soc_thermal']:
                if name in temps:
                    return temps[name][0].current
            return None
        except Exception:
            return None

    def is_overloaded(self) -> bool:
        """
        判定系统是否过载。
        """
        stats = self.get_hw_stats()
        
        # 1. 检查 CPU
        if stats["cpu_percent"] > self.cpu_threshold:
            logger.warning(f"CPU Overload detected: {stats['cpu_percent']}%")
            return True
        
        # 2. 检查温度 (如果可用)
        if stats["temperature"] and stats["temperature"] > self.temp_threshold:
            logger.warning(f"High Temperature detected: {stats['temperature']}C")
            return True
            
        return False

    def report(self):
        """打印详细硬件报告"""
        stats = self.get_hw_stats()
        temp_str = f"{stats['temperature']:.1f}C" if stats['temperature'] else "N/A"
        logger.info(
            f"Hardware: CPU={stats['cpu_percent']}% | "
            f"MEM={stats['memory_percent']}% | "
            f"TEMP={temp_str}"
        )
