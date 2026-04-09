import numpy as np
import logging
from typing import Dict, List, Any
from enum import Enum

logger = logging.getLogger(__name__)


class TerrainType(Enum):
    HARD = "hard"  # 硬地 (混凝土/瓷砖)
    SOFT = "soft"  # 软地 (草地/地毯)
    ROUGH = "rough"  # 粗糙/碎石 (碎石/凹凸不平)
    SLIPPERY = "slippery"  # 湿滑 (冰面/油)
    UNKNOWN = "unknown"


class TactileProcessor:
    """
    AGI-Walker V2.5 Tactile Fusion Plugin.
    Analyzes high-frequency foot pressure data to identify terrain and slips.
    """

    def __init__(self):
        # 历史缓冲区，用于滑动窗口分析
        self.pressure_history = []
        self.buffer_size = 20

    def process_tactile(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Main entry point for sensor fusion link.
        Expects sensor_data to contain 'foot_pressure' (array) or 'contacts'.
        """
        # 1. 提取原始压力数据
        # 假设 Godot 发送的是 [left_front, left_back, right_front, right_back]
        raw_pressure = (
            sensor_data.get("sensors", {})
            .get("contacts", {})
            .get("pressure_array", [0.0] * 4)
        )

        # 2. 更新历史
        self.pressure_history.append(raw_pressure)
        if len(self.pressure_history) > self.buffer_size:
            self.pressure_history.pop(0)

        # 3. 运行地形分类逻辑
        terrain = self._classify_terrain(raw_pressure)

        # 4. 打滑检测 (Slip Detection)
        is_slipping = self._detect_slip(raw_pressure)

        return {
            "terrain_type": terrain.value,
            "slip_warning": is_slipping,
            "contact_stability": self._calculate_stability(),
            "confidence": 0.85,
        }

    def _classify_terrain(self, pressure: List[float]) -> TerrainType:
        """简单的基于规则的地形分类"""
        p_avg = np.mean(pressure)
        p_std = np.std(pressure)

        if p_avg < 0.1:
            return TerrainType.UNKNOWN  # 悬空

        # 硬地通常压力分布集中且标准差小
        if p_std < 0.05:
            return TerrainType.HARD
        # 软地压力分布较均匀但数值适中
        elif 0.05 <= p_std < 0.15:
            return TerrainType.SOFT
        # 粗糙地面压力抖动非常剧烈
        else:
            return TerrainType.ROUGH

    def _detect_slip(self, pressure: List[float]) -> bool:
        """基于压力骤降的简单打滑检测"""
        if len(self.pressure_history) < 5:
            return False

        last_avg = np.mean(self.pressure_history[-2])
        curr_avg = np.mean(pressure)

        # 如果压力在极短时间内骤降（非跳跃状态），判定为打滑
        if last_avg > 0.5 and curr_avg < 0.2:
            return True
        return False

    def _calculate_stability(self) -> float:
        """计算足端稳定性指标 (0-1)"""
        if not self.pressure_history:
            return 1.0
        # 方差越小，稳定性越高
        variances = np.var(self.pressure_history, axis=0)
        stability = 1.0 - min(1.0, np.mean(variances) * 10)
        return float(stability)
