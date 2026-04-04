import random
import logging
import numpy as np
from typing import Dict, Any, List, Tuple

logger = logging.getLogger(__name__)

class RandomizationManager:
    """
    AGI-Walker V2.0 Auto-Domain Randomization (ADR).
    Dynamically adjusts simulation randomization ranges based on performance.
    """
    def __init__(self, initial_ranges: Dict[str, Tuple[float, float]]):
        # 初始随机化范围: {"mass": (0.8, 1.2), "friction": (0.5, 1.5)}
        self.ranges = initial_ranges
        self.success_history: List[bool] = []
        self.window_size = 50 # 评估窗口
        self.expansion_step = 0.05 # 扩充比例

    def get_randomized_config(self) -> Dict[str, float]:
        """
        生成一个随机化的环境配置。
        """
        config = {}
        for param, (low, high) in self.ranges.items():
            config[param] = random.uniform(low, high)
        return config

    def record_success(self, success: bool):
        """
        记录一次任务的成败。
        """
        self.success_history.append(success)
        if len(self.success_history) > self.window_size:
            self.success_history.pop(0)
            self._adjust_ranges()

    def _adjust_ranges(self):
        """
        自动调整随机化范围。
        成功率高则扩大范围 (增加难度)，成功率低则缩小范围。
        """
        success_rate = sum(self.success_history) / self.window_size
        
        if success_rate > 0.90:
            # 表现优异，扩大范围
            for param in self.ranges:
                low, high = self.ranges[param]
                center = (low + high) / 2
                half_width = (high - low) / 2
                half_width *= (1 + self.expansion_step)
                self.ranges[param] = (center - half_width, center + half_width)
            logger.info(f"ADR: Success rate {success_rate:.1f} - Expanding ranges.")
            
        elif success_rate < 0.70:
            # 表现欠佳，缩小范围
            for param in self.ranges:
                low, high = self.ranges[param]
                center = (low + high) / 2
                half_width = (high - low) / 2
                half_width *= (1 - self.expansion_step)
                self.ranges[param] = (center - half_width, center + half_width)
            logger.warning(f"ADR: Success rate {success_rate:.1f} - Shrinking ranges.")

    def get_current_ranges(self) -> Dict[str, Tuple[float, float]]:
        return self.ranges
