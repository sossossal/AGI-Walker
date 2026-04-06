import logging
import numpy as np
import json
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Dict, Any

logger = logging.getLogger(__name__)

from enum import IntEnum

class RiskLevel(IntEnum):
    """风险等级 (支持比较)"""
    SAFE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3
    CRITICAL = 4

@dataclass
class SafetyCheckResult:
    """安全检查结果"""
    safe: bool
    risk_level: RiskLevel
    reasons: List[str]
    modified_action: Optional[dict] = None

class SimplifiedPhysicsModel:
    """AGI-Walker 简化物理模型，用于快速轨迹预测"""
    def __init__(self, dt: float = 0.033):
        self.dt = dt
        self.gravity = 9.81
        self.mass = 12.0 # kg

    def simulate_step(self, state: dict, action: dict) -> dict:
        """模拟单步物理变化 (核心逻辑简化)"""
        orient = state.get("sensors", {}).get("imu", {}).get("orient", [0, 0, 0])
        height = state.get("torso_height", 1.0)
        
        # 简单动作模拟：动作幅度越大，倾斜风险越高
        motors = action.get("motors", {})
        total_effort = sum(abs(v) for v in motors.values())
        
        # 模拟下一时刻姿态
        new_roll = orient[0] + (total_effort * 0.01) * np.sin(time.time())
        new_height = height - (total_effort * 0.001)
        
        return {
            "sensors": {"imu": {"orient": [new_roll, orient[1], orient[2]]}},
            "torso_height": max(0.1, new_height)
        }

    def predict_trajectory(self, initial_state: dict, action: dict, horizon: int) -> List[dict]:
        trajectory = []
        current = initial_state
        for _ in range(horizon):
            current = self.simulate_step(current, action)
            trajectory.append(current)
        return trajectory

class PredictiveSafetyChecker:
    """
    AGI-Walker V2.1 MPC Safety Supervisor.
    Predicts consequences and filters actions.
    """
    THRESHOLDS = {
        "max_roll": 15.0,      # 收紧: 原为 30.0
        "max_pitch": 15.0,     # 收紧
        "min_height": 0.55,    # 提高安全高度
        "critical_roll": 30.0,
        "critical_pitch": 30.0
    }

    def __init__(self, physics_model: Optional[SimplifiedPhysicsModel] = None, prediction_horizon: int = 8):
        self.physics_model = physics_model or SimplifiedPhysicsModel()
        self.prediction_horizon = prediction_horizon
        self.checks_performed = 0
        self.actions_modified = 0
        self.interventions = []

    def check_and_filter(self, current_state: dict, proposed_action: dict) -> dict:
        self.checks_performed += 1
        result = self.check_action(current_state, proposed_action)
        
        if result.safe:
            return proposed_action
        
        self.actions_modified += 1
        self.interventions.append({
            "ts": current_state.get("timestamp", 0),
            "reasons": result.reasons
        })
        return result.modified_action

    def check_action(self, current_state: dict, action: dict) -> SafetyCheckResult:
        trajectory = self.physics_model.predict_trajectory(current_state, action, self.prediction_horizon)
        reasons = []
        max_risk = RiskLevel.SAFE

        for state in trajectory:
            orient = state.get("sensors", {}).get("imu", {}).get("orient", [0,0,0])
            height = state.get("torso_height", 1.0)
            
            if abs(orient[0]) > self.THRESHOLDS["critical_roll"]:
                reasons.append("Extreme roll predicted")
                max_risk = RiskLevel.CRITICAL
            elif abs(orient[0]) > self.THRESHOLDS["max_roll"]:
                reasons.append("High roll predicted")
                max_risk = max(max_risk, RiskLevel.HIGH)
                
            if height < self.THRESHOLDS["min_height"]:
                reasons.append("Fall predicted (too low)")
                max_risk = max(max_risk, RiskLevel.HIGH)

        modified = None
        if max_risk != RiskLevel.SAFE:
            modified = self._mitigate(action, max_risk)

        return SafetyCheckResult(
            safe=(max_risk == RiskLevel.SAFE),
            risk_level=max_risk,
            reasons=list(set(reasons)),
            modified_action=modified
        )

    def _mitigate(self, action: dict, risk: RiskLevel) -> dict:
        scale = 0.5 if risk == RiskLevel.HIGH else 0.1
        safe_action = {"motors": {}}
        for k, v in action.get("motors", {}).items():
            safe_action["motors"][k] = v * scale
        return safe_action

    def get_stats(self) -> dict:
        return {"checks": self.checks_performed, "modified": self.actions_modified}

import time # For simulation dummy logic
