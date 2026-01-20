"""
预测性安全检查模块
使用简化物理模型模拟未来状态，预防潜在碰撞和失稳
"""

import time
import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class RiskLevel(Enum):
    """风险等级"""
    SAFE = "safe"           # 安全
    LOW = "low"             # 低风险
    MEDIUM = "medium"       # 中风险
    HIGH = "high"           # 高风险
    CRITICAL = "critical"   # 危险


@dataclass
class SafetyCheckResult:
    """安全检查结果"""
    safe: bool
    risk_level: RiskLevel
    risk_score: float  # 0-1
    reasons: List[str]
    modified_action: Optional[dict]
    predicted_states: List[dict]


class SimplifiedPhysicsModel:
    """
    简化物理模型
    
    用于快速模拟机器人未来状态
    轻量级，可在毫秒级完成多步预测
    """
    
    def __init__(
        self,
        dt: float = 0.033,  # 时间步长
        gravity: float = 9.8,
        damping: float = 0.1
    ):
        self.dt = dt
        self.gravity = gravity
        self.damping = damping
        
        # 机器人参数
        self.torso_mass = 5.0
        self.leg_mass = 2.0
        self.torso_height = 0.4
        self.leg_length = 0.8
        
        # 关节限位
        self.joint_limits = {
            "hip_left": (-45, 90),
            "hip_right": (-45, 90)
        }
    
    def simulate_step(
        self,
        state: dict,
        action: dict
    ) -> dict:
        """
        模拟单步
        
        Args:
            state: 当前状态
            action: 控制动作
        
        Returns:
            下一状态
        """
        # 提取状态
        orient = state.get('sensors', {}).get('imu', {}).get('orient', [0, 0, 0])
        height = state.get('torso_height', 1.5)
        joints = state.get('sensors', {}).get('joints', {})
        
        roll, pitch, yaw = orient[0], orient[1], orient[2]
        hip_left = joints.get('hip_left', {}).get('angle', 0)
        hip_right = joints.get('hip_right', {}).get('angle', 0)
        
        # 获取动作
        motors = action.get('motors', {})
        target_left = motors.get('hip_left', hip_left)
        target_right = motors.get('hip_right', hip_right)
        
        # 模拟关节运动（一阶响应）
        motor_speed = 5.0  # 度/秒
        new_hip_left = hip_left + np.clip(target_left - hip_left, -motor_speed * self.dt, motor_speed * self.dt) * 30
        new_hip_right = hip_right + np.clip(target_right - hip_right, -motor_speed * self.dt, motor_speed * self.dt) * 30
        
        # 限位
        new_hip_left = np.clip(new_hip_left, *self.joint_limits["hip_left"])
        new_hip_right = np.clip(new_hip_right, *self.joint_limits["hip_right"])
        
        # 通力矩估算姿态变化
        # 简化模型：关节角度变化产生力矩，影响roll和pitch
        torque_left = (new_hip_left - hip_left) * 0.1
        torque_right = (new_hip_right - hip_right) * 0.1
        
        # 重力效应
        gravity_torque_roll = self.gravity * self.torso_mass * np.sin(np.radians(roll)) * 0.01
        gravity_torque_pitch = self.gravity * self.torso_mass * np.sin(np.radians(pitch)) * 0.01
        
        # 姿态更新
        new_roll = roll + torque_left - torque_right - gravity_torque_roll * self.dt * 10
        new_pitch = pitch + (torque_left + torque_right) / 2 - gravity_torque_pitch * self.dt * 10
        
        # 阻尼
        new_roll *= (1 - self.damping)
        new_pitch *= (1 - self.damping)
        
        # 高度变化（简化）
        # 如果关节弯曲过多，高度下降
        avg_bend = (abs(new_hip_left) + abs(new_hip_right)) / 2
        height_factor = 1 - avg_bend / 180 * 0.3  # 最多下降30%
        new_height = height * height_factor
        
        # 构建新状态
        new_state = {
            "sensors": {
                "imu": {
                    "orient": [new_roll, new_pitch, yaw],
                    "gyro": [(new_roll - roll) / self.dt, (new_pitch - pitch) / self.dt, 0]
                },
                "joints": {
                    "hip_left": {"angle": new_hip_left, "velocity": (new_hip_left - hip_left) / self.dt},
                    "hip_right": {"angle": new_hip_right, "velocity": (new_hip_right - hip_right) / self.dt}
                },
                "contacts": state.get('sensors', {}).get('contacts', {"foot_left": True, "foot_right": True})
            },
            "torso_height": new_height,
            "timestamp": state.get('timestamp', 0) + self.dt
        }
        
        return new_state
    
    def simulate_trajectory(
        self,
        initial_state: dict,
        actions: List[dict],
        num_steps: Optional[int] = None
    ) -> List[dict]:
        """
        模拟轨迹
        
        Args:
            initial_state: 初始状态
            actions: 动作序列（如果只有一个动作，将重复使用）
            num_steps: 步数
        
        Returns:
            状态轨迹
        """
        if num_steps is None:
            num_steps = len(actions)
        
        trajectory = [initial_state]
        current_state = initial_state
        
        for i in range(num_steps):
            action = actions[i] if i < len(actions) else actions[-1]
            next_state = self.simulate_step(current_state, action)
            trajectory.append(next_state)
            current_state = next_state
        
        return trajectory


class PredictiveSafetyChecker:
    """
    预测性安全检查器
    
    功能：
    1. 模拟动作的物理后果
    2. 检测潜在碰撞和失稳
    3. 必要时修正动作
    """
    
    # 安全阈值
    THRESHOLDS = {
        "max_roll": 35.0,        # 最大Roll角度
        "max_pitch": 35.0,       # 最大Pitch角度
        "min_height": 0.4,       # 最小高度
        "max_joint_velocity": 100.0,  # 最大关节速度
        "critical_roll": 45.0,   # 危险Roll角度
        "critical_pitch": 45.0,  # 危险Pitch角度
    }
    
    def __init__(
        self,
        physics_model: Optional[SimplifiedPhysicsModel] = None,
        prediction_horizon: int = 5
    ):
        self.physics_model = physics_model or SimplifiedPhysicsModel()
        self.prediction_horizon = prediction_horizon
        
        # 统计
        self.checks_performed = 0
        self.actions_modified = 0
        self.actions_rejected = 0
    
    def check_action(
        self,
        current_state: dict,
        proposed_action: dict
    ) -> SafetyCheckResult:
        """
        检查动作是否安全
        
        Args:
            current_state: 当前状态
            proposed_action: 提议的动作
        
        Returns:
            安全检查结果
        """
        self.checks_performed += 1
        
        # 模拟未来状态
        actions = [proposed_action] * self.prediction_horizon
        future_states = self.physics_model.simulate_trajectory(
            current_state,
            actions,
            self.prediction_horizon
        )
        
        # 检查各项指标
        reasons = []
        risk_score = 0.0
        
        for i, state in enumerate(future_states[1:], 1):
            orient = state['sensors']['imu']['orient']
            height = state['torso_height']
            roll, pitch = abs(orient[0]), abs(orient[1])
            
            # Roll检查
            if roll > self.THRESHOLDS["critical_roll"]:
                reasons.append(f"步骤{i}: Roll角度超过危险阈值 ({roll:.1f}°)")
                risk_score = max(risk_score, 1.0)
            elif roll > self.THRESHOLDS["max_roll"]:
                reasons.append(f"步骤{i}: Roll角度过大 ({roll:.1f}°)")
                risk_score = max(risk_score, 0.6)
            
            # Pitch检查
            if pitch > self.THRESHOLDS["critical_pitch"]:
                reasons.append(f"步骤{i}: Pitch角度超过危险阈值 ({pitch:.1f}°)")
                risk_score = max(risk_score, 1.0)
            elif pitch > self.THRESHOLDS["max_pitch"]:
                reasons.append(f"步骤{i}: Pitch角度过大 ({pitch:.1f}°)")
                risk_score = max(risk_score, 0.6)
            
            # 高度检查
            if height < self.THRESHOLDS["min_height"]:
                reasons.append(f"步骤{i}: 高度过低 ({height:.2f}m)")
                risk_score = max(risk_score, 0.8)
        
        # 确定风险等级
        if risk_score >= 0.9:
            risk_level = RiskLevel.CRITICAL
        elif risk_score >= 0.7:
            risk_level = RiskLevel.HIGH
        elif risk_score >= 0.4:
            risk_level = RiskLevel.MEDIUM
        elif risk_score > 0:
            risk_level = RiskLevel.LOW
        else:
            risk_level = RiskLevel.SAFE
        
        # 决定是否修正动作
        modified_action = None
        safe = risk_level in (RiskLevel.SAFE, RiskLevel.LOW)
        
        if not safe:
            modified_action = self._modify_action(
                current_state,
                proposed_action,
                risk_level
            )
            self.actions_modified += 1
        
        return SafetyCheckResult(
            safe=safe,
            risk_level=risk_level,
            risk_score=risk_score,
            reasons=reasons[:5],  # 最多5条原因
            modified_action=modified_action,
            predicted_states=future_states
        )
    
    def _modify_action(
        self,
        state: dict,
        action: dict,
        risk_level: RiskLevel
    ) -> dict:
        """修正动作"""
        motors = action.get('motors', {}).copy()
        
        # 根据风险等级调整
        if risk_level == RiskLevel.CRITICAL:
            # 危险：返回安全姿态
            return {
                "motors": {"hip_left": 0.0, "hip_right": 0.0},
                "confidence": 0.0,
                "_safety_modified": True,
                "_original_action": action
            }
        
        elif risk_level == RiskLevel.HIGH:
            # 高风险：大幅缩减动作幅度
            scale = 0.3
        else:
            # 中风险：缩减动作幅度
            scale = 0.6
        
        # 获取当前关节角度
        joints = state.get('sensors', {}).get('joints', {})
        current_left = joints.get('hip_left', {}).get('angle', 0)
        current_right = joints.get('hip_right', {}).get('angle', 0)
        
        # 缩减变化幅度
        target_left = motors.get('hip_left', current_left)
        target_right = motors.get('hip_right', current_right)
        
        modified_left = current_left + (target_left - current_left) * scale
        modified_right = current_right + (target_right - current_right) * scale
        
        return {
            "motors": {
                "hip_left": modified_left,
                "hip_right": modified_right
            },
            "confidence": action.get('confidence', 0.5) * scale,
            "_safety_modified": True,
            "_scale_applied": scale,
            "_original_action": action
        }
    
    def check_immediate_safety(self, state: dict) -> bool:
        """快速检查当前状态是否安全"""
        orient = state.get('sensors', {}).get('imu', {}).get('orient', [0, 0, 0])
        height = state.get('torso_height', 1.5)
        
        roll, pitch = abs(orient[0]), abs(orient[1])
        
        return (
            roll < self.THRESHOLDS["critical_roll"] and
            pitch < self.THRESHOLDS["critical_pitch"] and
            height > self.THRESHOLDS["min_height"]
        )
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "checks_performed": self.checks_performed,
            "actions_modified": self.actions_modified,
            "actions_rejected": self.actions_rejected,
            "modification_rate": (
                self.actions_modified / self.checks_performed
                if self.checks_performed > 0 else 0
            )
        }


# 测试代码
if __name__ == "__main__":
    import json
    
    print("预测性安全检查测试\n")
    
    # 创建检查器
    checker = PredictiveSafetyChecker(prediction_horizon=5)
    
    # 正常状态和动作
    normal_state = {
        "sensors": {
            "imu": {"orient": [2.0, -1.0, 0.0], "gyro": [0, 0, 0]},
            "joints": {
                "hip_left": {"angle": 5.0, "velocity": 0.0},
                "hip_right": {"angle": -3.0, "velocity": 0.0}
            },
            "contacts": {"foot_left": True, "foot_right": True}
        },
        "torso_height": 1.45
    }
    
    safe_action = {
        "motors": {"hip_left": 8.0, "hip_right": -5.0}
    }
    
    print("=== 测试安全动作 ===")
    result = checker.check_action(normal_state, safe_action)
    print(f"安全: {result.safe}")
    print(f"风险等级: {result.risk_level.value}")
    print(f"风险分数: {result.risk_score:.2f}")
    
    # 危险动作
    dangerous_action = {
        "motors": {"hip_left": 80.0, "hip_right": -75.0}
    }
    
    print("\n=== 测试危险动作 ===")
    result = checker.check_action(normal_state, dangerous_action)
    print(f"安全: {result.safe}")
    print(f"风险等级: {result.risk_level.value}")
    print(f"风险分数: {result.risk_score:.2f}")
    print(f"原因: {result.reasons}")
    
    if result.modified_action:
        print(f"修正后动作: {result.modified_action['motors']}")
    
    # 不稳定状态
    unstable_state = {
        "sensors": {
            "imu": {"orient": [25.0, -20.0, 0.0], "gyro": [2, -1, 0]},
            "joints": {
                "hip_left": {"angle": 30.0, "velocity": 5.0},
                "hip_right": {"angle": -25.0, "velocity": -3.0}
            },
            "contacts": {"foot_left": True, "foot_right": False}
        },
        "torso_height": 1.2
    }
    
    print("\n=== 测试不稳定状态 ===")
    result = checker.check_action(unstable_state, safe_action)
    print(f"安全: {result.safe}")
    print(f"风险等级: {result.risk_level.value}")
    
    # 统计
    print("\n=== 统计信息 ===")
    print(json.dumps(checker.get_stats(), indent=2))
