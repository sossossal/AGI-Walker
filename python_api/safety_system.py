"""
安全系统
Safety System

功能:
- 紧急停止
- 碰撞检测
- 力限制检查
- 安全距离监控
- 故障安全模式
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from enum import Enum


class SafetyLevel(Enum):
    """安全级别"""
    SAFE = 0          # 安全
    WARNING = 1       # 警告
    DANGER = 2        # 危险
    EMERGENCY = 3     # 紧急


class SafetyViolation:
    """安全违规记录"""
    
    def __init__(self, level: SafetyLevel, message: str, timestamp: float):
        self.level = level
        self.message = message
        self.timestamp = timestamp
        self.acknowledged = False


class SafetySystem:
    """安全系统"""
    
    def __init__(self, robot_params: Dict):
        self.robot_params = robot_params
        
        # 安全限制
        self.max_velocity = 2.0  # m/s
        self.max_acceleration = 5.0  # m/s²
        self.max_joint_torque = 100.0  # Nm
        self.max_force = 500.0  # N
        self.safe_distance = 0.3  # m (与障碍物)
        self.max_tilt_angle = 30.0  # degrees
        
        # 状态
        self.emergency_stop_active = False
        self.safety_violations = []
        self.last_check_time = 0
        
        # 统计
        self.total_checks = 0
        self.violations_count = {level: 0 for level in SafetyLevel}
    
    def emergency_stop(self, reason: str = "手动触发"):
        """紧急停止"""
        self.emergency_stop_active = True
        violation = SafetyViolation(
            SafetyLevel.EMERGENCY,
            f"紧急停止: {reason}",
            self.last_check_time
        )
        self.safety_violations.append(violation)
        self.violations_count[SafetyLevel.EMERGENCY] += 1
        
        return {
            'stopped': True,
            'reason': reason,
            'safe_to_resume': False
        }
    
    def check_velocity(self, velocity: float) -> Optional[SafetyViolation]:
        """检查速度限制"""
        if abs(velocity) > self.max_velocity:
            return SafetyViolation(
                SafetyLevel.DANGER,
                f"速度超限: {velocity:.2f} m/s (限制: {self.max_velocity:.2f} m/s)",
                self.last_check_time
            )
        elif abs(velocity) > self.max_velocity * 0.9:
            return SafetyViolation(
                SafetyLevel.WARNING,
                f"速度接近限制: {velocity:.2f} m/s",
                self.last_check_time
            )
        return None
    
    def check_acceleration(self, acceleration: float) -> Optional[SafetyViolation]:
        """检查加速度限制"""
        if abs(acceleration) > self.max_acceleration:
            return SafetyViolation(
                SafetyLevel.DANGER,
                f"加速度超限: {acceleration:.2f} m/s² (限制: {self.max_acceleration:.2f} m/s²)",
                self.last_check_time
            )
        return None
    
    def check_joint_torque(self, torques: List[float]) -> Optional[SafetyViolation]:
        """检查关节扭矩"""
        max_torque = max(abs(t) for t in torques)
        
        if max_torque > self.max_joint_torque:
            return SafetyViolation(
                SafetyLevel.DANGER,
                f"关节扭矩超限: {max_torque:.2f} Nm (限制: {self.max_joint_torque:.2f} Nm)",
                self.last_check_time
            )
        elif max_torque > self.max_joint_torque * 0.9:
            return SafetyViolation(
                SafetyLevel.WARNING,
                f"关节扭矩接近限制: {max_torque:.2f} Nm",
                self.last_check_time
            )
        return None
    
    def check_collision(self, distances: List[float]) -> Optional[SafetyViolation]:
        """碰撞检测"""
        min_distance = min(distances) if distances else float('inf')
        
        if min_distance < self.safe_distance * 0.5:
            # 极度危险
            return SafetyViolation(
                SafetyLevel.EMERGENCY,
                f"即将碰撞! 距离: {min_distance:.3f} m",
                self.last_check_time
            )
        elif min_distance < self.safe_distance:
            return SafetyViolation(
                SafetyLevel.WARNING,
                f"接近障碍物: {min_distance:.3f} m",
                self.last_check_time
            )
        return None
    
    def check_balance(self, tilt_angle: float) -> Optional[SafetyViolation]:
        """检查平衡"""
        if abs(tilt_angle) > self.max_tilt_angle:
            return SafetyViolation(
                SafetyLevel.EMERGENCY,
                f"倾斜角过大: {tilt_angle:.1f}° (限制: {self.max_tilt_angle:.1f}°)",
                self.last_check_time
            )
        elif abs(tilt_angle) > self.max_tilt_angle * 0.8:
            return SafetyViolation(
                SafetyLevel.DANGER,
                f"倾斜角接近限制: {tilt_angle:.1f}°",
                self.last_check_time
            )
        return None
    
    def check_force(self, forces: List[float]) -> Optional[SafetyViolation]:
        """检查力限制"""
        max_force = max(abs(f) for f in forces) if forces else 0
        
        if max_force > self.max_force:
            return SafetyViolation(
                SafetyLevel.DANGER,
                f"力超限: {max_force:.1f} N (限制: {self.max_force:.1f} N)",
                self.last_check_time
            )
        return None
    
    def comprehensive_safety_check(self, state: Dict) -> Dict:
        """
        综合安全检查
        
        参数:
            state: 包含速度、加速度、扭矩、距离等的状态字典
        
        返回:
            安全检查结果
        """
        self.total_checks += 1
        self.last_check_time = state.get('time', 0)
        
        # 如果已经紧急停止，直接返回
        if self.emergency_stop_active:
            return {
                'safe': False,
                'level': SafetyLevel.EMERGENCY,
                'action': 'STOP',
                'message': '紧急停止已激活'
            }
        
        violations = []
        
        # 检查各项指标
        if 'velocity' in state:
            v = self.check_velocity(state['velocity'])
            if v: violations.append(v)
        
        if 'acceleration' in state:
            a = self.check_acceleration(state['acceleration'])
            if a: violations.append(a)
        
        if 'joint_torques' in state:
            t = self.check_joint_torque(state['joint_torques'])
            if t: violations.append(t)
        
        if 'distances' in state:
            c = self.check_collision(state['distances'])
            if c: violations.append(c)
        
        if 'tilt_angle' in state:
            b = self.check_balance(state['tilt_angle'])
            if b: violations.append(b)
        
        if 'forces' in state:
            f = self.check_force(state['forces'])
            if f: violations.append(f)
        
        # 记录违规
        for violation in violations:
            self.safety_violations.append(violation)
            self.violations_count[violation.level] += 1
        
        # 确定最高安全级别
        if violations:
            max_level = max(v.level for v in violations)
            
            # 根据级别决定动作
            if max_level == SafetyLevel.EMERGENCY:
                action = 'EMERGENCY_STOP'
                self.emergency_stop("检测到紧急情况")
            elif max_level == SafetyLevel.DANGER:
                action = 'REDUCE_SPEED'
            elif max_level == SafetyLevel.WARNING:
                action = 'MONITOR'
            else:
                action = 'CONTINUE'
            
            return {
                'safe': max_level.value <= SafetyLevel.WARNING.value,
                'level': max_level,
                'action': action,
                'violations': [v.message for v in violations],
                'message': violations[0].message
            }
        
        # 一切正常
        return {
            'safe': True,
            'level': SafetyLevel.SAFE,
            'action': 'CONTINUE',
            'violations': [],
            'message': '系统安全'
        }
    
    def reset_emergency_stop(self, force: bool = False) -> bool:
        """
        重置紧急停止
        
        参数:
            force: 强制重置（即使不安全）
        
        返回:
            是否成功重置
        """
        if not force:
            # 检查是否安全
            recent_violations = [
                v for v in self.safety_violations[-10:]
                if v.level in [SafetyLevel.DANGER, SafetyLevel.EMERGENCY]
                and not v.acknowledged
            ]
            
            if recent_violations:
                return False
        
        self.emergency_stop_active = False
        return True
    
    def get_safety_report(self) -> str:
        """生成安全报告"""
        report = []
        report.append("="*70)
        report.append("安全系统报告")
        report.append("="*70)
        
        report.append(f"\n系统状态:")
        status = "紧急停止" if self.emergency_stop_active else "正常运行"
        report.append(f"  状态: {status}")
        report.append(f"  总检查次数: {self.total_checks}")
        
        report.append(f"\n安全限制:")
        report.append(f"  最大速度: {self.max_velocity} m/s")
        report.append(f"  最大加速度: {self.max_acceleration} m/s²")
        report.append(f"  最大关节扭矩: {self.max_joint_torque} Nm")
        report.append(f"  最大力: {self.max_force} N")
        report.append(f"  安全距离: {self.safe_distance} m")
        report.append(f"  最大倾斜角: {self.max_tilt_angle}°")
        
        report.append(f"\n违规统计:")
        for level, count in self.violations_count.items():
            if count > 0:
                report.append(f"  {level.name}: {count} 次")
        
        if self.safety_violations:
            report.append(f"\n最近违规 (最多10条):")
            for violation in self.safety_violations[-10:]:
                report.append(f"  [{violation.level.name}] {violation.message}")
        
        return "\n".join(report)


if __name__ == "__main__":
    print("安全系统加载完成")
    
    # 示例
    safety = SafetySystem({})
    
    # 模拟检查
    state = {
        'velocity': 1.5,
        'acceleration': 3.0,
        'joint_torques': [50, 60, 45, 55, 48, 52],
        'distances': [0.5, 0.8, 1.2],
        'tilt_angle': 15.0,
        'time': 1.0
    }
    
    result = safety.comprehensive_safety_check(state)
    print(f"检查结果: {result}")
    
    print("\n" + safety.get_safety_report())
