"""
故障诊断系统
Fault Diagnosis System

功能:
- 零件磨损模拟
- 故障概率计算
- 寿命预测
- 维护建议
- 故障检测
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from enum import Enum
from dataclasses import dataclass


class ComponentHealth(Enum):
    """组件健康状态"""
    EXCELLENT = "优秀"
    GOOD = "良好"
    FAIR = "一般"
    POOR = "较差"
    CRITICAL = "危急"
    FAILED = "失效"


@dataclass
class WearProfile:
    """磨损特性"""
    normal_wear_rate: float = 0.001  # 正常磨损率 (/小时)
    stress_multiplier: float = 1.5   # 应力倍增因子
    temperature_factor: float = 0.1  # 温度影响因子
    expected_lifetime_hours: float = 10000  # 预期寿命


class Component:
    """可诊断组件"""
    
    def __init__(self, name: str, component_type: str, wear_profile: WearProfile):
        self.name = name
        self.component_type = component_type
        self.wear_profile = wear_profile
        
        # 状态
        self.wear_level = 0.0  # 磨损程度 (0-1)
        self.operating_hours = 0.0
        self.failure_count = 0
        self.last_maintenance_hours = 0.0
        
        # 运行条件历史
        self.stress_history = []
        self.temp_history = []
        
        # 故障历史
        self.failures = []
    
    def update_wear(self, operating_hours: float, stress_level: float = 1.0, 
                    temperature: float = 25.0):
        """
        更新磨损状态
        
        参数:
            operating_hours: 新增运行小时
            stress_level: 应力水平 (1.0=正常)
            temperature: 温度 (°C)
        """
        # 计算磨损增量
        base_wear = self.wear_profile.normal_wear_rate * operating_hours
        
        # 应力影响
        if stress_level > 1.0:
            stress_factor = 1 + (stress_level - 1) * self.wear_profile.stress_multiplier
        else:
            stress_factor = 1.0
        
        # 温度影响
        temp_excess = max(0, temperature - 50)  # 超过50°C开始加速老化
        temp_factor = 1 + temp_excess * self.wear_profile.temperature_factor
        
        # 总磨损
        wear_increment = base_wear * stress_factor * temp_factor
        self.wear_level = min(1.0, self.wear_level + wear_increment)
        
        # 更新运行时间
        self.operating_hours += operating_hours
        
        # 记录历史
        self.stress_history.append(stress_level)
        self.temp_history.append(temperature)
        
        # 检查是否失效
        if self.wear_level >= 1.0 and self.failure_count == 0:
            self.record_failure("磨损失效")
    
    def record_failure(self, reason: str):
        """记录故障"""
        self.failures.append({
            'hours': self.operating_hours,
            'wear_level': self.wear_level,
            'reason': reason
        })
        self.failure_count += 1
    
    def perform_maintenance(self):
        """执行维护"""
        # 维护可以降低磨损
        self.wear_level = max(0, self.wear_level - 0.3)
        self.last_maintenance_hours = self.operating_hours
    
    def replace(self):
        """更换组件"""
        self.wear_level = 0.0
        self.last_maintenance_hours = self.operating_hours
    
    def get_health_status(self) -> ComponentHealth:
        """获取健康状态"""
        if self.wear_level >= 1.0:
            return ComponentHealth.FAILED
        elif self.wear_level >= 0.9:
            return ComponentHealth.CRITICAL
        elif self.wear_level >= 0.7:
            return ComponentHealth.POOR
        elif self.wear_level >= 0.5:
            return ComponentHealth.FAIR
        elif self.wear_level >= 0.3:
            return ComponentHealth.GOOD
        else:
            return ComponentHealth.EXCELLENT
    
    def estimate_remaining_life(self) -> float:
        """
        估算剩余寿命
        
        返回:
            剩余寿命 (小时)
        """
        if self.wear_level >= 1.0:
            return 0.0
        
        # 基于当前磨损率预测
        if len(self.stress_history) > 10:
            recent_stress = np.mean(self.stress_history[-10:])
            recent_temp = np.mean(self.temp_history[-10:])
            
            # 估算当前磨损率
            current_wear_rate = self.wear_profile.normal_wear_rate
            if recent_stress > 1.0:
                current_wear_rate *= 1 + (recent_stress - 1) * self.wear_profile.stress_multiplier
            
            temp_excess = max(0, recent_temp - 50)
            current_wear_rate *= 1 + temp_excess * self.wear_profile.temperature_factor
            
            remaining_wear = 1.0 - self.wear_level
            return remaining_wear / current_wear_rate if current_wear_rate > 0 else float('inf')
        
        # 默认估算
        return self.wear_profile.expected_lifetime_hours - self.operating_hours


class FaultDiagnostics:
    """故障诊断系统"""
    
    def __init__(self):
        self.components = []
        
        # 诊断规则
        self.diagnostic_rules = {
            'high_wear_rate': self._check_high_wear_rate,
            'abnormal_stress': self._check_abnormal_stress,
            'overheating': self._check_overheating,
            'maintenance_due': self._check_maintenance_due
        }
        
        # 诊断结果
        self.diagnoses = []
    
    def add_component(self, component: Component):
        """添加组件"""
        self.components.append(component)
    
    def run_diagnostics(self) -> List[Dict]:
        """
        运行诊断
        
        返回:
            诊断结果列表
        """
        self.diagnoses = []
        
        for component in self.components:
            for rule_name, rule_func in self.diagnostic_rules.items():
                result = rule_func(component)
                if result:
                    self.diagnoses.append({
                        'component': component.name,
                        'rule': rule_name,
                        'severity': result['severity'],
                        'message': result['message'],
                        'recommendation': result['recommendation']
                    })
        
        return self.diagnoses
    
    def _check_high_wear_rate(self, component: Component) -> Optional[Dict]:
        """检查高磨损率"""
        if component.operating_hours > 10 and component.wear_level / component.operating_hours > 0.0005:
            return {
                'severity': 'WARNING',
                'message': f'{component.name} 磨损率高于预期',
                'recommendation': '检查运行条件，降低应力'
            }
        return None
    
    def _check_abnormal_stress(self, component: Component) -> Optional[Dict]:
        """检查异常应力"""
        if len(component.stress_history) > 10:
            recent_stress = np.mean(component.stress_history[-10:])
            if recent_stress > 1.5:
                return {
                    'severity': 'WARNING',
                    'message': f'{component.name} 承受高应力 ({recent_stress:.1f}x)',
                    'recommendation': '降低负载或增强组件'
                }
        return None
    
    def _check_overheating(self, component: Component) -> Optional[Dict]:
        """检查过热"""
        if len(component.temp_history) > 10:
            recent_temp = np.mean(component.temp_history[-10:])
            if recent_temp > 80:
                return {
                    'severity': 'CRITICAL',
                    'message': f'{component.name} 温度过高 ({recent_temp:.1f}°C)',
                    'recommendation': '改善散热，立即降低负载'
                }
            elif recent_temp > 70:
                return {
                    'severity': 'WARNING',
                    'message': f'{component.name} 温度偏高 ({recent_temp:.1f}°C)',
                    'recommendation': '增加散热措施'
                }
        return None
    
    def _check_maintenance_due(self, component: Component) -> Optional[Dict]:
        """检查维护到期"""
        hours_since_maintenance = component.operating_hours - component.last_maintenance_hours
        
        if component.wear_level > 0.7:
            return {
                'severity': 'CRITICAL',
                'message': f'{component.name} 需要立即维护 (磨损 {component.wear_level*100:.0f}%)',
                'recommendation': '安排紧急维护或更换'
            }
        elif hours_since_maintenance > 1000:
            return {
                'severity': 'INFO',
                'message': f'{component.name} 建议进行定期维护 ({hours_since_maintenance:.0f}小时)',
                'recommendation': '安排维护检查'
            }
        return None
    
    def get_maintenance_plan(self) -> List[Dict]:
        """生成维护计划"""
        plan = []
        
        for component in self.components:
            health = component.get_health_status()
            remaining_life = component.estimate_remaining_life()
            
            if health in [ComponentHealth.CRITICAL, ComponentHealth.FAILED]:
                urgency = 'URGENT'
                action = '立即更换'
            elif health == ComponentHealth.POOR:
                urgency = 'HIGH'
                action = '计划维护'
            elif health == ComponentHealth.FAIR:
                urgency = 'MEDIUM'
                action = '监控'
            else:
                urgency = 'LOW'
                action = '正常运行'
            
            plan.append({
                'component': component.name,
                'health': health.value,
                'wear': f'{component.wear_level*100:.0f}%',
                'remaining_life_hours': remaining_life,
                'urgency': urgency,
                'action': action
            })
        
        # 按紧急程度排序
        urgency_order = {'URGENT': 0, 'HIGH': 1, 'MEDIUM': 2, 'LOW': 3}
        plan.sort(key=lambda x: urgency_order[x['urgency']])
        
        return plan
    
    def get_diagnostics_report(self) -> str:
        """生成诊断报告"""
        report = []
        report.append("="*70)
        report.append("故障诊断报告")
        report.append("="*70)
        
        # 组件状态
        report.append(f"\n组件状态 (共{len(self.components)}个):")
        report.append("-"*70)
        report.append(f"{'组件':<20} {'运行时间':<12} {'磨损':<10} {'健康':<10} {'剩余寿命':<15}")
        report.append("-"*70)
        
        for component in self.components:
            health = component.get_health_status()
            remaining = component.estimate_remaining_life()
            
            remaining_str = f"{remaining:.0f}h" if remaining < float('inf') else "长期"
            
            report.append(
                f"{component.name:<20} "
                f"{component.operating_hours:<12.1f}h "
                f"{component.wear_level*100:<10.0f}% "
                f"{health.value:<10} "
                f"{remaining_str:<15}"
            )
        
        # 诊断问题
        if self.diagnoses:
            report.append(f"\n发现问题 (共{len(self.diagnoses)}项):")
            for diag in self.diagnoses:
                report.append(f"\n[{diag['severity']}] {diag['component']}")
                report.append(f"  问题: {diag['message']}")
                report.append(f"  建议: {diag['recommendation']}")
        
        # 维护计划
        plan = self.get_maintenance_plan()
        urgent_items = [p for p in plan if p['urgency'] in ['URGENT', 'HIGH']]
        
        if urgent_items:
            report.append(f"\n紧急维护项目:")
            for item in urgent_items:
                report.append(f"  [{item['urgency']}] {item['component']}: {item['action']}")
        
        return "\n".join(report)


if __name__ == "__main__":
    print("故障诊断系统加载完成")
    
    # 创建诊断系统
    diagnostics = FaultDiagnostics()
    
    # 添加组件
    motor_wear = WearProfile(normal_wear_rate=0.002, expected_lifetime_hours=5000)
    motor = Component("电机_1", "电机", motor_wear)
    diagnostics.add_component(motor)
    
    bearing_wear = WearProfile(normal_wear_rate=0.001, expected_lifetime_hours=10000)
    bearing = Component("轴承_1", "轴承", bearing_wear)
    diagnostics.add_component(bearing)
    
    # 模拟运行
    print("\n模拟1000小时运行...")
    for hour in range(100):
        # 电机在高应力下运行
        stress = 1.5 if hour % 10 < 5 else 1.0
        temp = 60 + np.random.uniform(-5, 10)
        motor.update_wear(10, stress, temp)
        
        # 轴承正常运行
        bearing.update_wear(10, 1.0, 25 + np.random.uniform(-2, 2))
    
    # 运行诊断
    print("运行诊断...")
    results = diagnostics.run_diagnostics()
    
    print("\n" + diagnostics.get_diagnostics_report())
