"""
热管理系统
Thermal Management System

功能:
- 温度模拟
- 过热检测
- 散热需求计算
- 热节流策略
"""

import numpy as np
from typing import Dict, List, Tuple


class ThermalComponent:
    """热组件（电机、驱动器等）"""
    
    def __init__(self, name: str, thermal_resistance: float, thermal_capacity: float):
        """
        初始化热组件
        
        参数:
            name: 组件名称
            thermal_resistance: 热阻 (°C/W)
            thermal_capacity: 热容 (J/°C)
        """
        self.name = name
        self.thermal_resistance = thermal_resistance  # °C/W
        self.thermal_capacity = thermal_capacity      # J/°C
        self.temperature = 25.0  # 当前温度 °C
        self.max_temp = 85.0     # 最大允许温度
        self.warning_temp = 70.0  # 警告温度
        
        # 散热器
        self.has_heatsink = False
        self.heatsink_efficiency = 0.5
    
    def add_heatsink(self, efficiency: float = 0.5):
        """添加散热器"""
        self.has_heatsink = True
        self.heatsink_efficiency = efficiency
        # 散热器降低热阻
        self.thermal_resistance *= (1 - efficiency)
    
    def update_temperature(self, power_w: float, ambient_temp: float, dt: float):
        """
        更新温度
        
        参数:
            power_w: 功耗 (W)
            ambient_temp: 环境温度 (°C)
            dt: 时间步长 (s)
        """
        # 温升 = 功耗 × 热阻
        steady_state_temp = ambient_temp + power_w * self.thermal_resistance
        
        # 一阶热传导模型
        # dT/dt = (T_ss - T) / τ, 其中 τ = R * C
        tau = self.thermal_resistance * self.thermal_capacity
        
        # 指数逼近
        alpha = dt / tau
        self.temperature = self.temperature + alpha * (steady_state_temp - self.temperature)
    
    def get_status(self) -> Dict:
        """获取状态"""
        if self.temperature >= self.max_temp:
            status = "过热"
            level = "CRITICAL"
        elif self.temperature >= self.warning_temp:
            status = "警告"
            level = "WARNING"
        else:
            status = "正常"
            level = "OK"
        
        margin = self.max_temp - self.temperature
        
        return {
            'temperature': self.temperature,
            'status': status,
            'level': level,
            'margin': margin,
            'utilization': (self.temperature - 25) / (self.max_temp - 25) * 100
        }


class ThermalManager:
    """热管理系统"""
    
    def __init__(self, parts_config: Dict, ambient_temp: float = 25.0):
        self.ambient_temp = ambient_temp
        self.components = []
        
        # 从零件配置创建热组件
        self._initialize_components(parts_config)
        
        # 热历史
        self.temp_history = {comp.name: [] for comp in self.components}
        self.time_history = []
        
        # 节流状态
        self.throttling_active = False
        self.throttle_factor = 1.0
    
    def _initialize_components(self, parts_config: Dict):
        """初始化热组件"""
        motor_power = parts_config.get('motor_power_multiplier', 1.0)
        num_motors = parts_config.get('num_motors', 6)
        
        # 电机
        for i in range(num_motors):
            motor = ThermalComponent(
                f"电机_{i+1}",
                thermal_resistance=15.0,  # °C/W
                thermal_capacity=50.0     # J/°C
            )
            # 根据配置决定是否有散热器
            if parts_config.get('has_heatsink', True):
                motor.add_heatsink(efficiency=0.4)
            
            self.components.append(motor)
        
        # 电机驱动器
        for i in range(num_motors // 2):
            driver = ThermalComponent(
                f"驱动器_{i+1}",
                thermal_resistance=10.0,
                thermal_capacity=30.0
            )
            driver.max_temp = 90.0
            driver.warning_temp = 75.0
            
            if parts_config.get('has_heatsink', True):
                driver.add_heatsink(efficiency=0.5)
            
            self.components.append(driver)
    
    def simulate_step(self, power_distribution: Dict, dt: float = 0.01):
        """
        模拟一步热传导
        
        参数:
            power_distribution: {component_name: power_w}
            dt: 时间步长
        """
        critical_components = []
        
        for component in self.components:
            # 获取该组件的功耗
            power = power_distribution.get(component.name, 0.0)
            
            # 如果正在节流，降低功耗
            if self.throttling_active:
                power *= self.throttle_factor
            
            # 更新温度
            component.update_temperature(power, self.ambient_temp, dt)
            
            # 记录历史
            self.temp_history[component.name].append(component.temperature)
            
            # 检查过热
            status = component.get_status()
            if status['level'] in ['WARNING', 'CRITICAL']:
                critical_components.append((component, status))
        
        # 记录时间
        if len(self.time_history) == 0:
            self.time_history.append(dt)
        else:
            self.time_history.append(self.time_history[-1] + dt)
        
        # 热节流决策
        self._update_throttling(critical_components)
        
        return {
            'max_temp': max(c.temperature for c in self.components),
            'avg_temp': np.mean([c.temperature for c in self.components]),
            'throttling': self.throttling_active,
            'throttle_factor': self.throttle_factor,
            'critical_components': [c[0].name for c in critical_components]
        }
    
    def _update_throttling(self, critical_components: List):
        """更新热节流状态"""
        if not critical_components:
            # 没有过热组件，逐渐恢复
            if self.throttling_active:
                self.throttle_factor = min(1.0, self.throttle_factor + 0.01)
                if self.throttle_factor >= 0.99:
                    self.throttling_active = False
            return
        
        # 有过热组件
        critical_count = sum(1 for _, status in critical_components if status['level'] == 'CRITICAL')
        
        if critical_count > 0:
            # 严重过热，大幅降低功率
            self.throttling_active = True
            self.throttle_factor = max(0.5, self.throttle_factor - 0.05)
        else:
            # 只是警告，轻微降低
            self.throttling_active = True
            self.throttle_factor = max(0.7, self.throttle_factor - 0.02)
    
    def check_cooling_requirements(self) -> Dict:
        """检查散热需求"""
        requirements = {
            'needs_heatsink': [],
            'needs_fan': [],
            'needs_liquid_cooling': []
        }
        
        for component in self.components:
            avg_temp = np.mean(self.temp_history[component.name][-100:]) if len(self.temp_history[component.name]) > 0 else 25.0
            
            if avg_temp > component.warning_temp:
                if not component.has_heatsink:
                    requirements['needs_heatsink'].append(component.name)
                elif avg_temp > component.warning_temp + 10:
                    requirements['needs_fan'].append(component.name)
                
                if avg_temp > component.max_temp - 5:
                    requirements['needs_liquid_cooling'].append(component.name)
        
        return requirements
    
    def get_thermal_report(self) -> str:
        """生成热管理报告"""
        report = []
        report.append("="*70)
        report.append("热管理报告")
        report.append("="*70)
        
        report.append(f"\n环境温度: {self.ambient_temp:.1f}°C")
        report.append(f"节流状态: {'激活' if self.throttling_active else '未激活'}")
        if self.throttling_active:
            report.append(f"节流因子: {self.throttle_factor:.2f} ({(1-self.throttle_factor)*100:.0f}% 功率降低)")
        
        report.append(f"\n组件温度:")
        report.append(f"{'组件':<15} {'当前温度':<12} {'状态':<10} {'余量':<10} {'利用率':<10}")
        report.append("-"*70)
        
        for component in self.components:
            status = component.get_status()
            report.append(
                f"{component.name:<15} "
                f"{status['temperature']:<12.1f}°C "
                f"{status['status']:<10} "
                f"{status['margin']:<10.1f}°C "
                f"{status['utilization']:<10.1f}%"
            )
        
        # 散热建议
        cooling_req = self.check_cooling_requirements()
        
        if any(cooling_req.values()):
            report.append(f"\n散热建议:")
            if cooling_req['needs_heatsink']:
                report.append(f"  需要散热器: {', '.join(cooling_req['needs_heatsink'])}")
            if cooling_req['needs_fan']:
                report.append(f"  需要风扇: {', '.join(cooling_req['needs_fan'])}")
            if cooling_req['needs_liquid_cooling']:
                report.append(f"  需要液冷: {', '.join(cooling_req['needs_liquid_cooling'])}")
        
        return "\n".join(report)


if __name__ == "__main__":
    print("热管理系统加载完成")
    
    # 示例
    parts_config = {
        'motor_power_multiplier': 1.0,
        'num_motors': 6,
        'has_heatsink': True
    }
    
    thermal_mgr = ThermalManager(parts_config, ambient_temp=30.0)
    
    # 模拟运行
    for i in range(1000):
        # 模拟功耗分配
        power_dist = {}
        for comp in thermal_mgr.components:
            if "电机" in comp.name:
                power_dist[comp.name] = 300 * 0.7  # 70%负载
            elif "驱动器" in comp.name:
                power_dist[comp.name] = 50 * 0.7
        
        result = thermal_mgr.simulate_step(power_dist, dt=0.1)
        
        if i % 100 == 0:
            print(f"时间: {i*0.1:.1f}s, 最高温度: {result['max_temp']:.1f}°C, "
                  f"节流: {result['throttling']}")
    
    print("\n" + thermal_mgr.get_thermal_report())
