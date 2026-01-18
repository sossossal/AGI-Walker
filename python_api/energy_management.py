"""
能量管理系统
Energy Management System

功能:
- 电池容量管理
- 实时功耗计算
- 续航时间预测
- 能效优化建议
"""

import numpy as np
from typing import Dict, List, Tuple


class Battery:
    """电池模型"""
    
    def __init__(self, capacity_wh: float, voltage: float = 24.0):
        """
        初始化电池
        
        参数:
            capacity_wh: 电池容量 (Wh)
            voltage: 标称电压 (V)
        """
        self.capacity_wh = capacity_wh
        self.voltage = voltage
        self.current_charge_wh = capacity_wh  # 当前电量
        self.charge_cycles = 0
        self.health = 1.0  # 电池健康度 (0-1)
        
        # 电池特性
        self.internal_resistance = 0.1  # 内阻 (Ω)
        self.discharge_efficiency = 0.95  # 放电效率
        self.charge_efficiency = 0.90     # 充电效率
    
    def discharge(self, power_w: float, duration_s: float) -> bool:
        """
        放电
        
        参数:
            power_w: 功率 (W)
            duration_s: 持续时间 (s)
        
        返回:
            是否成功（电量是否足够）
        """
        energy_required_wh = (power_w * duration_s / 3600) / self.discharge_efficiency
        
        if energy_required_wh > self.current_charge_wh:
            # 电量不足
            self.current_charge_wh = 0
            return False
        
        self.current_charge_wh -= energy_required_wh
        
        # 电池老化
        if self.current_charge_wh < self.capacity_wh * 0.2:
            # 深度放电会加速老化
            self.health *= 0.9999
        
        return True
    
    def charge(self, power_w: float, duration_s: float):
        """充电"""
        energy_added_wh = (power_w * duration_s / 3600) * self.charge_efficiency
        self.current_charge_wh = min(
            self.current_charge_wh + energy_added_wh,
            self.capacity_wh * self.health
        )
        
        if self.current_charge_wh >= self.capacity_wh * 0.99:
            self.charge_cycles += 1
            # 充电循环次数影响健康度
            self.health *= 0.9998
    
    def get_state_of_charge(self) -> float:
        """获取电量百分比"""
        return (self.current_charge_wh / self.capacity_wh) * 100
    
    def get_voltage(self) -> float:
        """获取当前电压（考虑负载）"""
        soc = self.get_state_of_charge() / 100
        # 简化的电压曲线
        return self.voltage * (0.85 + 0.15 * soc)
    
    def estimate_remaining_time(self, avg_power_w: float) -> float:
        """
        估算剩余时间
        
        参数:
            avg_power_w: 平均功率
        
        返回:
            剩余时间 (小时)
        """
        if avg_power_w <= 0:
            return float('inf')
        
        useable_energy = self.current_charge_wh * self.discharge_efficiency
        return useable_energy / avg_power_w


class PowerConsumer:
    """功耗设备"""
    
    def __init__(self, name: str, rated_power_w: float, usage_factor: float = 0.7):
        self.name = name
        self.rated_power_w = rated_power_w
        self.usage_factor = usage_factor  # 平均使用率
        self.is_active = False
        self.runtime_hours = 0
    
    def get_current_power(self) -> float:
        """获取当前功耗"""
        if self.is_active:
            # 添加一些随机波动
            variation = np.random.uniform(0.9, 1.1)
            return self.rated_power_w * self.usage_factor * variation
        return 0.0


class EnergyManager:
    """能量管理系统"""
    
    def __init__(self, battery: Battery, parts_config: Dict):
        self.battery = battery
        self.consumers = []
        
        # 从零件配置创建功耗设备
        self._initialize_consumers(parts_config)
        
        # 能量使用历史
        self.power_history = []
        self.time_history = []
    
    def _initialize_consumers(self, parts_config: Dict):
        """初始化功耗设备列表"""
        # 电机
        motor_power = parts_config.get('motor_power_multiplier', 1.0)
        num_motors = parts_config.get('num_motors', 6)
        for i in range(num_motors):
            self.consumers.append(
                PowerConsumer(f"电机_{i+1}", 500 * motor_power, 0.6)
            )
        
        # 控制器
        self.consumers.append(PowerConsumer("主控制器", 15, 1.0))
        self.consumers.append(PowerConsumer("从控制器", 5, 1.0))
        
        # 传感器
        self.consumers.append(PowerConsumer("IMU传感器", 0.5, 1.0))
        self.consumers.append(PowerConsumer("力传感器", 2.0, 0.8))
        
        # 通信
        self.consumers.append(PowerConsumer("WiFi模块", 2.0, 0.5))
    
    def simulate_step(self, dt: float = 0.01, motor_activity: float = 0.7):
        """
        模拟一步能量消耗
        
        参数:
            dt: 时间步长 (s)
            motor_activity: 电机活动度 (0-1)
        """
        total_power = 0
        
        # 激活设备并计算总功耗
        for consumer in self.consumers:
            if "电机" in consumer.name:
                consumer.is_active = motor_activity > 0.1
                consumer.usage_factor = motor_activity
            else:
                consumer.is_active = True
            
            total_power += consumer.get_current_power()
            consumer.runtime_hours += dt / 3600
        
        # 电池放电
        success = self.battery.discharge(total_power, dt)
        
        # 记录历史
        self.power_history.append(total_power)
        if len(self.time_history) == 0:
            self.time_history.append(dt)
        else:
            self.time_history.append(self.time_history[-1] + dt)
        
        return {
            'success': success,
            'total_power_w': total_power,
            'battery_soc': self.battery.get_state_of_charge(),
            'battery_voltage': self.battery.get_voltage()
        }
    
    def get_energy_report(self) -> str:
        """生成能量报告"""
        soc = self.battery.get_state_of_charge()
        avg_power = np.mean(self.power_history) if self.power_history else 0
        remaining_time = self.battery.estimate_remaining_time(avg_power)
        
        report = []
        report.append("="*70)
        report.append("能量管理报告")
        report.append("="*70)
        
        report.append(f"\n电池状态:")
        report.append(f"  容量: {self.battery.capacity_wh:.1f} Wh")
        report.append(f"  当前电量: {self.battery.current_charge_wh:.1f} Wh ({soc:.1f}%)")
        report.append(f"  健康度: {self.battery.health*100:.1f}%")
        report.append(f"  充电周期: {self.battery.charge_cycles}")
        
        report.append(f"\n功耗统计:")
        report.append(f"  平均功耗: {avg_power:.1f} W")
        if self.power_history:
            report.append(f"  峰值功耗: {max(self.power_history):.1f} W")
            report.append(f"  最低功耗: {min(self.power_history):.1f} W")
        
        report.append(f"\n续航预测:")
        if remaining_time < float('inf'):
            if remaining_time >= 1:
                report.append(f"  剩余时间: {remaining_time:.2f} 小时")
            else:
                report.append(f"  剩余时间: {remaining_time*60:.1f} 分钟")
        else:
            report.append(f"  剩余时间: 无限（无负载）")
        
        report.append(f"\n设备详情:")
        active_consumers = [c for c in self.consumers if c.is_active]
        for consumer in active_consumers:
            power = consumer.get_current_power()
            percentage = (power / avg_power * 100) if avg_power > 0 else 0
            report.append(f"  {consumer.name:<15} {power:>6.1f}W ({percentage:>5.1f}%)")
        
        # 能效建议
        report.append(f"\n能效建议:")
        if soc < 20:
            report.append("  ⚠️  电量低于20%，建议充电")
        if avg_power > 200:
            report.append("  💡 功耗较高，考虑降低电机功率")
        if self.battery.health < 0.8:
            report.append("  ⚠️  电池健康度低于80%，建议更换")
        
        return "\n".join(report)
    
    def optimize_power_distribution(self) -> Dict:
        """优化功耗分配"""
        avg_power = np.mean(self.power_history) if self.power_history else 0
        soc = self.battery.get_state_of_charge()
        
        recommendations = {}
        
        # 根据电量调整策略
        if soc < 30:
            # 低电量模式
            recommendations['motor_power_multiplier'] = 0.7
            recommendations['mode'] = '节能模式'
        elif soc < 50:
            # 平衡模式
            recommendations['motor_power_multiplier'] = 0.9
            recommendations['mode'] = '平衡模式'
        else:
            # 性能模式
            recommendations['motor_power_multiplier'] = 1.0
            recommendations['mode'] = '性能模式'
        
        return recommendations


if __name__ == "__main__":
    print("能量管理系统加载完成")
    
    # 示例
    battery = Battery(capacity_wh=111, voltage=22.2)  # 6S 5000mAh
    parts_config = {
        'motor_power_multiplier': 1.0,
        'num_motors': 6
    }
    
    energy_mgr = EnergyManager(battery, parts_config)
    
    # 模拟10秒运行
    for i in range(1000):
        result = energy_mgr.simulate_step(dt=0.01, motor_activity=0.7)
        
        if i % 100 == 0:
            print(f"时间: {i*0.01:.1f}s, 功耗: {result['total_power_w']:.1f}W, "
                  f"电量: {result['battery_soc']:.1f}%")
    
    print("\n" + energy_mgr.get_energy_report())
