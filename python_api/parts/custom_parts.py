"""
定制化零件系统
允许用户自定义零件参数并获得实时仿真反馈
"""

import json
import numpy as np
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt


class CustomPart:
    """可定制零件基类"""
    
    def __init__(self, part_type: str, base_params: Dict):
        self.part_type = part_type
        self.params = base_params.copy()
        self.performance_metrics = {}
    
    def update_param(self, param_name: str, value):
        """更新参数"""
        if param_name in self.params:
            old_value = self.params[param_name]
            self.params[param_name] = value
            
            # 重新计算性能
            self._recalculate_performance()
            
            return {
                'param': param_name,
                'old_value': old_value,
                'new_value': value,
                'impact': self._calculate_impact(param_name, old_value, value)
            }
        else:
            raise ValueError(f"参数 '{param_name}' 不存在")
    
    def _recalculate_performance(self):
        """重新计算性能指标（子类实现）"""
        pass
    
    def _calculate_impact(self, param_name: str, old_value, new_value) -> Dict:
        """计算参数变化的影响（子类实现）"""
        return {}
    
    def get_performance_report(self) -> str:
        """获取性能报告"""
        report = []
        report.append(f"零件类型: {self.part_type}")
        report.append("\n当前参数:")
        for key, value in self.params.items():
            report.append(f"  {key}: {value}")
        
        report.append("\n性能指标:")
        for key, value in self.performance_metrics.items():
            if isinstance(value, float):
                report.append(f"  {key}: {value:.3f}")
            else:
                report.append(f"  {key}: {value}")
        
        return "\n".join(report)


class CustomMotor(CustomPart):
    """可定制电机"""
    
    def __init__(self, base_params: Dict = None):
        default_params = {
            'power': 500,           # W
            'voltage': 24,          # V
            'gear_ratio': 50,       # 减速比
            'efficiency': 0.85,     # 效率
            'weight': 0.8,          # kg
            'max_current': 20       # A
        }
        
        if base_params:
            default_params.update(base_params)
        
        super().__init__('custom_motor', default_params)
        self._recalculate_performance()
    
    def _recalculate_performance(self):
        """计算电机性能"""
        # 扭矩计算
        power_w = self.params['power']
        gear_ratio = self.params['gear_ratio']
        efficiency = self.params['efficiency']
        voltage = self.params['voltage']
        
        # 无负载转速 (RPM)
        base_rpm = 3000  # 假设基础转速
        rpm = base_rpm / gear_ratio
        
        # 输出扭矩 (Nm)
        # T = (P * 60) / (2π * n) * η
        torque = (power_w * 60 * efficiency) / (2 * np.pi * rpm)
        
        # 电流估算 (A)
        current = power_w / voltage
        
        # 热功率 (W)
        heat = power_w * (1 - efficiency)
        
        # 成本估算 (基于功率和减速比)
        base_cost = 50
        power_cost = power_w * 0.15
        gear_cost = gear_ratio * 0.5
        estimated_cost = base_cost + power_cost + gear_cost
        
        self.performance_metrics = {
            'output_torque_nm': torque,
            'output_speed_rpm': rpm,
            'current_draw_a': current,
            'heat_generation_w': heat,
            'torque_to_weight_ratio': torque / self.params['weight'],
            'estimated_cost_usd': estimated_cost
        }
    
    def _calculate_impact(self, param_name: str, old_value, new_value) -> Dict:
        """计算参数变化影响"""
        impact = {}
        
        if param_name == 'gear_ratio':
            ratio_change = (new_value - old_value) / old_value
            impact['torque_change'] = f"+{ratio_change*100:.1f}%"
            impact['speed_change'] = f"-{ratio_change*100:.1f}%"
            impact['cost_change'] = f"+${abs(new_value - old_value) * 0.5:.2f}"
        
        elif param_name == 'power':
            power_change = (new_value - old_value) / old_value
            impact['torque_change'] = f"+{power_change*100:.1f}%"
            impact['cost_change'] = f"+${abs(new_value - old_value) * 0.15:.2f}"
            impact['heat_change'] = f"+{power_change*100:.1f}%"
        
        elif param_name == 'efficiency':
            eff_change = new_value - old_value
            impact['torque_change'] = f"+{eff_change*100:.1f}%"
            impact['heat_change'] = f"-{eff_change*100:.1f}%"
        
        return impact


class CustomJoint(CustomPart):
    """可定制关节装置"""
    
    def __init__(self, base_params: Dict = None):
        default_params = {
            'type': 'harmonic_drive',     # 谐波减速器
            'reduction_ratio': 100,        # 减速比
            'max_torque': 50,             # Nm
            'backlash': 0.05,             # 度（回差）
            'efficiency': 0.90,           # 效率
            'weight': 0.3,                # kg
            'stiffness': 5000,            # Nm/rad（刚度）
            'max_speed': 100              # RPM
        }
        
        if base_params:
            default_params.update(base_params)
        
        super().__init__('custom_joint', default_params)
        self._recalculate_performance()
    
    def _recalculate_performance(self):
        """计算关节性能"""
        ratio = self.params['reduction_ratio']
        max_torque = self.params['max_torque']
        backlash = self.params['backlash']
        efficiency = self.params['efficiency']
        weight = self.params['weight']
        stiffness = self.params['stiffness']
        
        # 定位精度 (与回差和减速比相关)
        positioning_accuracy = backlash / ratio  # 度
        
        # 扭矩密度
        torque_density = max_torque / weight  # Nm/kg
        
        # 动态响应 (与刚度相关)
        # 简化模型: resonance_freq ≈ sqrt(k/m)
        resonance_freq = np.sqrt(stiffness / weight) / (2 * np.pi)  # Hz
        
        # 损耗功率 (假设满载)
        output_power = max_torque * (self.params['max_speed'] * 2 * np.pi / 60)
        loss_power = output_power * (1 - efficiency) / efficiency
        
        # 成本估算
        base_cost = 80
        ratio_cost = ratio * 0.3
        torque_cost = max_torque * 1.5
        precision_cost = (1 / backlash) * 10 if backlash > 0 else 100
        estimated_cost = base_cost + ratio_cost + torque_cost + precision_cost
        
        self.performance_metrics = {
            'positioning_accuracy_deg': positioning_accuracy,
            'torque_density_nm_per_kg': torque_density,
            'resonance_frequency_hz': resonance_freq,
            'power_loss_w': loss_power,
            'estimated_cost_usd': estimated_cost,
            'control_bandwidth_hz': resonance_freq * 0.1  # 控制带宽约为共振频率的1/10
        }
    
    def _calculate_impact(self, param_name: str, old_value, new_value) -> Dict:
        """计算参数变化影响"""
        impact = {}
        
        if param_name == 'reduction_ratio':
            ratio_change = (new_value - old_value) / old_value
            impact['output_torque'] = f"+{ratio_change*100:.1f}%"
            impact['positioning_accuracy'] = f"+{ratio_change*100:.1f}% (更好)"
            impact['cost'] = f"+${abs(new_value - old_value) * 0.3:.2f}"
        
        elif param_name == 'backlash':
            if old_value > 0:
                backlash_change = (old_value - new_value) / old_value
                impact['positioning_accuracy'] = f"+{backlash_change*100:.1f}% (更好)"
                impact['cost'] = f"+${abs(1/new_value - 1/old_value) * 10:.2f}"
        
        elif param_name == 'stiffness':
            stiffness_change = (new_value - old_value) / old_value
            impact['dynamic_response'] = f"+{stiffness_change*50:.1f}%"
            impact['control_bandwidth'] = f"+{stiffness_change*50:.1f}%"
        
        return impact


class CustomSensor(CustomPart):
    """可定制传感器"""
    
    def __init__(self, sensor_type: str, base_params: Dict = None):
        default_params = {
            'resolution': 12,          # bits
            'sample_rate': 1000,       # Hz
            'noise_level': 0.01,       # % of full scale
            'power_consumption': 0.1,  # W
            'weight': 0.02,           # kg
            'interface': 'SPI'
        }
        
        if base_params:
            default_params.update(base_params)
        
        super().__init__(f'custom_sensor_{sensor_type}', default_params)
        self.sensor_type = sensor_type
        self._recalculate_performance()
    
    def _recalculate_performance(self):
        """计算传感器性能"""
        resolution_bits = self.params['resolution']
        sample_rate = self.params['sample_rate']
        noise_level = self.params['noise_level']
        
        # 有效位数 (ENOB) - 考虑噪声
        enob = resolution_bits - np.log2(1 + noise_level * 100)
        
        # 信噪比 (SNR)
        snr_db = 6.02 * enob + 1.76
        
        # 数据率
        data_rate_mbps = (resolution_bits * sample_rate) / 1_000_000
        
        # 成本估算
        base_cost = 15
        resolution_cost = (2 ** resolution_bits) * 0.001
        sample_cost = sample_rate * 0.01
        estimated_cost = base_cost + resolution_cost + sample_cost
        
        self.performance_metrics = {
            'effective_bits': enob,
            'snr_db': snr_db,
            'data_rate_mbps': data_rate_mbps,
            'estimated_cost_usd': estimated_cost
        }


class PartCustomizer:
    """零件定制工具"""
    
    def __init__(self):
        self.parts = {}
    
    def create_motor(self, name: str, params: Dict = None) -> CustomMotor:
        """创建定制电机"""
        motor = CustomMotor(params)
        self.parts[name] = motor
        return motor
    
    def create_joint(self, name: str, params: Dict = None) -> CustomJoint:
        """创建定制关节"""
        joint = CustomJoint(params)
        self.parts[name] = joint
        return joint
    
    def create_sensor(self, name: str, sensor_type: str, params: Dict = None) -> CustomSensor:
        """创建定制传感器"""
        sensor = CustomSensor(sensor_type, params)
        self.parts[name] = sensor
        return sensor
    
    def compare_configurations(self, part_name: str, configs: List[Dict]):
        """对比不同配置"""
        if part_name not in self.parts:
            raise ValueError(f"零件 '{part_name}' 不存在")
        
        base_part = self.parts[part_name]
        results = []
        
        for i, config in enumerate(configs):
            # 创建临时零件
            if isinstance(base_part, CustomMotor):
                temp_part = CustomMotor(config)
            elif isinstance(base_part, CustomJoint):
                temp_part = CustomJoint(config)
            else:
                continue
            
            results.append({
                'config_id': i + 1,
                'params': config,
                'metrics': temp_part.performance_metrics
            })
        
        return results
    
    def visualize_parameter_sweep(self, part_type: str, param_name: str, 
                                  param_range: Tuple[float, float], steps: int = 20):
        """参数扫描可视化"""
        if part_type == 'motor':
            base_part = CustomMotor()
        elif part_type == 'joint':
            base_part = CustomJoint()
        else:
            raise ValueError("不支持的零件类型")
        
        # 参数扫描
        param_values = np.linspace(param_range[0], param_range[1], steps)
        metrics_history = {key: [] for key in base_part.performance_metrics.keys()}
        
        for value in param_values:
            base_part.update_param(param_name, value)
            for key, metric_value in base_part.performance_metrics.items():
                metrics_history[key].append(metric_value)
        
        # 绘图
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        axes = axes.flatten()
        
        metric_keys = list(metrics_history.keys())[:4]  # 显示前4个指标
        
        for idx, key in enumerate(metric_keys):
            axes[idx].plot(param_values, metrics_history[key], 'b-', linewidth=2)
            axes[idx].set_xlabel(param_name)
            axes[idx].set_ylabel(key)
            axes[idx].set_title(f'{key} vs {param_name}')
            axes[idx].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(f'param_sweep_{part_type}_{param_name}.png', dpi=150)
        print(f"图表已保存: param_sweep_{part_type}_{param_name}.png")
        
        return metrics_history


if __name__ == "__main__":
    print("定制化零件系统示例\n")
    
    # 创建定制器
    customizer = PartCustomizer()
    
    # 示例 1: 定制电机
    print("="*60)
    print("示例 1: 定制电机")
    print("="*60)
    
    motor = customizer.create_motor("my_motor", {
        'power': 500,
        'gear_ratio': 50
    })
    
    print("\n初始配置:")
    print(motor.get_performance_report())
    
    # 修改参数
    print("\n修改减速比 50 → 100:")
    impact = motor.update_param('gear_ratio', 100)
    print(f"影响: {impact['impact']}")
    
    print("\n更新后:")
    print(motor.get_performance_report())
    
    # 示例 2: 定制关节
    print("\n" + "="*60)
    print("示例 2: 定制关节装置")
    print("="*60)
    
    joint = customizer.create_joint("my_joint", {
        'reduction_ratio': 100,
        'backlash': 0.05
    })
    
    print(joint.get_performance_report())
