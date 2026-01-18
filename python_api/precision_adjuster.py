"""
精确零件参数调节器
支持0.1精度，从0到满功率的精细调节
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.custom_parts import CustomMotor, CustomJoint
from python_api.physics_validator import PhysicsValidator
import numpy as np


class PrecisionPartAdjuster:
    """精确零件参数调节器"""
    
    def __init__(self, part_type='motor'):
        self.part_type = part_type
        self.validator = PhysicsValidator()
        
        if part_type == 'motor':
            self.part = CustomMotor()
            self.param_ranges = {
                'power': (0.0, 1500.0, 'W'),           # 0-1500W
                'voltage': (0.0, 60.0, 'V'),           # 0-60V
                'gear_ratio': (1.0, 200.0, ':1'),      # 1-200倍
                'efficiency': (0.0, 1.0, '%'),         # 0-100%
                'weight': (0.1, 5.0, 'kg')             # 0.1-5kg
            }
        elif part_type == 'joint':
            self.part = CustomJoint()
            self.param_ranges = {
                'reduction_ratio': (1.0, 200.0, ':1'),
                'max_torque': (0.0, 150.0, 'Nm'),
                'backlash': (0.001, 1.0, '°'),
                'efficiency': (0.0, 1.0, '%'),
                'weight': (0.05, 3.0, 'kg'),
                'stiffness': (100.0, 15000.0, 'Nm/rad')
            }
    
    def round_to_precision(self, value: float, precision: float = 0.1) -> float:
        """四舍五入到指定精度"""
        return round(value / precision) * precision
    
    def set_parameter(self, param_name: str, value: float) -> dict:
        """
        设置参数值（精确到0.1）
        
        参数:
            param_name: 参数名称
            value: 参数值（将自动四舍五入到0.1精度）
        
        返回:
            调整结果和影响分析
        """
        # 检查参数是否存在
        if param_name not in self.param_ranges:
            return {
                'success': False,
                'error': f'未知参数: {param_name}'
            }
        
        # 四舍五入到0.1精度
        precise_value = self.round_to_precision(value, 0.1)
        
        # 检查范围
        min_val, max_val, unit = self.param_ranges[param_name]
        if precise_value < min_val or precise_value > max_val:
            return {
                'success': False,
                'error': f'参数超出范围 [{min_val}, {max_val}]',
                'input_value': value,
                'rounded_value': precise_value
            }
        
        # 特殊处理百分比参数
        if unit == '%':
            display_value = precise_value
            internal_value = precise_value  # 已经是0-1范围
        else:
            display_value = precise_value
            internal_value = precise_value
        
        # 更新参数
        old_value = self.part.params.get(param_name, 0)
        result = self.part.update_param(param_name, internal_value)
        
        # 重新计算性能
        self.part._recalculate_performance()
        
        # 验证物理合理性
        if self.part_type == 'motor':
            is_valid, issues = self.validator.validate_motor_config(self.part.params)
        else:
            is_valid, issues = self.validator.validate_joint_config(self.part.params)
        
        return {
            'success': True,
            'parameter': param_name,
            'old_value': old_value,
            'new_value': internal_value,
            'display_value': f'{display_value:.1f} {unit}',
            'precision': 0.1,
            'range': f'[{min_val:.1f}, {max_val:.1f}] {unit}',
            'impact': result.get('impact', {}),
            'performance': self.part.performance_metrics,
            'is_valid': is_valid,
            'warnings': issues
        }
    
    def sweep_parameter(self, param_name: str, steps: int = 20) -> list:
        """
        扫描参数范围，返回性能曲线
        
        参数:
            param_name: 参数名称
            steps: 步数
        
        返回:
            [(value, performance_metrics), ...]
        """
        if param_name not in self.param_ranges:
            return []
        
        min_val, max_val, unit = self.param_ranges[param_name]
        
        # 生成扫描点（精确到0.1）
        values = []
        step_size = (max_val - min_val) / (steps - 1)
        for i in range(steps):
            value = min_val + i * step_size
            precise_value = self.round_to_precision(value, 0.1)
            values.append(precise_value)
        
        # 去重
        values = sorted(list(set(values)))
        
        results = []
        original_value = self.part.params.get(param_name, 0)
        
        for value in values:
            self.part.update_param(param_name, value)
            self.part._recalculate_performance()
            results.append((
                value,
                self.part.performance_metrics.copy()
            ))
        
        # 恢复原值
        self.part.update_param(param_name, original_value)
        self.part._recalculate_performance()
        
        return results
    
    def get_current_state(self) -> dict:
        """获取当前状态"""
        return {
            'part_type': self.part_type,
            'parameters': self.part.params.copy(),
            'performance': self.part.performance_metrics.copy(),
            'parameter_ranges': self.param_ranges
        }
    
    def reset_to_default(self):
        """重置到默认值"""
        if self.part_type == 'motor':
            self.part = CustomMotor()
        else:
            self.part = CustomJoint()


class InteractivePrecisionTuner:
    """交互式精确调节工具"""
    
    def __init__(self):
        self.adjuster = None
        self.part_type = None
    
    def start(self):
        """启动交互式调节"""
        print("="*70)
        print("精确零件参数调节器 (0.1精度)")
        print("="*70)
        
        # 选择零件类型
        print("\n选择零件类型:")
        print("  1. 电机 (Motor)")
        print("  2. 关节装置 (Joint)")
        
        choice = input("\n请选择 (1/2): ").strip()
        
        if choice == '1':
            self.part_type = 'motor'
            self.adjuster = PrecisionPartAdjuster('motor')
            print("\n✓ 已创建电机调节器")
        elif choice == '2':
            self.part_type = 'joint'
            self.adjuster = PrecisionPartAdjuster('joint')
            print("\n✓ 已创建关节调节器")
        else:
            print("无效选择")
            return
        
        self.show_help()
        self.interactive_loop()
    
    def show_help(self):
        """显示帮助"""
        print("\n命令:")
        print("  set <参数名> <值>  - 设置参数 (精确到0.1)")
        print("  get                - 显示当前所有参数")
        print("  perf               - 显示性能指标")
        print("  range <参数名>     - 显示参数范围")
        print("  sweep <参数名>     - 扫描参数影响")
        print("  reset              - 重置到默认值")
        print("  help               - 显示帮助")
        print("  quit               - 退出")
    
    def show_parameters(self):
        """显示当前参数"""
        state = self.adjuster.get_current_state()
        
        print("\n当前参数:")
        print("-"*70)
        for param, value in state['parameters'].items():
            if param in state['parameter_ranges']:
                min_val, max_val, unit = state['parameter_ranges'][param]
                if unit == '%':
                    display = f"{value*100:.1f}%"
                else:
                    display = f"{value:.1f} {unit}"
                
                # 计算百分比
                percentage = (value - min_val) / (max_val - min_val) * 100
                bar_length = int(percentage / 5)  # 20格
                bar = '█' * bar_length + '░' * (20 - bar_length)
                
                print(f"  {param:<20} {display:>15}  [{bar}] {percentage:.1f}%")
    
    def show_performance(self):
        """显示性能指标"""
        state = self.adjuster.get_current_state()
        
        print("\n性能指标:")
        print("-"*70)
        for metric, value in state['performance'].items():
            if isinstance(value, (int, float)):
                print(f"  {metric:<30} {value:.3f}")
            else:
                print(f"  {metric:<30} {value}")
    
    def interactive_loop(self):
        """交互循环"""
        while True:
            try:
                cmd = input("\n> ").strip().split()
                
                if not cmd:
                    continue
                
                if cmd[0] == 'quit':
                    print("再见！")
                    break
                
                elif cmd[0] == 'help':
                    self.show_help()
                
                elif cmd[0] == 'get':
                    self.show_parameters()
                
                elif cmd[0] == 'perf':
                    self.show_performance()
                
                elif cmd[0] == 'set' and len(cmd) == 3:
                    param_name = cmd[1]
                    try:
                        value = float(cmd[2])
                        result = self.adjuster.set_parameter(param_name, value)
                        
                        if result['success']:
                            print(f"\n✓ 参数已更新")
                            print(f"  {param_name}: {result['display_value']}")
                            print(f"  精度: ±{result['precision']}")
                            print(f"  范围: {result['range']}")
                            
                            if result['impact']:
                                print(f"\n  影响:")
                                for key, val in result['impact'].items():
                                    print(f"    {key}: {val}")
                            
                            if not result['is_valid']:
                                print(f"\n  ⚠️  警告:")
                                for warning in result['warnings']:
                                    print(f"    {warning}")
                        else:
                            print(f"\n✗ 错误: {result['error']}")
                    
                    except ValueError:
                        print("✗ 错误: 值必须是数字")
                
                elif cmd[0] == 'range' and len(cmd) == 2:
                    param_name = cmd[1]
                    state = self.adjuster.get_current_state()
                    
                    if param_name in state['parameter_ranges']:
                        min_val, max_val, unit = state['parameter_ranges'][param_name]
                        current = state['parameters'].get(param_name, 0)
                        
                        print(f"\n参数: {param_name}")
                        print(f"  当前值: {current:.1f} {unit}")
                        print(f"  最小值: {min_val:.1f} {unit}")
                        print(f"  最大值: {max_val:.1f} {unit}")
                        print(f"  精度: 0.1 {unit}")
                        print(f"  可调节步数: {int((max_val - min_val) / 0.1)}")
                    else:
                        print(f"✗ 未知参数: {param_name}")
                
                elif cmd[0] == 'sweep' and len(cmd) == 2:
                    param_name = cmd[1]
                    print(f"\n扫描参数: {param_name}")
                    
                    results = self.adjuster.sweep_parameter(param_name, steps=10)
                    
                    if results:
                        print("-"*70)
                        print(f"{'值':<15} {'关键指标变化':<50}")
                        print("-"*70)
                        
                        # 获取第一个关键指标
                        first_metric = list(results[0][1].keys())[0]
                        
                        for value, metrics in results:
                            metric_value = metrics.get(first_metric, 0)
                            print(f"{value:<15.1f} {first_metric}: {metric_value:.3f}")
                
                elif cmd[0] == 'reset':
                    self.adjuster.reset_to_default()
                    print("✓ 已重置到默认值")
                
                else:
                    print("未知命令，输入 'help' 查看帮助")
            
            except KeyboardInterrupt:
                print("\n\n再见！")
                break
            except Exception as e:
                print(f"错误: {e}")


if __name__ == "__main__":
    tuner = InteractivePrecisionTuner()
    tuner.start()
