"""
定制化零件系统完整示例
展示如何自定义零件并查看参数变化的影响
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.custom_parts import (
    PartCustomizer, CustomMotor, CustomJoint, CustomSensor
)

def demo_custom_motor():
    """示例 1: 定制电机"""
    print("="*70)
    print("示例 1: 定制电机并分析参数影响")
    print("="*70)
    
    customizer = PartCustomizer()
    
    # 创建初始电机
    print("\n创建基础电机配置...")
    motor = customizer.create_motor("my_motor", {
        'power': 500,
        'voltage': 24,
        'gear_ratio': 50,
        'efficiency': 0.85
    })
    
    print("\n初始性能:")
    print(motor.get_performance_report())
    
    # 实验 1: 增加减速比
    print("\n" + "-"*70)
    print("实验 1: 将减速比从 50 增加到 100")
    print("-"*70)
    result = motor.update_param('gear_ratio', 100)
    print(f"\n参数影响:")
    for key, value in result['impact'].items():
        print(f"  {key}: {value}")
    
    print("\n更新后性能:")
    print(motor.get_performance_report())
    
    # 实验 2: 增加功率
    print("\n" + "-"*70)
    print("实验 2: 将功率从 500W 增加到 750W")
    print("-"*70)
    result = motor.update_param('power', 750)
    print(f"\n参数影响:")
    for key, value in result['impact'].items():
        print(f"  {key}: {value}")


def demo_custom_joint():
    """示例 2: 定制关节装置"""
    print("\n" + "="*70)
    print("示例 2: 定制关节装置")
    print("="*70)
    
    customizer = PartCustomizer()
    
    # 创建谐波减速器
    print("\n创建谐波减速器...")
    joint = customizer.create_joint("harmonic_joint", {
        'type': 'harmonic_drive',
        'reduction_ratio': 100,
        'backlash': 0.05,
        'stiffness': 5000
    })
    
    print("\n初始性能:")
    print(joint.get_performance_report())
    
    # 实验: 减少回差
    print("\n" + "-"*70)
    print("实验: 将回差从 0.05° 减少到 0.01° (提高精度)")
    print("-"*70)
    result = joint.update_param('backlash', 0.01)
    print(f"\n参数影响:")
    for key, value in result['impact'].items():
        print(f"  {key}: {value}")
    
    print("\n更新后性能:")
    print(joint.get_performance_report())


def demo_configuration_comparison():
    """示例 3: 配置对比"""
    print("\n" + "="*70)
    print("示例 3: 对比不同电机配置")
    print("="*70)
    
    customizer = PartCustomizer()
    
    # 定义待对比的配置
    configs = [
        {'power': 200, 'gear_ratio': 30, 'efficiency': 0.85},
        {'power': 500, 'gear_ratio': 50, 'efficiency': 0.85},
        {'power': 1000, 'gear_ratio': 100, 'efficiency': 0.90},
    ]
    
    # 创建基准电机
    motor = customizer.create_motor("base_motor")
    
    # 对比
    results = customizer.compare_configurations("base_motor", configs)
    
    print("\n配置对比结果:")
    print("-"*70)
    print(f"{'配置':<8} {'功率':<10} {'减速比':<10} {'扭矩':<12} {'成本':<10}")
    print("-"*70)
    
    for result in results:
        config = result['params']
        metrics = result['metrics']
        print(f"#{result['config_id']:<7} "
              f"{config['power']}W{'':<5} "
              f"{config['gear_ratio']}:1{'':<6} "
              f"{metrics['output_torque_nm']:.2f} Nm{'':<4} "
              f"${metrics['estimated_cost_usd']:.2f}")


def demo_parameter_sweep():
    """示例 4: 参数扫描"""
    print("\n" + "="*70)
    print("示例 4: 参数扫描和可视化")
    print("="*70)
    
    customizer = PartCustomizer()
    
    print("\n扫描电机减速比 (20 到 120)...")
    customizer.visualize_parameter_sweep(
        part_type='motor',
        param_name='gear_ratio',
        param_range=(20, 120),
        steps=20
    )
    
    print("\n扫描关节刚度 (1000 到 10000 Nm/rad)...")
    customizer.visualize_parameter_sweep(
        part_type='joint',
        param_name='stiffness',
        param_range=(1000, 10000),
        steps=20
    )


def main():
    print("\n" + "="*70)
    print("定制化零件系统 - 完整示例")
    print("="*70)
    print("\n本示例将展示:")
    print("  1. 创建和定制电机")
    print("  2. 创建和定制关节装置")
    print("  3. 对比不同配置")
    print("  4. 参数扫描可视化")
    
    input(

"\n按回车键开始...")
    
    # 运行示例
    demo_custom_motor()
    demo_custom_joint()
    demo_configuration_comparison()
    
    try:
        demo_parameter_sweep()
    except ImportError:
        print("\n注意: matplotlib 未安装，跳过可视化示例")
        print("安装方法: pip install matplotlib")
    
    print("\n" + "="*70)
    print("示例完成！")
    print("="*70)
    print("\n关键发现:")
    print("  ✓ 增加减速比可以提升扭矩，但会降低速度")
    print("  ✓ 减少回差可以提高精度，但会增加成本")
    print("  ✓ 增加功率可以提升性能，但增加热量和成本")
    print("  ✓ 刚度影响动态响应和控制带宽")
    
    print("\n下一步:")
    print("  1. 使用这些定制零件创建机器人")
    print("  2. 在仿真中测试性能")
    print("  3. 根据结果优化配置")


if __name__ == "__main__":
    main()
