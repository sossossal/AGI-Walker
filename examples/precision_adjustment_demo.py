"""
精确参数调节演示
展示0.1精度的参数控制
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.precision_adjuster import PrecisionPartAdjuster


def demo_motor_precision():
    """演示电机精确调节"""
    print("="*70)
    print("演示: 电机功率精确调节 (0.1W 精度)")
    print("="*70)
    
    adjuster = PrecisionPartAdjuster('motor')
    
    # 显示初始状态
    state = adjuster.get_current_state()
    print(f"\n初始功率: {state['parameters']['power']:.1f} W")
    
    # 精确调节功率
    test_values = [100.0, 250.3, 500.5, 749.9, 1000.0]
    
    print("\n精确功率调节测试:")
    print("-"*70)
    print(f"{'设定值(W)':<12} {'实际值(W)':<12} {'扭矩(Nm)':<15} {'成本($)':<12} {'验证':<10}")
    print("-"*70)
    
    for value in test_values:
        result = adjuster.set_parameter('power', value)
        
        if result['success']:
            actual_value = result['new_value']
            torque = result['performance']['output_torque_nm']
            cost = result['performance']['estimated_cost_usd']
            valid = "✓" if result['is_valid'] else "⚠️"
            
            print(f"{value:<12.1f} {actual_value:<12.1f} {torque:<15.3f} ${cost:<11.2f} {valid:<10}")
    
    print("\n功率范围扫描 (100W 到 1000W, 步长0.1W):")
    print(f"可调节步数: {int((1000.0 - 100.0) / 0.1)} 步")
    print(f"精度: ±0.1W")


def demo_joint_precision():
    """演示关节精确调节"""
    print("\n" + "="*70)
    print("演示: 关节刚度精确调节 (0.1 Nm/rad 精度)")
    print("="*70)
    
    adjuster = PrecisionPartAdjuster('joint')
    
    # 精确调节刚度
    test_values = [1000.0, 2500.5, 5000.0, 7500.3, 10000.0]
    
    print("\n精确刚度调节测试:")
    print("-"*70)
    print(f"{'设定值':<15} {'实际值':<15} {'控制带宽(Hz)':<20} {'验证':<10}")
    print("-"*70)
    
    for value in test_values:
        result = adjuster.set_parameter('stiffness', value)
        
        if result['success']:
            actual_value = result['new_value']
            bandwidth = result['performance']['control_bandwidth_hz']
            valid = "✓" if result['is_valid'] else "⚠️"
            
            print(f"{value:<15.1f} {actual_value:<15.1f} {bandwidth:<20.3f} {valid:<10}")


def demo_efficiency_precision():
    """演示效率精确调节"""
    print("\n" + "="*70)
    print("演示: 效率精确调节 (0.1% 精度)")
    print("="*70)
    
    adjuster = PrecisionPartAdjuster('motor')
    
    # 效率从60%到95%
    print("\n效率调节测试 (60.0% - 95.0%):")
    print("-"*70)
    print(f"{'效率(%)':<12} {'热量(W)':<15} {'扭矩':<15}")
    print("-"*70)
    
    for eff_pct in [60.0, 70.5, 80.0, 90.3, 95.0]:
        eff_value = eff_pct / 100.0  # 转换为0-1范围
        result = adjuster.set_parameter('efficiency', eff_value)
        
        if result['success']:
            heat = result['performance']['heat_generation_w']
            torque = result['performance']['output_torque_nm']
            
            print(f"{eff_pct:<12.1f} {heat:<15.3f} {torque:<15.3f}")


def demo_parameter_sweep():
    """演示参数扫描"""
    print("\n" + "="*70)
    print("演示: 减速比参数扫描")
    print("="*70)
    
    adjuster = PrecisionPartAdjuster('motor')
    
    print("\n扫描减速比 (20.0 到 120.0, 精度0.1):")
    results = adjuster.sweep_parameter('gear_ratio', steps=15)
    
    print("-"*70)
    print(f"{'减速比':<12} {'扭矩(Nm)':<15} {'转速(RPM)':<15}")
    print("-"*70)
    
    for value, metrics in results:
        torque = metrics['output_torque_nm']
        speed = metrics['output_speed_rpm']
        print(f"{value:<12.1f} {torque:<15.3f} {speed:<15.3f}")


def demo_validation():
    """演示物理验证"""
    print("\n" + "="*70)
    print("演示: 物理参数验证")
    print("="*70)
    
    adjuster = PrecisionPartAdjuster('motor')
    
    # 设置功率为500W
    adjuster.set_parameter('power', 500.0)
    
    print("\n测试不同质量配置:")
    print("-"*70)
    print(f"{'重量(kg)':<12} {'功率密度':<15} {'验证结果':<30}")
    print("-"*70)
    
    for weight in [0.5, 1.0, 2.0, 3.5, 5.0]:
        result = adjuster.set_parameter('weight', weight)
        
        if result['success']:
            power = result['performance'].get('output_torque_nm', 0) * 10  # 简化计算
            density = 500.0 / weight
            
            status = "✓ 正常" if result['is_valid'] else "⚠️ 有警告"
            if result['warnings']:
                status = result['warnings'][0][:25] + "..."
            
            print(f"{weight:<12.1f} {density:<15.0f} {status:<30}")


def main():
    """主演示"""
    print("\n" + "="*70)
    print("精确零件参数调节系统演示")
    print("精度: 0.1 (所有参数)")
    print("="*70)
    
    demos = [
        demo_motor_precision,
        demo_joint_precision,
        demo_efficiency_precision,
        demo_parameter_sweep,
        demo_validation
    ]
    
    for demo in demos:
        demo()
        input("\n按回车继续...")
    
    print("\n" + "="*70)
    print("演示完成！")
    print("="*70)
    print("\n关键特性:")
    print("  ✓ 所有参数精确到0.1")
    print("  ✓ 自动验证物理合理性")
    print("  ✓ 实时性能计算")
    print("  ✓ 参数范围限制")
    print("\n交互式调节:")
    print("  python python_api/precision_adjuster.py")


if __name__ == "__main__":
    main()
