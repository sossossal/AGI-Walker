"""
自动验证定制化零件系统
测试所有参数调整和反馈功能
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.custom_parts import (
    CustomMotor, CustomJoint, CustomSensor, PartCustomizer
)


def test_motor_customization():
    """测试电机定制功能"""
    print("="*70)
    print("测试 1: 电机参数调整")
    print("="*70)
    
    # 创建电机
    motor = CustomMotor({
        'power': 500,
        'voltage': 24,
        'gear_ratio': 50,
        'efficiency': 0.85,
        'weight': 0.8
    })
    
    print("\n✓ 初始配置创建成功")
    print(f"  功率: 500W, 减速比: 50:1")
    
    # 获取初始性能
    initial_torque = motor.performance_metrics['output_torque_nm']
    initial_cost = motor.performance_metrics['estimated_cost_usd']
    
    print(f"  初始扭矩: {initial_torque:.2f} Nm")
    print(f"  初始成本: ${initial_cost:.2f}")
    
    # 测试减速比变化
    print("\n测试: 减速比 50 → 100")
    result = motor.update_param('gear_ratio', 100)
    
    new_torque = motor.performance_metrics['output_torque_nm']
    new_cost = motor.performance_metrics['estimated_cost_usd']
    
    torque_change = (new_torque - initial_torque) / initial_torque * 100
    cost_change = new_cost - initial_cost
    
    print(f"  新扭矩: {new_torque:.2f} Nm (变化: +{torque_change:.1f}%)")
    print(f"  新成本: ${new_cost:.2f} (变化: +${cost_change:.2f})")
    print(f"  影响反馈: {result['impact']}")
    
    # 验证
    assert new_torque > initial_torque, "❌ 扭矩应该增加"
    assert new_cost > initial_cost, "❌ 成本应该增加"
    print("\n✓ 电机参数调整测试通过")
    
    return True


def test_joint_customization():
    """测试关节定制功能"""
    print("\n" + "="*70)
    print("测试 2: 关节参数调整")
    print("="*70)
    
    # 创建关节
    joint = CustomJoint({
        'type': 'harmonic_drive',
        'reduction_ratio': 100,
        'max_torque': 50,
        'backlash': 0.05,
        'efficiency': 0.90,
        'weight': 0.3,
        'stiffness': 5000
    })
    
    print("\n✓ 谐波减速器创建成功")
    print(f"  减速比: 100:1, 回差: 0.05°")
    
    # 获取初始性能
    initial_accuracy = joint.performance_metrics['positioning_accuracy_deg']
    initial_cost = joint.performance_metrics['estimated_cost_usd']
    
    print(f"  初始定位精度: {initial_accuracy:.6f}°")
    print(f"  初始成本: ${initial_cost:.2f}")
    
    # 测试回差减少（提高精度）
    print("\n测试: 回差 0.05° → 0.01° (精度提升)")
    result = joint.update_param('backlash', 0.01)
    
    new_accuracy = joint.performance_metrics['positioning_accuracy_deg']
    new_cost = joint.performance_metrics['estimated_cost_usd']
    
    accuracy_improvement = (initial_accuracy - new_accuracy) / initial_accuracy * 100
    cost_increase = new_cost - initial_cost
    
    print(f"  新定位精度: {new_accuracy:.6f}° (提升: {accuracy_improvement:.1f}%)")
    print(f"  新成本: ${new_cost:.2f} (增加: +${cost_increase:.2f})")
    print(f"  影响反馈: {result['impact']}")
    
    # 验证
    assert new_accuracy < initial_accuracy, "❌ 精度应该提高（值应该减小）"
    assert new_cost > initial_cost, "❌ 成本应该增加"
    print("\n✓ 关节参数调整测试通过")
    
    return True


def test_stiffness_impact():
    """测试刚度对性能的影响"""
    print("\n" + "="*70)
    print("测试 3: 刚度对动态性能的影响")
    print("="*70)
    
    joint = CustomJoint({
        'stiffness': 3000,
        'weight': 0.3
    })
    
    initial_bandwidth = joint.performance_metrics['control_bandwidth_hz']
    print(f"\n初始刚度: 3000 Nm/rad")
    print(f"初始控制带宽: {initial_bandwidth:.2f} Hz")
    
    # 增加刚度
    print("\n测试: 刚度 3000 → 8000 Nm/rad")
    joint.update_param('stiffness', 8000)
    
    new_bandwidth = joint.performance_metrics['control_bandwidth_hz']
    bandwidth_increase = (new_bandwidth - initial_bandwidth) / initial_bandwidth * 100
    
    print(f"新控制带宽: {new_bandwidth:.2f} Hz (提升: +{bandwidth_increase:.1f}%)")
    
    # 验证
    assert new_bandwidth > initial_bandwidth, "❌ 控制带宽应该增加"
    print("\n✓ 刚度影响测试通过")
    
    return True


def test_configuration_comparison():
    """测试配置对比功能"""
    print("\n" + "="*70)
    print("测试 4: 配置对比")
    print("="*70)
    
    customizer = PartCustomizer()
    motor = customizer.create_motor("test_motor")
    
    configs = [
        {'power': 200, 'gear_ratio': 30},
        {'power': 500, 'gear_ratio': 50},
        {'power': 1000, 'gear_ratio': 100},
    ]
    
    print(f"\n对比 {len(configs)} 种配置...")
    results = customizer.compare_configurations("test_motor", configs)
    
    print("\n配置对比结果:")
    print("-"*70)
    print(f"{'配置':<8} {'功率':<12} {'减速比':<12} {'扭矩':<15} {'成本':<12}")
    print("-"*70)
    
    for result in results:
        config = result['params']
        metrics = result['metrics']
        print(f"#{result['config_id']:<7} "
              f"{config['power']:>4}W{'':<7} "
              f"{config['gear_ratio']:>3}:1{'':<8} "
              f"{metrics['output_torque_nm']:>6.2f} Nm{'':<6} "
              f"${metrics['estimated_cost_usd']:>7.2f}")
    
    # 验证结果数量
    assert len(results) == 3, "❌ 应该有3个配置结果"
    
    # 验证扭矩递增
    torques = [r['metrics']['output_torque_nm'] for r in results]
    assert torques[0] < torques[1] < torques[2], "❌ 扭矩应该递增"
    
    print("\n✓ 配置对比测试通过")
    
    return True


def test_performance_metrics():
    """测试性能指标计算"""
    print("\n" + "="*70)
    print("测试 5: 性能指标计算")
    print("="*70)
    
    motor = CustomMotor({
        'power': 500,
        'gear_ratio': 50,
        'weight': 0.8
    })
    
    metrics = motor.performance_metrics
    
    print("\n计算的性能指标:")
    required_metrics = [
        'output_torque_nm',
        'output_speed_rpm',
        'current_draw_a',
        'heat_generation_w',
        'torque_to_weight_ratio',
        'estimated_cost_usd'
    ]
    
    for metric in required_metrics:
        value = metrics.get(metric)
        assert value is not None, f"❌ 缺少指标: {metric}"
        assert value > 0, f"❌ 指标值无效: {metric} = {value}"
        print(f"  ✓ {metric}: {value:.3f}")
    
    print("\n✓ 性能指标计算测试通过")
    
    return True


def run_all_tests():
    """运行所有测试"""
    print("\n" + "="*70)
    print("定制化零件系统自动验证")
    print("="*70)
    
    tests = [
        ("电机参数调整", test_motor_customization),
        ("关节参数调整", test_joint_customization),
        ("刚度影响分析", test_stiffness_impact),
        ("配置对比", test_configuration_comparison),
        ("性能指标计算", test_performance_metrics),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            success = test_func()
            results.append((test_name, success, None))
        except Exception as e:
            results.append((test_name, False, str(e)))
            print(f"\n❌ 测试失败: {e}")
    
    # 总结
    print("\n" + "="*70)
    print("验证总结")
    print("="*70)
    
    passed = sum(1 for _, success, _ in results if success)
    total = len(results)
    
    for test_name, success, error in results:
        status = "✓ 通过" if success else "✗ 失败"
        print(f"{test_name}: {status}")
        if error:
            print(f"  错误: {error}")
    
    print(f"\n总计: {passed}/{total} 测试通过")
    
    if passed == total:
        print("\n🎉 所有测试通过！定制化零件系统工作正常。")
        return True
    else:
        print(f"\n⚠️  {total - passed} 个测试失败，需要检查。")
        return False


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
