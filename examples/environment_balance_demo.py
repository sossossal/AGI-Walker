"""
环境平衡系统演示
测试机器人在不同环境条件下的平衡能力
"""

import sys
import os
from typing import Dict
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.environment_balance import (
    EnvironmentBalanceSimulator,
    Environments,
    EnvironmentCondition
)


def test_environment(env: EnvironmentCondition, robot_params: Dict = None):
    """测试特定环境"""
    if robot_params is None:
        robot_params = {
            'motor_power_multiplier': 1.0,
            'mass_multiplier': 1.0,
            'joint_stiffness': 1.0,
            'joint_damping': 0.5,
            'friction': 0.9,
            'gravity': 9.81
        }
    
    sim = EnvironmentBalanceSimulator(robot_params, env)
    result = sim.simulate_forward(target_distance=1.0, max_time=10.0)
    
    return result


def demo_different_environments():
    """演示不同环境"""
    print("="*70)
    print("环境平衡系统演示")
    print("="*70)
    
    environments = [
        ("平地", Environments.flat_ground()),
        ("5度上坡", Environments.uphill_5deg()),
        ("5度下坡", Environments.downhill_5deg()),
        ("10度陡坡", Environments.uphill_10deg()),
        ("大风环境", Environments.windy()),
        ("冰面", Environments.icy_ground()),
        ("崎岖地形", Environments.rough_terrain()),
        ("极端条件", Environments.extreme_conditions()),
    ]
    
    print("\n测试: 标准配置在不同环境下的表现")
    print("-"*70)
    print(f"{'环境':<15} {'成功':<8} {'距离(m)':<12} {'最大倾斜':<12} {'评价':<20}")
    print("-"*70)
    
    for env_name, env in environments:
        result = test_environment(env)
        
        success = "✓" if result['success'] else "✗"
        distance = result['distance_traveled']
        
        # 获取最大倾斜角
        if 'trajectory' in result:
            max_tilt = max(abs(s.get('tilt_angle', 0)) for s in result['trajectory'])
        else:
            max_tilt = 0
        
        if result['success']:
            评价 = "通过"
        elif 'reason' in result and '倾斜' in result['reason']:
            评价 = "倾倒"
        elif distance < 0.3:
            评价 = "无法前进"
        else:
            评价 = result.get('reason', '失败')[:15]
        
        print(f"{env_name:<15} {success:<8} {distance:<12.2f} {max_tilt:<12.1f}° {评价:<20}")


def demo_parameter_tuning_for_slopes():
    """演示针对坡度的参数调整"""
    print("\n" + "="*70)
    print("坡度适应 - 参数调整策略")
    print("="*70)
    
    slope_env = Environments.uphill_10deg()
    
    configs = [
        ("标准配置", {
            'motor_power_multiplier': 1.0,
            'joint_stiffness': 1.0,
            'joint_damping': 0.5
        }),
        ("增加功率", {
            'motor_power_multiplier': 1.5,
            'joint_stiffness': 1.0,
            'joint_damping': 0.5
        }),
        ("增强刚度", {
            'motor_power_multiplier': 1.0,
            'joint_stiffness': 1.5,
            'joint_damping': 0.6
        }),
        ("综合优化", {
            'motor_power_multiplier': 1.4,
            'joint_stiffness': 1.3,
            'joint_damping': 0.7
        })
    ]
    
    print(f"\n测试环境: {slope_env}")
    print("-"*70)
    print(f"{'配置':<15} {'功率':<10} {'刚度':<10} {'成功':<8} {'距离(m)':<12} {'评价':<15}")
    print("-"*70)
    
    for config_name, params in configs:
        full_params = {
            'motor_power_multiplier': params['motor_power_multiplier'],
            'mass_multiplier': 1.0,
            'joint_stiffness': params['joint_stiffness'],
            'joint_damping': params['joint_damping'],
            'friction': 0.9,
            'gravity': 9.81
        }
        
        result = test_environment(slope_env, full_params)
        
        success = "✓" if result['success'] else "✗"
        distance = result['distance_traveled']
        
        if result['success']:
            评价 = "成功爬坡"
        else:
            评价 = "失败"
        
        print(f"{config_name:<15} {params['motor_power_multiplier']:<10.1f} "
              f"{params['joint_stiffness']:<10.1f} {success:<8} "
              f"{distance:<12.2f} {评价:<15}")


def demo_balance_control():
    """演示平衡控制效果"""
    print("\n" + "="*70)
    print("平衡控制系统效果")
    print("="*70)
    
    # 大风环境
    windy_env = Environments.windy()
    
    print(f"\n测试环境: {windy_env}")
    print("对比: 不同刚度和阻尼对平衡的影响")
    print("-"*70)
    print(f"{'配置':<20} {'刚度':<10} {'阻尼':<10} {'最大倾斜':<12} {'成功':<8}")
    print("-"*70)
    
    balance_configs = [
        ("低刚度低阻尼", 0.5, 0.3),
        ("标准配置", 1.0, 0.5),
        ("高刚度高阻尼", 1.5, 0.7),
        ("超高刚度", 2.0, 0.8)
    ]
    
    for config_name, stiffness, damping in balance_configs:
        params = {
            'motor_power_multiplier': 1.0,
            'mass_multiplier': 1.0,
            'joint_stiffness': stiffness,
            'joint_damping': damping,
            'friction': 0.9,
            'gravity': 9.81
        }
        
        result = test_environment(windy_env, params)
        
        if 'trajectory' in result:
            max_tilt = max(abs(s.get('tilt_angle', 0)) for s in result['trajectory'])
        else:
            max_tilt = 0
        
        success = "✓" if result['success'] else "✗"
        
        print(f"{config_name:<20} {stiffness:<10.1f} {damping:<10.1f} "
              f"{max_tilt:<12.1f}° {success:<8}")


def demo_custom_environment():
    """演示自定义环境"""
    print("\n" + "="*70)
    print("自定义环境")
    print("="*70)
    
    # 创建自定义环境
    custom_env = EnvironmentCondition("火星模拟")
    custom_env.friction_coef = 0.8  # 沙地
    custom_env.wind_force = 1.5     # 沙尘暴
    custom_env.disturbance = 1.0    # 地形不平
    
    print(f"\n自定义环境: {custom_env}")
    
    # 测试机器人适应性
    params = {
        'motor_power_multiplier': 1.2,
        'mass_multiplier': 0.9,  # 轻量化
        'joint_stiffness': 1.3,
        'joint_damping': 0.6,
        'friction': 0.9,
        'gravity': 3.71  # 火星重力
    }
    
    result = test_environment(custom_env, params)
    
    print(f"\n结果:")
    print(f"  成功: {result['success']}")
    print(f"  距离: {result['distance_traveled']:.2f} m")
    if result['success']:
        print(f"  用时: {result['time_taken']:.2f} s")
        print(f"  平均速度: {result['avg_speed']:.2f} m/s")


def main():
    """主演示"""
    print("\n" + "="*70)
    print("环境平衡系统 - 完整演示")
    print("="*70)
    
    print("\n本演示包含:")
    print("  1. 不同环境测试")
    print("  2. 坡度适应参数调整")
    print("  3. 平衡控制效果")
    print("  4. 自定义环境")
    
    input("\n按回车开始演示...")
    
    demo_different_environments()
    input("\n按回车继续...")
    
    demo_parameter_tuning_for_slopes()
    input("\n按回车继续...")
    
    demo_balance_control()
    input("\n按回车继续...")
    
    demo_custom_environment()
    
    print("\n" + "="*70)
    print("演示完成！")
    print("="*70)
    print("\n关键发现:")
    print("  ✓ 环境因素显著影响机器人性能")
    print("  ✓ 参数调整可以适应不同环境")
    print("  ✓ 平衡控制系统可抵抗扰动")
    print("  ✓ 支持自定义复杂环境")


if __name__ == "__main__":
    main()
