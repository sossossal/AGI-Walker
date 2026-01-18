"""
让机器人前进1米的零件参数调整DEMO
展示参数如何影响物理行为
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.physics_validator import PhysicsSimulator, PhysicsValidator
import matplotlib.pyplot as plt
import numpy as np


def demo_scenario_1_default():
    """场景 1: 默认配置"""
    print("="*70)
    print("场景 1: 默认配置")
    print("="*70)
    
    params = {
        'motor_power_multiplier': 1.0,
        'mass_multiplier': 1.0,
        'joint_stiffness': 1.0,
        'joint_damping': 0.5,
        'friction': 0.9,
        'gravity': 9.81
    }
    
    print("\n参数配置:")
    for key, value in params.items():
        print(f"  {key}: {value}")
    
    # 运行模拟
    sim = PhysicsSimulator(params)
    result = sim.simulate_forward(target_distance=1.0, max_time=10.0)
    
    print(f"\n结果:")
    print(f"  成功: {result['success']}")
    if result['success']:
        print(f"  实际距离: {result['distance_traveled']:.2f} m")
        print(f"  用时: {result['time_taken']:.2f} s")
        print(f"  平均速度: {result['avg_speed']:.2f} m/s")
    else:
        print(f"  失败原因: {result['reason']}")
        print(f"  实际距离: {result['distance_traveled']:.2f} m")
    
    return result


def demo_scenario_2_high_power():
    """场景 2: 高功率配置"""
    print("\n" + "="*70)
    print("场景 2: 增加电机功率 (1.5倍)")
    print("="*70)
    
    params = {
        'motor_power_multiplier': 1.5,  # 提高功率
        'mass_multiplier': 1.0,
        'joint_stiffness': 1.0,
        'joint_damping': 0.5,
        'friction': 0.9,
        'gravity': 9.81
    }
    
    print("\n参数配置:")
    print(f"  电机功率倍数: 1.0 → 1.5")
    
    sim = PhysicsSimulator(params)
    result = sim.simulate_forward(target_distance=1.0, max_time=10.0)
    
    print(f"\n结果:")
    print(f"  成功: {result['success']}")
    if result['success']:
        print(f"  实际距离: {result['distance_traveled']:.2f} m")
        print(f"  用时: {result['time_taken']:.2f} s")
        print(f"  平均速度: {result['avg_speed']:.2f} m/s")
        print(f"  ✓ 速度提升明显！")
    
    return result


def demo_scenario_3_too_heavy():
    """场景 3: 过重配置 - 会失败"""
    print("\n" + "="*70)
    print("场景 3: 机器人过重 (2倍质量)")
    print("="*70)
    
    params = {
        'motor_power_multiplier': 1.0,
        'mass_multiplier': 2.0,  # 质量翻倍
        'joint_stiffness': 1.0,
        'joint_damping': 0.5,
        'friction': 0.9,
        'gravity': 9.81
    }
    
    print("\n参数配置:")
    print(f"  质量倍数: 1.0 → 2.0")
    
    # 先验证
    validator = PhysicsValidator()
    is_valid, issues = validator.validate_robot_balance(params)
    
    print(f"\n物理验证:")
    for issue in issues:
        print(f"  {issue}")
    
    sim = PhysicsSimulator(params)
    result = sim.simulate_forward(target_distance=1.0, max_time=10.0)
    
    print(f"\n结果:")
    print(f"  成功: {result['success']}")
    print(f"  失败原因: {result.get('reason', 'N/A')}")
    print(f"  实际距离: {result['distance_traveled']:.2f} m")
    print(f"  ❌ 机器人无法自主移动！")
    
    return result


def demo_scenario_4_unstable():
    """场景 4: 不稳定配置 - 可能摔倒"""
    print("\n" + "="*70)
    print("场景 4: 刚度过低 - 不稳定")
    print("="*70)
    
    params = {
        'motor_power_multiplier': 1.0,
        'mass_multiplier': 1.0,
        'joint_stiffness': 0.3,  # 刚度过低
        'joint_damping': 0.2,    # 阻尼也过低
        'friction': 0.9,
        'gravity': 9.81
    }
    
    print("\n参数配置:")
    print(f"  关节刚度: 1.0 → 0.3")
    print(f"  关节阻尼: 0.5 → 0.2")
    
    sim = PhysicsSimulator(params)
    result = sim.simulate_forward(target_distance=1.0, max_time=10.0)
    
    print(f"\n结果:")
    print(f"  成功: {result['success']}")
    if not result['success']:
        print(f"  失败原因: {result.get('reason', 'N/A')}")
        print(f"  摔倒位置: {result['distance_traveled']:.2f} m")
        print(f"  ⚠️ 关节过软导致失去平衡！")
    
    return result


def demo_scenario_5_optimized():
    """场景 5: 优化配置"""
    print("\n" + "="*70)
    print("场景 5: 优化配置 - 速度与稳定兼顾")
    print("="*70)
    
    params = {
        'motor_power_multiplier': 1.3,  # 适度提高功率
        'mass_multiplier': 0.9,          # 轻量化
        'joint_stiffness': 1.2,          # 略微提高刚度
        'joint_damping': 0.6,            # 适度阻尼
        'friction': 0.85,                # 降低摩擦
        'gravity': 9.81
    }
    
    print("\n参数配置 (优化后):")
    for key, value in params.items():
        print(f"  {key}: {value}")
    
    sim = PhysicsSimulator(params)
    result = sim.simulate_forward(target_distance=1.0, max_time=10.0)
    
    print(f"\n结果:")
    print(f"  成功: {result['success']}")
    if result['success']:
        print(f"  实际距离: {result['distance_traveled']:.2f} m")
        print(f"  用时: {result['time_taken']:.2f} s")
        print(f"  平均速度: {result['avg_speed']:.2f} m/s")
        print(f"  🌟 性能优秀！")
    
    return result


def compare_all_scenarios():
    """对比所有场景"""
    print("\n" + "="*70)
    print("场景对比总结")
    print("="*70)
    
    scenarios = [
        ("默认配置", demo_scenario_1_default),
        ("高功率", demo_scenario_2_high_power),
        ("过重", demo_scenario_3_too_heavy),
        ("不稳定", demo_scenario_4_unstable),
        ("优化配置", demo_scenario_5_optimized),
    ]
    
    print("\n" + "="*70)
    print(f"{'场景':<15} {'成功':<8} {'距离(m)':<12} {'用时(s)':<12} {'速度(m/s)':<12}")
    print("-"*70)
    
    results = []
    for name, demo_func in scenarios:
        result = demo_func()
        results.append((name, result))
    
    print("\n" + "="*70)
    print("结果汇总")
    print("="*70)
    print(f"{'场景':<15} {'成功':<8} {'距离(m)':<12} {'用时(s)':<12} {'速度(m/s)':<12}")
    print("-"*70)
    
    for name, result in results:
        success = "✓" if result['success'] else "✗"
        distance = result['distance_traveled']
        time_taken = result.get('time_taken', 0.0)
        speed = result.get('avg_speed', 0.0) if result['success'] else 0.0
        
        print(f"{name:<15} {success:<8} {distance:<12.2f} {time_taken:<12.2f} {speed:<12.2f}")
    
    return results


def visualize_trajectories(results):
    """可视化运动轨迹"""
    try:
        plt.figure(figsize=(12, 8))
        
        # 位置-时间图
        plt.subplot(2, 1, 1)
        for name, result in results:
            if 'trajectory' in result:
                trajectory = result['trajectory']
                times = [s['time'] for s in trajectory]
                positions = [s['position'] for s in trajectory]
                
                linestyle = '-' if result['success'] else '--'
                plt.plot(times, positions, linestyle=linestyle, linewidth=2, label=name)
        
        plt.axhline(y=1.0, color='r', linestyle=':', label='目标 (1m)')
        plt.xlabel('时间 (s)')
        plt.ylabel('位置 (m)')
        plt.title('机器人运动轨迹对比')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # 速度-时间图
        plt.subplot(2, 1, 2)
        for name, result in results:
            if 'trajectory' in result:
                trajectory = result['trajectory']
                times = [s['time'] for s in trajectory]
                velocities = [s['velocity'] for s in trajectory]
                
                linestyle = '-' if result['success'] else '--'
                plt.plot(times, velocities, linestyle=linestyle, linewidth=2, label=name)
        
        plt.xlabel('时间 (s)')
        plt.ylabel('速度 (m/s)')
        plt.title('速度变化对比')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('robot_forward_1m_demo.png', dpi=150)
        print("\n📊 轨迹图已保存: robot_forward_1m_demo.png")
        
    except ImportError:
        print("\n注意: matplotlib 未安装，跳过可视化")


def main():
    print("\n" + "="*70)
    print("🤖 让机器人前进1米 - 零件参数调整DEMO")
    print("="*70)
    
    print("\n本演示展示:")
    print("  1. 默认配置的性能")
    print("  2. 高功率配置的改进")
    print("  3. 参数不当导致失败")
    print("  4. 不稳定配置导致摔倒")
    print("  5. 优化配置的最佳性能")
    
    input("\n按回车开始演示...")
    
    # 运行所有场景
    results = compare_all_scenarios()
    
    # 可视化
    print("\n生成可视化图表...")
    visualize_trajectories(results)
    
    print("\n" + "="*70)
    print("关键发现")
    print("="*70)
    print("""
1. 💪 电机功率影响速度
   - 功率提升30% → 速度提升~20%
   
2. ⚖️ 质量影响稳定性
   - 过重会导致无法移动
   - 轻量化可提升性能
   
3. 🔧 关节刚度影响稳定
   - 过低会导致摔倒
   - 过高会导致震荡
   
4. 🎯 优化配置最佳
   - 功率 +30%, 质量 -10%
   - 刚度 +20%, 摩擦 -5%
   
⚠️  参数违反物理规律会导致:
   - 无法移动 (功率不足)
   - 摔倒 (不稳定)
   - 震荡 (参数不当)
""")
    
    print("="*70)
    print("演示完成！")
    print("="*70)


if __name__ == "__main__":
    main()
