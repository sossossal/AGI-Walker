"""
参数化控制完整示例
展示如何通过调整物理参数来控制机器人
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.parametric_control import (
    ParametricRobotController,
    InteractiveParameterTuner
)
import numpy as np


def demo_basic_control():
    """示例 1: 基础参数控制"""
    print("="*70)
    print("示例 1: 基础参数化控制")
    print("="*70)
    
    # 创建控制器
    controller = ParametricRobotController()
    
    print("\n初始配置:")
    for param, value in controller.physics_params.items():
        print(f"  {param}: {value:.3f}")
    
    # 测试默认配置
    print("\n测试默认配置...")
    result = controller.run_episode(max_steps=500, render=False)
    default_reward = result['total_reward']
    print(f"  默认奖励: {default_reward:.2f}")
    print(f"  存活步数: {result['steps']}")
    
    # 实验 1: 增加电机功率
    print("\n" + "-"*70)
    print("实验 1: 增加电机功率 1.0 → 1.5")
    print("-"*70)
    
    change = controller.set_physics_param('motor_power_multiplier', 1.5)
    print(f"预期影响: {change['impact']}")
    
    result = controller.run_episode(max_steps=500, render=False)
    print(f"\n实际结果:")
    print(f"  奖励: {result['total_reward']:.2f} (变化: {result['total_reward'] - default_reward:+.2f})")
    print(f"  步数: {result['steps']}")
    
    # 实验 2: 调整关节刚度
    print("\n" + "-"*70)
    print("实验 2: 增加关节刚度 1.0 → 2.0")
    print("-"*70)
    
    controller.set_physics_param('motor_power_multiplier', 1.0)  # 重置
    change = controller.set_physics_param('joint_stiffness', 2.0)
    print(f"预期影响: {change['impact']}")
    
    result = controller.run_episode(max_steps=500, render=False)
    print(f"\n实际结果:")
    print(f"  奖励: {result['total_reward']:.2f} (变化: {result['total_reward'] - default_reward:+.2f})")
    print(f"  步数: {result['steps']}")


def demo_parameter_sweep():
    """示例 2: 参数扫描"""
    print("\n" + "="*70)
    print("示例 2: 电机功率参数扫描")
    print("="*70)
    
    controller = ParametricRobotController()
    
    # 扫描电机功率
    power_values = np.linspace(0.5, 2.0, 7)
    results = []
    
    print("\n扫描电机功率倍数 (0.5 到 2.0):")
    print("-"*70)
    print(f"{'功率倍数':<12} {'奖励':<12} {'步数':<8} {'评价':<15}")
    print("-"*70)
    
    for power in power_values:
        controller.set_physics_param('motor_power_multiplier', power)
        result = controller.run_episode(max_steps=500, render=False)
        results.append((power, result['total_reward'], result['steps']))
        
        rating = "优秀" if result['total_reward'] > 200 else ("良好" if result['total_reward'] > 100 else "一般")
        print(f"{power:<12.2f} {result['total_reward']:<12.2f} {result['steps']:<8} {rating:<15}")
    
    # 找到最优值
    best_idx = max(range(len(results)), key=lambda i: results[i][1])
    best_power, best_reward, best_steps = results[best_idx]
    
    print("-"*70)
    print(f"最优配置: 功率倍数 = {best_power:.2f}, 奖励 = {best_reward:.2f}")


def demo_auto_optimization():
    """示例 3: 自动参数优化"""
    print("\n" + "="*70)
    print("示例 3: 自动参数优化")
    print("="*70)
    
    controller = ParametricRobotController()
    
    # 定义搜索空间
    param_ranges = {
        'motor_power_multiplier': (0.8, 1.5),
        'joint_stiffness': (0.5, 2.0),
        'joint_damping': (0.3, 0.8)
    }
    
    # 搜索最优参数
    result = controller.find_optimal_params(param_ranges, n_trials=15)
    
    print("\n优化结果:")
    print("-"*70)
    print("最优参数:")
    for param, value in result['best_params'].items():
        print(f"  {param}: {value:.3f}")
    print(f"\n最高奖励: {result['best_reward']:.2f}")
    
    # 对比默认配置
    controller_default = ParametricRobotController()
    default_result = controller_default.run_episode(max_steps=500, render=False)
    
    print(f"\n性能提升:")
    improvement = (result['best_reward'] - default_result['total_reward']) / default_result['total_reward'] * 100
    print(f"  默认配置: {default_result['total_reward']:.2f}")
    print(f"  优化配置: {result['best_reward']:.2f}")
    print(f"  提升: {improvement:+.1f}%")


def demo_interactive_mode():
    """示例 4: 交互式调整"""
    print("\n" + "="*70)
    print("示例 4: 交互式参数调整")
    print("="*70)
    
    controller = ParametricRobotController()
    tuner = InteractiveParameterTuner(controller)
    
    print("\n你可以实时调整参数并立即看到效果！")
    print("建议尝试:")
    print("  1. set motor_power_multiplier 1.2")
    print("  2. test")
    print("  3. set joint_damping 0.7")
    print("  4. test")
    
    tuner.interactive_tuning()


def demo_comparison():
    """示例 5: 配置对比"""
    print("\n" + "="*70)
    print("示例 5: 不同配置对比")
    print("="*70)
    
    configurations = [
        {
            'name': '默认配置',
            'params': {}
        },
        {
            'name': '高功率配置',
            'params': {'motor_power_multiplier': 1.5}
        },
        {
            'name': '高刚度配置',
            'params': {'joint_stiffness': 2.0, 'joint_damping': 0.7}
        },
        {
            'name': '轻量化配置',
            'params': {'mass_multiplier': 0.8, 'motor_power_multiplier': 0.9}
        }
    ]
    
    print("\n对比4种配置:")
    print("-"*70)
    print(f"{'配置':<20} {'奖励':<12} {'步数':<8} {'成功率':<10}")
    print("-"*70)
    
    for config in configurations:
        controller = ParametricRobotController()
        
        # 应用配置
        for param, value in config['params'].items():
            controller.set_physics_param(param, value)
        
        # 测试多次
        rewards = []
        for _ in range(3):
            result = controller.run_episode(max_steps=500, render=False)
            rewards.append(result['total_reward'])
        
        avg_reward = np.mean(rewards)
        avg_steps = int(np.mean([controller.run_episode(max_steps=500)['steps'] for _ in range(3)]))
        success_rate = sum(1 for r in rewards if r > 150) / len(rewards)
        
        print(f"{config['name']:<20} {avg_reward:<12.2f} {avg_steps:<8} {success_rate*100:<10.0f}%")
    
    print("-"*70)


def main():
    print("\n" + "="*70)
    print("参数化机器人控制系统 - 完整演示")
    print("="*70)
    
    print("\n本演示包含:")
    print("  1. 基础参数控制")
    print("  2. 参数扫描分析")
    print("  3. 自动参数优化")
    print("  4. 交互式调整")
    print("  5. 配置对比")
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', type=int, default=0, 
                        help='运行指定示例 (1-5), 0=全部')
    parser.add_argument('--interactive', action='store_true',
                        help='直接进入交互模式')
    
    args = parser.parse_args()
    
    if args.interactive:
        controller = ParametricRobotController()
        tuner = InteractiveParameterTuner(controller)
        tuner.interactive_tuning()
        return
    
    demos = [
        demo_basic_control,
        demo_parameter_sweep,
        demo_auto_optimization,
        demo_comparison,
        demo_interactive_mode
    ]
    
    if args.demo == 0:
        # 运行前4个示例（跳过交互式）
        for i, demo in enumerate(demos[:-1], 1):
            demo()
            if i < len(demos) - 1:
                input("\n按回车继续下一个示例...")
    else:
        demos[args.demo - 1]()
    
    print("\n" + "="*70)
    print("演示完成！")
    print("="*70)
    print("\n关键发现:")
    print("  ✓ 电机功率影响最大驱动力")
    print("  ✓ 关节刚度影响精度和震荡")
    print("  ✓ 阻尼影响稳定性")
    print("  ✓ 参数优化可提升性能20-50%")


if __name__ == "__main__":
    main()
