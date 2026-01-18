"""
简化版参数化控制演示（不依赖特定环境）
"""

import numpy as np
from python_api.parametric_control import ParametricRobotController


class SimpleController:
    """简化的参数化控制器，用于演示"""
    
    def __init__(self):
        self.physics_params = {
            'motor_power_multiplier': 1.0,
            'joint_stiffness': 1.0,
            'joint_damping': 0.5,
            'friction': 0.9,
            'mass_multiplier': 1.0
        }
    
    def set_physics_param(self, param_name: str, value: float) -> Dict:
        """设置参数"""
        old_value = self.physics_params[param_name]
        self.physics_params[param_name] = value
        
        # 影响分析
        impact = {}
        change = (value - old_value) / old_value if old_value != 0 else 0
        
        if param_name == 'motor_power_multiplier':
            impact['max_torque'] = f"{change*100:+.1f}%"
            impact['expected'] = "更强驱动" if value > old_value else "更弱驱动"
        elif param_name == 'joint_stiffness':
            impact['precision'] = f"{change*50:+.1f}%"
            impact['expected'] = "更精确" if value > old_value else "更柔和"
        
        return {'parameter': param_name, 'old_value': old_value, 'new_value': value, 'impact': impact}
    
    def simulate(self) -> float:
        """模拟性能评分"""
        score = 100.0
        score *= self.physics_params['motor_power_multiplier'] ** 0.5
        score *= self.physics_params['joint_stiffness'] ** 0.3
        score *= (1 + self.physics_params['joint_damping']) ** 0.2
        
        # 添加随机性
        score *= (0.9 + np.random.random() * 0.2)
        
        return score


def main():
    print("="*70)
    print("参数化机器人控制系统 - 简化演示")
    print("="*70)
    
    controller = SimpleController()
    
    print("\n初始配置:")
    for param, value in controller.physics_params.items():
        print(f"  {param}: {value:.3f}")
    
    baseline_score = controller.simulate()
    print(f"\n基线性能评分: {baseline_score:.2f}")
    
    # 测试1: 增加电机功率
    print("\n" + "-"*70)
    print("测试 1: 增加电机功率")
    print("-"*70)
    result = controller.set_physics_param('motor_power_multiplier', 1.3)
    print(f"参数变化: {result['old_value']} → {result['new_value']}")
    print(f"影响: {result['impact']}")
    
    new_score = controller.simulate()
    print(f"新性能评分: {new_score:.2f} (变化: {new_score - baseline_score:+.2f})")
    
    # 测试2: 调整刚度
    print("\n" + "-"*70)
    print("测试 2: 增加关节刚度")
    print("-"*70)
    controller.set_physics_param('motor_power_multiplier', 1.0)  # 重置
    result = controller.set_physics_param('joint_stiffness', 1.5)
    print(f"参数变化: {result['old_value']} → {result['new_value']}")
    print(f"影响: {result['impact']}")
    
    new_score = controller.simulate()
    print(f"新性能评分: {new_score:.2f} (变化: {new_score - baseline_score:+.2f})")
    
    # 参数扫描
    print("\n" + "="*70)
    print("参数扫描: 电机功率倍数")
    print("="*70)
    
    print(f"\n{'功率倍数':<12} {'性能评分':<12} {'相对提升':<12}")
    print("-"*70)
    
    for power in [0.7, 0.9, 1.0, 1.2, 1.5, 1.8]:
        controller.set_physics_param('motor_power_multiplier', power)
        score = controller.simulate()
        improvement = (score - baseline_score) / baseline_score * 100
        print(f"{power:<12.1f} {score:<12.2f} {improvement:+.1f}%")
    
    print("\n" + "="*70)
    print("✓ 演示完成！")
    print("="*70)
    print("\n关键发现:")
    print("  • 电机功率直接影响性能")
    print("  • 关节刚度影响精度和稳定性")
    print("  • 参数优化可显著提升性能")
    print("\n完整功能需要安装 gymnasium 环境")
    print("安装方法: pip install gymnasium")


if __name__ == "__main__":
    main()
