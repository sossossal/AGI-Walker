"""
参数化机器人控制接口
通过调整零件参数来间接控制机器人行为
"""

import gymnasium as gym
import numpy as np
from typing import Dict, Optional, Tuple
from python_api.custom_parts import CustomMotor, CustomJoint


class ParametricRobotController:
    """
    参数化机器人控制器
    通过调整物理参数来影响机器人行为，而非直接发送动作命令
    """
    
    def __init__(self, env_id: str = 'AGI-Walker/Walker2D-v0'):
        """
        初始化参数化控制器
        
        参数:
            env_id: Gymnasium 环境ID
        """
        self.env_id = env_id
        self.env = gym.make(env_id)
        
        # 机器人零件配置
        self.joints = {}
        self.motors = {}
        
        # 当前物理参数
        self.physics_params = {
            'gravity': 9.81,           # 重力加速度
            'motor_power_multiplier': 1.0,   # 电机功率倍数
            'joint_stiffness': 1.0,    # 关节刚度倍数
            'joint_damping': 0.5,      # 关节阻尼
            'friction': 0.9,           # 摩擦系数
            'mass_multiplier': 1.0     # 质量倍数
        }
        
        # 控制模式
        self.control_mode = 'parametric'  # 或 'direct'
        
    def add_joint(self, joint_name: str, joint: CustomJoint):
        """添加关节"""
        self.joints[joint_name] = joint
        
    def add_motor(self, motor_name: str, motor: CustomMotor):
        """添加电机"""
        self.motors[motor_name] = motor
    
    def set_physics_param(self, param_name: str, value: float) -> Dict:
        """
        设置物理参数
        
        参数:
            param_name: 参数名称
            value: 新值
            
        返回:
            影响报告
        """
        if param_name not in self.physics_params:
            raise ValueError(f"未知参数: {param_name}")
        
        old_value = self.physics_params[param_name]
        self.physics_params[param_name] = value
        
        # 分析影响
        impact = self._analyze_physics_impact(param_name, old_value, value)
        
        return {
            'parameter': param_name,
            'old_value': old_value,
            'new_value': value,
            'impact': impact
        }
    
    def _analyze_physics_impact(self, param_name: str, old_value: float, new_value: float) -> Dict:
        """分析物理参数变化的影响"""
        impact = {}
        change_ratio = (new_value - old_value) / old_value if old_value != 0 else 0
        
        if param_name == 'motor_power_multiplier':
            impact['max_torque'] = f"{change_ratio*100:+.1f}%"
            impact['max_speed'] = f"{change_ratio*100:+.1f}%"
            impact['energy_consumption'] = f"{change_ratio*100:+.1f}%"
            impact['expected_behavior'] = "更强的驱动力" if new_value > old_value else "更弱的驱动力"
            
        elif param_name == 'joint_stiffness':
            impact['position_accuracy'] = f"{change_ratio*50:+.1f}%"
            impact['response_time'] = f"{-change_ratio*30:+.1f}%"
            impact['expected_behavior'] = "更精确但可能更震荡" if new_value > old_value else "更柔和但精度降低"
            
        elif param_name == 'joint_damping':
            impact['oscillation'] = f"{-change_ratio*60:+.1f}%"
            impact['settling_time'] = f"{change_ratio*40:+.1f}%"
            impact['expected_behavior'] = "更稳定但响应慢" if new_value > old_value else "响应快但可能震荡"
            
        elif param_name == 'friction':
            impact['滑动阻力'] = f"{change_ratio*100:+.1f}%"
            impact['能量损失'] = f"{change_ratio*50:+.1f}%"
            impact['expected_behavior'] = "更稳定但能耗高" if new_value > old_value else "能耗低但可能打滑"
            
        elif param_name == 'mass_multiplier':
            impact['惯性'] = f"{change_ratio*100:+.1f}%"
            impact['required_torque'] = f"{change_ratio*100:+.1f}%"
            impact['expected_behavior'] = "更稳定但需要更大力矩" if new_value > old_value else "灵活但稳定性降低"
        
        return impact
    
    def compute_action_from_params(self, observation: np.ndarray) -> np.ndarray:
        """
        根据当前物理参数计算动作
        而非直接指定动作值
        
        参数:
            observation: 当前观测
            
        返回:
            调整后的动作
        """
        # 简化的PD控制器，参数由物理配置决定
        action_dim = self.env.action_space.shape[0]
        
        # 提取观测信息
        # 假设观测包含: [位置, 速度, ...]
        if len(observation) >= action_dim * 2:
            positions = observation[:action_dim]
            velocities = observation[action_dim:action_dim*2]
        else:
            positions = observation[:action_dim] if len(observation) >= action_dim else np.zeros(action_dim)
            velocities = np.zeros(action_dim)
        
        # 使用物理参数计算PD控制
        kp = self.physics_params['joint_stiffness'] * 50.0  # 比例增益
        kd = self.physics_params['joint_damping'] * 10.0    # 微分增益
        
        # 目标位置（可以根据任务调整）
        target_positions = np.zeros(action_dim)
        
        # PD控制
        action = kp * (target_positions - positions) - kd * velocities
        
        # 应用电机功率限制
        max_torque = self.physics_params['motor_power_multiplier'] * 1.0
        action = np.clip(action, -max_torque, max_torque)
        
        return action
    
    def step_with_params(self, observation: Optional[np.ndarray] = None) -> Tuple:
        """
        使用参数化控制执行一步
        
        参数:
            observation: 当前观测（如果为None则从环境获取）
            
        返回:
            (observation, reward, terminated, truncated, info)
        """
        if observation is None:
            observation, _ = self.env.reset()
        
        # 根据参数计算动作
        action = self.compute_action_from_params(observation)
        
        # 执行动作
        return self.env.step(action)
    
    def run_episode(self, max_steps: int = 1000, render: bool = False) -> Dict:
        """
        运行完整回合，使用当前参数配置
        
        参数:
            max_steps: 最大步数
            render: 是否渲染
            
        返回:
            回合统计信息
        """
        if render:
            self.env = gym.make(self.env_id, render_mode='human')
        
        obs, _ = self.env.reset()
        total_reward = 0
        steps = 0
        
        for step in range(max_steps):
            action = self.compute_action_from_params(obs)
            obs, reward, terminated, truncated, info = self.env.step(action)
            
            total_reward += reward
            steps += 1
            
            if terminated or truncated:
                break
        
        return {
            'total_reward': total_reward,
            'steps': steps,
            'success': steps >= max_steps * 0.8,
            'physics_params': self.physics_params.copy()
        }
    
    def find_optimal_params(self, param_ranges: Dict, n_trials: int = 10):
        """
        搜索最优参数配置
        
        参数:
            param_ranges: 参数范围字典 {param_name: (min, max)}
            n_trials: 试验次数
            
        返回:
            最优参数和性能
        """
        best_params = None
        best_reward = -float('inf')
        results = []
        
        print(f"\n开始参数优化 ({n_trials} 次试验)...")
        
        for trial in range(n_trials):
            # 随机采样参数
            test_params = {}
            for param_name, (min_val, max_val) in param_ranges.items():
                test_params[param_name] = np.random.uniform(min_val, max_val)
                self.set_physics_param(param_name, test_params[param_name])
            
            # 运行回合
            result = self.run_episode(max_steps=500, render=False)
            results.append({
                'params': test_params.copy(),
                'reward': result['total_reward']
            })
            
            # 更新最优
            if result['total_reward'] > best_reward:
                best_reward = result['total_reward']
                best_params = test_params.copy()
            
            print(f"  试验 {trial+1}/{n_trials}: 奖励 = {result['total_reward']:.2f}")
        
        # 恢复最优参数
        for param_name, value in best_params.items():
            self.set_physics_param(param_name, value)
        
        return {
            'best_params': best_params,
            'best_reward': best_reward,
            'all_results': results
        }


class InteractiveParameterTuner:
    """交互式参数调整工具"""
    
    def __init__(self, controller: ParametricRobotController):
        self.controller = controller
        
    def show_current_params(self):
        """显示当前参数"""
        print("\n当前物理参数:")
        print("-" * 60)
        for param, value in self.controller.physics_params.items():
            print(f"  {param}: {value:.3f}")
        print("-" * 60)
    
    def interactive_tuning(self):
        """交互式调整"""
        print("\n=== 交互式参数调整 ===")
        print("命令:")
        print("  set <参数名> <值>  - 设置参数")
        print("  test              - 测试当前配置")
        print("  show              - 显示当前参数")
        print("  reset             - 重置参数")
        print("  quit              - 退出")
        
        while True:
            cmd = input("\n> ").strip().split()
            
            if not cmd:
                continue
            
            if cmd[0] == 'quit':
                break
            
            elif cmd[0] == 'show':
                self.show_current_params()
            
            elif cmd[0] == 'set' and len(cmd) == 3:
                param_name = cmd[1]
                try:
                    value = float(cmd[2])
                    result = self.controller.set_physics_param(param_name, value)
                    print(f"\n✓ 参数已更新")
                    print(f"  影响: {result['impact']}")
                except ValueError as e:
                    print(f"✗ 错误: {e}")
            
            elif cmd[0] == 'test':
                print("\n运行测试...")
                result = self.controller.run_episode(max_steps=500, render=False)
                print(f"\n测试结果:")
                print(f"  总奖励: {result['total_reward']:.2f}")
                print(f"  步数: {result['steps']}")
                print(f"  成功: {'是' if result['success'] else '否'}")
            
            elif cmd[0] == 'reset':
                # 重置到默认值
                for param in self.controller.physics_params:
                    if param == 'gravity':
                        self.controller.physics_params[param] = 9.81
                    elif param in ['motor_power_multiplier', 'joint_stiffness', 'mass_multiplier']:
                        self.controller.physics_params[param] = 1.0
                    elif param == 'joint_damping':
                        self.controller.physics_params[param] = 0.5
                    elif param == 'friction':
                        self.controller.physics_params[param] = 0.9
                print("✓ 参数已重置")
            
            else:
                print("未知命令")


if __name__ == "__main__":
    print("参数化机器人控制系统")
    print("示例请运行: examples/parametric_control_demo.py")
