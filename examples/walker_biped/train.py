"""
双足机器人训练脚本
支持基础训练、完整训练和域随机化
"""

import argparse
import json
import os
from typing import Optional

from godot_robot_env import GodotRobotEnv, DomainRandomizationWrapper
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor


def load_robot_config():
    """加载机器人配置"""
    config_path = os.path.join(os.path.dirname(__file__), 'robot_config.json')
    with open(config_path, 'r', encoding='utf-8') as f:
        return json.load(f)


def create_env(domain_randomization=False, render=False):
    """创建训练环境"""
    env = GodotRobotEnv(
        scene_name="biped_walker",
        env_preset="earth",
        ground_material="concrete",
        render_mode="human" if render else None
    )
    
    if domain_randomization:
        env = DomainRandomizationWrapper(env)
    
    env = Monitor(env)
    return env


def make_env(domain_randomization=False):
    """环境工厂函数（用于并行）"""
    def _init():
        return create_env(domain_randomization)
    return _init


class BipedRewardCalculator:
    """双足机器人奖励计算器"""
    
    def __init__(self, config):
        self.weights = config['reward_weights']
    
    def calculate(self, obs, action, info):
        """
        计算奖励
        
        obs: 观察 [position, velocity, orientation, joint_angles, ...]
        action: 动作
        info: 额外信息
        """
        # 解析观察
        vel_x = obs[0]  # 前进速度
        vel_y = obs[1]  # 侧向速度
        vel_ang = obs[2]  # 角速度
        height = obs[3]  # 高度
        
        # 计算各项奖励
        forward_reward = vel_x * self.weights['forward_velocity']
        lateral_penalty = abs(vel_y) * self.weights['lateral_penalty']
        angular_penalty = abs(vel_ang) * self.weights['angular_penalty']
        
        # 能耗惩罚
        energy = sum([abs(a) for a in action])
        energy_penalty = energy * self.weights['energy_penalty']
        
        # 站立奖励
        upright_bonus = height * self.weights['upright_bonus'] if height > 0.4 else 0
        
        # 生存奖励
        survival_bonus = self.weights['survival_bonus']
        
        total_reward = (
            forward_reward +
            lateral_penalty +
            angular_penalty +
            energy_penalty +
            upright_bonus +
            survival_bonus
        )
        
        return total_reward


def train_basic(config, timesteps=100000):
    """基础训练模式"""
    print("\n基础训练模式")
    print("-" * 60)
    print(f"总步数: {timesteps}")
    print(f"环境: 单个")
    print(f"域随机化: 否")
    
    env = create_env()
    
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        **config['training_config']
    )
    
    model.learn(total_timesteps=timesteps)
    model.save("walker_policy_basic")
    
    print("\n✓ 训练完成，模型保存为: walker_policy_basic.zip")
    env.close()


def train_full(config, timesteps=1000000, domain_randomization=False, n_envs=4):
    """完整训练模式"""
    print("\n完整训练模式")
    print("-" * 60)
    print(f"总步数: {timesteps}")
    print(f"并行环境: {n_envs}")
    print(f"域随机化: {'是' if domain_randomization else '否'}")
    
    # 创建并行环境
    if n_envs > 1:
        env = SubprocVecEnv([make_env(domain_randomization) for _ in range(n_envs)])
    else:
        env = DummyVecEnv([make_env(domain_randomization)])
    
    # 回调
    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path='./checkpoints/',
        name_prefix='walker'
    )
    
    # 创建模型
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log="./tensorboard/",
        **config['training_config']
    )
    
    # 训练
    model.learn(
        total_timesteps=timesteps,
        callback=checkpoint_callback
    )
    
    # 保存
    save_name = "walker_policy_full_dr" if domain_randomization else "walker_policy_full"
    model.save(save_name)
    
    print(f"\n✓ 训练完成，模型保存为: {save_name}.zip")
    env.close()


def test_policy(model_path: str):
    """测试策略"""
    print("\n测试模式")
    print("-" * 60)
    print(f"加载模型: {model_path}")
    
    # 加载模型
    model = PPO.load(model_path)
    
    # 创建环境（带渲染）
    env = create_env(render=True)
    
    # 测试 10 个回合
    for episode in range(10):
        obs, info = env.reset()
        episode_reward = 0
        steps = 0
        
        while True:
            action, _states = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            episode_reward += reward
            steps += 1
            
            if terminated or truncated:
                break
        
        print(f"回合 {episode + 1}: 奖励 = {episode_reward:.2f}, 步数 = {steps}")
    
    env.close()
    print("\n✓ 测试完成")


def main():
    parser = argparse.ArgumentParser(description='双足机器人训练')
    parser.add_argument('--mode', choices=['basic', 'full'], default='basic',
                      help='训练模式')
    parser.add_argument('--timesteps', type=int, default=None,
                      help='训练步数')
    parser.add_argument('--domain-randomization', action='store_true',
                      help='启用域随机化')
    parser.add_argument('--n-envs', type=int, default=4,
                      help='并行环境数量')
    parser.add_argument('--test', action='store_true',
                      help='测试模式')
    parser.add_argument('--model', type=str, default='walker_policy_full.zip',
                      help='模型路径（测试时使用）')
    
    args = parser.parse_args()
    
    # 加载配置
    config = load_robot_config()
    
    print("=" * 60)
    print("双足行走机器人训练")
    print("=" * 60)
    print(f"\n机器人配置: {config['project_name']}")
    print(f"关节数量: {config['specifications']['num_joints']}")
    print(f"总成本: ${config['cost_summary']['total']}")
    
    if args.test:
        test_policy(args.model)
    elif args.mode == 'basic':
        timesteps = args.timesteps or 100000
        train_basic(config, timesteps)
    elif args.mode == 'full':
        timesteps = args.timesteps or 1000000
        train_full(config, timesteps, args.domain_randomization, args.n_envs)


if __name__ == "__main__":
    main()
