"""
模仿学习模块
包括行为克隆（BC）和生成对抗模仿学习（GAIL）
"""

import numpy as np
import gymnasium as gym
from typing import Optional, List, Union
import os


class ImitationLearner:
    """模仿学习基类"""
    
    def __init__(self, env_id: str):
        self.env_id = env_id
        self.env = gym.make(env_id)
        
    def collect_expert_demos(
        self, 
        expert_policy,
        n_episodes: int = 100,
        save_path: Optional[str] = None
    ):
        """
        从专家策略收集演示
        
        参数:
            expert_policy: 专家策略（trained model）
            n_episodes: 收集的回合数
            save_path: 保存路径（可选）
        
        返回:
            演示轨迹列表
        """
        from imitation.data import rollout
        from stable_baselines3.common.vec_env import DummyVecEnv
        
        print(f"收集专家演示: {n_episodes} 回合")
        
        # 创建向量化环境
        venv = DummyVecEnv([lambda: gym.make(self.env_id)])
        
        # 收集轨迹
        trajectories = rollout.rollout(
            expert_policy,
            venv,
            rollout.make_sample_until(min_episodes=n_episodes)
        )
        
        print(f"收集完成: {len(trajectories)} 条轨迹")
        
        # 保存（如果指定路径）
        if save_path:
            import pickle
            with open(save_path, 'wb') as f:
                pickle.dump(trajectories, f)
            print(f"轨迹已保存: {save_path}")
        
        return trajectories


class BehaviorCloning(ImitationLearner):
    """
    行为克隆（Behavior Cloning）
    最简单直接的模仿学习方法
    """
    
    def __init__(self, env_id: str):
        super().__init__(env_id)
        self.bc_trainer = None
        
    def train(
        self,
        expert_trajectories,
        n_epochs: int = 10,
        batch_size: int = 64,
        learning_rate: float = 1e-3
    ):
        """
        训练 BC 策略
        
        参数:
            expert_trajectories: 专家轨迹
            n_epochs: 训练轮数
            batch_size: 批大小
            learning_rate: 学习率
        """
        from imitation.algorithms import bc
        
        print("\n开始行为克隆训练")
        print(f"轮数: {n_epochs}, 批大小: {batch_size}")
        
        # 创建 BC 训练器
        self.bc_trainer = bc.BC(
            observation_space=self.env.observation_space,
            action_space=self.env.action_space,
            demonstrations=expert_trajectories,
            rng=np.random.default_rng(0)
        )
        
        # 训练
        self.bc_trainer.train(
            n_epochs=n_epochs,
            log_interval=1,
            progress_bar=True
        )
        
        print("行为克隆训练完成!")
    
    def save(self, path: str):
        """保存模型"""
        if self.bc_trainer is None:
            raise ValueError("模型未训练")
        
        self.bc_trainer.save_policy(path)
        print(f"BC 模型已保存: {path}")
    
    def load(self, path: str):
        """加载模型"""
        from imitation.algorithms import bc
        
        self.bc_trainer = bc.BC(
            observation_space=self.env.observation_space,
            action_space=self.env.action_space,
            rng=np.random.default_rng(0)
        )
        self.bc_trainer.load_policy(path)
        print(f"BC 模型已加载: {path}")
    
    def evaluate(self, n_episodes: int = 10):
        """评估策略"""
        if self.bc_trainer is None:
            raise ValueError("模型未训练")
        
        from stable_baselines3.common.evaluation import evaluate_policy
        from stable_baselines3.common.vec_env import DummyVecEnv
        
        venv = DummyVecEnv([lambda: gym.make(self.env_id)])
        
        mean_reward, std_reward = evaluate_policy(
            self.bc_trainer.policy,
            venv,
            n_eval_episodes=n_episodes,
            deterministic=True
        )
        
        print(f"\nBC 评估结果 ({n_episodes} 回合):")
        print(f"  平均奖励: {mean_reward:.2f} ± {std_reward:.2f}")
        
        return mean_reward, std_reward


class GAIL(ImitationLearner):
    """
    生成对抗模仿学习（Generative Adversarial Imitation Learning）
    无需手动定义奖励函数
    """
    
    def __init__(self, env_id: str, gen_algo='ppo'):
        super().__init__(env_id)
        self.gen_algo_name = gen_algo
        self.gail_trainer = None
        
    def train(
        self,
        expert_trajectories,
        total_timesteps: int = 500_000,
        n_disc_updates_per_round: int = 4
    ):
        """
        训练 GAIL
        
        参数:
            expert_trajectories: 专家轨迹
            total_timesteps: 总训练步数
            n_disc_updates_per_round: 判别器更新次数
        """
        from imitation.algorithms.adversarial import gail
        from imitation.rewards.reward_nets import BasicRewardNet
        from imitation.util.networks import RunningNorm
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv
        
        print("\n开始 GAIL 训练")
        print(f"总步数: {total_timesteps:,}")
        
        # 创建向量化环境
        venv = DummyVecEnv([lambda: gym.make(self.env_id)])
        
        # 创建生成器（PPO策略）
        gen_algo = PPO(
            "MlpPolicy",
            venv,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            verbose=1
        )
        
        # 创建奖励网络（判别器）
        reward_net = BasicRewardNet(
            observation_space=self.env.observation_space,
            action_space=self.env.action_space,
            normalize_input_layer=RunningNorm
        )
        
        # 创建 GAIL 训练器
        self.gail_trainer = gail.GAIL(
            demonstrations=expert_trajectories,
            demo_batch_size=1024,
            gen_replay_buffer_capacity=2048,
            n_disc_updates_per_round=n_disc_updates_per_round,
            venv=venv,
            gen_algo=gen_algo,
            reward_net=reward_net
        )
        
        # 训练
        self.gail_trainer.train(total_timesteps=total_timesteps)
        
        print("GAIL 训练完成!")
    
    def save(self, path: str):
        """保存模型"""
        if self.gail_trainer is None:
            raise ValueError("模型未训练")
        
        # 保存生成器（策略）
        self.gail_trainer.gen_algo.save(path)
        print(f"GAIL 模型已保存: {path}")
    
    def evaluate(self, n_episodes: int = 10):
        """评估策略"""
        if self.gail_trainer is None:
            raise ValueError("模型未训练")
        
        from stable_baselines3.common.evaluation import evaluate_policy
        
        mean_reward, std_reward = evaluate_policy(
            self.gail_trainer.gen_algo,
            self.gail_trainer.venv,
            n_eval_episodes=n_episodes,
            deterministic=True
        )
        
        print(f"\nGAIL 评估结果 ({n_episodes} 回合):")
        print(f"  平均奖励: {mean_reward:.2f} ± {std_reward:.2f}")
        
        return mean_reward, std_reward


if __name__ == "__main__":
    print("模仿学习模块加载成功")
    print("使用示例请查看: examples/imitation_learning_demo.py")
