"""
AGI-Walker 离线强化学习模块
使用 d3rlpy 实现 CQL (Conservative Q-Learning)
"""

import numpy as np
import gymnasium as gym
from typing import Dict, List, Tuple, Optional
import pickle
import os


class ExpertDataCollector:
    """专家数据收集器"""
    
    def __init__(self, env_id: str, save_dir: str = "offline_data"):
        self.env_id = env_id
        self.save_dir = save_dir
        os.makedirs(save_dir, exist_ok=True)
        
    def collect_from_policy(
        self, 
        policy,
        n_episodes: int = 1000,
        deterministic: bool = True
    ) -> Dict:
        """
        从专家策略收集演示数据
        
        参数:
            policy: 训练好的策略
            n_episodes: 收集的回合数
            deterministic: 是否使用确定性动作
        
        返回:
            包含 observations, actions, rewards, terminals 的字典
        """
        env = gym.make(self.env_id)
        
        observations = []
        actions = []
        rewards = []
        terminals = []
        
        print(f"开始收集专家数据，目标回合数: {n_episodes}")
        
        for episode in range(n_episodes):
            obs, _ = env.reset()
            done = False
            episode_reward = 0
            
            while not done:
                # 获取动作
                action, _ = policy.predict(obs, deterministic=deterministic)
                
                # 存储状态和动作
                observations.append(obs)
                actions.append(action)
                
                # 执行动作
                obs, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
                
                # 存储奖励和终止标志
                rewards.append(reward)
                terminals.append(done)
                episode_reward += reward
            
            if (episode + 1) % 100 == 0:
                print(f"已收集 {episode + 1}/{n_episodes} 回合, "
                      f"最近回合奖励: {episode_reward:.2f}")
        
        env.close()
        
        # 转换为 numpy 数组
        dataset = {
            'observations': np.array(observations),
            'actions': np.array(actions),
            'rewards': np.array(rewards),
            'terminals': np.array(terminals)
        }
        
        print(f"\n数据收集完成:")
        print(f"  - 样本总数: {len(observations)}")
        print(f"  - 观测维度: {observations[0].shape}")
        print(f"  - 动作维度: {actions[0].shape}")
        print(f"  - 平均奖励: {np.mean(rewards):.2f}")
        
        return dataset
    
    def save_dataset(self, dataset: Dict, filename: str):
        """保存数据集"""
        filepath = os.path.join(self.save_dir, filename)
        with open(filepath, 'wb') as f:
            pickle.dump(dataset, f)
        print(f"数据集已保存至: {filepath}")
    
    def load_dataset(self, filename: str) -> Dict:
        """加载数据集"""
        filepath = os.path.join(self.save_dir, filename)
        with open(filepath, 'rb') as f:
            dataset = pickle.load(f)
        print(f"数据集已加载: {filepath}")
        return dataset


class OfflineRLTrainer:
    """离线 RL 训练器（使用 CQL）"""
    
    def __init__(
        self,
        env_id: str,
        algorithm: str = "cql",
        actor_lr: float = 3e-4,
        critic_lr: float = 3e-4,
        batch_size: int = 256,
        n_critics: int = 2
    ):
        self.env_id = env_id
        self.algorithm = algorithm
        
        # 延迟导入 d3rlpy（如果未安装会给出友好提示）
        try:
            from d3rlpy.algos import CQL, SAC
            from d3rlpy.dataset import MDPDataset
            self.CQL = CQL
            self.SAC = SAC
            self.MDPDataset = MDPDataset
        except ImportError:
            print("错误: 未安装 d3rlpy")
            print("请运行: pip install d3rlpy")
            raise
        
        # CQL 配置
        if algorithm == "cql":
            self.model = self.CQL(
                actor_learning_rate=actor_lr,
                critic_learning_rate=critic_lr,
                batch_size=batch_size,
                n_critics=n_critics,
                conservative_weight=5.0,  # CQL 保守性权重
            )
        else:
            raise ValueError(f"不支持的算法: {algorithm}")
    
    def prepare_dataset(self, raw_dataset: Dict):
        """将原始数据转换为 d3rlpy MDPDataset"""
        dataset = self.MDPDataset(
            observations=raw_dataset['observations'],
            actions=raw_dataset['actions'],
            rewards=raw_dataset['rewards'],
            terminals=raw_dataset['terminals']
        )
        return dataset
    
    def train_offline(
        self,
        dataset,
        n_steps: int = 100000,
        save_interval: int = 10000,
        save_dir: str = "offline_models"
    ):
        """离线训练"""
        os.makedirs(save_dir, exist_ok=True)
        
        print(f"\n开始离线训练 ({self.algorithm.upper()})")
        print(f"训练步数: {n_steps}")
        
        # 训练
        self.model.fit(
            dataset,
            n_steps=n_steps,
            save_interval=save_interval,
            save_dir=save_dir,
            verbose=True
        )
        
        print("\n离线训练完成!")
    
    def finetune_online(
        self,
        env,
        n_steps: int = 10000,
        save_dir: str = "finetuned_models"
    ):
        """在线 Fine-tuning"""
        os.makedirs(save_dir, exist_ok=True)
        
        print(f"\n开始在线 Fine-tuning")
        print(f"训练步数: {n_steps}")
        
        self.model.fit_online(
            env,
            n_steps=n_steps,
            save_dir=save_dir
        )
        
        print("\nFine-tuning 完成!")
    
    def save(self, filepath: str):
        """保存模型"""
        self.model.save(filepath)
        print(f"模型已保存: {filepath}")
    
    def load(self, filepath: str):
        """加载模型"""
        self.model.load(filepath)
        print(f"模型已加载: {filepath}")


if __name__ == "__main__":
    print("离线强化学习模块加载成功")
    print("使用示例请查看: examples/offline_rl_demo.py")
