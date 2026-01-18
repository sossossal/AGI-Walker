"""
性能评估工具 - 对比不同 RL 算法的效果
"""

import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
from typing import List, Dict
import time


class RLBenchmark:
    """强化学习算法性能基准测试"""
    
    def __init__(self, env_id: str, n_eval_episodes: int = 10):
        self.env_id = env_id
        self.n_eval_episodes = n_eval_episodes
        
    def evaluate_policy(self, policy, deterministic: bool = True) -> Dict:
        """
        评估策略性能
        
        返回:
            字典包含 mean_reward, std_reward, mean_length, success_rate
        """
        env = gym.make(self.env_id)
        
        episode_rewards = []
        episode_lengths = []
        successes = []
        
        for _ in range(self.n_eval_episodes):
            obs, _ = env.reset()
            done = False
            episode_reward = 0
            episode_length = 0
            
            while not done:
                action, _ = policy.predict(obs, deterministic=deterministic)
                obs, reward, terminated, truncated, info = env.step(action)
                done = terminated or truncated
                
                episode_reward += reward
                episode_length += 1
            
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            
            # 判断是否成功（如果环境提供此信息）
            if 'is_success' in info:
                successes.append(info['is_success'])
        
        env.close()
        
        results = {
            'mean_reward': np.mean(episode_rewards),
            'std_reward': np.std(episode_rewards),
            'mean_length': np.mean(episode_lengths),
            'success_rate': np.mean(successes) if successes else None
        }
        
        return results
    
    def compare_algorithms(
        self,
        policies: Dict[str, any],
        save_path: str = "benchmark_results.png"
    ):
        """
        对比多个算法的性能
        
        参数:
            policies: {算法名称: 策略} 字典
            save_path: 保存图表的路径
        """
        results = {}
        
        print("="*60)
        print("性能基准测试")
        print("="*60)
        
        for name, policy in policies.items():
            print(f"\n评估 {name}...")
            start_time = time.time()
            result = self.evaluate_policy(policy)
            eval_time = time.time() - start_time
            
            result['eval_time'] = eval_time
            results[name] = result
            
            print(f"  平均奖励: {result['mean_reward']:.2f} ± {result['std_reward']:.2f}")
            print(f"  平均长度: {result['mean_length']:.1f}")
            print(f"  评估时间: {eval_time:.2f}s")
            if result['success_rate'] is not None:
                print(f"  成功率: {result['success_rate']*100:.1f}%")
        
        # 可视化
        self._plot_comparison(results, save_path)
        
        return results
    
    def _plot_comparison(self, results: Dict, save_path: str):
        """绘制对比图表"""
        names = list(results.keys())
        mean_rewards = [results[name]['mean_reward'] for name in names]
        std_rewards = [results[name]['std_reward'] for name in names]
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        # 奖励对比
        x = np.arange(len(names))
        ax1.bar(x, mean_rewards, yerr=std_rewards, capsize=5)
        ax1.set_xlabel('算法')
        ax1.set_ylabel('平均回合奖励')
        ax1.set_title('算法性能对比')
        ax1.set_xticks(x)
        ax1.set_xticklabels(names, rotation=45, ha='right')
        ax1.grid(axis='y', alpha=0.3)
        
        # 样本效率对比（如果可用）
        if 'training_steps' in results[names[0]]:
            steps = [results[name]['training_steps'] for name in names]
            ax2.scatter(steps, mean_rewards, s=100)
            for i, name in enumerate(names):
                ax2.annotate(name, (steps[i], mean_rewards[i]))
            ax2.set_xlabel('训练步数')
            ax2.set_ylabel('最终性能')
            ax2.set_title('样本效率对比')
            ax2.grid(alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\n对比图表已保存: {save_path}")
        
        return fig


def measure_sample_efficiency(
    env_id: str,
    algorithms: Dict[str, callable],
    max_steps: int = 100000,
    eval_freq: int = 5000
) -> Dict:
    """
    测试样本效率 - 达到相同性能需要多少样本
    
    参数:
        env_id: 环境 ID
        algorithms: {算法名: 训练函数} 字典
        max_steps: 最大训练步数
        eval_freq: 评估频率
    
    返回:
        包含各算法学习曲线的字典
    """
    results = {}
    
    for name, train_fn in algorithms.items():
        print(f"\n训练 {name}...")
        # TODO: 实现训练过程监控
        # 记录每个 eval_freq 的性能
        pass
    
    return results


if __name__ == "__main__":
    print("性能评估工具加载成功")
    print("使用示例:")
    print("  from python_api.benchmark import RLBenchmark")
    print("  benchmark = RLBenchmark('AGI-Walker/Walker2D-v0')")
    print("  results = benchmark.compare_algorithms(policies)")
