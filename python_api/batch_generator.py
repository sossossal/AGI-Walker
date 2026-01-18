"""
批量数据生成器
Batch Data Generator

功能:
- 批量配置生成
- 多进程并行运行
- 进度追踪
- 断点续传
- 自动数据收集
"""

import os
import json
import numpy as np
from typing import Dict, List, Optional, Callable
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, asdict
import time
from tqdm import tqdm
import pickle


@dataclass
class GenerationConfig:
    """生成配置"""
    # 基础配置
    num_episodes: int = 100
    episode_length: int = 1000
    dt: float = 0.01
    
    # 并行配置
    num_workers: int = 4
    
    # 输出配置
    output_dir: str = "data/generated"
    format: str = "json"  # json, pickle, both
    
    # 参数范围
    motor_power_range: tuple = (0.8, 1.5)
    joint_stiffness_range: tuple = (0.7, 1.5)
    joint_damping_range: tuple = (0.3, 0.8)
    mass_range: tuple = (0.9, 1.1)
    friction_range: tuple = (0.8, 1.2)
    
    # 环境配置
    environments: List[str] = None
    
    # 断点续传
    resume: bool = False
    checkpoint_interval: int = 10


class EpisodeConfig:
    """单个Episode的配置"""
    
    def __init__(self, episode_id: int, robot_params: Dict, environment: str):
        self.episode_id = episode_id
        self.robot_params = robot_params
        self.environment = environment
    
    def to_dict(self):
        return {
            'episode_id': self.episode_id,
            'robot_params': self.robot_params,
            'environment': self.environment
        }


class BatchDataGenerator:
    """批量数据生成器"""
    
    def __init__(self, config: GenerationConfig):
        self.config = config
        
        # 创建输出目录
        Path(config.output_dir).mkdir(parents=True, exist_ok=True)
        
        # 进度追踪
        self.completed_episodes = []
        self.failed_episodes = []
        
        # 断点续传
        self.checkpoint_file = Path(config.output_dir) / "checkpoint.pkl"
        
        # 统计
        self.total_time = 0
        self.total_steps = 0
        
        # 默认环境
        if config.environments is None:
            self.config.environments = ['flat', 'uphill_5deg', 'windy']
    
    def sample_robot_params(self) -> Dict:
        """采样机器人参数"""
        return {
            'motor_power_multiplier': np.random.uniform(*self.config.motor_power_range),
            'joint_stiffness': np.random.uniform(*self.config.joint_stiffness_range),
            'joint_damping': np.random.uniform(*self.config.joint_damping_range),
            'mass_multiplier': np.random.uniform(*self.config.mass_range),
            'friction': np.random.uniform(*self.config.friction_range),
        }
    
    def generate_episode_configs(self) -> List[EpisodeConfig]:
        """生成所有Episode配置"""
        configs = []
        
        for i in range(self.config.num_episodes):
            robot_params = self.sample_robot_params()
            environment = np.random.choice(self.config.environments)
            
            configs.append(EpisodeConfig(i, robot_params, environment))
        
        return configs
    
    def run_single_episode(self, episode_config: EpisodeConfig) -> Dict:
        """
        运行单个Episode
        
        这是在子进程中执行的函数
        """
        # 导入需要在子进程中使用的模块
        from python_api.data_recorder import DataRecorder
        from python_api.physics_validator import PhysicsSimulator
        from python_api.environment_balance import EnvironmentBalanceSimulator, Environments
        
        episode_id = episode_config.episode_id
        
        try:
            # 创建记录器
            recorder = DataRecorder(f"episode_{episode_id:06d}")
            
            # 创建环境
            if episode_config.environment == 'flat':
                env_config = Environments.flat()
            elif episode_config.environment == 'uphill_5deg':
                env_config = Environments.uphill_5deg()
            elif episode_config.environment == 'windy':
                env_config = Environments.windy()
            else:
                env_config = Environments.flat()
            
            # 创建模拟器
            sim = EnvironmentBalanceSimulator(
                episode_config.robot_params,
                env_config
            )
            
            # 运行模拟
            states = []
            actions = []
            rewards = []
            
            for step in range(self.config.episode_length):
                timestamp = step * self.config.dt
                
                # 执行步骤
                result = sim.step(self.config.dt)
                
                # 记录状态
                state = {
                    'position': result.get('distance', 0),
                    'velocity': result.get('velocity', 0),
                    'stable': result.get('stable', True)
                }
                
                # 记录动作 (参数化动作)
                action = episode_config.robot_params.copy()
                
                # 计算奖励 (简单示例)
                reward = 1.0 if result.get('stable', False) else -1.0
                reward += result.get('velocity', 0) * 0.1  # 速度奖励
                
                states.append(state)
                actions.append(action)
                rewards.append(reward)
                
                # 记录到recorder
                recorder.record_state(timestamp, state)
                
                # 检查是否终止
                if not result.get('stable', True):
                    break
            
            # 保存数据
            output_path = Path(self.config.output_dir) / f"episode_{episode_id:06d}"
            
            if self.config.format in ['json', 'both']:
                recorder.save_json(f"{output_path}.json")
            
            if self.config.format in ['pickle', 'both']:
                episode_data = {
                    'config': episode_config.to_dict(),
                    'states': states,
                    'actions': actions,
                    'rewards': rewards,
                    'total_reward': sum(rewards),
                    'episode_length': len(states)
                }
                
                with open(f"{output_path}.pkl", 'wb') as f:
                    pickle.dump(episode_data, f)
            
            # 返回统计
            return {
                'episode_id': episode_id,
                'success': True,
                'steps': len(states),
                'total_reward': sum(rewards),
                'final_distance': states[-1]['position'] if states else 0
            }
        
        except Exception as e:
            return {
                'episode_id': episode_id,
                'success': False,
                'error': str(e)
            }
    
    def load_checkpoint(self) -> Optional[Dict]:
        """加载断点"""
        if self.checkpoint_file.exists():
            with open(self.checkpoint_file, 'rb') as f:
                return pickle.load(f)
        return None
    
    def save_checkpoint(self):
        """保存断点"""
        checkpoint = {
            'completed_episodes': self.completed_episodes,
            'failed_episodes': self.failed_episodes,
            'total_time': self.total_time,
            'total_steps': self.total_steps
        }
        
        with open(self.checkpoint_file, 'wb') as f:
            pickle.dump(checkpoint, f)
    
    def generate(self) -> Dict:
        """
        执行批量生成
        
        返回:
            生成统计
        """
        print("="*70)
        print("批量数据生成器")
        print("="*70)
        
        # 检查断点续传
        start_idx = 0
        if self.config.resume:
            checkpoint = self.load_checkpoint()
            if checkpoint:
                self.completed_episodes = checkpoint['completed_episodes']
                self.failed_episodes = checkpoint['failed_episodes']
                self.total_time = checkpoint['total_time']
                self.total_steps = checkpoint['total_steps']
                start_idx = len(self.completed_episodes) + len(self.failed_episodes)
                print(f"从断点恢复: 已完成 {len(self.completed_episodes)} episodes")
        
        # 生成配置
        all_configs = self.generate_episode_configs()
        remaining_configs = all_configs[start_idx:]
        
        print(f"\n配置:")
        print(f"  总Episodes: {self.config.num_episodes}")
        print(f"  待生成: {len(remaining_configs)}")
        print(f"  并行度: {self.config.num_workers}")
        print(f"  输出目录: {self.config.output_dir}")
        print(f"  输出格式: {self.config.format}")
        
        # 并行运行
        start_time = time.time()
        
        with ProcessPoolExecutor(max_workers=self.config.num_workers) as executor:
            # 提交任务
            futures = {
                executor.submit(self.run_single_episode, config): config
                for config in remaining_configs
            }
            
            # 进度条
            with tqdm(total=len(remaining_configs), desc="生成进度") as pbar:
                for future in as_completed(futures):
                    result = future.result()
                    
                    if result['success']:
                        self.completed_episodes.append(result['episode_id'])
                        self.total_steps += result['steps']
                    else:
                        self.failed_episodes.append({
                            'episode_id': result['episode_id'],
                            'error': result['error']
                        })
                    
                    pbar.update(1)
                    pbar.set_postfix({
                        '成功': len(self.completed_episodes),
                        '失败': len(self.failed_episodes)
                    })
                    
                    # 定期保存断点
                    if len(self.completed_episodes) % self.config.checkpoint_interval == 0:
                        self.save_checkpoint()
        
        self.total_time = time.time() - start_time
        
        # 最终保存
        self.save_checkpoint()
        
        # 生成报告
        return self.generate_report()
    
    def generate_report(self) -> Dict:
        """生成统计报告"""
        report = {
            'total_episodes': self.config.num_episodes,
            'completed': len(self.completed_episodes),
            'failed': len(self.failed_episodes),
            'success_rate': len(self.completed_episodes) / self.config.num_episodes * 100,
            'total_time_seconds': self.total_time,
            'total_steps': self.total_steps,
            'avg_time_per_episode': self.total_time / max(len(self.completed_episodes), 1),
            'episodes_per_hour': len(self.completed_episodes) / (self.total_time / 3600) if self.total_time > 0 else 0
        }
        
        # 保存报告
        report_path = Path(self.config.output_dir) / "generation_report.json"
        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2)
        
        # 打印报告
        print("\n" + "="*70)
        print("生成完成!")
        print("="*70)
        print(f"总Episodes: {report['total_episodes']}")
        print(f"成功: {report['completed']} ({report['success_rate']:.1f}%)")
        print(f"失败: {report['failed']}")
        print(f"总时间: {report['total_time_seconds']:.1f}s ({report['total_time_seconds']/60:.1f}分钟)")
        print(f"总步数: {report['total_steps']:,}")
        print(f"平均时间/Episode: {report['avg_time_per_episode']:.2f}s")
        print(f"生成速度: {report['episodes_per_hour']:.1f} episodes/小时")
        
        if self.failed_episodes:
            print(f"\n失败Episodes:")
            for fail in self.failed_episodes[:5]:
                print(f"  Episode {fail['episode_id']}: {fail['error']}")
        
        print(f"\n数据已保存到: {self.config.output_dir}")
        
        return report


if __name__ == "__main__":
    print("批量数据生成器加载完成")
    
    # 示例: 生成小规模数据集
    config = GenerationConfig(
        num_episodes=10,
        episode_length=100,
        num_workers=2,
        output_dir="data/test_batch",
        format="both",
        environments=['flat', 'uphill_5deg']
    )
    
    generator = BatchDataGenerator(config)
    report = generator.generate()
    
    print("\n完成!")
