"""
IMC-22数据准备工具
为IMC-22 NPU准备训练数据

功能:
- 从AGI-Walker数据集提取特征
- 简化和归一化数据
- 生成INT8友好的数据格式
"""

import numpy as np
import pickle
from pathlib import Path
from typing import Dict, List, Tuple
import json
from tqdm import tqdm


class IMC22DatasetPreparer:
    """为IMC-22准备数据集"""
    
    def __init__(self, input_dir: str, output_dir: str):
        """
        初始化数据准备器
        
        参数:
            input_dir: AGI-Walker数据集目录
            output_dir: 输出目录
        """
        self.input_dir = Path(input_dir)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # 统计信息
        self.stats = {
            'num_episodes': 0,
            'num_samples': 0,
            'state_dim': 3,
            'action_dim': 3
        }
    
    def prepare(self, max_episodes: int = None):
        """
        准备数据集
        
        参数:
            max_episodes: 最大episode数量，None表示全部
        """
        print("="*70)
        print("IMC-22数据集准备")
        print("="*70)
        
        print(f"\n输入目录: {self.input_dir}")
        print(f"输出目录: {self.output_dir}")
        
        # 收集所有数据
        all_states = []
        all_actions = []
        all_rewards = []
        
        # 遍历episodes
        episode_files = sorted(self.input_dir.glob("episode_*.pkl"))
        
        if max_episodes:
            episode_files = episode_files[:max_episodes]
        
        print(f"\n处理 {len(episode_files)} 个episodes...")
        
        for ep_file in tqdm(episode_files, desc="加载数据"):
            try:
                with open(ep_file, 'rb') as f:
                    data = pickle.load(f)
                
                # 提取状态和动作
                states = data.get('states', [])
                actions = data.get('actions', [])
                rewards = data.get('rewards', [])
                
                for state, action, reward in zip(states, actions, rewards):
                    # 简化状态
                    simplified_state = self.simplify_state(state)
                    
                    # 简化动作
                    simplified_action = self.simplify_action(action)
                    
                    all_states.append(simplified_state)
                    all_actions.append(simplified_action)
                    all_rewards.append(reward)
                
                self.stats['num_episodes'] += 1
            
            except Exception as e:
                print(f"警告: 跳过 {ep_file.name}: {e}")
        
        # 转换为numpy数组
        states_array = np.array(all_states, dtype=np.float32)
        actions_array = np.array(all_actions, dtype=np.float32)
        rewards_array = np.array(all_rewards, dtype=np.float32)
        
        self.stats['num_samples'] = len(states_array)
        
        print(f"\n数据收集完成:")
        print(f"  Episodes: {self.stats['num_episodes']}")
        print(f"  Samples: {self.stats['num_samples']:,}")
        print(f"  State shape: {states_array.shape}")
        print(f"  Action shape: {actions_array.shape}")
        
        # 归一化
        print("\n归一化数据...")
        states_normalized, state_stats = self.normalize(states_array)
        actions_normalized, action_stats = self.normalize(actions_array)
        
        # 分割数据集
        print("\n分割数据集 (train/val/test = 70/15/15)...")
        datasets = self.split_dataset(
            states_normalized, 
            actions_normalized, 
            rewards_array
        )
        
        # 保存
        print("\n保存数据...")
        self.save_datasets(datasets, state_stats, action_stats)
        
        # 生成报告
        self.generate_report(datasets, state_stats, action_stats)
        
        print("\n✓ 数据准备完成!")
    
    def simplify_state(self, state: Dict) -> np.ndarray:
        """
        简化状态表示
        
        从完整状态中提取关键特征用于IMC-22
        
        返回:
            [位置, 速度, 稳定性]
        """
        position = state.get('position', 0)
        velocity = state.get('velocity', 0)
        stable = 1.0 if state.get('stable', True) else 0.0
        
        return np.array([position, velocity, stable], dtype=np.float32)
    
    def simplify_action(self, action: Dict) -> np.ndarray:
        """
        简化动作表示
        
        提取关键控制参数
        
        返回:
            [电机功率, 刚度, 阻尼]
        """
        motor_power = action.get('motor_power_multiplier', 1.0)
        stiffness = action.get('joint_stiffness', 1.0)
        damping = action.get('joint_damping', 0.5)
        
        return np.array([motor_power, stiffness, damping], dtype=np.float32)
    
    def normalize(self, data: np.ndarray) -> Tuple[np.ndarray, Dict]:
        """
        归一化数据
        
        返回:
            (归一化数据, 统计信息)
        """
        mean = data.mean(axis=0)
        std = data.std(axis=0) + 1e-8  # 避免除零
        
        normalized = (data - mean) / std
        
        stats = {
            'mean': mean.tolist(),
            'std': std.tolist(),
            'min': data.min(axis=0).tolist(),
            'max': data.max(axis=0).tolist()
        }
        
        return normalized, stats
    
    def split_dataset(self, states: np.ndarray, actions: np.ndarray, 
                     rewards: np.ndarray, 
                     train_ratio: float = 0.7,
                     val_ratio: float = 0.15) -> Dict:
        """分割数据集"""
        n = len(states)
        
        # 随机打乱
        indices = np.random.permutation(n)
        
        # 分割点
        train_end = int(n * train_ratio)
        val_end = int(n * (train_ratio + val_ratio))
        
        train_idx = indices[:train_end]
        val_idx = indices[train_end:val_end]
        test_idx = indices[val_end:]
        
        return {
            'train': {
                'states': states[train_idx],
                'actions': actions[train_idx],
                'rewards': rewards[train_idx]
            },
            'val': {
                'states': states[val_idx],
                'actions': actions[val_idx],
                'rewards': rewards[val_idx]
            },
            'test': {
                'states': states[test_idx],
                'actions': actions[test_idx],
                'rewards': rewards[test_idx]
            }
        }
    
    def save_datasets(self, datasets: Dict, state_stats: Dict, action_stats: Dict):
        """保存数据集"""
        # 保存numpy格式
        for split_name, data in datasets.items():
            split_dir = self.output_dir / split_name
            split_dir.mkdir(exist_ok=True)
            
            np.save(split_dir / 'states.npy', data['states'])
            np.save(split_dir / 'actions.npy', data['actions'])
            np.save(split_dir / 'rewards.npy', data['rewards'])
        
        # 保存统计信息
        stats = {
            'state_stats': state_stats,
            'action_stats': action_stats,
            'dataset_stats': self.stats
        }
        
        with open(self.output_dir / 'dataset_stats.json', 'w') as f:
            json.dump(stats, f, indent=2)
        
        print(f"  保存到: {self.output_dir}")
    
    def generate_report(self, datasets: Dict, state_stats: Dict, action_stats: Dict):
        """生成报告"""
        report_file = self.output_dir / 'dataset_report.txt'
        
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("="*70 + "\n")
            f.write("IMC-22数据集报告\n")
            f.write("="*70 + "\n\n")
            
            f.write(f"总Episodes: {self.stats['num_episodes']}\n")
            f.write(f"总样本数: {self.stats['num_samples']:,}\n\n")
            
            f.write("数据集分割:\n")
            for split_name, data in datasets.items():
                f.write(f"  {split_name}: {len(data['states']):,} 样本\n")
            
            f.write("\n状态统计:\n")
            f.write(f"  维度: {self.stats['state_dim']}\n")
            f.write(f"  均值: {state_stats['mean']}\n")
            f.write(f"  标准差: {state_stats['std']}\n")
            f.write(f"  范围: {state_stats['min']} ~ {state_stats['max']}\n")
            
            f.write("\n动作统计:\n")
            f.write(f"  维度: {self.stats['action_dim']}\n")
            f.write(f"  均值: {action_stats['mean']}\n")
            f.write(f"  标准差: {action_stats['std']}\n")
            f.write(f"  范围: {action_stats['min']} ~ {action_stats['max']}\n")
        
        print(f"  报告: {report_file}")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("使用方法: python imc22_data_preparer.py <input_dir> [output_dir]")
        print("\n示例:")
        print("  python tools/imc22_data_preparer.py data/generated data/imc22_dataset")
        sys.exit(1)
    
    input_dir = sys.argv[1]
    output_dir = sys.argv[2] if len(sys.argv) > 2 else "data/imc22_dataset"
    
    preparer = IMC22DatasetPreparer(input_dir, output_dir)
    preparer.prepare()
