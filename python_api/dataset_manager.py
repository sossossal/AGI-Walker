"""
数据集管理工具
Dataset Management Tools

功能:
- 数据集分割 (train/val/test)
- 数据统计分析
- 格式转换
- 数据集可视化
- 质量检查
"""

import os
import json
import numpy as np
import pickle
from typing import Dict, List, Tuple, Optional
from pathlib import Path
from collections import defaultdict
import shutil
from datetime import datetime


class DatasetManager:
    """数据集管理器"""
    
    def __init__(self, dataset_dir: str):
        """
        初始化数据集管理器
        
        参数:
            dataset_dir: 数据集目录
        """
        self.dataset_dir = Path(dataset_dir)
        
        if not self.dataset_dir.exists():
            raise ValueError(f"数据集目录不存在: {dataset_dir}")
        
        # 扫描数据集
        self.episodes = self._scan_episodes()
        
        print(f"数据集管理器初始化完成")
        print(f"  目录: {dataset_dir}")
        print(f"  Episodes数量: {len(self.episodes)}")
    
    def _scan_episodes(self) -> List[Path]:
        """扫描数据集中的所有episodes"""
        episodes = []
        
        # 查找所有episode文件
        for format_ext in ['.json', '.pkl']:
            episodes.extend(self.dataset_dir.glob(f"episode_*{format_ext}"))
        
        # 去重（同一episode可能有多种格式）
        episode_ids = set()
        unique_episodes = []
        
        for ep in episodes:
            ep_id = ep.stem  # 不带扩展名的文件名
            if ep_id not in episode_ids:
                episode_ids.add(ep_id)
                unique_episodes.append(ep)
        
        return sorted(unique_episodes)
    
    def split_dataset(self, train_ratio: float = 0.7, val_ratio: float = 0.15,
                     test_ratio: float = 0.15, seed: int = 42) -> Dict[str, List]:
        """
        分割数据集
        
        参数:
            train_ratio: 训练集比例
            val_ratio: 验证集比例
            test_ratio: 测试集比例
            seed: 随机种子
        
        返回:
            分割后的数据集索引
        """
        assert abs(train_ratio + val_ratio + test_ratio - 1.0) < 1e-6, \
            "比例之和必须为1"
        
        np.random.seed(seed)
        
        # 打乱
        indices = np.random.permutation(len(self.episodes))
        
        # 计算分割点
        n_train = int(len(indices) * train_ratio)
        n_val = int(len(indices) * val_ratio)
        
        train_idx = indices[:n_train]
        val_idx = indices[n_train:n_train+n_val]
        test_idx = indices[n_train+n_val:]
        
        # 创建分割目录
        splits = {
            'train': train_idx,
            'val': val_idx,
            'test': test_idx
        }
        
        for split_name, split_indices in splits.items():
            split_dir = self.dataset_dir / split_name
            split_dir.mkdir(exist_ok=True)
            
            # 复制或链接文件
            for idx in split_indices:
                episode_file = self.episodes[idx]
                
                # 查找所有相关格式
                for ext in ['.json', '.pkl']:
                    src = self.dataset_dir / f"{episode_file.stem}{ext}"
                    if src.exists():
                        dst = split_dir / src.name
                        if not dst.exists():
                            shutil.copy2(src, dst)
        
        print("\n数据集分割完成:")
        print(f"  训练集: {len(train_idx)} episodes")
        print(f"  验证集: {len(val_idx)} episodes")
        print(f"  测试集: {len(test_idx)} episodes")
        
        # 保存分割信息
        split_info = {
            'train_ratio': train_ratio,
            'val_ratio': val_ratio,
            'test_ratio': test_ratio,
            'seed': seed,
            'train_count': len(train_idx),
            'val_count': len(val_idx),
            'test_count': len(test_idx),
            'timestamp': datetime.now().isoformat()
        }
        
        with open(self.dataset_dir / 'split_info.json', 'w') as f:
            json.dump(split_info, f, indent=2)
        
        return splits
    
    def compute_statistics(self) -> Dict:
        """计算数据集统计信息"""
        stats = {
            'num_episodes': len(self.episodes),
            'total_steps': 0,
            'avg_episode_length': 0,
            'total_rewards': [],
            'success_rate': 0,
            'parameter_ranges': defaultdict(lambda: {'min': float('inf'), 'max': float('-inf')})
        }
        
        successful_episodes = 0
        
        # 遍历所有episodes
        for episode_file in self.episodes:
            # 尝试加载pickle格式（包含更多信息）
            pkl_file = self.dataset_dir / f"{episode_file.stem}.pkl"
            
            if pkl_file.exists():
                try:
                    with open(pkl_file, 'rb') as f:
                        data = pickle.load(f)
                    
                    # 统计步数
                    episode_length = data.get('episode_length', 0)
                    stats['total_steps'] += episode_length
                    
                    # 统计奖励
                    total_reward = data.get('total_reward', 0)
                    stats['total_rewards'].append(total_reward)
                    
                    # 成功率（简单定义：reward > 0）
                    if total_reward > 0:
                        successful_episodes += 1
                    
                    # 参数范围
                    robot_params = data['config']['robot_params']
                    for param, value in robot_params.items():
                        stats['parameter_ranges'][param]['min'] = min(
                            stats['parameter_ranges'][param]['min'], value
                        )
                        stats['parameter_ranges'][param]['max'] = max(
                            stats['parameter_ranges'][param]['max'], value
                        )
                
                except Exception as e:
                    print(f"警告: 无法加载 {pkl_file}: {e}")
        
        # 计算平均值
        if stats['num_episodes'] > 0:
            stats['avg_episode_length'] = stats['total_steps'] / stats['num_episodes']
            stats['success_rate'] = successful_episodes / stats['num_episodes'] * 100
        
        if stats['total_rewards']:
            stats['avg_reward'] = np.mean(stats['total_rewards'])
            stats['std_reward'] = np.std(stats['total_rewards'])
            stats['min_reward'] = np.min(stats['total_rewards'])
            stats['max_reward'] = np.max(stats['total_rewards'])
        
        # 转换parameter_ranges为普通字典
        stats['parameter_ranges'] = dict(stats['parameter_ranges'])
        
        return stats
    
    def generate_report(self, output_file: Optional[str] = None) -> str:
        """生成数据集报告"""
        stats = self.compute_statistics()
        
        report = []
        report.append("="*70)
        report.append("数据集统计报告")
        report.append("="*70)
        
        report.append(f"\n数据集目录: {self.dataset_dir}")
        report.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        report.append(f"\n基本信息:")
        report.append(f"  Episodes数量: {stats['num_episodes']}")
        report.append(f"  总步数: {stats['total_steps']:,}")
        report.append(f"  平均Episode长度: {stats['avg_episode_length']:.1f}")
        
        if 'avg_reward' in stats:
            report.append(f"\n奖励统计:")
            report.append(f"  平均奖励: {stats['avg_reward']:.2f}")
            report.append(f"  标准差: {stats['std_reward']:.2f}")
            report.append(f"  最小值: {stats['min_reward']:.2f}")
            report.append(f"  最大值: {stats['max_reward']:.2f}")
            report.append(f"  成功率: {stats['success_rate']:.1f}%")
        
        if stats['parameter_ranges']:
            report.append(f"\n参数范围:")
            for param, ranges in stats['parameter_ranges'].items():
                report.append(f"  {param}:")
                report.append(f"    最小: {ranges['min']:.3f}")
                report.append(f"    最大: {ranges['max']:.3f}")
        
        report_text = "\n".join(report)
        
        # 保存报告
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(report_text)
            print(f"\n报告已保存到: {output_file}")
        
        return report_text
    
    def convert_format(self, output_format: str = 'numpy',
                      output_dir: Optional[str] = None):
        """
        转换数据集格式
        
        参数:
            output_format: 输出格式 (numpy, hdf5, tfrecord)
            output_dir: 输出目录
        """
        if output_dir is None:
            output_dir = self.dataset_dir / f"converted_{output_format}"
        
        output_path = Path(output_dir)
        output_path.mkdir(parents=True, exist_ok=True)
        
        print(f"\n转换格式为: {output_format}")
        print(f"输出目录: {output_dir}")
        
        if output_format == 'numpy':
            self._convert_to_numpy(output_path)
        elif output_format == 'hdf5':
            self._convert_to_hdf5(output_path)
        else:
            print(f"不支持的格式: {output_format}")
    
    def _convert_to_numpy(self, output_dir: Path):
        """转换为numpy格式"""
        all_states = []
        all_actions = []
        all_rewards = []
        
        for episode_file in self.episodes:
            pkl_file = self.dataset_dir / f"{episode_file.stem}.pkl"
            
            if pkl_file.exists():
                try:
                    with open(pkl_file, 'rb') as f:
                        data = pickle.load(f)
                    
                    all_states.extend(data['states'])
                    all_actions.extend(data['actions'])
                    all_rewards.extend(data['rewards'])
                
                except Exception as e:
                    print(f"警告: 跳过 {pkl_file}: {e}")
        
        # 保存为numpy数组
        if all_states:
            np.save(output_dir / 'states.npy', all_states)
            np.save(output_dir / 'actions.npy', all_actions)
            np.save(output_dir / 'rewards.npy', all_rewards)
            
            print(f"转换完成:")
            print(f"  状态数: {len(all_states)}")
            print(f"  动作数: {len(all_actions)}")
            print(f"  奖励数: {len(all_rewards)}")
    
    def _convert_to_hdf5(self, output_dir: Path):
        """转换为HDF5格式"""
        try:
            import h5py
        except ImportError:
            print("需要安装h5py: pip install h5py")
            return
        
        output_file = output_dir / 'dataset.h5'
        
        with h5py.File(output_file, 'w') as f:
            for i, episode_file in enumerate(self.episodes):
                pkl_file = self.dataset_dir / f"{episode_file.stem}.pkl"
                
                if pkl_file.exists():
                    try:
                        with open(pkl_file, 'rb') as pf:
                            data = pickle.load(pf)
                        
                        # 创建episode组
                        ep_group = f.create_group(f'episode_{i:06d}')
                        
                        # 保存数据
                        ep_group.create_dataset('states', data=data['states'])
                        ep_group.create_dataset('actions', data=data['actions'])
                        ep_group.create_dataset('rewards', data=data['rewards'])
                        
                        # 保存元数据
                        ep_group.attrs['total_reward'] = data['total_reward']
                        ep_group.attrs['episode_length'] = data['episode_length']
                    
                    except Exception as e:
                        print(f"警告: 跳过 {pkl_file}: {e}")
        
        print(f"HDF5文件已保存: {output_file}")


if __name__ == "__main__":
    print("数据集管理工具加载完成")
    
    # 示例
    # manager = DatasetManager("data/test_batch")
    # 
    # # 生成报告
    # report = manager.generate_report("dataset_report.txt")
    # print(report)
    # 
    # # 分割数据集
    # manager.split_dataset(train_ratio=0.7, val_ratio=0.15, test_ratio=0.15)
    # 
    # # 转换格式
    # manager.convert_format('numpy')
