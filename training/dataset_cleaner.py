"""
数据集清洗工具
去重、过滤、类别平衡和数据增强
"""

import json
import random
from pathlib import Path
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from collections import Counter
import numpy as np
import hashlib


@dataclass
class CleaningStats:
    """清洗统计"""
    original_count: int = 0
    after_dedup: int = 0
    after_filter: int = 0
    after_balance: int = 0
    after_augment: int = 0


class DatasetCleaner:
    """
    数据集清洗工具
    
    功能：
    1. 去重
    2. 质量过滤
    3. 类别平衡
    4. 数据增强
    """
    
    def __init__(self, seed: int = 42):
        random.seed(seed)
        np.random.seed(seed)
        self.stats = CleaningStats()
    
    def clean_pipeline(
        self,
        dataset: List[dict],
        quality_threshold: float = 0.3,
        balance: bool = True,
        augment_factor: int = 1
    ) -> List[dict]:
        """
        完整清洗管道
        
        Args:
            dataset: 原始数据集
            quality_threshold: 质量阈值
            balance: 是否平衡类别
            augment_factor: 增强倍数
        
        Returns:
            清洗后的数据集
        """
        print(f"🧹 开始数据清洗...")
        print(f"   原始样本数: {len(dataset)}")
        
        self.stats.original_count = len(dataset)
        
        # 1. 去重
        dataset = self.remove_duplicates(dataset)
        self.stats.after_dedup = len(dataset)
        print(f"   去重后: {len(dataset)}")
        
        # 2. 质量过滤
        dataset = self.filter_by_quality(dataset, quality_threshold)
        self.stats.after_filter = len(dataset)
        print(f"   过滤后: {len(dataset)}")
        
        # 3. 类别平衡
        if balance:
            dataset = self.balance_classes(dataset)
            self.stats.after_balance = len(dataset)
            print(f"   平衡后: {len(dataset)}")
        
        # 4. 数据增强
        if augment_factor > 1:
            dataset = self.augment(dataset, augment_factor)
            self.stats.after_augment = len(dataset)
            print(f"   增强后: {len(dataset)}")
        
        print(f"✅ 清洗完成")
        return dataset
    
    def remove_duplicates(self, dataset: List[dict]) -> List[dict]:
        """去重"""
        seen_hashes = set()
        unique_samples = []
        
        for sample in dataset:
            # 计算样本哈希
            sample_hash = self._compute_hash(sample)
            
            if sample_hash not in seen_hashes:
                seen_hashes.add(sample_hash)
                unique_samples.append(sample)
        
        return unique_samples
    
    def _compute_hash(self, sample: dict) -> str:
        """计算样本哈希"""
        # 使用关键字段计算哈希
        key_fields = ['label', 'metadata']
        hash_input = json.dumps(
            {k: sample.get(k) for k in key_fields},
            sort_keys=True
        )
        return hashlib.md5(hash_input.encode()).hexdigest()
    
    def filter_by_quality(
        self,
        dataset: List[dict],
        threshold: float = 0.3
    ) -> List[dict]:
        """按质量过滤"""
        filtered = []
        
        for sample in dataset:
            quality = sample.get('quality_score', 0.5)
            
            if quality >= threshold:
                filtered.append(sample)
        
        return filtered
    
    def filter_by_condition(
        self,
        dataset: List[dict],
        condition: Callable[[dict], bool]
    ) -> List[dict]:
        """按自定义条件过滤"""
        return [s for s in dataset if condition(s)]
    
    def balance_classes(
        self,
        dataset: List[dict],
        strategy: str = "oversample"
    ) -> List[dict]:
        """
        类别平衡
        
        Args:
            dataset: 数据集
            strategy: 策略 ("oversample", "undersample", "both")
        
        Returns:
            平衡后的数据集
        """
        # 统计类别分布
        label_counts = Counter(s.get('label', 'unknown') for s in dataset)
        
        if not label_counts:
            return dataset
        
        # 按类别分组
        by_label = {}
        for sample in dataset:
            label = sample.get('label', 'unknown')
            if label not in by_label:
                by_label[label] = []
            by_label[label].append(sample)
        
        if strategy == "undersample":
            # 下采样到最小类别数量
            min_count = min(label_counts.values())
            balanced = []
            for label, samples in by_label.items():
                balanced.extend(random.sample(samples, min(len(samples), min_count)))
        
        elif strategy == "oversample":
            # 上采样到最大类别数量
            max_count = max(label_counts.values())
            balanced = []
            for label, samples in by_label.items():
                if len(samples) < max_count:
                    # 重复采样
                    oversampled = samples.copy()
                    while len(oversampled) < max_count:
                        oversampled.append(random.choice(samples))
                    balanced.extend(oversampled)
                else:
                    balanced.extend(samples)
        
        else:  # both
            # 采样到中位数
            median_count = int(np.median(list(label_counts.values())))
            balanced = []
            for label, samples in by_label.items():
                if len(samples) > median_count:
                    balanced.extend(random.sample(samples, median_count))
                elif len(samples) < median_count:
                    oversampled = samples.copy()
                    while len(oversampled) < median_count:
                        oversampled.append(random.choice(samples))
                    balanced.extend(oversampled)
                else:
                    balanced.extend(samples)
        
        random.shuffle(balanced)
        return balanced
    
    def augment(
        self,
        dataset: List[dict],
        factor: int = 2
    ) -> List[dict]:
        """
        数据增强
        
        Args:
            dataset: 数据集
            factor: 增强倍数
        
        Returns:
            增强后的数据集
        """
        augmented = dataset.copy()
        
        for _ in range(factor - 1):
            for sample in dataset:
                aug_sample = self._augment_sample(sample)
                augmented.append(aug_sample)
        
        return augmented
    
    def _augment_sample(self, sample: dict) -> dict:
        """增强单个样本"""
        import copy
        aug = copy.deepcopy(sample)
        
        # 修改ID
        aug['id'] = f"{sample.get('id', 'sample')}_aug_{random.randint(1000, 9999)}"
        
        # 添加噪声到元数据
        metadata = aug.get('metadata', {})
        
        if 'max_roll' in metadata:
            metadata['max_roll'] += random.gauss(0, 1)
        if 'max_pitch' in metadata:
            metadata['max_pitch'] += random.gauss(0, 1)
        if 'duration' in metadata:
            metadata['duration'] = max(1, metadata['duration'] + random.randint(-5, 5))
        
        aug['metadata'] = metadata
        aug['_augmented'] = True
        
        return aug
    
    def split_dataset(
        self,
        dataset: List[dict],
        train_ratio: float = 0.8,
        val_ratio: float = 0.1,
        test_ratio: float = 0.1
    ) -> tuple:
        """划分数据集"""
        assert abs(train_ratio + val_ratio + test_ratio - 1.0) < 0.01
        
        # 打乱
        shuffled = dataset.copy()
        random.shuffle(shuffled)
        
        n = len(shuffled)
        train_end = int(n * train_ratio)
        val_end = int(n * (train_ratio + val_ratio))
        
        train = shuffled[:train_end]
        val = shuffled[train_end:val_end]
        test = shuffled[val_end:]
        
        return train, val, test
    
    def get_label_distribution(self, dataset: List[dict]) -> Dict[str, int]:
        """获取类别分布"""
        return dict(Counter(s.get('label', 'unknown') for s in dataset))
    
    def save_dataset(
        self,
        dataset: List[dict],
        output_path: str
    ):
        """保存数据集"""
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(dataset, f, indent=2, ensure_ascii=False)
        print(f"💾 保存到: {output_path}")
    
    def get_stats(self) -> dict:
        """获取清洗统计"""
        return {
            "original_count": self.stats.original_count,
            "after_dedup": self.stats.after_dedup,
            "after_filter": self.stats.after_filter,
            "after_balance": self.stats.after_balance,
            "after_augment": self.stats.after_augment,
            "dedup_removed": self.stats.original_count - self.stats.after_dedup,
            "filter_removed": self.stats.after_dedup - self.stats.after_filter
        }


# 测试代码
if __name__ == "__main__":
    print("数据集清洗工具测试\n")
    
    # 创建清洗器
    cleaner = DatasetCleaner(seed=42)
    
    # 模拟数据集
    dataset = []
    labels = ["successful_gait", "fall_forward", "unstable_balance", "energy_inefficient"]
    
    # 不平衡分布
    for i in range(100):
        label = random.choices(labels, weights=[60, 15, 15, 10])[0]
        dataset.append({
            "id": f"sample_{i}",
            "label": label,
            "quality_score": random.uniform(0.2, 1.0),
            "metadata": {
                "max_roll": random.uniform(0, 30),
                "max_pitch": random.uniform(0, 30),
                "duration": random.randint(10, 200)
            }
        })
    
    # 添加重复
    dataset.append(dataset[0].copy())
    dataset.append(dataset[1].copy())
    
    print("=== 原始分布 ===")
    print(cleaner.get_label_distribution(dataset))
    
    # 清洗
    cleaned = cleaner.clean_pipeline(
        dataset,
        quality_threshold=0.3,
        balance=True,
        augment_factor=1
    )
    
    print("\n=== 清洗后分布 ===")
    print(cleaner.get_label_distribution(cleaned))
    
    # 划分
    train, val, test = cleaner.split_dataset(cleaned)
    print(f"\n=== 数据划分 ===")
    print(f"训练集: {len(train)}")
    print(f"验证集: {len(val)}")
    print(f"测试集: {len(test)}")
    
    # 统计
    print("\n=== 清洗统计 ===")
    print(json.dumps(cleaner.get_stats(), indent=2))
