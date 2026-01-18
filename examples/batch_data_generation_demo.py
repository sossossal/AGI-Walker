"""
批量数据生成示例
演示如何使用批量生成器生成大规模训练数据
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_api.batch_generator import BatchDataGenerator, GenerationConfig
from python_api.dataset_manager import DatasetManager


def demo_small_batch():
    """演示: 生成小规模数据集"""
    print("="*70)
    print("演示1: 小规模数据集生成")
    print("="*70)
    
    config = GenerationConfig(
        num_episodes=20,
        episode_length=100,
        dt=0.01,
        num_workers=2,
        output_dir="data/demo_small",
        format="both",
        environments=['flat', 'uphill_5deg', 'windy']
    )
    
    generator = BatchDataGenerator(config)
    report = generator.generate()
    
    return config.output_dir


def demo_medium_batch():
    """演示: 生成中等规模数据集"""
    print("\n" + "="*70)
    print("演示2: 中等规模数据集生成")
    print("="*70)
    
    config = GenerationConfig(
        num_episodes=100,
        episode_length=500,
        dt=0.01,
        num_workers=4,
        output_dir="data/demo_medium",
        format="pickle",  # pickle更紧凑
        environments=['flat', 'uphill_5deg', 'downhill_5deg', 'windy', 'icy']
    )
    
    generator = BatchDataGenerator(config)
    report = generator.generate()
    
    return config.output_dir


def demo_dataset_management():
    """演示: 数据集管理"""
    print("\n" + "="*70)
    print("演示3: 数据集管理")
    print("="*70)
    
    # 使用小规模数据集演示
    dataset_dir = "data/demo_small"
    
    if not os.path.exists(dataset_dir):
        print(f"数据集目录不存在: {dataset_dir}")
        print("请先运行 demo_small_batch()")
        return
    
    # 创建管理器
    manager = DatasetManager(dataset_dir)
    
    # 1. 生成统计报告
    print("\n" + "-"*70)
    print("1. 统计报告")
    print("-"*70)
    report = manager.generate_report(f"{dataset_dir}/dataset_report.txt")
    print(report)
    
    # 2. 分割数据集
    print("\n" + "-"*70)
    print("2. 分割数据集")
    print("-"*70)
    splits = manager.split_dataset(
        train_ratio=0.7,
        val_ratio=0.15,
        test_ratio=0.15,
        seed=42
    )
    
    # 3. 格式转换
    print("\n" + "-"*70)
    print("3. 格式转换")
    print("-"*70)
    manager.convert_format('numpy', f"{dataset_dir}/numpy_format")


def demo_resume():
    """演示: 断点续传"""
    print("\n" + "="*70)
    print("演示4: 断点续传")
    print("="*70)
    
    # 第一次生成（模拟中断）
    config = GenerationConfig(
        num_episodes=50,
        episode_length=100,
        num_workers=2,
        output_dir="data/demo_resume",
        format="pickle",
        checkpoint_interval=5
    )
    
    print("\n第一次运行 (生成前25个)...")
    config.num_episodes = 25
    generator = BatchDataGenerator(config)
    generator.generate()
    
    # 模拟恢复
    print("\n模拟系统中断...")
    print("恢复运行 (继续生成剩余25个)...")
    
    config.num_episodes = 50
    config.resume = True
    generator2 = BatchDataGenerator(config)
    generator2.generate()


def demo_parameter_sweep():
    """演示: 参数扫描"""
    print("\n" + "="*70)
    print("演示5: 参数扫描 (系统化采样)")
    print("="*70)
    
    # 创建多个配置，系统化地扫描参数空间
    configs = []
    
    # 扫描电机功率
    for power in [0.8, 1.0, 1.2, 1.5]:
        config = GenerationConfig(
            num_episodes=10,
            episode_length=100,
            num_workers=2,
            output_dir=f"data/sweep_power_{power:.1f}",
            format="pickle",
            motor_power_range=(power, power),  # 固定值
            environments=['flat']
        )
        configs.append(config)
    
    # 批量运行
    for i, cfg in enumerate(configs, 1):
        print(f"\n运行配置 {i}/{len(configs)}: 功率={cfg.motor_power_range[0]:.1f}")
        generator = BatchDataGenerator(cfg)
        generator.generate()


def main():
    """主演示函数"""
    print("\n" + "="*70)
    print("批量数据生成演示")
    print("="*70)
    
    print("\n选择演示:")
    print("  1. 小规模数据集 (20 episodes, 快速)")
    print("  2. 中等规模数据集 (100 episodes)")
    print("  3. 数据集管理 (分割、统计、转换)")
    print("  4. 断点续传演示")
    print("  5. 参数扫描")
    print("  6. 运行所有演示")
    
    choice = input("\n输入选择 (1-6): ").strip()
    
    if choice == '1':
        demo_small_batch()
        print("\n提示: 可以运行演示3来管理生成的数据集")
    elif choice == '2':
        demo_medium_batch()
    elif choice == '3':
        demo_dataset_management()
    elif choice == '4':
        demo_resume()
    elif choice == '5':
        demo_parameter_sweep()
    elif choice == '6':
        print("\n运行所有演示...")
        dataset_dir = demo_small_batch()
        demo_dataset_management()
        demo_resume()
    else:
        print("无效选择")
    
    print("\n" + "="*70)
    print("演示完成!")
    print("="*70)


if __name__ == "__main__":
    main()
