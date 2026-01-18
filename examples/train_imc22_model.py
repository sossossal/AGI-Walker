"""
IMC-22训练脚本
使用AGI-Walker数据训练控制网络

使用方法:
    python examples/train_imc22_model.py
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import torch
from torch.utils.data import DataLoader, TensorDataset
import numpy as np
from pathlib import Path

from models.imc22_control_net import IMC22ControlNet, IMC22Trainer


def load_dataset(dataset_dir: str):
    """加载数据集"""
    dataset_path = Path(dataset_dir)
    
    print("加载数据集...")
    
    # 加载训练集
    train_states = np.load(dataset_path / 'train' / 'states.npy')
    train_actions = np.load(dataset_path / 'train' / 'actions.npy')
    
    # 加载验证集
    val_states = np.load(dataset_path / 'val' / 'states.npy')
    val_actions = np.load(dataset_path / 'val' / 'actions.npy')
    
    print(f"  训练集: {len(train_states):,} 样本")
    print(f"  验证集: {len(val_states):,} 样本")
    
    return (train_states, train_actions), (val_states, val_actions)


def create_data_loaders(train_data, val_data, batch_size: int = 256):
    """创建数据加载器"""
    train_states, train_actions = train_data
    val_states, val_actions = val_data
    
    # 转换为Tensor
    train_states_tensor = torch.FloatTensor(train_states)
    train_actions_tensor = torch.FloatTensor(train_actions)
    val_states_tensor = torch.FloatTensor(val_states)
    val_actions_tensor = torch.FloatTensor(val_actions)
    
    # 创建Dataset
    train_dataset = TensorDataset(train_states_tensor, train_actions_tensor)
    val_dataset = TensorDataset(val_states_tensor, val_actions_tensor)
    
    # 创建DataLoader
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
    
    return train_loader, val_loader


def main():
    print("="*70)
    print("IMC-22控制网络训练")
    print("="*70)
    
    # 配置
    dataset_dir = "data/imc22_dataset"
    model_save_dir = "models"
    
    # 检查数据集
    if not Path(dataset_dir).exists():
        print(f"\n错误: 数据集目录不存在: {dataset_dir}")
        print("请先运行数据准备脚本:")
        print("  python tools/imc22_data_preparer.py <input_dir> data/imc22_dataset")
        return
    
    # 加载数据
    train_data, val_data = load_dataset(dataset_dir)
    
    # 创建数据加载器
    train_loader, val_loader = create_data_loaders(train_data, val_data, batch_size=256)
    
    # 创建模型
    print("\n创建模型...")
    model = IMC22ControlNet(state_dim=3, action_dim=3, hidden_dim=16)
    model.print_model_info()
    
    # 创建训练器
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f"\n使用设备: {device}")
    
    trainer = IMC22Trainer(model, device=device)
    trainer.setup_optimizer(lr=0.001)
    
    # 训练
    trainer.train(train_loader, val_loader, epochs=100)
    
    # 保存最终模型
    final_model_path = Path(model_save_dir) / 'imc22_control_net_final.pth'
    trainer.save_checkpoint(str(final_model_path))
    print(f"\n模型已保存到: {final_model_path}")
    
    # 测试推理
    print("\n测试推理...")
    model.eval()
    
    test_states = torch.FloatTensor([
        [0.5, 0.3, 1.0],   # 正常状态
        [0.1, 0.1, 1.0],   # 慢速
        [0.8, 0.8, 0.0],   # 不稳定
    ]).to(device)
    
    with torch.no_grad():
        test_actions = model(test_states)
    
    print("\n示例预测:")
    for i, (state, action) in enumerate(zip(test_states.cpu().numpy(), 
                                            test_actions.cpu().numpy())):
        print(f"  状态 {i+1}: {state} → 动作: {action}")
    
    print("\n✓ 训练完成!")


if __name__ == "__main__":
    main()
