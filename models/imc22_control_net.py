"""
IMC-22控制网络模型
为IMC-22 NPU设计的轻量级神经网络

特点:
- 参数量 < 50KB (适应256KB SRAM)
- 支持INT8量化
- 只使用NPU友好的算子 (全连接 + ReLU)
- 实时推理 < 2ms
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from pathlib import Path
import json


class IMC22ControlNet(nn.Module):
    """
    IMC-22控制网络
    
    输入: [位置, 速度, 稳定性] (3维)
    输出: [电机功率, 刚度, 阻尼] (3维)
    
    约束:
    - 总参数 < 10KB (INT8)
    - 延迟 < 2ms @ 100MHz
    - 能效 > 30 TOPs/W
    """
    
    def __init__(self, state_dim: int = 3, action_dim: int = 3, 
                 hidden_dim: int = 16):
        super().__init__()
        
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.hidden_dim = hidden_dim
        
        # 网络层
        self.fc1 = nn.Linear(state_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, action_dim)
        
        # 初始化权重
        self._initialize_weights()
    
    def _initialize_weights(self):
        """初始化权重（适合量化）"""
        for m in self.modules():
            if isinstance(m, nn.Linear):
                # Xavier初始化
                nn.init.xavier_uniform_(m.weight)
                if m.bias is not None:
                    nn.init.constant_(m.bias, 0)
    
    def forward(self, x):
        """前向传播"""
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = torch.tanh(self.fc3(x))  # 输出 [-1, 1]
        return x
    
    def count_parameters(self):
        """统计参数数量"""
        return sum(p.numel() for p in self.parameters())
    
    def estimate_memory_usage(self):
        """估算内存使用"""
        # FP32参数
        params_fp32 = self.count_parameters() * 4  # 4 bytes per float
        
        # INT8参数
        params_int8 = self.count_parameters()  # 1 byte per int8
        
        # 激活值（最大hidden_dim个FP32）
        activations = self.hidden_dim * 4
        
        return {
            'parameters_fp32_bytes': params_fp32,
            'parameters_int8_bytes': params_int8,
            'activations_bytes': activations,
            'total_fp32_bytes': params_fp32 + activations,
            'total_int8_bytes': params_int8 + activations
        }
    
    def print_model_info(self):
        """打印模型信息"""
        print("="*70)
        print("IMC-22控制网络模型信息")
        print("="*70)
        
        print(f"\n网络结构:")
        print(f"  输入维度: {self.state_dim}")
        print(f"  隐藏维度: {self.hidden_dim}")
        print(f"  输出维度: {self.action_dim}")
        
        print(f"\n参数统计:")
        total_params = self.count_parameters()
        print(f"  总参数: {total_params:,}")
        
        for name, param in self.named_parameters():
            print(f"    {name}: {param.numel():,} ({tuple(param.shape)})")
        
        print(f"\n内存使用:")
        mem = self.estimate_memory_usage()
        print(f"  FP32模型: {mem['total_fp32_bytes']:,} bytes ({mem['total_fp32_bytes']/1024:.1f} KB)")
        print(f"  INT8模型: {mem['total_int8_bytes']:,} bytes ({mem['total_int8_bytes']/1024:.1f} KB)")
        
        # 检查是否适合IMC-22
        sram_limit = 256 * 1024  # 256KB
        if mem['total_int8_bytes'] < sram_limit:
            print(f"  ✓ 适合IMC-22 SRAM ({sram_limit/1024:.0f}KB)")
        else:
            print(f"  ✗ 超出IMC-22 SRAM ({sram_limit/1024:.0f}KB)")


class IMC22Trainer:
    """IMC-22模型训练器"""
    
    def __init__(self, model: IMC22ControlNet, device: str = 'cpu'):
        self.model = model.to(device)
        self.device = device
        self.optimizer = None
        self.criterion = nn.MSELoss()
        self.history = {
            'train_loss': [],
            'val_loss': []
        }
    
    def setup_optimizer(self, lr: float = 0.001):
        """设置优化器"""
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=lr)
    
    def train_epoch(self, train_loader):
        """训练一个epoch"""
        self.model.train()
        total_loss = 0
        
        for states, actions in train_loader:
            states = states.to(self.device)
            actions = actions.to(self.device)
            
            # 前向传播
            self.optimizer.zero_grad()
            predictions = self.model(states)
            loss = self.criterion(predictions, actions)
            
            # 反向传播
            loss.backward()
            self.optimizer.step()
            
            total_loss += loss.item()
        
        avg_loss = total_loss / len(train_loader)
        return avg_loss
    
    def validate(self, val_loader):
        """验证"""
        self.model.eval()
        total_loss = 0
        
        with torch.no_grad():
            for states, actions in val_loader:
                states = states.to(self.device)
                actions = actions.to(self.device)
                
                predictions = self.model(states)
                loss = self.criterion(predictions, actions)
                
                total_loss += loss.item()
        
        avg_loss = total_loss / len(val_loader)
        return avg_loss
    
    def train(self, train_loader, val_loader, epochs: int = 100):
        """完整训练流程"""
        print("\n开始训练...")
        
        best_val_loss = float('inf')
        
        for epoch in range(epochs):
            train_loss = self.train_epoch(train_loader)
            val_loss = self.validate(val_loader)
            
            self.history['train_loss'].append(train_loss)
            self.history['val_loss'].append(val_loss)
            
            if (epoch + 1) % 10 == 0:
                print(f"Epoch {epoch+1}/{epochs} - "
                      f"Train Loss: {train_loss:.6f}, Val Loss: {val_loss:.6f}")
            
            # 保存最佳模型
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                self.save_checkpoint('models/imc22_control_net_best.pth')
        
        print(f"\n训练完成! 最佳验证损失: {best_val_loss:.6f}")
    
    def save_checkpoint(self, filepath: str):
        """保存检查点"""
        Path(filepath).parent.mkdir(parents=True, exist_ok=True)
        
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'history': self.history
        }, filepath)
    
    def load_checkpoint(self, filepath: str):
        """加载检查点"""
        checkpoint = torch.load(filepath)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        if self.optimizer:
            self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.history = checkpoint['history']


if __name__ == "__main__":
    print("IMC-22控制网络模型")
    print("="*70)
    
    # 创建模型
    model = IMC22ControlNet(state_dim=3, action_dim=3, hidden_dim=16)
    
    # 打印信息
    model.print_model_info()
    
    # 测试前向传播
    print("\n测试推理:")
    test_state = torch.randn(1, 3)
    test_action = model(test_state)
    print(f"  输入: {test_state.numpy()[0]}")
    print(f"  输出: {test_action.detach().numpy()[0]}")
    
    print("\n✓ 模型创建成功!")
