"""
Sim2Real Gap 估计与校正模块
使用机器学习估计仿真与真实世界的差距，动态调整仿真参数
"""

import json
import time
import pickle
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
import numpy as np

# 延迟导入PyTorch（可能未安装）
torch = None
nn = None


def _init_torch():
    """延迟初始化PyTorch"""
    global torch, nn
    if torch is None:
        try:
            import torch as _torch
            import torch.nn as _nn
            torch = _torch
            nn = _nn
            return True
        except ImportError:
            print("⚠️ PyTorch未安装，Gap校正模型将不可用")
            return False
    return True


@dataclass
class PhysicsParams:
    """物理参数集"""
    friction_coefficient: float = 0.8       # 摩擦系数
    damping_ratio: float = 0.1              # 阻尼比
    contact_stiffness: float = 1000.0       # 接触刚度
    joint_friction: float = 0.05            # 关节摩擦
    gravity: float = 9.8                    # 重力加速度
    mass_scale: float = 1.0                 # 质量缩放
    
    def to_dict(self) -> dict:
        return {
            "friction_coefficient": self.friction_coefficient,
            "damping_ratio": self.damping_ratio,
            "contact_stiffness": self.contact_stiffness,
            "joint_friction": self.joint_friction,
            "gravity": self.gravity,
            "mass_scale": self.mass_scale
        }
    
    @classmethod
    def from_dict(cls, d: dict) -> 'PhysicsParams':
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


@dataclass
class Sim2RealDataPair:
    """仿真-真实数据对"""
    timestamp: float
    sim_state: Dict
    real_state: Optional[Dict] = None
    physics_params: Optional[Dict] = None


class GapCorrectionNet:
    """
    Sim2Real Gap校正网络
    
    输入: 仿真状态向量 (12维: 6 IMU + 6 关节)
    输出: 参数修正量 (6维: 摩擦、阻尼、刚度、关节摩擦、重力、质量)
    """
    
    def __init__(self, input_dim: int = 12, output_dim: int = 6):
        if not _init_torch():
            self.model = None
            return
        
        self.input_dim = input_dim
        self.output_dim = output_dim
        
        # 简单3层MLP
        self.model = nn.Sequential(
            nn.Linear(input_dim * 2, 32),  # sim + real 拼接
            nn.ReLU(),
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Linear(16, output_dim),
            nn.Tanh()  # 输出范围 [-1, 1]
        )
        
        # 输出缩放因子（各参数的调整范围）
        self.output_scales = torch.tensor([
            0.3,   # friction: ±0.3
            0.05,  # damping: ±0.05
            200.0, # stiffness: ±200
            0.02,  # joint_friction: ±0.02
            0.5,   # gravity: ±0.5
            0.1    # mass_scale: ±0.1
        ])
        
        # 训练器
        self.optimizer = None
        self.trained = False
    
    def forward(self, sim_state: np.ndarray, real_state: np.ndarray) -> np.ndarray:
        """前向传播"""
        if self.model is None:
            return np.zeros(self.output_dim)
        
        # 拼接输入
        sim_t = torch.tensor(sim_state, dtype=torch.float32)
        real_t = torch.tensor(real_state, dtype=torch.float32)
        x = torch.cat([sim_t, real_t], dim=-1)
        
        # 推理
        with torch.no_grad():
            raw_output = self.model(x)
            scaled_output = raw_output * self.output_scales
        
        return scaled_output.numpy()
    
    def train_step(self, sim_states: np.ndarray, real_states: np.ndarray, 
                   target_corrections: np.ndarray) -> float:
        """训练步骤"""
        if self.model is None:
            return 0.0
        
        if self.optimizer is None:
            self.optimizer = torch.optim.Adam(self.model.parameters(), lr=0.001)
        
        # 转换为tensor
        sim_t = torch.tensor(sim_states, dtype=torch.float32)
        real_t = torch.tensor(real_states, dtype=torch.float32)
        target_t = torch.tensor(target_corrections, dtype=torch.float32)
        
        x = torch.cat([sim_t, real_t], dim=-1)
        
        # 前向
        self.optimizer.zero_grad()
        output = self.model(x) * self.output_scales
        
        # MSE损失
        loss = nn.functional.mse_loss(output, target_t)
        
        # 反向
        loss.backward()
        self.optimizer.step()
        
        return loss.item()
    
    def save(self, path: str):
        """保存模型"""
        if self.model is None:
            return
        torch.save(self.model.state_dict(), path)
    
    def load(self, path: str):
        """加载模型"""
        if self.model is None:
            return
        self.model.load_state_dict(torch.load(path))
        self.trained = True


class Sim2RealGapEstimator:
    """
    Sim2Real Gap估计器
    
    功能：
    1. 收集仿真-真实数据对
    2. 训练Gap校正网络
    3. 在线估计并校正物理参数
    """
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        data_dir: str = "d:/新建文件夹/AGI-Walker/offline_data/sim2real"
    ):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
        # Gap校正网络
        self.gap_model = GapCorrectionNet()
        
        # 加载预训练模型
        if model_path and Path(model_path).exists():
            print(f"加载Sim2Real模型: {model_path}")
            self.gap_model.load(model_path)
        
        # 当前物理参数
        self.current_params = PhysicsParams()
        
        # 数据收集缓冲区
        self.data_buffer: List[Sim2RealDataPair] = []
        self.max_buffer_size = 10000
        
        # 在线校正状态
        self.correction_enabled = True
        self.correction_smoothing = 0.1  # 平滑因子
        self.last_correction = np.zeros(6)
        
        # 统计
        self.corrections_applied = 0
        self.data_pairs_collected = 0
    
    def collect_data_pair(
        self,
        sim_state: dict,
        real_state: Optional[dict] = None
    ):
        """
        收集数据对
        
        Args:
            sim_state: 仿真状态
            real_state: 真实状态（如果可用）
        """
        pair = Sim2RealDataPair(
            timestamp=time.time(),
            sim_state=sim_state,
            real_state=real_state,
            physics_params=self.current_params.to_dict()
        )
        
        self.data_buffer.append(pair)
        self.data_pairs_collected += 1
        
        # 限制缓冲区大小
        if len(self.data_buffer) > self.max_buffer_size:
            self.data_buffer.pop(0)
    
    def estimate_gap(self, sim_state: dict, real_state: dict) -> dict:
        """
        估计Sim2Real Gap
        
        Args:
            sim_state: 仿真状态
            real_state: 真实状态
        
        Returns:
            Gap估计结果
        """
        # 提取状态向量
        sim_vec = self._state_to_vector(sim_state)
        real_vec = self._state_to_vector(real_state)
        
        # 计算直接差异
        direct_gap = real_vec - sim_vec
        
        # 使用网络估计参数修正
        if self.gap_model.trained:
            correction = self.gap_model.forward(sim_vec, real_vec)
        else:
            correction = np.zeros(6)
        
        return {
            "direct_gap": direct_gap.tolist(),
            "state_rmse": float(np.sqrt(np.mean(direct_gap**2))),
            "param_correction": correction.tolist(),
            "correction_names": [
                "friction", "damping", "stiffness", 
                "joint_friction", "gravity", "mass_scale"
            ]
        }
    
    def correct_parameters(self, sim_state: dict, real_state: dict) -> PhysicsParams:
        """
        校正物理参数
        
        Args:
            sim_state: 仿真状态
            real_state: 真实状态
        
        Returns:
            校正后的物理参数
        """
        if not self.correction_enabled:
            return self.current_params
        
        # 估计修正量
        gap = self.estimate_gap(sim_state, real_state)
        correction = np.array(gap["param_correction"])
        
        # 平滑处理
        smoothed = self.correction_smoothing * correction + \
                   (1 - self.correction_smoothing) * self.last_correction
        self.last_correction = smoothed
        
        # 应用修正
        new_params = PhysicsParams(
            friction_coefficient=self.current_params.friction_coefficient + smoothed[0],
            damping_ratio=self.current_params.damping_ratio + smoothed[1],
            contact_stiffness=self.current_params.contact_stiffness + smoothed[2],
            joint_friction=self.current_params.joint_friction + smoothed[3],
            gravity=self.current_params.gravity + smoothed[4],
            mass_scale=self.current_params.mass_scale + smoothed[5]
        )
        
        # 参数限制
        new_params.friction_coefficient = max(0.1, min(1.5, new_params.friction_coefficient))
        new_params.damping_ratio = max(0.01, min(0.5, new_params.damping_ratio))
        new_params.contact_stiffness = max(100, min(5000, new_params.contact_stiffness))
        new_params.joint_friction = max(0.0, min(0.2, new_params.joint_friction))
        new_params.gravity = max(9.0, min(10.5, new_params.gravity))
        new_params.mass_scale = max(0.8, min(1.2, new_params.mass_scale))
        
        self.current_params = new_params
        self.corrections_applied += 1
        
        return new_params
    
    def update_friction(self, measured_friction: float):
        """直接更新摩擦系数"""
        self.current_params.friction_coefficient = max(0.1, min(1.5, measured_friction))
    
    def _state_to_vector(self, state: dict) -> np.ndarray:
        """将状态字典转换为向量"""
        sensors = state.get('sensors', {})
        imu = sensors.get('imu', {})
        joints = sensors.get('joints', {})
        
        orient = imu.get('orient', [0, 0, 0])
        gyro = imu.get('gyro', [0, 0, 0])
        
        hip_left = joints.get('hip_left', {}).get('angle', 0)
        hip_right = joints.get('hip_right', {}).get('angle', 0)
        hip_left_vel = joints.get('hip_left', {}).get('velocity', 0)
        hip_right_vel = joints.get('hip_right', {}).get('velocity', 0)
        
        height = state.get('torso_height', 1.0)
        
        return np.array([
            orient[0], orient[1], orient[2],
            gyro[0], gyro[1], gyro[2],
            hip_left, hip_right,
            hip_left_vel, hip_right_vel,
            height, 0  # 填充到12维
        ], dtype=np.float32)
    
    def train_from_buffer(self, epochs: int = 100) -> float:
        """从缓冲区训练模型"""
        if len(self.data_buffer) < 100:
            print("数据不足，需要至少100个数据对")
            return 0.0
        
        # 只使用有真实数据的样本
        valid_pairs = [p for p in self.data_buffer if p.real_state is not None]
        
        if len(valid_pairs) < 50:
            print("有效数据对不足")
            return 0.0
        
        print(f"开始训练，{len(valid_pairs)}个数据对，{epochs}轮")
        
        total_loss = 0.0
        for epoch in range(epochs):
            epoch_loss = 0.0
            
            for pair in valid_pairs:
                sim_vec = self._state_to_vector(pair.sim_state)
                real_vec = self._state_to_vector(pair.real_state)
                
                # 目标修正量（简化：使用直接差异作为监督信号）
                target = (real_vec - sim_vec)[:6] * 0.1
                
                loss = self.gap_model.train_step(
                    sim_vec.reshape(1, -1),
                    real_vec.reshape(1, -1),
                    target.reshape(1, -1)
                )
                epoch_loss += loss
            
            total_loss = epoch_loss / len(valid_pairs)
            
            if (epoch + 1) % 10 == 0:
                print(f"Epoch {epoch+1}/{epochs}, Loss: {total_loss:.6f}")
        
        self.gap_model.trained = True
        return total_loss
    
    def save_data(self, filename: str = "sim2real_data.pkl"):
        """保存收集的数据"""
        path = self.data_dir / filename
        with open(path, 'wb') as f:
            pickle.dump(self.data_buffer, f)
        print(f"保存{len(self.data_buffer)}条数据到: {path}")
    
    def load_data(self, filename: str = "sim2real_data.pkl"):
        """加载数据"""
        path = self.data_dir / filename
        if path.exists():
            with open(path, 'rb') as f:
                self.data_buffer = pickle.load(f)
            print(f"加载{len(self.data_buffer)}条数据")
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "corrections_applied": self.corrections_applied,
            "data_pairs_collected": self.data_pairs_collected,
            "buffer_size": len(self.data_buffer),
            "model_trained": self.gap_model.trained if self.gap_model.model else False,
            "current_params": self.current_params.to_dict()
        }


# 测试代码
if __name__ == "__main__":
    print("Sim2Real Gap估计器测试\n")
    
    # 创建估计器
    estimator = Sim2RealGapEstimator()
    
    # 模拟数据
    sim_state = {
        "sensors": {
            "imu": {"orient": [5.0, -3.0, 0.0], "gyro": [0.1, 0.0, 0.0]},
            "joints": {
                "hip_left": {"angle": 10.0, "velocity": 0.5},
                "hip_right": {"angle": -8.0, "velocity": -0.3}
            }
        },
        "torso_height": 1.45
    }
    
    # 模拟真实数据（带偏差）
    real_state = {
        "sensors": {
            "imu": {"orient": [5.5, -2.8, 0.1], "gyro": [0.12, 0.02, 0.01]},
            "joints": {
                "hip_left": {"angle": 10.5, "velocity": 0.55},
                "hip_right": {"angle": -7.5, "velocity": -0.25}
            }
        },
        "torso_height": 1.43
    }
    
    # 测试Gap估计
    print("=== 测试Gap估计 ===")
    gap = estimator.estimate_gap(sim_state, real_state)
    print(f"状态RMSE: {gap['state_rmse']:.4f}")
    
    # 收集数据
    print("\n=== 收集数据 ===")
    for i in range(100):
        estimator.collect_data_pair(sim_state, real_state)
    print(f"已收集: {len(estimator.data_buffer)}条")
    
    # 统计
    print("\n=== 统计信息 ===")
    stats = estimator.get_stats()
    print(json.dumps(stats, indent=2))
