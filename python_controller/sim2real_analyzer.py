"""
Sim2Real 数据差异分析系统
对比"理论预期"与"实际反馈"，支持功率分析和参数校准
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import time
import json
from pathlib import Path

@dataclass
class PowerStats:
    """功率统计"""
    theoretical_power: float = 0.0  # 理论机械功率 (W) = Torque * Velocity
    real_power_mech: float = 0.0    # 实际机械功率 (W)
    real_power_elec: float = 0.0    # 实际电功率 (W) = Voltage * Current
    efficiency: float = 0.0         # 效率 (Mech / Elec)
    power_loss: float = 0.0         # 损耗 (Theor - Real_Mech)
    timestamp: float = 0.0

@dataclass
class GapReport:
    """差异报告"""
    avg_efficiency_gap: float       # 平均效率偏差
    avg_response_lag: float         # 平均响应滞后 (s)
    torque_scale_factor: float      # 建议转矩缩放系数 (用于校准)
    friction_estimate: float        # 估计摩擦系数修正
    details: List[Dict]             # 详细数据点

class Sim2RealAnalyzer:
    """
    Sim2Real 差异分析器
    
    功能：
    1. 实时计算理论功率与实际功率
    2. 分析响应滞后
    3. 生成参数校准建议
    """
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        self.history: List[PowerStats] = []
        self.command_history: List[Tuple[float, float]] = [] # (timestamp, value)
        self.feedback_history: List[Tuple[float, float]] = [] # (timestamp, value)
        
    def process_step(
        self,
        cmd_torque: float,
        cmd_velocity: float,
        real_torque: float,
        real_velocity: float,
        real_voltage: float = 12.0,
        real_current: float = 0.0
    ) -> PowerStats:
        """
        处理单步数据
        
        Args:
            cmd_torque: 指令转矩 (Nm)
            cmd_velocity: 指令速度 (rad/s)
            real_torque: 实际转矩 (Nm)
            real_velocity: 实际速度 (rad/s)
            real_voltage: 电池电压 (V)
            real_current: 总电流 (A)
        """
        timestamp = time.time()
        
        # 1. 理论功率 (无损耗机械功率)
        theoretical_power = abs(cmd_torque * cmd_velocity)
        
        # 2. 实际机械功率
        real_power_mech = abs(real_torque * real_velocity)
        
        # 3. 实际电功率
        real_power_elec = real_voltage * real_current
        
        # 4. 效率与损耗
        efficiency = 0.0
        if real_power_elec > 0.1:
            efficiency = real_power_mech / real_power_elec
            
        power_loss = theoretical_power - real_power_mech
        
        stats = PowerStats(
            theoretical_power=theoretical_power,
            real_power_mech=real_power_mech,
            real_power_elec=real_power_elec,
            efficiency=efficiency,
            power_loss=power_loss,
            timestamp=timestamp
        )
        
        # 记录历史
        self.history.append(stats)
        if len(self.history) > self.window_size:
            self.history.pop(0)
            
        # 记录用于滞后分析的数据 (采样电机0的转矩)
        self.command_history.append((timestamp, cmd_torque))
        self.feedback_history.append((timestamp, real_torque))
        
        # 保持历史长度适中
        if len(self.command_history) > self.window_size * 2:
            self.command_history.pop(0)
            self.feedback_history.pop(0)
            
        return stats
    
    def analyze_gap(self) -> GapReport:
        """分析差异并生成报告"""
        if not self.history:
            return GapReport(0, 0, 1.0, 0, [])
            
        # 1. 功率/效率分析
        avg_loss = np.mean([s.power_loss for s in self.history])
        avg_theor = np.mean([s.theoretical_power for s in self.history])
        
        # 如果理论功率很小，避免除零
        loss_ratio = 0.0
        if avg_theor > 0.1:
            loss_ratio = avg_loss / avg_theor
            
        # 建议修正系数：如果实际比理论小，说明仿真效率设置过高，需要降低 motor_strength
        # Example: Theor=20W, Real=18W. Loss=2W. Ratio=0.1. Scale=0.9
        torque_scale_factor = 1.0 - max(0, min(0.5, loss_ratio))
        
        # 2. 滞后分析 (简单互相关)
        lag = self._estimate_lag()
        
        # 3. 摩擦估计 (简化的)
        # 假设一部分功率损耗来自摩擦: P_loss = Friction * Velocity^2 ? 
        # 这里简化为剩余未解释损耗归结为摩擦
        friction_estimate = max(0, loss_ratio * 0.5) 
        
        report = GapReport(
            avg_efficiency_gap=loss_ratio,
            avg_response_lag=lag,
            torque_scale_factor=torque_scale_factor,
            friction_estimate=friction_estimate,
            details=[
                {
                    "t": s.timestamp,
                    "theory": s.theoretical_power,
                    "real": s.real_power_mech
                }
                for s in self.history[-10:] # 只返回最近10个点供显示
            ]
        )
        
        return report

    def _estimate_lag(self) -> float:
        """估计响应滞后 (秒)"""
        if len(self.command_history) < 10:
            return 0.0
            
        # 提取序列
        timestamps = [t for t, v in self.command_history]
        cmds = np.array([v for t, v in self.command_history])
        feeds = np.array([v for t, v in self.feedback_history])
        
        # 确保长度一致
        min_len = min(len(cmds), len(feeds))
        cmds = cmds[-min_len:]
        feeds = feeds[-min_len:]
        
        # 归一化
        if np.std(cmds) < 1e-3 or np.std(feeds) < 1e-3:
            return 0.0
            
        cmds = (cmds - np.mean(cmds)) / np.std(cmds)
        feeds = (feeds - np.mean(feeds)) / np.std(feeds)
        
        # 互相关
        correlation = np.correlate(cmds, feeds, mode='full')
        lags = np.arange(-min_len + 1, min_len)
        best_lag_idx = np.argmax(correlation)
        best_lag_steps = lags[best_lag_idx]
        
        # 转换为时间 (假设采样均匀)
        avg_dt = (timestamps[-1] - timestamps[0]) / len(timestamps)
        lag_time = abs(best_lag_steps) * avg_dt
        
        return lag_time

    def generate_visualization(self, output_path: str = "gap_analysis.json"):
        """生成详细分析文件供可视化"""
        data = {
            "summary": self.analyze_gap().__dict__,
            "history": [s.__dict__ for s in self.history]
        }
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"✅ 差异分析已保存: {output_path}")

# 测试代码
if __name__ == "__main__":
    analyzer = Sim2RealAnalyzer()
    
    print("Sim2Real 差异分析测试")
    print("模拟场景: 理论20W输出，实际18W反馈 (10%损耗)")
    
    # 模拟数据流
    # 模拟 1秒的数据，100Hz
    for i in range(100):
        t = i * 0.01
        
        # 生成命令 (模拟正弦波)
        cmd_vel = 10.0 + np.sin(t*10)
        cmd_trq = 2.0
        
        # 生成"真实"反馈 (添加损耗和滞后)
        # 实际转矩 = 理论 * 0.9 (10%损耗)
        # 实际速度 = 理论 (假设跟随完美)
        real_trq = cmd_trq * 0.9 
        real_vel = cmd_vel
        
        # 计算电功率 (假设电机效率 80%)
        real_elec_power = (real_trq * real_vel) / 0.8
        real_current = real_elec_power / 12.0
        
        stats = analyzer.process_step(
            cmd_torque=cmd_trq,
            cmd_velocity=cmd_vel,
            real_torque=real_trq,
            real_velocity=real_vel,
            real_voltage=12.0,
            real_current=real_current
        )
        
        if i % 20 == 0:
            print(f"Step {i}: Theory={stats.theoretical_power:.1f}W, Real={stats.real_power_mech:.1f}W, Loss={stats.power_loss:.1f}W")
            
    report = analyzer.analyze_gap()
    print("\n=== 分析报告 ===")
    print(f"平均效率偏差: {report.avg_efficiency_gap*100:.1f}%")
    print(f"建议转矩缩放: {report.torque_scale_factor:.3f}")
    print(f"响应滞后: {report.avg_response_lag*1000:.1f}ms")
