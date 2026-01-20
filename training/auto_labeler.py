"""
自动数据标记模块
使用大模型自动标记轨迹数据
"""

import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import numpy as np


class TrajectoryLabel(Enum):
    """轨迹标签"""
    SUCCESSFUL_GAIT = "successful_gait"      # 成功步态
    FALL_FORWARD = "fall_forward"            # 向前跌倒
    FALL_BACKWARD = "fall_backward"          # 向后跌倒
    FALL_SIDEWAYS = "fall_sideways"          # 侧向跌倒
    UNSTABLE_BALANCE = "unstable_balance"    # 不稳定平衡
    ENERGY_INEFFICIENT = "energy_inefficient"  # 能量效率低
    SLOW_PROGRESS = "slow_progress"          # 进展缓慢
    OSCILLATING = "oscillating"              # 振荡


@dataclass
class LabeledTrajectory:
    """带标签的轨迹"""
    trajectory_id: str
    label: TrajectoryLabel
    confidence: float
    explanation: str
    quality_score: float  # 0-1
    metadata: Dict


class AutoLabeler:
    """
    自动数据标记器
    
    功能：
    1. 基于规则的快速标记
    2. 大模型辅助标记（可选）
    3. 生成失败原因解释
    """
    
    # 标记阈值
    THRESHOLDS = {
        "fall_roll": 45.0,        # 跌倒Roll阈值
        "fall_pitch": 45.0,       # 跌倒Pitch阈值
        "fall_height": 0.4,       # 跌倒高度阈值
        "unstable_roll": 20.0,    # 不稳定Roll阈值
        "unstable_pitch": 20.0,   # 不稳定Pitch阈值
        "success_duration": 100,  # 成功持续步数
        "energy_threshold": 0.8,  # 能量效率阈值
        "velocity_threshold": 0.1 # 速度阈值
    }
    
    def __init__(
        self,
        use_llm: bool = False,
        llm_model: str = "phi3:mini"
    ):
        """
        初始化标记器
        
        Args:
            use_llm: 是否使用大模型辅助标记
            llm_model: 大模型名称
        """
        self.use_llm = use_llm
        self.llm_model = llm_model
        
        # LLM客户端（延迟初始化）
        self._llm_client = None
        
        # 统计
        self.labeled_count = 0
        self.label_distribution: Dict[str, int] = {}
    
    def label_trajectory(
        self,
        trajectory: dict,
        trajectory_id: Optional[str] = None
    ) -> LabeledTrajectory:
        """
        标记单条轨迹
        
        Args:
            trajectory: 轨迹数据
            trajectory_id: 轨迹ID
        
        Returns:
            带标签的轨迹
        """
        if trajectory_id is None:
            trajectory_id = f"traj_{self.labeled_count}"
        
        # 提取特征
        features = self._extract_features(trajectory)
        
        # 规则标记
        label, confidence, explanation = self._rule_based_label(features)
        
        # LLM辅助（如果启用）
        if self.use_llm and confidence < 0.7:
            llm_label, llm_confidence, llm_explanation = self._llm_label(trajectory, features)
            if llm_confidence > confidence:
                label = llm_label
                confidence = llm_confidence
                explanation = llm_explanation
        
        # 计算质量分数
        quality_score = self._compute_quality_score(features, label)
        
        # 更新统计
        self.labeled_count += 1
        label_name = label.value
        self.label_distribution[label_name] = self.label_distribution.get(label_name, 0) + 1
        
        return LabeledTrajectory(
            trajectory_id=trajectory_id,
            label=label,
            confidence=confidence,
            explanation=explanation,
            quality_score=quality_score,
            metadata=features
        )
    
    def _extract_features(self, trajectory: dict) -> dict:
        """提取轨迹特征"""
        states = trajectory.get('states', [])
        actions = trajectory.get('actions', [])
        
        if not states:
            return {
                "duration": 0,
                "max_roll": 0,
                "max_pitch": 0,
                "min_height": 1.5,
                "avg_velocity": 0,
                "energy_usage": 0,
                "terminated": False
            }
        
        # 姿态统计
        rolls = []
        pitches = []
        heights = []
        
        for state in states:
            sensors = state.get('sensors', {})
            imu = sensors.get('imu', {})
            orient = imu.get('orient', [0, 0, 0])
            
            rolls.append(abs(orient[0]))
            pitches.append(abs(orient[1]))
            heights.append(state.get('torso_height', 1.5))
        
        # 动作统计
        action_magnitudes = []
        for action in actions:
            if isinstance(action, dict):
                motors = action.get('motors', {})
                magnitude = sum(abs(v) for v in motors.values())
            elif isinstance(action, (list, np.ndarray)):
                magnitude = np.sum(np.abs(action))
            else:
                magnitude = 0
            action_magnitudes.append(magnitude)
        
        return {
            "duration": len(states),
            "max_roll": max(rolls) if rolls else 0,
            "max_pitch": max(pitches) if pitches else 0,
            "avg_roll": np.mean(rolls) if rolls else 0,
            "avg_pitch": np.mean(pitches) if pitches else 0,
            "min_height": min(heights) if heights else 1.5,
            "final_height": heights[-1] if heights else 1.5,
            "avg_velocity": trajectory.get('avg_velocity', 0),
            "total_distance": trajectory.get('total_distance', 0),
            "energy_usage": np.mean(action_magnitudes) if action_magnitudes else 0,
            "action_variance": np.var(action_magnitudes) if len(action_magnitudes) > 1 else 0,
            "terminated": trajectory.get('terminated', False)
        }
    
    def _rule_based_label(self, features: dict) -> Tuple[TrajectoryLabel, float, str]:
        """基于规则的标记"""
        max_roll = features.get('max_roll', 0)
        max_pitch = features.get('max_pitch', 0)
        min_height = features.get('min_height', 1.5)
        final_height = features.get('final_height', 1.5)
        duration = features.get('duration', 0)
        avg_velocity = features.get('avg_velocity', 0)
        energy_usage = features.get('energy_usage', 0)
        action_variance = features.get('action_variance', 0)
        
        # 检查跌倒
        if min_height < self.THRESHOLDS['fall_height']:
            if max_pitch > max_roll:
                if features.get('avg_pitch', 0) > 0:
                    return TrajectoryLabel.FALL_FORWARD, 0.9, "机器人向前跌倒，Pitch角度过大"
                else:
                    return TrajectoryLabel.FALL_BACKWARD, 0.9, "机器人向后跌倒，Pitch角度过大"
            else:
                return TrajectoryLabel.FALL_SIDEWAYS, 0.9, "机器人侧向跌倒，Roll角度过大"
        
        # 检查不稳定
        if max_roll > self.THRESHOLDS['unstable_roll'] or max_pitch > self.THRESHOLDS['unstable_pitch']:
            return TrajectoryLabel.UNSTABLE_BALANCE, 0.8, f"姿态不稳定，最大Roll: {max_roll:.1f}°, 最大Pitch: {max_pitch:.1f}°"
        
        # 检查振荡
        if action_variance > 50:
            return TrajectoryLabel.OSCILLATING, 0.7, "动作振荡严重，控制不稳定"
        
        # 检查能量效率
        if energy_usage > self.THRESHOLDS['energy_threshold'] and avg_velocity < self.THRESHOLDS['velocity_threshold']:
            return TrajectoryLabel.ENERGY_INEFFICIENT, 0.7, "能量消耗高但前进速度慢"
        
        # 检查进展
        if avg_velocity < self.THRESHOLDS['velocity_threshold'] and duration > self.THRESHOLDS['success_duration']:
            return TrajectoryLabel.SLOW_PROGRESS, 0.6, "前进速度过慢"
        
        # 成功
        if duration >= self.THRESHOLDS['success_duration']:
            return TrajectoryLabel.SUCCESSFUL_GAIT, 0.85, f"成功完成{duration}步，姿态稳定"
        
        # 默认：短时间不稳定
        return TrajectoryLabel.UNSTABLE_BALANCE, 0.5, f"轨迹过短（{duration}步），无法确定"
    
    def _llm_label(self, trajectory: dict, features: dict) -> Tuple[TrajectoryLabel, float, str]:
        """使用LLM进行标记"""
        if self._llm_client is None:
            try:
                import ollama
                self._llm_client = ollama
            except ImportError:
                return TrajectoryLabel.UNSTABLE_BALANCE, 0.0, "LLM不可用"
        
        # 构建Prompt
        prompt = f"""分析以下机器人轨迹数据并给出标签：

特征：
- 持续时间: {features['duration']}步
- 最大Roll角度: {features['max_roll']:.1f}°
- 最大Pitch角度: {features['max_pitch']:.1f}°
- 最低高度: {features['min_height']:.2f}m
- 平均速度: {features['avg_velocity']:.3f}m/s
- 能量消耗: {features['energy_usage']:.2f}

可选标签：
1. successful_gait - 成功步态
2. fall_forward - 向前跌倒
3. fall_backward - 向后跌倒
4. fall_sideways - 侧向跌倒
5. unstable_balance - 不稳定平衡
6. energy_inefficient - 能量效率低
7. slow_progress - 进展缓慢
8. oscillating - 振荡

请返回JSON格式：
{{"label": "标签名", "confidence": 0.0-1.0, "explanation": "解释"}}
"""
        
        try:
            response = self._llm_client.chat(
                model=self.llm_model,
                messages=[{"role": "user", "content": prompt}]
            )
            
            result = json.loads(response['message']['content'])
            label = TrajectoryLabel(result['label'])
            confidence = result['confidence']
            explanation = result['explanation']
            
            return label, confidence, explanation
            
        except Exception as e:
            return TrajectoryLabel.UNSTABLE_BALANCE, 0.0, f"LLM调用失败: {e}"
    
    def _compute_quality_score(self, features: dict, label: TrajectoryLabel) -> float:
        """计算质量分数"""
        # 成功轨迹质量最高
        if label == TrajectoryLabel.SUCCESSFUL_GAIT:
            base_score = 0.9
            # 根据持续时间加分
            duration_bonus = min(0.1, features['duration'] / 1000)
            # 根据稳定性加分
            stability_bonus = 0.1 * (1 - features['avg_roll'] / 45)
            return min(1.0, base_score + duration_bonus + stability_bonus)
        
        # 失败轨迹也有价值（用于学习）
        elif label in (TrajectoryLabel.FALL_FORWARD, TrajectoryLabel.FALL_BACKWARD, TrajectoryLabel.FALL_SIDEWAYS):
            return 0.6  # 失败数据也有训练价值
        
        else:
            return 0.4  # 其他类型
    
    def batch_label(
        self,
        dataset_path: str,
        output_path: Optional[str] = None
    ) -> str:
        """
        批量标记数据集
        
        Args:
            dataset_path: 数据集路径
            output_path: 输出路径
        
        Returns:
            输出文件路径
        """
        dataset_path = Path(dataset_path)
        
        if not dataset_path.exists():
            raise FileNotFoundError(f"数据集不存在: {dataset_path}")
        
        if output_path is None:
            output_path = dataset_path.parent / f"labeled_{dataset_path.stem}.json"
        
        print(f"📝 开始批量标记...")
        print(f"   输入: {dataset_path}")
        print(f"   输出: {output_path}")
        
        # 加载数据
        with open(dataset_path, 'r', encoding='utf-8') as f:
            trajectories = json.load(f)
        
        if not isinstance(trajectories, list):
            trajectories = [trajectories]
        
        # 标记
        labeled_data = []
        for i, traj in enumerate(trajectories):
            labeled = self.label_trajectory(traj, f"traj_{i}")
            labeled_data.append({
                "id": labeled.trajectory_id,
                "label": labeled.label.value,
                "confidence": labeled.confidence,
                "explanation": labeled.explanation,
                "quality_score": labeled.quality_score,
                "metadata": labeled.metadata
            })
            
            if (i + 1) % 100 == 0:
                print(f"   已处理: {i + 1}/{len(trajectories)}")
        
        # 保存
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(labeled_data, f, indent=2, ensure_ascii=False)
        
        print(f"✅ 标记完成，共{len(labeled_data)}条")
        print(f"   分布: {self.label_distribution}")
        
        return str(output_path)
    
    def generate_failure_explanation(self, trajectory: dict) -> str:
        """生成失败原因详细解释"""
        features = self._extract_features(trajectory)
        labeled = self.label_trajectory(trajectory)
        
        explanation = f"## 轨迹分析报告\n\n"
        explanation += f"**标签**: {labeled.label.value}\n"
        explanation += f"**置信度**: {labeled.confidence:.2f}\n"
        explanation += f"**质量分数**: {labeled.quality_score:.2f}\n\n"
        
        explanation += "### 关键指标\n"
        explanation += f"- 持续时间: {features['duration']}步\n"
        explanation += f"- 最大Roll: {features['max_roll']:.1f}°\n"
        explanation += f"- 最大Pitch: {features['max_pitch']:.1f}°\n"
        explanation += f"- 最低高度: {features['min_height']:.2f}m\n"
        explanation += f"- 平均速度: {features['avg_velocity']:.3f}m/s\n\n"
        
        explanation += "### 失败原因\n"
        explanation += f"{labeled.explanation}\n\n"
        
        # 改进建议
        explanation += "### 改进建议\n"
        if labeled.label in (TrajectoryLabel.FALL_FORWARD, TrajectoryLabel.FALL_BACKWARD):
            explanation += "- 增加平衡控制的Pitch响应\n"
            explanation += "- 降低前进速度\n"
        elif labeled.label == TrajectoryLabel.FALL_SIDEWAYS:
            explanation += "- 增加Roll方向的稳定性\n"
            explanation += "- 检查关节对称性\n"
        elif labeled.label == TrajectoryLabel.OSCILLATING:
            explanation += "- 降低控制增益\n"
            explanation += "- 增加阻尼\n"
        
        return explanation
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "labeled_count": self.labeled_count,
            "label_distribution": self.label_distribution,
            "use_llm": self.use_llm
        }


# 测试代码
if __name__ == "__main__":
    print("自动数据标记器测试\n")
    
    # 创建标记器
    labeler = AutoLabeler(use_llm=False)
    
    # 模拟成功轨迹
    success_traj = {
        "states": [
            {"sensors": {"imu": {"orient": [2.0, 1.0, 0.0]}}, "torso_height": 1.45}
            for _ in range(150)
        ],
        "actions": [{"motors": {"hip_left": 5.0, "hip_right": -5.0}} for _ in range(150)],
        "avg_velocity": 0.3,
        "total_distance": 1.5
    }
    
    print("=== 成功轨迹 ===")
    labeled = labeler.label_trajectory(success_traj, "success_1")
    print(f"标签: {labeled.label.value}")
    print(f"置信度: {labeled.confidence:.2f}")
    print(f"解释: {labeled.explanation}")
    
    # 模拟跌倒轨迹
    fall_traj = {
        "states": [
            {"sensors": {"imu": {"orient": [i*2, i*1.5, 0.0]}}, "torso_height": max(0.2, 1.45 - i*0.1)}
            for i in range(20)
        ],
        "actions": [{"motors": {"hip_left": 10.0, "hip_right": -10.0}} for _ in range(20)],
        "terminated": True
    }
    
    print("\n=== 跌倒轨迹 ===")
    labeled = labeler.label_trajectory(fall_traj, "fall_1")
    print(f"标签: {labeled.label.value}")
    print(f"置信度: {labeled.confidence:.2f}")
    print(f"解释: {labeled.explanation}")
    
    # 生成详细解释
    print("\n=== 失败分析报告 ===")
    report = labeler.generate_failure_explanation(fall_traj)
    print(report)
    
    # 统计
    print("\n=== 统计信息 ===")
    print(json.dumps(labeler.get_stats(), indent=2))
