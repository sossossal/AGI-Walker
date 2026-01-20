"""
多模态融合模块（Multimodal Fusion）
融合IMU、关节、视觉、触觉等多模态传感器数据
"""

import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
import numpy as np


@dataclass
class FusionConfig:
    """融合配置"""
    # 传感器权重
    imu_weight: float = 0.40
    joint_weight: float = 0.30
    vision_weight: float = 0.20
    contact_weight: float = 0.10
    
    # 融合参数
    confidence_threshold: float = 0.3
    obstacle_distance_threshold: float = 1.5  # 米
    stability_history_size: int = 20
    
    # 视觉参数
    vision_update_interval: float = 0.1  # 100ms


class MultimodalFusion:
    """
    多模态传感器融合
    
    功能：
    - 融合IMU、关节、视觉、接触传感器数据
    - 计算综合置信度
    - 障碍物信息整合
    - 稳定性评估
    """
    
    def __init__(
        self,
        vision_processor=None,
        config: Optional[FusionConfig] = None
    ):
        """
        初始化多模态融合模块
        
        Args:
            vision_processor: 视觉处理器实例
            config: 融合配置
        """
        self.config = config or FusionConfig()
        self.vision_processor = vision_processor
        
        # 状态历史
        self.stability_history: List[float] = []
        self.last_vision_update = 0.0
        self.last_vision_result: Optional[dict] = None
        
        # 缓存
        self.obstacle_cache: List[dict] = []
        self.environment_state: Dict = {}
        
        # 统计
        self.fusion_count = 0
        self.total_fusion_time = 0.0
    
    def fuse_sensors(
        self,
        sensor_data: dict,
        vision_data: Optional[np.ndarray] = None
    ) -> dict:
        """
        融合多模态传感器数据
        
        Args:
            sensor_data: 基础传感器数据（IMU、关节等）
            vision_data: 可选的视觉数据（图像帧）
        
        Returns:
            融合后的传感器数据
        """
        start_time = time.time()
        
        # 提取各传感器数据
        imu_data = self._extract_imu(sensor_data)
        joint_data = self._extract_joints(sensor_data)
        contact_data = self._extract_contacts(sensor_data)
        
        # 处理视觉数据
        vision_result = self._process_vision(vision_data)
        
        # 融合并计算综合状态
        fused = {
            **sensor_data,
            "fusion": {
                "timestamp": time.time(),
                "imu_summary": imu_data,
                "joint_summary": joint_data,
                "contact_summary": contact_data,
                "vision_summary": vision_result,
                "obstacles": self._fuse_obstacles(vision_result),
                "stability": self._calculate_stability(imu_data),
                "confidence": self._calculate_confidence(
                    imu_data, joint_data, contact_data, vision_result
                ),
                "environment": self._update_environment_state()
            }
        }
        
        # 更新统计
        self.fusion_count += 1
        self.total_fusion_time += time.time() - start_time
        
        return fused
    
    def _extract_imu(self, sensor_data: dict) -> dict:
        """提取和处理IMU数据"""
        imu = sensor_data.get('sensors', {}).get('imu', {})
        orient = imu.get('orient', [0, 0, 0])
        gyro = imu.get('gyro', [0, 0, 0])
        accel = imu.get('accel', [0, 0, 0])
        
        roll, pitch, yaw = orient[0], orient[1], orient[2]
        
        # 计算倾斜程度
        tilt_magnitude = np.sqrt(roll**2 + pitch**2)
        
        # 判断姿态状态
        if tilt_magnitude < 5:
            posture_state = "stable"
        elif tilt_magnitude < 15:
            posture_state = "slight_tilt"
        elif tilt_magnitude < 30:
            posture_state = "tilting"
        else:
            posture_state = "critical"
        
        return {
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
            "tilt_magnitude": float(tilt_magnitude),
            "posture_state": posture_state,
            "angular_velocity": gyro,
            "linear_acceleration": accel
        }
    
    def _extract_joints(self, sensor_data: dict) -> dict:
        """提取和处理关节数据"""
        joints = sensor_data.get('sensors', {}).get('joints', {})
        
        summary = {}
        for name, data in joints.items():
            angle = data.get('angle', 0)
            velocity = data.get('velocity', 0)
            
            summary[name] = {
                "angle": float(angle),
                "velocity": float(velocity),
                "at_limit": abs(angle) > 80  # 接近限位
            }
        
        # 计算对称性（左右对称程度）
        left_hip = joints.get('hip_left', {}).get('angle', 0)
        right_hip = joints.get('hip_right', {}).get('angle', 0)
        symmetry = 1.0 - min(1.0, abs(left_hip - right_hip) / 45.0)
        
        return {
            "joints": summary,
            "symmetry": float(symmetry),
            "any_at_limit": any(s.get("at_limit", False) for s in summary.values())
        }
    
    def _extract_contacts(self, sensor_data: dict) -> dict:
        """提取和处理接触传感器数据"""
        contacts = sensor_data.get('sensors', {}).get('contacts', {})
        
        foot_left = contacts.get('foot_left', False)
        foot_right = contacts.get('foot_right', False)
        
        # 判断支撑状态
        if foot_left and foot_right:
            support_state = "double_support"
        elif foot_left or foot_right:
            support_state = "single_support"
        else:
            support_state = "no_support"  # 可能在空中
        
        return {
            "foot_left": foot_left,
            "foot_right": foot_right,
            "support_state": support_state,
            "is_grounded": foot_left or foot_right
        }
    
    def _process_vision(self, vision_data: Optional[np.ndarray]) -> Optional[dict]:
        """处理视觉数据"""
        current_time = time.time()
        
        # 检查更新间隔
        if current_time - self.last_vision_update < self.config.vision_update_interval:
            return self.last_vision_result
        
        if vision_data is None or self.vision_processor is None:
            return self.last_vision_result
        
        try:
            # 处理视觉帧
            result = self.vision_processor.process_frame(vision_data)
            
            self.last_vision_update = current_time
            self.last_vision_result = result
            
            return result
            
        except Exception as e:
            print(f"⚠️ 视觉处理错误: {e}")
            return None
    
    def _fuse_obstacles(self, vision_result: Optional[dict]) -> List[dict]:
        """融合障碍物信息"""
        if vision_result is None:
            return self.obstacle_cache
        
        obstacles = vision_result.get('obstacles', [])
        
        # 过滤距离过远的障碍物
        filtered = [
            o for o in obstacles
            if o.get('distance_estimate', 10) < self.config.obstacle_distance_threshold
        ]
        
        # 更新缓存
        self.obstacle_cache = filtered
        
        return filtered
    
    def _calculate_stability(self, imu_summary: dict) -> float:
        """计算稳定性分数（0-1）"""
        tilt = imu_summary.get('tilt_magnitude', 0)
        
        # 倾斜越小，稳定性越高
        stability = 1.0 - min(1.0, tilt / 45.0)
        
        # 平滑处理
        self.stability_history.append(stability)
        if len(self.stability_history) > self.config.stability_history_size:
            self.stability_history.pop(0)
        
        # 返回平均稳定性
        avg_stability = sum(self.stability_history) / len(self.stability_history)
        
        return float(avg_stability)
    
    def _calculate_confidence(
        self,
        imu_data: dict,
        joint_data: dict,
        contact_data: dict,
        vision_data: Optional[dict]
    ) -> float:
        """计算融合置信度"""
        scores = []
        weights = []
        
        # IMU置信度
        imu_conf = 1.0 if imu_data.get('posture_state') != 'critical' else 0.3
        scores.append(imu_conf)
        weights.append(self.config.imu_weight)
        
        # 关节置信度
        joint_conf = 0.8 if not joint_data.get('any_at_limit') else 0.4
        joint_conf *= joint_data.get('symmetry', 1.0)
        scores.append(joint_conf)
        weights.append(self.config.joint_weight)
        
        # 接触置信度
        contact_conf = 1.0 if contact_data.get('is_grounded') else 0.2
        if contact_data.get('support_state') == 'double_support':
            contact_conf = 1.0
        elif contact_data.get('support_state') == 'single_support':
            contact_conf = 0.7
        scores.append(contact_conf)
        weights.append(self.config.contact_weight)
        
        # 视觉置信度
        if vision_data is not None:
            vision_conf = 0.8 if not vision_data.get('dummy', False) else 0.3
            scores.append(vision_conf)
            weights.append(self.config.vision_weight)
        
        # 加权平均
        total_weight = sum(weights)
        if total_weight == 0:
            return 0.0
        
        confidence = sum(s * w for s, w in zip(scores, weights)) / total_weight
        
        return float(confidence)
    
    def _update_environment_state(self) -> dict:
        """更新环境状态"""
        self.environment_state = {
            "last_update": time.time(),
            "obstacle_count": len(self.obstacle_cache),
            "nearest_obstacle": (
                min(o.get('distance_estimate', 10) for o in self.obstacle_cache)
                if self.obstacle_cache else None
            ),
            "avg_stability": (
                sum(self.stability_history) / len(self.stability_history)
                if self.stability_history else 1.0
            )
        }
        
        return self.environment_state
    
    def get_obstacle_avoidance_action(self) -> Optional[dict]:
        """根据障碍物信息生成避障建议"""
        if not self.obstacle_cache:
            return None
        
        # 找到最近的障碍物
        nearest = min(self.obstacle_cache, key=lambda o: o.get('distance_estimate', 10))
        distance = nearest.get('distance_estimate', 10)
        angle = nearest.get('angle_deg', 0)
        
        if distance > self.config.obstacle_distance_threshold:
            return None
        
        # 生成避障建议
        action = {
            "type": "obstacle_avoidance",
            "nearest_distance": distance,
            "nearest_angle": angle,
            "urgency": 1.0 - distance / self.config.obstacle_distance_threshold
        }
        
        # 转向建议
        if angle < 0:
            action["turn_direction"] = "right"
            action["turn_intensity"] = abs(angle) / 30.0
        else:
            action["turn_direction"] = "left"
            action["turn_intensity"] = abs(angle) / 30.0
        
        return action
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        avg_time = (
            self.total_fusion_time / self.fusion_count
            if self.fusion_count > 0 else 0
        )
        
        return {
            "fusion_count": self.fusion_count,
            "avg_fusion_time_ms": avg_time * 1000,
            "obstacle_cache_size": len(self.obstacle_cache),
            "stability_history_size": len(self.stability_history),
            "environment_state": self.environment_state
        }


def create_multimodal_fusion(
    vision_processor=None,
    config: Optional[FusionConfig] = None
) -> MultimodalFusion:
    """工厂函数：创建多模态融合模块"""
    return MultimodalFusion(vision_processor, config)


# 测试代码
if __name__ == "__main__":
    import json
    
    print("多模态融合模块测试\n")
    
    # 创建融合模块
    fusion = create_multimodal_fusion()
    
    # 模拟传感器数据
    sensor_data = {
        "sensors": {
            "imu": {
                "orient": [5.0, -3.0, 0.0],
                "gyro": [0.1, -0.05, 0.0],
                "accel": [0.0, 0.0, -9.8]
            },
            "joints": {
                "hip_left": {"angle": 10.0, "velocity": 0.5},
                "hip_right": {"angle": -8.0, "velocity": -0.3}
            },
            "contacts": {
                "foot_left": True,
                "foot_right": True
            }
        },
        "torso_height": 1.45
    }
    
    # 测试融合
    print("=== 测试传感器融合 ===")
    fused = fusion.fuse_sensors(sensor_data)
    
    fusion_result = fused.get('fusion', {})
    print(f"IMU摘要: {json.dumps(fusion_result.get('imu_summary'), indent=2)}")
    print(f"稳定性: {fusion_result.get('stability', 0):.3f}")
    print(f"置信度: {fusion_result.get('confidence', 0):.3f}")
    print(f"支撑状态: {fusion_result.get('contact_summary', {}).get('support_state')}")
    
    # 模拟不稳定状态
    print("\n=== 测试不稳定状态 ===")
    unstable_data = {
        "sensors": {
            "imu": {"orient": [25.0, -15.0, 0.0], "gyro": [1.0, -0.5, 0.0], "accel": [0.0, 0.0, -9.8]},
            "joints": {
                "hip_left": {"angle": 40.0, "velocity": 2.0},
                "hip_right": {"angle": -35.0, "velocity": -1.5}
            },
            "contacts": {"foot_left": True, "foot_right": False}
        },
        "torso_height": 1.2
    }
    
    fused = fusion.fuse_sensors(unstable_data)
    fusion_result = fused.get('fusion', {})
    print(f"姿态状态: {fusion_result.get('imu_summary', {}).get('posture_state')}")
    print(f"稳定性: {fusion_result.get('stability', 0):.3f}")
    print(f"置信度: {fusion_result.get('confidence', 0):.3f}")
    
    # 统计
    print("\n=== 统计信息 ===")
    stats = fusion.get_stats()
    print(json.dumps(stats, indent=2))
