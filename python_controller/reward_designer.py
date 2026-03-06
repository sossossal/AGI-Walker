"""
奖励函数设计工具
可配置、可组合的奖励函数系统
"""

import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum


class RewardComponent(Enum):
    """奖励组件类型"""

    FORWARD_VELOCITY = "forward_velocity"
    STABILITY = "stability"
    ENERGY_EFFICIENCY = "energy_efficiency"
    SURVIVAL = "survival"
    HEIGHT_MAINTENANCE = "height_maintenance"
    SMOOTH_MOTION = "smooth_motion"
    SYMMETRY = "symmetry"
    TARGET_TRACKING = "target_tracking"


@dataclass
class RewardConfig:
    """奖励配置"""

    # 组件权重
    weights: Dict[str, float] = field(
        default_factory=lambda: {
            "forward_velocity": 1.0,
            "stability": 0.5,
            "energy_efficiency": 0.3,
            "survival": 0.2,
            "height_maintenance": 0.4,
            "smooth_motion": 0.2,
            "symmetry": 0.1,
        }
    )

    # 阈值参数
    max_roll: float = 30.0  # 最大Roll角度
    max_pitch: float = 30.0  # 最大Pitch角度
    target_height: float = 1.45  # 目标高度
    height_tolerance: float = 0.2  # 高度容差

    # 惩罚参数
    fall_penalty: float = -10.0  # 跌倒惩罚
    energy_scale: float = 0.01  # 能量消耗缩放


class RewardDesigner:
    """
    奖励函数设计器

    功能：
    1. 可配置的奖励组件
    2. 自动权重调整
    3. 奖励分解和可视化
    """

    def __init__(self, config: Optional[RewardConfig] = None):
        self.config = config or RewardConfig()

        # 历史记录（用于平滑和统计）
        self.prev_action: Optional[np.ndarray] = None
        self.prev_velocity: float = 0.0
        self.episode_rewards: List[float] = []
        self.component_history: Dict[str, List[float]] = {}

        # 统计
        self.total_calls = 0

    def compute_reward(
        self, obs: dict, action: np.ndarray, info: Optional[dict] = None
    ) -> Tuple[float, dict]:
        """
        计算组合奖励

        Args:
            obs: 观测数据
            action: 执行的动作
            info: 额外信息

        Returns:
            (总奖励, 分解详情)
        """
        self.total_calls += 1

        # 提取状态
        sensors = obs.get("sensors", {})
        imu = sensors.get("imu", {})
        orient = imu.get("orient", [0, 0, 0])
        height = obs.get("torso_height", 1.45)

        roll, pitch = orient[0], orient[1]

        # 计算各组件
        components = {}

        # 1. 前进速度奖励
        if "forward_velocity" in self.config.weights:
            velocity = info.get("forward_velocity", 0) if info else 0
            velocity_reward = self._compute_velocity_reward(velocity)
            components["forward_velocity"] = velocity_reward

        # 2. 稳定性奖励
        if "stability" in self.config.weights:
            stability_reward = self._compute_stability_reward(roll, pitch)
            components["stability"] = stability_reward

        # 3. 能量效率奖励
        if "energy_efficiency" in self.config.weights:
            energy_reward = self._compute_energy_reward(action)
            components["energy_efficiency"] = energy_reward

        # 4. 存活奖励
        if "survival" in self.config.weights:
            survival_reward = 1.0  # 每步存活+1
            components["survival"] = survival_reward

        # 5. 高度维持奖励
        if "height_maintenance" in self.config.weights:
            height_reward = self._compute_height_reward(height)
            components["height_maintenance"] = height_reward

        # 6. 动作平滑奖励
        if "smooth_motion" in self.config.weights:
            smooth_reward = self._compute_smooth_reward(action)
            components["smooth_motion"] = smooth_reward

        # 7. 对称性奖励
        if "symmetry" in self.config.weights:
            symmetry_reward = self._compute_symmetry_reward(obs)
            components["symmetry"] = symmetry_reward

        # 加权求和
        total_reward = 0.0
        for comp_name, comp_value in components.items():
            weight = self.config.weights.get(comp_name, 0.0)
            total_reward += weight * comp_value

        # 检查跌倒
        if self._check_fall(roll, pitch, height):
            total_reward += self.config.fall_penalty
            components["fall_penalty"] = self.config.fall_penalty

        # 更新历史
        self.prev_action = action.copy()
        self.episode_rewards.append(total_reward)

        for comp_name, comp_value in components.items():
            if comp_name not in self.component_history:
                self.component_history[comp_name] = []
            self.component_history[comp_name].append(comp_value)

        return total_reward, components

    def _compute_velocity_reward(self, velocity: float) -> float:
        """前进速度奖励"""
        # 期望速度范围: 0.5 - 1.0 m/s
        if velocity > 0:
            return min(velocity, 1.0)  # 上限1.0
        else:
            return velocity * 0.5  # 后退惩罚

    def _compute_stability_reward(self, roll: float, pitch: float) -> float:
        """稳定性奖励"""
        # 姿态越接近0，奖励越高
        roll_penalty = (roll / self.config.max_roll) ** 2
        pitch_penalty = (pitch / self.config.max_pitch) ** 2

        stability = 1.0 - min(1.0, roll_penalty + pitch_penalty)
        return stability

    def _compute_energy_reward(self, action: np.ndarray) -> float:
        """能量效率奖励（惩罚大动作）"""
        action_magnitude = np.sum(action**2)
        energy_cost = action_magnitude * self.config.energy_scale
        return -energy_cost

    def _compute_height_reward(self, height: float) -> float:
        """高度维持奖励"""
        height_error = abs(height - self.config.target_height)

        if height_error < self.config.height_tolerance:
            return 1.0
        else:
            return max(0.0, 1.0 - height_error / self.config.target_height)

    def _compute_smooth_reward(self, action: np.ndarray) -> float:
        """动作平滑奖励"""
        if self.prev_action is None:
            return 0.0

        action_diff = np.sum((action - self.prev_action) ** 2)
        smoothness = 1.0 - min(1.0, action_diff * 0.1)
        return smoothness

    def _compute_symmetry_reward(self, obs: dict) -> float:
        """对称性奖励"""
        joints = obs.get("sensors", {}).get("joints", {})

        hip_left = joints.get("hip_left", {}).get("angle", 0)
        hip_right = joints.get("hip_right", {}).get("angle", 0)

        # 理想情况下左右对称
        asymmetry = abs(hip_left + hip_right) / 90.0  # 归一化
        symmetry = 1.0 - min(1.0, asymmetry)

        return symmetry

    def _check_fall(self, roll: float, pitch: float, height: float) -> bool:
        """检查是否跌倒"""
        return abs(roll) > 45 or abs(pitch) > 45 or height < 0.3

    def reset(self):
        """重置episode状态"""
        self.prev_action = None
        self.prev_velocity = 0.0
        self.episode_rewards.clear()
        self.component_history.clear()

    def set_weight(self, component: str, weight: float):
        """设置组件权重"""
        if component in self.config.weights:
            self.config.weights[component] = weight
            print(f"设置 {component} 权重为 {weight}")
        else:
            print(f"⚠️ 未知组件: {component}")

    def auto_tune(
        self, demonstrations: List[dict], target_metric: str = "survival_time"
    ):
        """
        从演示数据自动调整权重

        Args:
            demonstrations: 演示轨迹列表
            target_metric: 目标指标
        """
        print("🔧 开始自动调整权重...")
        print(f"   演示数据: {len(demonstrations)}条")
        print(f"   目标指标: {target_metric}")

        # 分析演示数据
        success_demos = [d for d in demonstrations if d.get("success", False)]
        fail_demos = [d for d in demonstrations if not d.get("success", False)]

        if not success_demos:
            print("⚠️ 没有成功演示，无法调整")
            return

        # 计算成功演示的平均特征
        success_features = self._extract_features(success_demos)
        fail_features = self._extract_features(fail_demos) if fail_demos else {}

        # 调整权重
        for comp_name in self.config.weights:
            success_value = success_features.get(comp_name, 0.5)
            fail_value = fail_features.get(comp_name, 0.5)

            # 如果成功演示中该组件值更高，增加权重
            if success_value > fail_value:
                factor = 1.2
            else:
                factor = 0.8

            self.config.weights[comp_name] *= factor

        # 归一化权重
        total_weight = sum(self.config.weights.values())
        for comp_name in self.config.weights:
            self.config.weights[comp_name] /= total_weight

        print("✅ 权重调整完成")
        print(f"   新权重: {self.config.weights}")

    def _extract_features(self, demonstrations: List[dict]) -> dict:
        """从演示中提取特征"""
        features = {}

        for demo in demonstrations:
            for comp_name, values in demo.get("components", {}).items():
                if comp_name not in features:
                    features[comp_name] = []
                features[comp_name].extend(values)

        # 计算平均值
        for comp_name in features:
            features[comp_name] = (
                np.mean(features[comp_name]) if features[comp_name] else 0.0
            )

        return features

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "total_calls": self.total_calls,
            "weights": self.config.weights,
            "episode_reward_mean": (
                np.mean(self.episode_rewards) if self.episode_rewards else 0
            ),
            "episode_reward_std": (
                np.std(self.episode_rewards) if self.episode_rewards else 0
            ),
            "component_means": {
                name: np.mean(values) if values else 0
                for name, values in self.component_history.items()
            },
        }


def create_reward_designer(preset: str = "balanced") -> RewardDesigner:
    """
    工厂函数：创建奖励设计器

    presets:
    - "balanced": 平衡配置
    - "speed": 速度优先
    - "stability": 稳定性优先
    - "efficiency": 能量效率优先
    """
    presets = {
        "balanced": RewardConfig(),
        "speed": RewardConfig(
            weights={
                "forward_velocity": 2.0,
                "stability": 0.3,
                "energy_efficiency": 0.1,
                "survival": 0.2,
                "height_maintenance": 0.3,
                "smooth_motion": 0.1,
                "symmetry": 0.0,
            }
        ),
        "stability": RewardConfig(
            weights={
                "forward_velocity": 0.3,
                "stability": 2.0,
                "energy_efficiency": 0.2,
                "survival": 0.5,
                "height_maintenance": 0.8,
                "smooth_motion": 0.3,
                "symmetry": 0.2,
            }
        ),
        "efficiency": RewardConfig(
            weights={
                "forward_velocity": 0.5,
                "stability": 0.4,
                "energy_efficiency": 2.0,
                "survival": 0.2,
                "height_maintenance": 0.3,
                "smooth_motion": 0.5,
                "symmetry": 0.1,
            }
        ),
    }

    config = presets.get(preset, presets["balanced"])
    return RewardDesigner(config)


# 测试代码
if __name__ == "__main__":
    import json

    print("奖励函数设计器测试\n")

    # 创建设计器
    designer = create_reward_designer("balanced")

    # 模拟观测
    obs = {
        "sensors": {
            "imu": {"orient": [5.0, -3.0, 0.0]},
            "joints": {"hip_left": {"angle": 10.0}, "hip_right": {"angle": -8.0}},
        },
        "torso_height": 1.40,
    }

    action = np.array([0.5, -0.3])
    info = {"forward_velocity": 0.3}

    # 计算奖励
    print("=== 正常状态 ===")
    reward, components = designer.compute_reward(obs, action, info)
    print(f"总奖励: {reward:.3f}")
    print(f"组件分解: {json.dumps(components, indent=2)}")

    # 不稳定状态
    print("\n=== 不稳定状态 ===")
    unstable_obs = {
        "sensors": {
            "imu": {"orient": [25.0, -20.0, 0.0]},
            "joints": {"hip_left": {"angle": 30.0}, "hip_right": {"angle": -5.0}},
        },
        "torso_height": 1.2,
    }

    reward, components = designer.compute_reward(unstable_obs, action, info)
    print(f"总奖励: {reward:.3f}")

    # 跌倒状态
    print("\n=== 跌倒状态 ===")
    fall_obs = {
        "sensors": {
            "imu": {"orient": [50.0, -40.0, 0.0]},
            "joints": {"hip_left": {"angle": 0}, "hip_right": {"angle": 0}},
        },
        "torso_height": 0.2,
    }

    reward, components = designer.compute_reward(fall_obs, action, info)
    print(f"总奖励: {reward:.3f}")
    print(f"包含跌倒惩罚: {components.get('fall_penalty', 0)}")

    # 统计
    print("\n=== 统计信息 ===")
    print(json.dumps(designer.get_stats(), indent=2))
