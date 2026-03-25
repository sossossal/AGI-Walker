"""
物理参数校准器
支持在线校准和导出多种物理引擎格式（MuJoCo, Bullet, Godot）
"""

import json
import time
from pathlib import Path
from typing import Dict, Optional, List
from dataclasses import dataclass, asdict
import xml.etree.ElementTree as ET

from sim2real_gap import PhysicsParams, Sim2RealGapEstimator
from sim2real_analyzer import GapReport

import logging

logger = logging.getLogger(__name__)


@dataclass
class JointParams:
    """关节参数"""

    name: str
    damping: float = 0.1
    friction: float = 0.05
    armature: float = 0.01
    stiffness: float = 0.0
    range_low: float = -45.0
    range_high: float = 90.0


@dataclass
class BodyParams:
    """刚体参数"""

    name: str
    mass: float = 1.0
    inertia: List[float] = None  # [ixx, iyy, izz]
    friction: float = 0.8

    def __post_init__(self) -> None:
        if self.inertia is None:
            self.inertia = [0.01, 0.01, 0.01]


class PhysicsCalibrator:
    """
    物理参数实时校准器

    功能：
    1. 在线校准物理参数
    2. 导出MuJoCo/Bullet/Godot格式
    3. 参数历史和回滚
    """

    CALIBRATABLE_PARAMS = [
        "friction_coefficient",
        "damping_ratio",
        "contact_stiffness",
        "joint_friction",
        "gravity",
        "mass_scale",
    ]

    def __init__(
        self,
        base_params: Optional[PhysicsParams] = None,
        config_dir: Optional[str] = None,
    ):
        if config_dir is None:
            # 默认使用项目根目录下的 configs 文件夹
            config_dir = Path(__file__).resolve().parent.parent / "configs"

        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(parents=True, exist_ok=True)

        # 基础参数
        self.base_params = base_params or PhysicsParams()
        self.current_params = PhysicsParams(**asdict(self.base_params))

        # 关节和刚体参数
        self.joints: Dict[str, JointParams] = {
            "hip_left": JointParams("hip_left", damping=0.1, friction=0.05),
            "hip_right": JointParams("hip_right", damping=0.1, friction=0.05),
        }

        self.bodies: Dict[str, BodyParams] = {
            "torso": BodyParams("torso", mass=5.0, friction=0.8),
            "left_leg": BodyParams("left_leg", mass=2.0, friction=0.8),
            "right_leg": BodyParams("right_leg", mass=2.0, friction=0.8),
        }

        # 校准历史
        self.param_history: List[Dict] = []
        self.max_history = 100

        # Sim2Real集成
        self.gap_estimator: Optional[Sim2RealGapEstimator] = None
        try:
            from sim2real_analyzer import Sim2RealAnalyzer

            self.analyzer = Sim2RealAnalyzer()
        except ImportError:
            self.analyzer = None

        # 统计
        self.calibration_count = 0
        self.calibration_history = []

    def update_from_gap_analysis(self, gap_report: "GapReport") -> Dict[str, float]:
        """
        基于 Sim2Real 差异报告自动校准参数

        Args:
            gap_report: 差异分析报告

        Returns:
            调整后的参数增量
        """
        updates = {}
        current = self.current_params

        # 1. 修正摩擦系数 (基于损耗估算)
        # 增加摩擦系数
        if gap_report.friction_estimate > 0.0:
            # 假设摩擦导致了部分功率损耗
            current.friction_coefficient *= 1.0 + gap_report.friction_estimate
            updates["friction_coefficient"] = current.friction_coefficient

        # 2. 我们还可以根据 torque_scale_factor 调整电机参数
        # 这里假设 PhysicsParams 没有独立的 motor_strength，但我们可以记录建议值
        updates["suggested_torque_scale"] = gap_report.torque_scale_factor

        # 记录
        self.calibration_history.append(
            {
                "timestamp": time.time(),
                "type": "sim2real_gap",
                "updates": updates,
                "gap_report": (
                    asdict(gap_report)
                    if hasattr(gap_report, "__dict__")
                    else str(gap_report)
                ),
            }
        )

        logger.info(
            f"🔧 Sim2Real校准: 摩擦系数->{updates.get('friction_coefficient', 'N/A'):.3f}, 建议转矩缩放->{updates.get('suggested_torque_scale', 'N/A'):.3f}"
        )
        return updates

    def calibrate_online(self, observation: dict, target: Optional[dict] = None) -> None:
        """
        在线校准

        Args:
            observation: 当前观测
            target: 目标状态（如果有真实数据）
        """
        # 记录历史
        self._record_history()

        # 基于观测调整参数
        self._adjust_from_observation(observation)

        # 如果有真实数据，使用Gap估计器
        if target and self.gap_estimator:
            new_params = self.gap_estimator.correct_parameters(observation, target)
            self.current_params = new_params

        self.calibration_count += 1

    def _adjust_from_observation(self, observation: dict) -> None:
        """基于观测调整参数"""
        sensors = observation.get("sensors", {})
        imu = sensors.get("imu", {})
        orient = imu.get("orient", [0, 0, 0])

        roll, pitch = abs(orient[0]), abs(orient[1])

        # 如果不稳定，增加阻尼
        if roll > 15 or pitch > 15:
            self.current_params.damping_ratio = min(
                0.3, self.current_params.damping_ratio * 1.05
            )
        else:
            # 逐渐恢复到基础值
            self.current_params.damping_ratio = (
                0.9 * self.current_params.damping_ratio
                + 0.1 * self.base_params.damping_ratio
            )

    def _record_history(self) -> None:
        """记录参数历史"""
        self.param_history.append(
            {"timestamp": time.time(), "params": self.current_params.to_dict()}
        )

        if len(self.param_history) > self.max_history:
            self.param_history.pop(0)

    def rollback(self, steps: int = 1) -> None:
        """回滚参数"""
        if steps > len(self.param_history):
            steps = len(self.param_history)

        if steps > 0:
            target = self.param_history[-(steps + 1)]
            self.current_params = PhysicsParams.from_dict(target["params"])
            logger.info(f"回滚{steps}步到 {target['timestamp']}")

    def set_gap_estimator(self, estimator: Sim2RealGapEstimator) -> None:
        """设置Gap估计器"""
        self.gap_estimator = estimator

    # =================== 导出功能 ===================

    def export_to_mujoco(self, filename: str = "robot_physics.xml") -> str:
        """导出MuJoCo XML配置"""
        root = ET.Element("mujoco", model="agi_walker")

        # 全局选项
        option = ET.SubElement(root, "option")
        option.set("gravity", f"0 0 -{self.current_params.gravity}")
        option.set("timestep", "0.001")

        # 默认值
        default = ET.SubElement(root, "default")
        joint_default = ET.SubElement(default, "joint")
        joint_default.set("damping", str(self.current_params.damping_ratio * 10))
        joint_default.set("frictionloss", str(self.current_params.joint_friction))

        geom_default = ET.SubElement(default, "geom")
        geom_default.set(
            "friction", f"{self.current_params.friction_coefficient} 0.005 0.0001"
        )

        # 世界体
        worldbody = ET.SubElement(root, "worldbody")

        # 地面
        floor = ET.SubElement(worldbody, "geom")
        floor.set("name", "floor")
        floor.set("type", "plane")
        floor.set("size", "10 10 0.1")
        floor.set("rgba", "0.9 0.9 0.9 1")

        # 机器人躯干
        torso = ET.SubElement(worldbody, "body")
        torso.set("name", "torso")
        torso.set("pos", "0 0 1.5")

        # 躯干几何体
        torso_geom = ET.SubElement(torso, "geom")
        torso_geom.set("type", "box")
        torso_geom.set("size", "0.3 0.2 0.4")
        torso_geom.set(
            "mass", str(self.bodies["torso"].mass * self.current_params.mass_scale)
        )

        # 躯干关节（自由浮动）
        torso_joint = ET.SubElement(torso, "freejoint")
        torso_joint.set("name", "root")

        # 左腿
        left_leg = ET.SubElement(torso, "body")
        left_leg.set("name", "left_leg")
        left_leg.set("pos", "-0.15 0 -0.4")

        left_leg_geom = ET.SubElement(left_leg, "geom")
        left_leg_geom.set("type", "capsule")
        left_leg_geom.set("size", "0.08 0.4")
        left_leg_geom.set(
            "mass", str(self.bodies["left_leg"].mass * self.current_params.mass_scale)
        )

        hip_left = ET.SubElement(left_leg, "joint")
        hip_left.set("name", "hip_left")
        hip_left.set("type", "hinge")
        hip_left.set("axis", "0 1 0")
        hip_left.set(
            "range",
            f"{self.joints['hip_left'].range_low} {self.joints['hip_left'].range_high}",
        )

        # 右腿
        right_leg = ET.SubElement(torso, "body")
        right_leg.set("name", "right_leg")
        right_leg.set("pos", "0.15 0 -0.4")

        right_leg_geom = ET.SubElement(right_leg, "geom")
        right_leg_geom.set("type", "capsule")
        right_leg_geom.set("size", "0.08 0.4")
        right_leg_geom.set(
            "mass", str(self.bodies["right_leg"].mass * self.current_params.mass_scale)
        )

        hip_right = ET.SubElement(right_leg, "joint")
        hip_right.set("name", "hip_right")
        hip_right.set("type", "hinge")
        hip_right.set("axis", "0 1 0")
        hip_right.set(
            "range",
            f"{self.joints['hip_right'].range_low} {self.joints['hip_right'].range_high}",
        )

        # 执行器
        actuator = ET.SubElement(root, "actuator")

        motor_left = ET.SubElement(actuator, "motor")
        motor_left.set("name", "motor_hip_left")
        motor_left.set("joint", "hip_left")
        motor_left.set("gear", "100")

        motor_right = ET.SubElement(actuator, "motor")
        motor_right.set("name", "motor_hip_right")
        motor_right.set("joint", "hip_right")
        motor_right.set("gear", "100")

        # 写入文件
        path = self.config_dir / filename
        tree = ET.ElementTree(root)
        ET.indent(tree, space="  ")
        tree.write(path, encoding="unicode", xml_declaration=True)

        logger.info(f"✅ MuJoCo配置已导出: {path}")
        return str(path)

    def export_to_bullet(self, filename: str = "robot_physics.json") -> dict:
        """导出Bullet参数"""
        config = {
            "physics_engine": "bullet",
            "gravity": [0, 0, -self.current_params.gravity],
            "time_step": 0.001,
            "global_params": {
                "friction_coefficient": self.current_params.friction_coefficient,
                "contact_stiffness": self.current_params.contact_stiffness,
                "contact_damping": self.current_params.damping_ratio * 100,
            },
            "bodies": {},
            "joints": {},
        }

        # 刚体参数
        for name, body in self.bodies.items():
            config["bodies"][name] = {
                "mass": body.mass * self.current_params.mass_scale,
                "friction": body.friction,
                "inertia": body.inertia,
                "linear_damping": 0.01,
                "angular_damping": 0.01,
            }

        # 关节参数
        for name, joint in self.joints.items():
            config["joints"][name] = {
                "type": "hinge",
                "damping": joint.damping,
                "friction": joint.friction + self.current_params.joint_friction,
                "stiffness": joint.stiffness,
                "limits": [joint.range_low, joint.range_high],
                "max_force": 500.0,
            }

        # 写入文件
        path = self.config_dir / filename
        with open(path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)

        logger.info(f"✅ Bullet配置已导出: {path}")
        return config

    def export_to_godot(self, filename: str = "robot_physics.tres") -> dict:
        """导出Godot资源格式"""
        config = {
            "resource_type": "PhysicsConfig",
            "physics_params": {
                "gravity_vector": [0, -self.current_params.gravity, 0],
                "default_friction": self.current_params.friction_coefficient,
                "default_bounce": 0.0,
            },
            "rigidbody_configs": {},
            "joint_configs": {},
        }

        # 刚体配置
        for name, body in self.bodies.items():
            config["rigidbody_configs"][name] = {
                "mass": body.mass * self.current_params.mass_scale,
                "physics_material": {"friction": body.friction, "bounce": 0.0},
                "linear_damp": 0.0,
                "angular_damp": self.current_params.damping_ratio,
            }

        # 关节配置
        for name, joint in self.joints.items():
            config["joint_configs"][name] = {
                "type": "HingeJoint3D",
                "angular_limit_lower": joint.range_low,
                "angular_limit_upper": joint.range_high,
                "motor_max_impulse": 500.0,
                "bias": 0.3,
                "softness": 0.9,
            }

        # 写入文件
        path = self.config_dir / filename.replace(".tres", ".json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)

        logger.info(f"✅ Godot配置已导出: {path}")
        return config

    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "calibration_count": self.calibration_count,
            "history_size": len(self.param_history),
            "current_params": self.current_params.to_dict(),
            "joints": {name: asdict(j) for name, j in self.joints.items()},
            "bodies": {name: asdict(b) for name, b in self.bodies.items()},
        }


# 测试代码
if __name__ == "__main__":
    logger.info("物理参数校准器测试\n")

    # 创建校准器
    calibrator = PhysicsCalibrator()

    # 模拟观测
    observation = {
        "sensors": {
            "imu": {"orient": [5.0, -3.0, 0.0]},
            "joints": {"hip_left": {"angle": 10.0}, "hip_right": {"angle": -8.0}},
        },
        "torso_height": 1.45,
    }

    # 在线校准
    logger.info("=== 在线校准 ===")
    for i in range(5):
        calibrator.calibrate_online(observation)
    logger.info(f"校准次数: {calibrator.calibration_count}")

    # 导出配置
    logger.info("\n=== 导出配置 ===")
    calibrator.export_to_mujoco()
    calibrator.export_to_bullet()
    calibrator.export_to_godot()

    # 统计
    logger.info("\n=== 统计信息 ===")
    logger.info(json.dumps(calibrator.get_stats(), indent=2))
