"""
多机器人模型支持
基类和工厂函数
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import json
from pathlib import Path


class RobotType(Enum):
    """机器人类型"""
    BIPED = "biped"          # 双足
    QUADRUPED = "quadruped"  # 四足
    WHEELED = "wheeled"      # 轮式
    HUMANOID = "humanoid"    # 人形


@dataclass
class JointConfig:
    """关节配置"""
    name: str
    type: str = "hinge"        # hinge, ball, prismatic
    axis: List[float] = field(default_factory=lambda: [0, 1, 0])
    range: Tuple[float, float] = (-90, 90)
    max_torque: float = 100.0
    damping: float = 0.1


@dataclass
class LinkConfig:
    """连杆配置"""
    name: str
    parent: str
    mass: float = 1.0
    shape: str = "box"         # box, capsule, sphere, cylinder
    size: List[float] = field(default_factory=lambda: [0.1, 0.1, 0.1])
    position: List[float] = field(default_factory=lambda: [0, 0, 0])


@dataclass
class RobotConfig:
    """机器人配置"""
    name: str
    type: RobotType
    description: str = ""
    
    # 物理参数
    total_mass: float = 10.0
    height: float = 1.5
    
    # 结构
    links: List[LinkConfig] = field(default_factory=list)
    joints: List[JointConfig] = field(default_factory=list)
    
    # 传感器
    sensors: List[str] = field(default_factory=lambda: ["imu", "joint_encoders"])
    
    # 控制参数
    control_frequency: float = 100.0
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            "name": self.name,
            "type": self.type.value,
            "description": self.description,
            "total_mass": self.total_mass,
            "height": self.height,
            "links": [
                {
                    "name": l.name,
                    "parent": l.parent,
                    "mass": l.mass,
                    "shape": l.shape,
                    "size": l.size,
                    "position": l.position
                }
                for l in self.links
            ],
            "joints": [
                {
                    "name": j.name,
                    "type": j.type,
                    "axis": j.axis,
                    "range": list(j.range),
                    "max_torque": j.max_torque,
                    "damping": j.damping
                }
                for j in self.joints
            ],
            "sensors": self.sensors,
            "control_frequency": self.control_frequency
        }
    
    @classmethod
    def from_dict(cls, d: dict) -> 'RobotConfig':
        """从字典创建"""
        links = [
            LinkConfig(
                name=l["name"],
                parent=l["parent"],
                mass=l.get("mass", 1.0),
                shape=l.get("shape", "box"),
                size=l.get("size", [0.1, 0.1, 0.1]),
                position=l.get("position", [0, 0, 0])
            )
            for l in d.get("links", [])
        ]
        
        joints = [
            JointConfig(
                name=j["name"],
                type=j.get("type", "hinge"),
                axis=j.get("axis", [0, 1, 0]),
                range=tuple(j.get("range", [-90, 90])),
                max_torque=j.get("max_torque", 100.0),
                damping=j.get("damping", 0.1)
            )
            for j in d.get("joints", [])
        ]
        
        return cls(
            name=d["name"],
            type=RobotType(d["type"]),
            description=d.get("description", ""),
            total_mass=d.get("total_mass", 10.0),
            height=d.get("height", 1.5),
            links=links,
            joints=joints,
            sensors=d.get("sensors", ["imu"]),
            control_frequency=d.get("control_frequency", 100.0)
        )


class BaseRobot(ABC):
    """
    机器人基类
    
    定义所有机器人共享的接口
    """
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.state: Dict = {}
        self.is_initialized = False
    
    @abstractmethod
    def init_sensors(self):
        """初始化传感器"""
        pass
    
    @abstractmethod
    def get_observation(self) -> Dict:
        """获取观测"""
        pass
    
    @abstractmethod
    def apply_action(self, action: Dict):
        """应用动作"""
        pass
    
    @abstractmethod
    def reset(self):
        """重置状态"""
        pass
    
    def get_joint_names(self) -> List[str]:
        """获取关节名称列表"""
        return [j.name for j in self.config.joints]
    
    def get_action_dim(self) -> int:
        """获取动作空间维度"""
        return len(self.config.joints)
    
    def get_observation_dim(self) -> int:
        """获取观测空间维度"""
        # 基础: IMU(6) + 关节位置 + 关节速度
        return 6 + 2 * len(self.config.joints)


class BipedRobot(BaseRobot):
    """双足机器人"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        if config is None:
            config = self._default_config()
        super().__init__(config)
    
    def _default_config(self) -> RobotConfig:
        """默认双足配置"""
        return RobotConfig(
            name="AGI-Walker Biped",
            type=RobotType.BIPED,
            description="双足步行机器人",
            total_mass=9.0,
            height=1.5,
            links=[
                LinkConfig("torso", "world", mass=5.0, shape="box", size=[0.3, 0.2, 0.4]),
                LinkConfig("left_thigh", "torso", mass=1.0, shape="capsule", size=[0.08, 0.3]),
                LinkConfig("right_thigh", "torso", mass=1.0, shape="capsule", size=[0.08, 0.3]),
                LinkConfig("left_shin", "left_thigh", mass=0.5, shape="capsule", size=[0.06, 0.3]),
                LinkConfig("right_shin", "right_thigh", mass=0.5, shape="capsule", size=[0.06, 0.3]),
            ],
            joints=[
                JointConfig("hip_left", "hinge", [1, 0, 0], (-45, 90), 200, 0.1),
                JointConfig("hip_right", "hinge", [1, 0, 0], (-45, 90), 200, 0.1),
                JointConfig("knee_left", "hinge", [1, 0, 0], (0, 120), 150, 0.1),
                JointConfig("knee_right", "hinge", [1, 0, 0], (0, 120), 150, 0.1),
            ],
            sensors=["imu", "joint_encoders", "foot_contact"],
            control_frequency=100.0
        )
    
    def init_sensors(self):
        self.state = {
            "imu": {"orient": [0, 0, 0], "gyro": [0, 0, 0]},
            "joints": {j.name: {"angle": 0, "velocity": 0} for j in self.config.joints},
            "contacts": {"left_foot": False, "right_foot": False}
        }
        self.is_initialized = True
    
    def get_observation(self) -> Dict:
        return self.state.copy()
    
    def apply_action(self, action: Dict):
        # 应用关节目标
        for joint_name, target in action.items():
            if joint_name in self.state["joints"]:
                # 模拟关节响应
                current = self.state["joints"][joint_name]["angle"]
                self.state["joints"][joint_name]["angle"] = 0.9 * current + 0.1 * target
    
    def reset(self):
        self.init_sensors()


class QuadrupedRobot(BaseRobot):
    """四足机器人"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        if config is None:
            config = self._default_config()
        super().__init__(config)
    
    def _default_config(self) -> RobotConfig:
        """默认四足配置"""
        legs = ["front_left", "front_right", "back_left", "back_right"]
        
        links = [LinkConfig("body", "world", mass=5.0, shape="box", size=[0.4, 0.2, 0.3])]
        joints = []
        
        for leg in legs:
            links.append(LinkConfig(f"{leg}_thigh", "body", mass=0.5, shape="capsule", size=[0.04, 0.15]))
            links.append(LinkConfig(f"{leg}_shin", f"{leg}_thigh", mass=0.3, shape="capsule", size=[0.03, 0.15]))
            
            joints.append(JointConfig(f"{leg}_hip", "hinge", [1, 0, 0], (-45, 45), 100, 0.1))
            joints.append(JointConfig(f"{leg}_knee", "hinge", [1, 0, 0], (0, 90), 80, 0.1))
        
        return RobotConfig(
            name="Quadruped Robot",
            type=RobotType.QUADRUPED,
            description="四足步行机器人",
            total_mass=8.0,
            height=0.4,
            links=links,
            joints=joints,
            sensors=["imu", "joint_encoders", "foot_contact"],
            control_frequency=200.0
        )
    
    def init_sensors(self):
        self.state = {
            "imu": {"orient": [0, 0, 0], "gyro": [0, 0, 0]},
            "joints": {j.name: {"angle": 0, "velocity": 0} for j in self.config.joints}
        }
        self.is_initialized = True
    
    def get_observation(self) -> Dict:
        return self.state.copy()
    
    def apply_action(self, action: Dict):
        for joint_name, target in action.items():
            if joint_name in self.state["joints"]:
                current = self.state["joints"][joint_name]["angle"]
                self.state["joints"][joint_name]["angle"] = 0.9 * current + 0.1 * target
    
    def reset(self):
        self.init_sensors()


class WheeledRobot(BaseRobot):
    """轮式机器人"""
    
    def __init__(self, config: Optional[RobotConfig] = None):
        if config is None:
            config = self._default_config()
        super().__init__(config)
    
    def _default_config(self) -> RobotConfig:
        """默认轮式配置"""
        return RobotConfig(
            name="Wheeled Robot",
            type=RobotType.WHEELED,
            description="差速轮式机器人",
            total_mass=5.0,
            height=0.3,
            links=[
                LinkConfig("chassis", "world", mass=4.0, shape="box", size=[0.3, 0.2, 0.1]),
                LinkConfig("left_wheel", "chassis", mass=0.5, shape="cylinder", size=[0.05, 0.02]),
                LinkConfig("right_wheel", "chassis", mass=0.5, shape="cylinder", size=[0.05, 0.02]),
            ],
            joints=[
                JointConfig("left_wheel", "continuous", [0, 1, 0], (-360, 360), 50, 0.01),
                JointConfig("right_wheel", "continuous", [0, 1, 0], (-360, 360), 50, 0.01),
            ],
            sensors=["imu", "wheel_encoders", "lidar"],
            control_frequency=50.0
        )
    
    def init_sensors(self):
        self.state = {
            "imu": {"orient": [0, 0, 0], "gyro": [0, 0, 0]},
            "wheels": {"left": 0, "right": 0},
            "velocity": {"linear": 0, "angular": 0}
        }
        self.is_initialized = True
    
    def get_observation(self) -> Dict:
        return self.state.copy()
    
    def apply_action(self, action: Dict):
        # 差速控制
        left_speed = action.get("left_wheel", 0)
        right_speed = action.get("right_wheel", 0)
        
        self.state["wheels"]["left"] = left_speed
        self.state["wheels"]["right"] = right_speed
        self.state["velocity"]["linear"] = (left_speed + right_speed) / 2
        self.state["velocity"]["angular"] = (right_speed - left_speed) / 0.3  # 轴距
    
    def reset(self):
        self.init_sensors()


def create_robot(
    robot_type: str,
    config_path: Optional[str] = None
) -> BaseRobot:
    """
    工厂函数：创建机器人实例
    
    Args:
        robot_type: 机器人类型（"biped", "quadruped", "wheeled"）
        config_path: 配置文件路径
    
    Returns:
        机器人实例
    """
    # 加载配置
    config = None
    if config_path:
        path = Path(config_path)
        if path.exists():
            with open(path, 'r', encoding='utf-8') as f:
                config_dict = json.load(f)
            config = RobotConfig.from_dict(config_dict)
    
    # 创建实例
    robot_type = robot_type.lower()
    
    if robot_type == "biped":
        return BipedRobot(config)
    elif robot_type == "quadruped":
        return QuadrupedRobot(config)
    elif robot_type == "wheeled":
        return WheeledRobot(config)
    else:
        raise ValueError(f"未知的机器人类型: {robot_type}")


def save_robot_config(config: RobotConfig, path: str):
    """保存机器人配置"""
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(config.to_dict(), f, indent=2, ensure_ascii=False)
    print(f"✅ 配置已保存: {path}")


# 测试代码
if __name__ == "__main__":
    print("多机器人模型测试\n")
    
    # 创建各类型机器人
    print("=== 双足机器人 ===")
    biped = create_robot("biped")
    biped.init_sensors()
    print(f"名称: {biped.config.name}")
    print(f"关节: {biped.get_joint_names()}")
    print(f"动作维度: {biped.get_action_dim()}")
    
    print("\n=== 四足机器人 ===")
    quadruped = create_robot("quadruped")
    quadruped.init_sensors()
    print(f"名称: {quadruped.config.name}")
    print(f"关节: {quadruped.get_joint_names()}")
    print(f"动作维度: {quadruped.get_action_dim()}")
    
    print("\n=== 轮式机器人 ===")
    wheeled = create_robot("wheeled")
    wheeled.init_sensors()
    print(f"名称: {wheeled.config.name}")
    print(f"控制频率: {wheeled.config.control_frequency}Hz")
    
    # 保存配置
    print("\n=== 保存配置 ===")
    save_robot_config(biped.config, "d:/新建文件夹/AGI-Walker/robot_models/biped/config.json")
    save_robot_config(quadruped.config, "d:/新建文件夹/AGI-Walker/robot_models/quadruped/config.json")
    save_robot_config(wheeled.config, "d:/新建文件夹/AGI-Walker/robot_models/wheeled/config.json")
