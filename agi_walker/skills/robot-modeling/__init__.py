"""
Robot Modeling Skill - 快速机器人建模工具

提供 RobotBuilder 类和预设模板加载功能。
"""

import json
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field


@dataclass
class RobotConfig:
    """机器人配置数据类"""
    name: str
    parts: List[Dict[str, Any]] = field(default_factory=list)
    connections: List[Dict[str, Any]] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "name": self.name,
            "parts": self.parts,
            "connections": self.connections,
            "metadata": self.metadata
        }
    
    def save(self, filepath: str) -> None:
        """保存配置到JSON文件"""
        output_path = Path(filepath)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(self.to_dict(), f, indent=2, ensure_ascii=False)
        
        print(f"✓ 机器人配置已保存到: {filepath}")


class RobotBuilder:
    """机器人构建器
    
    使用流式API快速创建机器人配置。
    
    示例:
        robot = (
            RobotBuilder("my_robot")
            .add_torso(height=0.5, mass=5.0)
            .add_leg_pair(thigh_length=0.3, shin_length=0.3)
            .build()
        )
    """
    
    def __init__(self, name: str):
        self.name = name
        self.parts: List[Dict] = []
        self.connections: List[Dict] = []
        self.metadata: Dict = {}
        self._part_id_counter = 0
    
    def _generate_part_id(self, part_type: str) -> str:
        """生成部件ID"""
        self._part_id_counter += 1
        return f"{part_type}_{self._part_id_counter}"
    
    def add_torso(self, height: float = 0.5, mass: float = 5.0, **kwargs) -> 'RobotBuilder':
        """添加躯干
        
        Args:
            height: 躯干高度 (m)
            mass: 质量 (kg)
        """
        part = {
            "id": self._generate_part_id("torso"),
            "type": "torso",
            "params": {
                "height": height,
                "mass": mass,
                **kwargs
            }
        }
        self.parts.append(part)
        return self
    
    def add_leg_pair(
        self,
        thigh_length: float = 0.3,
        shin_length: float = 0.3,
        hip_joint: str = "revolute",
        knee_joint: str = "revolute",
        mass_per_link: float = 1.0, # Added explicit mass arg since we need to split it
        **kwargs
    ) -> 'RobotBuilder':
        """添加一对腿 (双连杆结构: 大腿 + 小腿)
        
        Args:
            thigh_length: 大腿长度 (m)
            shin_length: 小腿长度 (m)
            hip_joint: 髋关节类型
            knee_joint: 膝关节类型
            mass_per_link: 每个连杆的质量 (kg)
        """
        # --- 左腿 ---
        l_thigh = {
            "id": self._generate_part_id("thigh_left"),
            "type": "thigh",
            "side": "left",
            "params": {"length": thigh_length, "mass": mass_per_link, **kwargs}
        }
        l_shin = {
            "id": self._generate_part_id("shin_left"),
            "type": "shin",
            "side": "left",
            "params": {"length": shin_length, "mass": mass_per_link, **kwargs}
        }
        
        # --- 右腿 ---
        r_thigh = {
            "id": self._generate_part_id("thigh_right"),
            "type": "thigh",
            "side": "right",
            "params": {"length": thigh_length, "mass": mass_per_link, **kwargs}
        }
        r_shin = {
            "id": self._generate_part_id("shin_right"),
            "type": "shin",
            "side": "right",
            "params": {"length": shin_length, "mass": mass_per_link, **kwargs}
        }
        
        self.parts.extend([l_thigh, l_shin, r_thigh, r_shin])
        
        # 建立连接: Torso -> Thigh -> Shin
        torso_parts = [p for p in self.parts if p["type"] == "torso"]
        if torso_parts:
            torso_id = torso_parts[0]["id"]
            
            # 1. Torso -> Thigh (Hip Joint)
            # Offset logic needs to be handled by urdf_generator or passed in via kwargs? 
            # Ideally passed via connection params, but currently simple.
            # We'll assume generator handles placement based on side/torso size.
            
            self.connections.append({
                "from": torso_id,
                "to": l_thigh["id"],
                "joint_type": hip_joint,
                "name": "hip_left"
            })
            self.connections.append({
                "from": torso_id,
                "to": r_thigh["id"],
                "joint_type": hip_joint,
                "name": "hip_right"
            })
            
            # 2. Thigh -> Shin (Knee Joint)
            self.connections.append({
                "from": l_thigh["id"],
                "to": l_shin["id"],
                "joint_type": knee_joint,
                "name": "knee_left"
            })
            self.connections.append({
                "from": r_thigh["id"],
                "to": r_shin["id"],
                "joint_type": knee_joint,
                "name": "knee_right"
            })
        
        return self
    
    def add_arm_pair(
        self,
        upper_arm_length: float = 0.3,
        forearm_length: float = 0.25,
        shoulder_joint: str = "revolute",
        **kwargs
    ) -> 'RobotBuilder':
        """添加一对手臂
        
        Args:
            upper_arm_length: 上臂长度 (m)
            forearm_length: 前臂长度 (m)
            shoulder_joint: 肩关节类型
        """
        # 左臂
        left_arm = {
            "id": self._generate_part_id("arm_left"),
            "type": "arm",
            "side": "left",
            "params": {
                "upper_arm_length": upper_arm_length,
                "forearm_length": forearm_length,
                "shoulder_joint": shoulder_joint,
                **kwargs
            }
        }
        # 右臂
        right_arm = {
            "id": self._generate_part_id("arm_right"),
            "type": "arm",
            "side": "right",
            "params": {
                "upper_arm_length": upper_arm_length,
                "forearm_length": forearm_length,
                "shoulder_joint": shoulder_joint,
                **kwargs
            }
        }
        
        self.parts.extend([left_arm, right_arm])
        
        # 自动连接到躯干
        torso_parts = [p for p in self.parts if p["type"] == "torso"]
        if torso_parts:
            torso_id = torso_parts[0]["id"]
            self.connections.append({
                "from": torso_id,
                "to": left_arm["id"],
                "joint_type": shoulder_joint
            })
            self.connections.append({
                "from": torso_id,
                "to": right_arm["id"],
                "joint_type": shoulder_joint
            })
        
        return self
    
    def set_joint_damping(self, damping: float) -> 'RobotBuilder':
        """设置全局关节阻尼"""
        self.metadata["joint_damping"] = damping
        return self
    
    def set_joint_limits(self, joint_name: str, min_angle: float, max_angle: float) -> 'RobotBuilder':
        """设置关节限位"""
        if "joint_limits" not in self.metadata:
            self.metadata["joint_limits"] = {}
        self.metadata["joint_limits"][joint_name] = {
            "min": min_angle,
            "max": max_angle
        }
        return self
    
    def customize(self, **params) -> 'RobotBuilder':
        """自定义参数"""
        for key, value in params.items():
            self.metadata[key] = value
        return self
    
    def build(self) -> RobotConfig:
        """构建机器人配置"""
        return RobotConfig(
            name=self.name,
            parts=self.parts,
            connections=self.connections,
            metadata=self.metadata
        )


def load_template(template_name: str) -> RobotConfig:
    """加载预设模板
    
    Args:
        template_name: 模板名称 (biped_basic/quadruped_dog/wheeled_base/humanoid_upper)
    
    Returns:
        RobotConfig 对象
    
    Raises:
        FileNotFoundError: 如果模板不存在
    """
    # 获取模板路径
    skill_dir = Path(__file__).parent
    template_path = skill_dir / "assets" / "templates" / f"{template_name}.json"
    
    if not template_path.exists():
        available = list((skill_dir / "assets" / "templates").glob("*.json"))
        available_names = [p.stem for p in available]
        raise FileNotFoundError(
            f"模板 '{template_name}' 不存在。\n"
            f"可用模板: {', '.join(available_names)}"
        )
    
    # 加载模板
    with open(template_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    return RobotConfig(
        name=data.get("name", template_name),
        parts=data.get("parts", []),
        connections=data.get("connections", []),
        metadata=data.get("metadata", {})
    )


def list_templates() -> List[str]:
    """列出所有可用模板"""
    skill_dir = Path(__file__).parent
    templates_dir = skill_dir / "assets" / "templates"
    
    if not templates_dir.exists():
        return []
    
    templates = list(templates_dir.glob("*.json"))
    return [t.stem for t in templates]
