"""
URDF Generator Skill - AGI-Walker到URDF/SDF格式转换器

支持转换到URDF、SDF和MJCF格式,用于Gazebo、MuJoCo、PyBullet等仿真器。
"""

import json
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
try:
    from lxml import etree
except ImportError:
    import xml.etree.ElementTree as etree


@dataclass
class URDFLink:
    """URDF链接"""
    name: str
    mass: float
    inertia: np.ndarray  # 3x3惯性张量
    visual_geometry: Dict[str, Any]
    collision_geometry: Dict[str, Any]
    origin: Tuple[float, float, float] = (0, 0, 0)


@dataclass
class URDFJoint:
    """URDF关节"""
    name: str
    type: str  # fixed/revolute/prismatic/continuous
    parent: str
    child: str
    origin: Tuple[float, float, float] = (0, 0, 0)
    axis: Tuple[float, float, float] = (1, 0, 0)
    limits: Optional[Dict[str, float]] = None


class URDFGenerator:
    """URDF生成器
    
    将AGI-Walker JSON配置转换为URDF/SDF格式。
    """
    
    def __init__(self):
        self.links: List[URDFLink] = []
        self.joints: List[URDFJoint] = []
        self.robot_name = "robot"
        self.config = None
    
    def load_config(self, config: Any) -> None:
        """加载机器人配置
        
        Args:
            config: 配置文件路径、字典或RobotConfig对象
        """
        if isinstance(config, (str, Path)):
            with open(config, 'r', encoding='utf-8') as f:
                self.config = json.load(f)
        elif isinstance(config, dict):
            self.config = config
        else:
            # RobotConfig对象
            self.config = config.to_dict()
        
        self.robot_name = self.config.get("name", "robot")
        self._build_urdf_structure()
    
    def _build_urdf_structure(self) -> None:
        """从配置构建URDF结构"""
        # 转换部件为links
        for part in self.config.get("parts", []):
            link = self._part_to_link(part)
            self.links.append(link)
        
        # 转换连接为joints
        for conn in self.config.get("connections", []):
            joint = self._connection_to_joint(conn)
            self.joints.append(joint)
        
        # 添加base_link (固定到地面)
        if self.links:
            base_joint = URDFJoint(
                name="base_joint",
                type="fixed",
                parent="world",
                child=self.links[0].name
            )
            self.joints.insert(0, base_joint)
    
    def _part_to_link(self, part: Dict) -> URDFLink:
        """将部件转换为URDF link"""
        part_id = part["id"]
        params = part["params"]
        part_type = part["type"]
        
        mass = params.get("mass", 1.0)
        
        # 简化惯性张量 (使用盒子模型)
        height = params.get("height", 0.3)
        width = params.get("width", 0.2)
        depth = params.get("depth", 0.2)
        
        ixx = (1.0/12.0) * mass * (width**2 + depth**2)
        iyy = (1.0/12.0) * mass * (height**2 + depth**2)
        izz = (1.0/12.0) * mass * (height**2 + width**2)
        
        inertia = np.array([
            [ixx, 0, 0],
            [0, iyy, 0],
            [0, 0, izz]
        ])
        
        # 几何形状
        if part_type == "torso":
            geometry = {
                "type": "box",
                "size": [width, depth, height]
            }
        elif part_type in ["thigh", "shin", "leg"]: # 'leg' kept for compatibility
            length = params.get("length", params.get("thigh_length", 0.3) + params.get("shin_length", 0))
            radius = 0.04
            geometry = {
                "type": "cylinder",
                "radius": radius,
                "length": length
            }
        else:
            geometry = {
                "type": "box",
                "size": [0.1, 0.1, 0.1]
            }
        
        return URDFLink(
            name=part_id,
            mass=mass,
            inertia=inertia,
            visual_geometry=geometry,
            collision_geometry=geometry
        )
    
    def _connection_to_joint(self, conn: Dict) -> URDFJoint:
        """将连接转换为URDF joint"""
        parent_id = conn["from"]
        child_id = conn["to"]
        joint_type = conn.get("joint_type", "fixed")
        joint_name = conn.get("name", f"{parent_id}_to_{child_id}")
        
        # 查找部件获取尺寸信息
        parent_part = next(p for p in self.config["parts"] if p["id"] == parent_id)
        child_part = next(p for p in self.config["parts"] if p["id"] == child_id)
        
        # 默认原点
        origin = [0, 0, 0]
        
        # --- 自动计算关节位置 ---
        
        # Case 1: Torso -> Thigh (Hip)
        if parent_part["type"] == "torso" and child_part["type"] == "thigh":
            side = child_part.get("side", "left")
            torso_width = parent_part["params"].get("size", [0.2,0.2,0.2])[0]
            # Hip at side of torso
            x_offset = (torso_width / 2) * (1 if side == "left" else -1)
            origin = [x_offset, 0, -0.05] # Slightly below torso center
            
        # Case 2: Thigh -> Shin (Knee)
        elif parent_part["type"] == "thigh" and child_part["type"] == "shin":
            # Knee at bottom of thigh
            thigh_len = parent_part["params"].get("length", 0.3)
            # Cylinder origin is center, so joint should be at -L/2
            # BUT: URDF cylinder origin is center. So if we attach at bottom of thigh...
            # Standard: Joint origin is relative to Parent Link Frame.
            # If Parent (Thigh) origin is its center:
            origin = [0, 0, -thigh_len] 
            
        # Case 3: Legacy support or generic
        elif parent_part["type"] == "torso" and child_part["type"] == "leg":
             side = child_part.get("side", "left")
             x_offset = 0.15 if side == "left" else -0.15
             origin = [x_offset, 0, -0.1]
        
        # 转换关节类型
        if joint_type == "revolute":
            urdf_type = "revolute"
            # Hip X/Y? Knee Y? Assuming simple pitch (walking forward) for now
            # Knee usually rotates around Y axis (pitch)
            axis = (0, 1, 0)
            limits = {
                "lower": -1.57,
                "upper": 1.57,
                "effort": 100.0,
                "velocity": 10.0
            }
        elif joint_type == "prismatic":
            urdf_type = "prismatic"
            axis = (0, 0, 1)
            limits = {
                "lower": -0.5,
                "upper": 0.5,
                "effort": 100.0,
                "velocity": 1.0
            }
        else:
            urdf_type = "fixed"
            axis = (0, 0, 0)
            limits = None
        
        return URDFJoint(
            name=joint_name,
            type=urdf_type,
            parent=parent_id,
            child=child_id,
            origin=tuple(origin),
            axis=axis,
            limits=limits
        )
    
    def generate_urdf_xml(self) -> etree.Element:
        """生成URDF XML结构"""
        root = etree.Element("robot", name=self.robot_name)
        
        # 添加links
        for link in self.links:
            link_elem = self._create_link_element(link)
            root.append(link_elem)
        
        # 添加joints
        for joint in self.joints:
            joint_elem = self._create_joint_element(joint)
            root.append(joint_elem)
        
        return root
    
    def _create_link_element(self, link: URDFLink) -> etree.Element:
        """创建link XML元素"""
        link_elem = etree.Element("link", name=link.name)
        
        # 惯性
        inertial = etree.SubElement(link_elem, "inertial")
        etree.SubElement(inertial, "mass", value=str(link.mass))
        
        inertia_elem = etree.SubElement(inertial, "inertia")
        inertia_elem.set("ixx", f"{link.inertia[0,0]:.6f}")
        inertia_elem.set("iyy", f"{link.inertia[1,1]:.6f}")
        inertia_elem.set("izz", f"{link.inertia[2,2]:.6f}")
        inertia_elem.set("ixy", "0")
        inertia_elem.set("ixz", "0")
        inertia_elem.set("iyz", "0")
        
        # 视觉
        visual = etree.SubElement(link_elem, "visual")
        vis_geom = etree.SubElement(visual, "geometry")
        self._add_geometry(vis_geom, link.visual_geometry)
        
        # 材质
        material = etree.SubElement(visual, "material", name=f"{link.name}_material")
        color = etree.SubElement(material, "color")
        color.set("rgba", "0.8 0.8 0.8 1.0")
        
        # 碰撞
        collision = etree.SubElement(link_elem, "collision")
        col_geom = etree.SubElement(collision, "geometry")
        self._add_geometry(col_geom, link.collision_geometry)
        
        return link_elem
    
    def _add_geometry(self, parent: etree.Element, geometry: Dict) -> None:
        """添加几何形状"""
        geom_type = geometry["type"]
        
        if geom_type == "box":
            box = etree.SubElement(parent, "box")
            size = geometry["size"]
            box.set("size", f"{size[0]} {size[1]} {size[2]}")
        
        elif geom_type == "cylinder":
            cylinder = etree.SubElement(parent, "cylinder")
            cylinder.set("radius", str(geometry["radius"]))
            cylinder.set("length", str(geometry["length"]))
        
        elif geom_type == "sphere":
            sphere = etree.SubElement(parent, "sphere")
            sphere.set("radius", str(geometry["radius"]))
    
    def _create_joint_element(self, joint: URDFJoint) -> etree.Element:
        """创建joint XML元素"""
        joint_elem = etree.Element("joint", name=joint.name, type=joint.type)
        
        # 父子关系
        etree.SubElement(joint_elem, "parent", link=joint.parent)
        etree.SubElement(joint_elem, "child", link=joint.child)
        
        # 原点
        origin = etree.SubElement(joint_elem, "origin")
        origin.set("xyz", f"{joint.origin[0]} {joint.origin[1]} {joint.origin[2]}")
        origin.set("rpy", "0 0 0")
        
        # 轴
        if joint.type != "fixed":
            axis = etree.SubElement(joint_elem, "axis")
            axis.set("xyz", f"{joint.axis[0]} {joint.axis[1]} {joint.axis[2]}")
        
        # 限位
        if joint.limits:
            limit = etree.SubElement(joint_elem, "limit")
            limit.set("lower", str(joint.limits["lower"]))
            limit.set("upper", str(joint.limits["upper"]))
            limit.set("effort", str(joint.limits["effort"]))
            limit.set("velocity", str(joint.limits["velocity"]))
        
        return joint_elem
    
    def export_urdf(self, output_file: str) -> None:
        """导出URDF文件"""
        xml_root = self.generate_urdf_xml()
        
        # 格式化XML
        # 格式化XML
        try:
            # 尝试使用 lxml 的 pretty_print
            xml_str = etree.tostring(
                xml_root,
                pretty_print=True,
                xml_declaration=True,
                encoding='UTF-8'
            )
        except TypeError:
            # xml.etree.ElementTree 不支持 pretty_print
            # 手动缩进 (简单实现)
            from xml.dom import minidom
            xml_str_raw = etree.tostring(xml_root, encoding='UTF-8')
            xml_str = minidom.parseString(xml_str_raw).toprettyxml(indent="  ", encoding='UTF-8')
        
        # 保存文件
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'wb') as f:
            f.write(xml_str)
        
        print(f"✓ URDF已导出到: {output_file}")


# 便捷函数

def convert_to_urdf(
    input_file: str,
    output_file: str,
    generate_meshes: bool = False
) -> None:
    """转换AGI-Walker配置为URDF
    
    Args:
        input_file: 输入JSON配置文件
        output_file: 输出URDF文件
        generate_meshes: 是否生成碰撞网格
        
    Example:
        >>> convert_to_urdf(
        ...     "configs/robot.json",
        ...     "exports/robot.urdf",
        ...     generate_meshes=True
        ... )
    """
    generator = URDFGenerator()
    generator.load_config(input_file)
    
    if generate_meshes:
        print("网格生成功能尚未实现")
    
    generator.export_urdf(output_file)


def convert_to_sdf(
    input_file: str,
    output_file: str,
    world_file: bool = False
) -> None:
    """转换为SDF格式 (简化版,基于URDF)
    
    Args:
        input_file: 输入JSON配置
        output_file: 输出SDF文件
        world_file: 是否生成世界文件
    """
    # 简化实现: 先转URDF,再转SDF
    urdf_temp = output_file.replace(".sdf", "_temp.urdf")
    convert_to_urdf(input_file, urdf_temp)
    
    print("注意: SDF转换当前通过URDF中转")
    print(f"临时URDF文件: {urdf_temp}")
    print(f"请使用 gz sdf -p {urdf_temp} > {output_file} 完成转换")


def validate_urdf(urdf_file: str) -> bool:
    """验证URDF文件有效性
    
    Args:
        urdf_file: URDF文件路径
        
    Returns:
        是否有效
    """
    try:
        tree = etree.parse(urdf_file)
        root = tree.getroot()
        
        if root.tag != "robot":
            print("错误: 根元素不是 'robot'")
            return False
        
        links = root.findall(".//link")
        joints = root.findall(".//joint")
        
        print("✓ URDF验证通过")
        print(f"  - Links: {len(links)}")
        print(f"  - Joints: {len(joints)}")
        
        return True
    
    except Exception as e:
        print(f"✗ URDF验证失败: {e}")
        return False
