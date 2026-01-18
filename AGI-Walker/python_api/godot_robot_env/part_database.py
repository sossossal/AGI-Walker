"""
Parts Database - Python interface to robot parts library
"""
import json
import os
from typing import Dict, List, Optional
from pathlib import Path


class PartsDatabase:
    """机器人零件数据库接口"""
    
    def __init__(self, parts_library_path: Optional[str] = None):
        """
        初始化零件数据库
        
        Args:
            parts_library_path: 零件库根目录路径
        """
        if parts_library_path is None:
            # 默认路径：相对于当前文件
            current_dir = Path(__file__).parent.parent.parent
            parts_library_path = current_dir / "parts_library"
        
        self.root_path = Path(parts_library_path)
        self.parts_cache: Dict[str, Dict] = {}
        
        if not self.root_path.exists():
            print(f"⚠️  Warning: Parts library not found at {self.root_path}")
        else:
            self._load_all_parts()
    
    def _load_all_parts(self):
        """加载所有零件数据"""
        # 扫描所有 JSON 文件
        json_files = list(self.root_path.rglob("*.json"))
        
        # 排除 schema 文件
        json_files = [f for f in json_files if "schema" not in str(f)]
        
        for json_file in json_files:
            try:
                with open(json_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    part_id = data.get("part_id")
                    if part_id:
                        self.parts_cache[part_id] = data
            except Exception as e:
                print(f"⚠️  Failed to load {json_file}: {e}")
        
        print(f"✅ Loaded {len(self.parts_cache)} parts from database")
    
    def get_part(self, part_id: str) -> Optional[Dict]:
        """
        获取零件数据
        
        Args:
            part_id: 零件 ID
        
        Returns:
            零件数据字典，如果不存在则返回 None
        """
        return self.parts_cache.get(part_id)
    
    def get_parts_by_category(self, category: str) -> List[Dict]:
        """
        按类别获取零件
        
        Args:
            category: 类别名称（如 "actuator_servo"）
        
        Returns:
            零件列表
        """
        return [
            part for part in self.parts_cache.values()
            if part.get("category") == category
        ]
    
    def get_parts_by_manufacturer(self, manufacturer: str) -> List[Dict]:
        """
        按制造商获取零件
        
        Args:
            manufacturer: 制造商名称
        
        Returns:
            零件列表
        """
        return [
            part for part in self.parts_cache.values()
            if part.get("manufacturer") == manufacturer
        ]
    
    def list_all_parts(self) -> List[str]:
        """列出所有零件 ID"""
        return list(self.parts_cache.keys())
    
    def validate_part(self, part_id: str) -> bool:
        """
        验证零件数据完整性
        
        Args:
            part_id: 零件 ID
        
        Returns:
            是否有效
        """
        part = self.get_part(part_id)
        if not part:
            return False
        
        # 检查必需字段
        required_fields = ["part_id", "category", "manufacturer", "model", "specifications"]
        for field in required_fields:
            if field not in part:
                return False
        
        return True
    
    def get_motor_specs(self, part_id: str) -> Optional[Dict]:
        """
        获取电机规格（用于 Gym 环境配置）
        
        Args:
            part_id: 电机零件 ID
        
        Returns:
            包含关键规格的字典
        """
        part = self.get_part(part_id)
        if not part or part.get("category") not in ["actuator_servo", "actuator_motor"]:
            return None
        
        specs = part.get("specifications", {})
        return {
            "stall_torque": specs.get("stall_torque", 1.0),
            "no_load_speed": specs.get("no_load_speed", 60.0),
            "weight": specs.get("weight", 0.1),
            "max_current": specs.get("max_current", 1.0),
            "voltage_range": specs.get("voltage_range", [6, 12]),
            "friction": specs.get("friction", {}),
            "thermal": specs.get("thermal", {})
        }
    
    def create_robot_config(self, parts_spec: List[Dict]) -> Dict:
        """
        从零件列表创建机器人配置
        
        Args:
            parts_spec: 零件规格列表，每项包含 part_id 和用途
                例如: [{"part_id": "dynamixel_xl430_w250", "joint": "hip_left"}]
        
        Returns:
            机器人配置字典
        """
        robot_config = {
            "parts": []
        }
        
        for spec in parts_spec:
            part_id = spec.get("part_id")
            part_data = self.get_part(part_id)
            
            if not part_data:
                print(f"⚠️  Warning: Part {part_id} not found")
                continue
            
            part_config = {
                "part_id": part_id,
                "category": part_data.get("category"),
                "specifications": part_data.get("specifications"),
                **spec  # 包含额外的配置（如 joint 名称）
            }
            
            robot_config["parts"].append(part_config)
        
        return robot_config
    
    def print_statistics(self):
        """打印数据库统计信息"""
        print("\n=== Parts Database Statistics ===")
        print(f"Total parts: {len(self.parts_cache)}")
        
        # 按类别统计
        categories = {}
        for part in self.parts_cache.values():
            cat = part.get("category", "unknown")
            categories[cat] = categories.get(cat, 0) + 1
        
        print("By category:")
        for cat, count in sorted(categories.items()):
            print(f"  - {cat}: {count}")
        
        # 按制造商统计
        manufacturers = {}
        for part in self.parts_cache.values():
            mfr = part.get("manufacturer", "unknown")
            manufacturers[mfr] = manufacturers.get(mfr, 0) + 1
        
        print("By manufacturer:")
        for mfr, count in sorted(manufacturers.items()):
            print(f"  - {mfr}: {count}")
        
        print("=================================\n")
