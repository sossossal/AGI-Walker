import json
import os
from typing import Dict, Optional, List
from dataclasses import dataclass

@dataclass
class PartSpec:
    id: str
    category: str # actuator, link, sensor, battery
    name: str
    weight_kg: float
    cost_usd: float
    specs: Dict # 具体的规格字典

class PartsManager:
    """
    零件库管理器
    负责加载零件数据库，查询零件信息，计算总BOM成本等
    """
    
    def __init__(self, library_path: str = None):
        if library_path is None:
            # 默认在同级目录下查找 parts_library.json
            current_dir = os.path.dirname(os.path.abspath(__file__))
            library_path = os.path.join(current_dir, "parts_library.json")
            
        self.library_path = library_path
        self.parts_db = {}
        self._load_library()
        
    def _load_library(self):
        """加载零件库"""
        if not os.path.exists(self.library_path):
            raise FileNotFoundError(f"Parts library not found at: {self.library_path}")
            
        with open(self.library_path, 'r', encoding='utf-8') as f:
            raw_db = json.load(f)
            
        # 解析并展平
        for category, items in raw_db.items():
            for part_id, specs in items.items():
                self.parts_db[part_id] = PartSpec(
                    id=part_id,
                    category=category,
                    name=specs.get("name", "Unknown Part"),
                    weight_kg=specs.get("weight_kg", 0.0),
                    cost_usd=specs.get("cost_usd", 0.0),
                    specs=specs
                )
                
    def get_part(self, part_id: str) -> Optional[PartSpec]:
        """获取零件详情"""
        return self.parts_db.get(part_id)
        
    def list_parts(self, category: str = None) -> List[PartSpec]:
        """列出零件"""
        if category:
            return [p for p in self.parts_db.values() if p.category == category]
        return list(self.parts_db.values())
        
    def calculate_bom(self, part_ids: List[str]) -> Dict:
        """计算 BOM 表信息 (重量、成本)"""
        total_weight = 0.0
        total_cost = 0.0
        details = []
        unknowns = []
        
        for pid in part_ids:
            part = self.get_part(pid)
            if part:
                total_weight += part.weight_kg
                total_cost += part.cost_usd
                details.append({
                    "id": pid,
                    "name": part.name,
                    "cost": part.cost_usd
                })
            else:
                unknowns.append(pid)
                
        return {
            "total_weight_kg": total_weight,
            "total_cost_usd": total_cost,
            "part_count": len(details),
            "details": details,
            "unknown_parts": unknowns
        }

# 测试代码
if __name__ == "__main__":
    pm = PartsManager()
    print(f"Loaded {len(pm.parts_db)} parts.")
    
    # 获取一个电机
    motor = pm.get_part("go_m8010")
    if motor:
        print(f"\nFound Motor: {motor.name}")
        print(f"  Torque: {motor.specs.get('max_torque_nm')} Nm")
        print(f"  Price: ${motor.cost_usd}")
        
    # 计算一个简单机器人的成本
    # 12个电机 + 1个电池 + 1个IMU
    robot_parts = ["go_m8010"] * 12 + ["lipo_4s_5000mah", "imu_comsumer"]
    bom = pm.calculate_bom(robot_parts)
    
    print("\nRobot BOM Analysis:")
    print(f"  Total Weight: {bom['total_weight_kg']:.2f} kg")
    print(f"  Total Cost: ${bom['total_cost_usd']:.2f}")
