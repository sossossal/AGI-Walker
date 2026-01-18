"""
零件库管理工具
用于查询、计算成本和生成清单
"""

import json
import os
from typing import List, Dict, Optional


class PartsLibrary:
    """零件库管理器"""
    
    def __init__(self, database_path='parts_library/complete_parts_database.json'):
        with open(database_path, 'r', encoding='utf-8') as f:
            self.data = json.load(f)
        
        self.parts = self.data['parts']
        self.assemblies = self.data['assemblies']
    
    def search_by_type(self, part_type: str) -> List[Dict]:
        """按类型搜索零件"""
        results = []
        for category, parts_list in self.parts.items():
            for part in parts_list:
                if part.get('type') == part_type:
                    results.append(part)
        return results
    
    def search_by_name(self, name: str) -> List[Dict]:
        """按名称搜索零件"""
        results = []
        for category, parts_list in self.parts.items():
            for part in parts_list:
                if name.lower() in part['name'].lower():
                    results.append(part)
        return results
    
    def get_part_by_id(self, part_id: str) -> Optional[Dict]:
        """根据ID获取零件"""
        for category, parts_list in self.parts.items():
            for part in parts_list:
                if part['id'] == part_id:
                    return part
        return None
    
    def calculate_assembly_cost(self, assembly_name: str) -> Dict:
        """计算组件成本"""
        # 查找组件
        assembly = None
        for kit in self.assemblies['complete_robot_kits']:
            if kit['name'] == assembly_name:
                assembly = kit
                break
        
        if not assembly:
            return {"error": "组件未找到"}
        
        # 计算详细成本
        parts_breakdown = []
        total_cost = 0
        
        for item in assembly['parts_list']:
            part = self.get_part_by_id(item['part_id'])
            if part:
                quantity = item['quantity']
                item_cost = part['price'] * quantity
                total_cost += item_cost
                
                parts_breakdown.append({
                    'id': part['id'],
                    'name': part['name'],
                    'unit_price': part['price'],
                    'quantity': quantity,
                    'total': item_cost
                })
        
        return {
            'assembly_name': assembly_name,
            'parts_breakdown': parts_breakdown,
            'total_cost': total_cost,
            'build_time': assembly.get('build_time', 'N/A'),
            'difficulty': assembly.get('difficulty', 'N/A')
        }
    
    def generate_bom(self, assembly_name: str, output_file: Optional[str] = None):
        """生成物料清单（BOM）"""
        cost_data = self.calculate_assembly_cost(assembly_name)
        
        if 'error' in cost_data:
            print(f"错误: {cost_data['error']}")
            return
        
        # 生成BOM文本
        bom_text = []
        bom_text.append("="*80)
        bom_text.append(f"物料清单 (BOM) - {cost_data['assembly_name']}")
        bom_text.append("="*80)
        bom_text.append("")
        bom_text.append(f"项目信息:")
        bom_text.append(f"  预计组装时间: {cost_data['build_time']}")
        bom_text.append(f"  难度等级: {cost_data['difficulty']}")
        bom_text.append("")
        bom_text.append("-"*80)
        bom_text.append(f"{'ID':<10} {'名称':<30} {'单价':>10} {'数量':>8} {'小计':>12}")
        bom_text.append("-"*80)
        
        for part in cost_data['parts_breakdown']:
            bom_text.append(
                f"{part['id']:<10} {part['name']:<30} "
                f"${part['unit_price']:>9.2f} {part['quantity']:>8} "
                f"${part['total']:>11.2f}"
            )
        
        bom_text.append("-"*80)
        bom_text.append(f"{'总计:':<51} ${cost_data['total_cost']:>11.2f}")
        bom_text.append("="*80)
        
        bom_content = "\n".join(bom_text)
        
        # 输出
        print(bom_content)
        
        # 保存到文件
        if output_file:
            with open(output_file, 'w', encoding='utf-8') as f:
                f.write(bom_content)
            print(f"\nBOM已保存至: {output_file}")
    
    def list_all_motors(self):
        """列出所有电机"""
        print("\n可用电机:")
        print("-"*80)
        for motor in self.parts['motors']:
            print(f"{motor['id']}: {motor['name']}")
            print(f"  功率: {motor['specs']['power']}, "
                  f"扭矩: {motor['specs']['max_torque']}, "
                  f"价格: ${motor['price']:.2f}")
    
    def list_all_sensors(self):
        """列出所有传感器"""
        print("\n可用传感器:")
        print("-"*80)
        for sensor in self.parts['sensors']:
            print(f"{sensor['id']}: {sensor['name']}")
            print(f"  类型: {sensor['type']}, 价格: ${sensor['price']:.2f}")


# 命令行接口
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='零件库管理工具')
    parser.add_argument('--search', help='搜索零件')
    parser.add_argument('--bom', help='生成BOM (双足/四足)')
    parser.add_argument('--list', choices=['motors', 'sensors', 'all'], 
                        help='列出零件')
    
    args = parser.parse_args()
    
    lib = PartsLibrary()
    
    if args.search:
        results = lib.search_by_name(args.search)
        print(f"\n搜索结果 ('{args.search}'):")
        for part in results:
            print(f"  {part['id']}: {part['name']} - ${part['price']:.2f}")
    
    elif args.bom:
        if args.bom == '双足':
            lib.generate_bom("双足行走机器人完整套件", "BOM_biped.txt")
        elif args.bom == '四足':
            lib.generate_bom("四足机器人完整套件", "BOM_quadruped.txt")
    
    elif args.list:
        if args.list == 'motors':
            lib.list_all_motors()
        elif args.list == 'sensors':
            lib.list_all_sensors()
        elif args.list == 'all':
            lib.list_all_motors()
            lib.list_all_sensors()
    
    else:
        print("使用方法:")
        print("  python parts_manager.py --search 电机")
        print("  python parts_manager.py --bom 双足")
        print("  python parts_manager.py --list motors")
