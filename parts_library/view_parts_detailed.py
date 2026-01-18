"""
零件参数可视化工具
显示所有零件的详细参数和量化数值
"""

import json
import os
from typing import Dict, List
from tabulate import tabulate


class PartsViewer:
    """零件参数查看器"""
    
    def __init__(self, database_path='parts_library/complete_parts_database.json'):
        with open(database_path, 'r', encoding='utf-8') as f:
            self.data = json.load(f)
        
        self.parts = self.data['parts']
        self.categories = self.data['categories']
    
    def show_all_motors(self):
        """显示所有电机参数"""
        print("\n" + "="*80)
        print("电机类别 - 详细参数")
        print("="*80)
        
        headers = ["ID", "名称", "功率", "电压", "最大扭矩", "最大转速", "重量", "效率", "价格"]
        rows = []
        
        for motor in self.parts['motors']:
            rows.append([
                motor['id'],
                motor['name'],
                motor['specs'].get('power', 'N/A'),
                motor['specs'].get('voltage', 'N/A'),
                motor['specs'].get('max_torque', motor['specs'].get('holding_torque', 'N/A')),
                motor['specs'].get('max_speed', 'N/A'),
                motor['specs'].get('weight', 'N/A'),
                motor['specs'].get('efficiency', 'N/A'),
                f"${motor['price']:.2f}"
            ])
        
        print(tabulate(rows, headers=headers, tablefmt='grid'))
        
        # 性能对比
        print("\n性能对比:")
        print("-"*80)
        for motor in self.parts['motors']:
            power_str = motor['specs'].get('power', '0W')
            power = float(power_str.replace('W', ''))
            weight_str = motor['specs'].get('weight', '0 kg')
            weight = float(weight_str.replace(' kg', ''))
            
            if weight > 0:
                power_density = power / weight
                print(f"{motor['name']:<30} 功率密度: {power_density:.0f} W/kg")
    
    def show_all_sensors(self):
        """显示所有传感器参数"""
        print("\n" + "="*80)
        print("传感器类别 - 详细参数")
        print("="*80)
        
        for sensor in self.parts['sensors']:
            print(f"\n{sensor['id']}: {sensor['name']}")
            print("-"*80)
            print(f"价格: ${sensor['price']:.2f}")
            print(f"供应商: {sensor['supplier']}")
            print(f"型号: {sensor.get('model', 'N/A')}")
            print("\n规格参数:")
            for key, value in sensor['specs'].items():
                print(f"  • {key}: {value}")
            print(f"\n应用: {', '.join(sensor['applications'])}")
    
    def show_all_joints(self):
        """显示所有关节装置参数"""
        print("\n" + "="*80)
        print("关节转动装置类别 - 详细参数")
        print("="*80)
        
        headers = ["ID", "名称", "类型", "减速比", "最大扭矩", "回差", "效率", "重量", "价格"]
        rows = []
        
        for joint in self.parts['joints']:
            rows.append([
                joint['id'],
                joint['name'],
                joint['type'],
                joint['specs'].get('reduction_ratio', 'N/A'),
                joint['specs'].get('max_torque', 'N/A'),
                joint['specs'].get('backlash', 'N/A'),
                joint['specs'].get('efficiency', 'N/A'),
                joint['specs'].get('weight', 'N/A'),
                f"${joint['price']:.2f}"
            ])
        
        print(tabulate(rows, headers=headers, tablefmt='grid'))
        
        # 性能分析
        print("\n性能分析:")
        print("-"*80)
        for joint in self.parts['joints']:
            torque_str = joint['specs'].get('max_torque', '0 Nm')
            if 'Nm' in str(torque_str):
                torque = float(str(torque_str).replace(' Nm', ''))
                weight_str = joint['specs'].get('weight', '1 kg')
                weight = float(str(weight_str).replace(' kg', ''))
                
                if weight > 0:
                    density = torque / weight
                    print(f"{joint['name']:<30} 扭矩密度: {density:.1f} Nm/kg")
    
    def show_all_controllers(self):
        """显示所有控制器参数"""
        print("\n" + "="*80)
        print("控制器类别 - 详细参数")
        print("="*80)
        
        for controller in self.parts['controllers']:
            print(f"\n{controller['id']}: {controller['name']}")
            print("-"*80)
            print(f"价格: ${controller['price']:.2f}")
            print(f"供应商: {controller['supplier']}")
            print(f"库存: {controller['stock']}")
            print("\n规格参数:")
            for key, value in controller['specs'].items():
                print(f"  • {key}: {value}")
            print(f"\n应用: {', '.join(controller['applications'])}")
    
    def show_complete_robot_kit(self, robot_type='biped'):
        """显示完整机器人套件的详细清单"""
        print("\n" + "="*80)
        if robot_type == 'biped':
            print("双足行走机器人完整套件 - 物料清单 (BOM)")
        else:
            print("四足机器人完整套件 - 物料清单 (BOM)")
        print("="*80)
        
        # 找到对应的套件
        for kit in self.data['assemblies']['complete_robot_kits']:
            if (robot_type == 'biped' and 'biped' in kit['type']) or \
               (robot_type == 'quadruped' and 'quadruped' in kit['type']):
                
                print(f"\n套件名称: {kit['name']}")
                print(f"组装时间: {kit['build_time']}")
                print(f"难度等级: {kit['difficulty']}")
                print(f"预计成本: ${kit['total_cost']:.2f}")
                
                # 详细物料清单
                print("\n详细物料清单:")
                print("-"*80)
                
                headers = ["序号", "零件ID", "零件名称", "单价", "数量", "小计"]
                rows = []
                total = 0
                
                for idx, item in enumerate(kit['parts_list'], 1):
                    # 查找零件信息
                    part = self._find_part(item['part_id'])
                    if part:
                        subtotal = part['price'] * item['quantity']
                        total += subtotal
                        rows.append([
                            idx,
                            item['part_id'],
                            part['name'],
                            f"${part['price']:.2f}",
                            item['quantity'],
                            f"${subtotal:.2f}"
                        ])
                
                print(tabulate(rows, headers=headers, tablefmt='grid'))
                print(f"\n总计: ${total:.2f}")
                
                # 按类别统计
                print("\n按类别统计:")
                print("-"*80)
                category_stats = {}
                for item in kit['parts_list']:
                    part = self._find_part(item['part_id'])
                    if part:
                        category = self._get_category(item['part_id'])
                        if category not in category_stats:
                            category_stats[category] = {'count': 0, 'cost': 0}
                        category_stats[category]['count'] += item['quantity']
                        category_stats[category]['cost'] += part['price'] * item['quantity']
                
                for category, stats in category_stats.items():
                    print(f"{self.categories.get(category, category):<20} "
                          f"数量: {stats['count']:<4} "
                          f"成本: ${stats['cost']:.2f}")
    
    def _find_part(self, part_id: str) -> Dict:
        """查找零件"""
        for category, parts_list in self.parts.items():
            for part in parts_list:
                if part['id'] == part_id:
                    return part
        return None
    
    def _get_category(self, part_id: str) -> str:
        """获取零件类别"""
        prefix = part_id.split('-')[0]
        mapping = {
            'MT': 'motors',
            'SN': 'sensors',
            'CT': 'controllers',
            'JT': 'joints',
            'ST': 'structure',
            'PW': 'power',
            'CM': 'communication',
            'AC': 'accessories'
        }
        return mapping.get(prefix, 'unknown')
    
    def show_quantitative_summary(self):
        """显示量化总结"""
        print("\n" + "="*80)
        print("零件库量化统计")
        print("="*80)
        
        # 统计每个类别
        stats = []
        total_parts = 0
        total_value = 0
        
        for category, parts_list in self.parts.items():
            count = len(parts_list)
            total_parts += count
            
            prices = [p['price'] for p in parts_list]
            min_price = min(prices)
            max_price = max(prices)
            avg_price = sum(prices) / len(prices)
            total_value += sum(prices)
            
            stats.append([
                self.categories.get(category, category),
                count,
                f"${min_price:.2f}",
                f"${max_price:.2f}",
                f"${avg_price:.2f}"
            ])
        
        headers = ["类别", "数量", "最低价", "最高价", "平均价"]
        print(tabulate(stats, headers=headers, tablefmt='grid'))
        
        print(f"\n总零件数: {total_parts}")
        print(f"零件总价值: ${total_value:.2f}")
        
        # 性能区间
        print("\n性能参数范围:")
        print("-"*80)
        
        # 电机功率范围
        motor_powers = []
        for motor in self.parts['motors']:
            power_str = motor['specs'].get('power', '0W')
            if 'W' in str(power_str):
                power = float(str(power_str).replace('W', '').replace('k', '000'))
                motor_powers.append(power)
        
        if motor_powers:
            print(f"电机功率范围: {min(motor_powers):.0f}W - {max(motor_powers):.0f}W")
        
        # 关节扭矩范围
        joint_torques = []
        for joint in self.parts['joints']:
            torque_str = joint['specs'].get('max_torque', '0 Nm')
            if 'Nm' in str(torque_str):
                torque = float(str(torque_str).replace(' Nm', ''))
                joint_torques.append(torque)
        
        if joint_torques:
            print(f"关节扭矩范围: {min(joint_torques):.0f}Nm - {max(joint_torques):.0f}Nm")


def main():
    """主函数"""
    try:
        from tabulate import tabulate
    except ImportError:
        print("请先安装 tabulate: pip install tabulate")
        print("或使用简化版显示")
        return
    
    viewer = PartsViewer()
    
    print("="*80)
    print("AGI-Walker 零件库 - 详细参数查看器")
    print("="*80)
    
    # 显示所有类别
    viewer.show_quantitative_summary()
    
    input("\n按回车查看电机详细参数...")
    viewer.show_all_motors()
    
    input("\n按回车查看传感器详细参数...")
    viewer.show_all_sensors()
    
    input("\n按回车查看关节装置详细参数...")
    viewer.show_all_joints()
    
    input("\n按回车查看控制器详细参数...")
    viewer.show_all_controllers()
    
    input("\n按回车查看双足机器人套件清单...")
    viewer.show_complete_robot_kit('biped')
    
    input("\n按回车查看四足机器人套件清单...")
    viewer.show_complete_robot_kit('quadruped')
    
    print("\n" + "="*80)
    print("查看完成！")
    print("="*80)


if __name__ == "__main__":
    main()
