"""
成本优化系统
Cost Optimization System

功能:
- 总拥有成本(TCO)计算
- 运营成本分析
- 性能/成本权衡
- 设计方案优化
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class CostBreakdown:
    """成本分解"""
    initial_cost: float = 0.0          # 初始购买成本
    energy_cost: float = 0.0           # 能量成本
    maintenance_cost: float = 0.0      # 维护成本
    replacement_cost: float = 0.0      # 更换成本
    labor_cost: float = 0.0            # 人工成本
    depreciation: float = 0.0          # 折旧


class CostModel:
    """成本模型"""
    
    def __init__(self, parts_config: Dict, economic_params: Dict = None):
        """
        初始化成本模型
        
        参数:
            parts_config: 零件配置
            economic_params: 经济参数 (电价、人工等)
        """
        self.parts_config = parts_config
        
        # 默认经济参数
        self.economic_params = economic_params or {
            'electricity_price_kwh': 1.0,  # 元/kWh
            'labor_rate_hour': 200.0,      # 元/小时
            'interest_rate': 0.05,         # 年利率5%
            'depreciation_years': 5        # 折旧年限
        }
    
    def calculate_initial_cost(self) -> float:
        """计算初始成本"""
        # 这里应该从零件库获取价格
        # 简化示例
        base_cost = 0
        
        # 电机成本
        num_motors = self.parts_config.get('num_motors', 6)
        motor_power = self.parts_config.get('motor_power_multiplier', 1.0)
        motor_unit_cost = 120 * motor_power  # 基础500W电机$120
        base_cost += num_motors * motor_unit_cost
        
        # 控制器
        base_cost += 55  # 树莓派
        
        # 传感器
        base_cost += 25  # IMU
        base_cost += 80 * 2  # 力传感器
        
        # 结构
        base_cost += 150  # 3D打印件
        
        # 电源
        base_cost += 88  # 电池
        
        return base_cost
    
    def calculate_energy_cost(self, avg_power_w: float, operating_hours: float) -> float:
        """
        计算能量成本
        
        参数:
            avg_power_w: 平均功率 (W)
            operating_hours: 运行小时数
        
        返回:
            能量成本 (元)
        """
        kwh = (avg_power_w * operating_hours) / 1000
        return kwh * self.economic_params['electricity_price_kwh']
    
    def calculate_maintenance_cost(self, operating_hours: float, 
                                   maintenance_freq: float = 500) -> float:
        """
        计算维护成本
        
        参数:
            operating_hours: 运行小时数
            maintenance_freq: 维护频率 (小时/次)
        
        返回:
            维护成本 (元)
        """
        num_maintenances = operating_hours / maintenance_freq
        
        # 每次维护成本 = 人工时间 × 工时费率 + 材料
        labor_hours_per_maintenance = 2.0
        materials_per_maintenance = 100.0
        
        cost_per_maintenance = (
            labor_hours_per_maintenance * self.economic_params['labor_rate_hour'] +
            materials_per_maintenance
        )
        
        return num_maintenances * cost_per_maintenance
    
    def calculate_replacement_cost(self, operating_hours: float,
                                   component_lifetimes: Dict) -> float:
        """
        计算更换成本
        
        参数:
            operating_hours: 运行小时数
            component_lifetimes: 组件寿命字典 {name: hours}
        
        返回:
            更换成本 (元)
        """
        total_cost = 0
        
        for component, lifetime in component_lifetimes.items():
            if operating_hours > 0:
                num_replacements = operating_hours / lifetime
                
                # 组件价格 (简化)
                if 'motor' in component.lower():
                    component_cost = 120
                elif 'bearing' in component.lower():
                    component_cost = 30
                elif 'sensor' in component.lower():
                    component_cost = 50
                else:
                    component_cost = 100
                
                # 更换人工
                labor_cost = 1.0 * self.economic_params['labor_rate_hour']
                
                total_cost += num_replacements * (component_cost + labor_cost)
        
        return total_cost
    
    def calculate_tco(self, operating_hours: float, 
                     avg_power_w: float = 200) -> CostBreakdown:
        """
        计算总拥有成本 (TCO)
        
        参数:
            operating_hours: 预计运行小时数
            avg_power_w: 平均功率
        
        返回:
            成本分解
        """
        breakdown = CostBreakdown()
        
        # 初始成本
        breakdown.initial_cost = self.calculate_initial_cost()
        
        # 能量成本
        breakdown.energy_cost = self.calculate_energy_cost(avg_power_w, operating_hours)
        
        # 维护成本
        breakdown.maintenance_cost = self.calculate_maintenance_cost(operating_hours)
        
        # 更换成本 (假设主要组件寿命)
        component_lifetimes = {
            'motor': 5000,
            'bearing': 10000,
            'sensor': 8000
        }
        breakdown.replacement_cost = self.calculate_replacement_cost(
            operating_hours, component_lifetimes
        )
        
        # 折旧
        depreciation_years = self.economic_params['depreciation_years']
        years = operating_hours / 8760  # 转换为年
        breakdown.depreciation = (breakdown.initial_cost / depreciation_years) * years
        
        return breakdown


class DesignOptimizer:
    """设计优化器"""
    
    def __init__(self, constraints: Dict = None):
        """
        初始化优化器
        
        参数:
            constraints: 约束条件
        """
        self.constraints = constraints or {
            'max_cost': 3000,           # 最大成本
            'min_performance': 0.8,     # 最小性能
            'max_weight': 15.0,         # 最大重量 (kg)
            'target_lifetime': 10000    # 目标寿命 (小时)
        }
    
    def evaluate_design(self, config: Dict) -> Dict:
        """
        评估设计方案
        
        参数:
            config: 配置参数
        
        返回:
            评估结果
        """
        cost_model = CostModel(config)
        
        # 计算TCO
        tco = cost_model.calculate_tco(
            operating_hours=self.constraints['target_lifetime'],
            avg_power_w=config.get('avg_power_w', 200)
        )
        
        total_cost = (tco.initial_cost + tco.energy_cost + 
                     tco.maintenance_cost + tco.replacement_cost)
        
        # 评估性能 (简化)
        motor_power = config.get('motor_power_multiplier', 1.0)
        performance_score = min(1.0, motor_power / 1.5)
        
        # 评估重量
        base_weight = 12.0  # kg
        weight = base_weight * config.get('mass_multiplier', 1.0)
        
        # 检查约束
        meets_constraints = (
            total_cost <= self.constraints['max_cost'] and
            performance_score >= self.constraints['min_performance'] and
            weight <= self.constraints['max_weight']
        )
        
        return {
            'config': config,
            'total_cost': total_cost,
            'cost_breakdown': tco,
            'performance_score': performance_score,
            'weight': weight,
            'cost_per_performance': total_cost / max(performance_score, 0.1),
            'meets_constraints': meets_constraints
        }
    
    def optimize(self, num_trials: int = 20) -> List[Dict]:
        """
        优化设计
        
        参数:
            num_trials: 尝试次数
        
        返回:
            优化结果列表 (按性价比排序)
        """
        results = []
        
        # 随机搜索不同配置
        for _ in range(num_trials):
            config = {
                'motor_power_multiplier': np.random.uniform(0.8, 1.5),
                'num_motors': np.random.choice([4, 6, 8]),
                'mass_multiplier': np.random.uniform(0.8, 1.2),
                'avg_power_w': np.random.uniform(150, 300)
            }
            
            result = self.evaluate_design(config)
            
            # 只保留满足约束的方案
            if result['meets_constraints']:
                results.append(result)
        
        # 按性价比排序
        results.sort(key=lambda x: x['cost_per_performance'])
        
        return results
    
    def compare_designs(self, designs: List[Dict]) -> str:
        """对比设计方案"""
        report = []
        report.append("="*70)
        report.append("设计方案对比")
        report.append("="*70)
        
        report.append(f"\n{'方案':<6} {'成本':<10} {'性能':<10} {'重量':<10} {'性价比':<12}")
        report.append("-"*70)
        
        for i, result in enumerate(designs, 1):
            report.append(
                f"{i:<6} "
                f"¥{result['total_cost']:<9.0f} "
                f"{result['performance_score']:<10.2f} "
                f"{result['weight']:<10.1f}kg "
                f"{result['cost_per_performance']:<12.0f}"
            )
        
        return "\n".join(report)


class ROICalculator:
    """投资回报率计算器"""
    
    @staticmethod
    def calculate_roi(initial_investment: float, annual_savings: float, years: int) -> Dict:
        """
        计算ROI
        
        参数:
            initial_investment: 初始投资
            annual_savings: 年节省
            years: 年数
        
        返回:
            ROI分析
        """
        total_savings = annual_savings * years
        net_profit = total_savings - initial_investment
        roi_percentage = (net_profit / initial_investment) * 100
        payback_years = initial_investment / annual_savings if annual_savings > 0 else float('inf')
        
        return {
            'initial_investment': initial_investment,
            'annual_savings': annual_savings,
            'years': years,
            'total_savings': total_savings,
            'net_profit': net_profit,
            'roi_percentage': roi_percentage,
            'payback_years': payback_years
        }


if __name__ == "__main__":
    print("成本优化系统加载完成")
    
    # 示例1: TCO计算
    print("\n" + "="*70)
    print("示例1: 总拥有成本计算")
    print("="*70)
    
    config = {
        'motor_power_multiplier': 1.0,
        'num_motors': 6
    }
    
    cost_model = CostModel(config)
    tco = cost_model.calculate_tco(operating_hours=5000, avg_power_w=200)
    
    print(f"\n运行5000小时的TCO:")
    print(f"  初始成本: ¥{tco.initial_cost:.0f}")
    print(f"  能量成本: ¥{tco.energy_cost:.0f}")
    print(f"  维护成本: ¥{tco.maintenance_cost:.0f}")
    print(f"  更换成本: ¥{tco.replacement_cost:.0f}")
    print(f"  总计: ¥{tco.initial_cost + tco.energy_cost + tco.maintenance_cost + tco.replacement_cost:.0f}")
    
    # 示例2: 设计优化
    print("\n" + "="*70)
    print("示例2: 设计优化")
    print("="*70)
    
    optimizer = DesignOptimizer({
        'max_cost': 10000,
        'min_performance': 0.7,
        'max_weight': 15.0,
        'target_lifetime': 5000
    })
    
    print("\n正在优化... (20个候选方案)")
    results = optimizer.optimize(num_trials=20)
    
    if results:
        print(f"\n找到 {len(results)} 个满足约束的方案")
        print("\n前5个最优方案:")
        print(optimizer.compare_designs(results[:5]))
    else:
        print("未找到满足约束的方案")
