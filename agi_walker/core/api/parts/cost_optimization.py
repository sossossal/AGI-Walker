"""
閹存劖婀版导妯哄缁崵绮?
Cost Optimization System

閸旂喕鍏?
- 閹粯瀚㈤張澶嬪灇閺?TCO)鐠侊紕鐣?
- 鏉╂劘鎯€閹存劖婀伴崚鍡樼€?
- 閹嗗厴/閹存劖婀伴弶鍐€€
- 鐠佹崘顓搁弬瑙勵攳娴兼ê瀵?
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass


@dataclass
class CostBreakdown:
    """閹存劖婀伴崚鍡毿?""

    initial_cost: float = 0.0  # 閸掓繂顫愮拹顓濇嫳閹存劖婀?
    energy_cost: float = 0.0  # 閼充粙鍣洪幋鎰拱
    maintenance_cost: float = 0.0  # 缂佸瓨濮㈤幋鎰拱
    replacement_cost: float = 0.0  # 閺囧瓨宕查幋鎰拱
    labor_cost: float = 0.0  # 娴滃搫浼愰幋鎰拱
    depreciation: float = 0.0  # 閹舵ɑ妫?


class CostModel:
    """閹存劖婀板Ο鈥崇€?""

    def __init__(self, parts_config: Dict, economic_params: Dict = None):
        """
        閸掓繂顫愰崠鏍ㄥ灇閺堫剚膩閸?

        閸欏倹鏆?
            parts_config: 闂嗘湹娆㈤柊宥囩枂
            economic_params: 缂佸繑绁归崣鍌涙殶 (閻㈠吀鐜妴浣锋眽瀹搞儳鐡?
        """
        self.parts_config = parts_config

        # 姒涙顓荤紒蹇旂ス閸欏倹鏆?
        self.economic_params = economic_params or {
            "electricity_price_kwh": 1.0,  # 閸?kWh
            "labor_rate_hour": 200.0,  # 閸?鐏忓繑妞?
            "interest_rate": 0.05,  # 楠炴潙鍩勯悳?%
            "depreciation_years": 5,  # 閹舵ɑ妫獮鎾
        }

    def calculate_initial_cost(self) -> float:
        """鐠侊紕鐣婚崚婵嗩潗閹存劖婀?""
        # 鏉╂瑩鍣锋惔鏃囶嚉娴犲酣娴傛禒璺虹氨閼惧嘲褰囨禒閿嬬壐
        # 缁犫偓閸栨牜銇氭笟?
        base_cost = 0

        # 閻㈠灚婧€閹存劖婀?
        num_motors = self.parts_config.get("num_motors", 6)
        motor_power = self.parts_config.get("motor_power_multiplier", 1.0)
        motor_unit_cost = 120 * motor_power  # 閸╄櫣顢?00W閻㈠灚婧€$120
        base_cost += num_motors * motor_unit_cost

        # 閹貉冨煑閸?
        base_cost += 55  # 閺嶆垼甯楀ú?

        # 娴肩姵鍔呴崳?
        base_cost += 25  # IMU
        base_cost += 80 * 2  # 閸旀稐绱堕幇鐔锋珤

        # 缂佹挻鐎?
        base_cost += 150  # 3D閹垫挸宓冩禒?

        # 閻㈠灚绨?
        base_cost += 88  # 閻㈠灚鐫?

        return base_cost

    def calculate_energy_cost(
        self, avg_power_w: float, operating_hours: float
    ) -> float:
        """
        鐠侊紕鐣婚懗浠嬪櫤閹存劖婀?

        閸欏倹鏆?
            avg_power_w: 楠炲啿娼庨崝鐔哄芳 (W)
            operating_hours: 鏉╂劘顢戠亸蹇旀閺?

        鏉╂柨娲?
            閼充粙鍣洪幋鎰拱 (閸?
        """
        kwh = (avg_power_w * operating_hours) / 1000
        return kwh * self.economic_params["electricity_price_kwh"]

    def calculate_maintenance_cost(
        self, operating_hours: float, maintenance_freq: float = 500
    ) -> float:
        """
        鐠侊紕鐣荤紒瀛樺Б閹存劖婀?

        閸欏倹鏆?
            operating_hours: 鏉╂劘顢戠亸蹇旀閺?
            maintenance_freq: 缂佸瓨濮㈡０鎴犲芳 (鐏忓繑妞?濞?

        鏉╂柨娲?
            缂佸瓨濮㈤幋鎰拱 (閸?
        """
        num_maintenances = operating_hours / maintenance_freq

        # 濮ｅ繑顐肩紒瀛樺Б閹存劖婀?= 娴滃搫浼愰弮鍫曟？ 鑴?瀹搞儲妞傜拹鍦芳 + 閺夋劖鏋?
        labor_hours_per_maintenance = 2.0
        materials_per_maintenance = 100.0

        cost_per_maintenance = (
            labor_hours_per_maintenance * self.economic_params["labor_rate_hour"]
            + materials_per_maintenance
        )

        return num_maintenances * cost_per_maintenance

    def calculate_replacement_cost(
        self, operating_hours: float, component_lifetimes: Dict
    ) -> float:
        """
        鐠侊紕鐣婚弴瀛樺床閹存劖婀?

        閸欏倹鏆?
            operating_hours: 鏉╂劘顢戠亸蹇旀閺?
            component_lifetimes: 缂佸嫪娆㈢€靛灝鎳＄€涙鍚€ {name: hours}

        鏉╂柨娲?
            閺囧瓨宕查幋鎰拱 (閸?
        """
        total_cost = 0

        for component, lifetime in component_lifetimes.items():
            if operating_hours > 0:
                num_replacements = operating_hours / lifetime

                # 缂佸嫪娆㈡禒閿嬬壐 (缁犫偓閸?
                if "motor" in component.lower():
                    component_cost = 120
                elif "bearing" in component.lower():
                    component_cost = 30
                elif "sensor" in component.lower():
                    component_cost = 50
                else:
                    component_cost = 100

                # 閺囧瓨宕叉禍鍝勪紣
                labor_cost = 1.0 * self.economic_params["labor_rate_hour"]

                total_cost += num_replacements * (component_cost + labor_cost)

        return total_cost

    def calculate_tco(
        self, operating_hours: float, avg_power_w: float = 200
    ) -> CostBreakdown:
        """
        鐠侊紕鐣婚幀缁樺閺堝鍨氶張?(TCO)

        閸欏倹鏆?
            operating_hours: 妫板嫯顓告潻鎰攽鐏忓繑妞傞弫?
            avg_power_w: 楠炲啿娼庨崝鐔哄芳

        鏉╂柨娲?
            閹存劖婀伴崚鍡毿?
        """
        breakdown = CostBreakdown()

        # 閸掓繂顫愰幋鎰拱
        breakdown.initial_cost = self.calculate_initial_cost()

        # 閼充粙鍣洪幋鎰拱
        breakdown.energy_cost = self.calculate_energy_cost(avg_power_w, operating_hours)

        # 缂佸瓨濮㈤幋鎰拱
        breakdown.maintenance_cost = self.calculate_maintenance_cost(operating_hours)

        # 閺囧瓨宕查幋鎰拱 (閸嬪洩顔曟稉鏄忣洣缂佸嫪娆㈢€靛灝鎳?
        component_lifetimes = {"motor": 5000, "bearing": 10000, "sensor": 8000}
        breakdown.replacement_cost = self.calculate_replacement_cost(
            operating_hours, component_lifetimes
        )

        # 閹舵ɑ妫?
        depreciation_years = self.economic_params["depreciation_years"]
        years = operating_hours / 8760  # 鏉烆剚宕叉稉鍝勫嬀
        breakdown.depreciation = (breakdown.initial_cost / depreciation_years) * years

        return breakdown


class DesignOptimizer:
    """鐠佹崘顓告导妯哄閸?""

    def __init__(self, constraints: Dict = None):
        """
        閸掓繂顫愰崠鏍︾喘閸栨牕娅?

        閸欏倹鏆?
            constraints: 缁撅附娼弶鈥叉
        """
        self.constraints = constraints or {
            "max_cost": 3000,  # 閺堚偓婢堆勫灇閺?
            "min_performance": 0.8,  # 閺堚偓鐏忓繑鈧嗗厴
            "max_weight": 15.0,  # 閺堚偓婢堆囧櫢闁?(kg)
            "target_lifetime": 10000,  # 閻╊喗鐖ｇ€靛灝鎳?(鐏忓繑妞?
        }

    def evaluate_design(self, config: Dict) -> Dict:
        """
        鐠囧嫪鍙婄拋鎹愵吀閺傝顢?

        閸欏倹鏆?
            config: 闁板秶鐤嗛崣鍌涙殶

        鏉╂柨娲?
            鐠囧嫪鍙婄紒鎾寸亯
        """
        cost_model = CostModel(config)

        # 鐠侊紕鐣籘CO
        tco = cost_model.calculate_tco(
            operating_hours=self.constraints["target_lifetime"],
            avg_power_w=config.get("avg_power_w", 200),
        )

        total_cost = (
            tco.initial_cost
            + tco.energy_cost
            + tco.maintenance_cost
            + tco.replacement_cost
        )

        # 鐠囧嫪鍙婇幀褑鍏?(缁犫偓閸?
        motor_power = config.get("motor_power_multiplier", 1.0)
        performance_score = min(1.0, motor_power / 1.5)

        # 鐠囧嫪鍙婇柌宥夊櫤
        base_weight = 12.0  # kg
        weight = base_weight * config.get("mass_multiplier", 1.0)

        # 濡偓閺屻儳瀹抽弶?
        meets_constraints = (
            total_cost <= self.constraints["max_cost"]
            and performance_score >= self.constraints["min_performance"]
            and weight <= self.constraints["max_weight"]
        )

        return {
            "config": config,
            "total_cost": total_cost,
            "cost_breakdown": tco,
            "performance_score": performance_score,
            "weight": weight,
            "cost_per_performance": total_cost / max(performance_score, 0.1),
            "meets_constraints": meets_constraints,
        }

    def optimize(self, num_trials: int = 20) -> List[Dict]:
        """
        娴兼ê瀵茬拋鎹愵吀

        閸欏倹鏆?
            num_trials: 鐏忔繆鐦▎鈩冩殶

        鏉╂柨娲?
            娴兼ê瀵茬紒鎾寸亯閸掓銆?(閹稿鈧傜幆濮ｆ梹甯撴惔?
        """
        results = []

        # 闂呭繑婧€閹兼粎鍌ㄦ稉宥呮倱闁板秶鐤?
        for _ in range(num_trials):
            config = {
                "motor_power_multiplier": np.random.uniform(0.8, 1.5),
                "num_motors": np.random.choice([4, 6, 8]),
                "mass_multiplier": np.random.uniform(0.8, 1.2),
                "avg_power_w": np.random.uniform(150, 300),
            }

            result = self.evaluate_design(config)

            # 閸欘亙绻氶悾娆愬姬鐡掑磭瀹抽弶鐔烘畱閺傝顢?
            if result["meets_constraints"]:
                results.append(result)

        # 閹稿鈧傜幆濮ｆ梹甯撴惔?
        results.sort(key=lambda x: x["cost_per_performance"])

        return results

    def compare_designs(self, designs: List[Dict]) -> str:
        """鐎佃鐦拋鎹愵吀閺傝顢?""
        report = []
        report.append("=" * 70)
        report.append("鐠佹崘顓搁弬瑙勵攳鐎佃鐦?)
        report.append("=" * 70)

        report.append(
            f"\n{'閺傝顢?:<6} {'閹存劖婀?:<10} {'閹嗗厴':<10} {'闁插秹鍣?:<10} {'閹傜幆濮?:<12}"
        )
        report.append("-" * 70)

        for i, result in enumerate(designs, 1):
            report.append(
                f"{i:<6} "
                f"妤納result['total_cost']:<9.0f} "
                f"{result['performance_score']:<10.2f} "
                f"{result['weight']:<10.1f}kg "
                f"{result['cost_per_performance']:<12.0f}"
            )

        return "\n".join(report)


class ROICalculator:
    """閹舵洝绁崶鐐村Г閻滃洩顓哥粻妤€娅?""

    @staticmethod
    def calculate_roi(
        initial_investment: float, annual_savings: float, years: int
    ) -> Dict:
        """
        鐠侊紕鐣籖OI

        閸欏倹鏆?
            initial_investment: 閸掓繂顫愰幎鏇＄カ
            annual_savings: 楠炵濡惇?
            years: 楠炲瓨鏆?

        鏉╂柨娲?
            ROI閸掑棙鐎?
        """
        total_savings = annual_savings * years
        net_profit = total_savings - initial_investment
        roi_percentage = (net_profit / initial_investment) * 100
        payback_years = (
            initial_investment / annual_savings if annual_savings > 0 else float("inf")
        )

        return {
            "initial_investment": initial_investment,
            "annual_savings": annual_savings,
            "years": years,
            "total_savings": total_savings,
            "net_profit": net_profit,
            "roi_percentage": roi_percentage,
            "payback_years": payback_years,
        }


if __name__ == "__main__":
    print("閹存劖婀版导妯哄缁崵绮洪崝鐘烘祰鐎瑰本鍨?)

    # 缁€杞扮伐1: TCO鐠侊紕鐣?
    print("\n" + "=" * 70)
    print("缁€杞扮伐1: 閹粯瀚㈤張澶嬪灇閺堫剝顓哥粻?)
    print("=" * 70)

    config = {"motor_power_multiplier": 1.0, "num_motors": 6}

    cost_model = CostModel(config)
    tco = cost_model.calculate_tco(operating_hours=5000, avg_power_w=200)

    print("\n鏉╂劘顢?000鐏忓繑妞傞惃鍑綜O:")
    print(f"  閸掓繂顫愰幋鎰拱: 妤納tco.initial_cost:.0f}")
    print(f"  閼充粙鍣洪幋鎰拱: 妤納tco.energy_cost:.0f}")
    print(f"  缂佸瓨濮㈤幋鎰拱: 妤納tco.maintenance_cost:.0f}")
    print(f"  閺囧瓨宕查幋鎰拱: 妤納tco.replacement_cost:.0f}")
    print(
        f"  閹槒顓? 妤納tco.initial_cost + tco.energy_cost + tco.maintenance_cost + tco.replacement_cost:.0f}"
    )

    # 缁€杞扮伐2: 鐠佹崘顓告导妯哄
    print("\n" + "=" * 70)
    print("缁€杞扮伐2: 鐠佹崘顓告导妯哄")
    print("=" * 70)

    optimizer = DesignOptimizer(
        {
            "max_cost": 10000,
            "min_performance": 0.7,
            "max_weight": 15.0,
            "target_lifetime": 5000,
        }
    )

    print("\n濮濓絽婀导妯哄... (20娑擃亜鈧瑩鈧鏌熷?")
    results = optimizer.optimize(num_trials=20)

    if results:
        print(f"\n閹垫儳鍩?{len(results)} 娑擃亝寮х搾宕囧閺夌喓娈戦弬瑙勵攳")
        print("\n閸?娑擃亝娓舵导妯绘煙濡?")
        print(optimizer.compare_designs(results[:5]))
    else:
        print("閺堫亝澹橀崚鐗堝姬鐡掑磭瀹抽弶鐔烘畱閺傝顢?)
