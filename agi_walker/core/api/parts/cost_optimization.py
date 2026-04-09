from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass
class CostBreakdown:
    """Lightweight total-cost-of-ownership breakdown for a robot configuration."""

    initial_cost: float = 0.0
    energy_cost: float = 0.0
    maintenance_cost: float = 0.0
    replacement_cost: float = 0.0
    labor_cost: float = 0.0
    depreciation: float = 0.0

    @property
    def total_cost(self) -> float:
        return (
            self.initial_cost
            + self.energy_cost
            + self.maintenance_cost
            + self.replacement_cost
            + self.labor_cost
        )

    def to_dict(self) -> Dict[str, float]:
        return {
            "initial_cost": self.initial_cost,
            "energy_cost": self.energy_cost,
            "maintenance_cost": self.maintenance_cost,
            "replacement_cost": self.replacement_cost,
            "labor_cost": self.labor_cost,
            "depreciation": self.depreciation,
            "total_cost": self.total_cost,
        }


class CostModel:
    """Simple compatibility cost model used by tests and demos."""

    def __init__(
        self,
        parts_config: Dict[str, Any],
        economic_params: Optional[Dict[str, float]] = None,
    ):
        self.parts_config = dict(parts_config)
        self.economic_params = {
            "electricity_price_kwh": 1.0,
            "labor_rate_hour": 200.0,
            "interest_rate": 0.05,
            "depreciation_years": 5.0,
        }
        if economic_params:
            self.economic_params.update(economic_params)

    def calculate_initial_cost(self) -> float:
        num_motors = int(self.parts_config.get("num_motors", 6))
        motor_power_multiplier = float(
            self.parts_config.get("motor_power_multiplier", 1.0)
        )
        sensor_count = int(self.parts_config.get("sensor_count", 3))
        frame_cost = float(self.parts_config.get("frame_cost", 150.0))
        controller_cost = float(self.parts_config.get("controller_cost", 88.0))
        battery_cost = float(self.parts_config.get("battery_cost", 55.0))
        actuator_unit_cost = float(
            self.parts_config.get("motor_unit_cost", 120.0 * motor_power_multiplier)
        )
        sensor_unit_cost = float(self.parts_config.get("sensor_unit_cost", 25.0))

        return (
            num_motors * actuator_unit_cost
            + sensor_count * sensor_unit_cost
            + frame_cost
            + controller_cost
            + battery_cost
        )

    def calculate_energy_cost(
        self,
        avg_power_w: float,
        operating_hours: float,
    ) -> float:
        kwh = (float(avg_power_w) * float(operating_hours)) / 1000.0
        return max(kwh, 0.0) * self.economic_params["electricity_price_kwh"]

    def calculate_maintenance_cost(
        self,
        operating_hours: float,
        maintenance_freq: float = 500.0,
    ) -> float:
        if maintenance_freq <= 0:
            raise ValueError("maintenance_freq must be positive")

        num_maintenances = max(float(operating_hours), 0.0) / maintenance_freq
        labor_hours_per_maintenance = float(
            self.parts_config.get("maintenance_labor_hours", 2.0)
        )
        materials_per_maintenance = float(
            self.parts_config.get("maintenance_materials_cost", 100.0)
        )
        cost_per_maintenance = (
            labor_hours_per_maintenance * self.economic_params["labor_rate_hour"]
            + materials_per_maintenance
        )
        return num_maintenances * cost_per_maintenance

    def calculate_replacement_cost(
        self,
        operating_hours: float,
        component_lifetimes: Dict[str, float],
    ) -> float:
        total_cost = 0.0
        for component, lifetime in component_lifetimes.items():
            if lifetime <= 0:
                continue

            num_replacements = max(float(operating_hours), 0.0) / float(lifetime)
            component_name = component.lower()
            if "motor" in component_name:
                component_cost = 120.0
            elif "bearing" in component_name:
                component_cost = 30.0
            elif "sensor" in component_name:
                component_cost = 50.0
            else:
                component_cost = 100.0

            labor_cost = 1.0 * self.economic_params["labor_rate_hour"]
            total_cost += num_replacements * (component_cost + labor_cost)

        return total_cost

    def calculate_tco(
        self,
        operating_hours: float,
        avg_power_w: float = 200.0,
    ) -> CostBreakdown:
        breakdown = CostBreakdown()
        breakdown.initial_cost = self.calculate_initial_cost()
        breakdown.energy_cost = self.calculate_energy_cost(avg_power_w, operating_hours)
        breakdown.maintenance_cost = self.calculate_maintenance_cost(operating_hours)

        component_lifetimes = {
            "motor": 5000.0,
            "bearing": 10000.0,
            "sensor": 8000.0,
        }
        breakdown.replacement_cost = self.calculate_replacement_cost(
            operating_hours,
            component_lifetimes,
        )

        years = max(float(operating_hours), 0.0) / 8760.0
        breakdown.depreciation = (
            breakdown.initial_cost / self.economic_params["depreciation_years"]
        ) * years
        return breakdown


class DesignOptimizer:
    """Brute-force search helper for low-fidelity design comparisons."""

    def __init__(self, constraints: Optional[Dict[str, float]] = None):
        self.constraints = {
            "max_cost": 3000.0,
            "min_performance": 0.8,
            "max_weight": 15.0,
            "target_lifetime": 10000.0,
        }
        if constraints:
            self.constraints.update(constraints)

    def evaluate_design(self, config: Dict[str, Any]) -> Dict[str, Any]:
        cost_model = CostModel(config)
        tco = cost_model.calculate_tco(
            operating_hours=self.constraints["target_lifetime"],
            avg_power_w=float(config.get("avg_power_w", 200.0)),
        )
        total_cost = tco.total_cost

        motor_power = float(config.get("motor_power_multiplier", 1.0))
        performance_score = min(1.0, motor_power / 1.5)
        base_weight = 12.0
        weight = base_weight * float(config.get("mass_multiplier", 1.0))
        meets_constraints = (
            total_cost <= self.constraints["max_cost"]
            and performance_score >= self.constraints["min_performance"]
            and weight <= self.constraints["max_weight"]
        )

        return {
            "config": dict(config),
            "total_cost": total_cost,
            "cost_breakdown": tco,
            "performance_score": performance_score,
            "weight": weight,
            "cost_per_performance": total_cost / max(performance_score, 0.1),
            "meets_constraints": meets_constraints,
        }

    def optimize(
        self,
        num_trials: int = 20,
        random_seed: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        rng = np.random.default_rng(random_seed)
        results: List[Dict[str, Any]] = []

        for _ in range(num_trials):
            config = {
                "motor_power_multiplier": float(rng.uniform(0.8, 1.5)),
                "num_motors": int(rng.choice([4, 6, 8])),
                "mass_multiplier": float(rng.uniform(0.8, 1.2)),
                "avg_power_w": float(rng.uniform(150.0, 300.0)),
            }
            result = self.evaluate_design(config)
            if result["meets_constraints"]:
                results.append(result)

        results.sort(key=lambda item: item["cost_per_performance"])
        return results

    def compare_designs(self, designs: List[Dict[str, Any]]) -> str:
        lines = [
            "=" * 72,
            "Design comparison",
            "=" * 72,
            "",
            f"{'Rank':<6} {'Cost':<12} {'Perf':<10} {'Weight':<12} {'Cost/Perf':<12}",
            "-" * 72,
        ]
        for index, result in enumerate(designs, start=1):
            lines.append(
                f"{index:<6} "
                f"${result['total_cost']:<11.0f} "
                f"{result['performance_score']:<10.2f} "
                f"{result['weight']:<11.1f} "
                f"{result['cost_per_performance']:<12.0f}"
            )
        return "\n".join(lines)


class ROICalculator:
    """Basic return-on-investment helper."""

    @staticmethod
    def calculate_roi(
        initial_investment: float,
        annual_savings: float,
        years: int,
    ) -> Dict[str, float]:
        total_savings = annual_savings * years
        net_profit = total_savings - initial_investment
        roi_percentage = (
            (net_profit / initial_investment) * 100.0
            if initial_investment
            else float("inf")
        )
        payback_years = (
            initial_investment / annual_savings
            if annual_savings > 0
            else float("inf")
        )
        return {
            "initial_investment": float(initial_investment),
            "annual_savings": float(annual_savings),
            "years": float(years),
            "total_savings": float(total_savings),
            "net_profit": float(net_profit),
            "roi_percentage": float(roi_percentage),
            "payback_years": float(payback_years),
        }


__all__ = [
    "CostBreakdown",
    "CostModel",
    "DesignOptimizer",
    "ROICalculator",
]
