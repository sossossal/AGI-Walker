from agi_walker.core.api.parts import (
    CostBreakdown,
    CostModel,
    DesignOptimizer,
    ROICalculator,
)


def test_cost_model_calculates_positive_tco() -> None:
    model = CostModel(
        {
            "num_motors": 6,
            "motor_power_multiplier": 1.1,
            "sensor_count": 4,
        }
    )

    breakdown = model.calculate_tco(operating_hours=5000, avg_power_w=220)

    assert isinstance(breakdown, CostBreakdown)
    assert breakdown.initial_cost > 0
    assert breakdown.energy_cost > 0
    assert breakdown.maintenance_cost > 0
    assert breakdown.replacement_cost > 0
    assert breakdown.total_cost >= breakdown.initial_cost


def test_design_optimizer_returns_sorted_results() -> None:
    optimizer = DesignOptimizer(
        {
            "max_cost": 9000,
            "min_performance": 0.5,
            "max_weight": 20.0,
            "target_lifetime": 4000,
        }
    )

    results = optimizer.optimize(num_trials=20, random_seed=7)

    assert results
    assert all(result["meets_constraints"] for result in results)

    costs = [result["cost_per_performance"] for result in results]
    assert costs == sorted(costs)


def test_roi_calculator_reports_expected_fields() -> None:
    roi = ROICalculator.calculate_roi(
        initial_investment=1000.0,
        annual_savings=400.0,
        years=4,
    )

    assert roi["total_savings"] == 1600.0
    assert roi["net_profit"] == 600.0
    assert roi["payback_years"] == 2.5
