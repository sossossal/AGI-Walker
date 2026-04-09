from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable

import numpy as np


@dataclass
class ImpactResult:
    param: str
    old_value: float | str
    new_value: float | str
    impact: Dict[str, str]


class CustomPart:
    """Base class for configurable hardware parts."""

    def __init__(self, part_type: str, base_params: Dict):
        self.part_type = part_type
        self.params = dict(base_params)
        self.performance_metrics: Dict[str, float] = {}

    def update_param(self, param_name: str, value):
        if param_name not in self.params:
            raise ValueError(f"Unknown parameter: {param_name}")

        old_value = self.params[param_name]
        self.params[param_name] = value
        self._recalculate_performance()
        impact = self._calculate_impact(param_name, old_value, value)
        return ImpactResult(param_name, old_value, value, impact).__dict__

    def _recalculate_performance(self) -> None:
        raise NotImplementedError

    def _calculate_impact(
        self, param_name: str, old_value, new_value
    ) -> Dict[str, str]:
        return {}

    def get_performance_report(self) -> str:
        lines = [f"Part type: {self.part_type}", "", "Parameters:"]
        for key, value in self.params.items():
            lines.append(f"  {key}: {value}")
        lines.append("")
        lines.append("Performance metrics:")
        for key, value in self.performance_metrics.items():
            if isinstance(value, float):
                lines.append(f"  {key}: {value:.3f}")
            else:
                lines.append(f"  {key}: {value}")
        return "\n".join(lines)


class CustomMotor(CustomPart):
    """Configurable motor model with lightweight engineering heuristics."""

    def __init__(self, base_params: Dict | None = None):
        defaults = {
            "power": 500.0,
            "voltage": 24.0,
            "gear_ratio": 50.0,
            "efficiency": 0.85,
            "weight": 0.8,
            "max_current": 20.0,
        }
        if base_params:
            defaults.update(base_params)
        super().__init__("custom_motor", defaults)
        self._recalculate_performance()

    def _recalculate_performance(self) -> None:
        power_w = float(self.params["power"])
        voltage = max(float(self.params["voltage"]), 1e-6)
        gear_ratio = max(float(self.params["gear_ratio"]), 1e-6)
        efficiency = float(np.clip(self.params["efficiency"], 1e-6, 1.0))
        weight = max(float(self.params["weight"]), 1e-6)

        base_rpm = 3000.0
        output_speed_rpm = base_rpm / gear_ratio
        output_torque_nm = (power_w * 60.0 * efficiency) / (
            2.0 * np.pi * max(output_speed_rpm, 1e-6)
        )
        current_draw_a = power_w / voltage
        heat_generation_w = power_w * (1.0 - efficiency)
        estimated_cost_usd = 50.0 + power_w * 0.15 + gear_ratio * 0.5 + weight * 10.0

        self.performance_metrics = {
            "output_torque_nm": float(output_torque_nm),
            "output_speed_rpm": float(output_speed_rpm),
            "current_draw_a": float(current_draw_a),
            "heat_generation_w": float(heat_generation_w),
            "torque_to_weight_ratio": float(output_torque_nm / weight),
            "estimated_cost_usd": float(estimated_cost_usd),
        }

    def _calculate_impact(
        self, param_name: str, old_value, new_value
    ) -> Dict[str, str]:
        impact: Dict[str, str] = {}
        if param_name == "gear_ratio":
            ratio_change = (float(new_value) - float(old_value)) / max(
                float(old_value), 1e-6
            )
            impact["torque_change"] = f"{ratio_change * 100:+.1f}%"
            impact["speed_change"] = f"{-ratio_change * 100:+.1f}%"
            impact["cost_change"] = (
                f"+${abs(float(new_value) - float(old_value)) * 0.5:.2f}"
            )
        elif param_name == "power":
            power_change = (float(new_value) - float(old_value)) / max(
                float(old_value), 1e-6
            )
            impact["torque_change"] = f"{power_change * 100:+.1f}%"
            impact["heat_change"] = f"{power_change * 100:+.1f}%"
            impact["cost_change"] = (
                f"+${abs(float(new_value) - float(old_value)) * 0.15:.2f}"
            )
        elif param_name == "efficiency":
            efficiency_change = float(new_value) - float(old_value)
            impact["torque_change"] = f"{efficiency_change * 100:+.1f}%"
            impact["heat_change"] = f"{-efficiency_change * 100:+.1f}%"
        return impact


class CustomJoint(CustomPart):
    """Configurable reducer/joint model."""

    def __init__(self, base_params: Dict | None = None):
        defaults = {
            "type": "harmonic_drive",
            "reduction_ratio": 100.0,
            "max_torque": 50.0,
            "backlash": 0.05,
            "efficiency": 0.9,
            "weight": 0.3,
            "stiffness": 5000.0,
            "max_speed": 100.0,
        }
        if base_params:
            defaults.update(base_params)
        super().__init__("custom_joint", defaults)
        self._recalculate_performance()

    def _recalculate_performance(self) -> None:
        ratio = max(float(self.params["reduction_ratio"]), 1e-6)
        max_torque = float(self.params["max_torque"])
        backlash = max(float(self.params["backlash"]), 1e-6)
        efficiency = float(np.clip(self.params["efficiency"], 1e-6, 1.0))
        weight = max(float(self.params["weight"]), 1e-6)
        stiffness = max(float(self.params["stiffness"]), 1e-6)
        max_speed = float(self.params["max_speed"])

        positioning_accuracy_deg = backlash / ratio
        torque_density_nm_per_kg = max_torque / weight
        resonance_frequency_hz = np.sqrt(stiffness / weight) / (2.0 * np.pi)
        output_power_w = max_torque * (max_speed * 2.0 * np.pi / 60.0)
        power_loss_w = output_power_w * (1.0 - efficiency) / efficiency
        estimated_cost_usd = (
            80.0 + ratio * 0.3 + max_torque * 1.5 + (1.0 / backlash) * 10.0
        )

        self.performance_metrics = {
            "positioning_accuracy_deg": float(positioning_accuracy_deg),
            "torque_density_nm_per_kg": float(torque_density_nm_per_kg),
            "resonance_frequency_hz": float(resonance_frequency_hz),
            "power_loss_w": float(power_loss_w),
            "estimated_cost_usd": float(estimated_cost_usd),
            "control_bandwidth_hz": float(resonance_frequency_hz * 0.1),
        }

    def _calculate_impact(
        self, param_name: str, old_value, new_value
    ) -> Dict[str, str]:
        impact: Dict[str, str] = {}
        if param_name == "reduction_ratio":
            ratio_change = (float(new_value) - float(old_value)) / max(
                float(old_value), 1e-6
            )
            impact["output_torque"] = f"{ratio_change * 100:+.1f}%"
            impact["positioning_accuracy"] = f"{ratio_change * 100:+.1f}%"
            impact["cost_change"] = (
                f"+${abs(float(new_value) - float(old_value)) * 0.3:.2f}"
            )
        elif param_name == "backlash":
            improvement = (float(old_value) - float(new_value)) / max(
                float(old_value), 1e-6
            )
            impact["positioning_accuracy"] = f"{improvement * 100:+.1f}%"
            impact["cost_change"] = (
                f"+${abs((1.0 / float(new_value)) - (1.0 / float(old_value))) * 10.0:.2f}"
            )
        elif param_name == "stiffness":
            stiffness_change = (float(new_value) - float(old_value)) / max(
                float(old_value), 1e-6
            )
            impact["dynamic_response"] = f"{stiffness_change * 50:+.1f}%"
            impact["control_bandwidth"] = f"{stiffness_change * 50:+.1f}%"
        return impact


class CustomSensor(CustomPart):
    """Configurable sensor model kept for compatibility."""

    def __init__(self, sensor_type: str, base_params: Dict | None = None):
        defaults = {
            "resolution": 12,
            "sample_rate": 1000,
            "noise_level": 0.01,
            "power_consumption": 0.1,
            "weight": 0.02,
            "interface": "SPI",
        }
        if base_params:
            defaults.update(base_params)
        super().__init__(f"custom_sensor_{sensor_type}", defaults)
        self.sensor_type = sensor_type
        self._recalculate_performance()

    def _recalculate_performance(self) -> None:
        resolution_bits = float(self.params["resolution"])
        sample_rate = float(self.params["sample_rate"])
        noise_level = max(float(self.params["noise_level"]), 1e-6)

        effective_bits = resolution_bits - np.log2(1.0 + noise_level * 100.0)
        snr_db = 6.02 * effective_bits + 1.76
        data_rate_mbps = (resolution_bits * sample_rate) / 1_000_000.0
        estimated_cost_usd = 15.0 + (2.0**resolution_bits) * 0.001 + sample_rate * 0.01

        self.performance_metrics = {
            "effective_bits": float(effective_bits),
            "snr_db": float(snr_db),
            "data_rate_mbps": float(data_rate_mbps),
            "estimated_cost_usd": float(estimated_cost_usd),
        }


class PartCustomizer:
    """Factory and comparison utilities for custom parts."""

    def __init__(self):
        self.parts: Dict[str, CustomPart] = {}

    def create_motor(self, name: str, params: Dict | None = None) -> CustomMotor:
        motor = CustomMotor(params)
        self.parts[name] = motor
        return motor

    def create_joint(self, name: str, params: Dict | None = None) -> CustomJoint:
        joint = CustomJoint(params)
        self.parts[name] = joint
        return joint

    def create_sensor(
        self, name: str, sensor_type: str, params: Dict | None = None
    ) -> CustomSensor:
        sensor = CustomSensor(sensor_type, params)
        self.parts[name] = sensor
        return sensor

    def compare_configurations(
        self, part_name: str, configs: Iterable[Dict]
    ) -> list[Dict]:
        if part_name not in self.parts:
            raise ValueError(f"Unknown part: {part_name}")

        prototype = self.parts[part_name]
        results: list[Dict] = []
        for index, config in enumerate(configs, start=1):
            if isinstance(prototype, CustomMotor):
                candidate = CustomMotor(config)
            elif isinstance(prototype, CustomJoint):
                candidate = CustomJoint(config)
            elif isinstance(prototype, CustomSensor):
                candidate = CustomSensor(prototype.sensor_type, config)
            else:
                continue

            results.append(
                {
                    "config_id": index,
                    "params": dict(config),
                    "metrics": dict(candidate.performance_metrics),
                }
            )
        return results
