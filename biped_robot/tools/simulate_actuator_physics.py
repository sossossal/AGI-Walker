from __future__ import annotations

import argparse
import json
import math
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
ACTUATOR_CONFIG = ROOT / "config" / "actuators.json"
ROBOT_CONFIG = ROOT / "config" / "robot.json"
DEFAULT_REPORT = ROOT / "test_env" / "actuator_physics_report.json"
DEFAULT_TELEMETRY = ROOT / "test_env" / "actuator_telemetry.jsonl"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def target_degrees(joint: dict[str, Any], phase: float) -> float:
    offset = math.radians(float(joint.get("phase_offset_degrees", 0.0)))
    raw = float(joint["target_amplitude_degrees"]) * math.sin(phase + offset)
    return max(0.0, raw) if joint.get("positive_only") else raw


def joint_limits(robot: dict[str, Any]) -> dict[str, tuple[float, float]]:
    return {joint["name"]: tuple(joint["limit"]) for joint in robot["joints"]}


def simulate(config: dict[str, Any], robot: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    defaults = config["defaults"]
    acceptance = config["acceptance"]
    limits = joint_limits(robot)
    dt = 1.0 / int(config["control_hz"])
    steps = int(float(config["duration_seconds"]) / dt)
    gait_cycle = float(robot["gait"]["cycle_seconds"])
    state = {
        joint["name"]: {
            "position_rad": 0.0,
            "velocity_rad_s": 0.0,
            "temperature_c": float(defaults["ambient_c"]),
            "energy_j": 0.0,
            "saturation_count": 0,
            "max_error_deg": 0.0,
            "max_current_amp": 0.0,
            "max_torque_nm": 0.0,
            "max_temperature_c": float(defaults["ambient_c"]),
        }
        for joint in config["joints"]
    }
    telemetry: list[dict[str, Any]] = []

    for step in range(steps):
        t = step * dt
        phase = 2.0 * math.pi * t / gait_cycle
        for joint in config["joints"]:
            name = joint["name"]
            current = state[name]
            target_deg = target_degrees(joint, phase)
            low, high = limits[name]
            target_deg = clamp(target_deg, low, high)
            target_rad = math.radians(target_deg)
            position = float(current["position_rad"])
            velocity = float(current["velocity_rad_s"])
            error = target_rad - position
            gravity = float(joint["gravity_load_nm"]) * math.sin(position)
            commanded_torque = (
                float(defaults["kp"]) * error
                - float(defaults["kd"]) * velocity
                + gravity
            )
            torque_limit = float(defaults["torque_limit_nm"])
            applied_torque = clamp(commanded_torque, -torque_limit, torque_limit)
            saturated = abs(applied_torque - commanded_torque) > 1e-9
            acceleration = (
                applied_torque
                - float(defaults["damping_nms_per_rad"]) * velocity
                - gravity
            ) / float(defaults["inertia_kg_m2"])
            velocity += acceleration * dt
            velocity_limit = math.radians(float(defaults["velocity_limit_degrees_per_second"]))
            velocity = clamp(velocity, -velocity_limit, velocity_limit)
            position += velocity * dt
            position = math.radians(clamp(math.degrees(position), low, high))
            motor_current = abs(applied_torque) / float(defaults["motor_constant_nm_per_amp"])
            copper_loss_w = motor_current * motor_current * float(defaults["winding_resistance_ohm"])
            mechanical_power_w = abs(applied_torque * velocity)
            thermal_out_w = (float(current["temperature_c"]) - float(defaults["ambient_c"])) / float(defaults["thermal_resistance_c_per_w"])
            temperature = float(current["temperature_c"]) + (copper_loss_w - thermal_out_w) * dt / float(defaults["thermal_capacity_j_per_c"])
            energy_j = float(current["energy_j"]) + (copper_loss_w + mechanical_power_w) * dt
            error_deg = abs(math.degrees(error))

            current.update(
                {
                    "position_rad": position,
                    "velocity_rad_s": velocity,
                    "temperature_c": temperature,
                    "energy_j": energy_j,
                    "saturation_count": int(current["saturation_count"]) + int(saturated),
                    "max_error_deg": max(float(current["max_error_deg"]), error_deg),
                    "max_current_amp": max(float(current["max_current_amp"]), motor_current),
                    "max_torque_nm": max(float(current["max_torque_nm"]), abs(applied_torque)),
                    "max_temperature_c": max(float(current["max_temperature_c"]), temperature),
                }
            )
            telemetry.append(
                {
                    "time_s": round(t, 5),
                    "joint": name,
                    "target_degrees": round(target_deg, 5),
                    "position_degrees": round(math.degrees(position), 5),
                    "velocity_degrees_per_second": round(math.degrees(velocity), 5),
                    "torque_nm": round(applied_torque, 5),
                    "current_amp": round(motor_current, 5),
                    "temperature_c": round(temperature, 5),
                    "power_w": round(copper_loss_w + mechanical_power_w, 5),
                    "energy_j": round(energy_j, 5),
                    "saturated": saturated,
                }
            )

    total_energy_wh = sum(float(values["energy_j"]) for values in state.values()) / 3600.0
    max_error = max(float(values["max_error_deg"]) for values in state.values())
    max_temp = max(float(values["max_temperature_c"]) for values in state.values())
    max_current = max(float(values["max_current_amp"]) for values in state.values())
    saturation_ratio = sum(int(values["saturation_count"]) for values in state.values()) / float(steps * len(state))
    checks = {
        "tracking_error_within_acceptance": max_error <= float(acceptance["max_tracking_error_degrees"]),
        "temperature_within_acceptance": max_temp <= float(acceptance["max_temperature_c"]),
        "current_within_acceptance": max_current <= float(acceptance["max_current_amp"]),
        "saturation_ratio_within_acceptance": saturation_ratio <= float(acceptance["max_saturation_ratio"]),
        "energy_within_acceptance": total_energy_wh <= float(acceptance["max_energy_wh"]),
    }
    report = {
        "schema_version": "biped-actuator-physics-report.v1",
        "created_utc": datetime.now(UTC).isoformat(),
        "status": "passed" if all(checks.values()) else "failed",
        "samples_per_joint": steps,
        "joint_count": len(state),
        "control_hz": config["control_hz"],
        "metrics": {
            "max_tracking_error_degrees": round(max_error, 5),
            "max_temperature_c": round(max_temp, 5),
            "max_current_amp": round(max_current, 5),
            "saturation_ratio": round(saturation_ratio, 6),
            "total_energy_wh": round(total_energy_wh, 6),
        },
        "checks": checks,
        "joint_summaries": {
            name: {
                "max_tracking_error_degrees": round(float(values["max_error_deg"]), 5),
                "max_current_amp": round(float(values["max_current_amp"]), 5),
                "max_torque_nm": round(float(values["max_torque_nm"]), 5),
                "max_temperature_c": round(float(values["max_temperature_c"]), 5),
                "energy_wh": round(float(values["energy_j"]) / 3600.0, 6),
                "saturation_count": int(values["saturation_count"]),
            }
            for name, values in state.items()
        },
        "limitations": [
            "Deterministic actuator-level approximation with PD control, inertia, damping, gravity load, current, thermal, and saturation.",
            "Not a replacement for hardware-in-the-loop or full rigid-body physics validation."
        ],
    }
    return report, telemetry


def write_jsonl(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True))
            handle.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run deterministic actuator physics simulation and telemetry retention.")
    parser.add_argument("--output", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--telemetry-output", type=Path, default=DEFAULT_TELEMETRY)
    args = parser.parse_args()

    report, telemetry = simulate(load_json(ACTUATOR_CONFIG), load_json(ROBOT_CONFIG))
    write_json(args.output, report)
    write_jsonl(args.telemetry_output, telemetry)
    print(json.dumps({"status": report["status"], "output": str(args.output), "telemetry_output": str(args.telemetry_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
