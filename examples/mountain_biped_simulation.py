"""Local mountain-terrain run simulation for a humanoid biped."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from agi_walker.core.api.robot_schema import (  # noqa: E402
    build_mechanical_topology_summary,
    validate_godot_robot_config,
)

REPORT_VERSION = "dynamic_mountain_biped_simulation.v1"
TRACE_VERSION = "dynamic_mountain_biped_trace.v1"
DEFAULT_ROBOT = REPO_ROOT / "configs" / "mountain_humanoid_biped.json"
DEFAULT_OUTPUT = (
    REPO_ROOT / "test_env" / "mountain_biped" / "mountain_biped_simulation_report.json"
)


@dataclass(frozen=True)
class TerrainConfig:
    seed: int = 42
    length_m: float = 12.0
    width_m: float = 3.0
    resolution_m: float = 0.2
    ridge_height_m: float = 0.55
    roughness_m: float = 0.08


@dataclass(frozen=True)
class RunConfig:
    steps: int = 420
    dt_s: float = 0.05
    nominal_speed_mps: float = 0.72
    step_frequency_hz: float = 1.55
    stance_width_m: float = 0.22


def load_robot_config(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    errors = validate_godot_robot_config(payload)
    if errors:
        joined = "; ".join(errors)
        raise ValueError(f"invalid robot config {path}: {joined}")
    return payload


def terrain_height(x: float, y: float, config: TerrainConfig) -> float:
    ridge = config.ridge_height_m * (x / config.length_m)
    wave = math.sin(1.7 * x + config.seed * 0.01) * config.roughness_m
    cross = math.cos(2.3 * y + 0.4 * x) * config.roughness_m * 0.65
    rocks = math.sin(5.1 * x + 3.7 * y + config.seed) * config.roughness_m * 0.35
    return round(ridge + wave + cross + rocks, 6)


def terrain_slope(x: float, y: float, config: TerrainConfig) -> tuple[float, float]:
    delta = config.resolution_m
    dz_dx = (
        terrain_height(x + delta, y, config) - terrain_height(x - delta, y, config)
    ) / (2.0 * delta)
    dz_dy = (
        terrain_height(x, y + delta, config) - terrain_height(x, y - delta, config)
    ) / (2.0 * delta)
    return dz_dx, dz_dy


def simulate_mountain_run(
    robot: dict[str, Any],
    terrain: TerrainConfig,
    run: RunConfig,
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    x = 0.0
    y = 0.0
    energy_j = 0.0
    trace: list[dict[str, Any]] = []

    for step in range(run.steps):
        time_s = step * run.dt_s
        phase = 2.0 * math.pi * run.step_frequency_hz * time_s
        height = terrain_height(x, y, terrain)
        slope_x, slope_y = terrain_slope(x, y, terrain)
        gait = _gait_state(phase, slope_x)
        stability_margin = _stability_margin(run.stance_width_m, slope_y, phase)
        speed = _forward_speed(run.nominal_speed_mps, slope_x, stability_margin)
        x = min(terrain.length_m, x + speed * run.dt_s)
        energy_j += _step_energy(gait, slope_x, run.dt_s)
        trace.append(
            _trace_step(step, time_s, x, y, height, slope_x, gait, stability_margin)
        )
        if stability_margin < -0.04:
            break
        y = 0.03 * math.sin(phase * 0.5)

    report = _build_report(robot, terrain, run, trace, energy_j)
    return report, trace


def _gait_state(phase: float, slope_x: float) -> dict[str, float]:
    stride = 0.42 + max(0.0, slope_x) * 0.08
    hip = math.sin(phase) * stride
    knee = -0.55 * max(0.0, math.sin(phase)) - max(0.0, slope_x) * 0.18
    ankle = -0.28 * math.sin(phase) + slope_x * 0.12
    arm = -0.45 * math.sin(phase)
    return {
        "left_hip_pitch": round(hip, 5),
        "right_hip_pitch": round(-hip, 5),
        "left_knee_pitch": round(knee, 5),
        "right_knee_pitch": round(-0.55 * max(0.0, -math.sin(phase)), 5),
        "left_ankle_pitch": round(ankle, 5),
        "right_ankle_pitch": round(-ankle, 5),
        "left_shoulder_swing": round(arm, 5),
        "right_shoulder_swing": round(-arm, 5),
    }


def _stability_margin(stance_width: float, slope_y: float, phase: float) -> float:
    lateral_com = 0.035 * math.sin(phase) + slope_y * 0.08
    active_support = stance_width * (0.5 if math.sin(phase) > 0.0 else 0.62)
    return round(active_support - abs(lateral_com), 5)


def _forward_speed(
    nominal_speed: float, slope_x: float, stability_margin: float
) -> float:
    grade_penalty = max(0.35, 1.0 - max(0.0, slope_x) * 1.4)
    stability_penalty = 0.55 if stability_margin < 0.02 else 1.0
    return max(0.0, nominal_speed * grade_penalty * stability_penalty)


def _step_energy(gait: dict[str, float], slope_x: float, dt_s: float) -> float:
    joint_work = sum(abs(value) for value in gait.values()) * 9.0
    uphill_work = max(0.0, slope_x) * 22.0
    return (joint_work + uphill_work) * dt_s


def _trace_step(
    step: int,
    time_s: float,
    x: float,
    y: float,
    height: float,
    slope_x: float,
    gait: dict[str, float],
    stability_margin: float,
) -> dict[str, Any]:
    return {
        "step": step,
        "time_s": round(time_s, 3),
        "base_position_m": [round(x, 4), round(y, 4), round(height + 1.02, 4)],
        "terrain_height_m": height,
        "terrain_pitch_deg": round(math.degrees(math.atan(slope_x)), 3),
        "stability_margin_m": stability_margin,
        "contacts": {
            "left_foot": math.sin(time_s * math.tau * 1.55) <= 0.08,
            "right_foot": math.sin(time_s * math.tau * 1.55) >= -0.08,
        },
        "joint_targets_rad": gait,
    }


def _build_report(
    robot: dict[str, Any],
    terrain: TerrainConfig,
    run: RunConfig,
    trace: list[dict[str, Any]],
    energy_j: float,
) -> dict[str, Any]:
    distances = [item["base_position_m"][0] for item in trace]
    pitches = [abs(item["terrain_pitch_deg"]) for item in trace]
    margins = [item["stability_margin_m"] for item in trace]
    completed = bool(trace) and distances[-1] >= terrain.length_m * 0.75
    return {
        "report_version": REPORT_VERSION,
        "robot": {
            "name": robot["name"],
            "schema_version": robot.get("schema_version"),
            "parts_count": len(robot["parts"]),
            "connections_count": len(robot["connections"]),
            "topology": build_mechanical_topology_summary(robot),
        },
        "environment": {
            "type": "mountain",
            "seed": terrain.seed,
            "length_m": terrain.length_m,
            "width_m": terrain.width_m,
            "resolution_m": terrain.resolution_m,
            "ridge_height_m": terrain.ridge_height_m,
            "roughness_m": terrain.roughness_m,
        },
        "run_status": {
            "status": "completed" if completed else "incomplete",
            "steps_recorded": len(trace),
            "fall_detected": any(item["stability_margin_m"] < -0.04 for item in trace),
            "distance_m": round(distances[-1] if distances else 0.0, 4),
            "average_speed_mps": (
                round((distances[-1] / (len(trace) * run.dt_s)), 4) if trace else 0.0
            ),
            "max_abs_pitch_deg": round(max(pitches) if pitches else 0.0, 3),
            "min_stability_margin_m": round(min(margins) if margins else 0.0, 5),
            "energy_estimate_j": round(energy_j, 3),
        },
        "trace_preview": trace[:5],
        "residual_risks": [
            "Deterministic local model; not a live Godot physics run.",
            "Terrain is analytic heightfield evidence, not a rendered Godot scene.",
        ],
    }


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", type=Path, default=DEFAULT_ROBOT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--trace-output", type=Path)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--steps", type=int, default=420)
    parser.add_argument("--length-m", type=float, default=12.0)
    parser.add_argument("--ridge-height-m", type=float, default=0.55)
    parser.add_argument("--roughness-m", type=float, default=0.08)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    terrain = TerrainConfig(
        seed=args.seed,
        length_m=args.length_m,
        ridge_height_m=args.ridge_height_m,
        roughness_m=args.roughness_m,
    )
    report, trace = simulate_mountain_run(
        load_robot_config(args.robot_config),
        terrain,
        RunConfig(steps=args.steps),
    )
    if args.trace_output:
        write_json(args.trace_output, {"trace_version": TRACE_VERSION, "steps": trace})
        report["trace_artifact"] = str(args.trace_output)
    write_json(args.output, report)
    status = report["run_status"]
    print(
        "mountain_biped_simulation "
        f"status={status['status']} distance_m={status['distance_m']} "
        f"avg_speed_mps={status['average_speed_mps']} output={args.output}"
    )
    return 0 if status["status"] == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
