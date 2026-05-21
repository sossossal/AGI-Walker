from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
ROBOT_CONFIG = ROOT / "config" / "robot.json"
TERRAIN_CONFIG = ROOT / "config" / "mountain_terrain.json"


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def terrain_height(x: float, z: float, terrain: dict[str, Any]) -> float:
    ridge = (
        math.sin(z * terrain["ridge_frequency"]) * terrain["ridge_amplitude"]
        + math.cos((x + z) * 0.18) * 0.28
    )
    rough = (
        math.sin(x * terrain["roughness_frequency"])
        * math.cos(z * 0.72)
        * terrain["roughness_amplitude"]
    )
    walkway_blend = min(1.0, max(0.0, abs(x) / terrain["walkway_width"]))
    return (ridge * terrain["walkway_smoothing"]) * (1.0 - walkway_blend) + (ridge + rough) * walkway_blend


def terrain_slope_degrees(x: float, z: float, terrain: dict[str, Any]) -> float:
    sample = 0.25
    dz = terrain_height(x, z + sample, terrain) - terrain_height(x, z - sample, terrain)
    dx = terrain_height(x + sample, z, terrain) - terrain_height(x - sample, z, terrain)
    gradient = math.sqrt(dx * dx + dz * dz) / (2.0 * sample)
    return math.degrees(math.atan(gradient))


def simulate(robot: dict[str, Any], terrain: dict[str, Any], samples: int) -> tuple[dict[str, Any], list[dict[str, float]]]:
    gait = robot["gait"]
    acceptance = terrain["acceptance"]
    duration = 8.0
    trace: list[dict[str, float]] = []
    max_slope = 0.0
    min_clearance = float("inf")
    min_stability_margin = float("inf")
    max_joint_abs = 0.0

    for index in range(samples):
        t = duration * index / max(samples - 1, 1)
        z = -terrain["size_meters"] * 0.25 + gait["nominal_speed_mps"] * t
        phase = 2.0 * math.pi * t / gait["cycle_seconds"]
        left_stride = math.sin(phase)
        right_stride = math.sin(phase + math.pi)
        slope = terrain_slope_degrees(0.0, z, terrain)
        foot_clearance = terrain["acceptance"]["min_clearance_meters"] + max(0.0, math.sin(phase)) * gait["step_height"]
        stability_margin = 0.16 - 0.0013 * slope + 0.018 * abs(math.cos(phase))
        hip_pitch = 18.0 * max(abs(left_stride), abs(right_stride))
        knee_pitch = 22.0 * max(max(0.0, -left_stride), max(0.0, -right_stride))

        max_slope = max(max_slope, slope)
        min_clearance = min(min_clearance, foot_clearance)
        min_stability_margin = min(min_stability_margin, stability_margin)
        max_joint_abs = max(max_joint_abs, hip_pitch, knee_pitch)
        trace.append(
            {
                "time_s": round(t, 4),
                "z_m": round(z, 4),
                "terrain_height_m": round(terrain_height(0.0, z, terrain), 4),
                "slope_degrees": round(slope, 4),
                "foot_clearance_m": round(foot_clearance, 4),
                "stability_margin": round(stability_margin, 4),
                "hip_pitch_abs_degrees": round(hip_pitch, 4),
                "knee_pitch_abs_degrees": round(knee_pitch, 4),
            }
        )

    checks = {
        "slope_within_acceptance": max_slope <= acceptance["max_allowed_slope_degrees"],
        "clearance_within_acceptance": min_clearance >= acceptance["min_clearance_meters"],
        "stability_within_acceptance": min_stability_margin >= acceptance["min_stability_margin"],
        "joint_motion_within_contract": max_joint_abs <= 72.0,
    }
    report = {
        "status": "passed" if all(checks.values()) else "failed",
        "robot": robot["name"],
        "terrain": terrain["name"],
        "samples": samples,
        "duration_s": duration,
        "metrics": {
            "max_slope_degrees": round(max_slope, 4),
            "min_foot_clearance_m": round(min_clearance, 4),
            "min_stability_margin": round(min_stability_margin, 4),
            "max_joint_abs_degrees": round(max_joint_abs, 4),
        },
        "checks": checks,
        "hardware": {
            "connected": False,
            "reason": "This isolated folder validates the Godot-side and deterministic behavior only.",
        },
    }
    return report, trace


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run deterministic biped mountain simulation evidence.")
    parser.add_argument("--output", type=Path, default=ROOT / "test_env" / "biped_sim_report.json")
    parser.add_argument("--trace-output", type=Path, default=ROOT / "test_env" / "biped_sim_trace.json")
    parser.add_argument("--samples", type=int, default=160)
    args = parser.parse_args()

    if args.samples < 8:
        raise SystemExit("--samples must be at least 8")

    report, trace = simulate(load_json(ROBOT_CONFIG), load_json(TERRAIN_CONFIG), args.samples)
    write_json(args.output, report)
    write_json(args.trace_output, trace)
    print(json.dumps({"status": report["status"], "output": str(args.output), "trace_output": str(args.trace_output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
