from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRACE = ROOT / "test_env" / "biped_sim_trace.json"
DEFAULT_OUTPUT = ROOT / "test_env" / "contact_stability_report.json"


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")


def build_report(trace: list[dict[str, Any]]) -> dict[str, Any]:
    if not trace:
        return {"status": "failed", "reason": "trace is empty"}

    min_clearance = min(float(row["foot_clearance_m"]) for row in trace)
    min_stability = min(float(row["stability_margin"]) for row in trace)
    max_slope = max(float(row["slope_degrees"]) for row in trace)
    max_knee = max(float(row["knee_pitch_abs_degrees"]) for row in trace)
    max_hip = max(float(row["hip_pitch_abs_degrees"]) for row in trace)

    checks = {
        "clearance_margin_positive": min_clearance >= 0.035,
        "stability_margin_positive": min_stability >= 0.10,
        "slope_within_local_mountain_contract": max_slope <= 30.0,
        "leg_joint_motion_within_contract": max(max_knee, max_hip) <= 72.0,
    }
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "samples": len(trace),
        "metrics": {
            "min_clearance_m": round(min_clearance, 4),
            "min_stability_margin": round(min_stability, 4),
            "max_slope_degrees": round(max_slope, 4),
            "max_leg_joint_abs_degrees": round(max(max_knee, max_hip), 4),
        },
        "checks": checks,
        "limitations": [
            "This is a deterministic kinematic contact check.",
            "Rigid-body actuator dynamics require a future hardware or physics integration task."
        ],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Check biped contact and stability from a local simulation trace.")
    parser.add_argument("--trace", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    report = build_report(load_json(args.trace))
    write_json(args.output, report)
    print(json.dumps({"status": report["status"], "output": str(args.output)}, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
