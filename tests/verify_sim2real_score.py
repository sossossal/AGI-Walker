from __future__ import annotations

import logging
import sys
from pathlib import Path

import numpy as np


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.sysid.metrics import Sim2RealScorer


def _configure_runtime() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if hasattr(sys.stderr, "reconfigure"):
        sys.stderr.reconfigure(encoding="utf-8", errors="replace")


def create_mock_data(offset_amplitude: float = 0.0) -> dict:
    measurements = []
    for index in range(100):
        base = float(np.sin(index * 0.1))
        offset = float(offset_amplitude * np.sin(index * 0.3))
        measurements.append(
            {
                "state": {
                    "joints": {
                        "hip_left": base + offset,
                        "hip_right": -base + offset,
                    }
                }
            }
        )
    return {"measurements": measurements}


def verify_sim2real_score() -> bool:
    logger.info("=== Sim2Real Score Verification ===")

    try:
        scorer = Sim2RealScorer(sensitivity=10.0)

        logger.info("[1/4] Building deterministic datasets...")
        real = create_mock_data(0.0)
        sim_perfect = create_mock_data(0.0)
        sim_good = create_mock_data(0.05)
        sim_bad = create_mock_data(0.5)

        logger.info("[2/4] Calculating scores...")
        perfect_score = scorer.calculate_score(real, sim_perfect)
        good_score = scorer.calculate_score(real, sim_good)
        bad_score = scorer.calculate_score(real, sim_bad)

        logger.info("[3/4] Checking score ordering...")
        if abs(perfect_score - 100.0) > 1e-6:
            raise AssertionError(f"Perfect score drifted: {perfect_score}")
        if not (perfect_score > good_score > bad_score):
            raise AssertionError(
                f"Score ordering invalid: perfect={perfect_score}, good={good_score}, bad={bad_score}"
            )
        if not (90.0 < good_score < 100.0):
            raise AssertionError(f"Good-score range invalid: {good_score}")
        if bad_score >= 50.0:
            raise AssertionError(f"Bad-score range invalid: {bad_score}")

        logger.info("[4/4] Validating report fields...")
        report = scorer.generate_report(real, sim_good)
        if set(report.get("joint_errors", {}).keys()) != {"hip_left", "hip_right"}:
            raise AssertionError(f"Joint error keys invalid: {report}")
        if abs(report["overall_score"] - good_score) > 1e-6:
            raise AssertionError(
                f"Report score mismatch: report={report['overall_score']} calc={good_score}"
            )
        if report["overall_mse"] <= 0.0:
            raise AssertionError(f"Expected positive MSE in report: {report}")

        logger.info("PASS: Sim2Real score verification completed")
        return True
    except Exception as exc:
        logger.exception("FAIL: %s", exc)
        return False


if __name__ == "__main__":
    _configure_runtime()
    raise SystemExit(0 if verify_sim2real_score() else 1)
