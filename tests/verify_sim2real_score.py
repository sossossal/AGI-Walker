import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import numpy as np
import json
from sysid.metrics import Sim2RealScorer


def create_mock_data(noise_level=0.0):
    steps = 100
    measurements = []
    for i in range(steps):
        val = np.sin(i * 0.1)
        noise = np.random.normal(0, noise_level)
        measurements.append(
            {"state": {"joints": {"hip_left": val + noise, "hip_right": -val + noise}}}
        )
    return {"measurements": measurements}


def test_scorer() -> None:
    logger.info("🧪 Verification: Sim2Real Scorer")
    scorer = Sim2RealScorer(sensitivity=10.0)  # High sensitivity for demo

    # Case 1: Perfect Match
    real = create_mock_data(0.0)
    sim_perfect = create_mock_data(0.0)
    score_p = scorer.calculate_score(real, sim_perfect)
    logger.info(f"   Case 1 (Perfect): Score = {score_p:.2f} (Expected 100.0)")
    assert abs(score_p - 100.0) < 0.1

    # Case 2: Small Noise (0.05)
    sim_good = create_mock_data(0.05)
    score_g = scorer.calculate_score(real, sim_good)
    logger.info(f"   Case 2 (Small Noise): Score = {score_g:.2f} (Expected ~High)")
    assert score_g < 100.0 and score_g > 50.0

    # Case 3: Large Noise (0.5)
    sim_bad = create_mock_data(0.5)
    score_b = scorer.calculate_score(real, sim_bad)
    logger.info(f"   Case 3 (Large Noise): Score = {score_b:.2f} (Expected ~Low)")
    assert score_b < 50.0

    logger.info("✅ All metric tests passed!")


if __name__ == "__main__":
    test_scorer()
