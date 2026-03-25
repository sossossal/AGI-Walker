import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import time
import numpy as np
from algorithms.dreamer_env import DreamerEnv


def test_dreamer_interface() -> None:
    logger.info("🧪 Verification: DreamerV3 Interface")

    # 1. Connect
    env = DreamerEnv(port=9000)
    logger.info("   Attempting connection...")
    try:
        obs, _ = env.reset()
        logger.info("   ✅ Connection Established & Reset Successful")
    except Exception as e:
        logger.info(f"   ❌ Connection Failed: {e}")
        return

    # 2. Latency Test
    logger.info("\n   📉 Running Latency Test (100 steps)...")
    latencies = []

    for i in range(100):
        # Random Action
        action = np.random.uniform(-1.0, 1.0, size=(12,))

        obs, reward, done, _, info = env.step(action)
        latencies.append(info["latency"])

        if i % 20 == 0:
            logger.info(f"      Step {i}: Latency={info['latency']:.2f}ms, Reward={reward}")

    avg_latency = np.mean(latencies)
    logger.info(f"\n   ✅ Average Latency: {avg_latency:.2f}ms")

    if avg_latency < 10.0:
        logger.info("   🚀 Performance: EXCELLENT (<10ms)")
    elif avg_latency < 20.0:
        logger.info("   ⚠️ Performance: GOOD (<20ms)")
    else:
        logger.info("   ❌ Performance: POOR (>20ms)")

    env.close()


if __name__ == "__main__":
    test_dreamer_interface()
