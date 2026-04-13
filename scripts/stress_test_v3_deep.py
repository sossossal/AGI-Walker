import time
import sys
import os
import threading
import numpy as np
import logging

# 确保能导入 agi_walker
sys.path.append(os.getcwd())

from web_panel.godot_session_bridge import TrajectoryRecorder

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("DeepStress")


def stress_test_io_pressure(frames=2000, frequency_hz=1000):
    """
    维度 D: 测试 1kHz 录制下对主回路的 IO 压力影响
    """
    logger.info(f"--- Testing Trajectory IO Pressure ({frequency_hz}Hz) ---")
    recorder = TrajectoryRecorder(session_id="stress_session")

    dummy_state = {"sensors": {"imu": {"orient": [0.1, 0.2, 0.3]}}}
    dummy_action = [0.5] * 12

    latencies = []
    start_time = time.time()
    interval = 1.0 / frequency_hz

    logger.info(f"Recording {frames} frames...")

    for i in range(frames):
        loop_start = time.time()

        # 执行录制 (包含潜在的自动 flush)
        recorder.record(dummy_state, dummy_action)

        elapsed = time.time() - loop_start
        latencies.append(elapsed * 1000)  # 转换为 ms

        # 频率控制
        sleep_time = interval - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

    total_elapsed = time.time() - start_time
    recorder.flush()  # 强制最后一次写入

    avg_lat = np.mean(latencies)
    max_lat = np.max(latencies)

    logger.info(f"Result: Recorded {frames} frames in {total_elapsed:.2f}s")
    logger.info(f"Avg Recording Latency: {avg_lat:.4f} ms")
    logger.info(f"Max Recording Latency (Spike): {max_lat:.4f} ms")

    if max_lat > 1.0:
        logger.warning(
            "⚠️ High latency spike detected during IO. Consider async flushing."
        )
    else:
        logger.info("✅ IO stability verified. Latency well within 1ms control window.")


def stress_test_fixed_adaptive_safety():
    """
    维度 C (修复版): 测试硬件感知响应
    """
    logger.info("--- Testing Adaptive Safety Response (Fixed) ---")
    from agi_walker.core.controllers.load_monitor import SystemMonitor

    monitor = SystemMonitor(cpu_threshold=30.0)

    stop_event = threading.Event()

    def cpu_burner():
        while not stop_event.is_set():
            _ = [x**2 for x in range(1000)]

    threads = [threading.Thread(target=cpu_burner) for _ in range(4)]

    logger.info("Monitoring system...")
    for t in threads:
        t.start()

    start = time.time()
    triggered = False
    for _ in range(15):
        stats = monitor.get_hw_stats()
        if monitor.is_overloaded():
            logger.info(
                f"🔥 Safety Triggered! CPU Usage: {stats['cpu_percent']}% at {time.time() - start:.2f}s"
            )
            triggered = True
            break
        time.sleep(0.5)

    stop_event.set()
    for t in threads:
        t.join()

    if not triggered:
        logger.warning(
            "Safety not triggered. Current CPU might be too powerful for 4 burner threads."
        )


if __name__ == "__main__":
    logger.info("=== AGI-Walker V3.0 Deep Performance Probe ===")

    # 1. IO 压测
    stress_test_io_pressure()

    # 2. 硬件自适应压测
    stress_test_fixed_adaptive_safety()

    logger.info("=== Deep Stress Test Complete ===")
