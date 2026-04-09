import psutil
import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


class OffloadingScheduler:
    """
    AGI-Walker V2.5 Cloud-Edge Offloading Scheduler.
    Determines whether to run AI inference locally or offload to the cloud.
    """

    def __init__(
        self,
        cpu_threshold: float = 85.0,
        latency_threshold_ms: float = 50.0,
        mem_threshold: float = 90.0,
    ):
        self.cpu_threshold = cpu_threshold
        self.latency_threshold = latency_threshold_ms
        self.mem_threshold = mem_threshold
        self.cloud_available = False
        self.last_rtt = 0.0

    def should_offload(self) -> bool:
        """
        Decision Logic:
        1. If local CPU > threshold -> Offload (if cloud OK).
        2. If local Memory > threshold -> Offload.
        3. If cloud RTT > threshold -> Keep local.
        """
        cpu_usage = psutil.cpu_percent()
        mem_usage = psutil.virtual_memory().percent

        # 1. Hardware Pressure Check
        is_locally_pressed = (
            cpu_usage > self.cpu_threshold or mem_usage > self.mem_threshold
        )

        # 2. Network Viability Check
        is_cloud_viable = self.cloud_available and (
            self.last_rtt < self.latency_threshold
        )

        if is_locally_pressed and is_cloud_viable:
            logger.info(f"⚖️ Offloading: CPU={cpu_usage}%, RTT={self.last_rtt:.1f}ms")
            return True

        return False

    def update_cloud_stats(self, rtt_ms: float, available: bool):
        """Update RTT and availability from Sidecar's Zenoh link."""
        self.last_rtt = rtt_ms
        self.cloud_available = available

    def get_status(self) -> Dict[str, Any]:
        return {
            "cpu": psutil.cpu_percent(),
            "mem": psutil.virtual_memory().percent,
            "cloud_available": self.cloud_available,
            "last_rtt": self.last_rtt,
            "mode": "offloading" if self.should_offload() else "local",
        }
