from __future__ import annotations

import logging
import os
import platform
import time
from enum import Enum
from typing import Any, Dict, Optional

import psutil


logger = logging.getLogger(__name__)


class ControlMode(Enum):
    OFF = "off"
    PID = "pid"
    AI = "ai"
    SAFETY = "safety"


class SimplePIDController:
    """Small balance-oriented PID helper used by fallback control."""

    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = 45.0
        self._integral_roll = 0.0
        self._integral_pitch = 0.0
        self._last_roll = 0.0
        self._last_pitch = 0.0

    def _axis_step(self, error: float, integral: float, previous: float, dt: float):
        safe_dt = max(dt, 1e-6)
        integral += error * safe_dt
        derivative = (error - previous) / safe_dt
        output = self.kp * error + self.ki * integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        return output, integral, error

    def compute(self, roll_error: float, pitch_error: float, dt: float = 0.02):
        roll_cmd, self._integral_roll, self._last_roll = self._axis_step(
            roll_error, self._integral_roll, self._last_roll, dt
        )
        pitch_cmd, self._integral_pitch, self._last_pitch = self._axis_step(
            pitch_error, self._integral_pitch, self._last_pitch, dt
        )

        hip_left = max(-self.output_limit, min(self.output_limit, -(roll_cmd + pitch_cmd)))
        hip_right = max(-self.output_limit, min(self.output_limit, roll_cmd - pitch_cmd))
        return float(hip_left), float(hip_right)


class SystemMonitor:
    """Tracks basic CPU/memory/temperature signals for overload fallback."""

    def __init__(self, cpu_threshold: float = 90.0, temp_threshold: float = 80.0):
        self.cpu_threshold = cpu_threshold
        self.temp_threshold = temp_threshold
        self.os_type = platform.system()

    def get_hw_stats(self) -> Dict[str, Any]:
        return {
            "cpu_percent": psutil.cpu_percent(interval=None),
            "memory_percent": psutil.virtual_memory().percent,
            "load_avg": self._get_load_avg(),
            "temperature": self._get_temperature(),
            "timestamp": time.time(),
        }

    def _get_load_avg(self):
        if hasattr(os, "getloadavg"):
            return os.getloadavg()
        return [0.0, 0.0, 0.0]

    def _get_temperature(self) -> Optional[float]:
        if self.os_type != "Linux":
            return None
        try:
            temps = psutil.sensors_temperatures()
            for name in ["coretemp", "cpu-thermal", "soc_thermal"]:
                if name in temps and temps[name]:
                    return temps[name][0].current
        except Exception:
            return None
        return None

    def is_overloaded(self) -> bool:
        stats = self.get_hw_stats()
        if stats["cpu_percent"] > self.cpu_threshold:
            return True
        if stats["temperature"] and stats["temperature"] > self.temp_threshold:
            return True
        return False

    def report(self) -> None:
        stats = self.get_hw_stats()
        temp_str = f"{stats['temperature']:.1f}C" if stats["temperature"] else "N/A"
        logger.info(
            "Hardware: CPU=%s%% | MEM=%s%% | TEMP=%s",
            stats["cpu_percent"],
            stats["memory_percent"],
            temp_str,
        )


class LoadMonitor:
    """Latency-aware control mode switcher with PID fallback."""

    def __init__(
        self,
        pid_controller: Optional[SimplePIDController] = None,
        fallback_latency_ms: float = 25.0,
        recovery_latency_ms: float = 15.0,
        ema_alpha: float = 0.25,
    ):
        self.system_monitor = SystemMonitor()
        self.pid = pid_controller or SimplePIDController()
        self.fallback_latency_ms = fallback_latency_ms
        self.recovery_latency_ms = recovery_latency_ms
        self.ema_alpha = ema_alpha

        self.current_mode = ControlMode.AI
        self.total_samples = 0
        self.ema_latency = 0.0
        self.on_mode_change = None
        self._high_latency_streak = 0
        self._low_latency_streak = 0

    def _set_mode(self, mode: ControlMode) -> None:
        if mode == self.current_mode:
            return
        self.current_mode = mode
        if self.on_mode_change:
            self.on_mode_change(mode)

    def record_latency(self, ms: float) -> None:
        latency = float(ms)
        self.total_samples += 1
        if self.total_samples == 1:
            self.ema_latency = latency
        else:
            self.ema_latency = (
                self.ema_alpha * latency + (1.0 - self.ema_alpha) * self.ema_latency
            )

        if latency >= self.fallback_latency_ms:
            self._high_latency_streak += 1
            self._low_latency_streak = 0
        elif latency <= self.recovery_latency_ms:
            self._low_latency_streak += 1
            self._high_latency_streak = 0
        else:
            self._high_latency_streak = 0
            self._low_latency_streak = 0

        if self._high_latency_streak >= 5:
            self._set_mode(ControlMode.PID)
        elif self.current_mode == ControlMode.PID and self._low_latency_streak >= 10:
            self._set_mode(ControlMode.AI)

    def get_control_action(self, sensor_data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        sensor_data = sensor_data or {}
        orient = (
            sensor_data.get("sensors", {})
            .get("imu", {})
            .get("orient", [0.0, 0.0, 0.0])
        )
        roll = float(orient[0]) if len(orient) > 0 else 0.0
        pitch = float(orient[1]) if len(orient) > 1 else 0.0

        if self.current_mode == ControlMode.PID:
            hip_left, hip_right = self.pid.compute(roll, pitch)
            return {
                "mode": self.current_mode.value,
                "motors": {
                    "hip_left": hip_left,
                    "hip_right": hip_right,
                },
            }

        if self.current_mode == ControlMode.SAFETY:
            return {
                "mode": self.current_mode.value,
                "motors": {
                    "hip_left": 0.0,
                    "hip_right": 0.0,
                },
            }

        return {
            "mode": self.current_mode.value,
            "policy": "ai_primary",
        }

    def check_system(self) -> None:
        if self.system_monitor.is_overloaded():
            self._set_mode(ControlMode.SAFETY)
        self.system_monitor.report()
