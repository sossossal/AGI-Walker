"""Compatibility wrapper for the legacy vision processor import path."""

from agi_walker.core.api.sensor.vision_processor import (
    VisionConfig,
    VisionProcessor,
    create_vision_processor,
)

__all__ = [
    "VisionConfig",
    "VisionProcessor",
    "create_vision_processor",
]
