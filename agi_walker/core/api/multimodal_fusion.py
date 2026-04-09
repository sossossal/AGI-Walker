"""Compatibility wrapper for the legacy multimodal fusion import path."""

from agi_walker.core.api.sensor.multimodal_fusion import (
    FusionConfig,
    MultimodalFusion,
    create_multimodal_fusion,
)

__all__ = [
    "FusionConfig",
    "MultimodalFusion",
    "create_multimodal_fusion",
]
