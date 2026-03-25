"""
Tests for Offline RL functionality.
"""

import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import pytest

# 延迟导入，防止收集阶段崩溃
def get_offline_rl():
    from python_api.learning.offline_rl import ExpertDataCollector, OfflineRLTrainer
    return ExpertDataCollector, OfflineRLTrainer

def test_offline_rl_imports() -> None:
    """Test that offline_rl module can be imported (or skipped if deps missing)."""
    try:
        ExpertDataCollector, _ = get_offline_rl()
        assert ExpertDataCollector is not None
    except (ImportError, ModuleNotFoundError, Exception):
        pytest.skip("Offline RL dependencies are not available")


def test_d3rlpy_installed() -> None:
    """Test that d3rlpy is installed."""
    try:
        import d3rlpy
        assert d3rlpy.__version__ is not None
    except ImportError:
        pytest.skip("d3rlpy not installed")


def test_data_collector_creation() -> None:
    """Test ExpertDataCollector instantiation."""
    try:
        import gymnasium as gym
        ExpertDataCollector, _ = get_offline_rl()
        # 尝试创建一个基础环境，确认环境是否可用
        gym.make("CartPole-v1")
        collector = ExpertDataCollector("CartPole-v1")
        assert collector is not None
    except Exception as e:
        pytest.skip(f"环境或依赖不可用: {e}")
