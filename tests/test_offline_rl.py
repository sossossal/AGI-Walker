"""
Tests for Offline RL functionality.
"""

import pytest
import sys
import os

# Add project path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


def test_offline_rl_imports():
    """Test that offline_rl module can be imported (or skipped if deps missing)."""
    # 彻底保护导入
    try:
        from python_api.learning.offline_rl import ExpertDataCollector, OfflineRLTrainer
        assert ExpertDataCollector is not None
    except (ImportError, ModuleNotFoundError):
        pytest.skip("Offline RL dependencies (like d3rlpy) are not available")


def test_d3rlpy_installed():
    """Test that d3rlpy is installed."""
    pytest.importorskip("d3rlpy")
    import d3rlpy
    assert d3rlpy.__version__ is not None


def test_data_collector_creation():
    """Test ExpertDataCollector instantiation."""
    pytest.importorskip("d3rlpy")
    pytest.importorskip("gymnasium")

    from python_api.learning.offline_rl import ExpertDataCollector

    try:
        collector = ExpertDataCollector("CartPole-v1")
        assert collector is not None
    except Exception as e:
        pytest.fail(f"Failed to create ExpertDataCollector: {e}")
