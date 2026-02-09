"""
Tests for Offline RL functionality.
"""

import pytest
import sys
import os

# Add project path if needed, though pytest usually handles this
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def test_offline_rl_imports():
    """Test that offline_rl module can be imported."""
    try:
        from python_api.offline_rl import ExpertDataCollector, OfflineRLTrainer
    except ImportError as e:
        pytest.fail(f"Failed to import offline_rl modules: {e}")

def test_d3rlpy_installed():
    """Test that d3rlpy is installed."""
    pytest.importorskip("d3rlpy")
    import d3rlpy
    assert d3rlpy.__version__ is not None

def test_data_collector_creation():
    """Test ExpertDataCollector instantiation."""
    pytest.importorskip("d3rlpy")
    pytest.importorskip("gymnasium")
    
    from python_api.offline_rl import ExpertDataCollector
    try:
        collector = ExpertDataCollector("CartPole-v1")
        assert collector is not None
    except Exception as e:
        pytest.fail(f"Failed to create ExpertDataCollector: {e}")

def test_trainer_creation():
    """Test OfflineRLTrainer instantiation."""
    pytest.importorskip("d3rlpy")
    pytest.importorskip("gymnasium")
    
    from python_api.offline_rl import OfflineRLTrainer
    try:
        trainer = OfflineRLTrainer("CartPole-v1", algorithm="cql")
        assert trainer is not None
    except Exception as e:
        pytest.fail(f"Failed to create OfflineRLTrainer: {e}")
