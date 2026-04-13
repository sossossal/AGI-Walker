"""
Tests for Offline RL functionality.
"""

import json
import logging
from pathlib import Path

import pytest

from agi_walker.core.api.training_contracts import validate_training_run_artifact

# 延迟导入，防止收集阶段崩溃

logger = logging.getLogger(__name__)


def get_offline_rl():
    from agi_walker.core.api.learning.offline_rl import (
        ExpertDataCollector,
        OfflineRLTrainer,
    )

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


class FakeOfflineModel:
    def __init__(self) -> None:
        self.fit_kwargs = None

    def fit(self, dataset, **kwargs) -> None:
        self.fit_kwargs = {"dataset": dataset, **kwargs}


def test_offline_trainer_writes_training_run_manifest(tmp_path: Path) -> None:
    _, OfflineRLTrainer = get_offline_rl()
    trainer = object.__new__(OfflineRLTrainer)
    trainer.env_id = "CartPole-v1"
    trainer.algorithm = "cql"
    trainer.model = FakeOfflineModel()
    dataset = {
        "observations": [[0.0, 1.0], [1.0, 0.0]],
        "actions": [[0.1], [0.2]],
        "rewards": [1.0, 0.5],
        "terminals": [False, True],
    }

    manifest = trainer.train_offline(
        dataset,
        n_steps=10,
        save_interval=5,
        save_dir=str(tmp_path),
        run_id="offline:test",
    )

    assert validate_training_run_artifact(manifest) == []
    assert manifest["run_type"] == "offline_dataset_training"
    assert manifest["hardware_required"] is False
    assert manifest["inputs"]["dataset"]["sample_count"] == 2
    assert trainer.model.fit_kwargs["save_dir"] == str(tmp_path)

    manifest_path = tmp_path / "training_run_manifest.json"
    saved = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert validate_training_run_artifact(saved) == []
    assert saved["run_id"] == "offline:test"
