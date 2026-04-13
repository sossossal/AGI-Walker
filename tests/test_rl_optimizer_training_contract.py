import json
import sys
import types
from pathlib import Path

from agi_walker.core.api.training_contracts import validate_training_run_artifact
from agi_walker.core.controllers.rl_optimizer import RLConfig, RLOptimizer


class FakeModel:
    def __init__(self) -> None:
        self.learn_kwargs = None

    def learn(self, **kwargs) -> None:
        self.learn_kwargs = kwargs

    def save(self, path: str) -> None:
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text("fake rl model", encoding="utf-8")


class FakeCheckpointCallback:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs


class FakeEvalCallback:
    def __init__(self, *args, **kwargs) -> None:
        self.args = args
        self.kwargs = kwargs


class FakeSimEnv:
    pass


class FakeHardwareEnv:
    pass


class DummyEnv:
    pass


def _install_fake_sb3_callbacks(monkeypatch) -> None:
    callbacks_module = types.ModuleType("stable_baselines3.common.callbacks")
    callbacks_module.CheckpointCallback = FakeCheckpointCallback
    callbacks_module.EvalCallback = FakeEvalCallback

    common_module = types.ModuleType("stable_baselines3.common")
    common_module.callbacks = callbacks_module

    root_module = types.ModuleType("stable_baselines3")
    root_module.common = common_module

    monkeypatch.setitem(sys.modules, "stable_baselines3", root_module)
    monkeypatch.setitem(sys.modules, "stable_baselines3.common", common_module)
    monkeypatch.setitem(
        sys.modules, "stable_baselines3.common.callbacks", callbacks_module
    )


def _make_optimizer(tmp_path: Path, env) -> RLOptimizer:
    optimizer = object.__new__(RLOptimizer)
    optimizer.config = RLConfig(algorithm="PPO")
    optimizer.save_dir = tmp_path
    optimizer.env = env
    optimizer.vec_env = None
    optimizer.model = FakeModel()
    optimizer.training_history = []
    optimizer.best_reward = float("-inf")
    return optimizer


def test_rl_optimizer_train_writes_sim_training_manifest(monkeypatch, tmp_path: Path):
    _install_fake_sb3_callbacks(monkeypatch)
    optimizer = _make_optimizer(tmp_path, FakeSimEnv())

    result = optimizer.train(
        total_timesteps=12,
        eval_freq=4,
        n_eval_episodes=2,
        run_id="sim:test",
    )

    manifest_path = Path(result["training_run_artifact"])
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))

    assert result["run_type"] == "sim_training"
    assert validate_training_run_artifact(payload) == []
    assert payload["run_id"] == "sim:test"
    assert payload["run_type"] == "sim_training"
    assert payload["environment"]["name"] == "FakeSimEnv"
    assert payload["inputs"]["total_timesteps"] == 12
    assert optimizer.model.learn_kwargs["total_timesteps"] == 12


def test_rl_optimizer_train_can_mark_hardware_in_the_loop(monkeypatch, tmp_path: Path):
    _install_fake_sb3_callbacks(monkeypatch)
    optimizer = _make_optimizer(tmp_path, FakeHardwareEnv())

    result = optimizer.train(
        total_timesteps=5,
        eval_freq=5,
        hardware_required=True,
        hardware_enabled=False,
        run_id="hardware:test",
    )

    payload = json.loads(
        Path(result["training_run_artifact"]).read_text(encoding="utf-8")
    )

    assert result["run_type"] == "hardware_in_the_loop"
    assert validate_training_run_artifact(payload) == []
    assert payload["hardware_required"] is True
    assert payload["hardware_enabled"] is False


def test_rl_optimizer_train_marks_dummy_env_as_mock_training(
    monkeypatch, tmp_path: Path
):
    _install_fake_sb3_callbacks(monkeypatch)
    optimizer = _make_optimizer(tmp_path, DummyEnv())

    result = optimizer.train(
        total_timesteps=3,
        eval_freq=3,
        run_id="mock:test",
    )

    payload = json.loads(
        Path(result["training_run_artifact"]).read_text(encoding="utf-8")
    )

    assert result["run_type"] == "mock_training"
    assert validate_training_run_artifact(payload) == []
    assert payload["run_type"] == "mock_training"
    assert payload["environment"]["name"] == "DummyEnv"
