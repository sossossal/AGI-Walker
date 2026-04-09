"""
Mock validation for the evolution loop.

Runs the current EvolutionManager stages with mocked optional ML dependencies,
UTF-8-safe stdio, and an isolated temporary workspace so the verification does
not pollute the repository.
"""

import asyncio
import json
import logging
import shutil
import sys
from pathlib import Path
from typing import Dict
from unittest.mock import MagicMock, patch


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent
_MISSING = object()


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


def setup_mocks() -> Dict[str, object]:
    """Install temporary dependency mocks and return original module bindings."""
    originals: Dict[str, object] = {}

    def install(name: str, module: object) -> None:
        originals[name] = sys.modules.get(name, _MISSING)
        sys.modules[name] = module

    mock_sb3 = MagicMock()
    mock_sb3.__version__ = "2.0.0"
    install("stable_baselines3", mock_sb3)
    install("stable_baselines3.common", MagicMock())
    install("stable_baselines3.common.vec_env", MagicMock())
    install("stable_baselines3.common.callbacks", MagicMock())
    install("stable_baselines3.common.evaluation", MagicMock())

    mock_model = MagicMock()
    mock_model.predict.return_value = ([0.1, 0.1], None)
    mock_sb3.PPO.return_value = mock_model
    mock_sb3.PPO.load.return_value = mock_model

    install("gymnasium", MagicMock())
    install("gymnasium.spaces", MagicMock())

    mock_peft = MagicMock()
    mock_peft.TaskType.CAUSAL_LM = "CAUSAL_LM"
    install("peft", mock_peft)
    install("peft.utils", MagicMock())
    install("peft.mapping", MagicMock())
    install("peft.peft_model", MagicMock())

    install("transformers", MagicMock())
    install("datasets", MagicMock())
    return originals


def restore_mocks(originals: Dict[str, object]) -> None:
    for name, original in originals.items():
        if original is _MISSING:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = original


async def _fast_sleep(_seconds: float) -> None:
    return None


def _fake_rl_train(self, total_timesteps: int = 0, **_kwargs) -> dict:
    final_path = self.save_dir / f"{self.config.algorithm.lower()}_final.zip"
    final_path.parent.mkdir(parents=True, exist_ok=True)
    final_path.write_text("mock rl model", encoding="utf-8")
    result = {
        "algorithm": self.config.algorithm,
        "total_timesteps": total_timesteps,
        "training_time": 0.0,
        "model_path": str(final_path),
    }
    self.training_history.append(result)
    return result


def _fake_export_policy_onnx(_self, output_path: str) -> None:
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("mock onnx export", encoding="utf-8")


async def run_verification() -> bool:
    _configure_stdio()
    logger.info("开始 Mock 验证进化循环...")

    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))

    originals = setup_mocks()

    try:
        from agi_walker.core.controllers.evolution_manager import (
            EvolutionConfig,
            EvolutionManager,
        )

        test_env_dir = PROJECT_ROOT / "test_env"
        test_env_dir.mkdir(parents=True, exist_ok=True)

        workspace_dir = test_env_dir / "verify_mocked_runtime"
        if workspace_dir.exists():
            shutil.rmtree(workspace_dir, ignore_errors=True)
        workspace_dir.mkdir(parents=True, exist_ok=True)

        try:
            config = EvolutionConfig(
                iteration_name="mock_test_v1",
                rl_timesteps=100,
                n_trajectories=10,
                peft_epochs=1,
                workspace_dir=str(workspace_dir),
            )
            manager = EvolutionManager(config)

            with patch(
                "agi_walker.core.controllers.evolution_manager.asyncio.sleep",
                new=_fast_sleep,
            ), patch(
                "agi_walker.core.controllers.evolution_manager.RLOptimizer.train",
                new=_fake_rl_train,
            ), patch(
                "agi_walker.core.controllers.evolution_manager.RLOptimizer.export_policy_onnx",
                new=_fake_export_policy_onnx,
            ):
                logger.info("--- 运行 Stage 1: RL Training ---")
                model_path = Path(await manager.stage_rl_training())
                if not model_path.exists():
                    raise FileNotFoundError(f"RL model not created: {model_path}")
                logger.info("PASS: RL Training")

                logger.info("--- 运行 Stage 2: Data Generation ---")
                raw_data_path = Path(await manager.stage_data_generation(str(model_path)))
                if not raw_data_path.exists():
                    raise FileNotFoundError(
                        f"Raw trajectory data not created: {raw_data_path}"
                    )
                logger.info("PASS: Data Generation")

                logger.info("--- 运行 Stage 3: Data Processing ---")
                clean_data_path = Path(
                    await manager.stage_data_processing(str(raw_data_path))
                )
                if not clean_data_path.exists():
                    raise FileNotFoundError(
                        f"Clean dataset not created: {clean_data_path}"
                    )
                clean_data = json.loads(clean_data_path.read_text(encoding="utf-8"))
                if len(clean_data) == 0:
                    raise AssertionError("Clean dataset is empty")
                logger.info("PASS: Data Processing")

                logger.info("--- 运行 Stage 4: PEFT Finetuning ---")
                final_model_path = Path(
                    await manager.stage_model_finetuning(str(clean_data_path))
                )
                model_info_path = final_model_path / "model_info.json"
                if not model_info_path.exists():
                    raise FileNotFoundError(
                        f"Fine-tuning output not created: {model_info_path}"
                    )
                model_info = json.loads(model_info_path.read_text(encoding="utf-8"))
                expected_output_dir = str(manager.models_dir / "peft")
                if model_info.get("output_dir") != expected_output_dir:
                    raise AssertionError(
                        f"Unexpected PEFT output_dir in stats: {model_info}"
                    )
                logger.info("PASS: PEFT Finetuning")

            logger.info("PASS: Mock evolution verification completed")
            return True
        finally:
            shutil.rmtree(workspace_dir, ignore_errors=True)
    except Exception as exc:
        logger.exception("FAIL: Mock verification failed: %s", exc)
        return False
    finally:
        restore_mocks(originals)


if __name__ == "__main__":
    raise SystemExit(0 if asyncio.run(run_verification()) else 1)
