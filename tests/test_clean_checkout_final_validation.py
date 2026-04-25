from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "run_clean_checkout_final_validation_module",
        PROJECT_ROOT / "tools" / "run_clean_checkout_final_validation.py",
    )
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_build_validation_commands_defaults_to_blocker_and_readiness() -> None:
    module = _load_module()

    commands = module.build_validation_commands(
        "python",
        skip_worktree_blocker=False,
        skip_readiness=False,
    )

    assert commands == [
        ["python", "tools/run_worktree_release_blocker.py"],
        ["python", "tools/check_release_readiness.py"],
    ]


def test_build_validation_commands_can_skip_each_step() -> None:
    module = _load_module()

    assert module.build_validation_commands(
        "python",
        skip_worktree_blocker=True,
        skip_readiness=False,
    ) == [["python", "tools/check_release_readiness.py"]]
    assert module.build_validation_commands(
        "python",
        skip_worktree_blocker=False,
        skip_readiness=True,
    ) == [["python", "tools/run_worktree_release_blocker.py"]]
    assert module.build_validation_commands(
        "python",
        skip_worktree_blocker=True,
        skip_readiness=True,
    ) == []
