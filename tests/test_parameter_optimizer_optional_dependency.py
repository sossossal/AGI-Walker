from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def test_parameter_optimizer_falls_back_without_scipy() -> None:
    script = f"""
import builtins
import sys

real_import = builtins.__import__

def blocked_import(name, globals=None, locals=None, fromlist=(), level=0):
    if name == "scipy" or name.startswith("scipy."):
        raise ModuleNotFoundError("No module named 'scipy'")
    return real_import(name, globals, locals, fromlist, level)

builtins.__import__ = blocked_import
sys.path.insert(0, r"{PROJECT_ROOT}")

import agi_walker
from agi_walker.skills.parameter_optimizer import (
    optimize_mass_distribution,
    tune_pid_controller,
)

robot = {{
    "parts": [
        {{"id": "torso", "type": "torso", "params": {{"mass": 5.0}}}},
        {{
            "id": "left_leg",
            "type": "leg",
            "params": {{"mass": 1.0, "thigh_length": 0.3, "shin_length": 0.3}},
        }},
    ],
    "connections": [{{"from": "torso", "to": "left_leg"}}],
    "metadata": {{}},
}}

mass_result = optimize_mass_distribution(
    robot,
    target_com_height=0.22,
    max_iterations=5,
)
pid_result = tune_pid_controller(
    robot,
    joint_name="hip_flex",
    method="genetic",
    population_size=4,
    generations=3,
)

assert hasattr(agi_walker, "list_skills")
assert mass_result.iterations > 0
assert mass_result.success is True
assert isinstance(pid_result.kp, float)
print("scipy_fallback_ok")
"""

    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )

    assert (
        result.returncode == 0
    ), f"stdout:\n{result.stdout}\n\nstderr:\n{result.stderr}"
    assert "scipy_fallback_ok" in result.stdout
