# AGI-Walker Core API

This directory contains the lower-level API surface used by the Godot simulation environment, communication clients, control helpers, and local examples.
It is part of the `agi_walker` package and no longer lives under a separate `python_api/` source tree.

## Install

Install from the repository root:

```bash
pip install -e .
```

If you want the RL and training extras used by some examples:

```bash
pip install -e ".[training]"
```

## Main Package Surface

The Godot-compatible environment package lives at:

```python
from agi_walker.core.api.godot_robot_env import GodotRobotEnv, PartsDatabase
```

This package provides:

- `GodotRobotEnv` for Gym/Gymnasium-style simulation loops
- `PartsDatabase` for robot part lookup and config assembly
- Communication clients under `comm/`
- Control, diagnostics, and sensor helpers under the surrounding `core/api/` package

## Local Example Scripts

The local examples in this directory still run as scripts from the repository checkout:

```bash
python agi_walker/core/api/examples/test_parts.py
python agi_walker/core/api/examples/train_walker_ppo.py --timesteps 100000
```

Those scripts add the local package directory to `sys.path` so they can be executed directly from the repo root.

## Godot Environment Example

```python
from agi_walker.core.api.godot_robot_env import GodotRobotEnv

env = GodotRobotEnv(host="127.0.0.1", port=9999)
obs, info = env.reset()

for _ in range(1000):
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

## Training Example

Before running PPO training, start a compatible Godot simulation endpoint on port `9999`, then run:

```bash
python agi_walker/core/api/examples/train_walker_ppo.py --timesteps 100000
```

To test a saved model:

```bash
python agi_walker/core/api/examples/train_walker_ppo.py --mode test --model-path ./models/walker_ppo/walker_ppo_final
```

## Notes

- The examples under this directory are legacy local demos, not the repository's main product entrypoints.
- For the main runtime surfaces, prefer the top-level CLI, Web Panel, and MCP server documented in the repository root `README.md`.
