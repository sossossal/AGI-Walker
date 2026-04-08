# Quadruped Example

This example describes the quadruped training flow that currently exists in the repository.

## What Exists

- Training script: `examples/quadruped_training.py`
- Gait controller: `agi_walker.core.api.control.gait_generator.GaitController`
- Gym environment id: `AGI-Walker/Quadruped-v0`

## Basic Environment Usage

```python
import gymnasium as gym

env = gym.make("AGI-Walker/Quadruped-v0")
obs, info = env.reset()

print(env.observation_space)
print(env.action_space)
```

## Gait Controller Usage

```python
from agi_walker.core.api.control.gait_generator import GaitController

controller = GaitController()

controller.set_gait("trot")
trot_targets = controller.get_joint_targets(phase=0.0)

controller.set_gait("gallop")
gallop_targets = controller.get_joint_targets(phase=0.0)
```

Supported gait names in the current controller are:

- `trot`
- `gallop`
- `walk`

## Train A Policy

```bash
python examples/quadruped_training.py --train --timesteps 1000000
```

The training script currently uses:

- `stable_baselines3.PPO`
- `DummyVecEnv`
- checkpoint saving under `models/quadruped`
- TensorBoard logs under `tensorboard_logs/quadruped`

## Test A Trained Policy

```bash
python examples/quadruped_training.py --test models/quadruped/ppo_quadruped_final.zip
```

## Notes

- This page documents the checked-in example script, not a separately maintained product workflow.
- If the environment id or registration changes, update this page together with the script.
- The old `python_api.gait_generator` import path is no longer current.
