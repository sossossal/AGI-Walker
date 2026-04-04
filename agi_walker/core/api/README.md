# Python API for Godot Robot Environment

OpenAI Gym/Gymnasium compatible interface for robot reinforcement learning.

## Installation

```bash
cd python_api
pip install -r requirements.txt
pip install -e .  # Install in development mode
```

## Quick Start

### 1. Run Parts Database Test

```bash
python examples/test_parts.py
```

### 2. Train a Walking Robot

First, start Godot simulator:
1. Open `godot_project` in Godot
2. Press F5 to run the main scene
3. Ensure TCP server is running on port 9999

Then run training:

```bash
python examples/train_walker_ppo.py --timesteps 100000
```

### 3. Test Trained Model

```bash
python examples/train_walker_ppo.py --mode test --model-path ./models/walker_ppo/walker_ppo_final
```

## Usage

### Basic Usage

```python
from godot_robot_env import GodotRobotEnv

# Create environment
env = GodotRobotEnv(host="127.0.0.1", port=9999)

# Reset environment
obs, info = env.reset()

# Run episode
for _ in range(1000):
    action = env.action_space.sample()  # Random action
    obs, reward, terminated, truncated, info = env.step(action)
    
    if terminated or truncated:
        obs, info = env.reset()

env.close()
```

### With Stable-Baselines3

```python
from godot_robot_env import GodotRobotEnv
from stable_baselines3 import PPO

env = GodotRobotEnv()
model = PPO("MultiInputPolicy", env, verbose=1)
model.learn(total_timesteps=100000)
model.save("walker_ppo")
```

## Features

- ✅ OpenAI Gym/Gymnasium compatible
- ✅ Dict observation space (IMU, joints, contacts)
- ✅ Box action space (motor targets)
- ✅ Customizable reward function
- ✅ Physics parameter modification
- ✅ Robot parts library integration
- ✅ Real-time communication with Godot

## Documentation

See `PYTHON_API_GUIDE.md` for detailed documentation.

## License

MIT License
