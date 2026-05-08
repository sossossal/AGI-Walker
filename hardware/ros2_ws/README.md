# AGI-Walker ROS 2 Workspace

This directory contains the repository's reference ROS 2 workspace under `hardware/ros2_ws`.
It is a secondary integration surface around the main Python package, not the primary CLI, Web, or MCP runtime.

## Workspace Layout

- `src/agi_walker_msgs`
  - Robot model/state messages: `Part.msg`, `Connection.msg`, `RobotState.msg`
  - Instruction/circuit messages: `InstructionStep.msg`, `InstructionSet.msg`, `SimulatedCircuit.msg`
  - Hardware recovery messages: `HardwareFault.msg`, `HardwareRecoveryAction.msg`, `HardwareRecoveryStatus.msg`
  - Behavior/navigation/perception messages: `BehaviorCommand.msg`, `NavigationGoal.msg`, `PerceptionSnapshot.msg`
  - Services: `LoadRobot.srv`, `ApplyInstructionSet.srv`, `ConfigureSimulatedCircuit.srv`, `HardwareRecovery.srv`
- `src/agi_walker_ros2`
  - Bridge implementation: `agi_walker_ros2/bridge_node.py`
  - Launch files: `launch/agi_walker.launch.py`, `launch/robot.launch.py`
  - Parameters: `config/params.yaml`
  - URDF: `urdf/agi_walker.urdf`

## Prerequisites

- Ubuntu 22.04 or another ROS 2 Humble-compatible environment
- ROS 2 Humble and `colcon`
- A checkout of this repository with the Python package installed from the repo root

Install the Python package from the repository root first:

```bash
cd /path/to/AGI-Walker
pip install -e .
```

## Build

```bash
cd hardware/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Launch

Minimal bridge launch:

```bash
ros2 launch agi_walker_ros2 agi_walker.launch.py
```

Wrapper launch with optional `robot_state_publisher` and RViz:

```bash
ros2 launch agi_walker_ros2 robot.launch.py
```

The bridge reads defaults from `src/agi_walker_ros2/config/params.yaml`.
You can select a layered profile with `config_file`, for example:

```bash
ros2 launch agi_walker_ros2 agi_walker.launch.py \
  config_file:=install/agi_walker_ros2/share/agi_walker_ros2/config/profiles/replay.yaml
```

Available checked-in profiles:

- `config/profiles/local.yaml`
- `config/profiles/replay.yaml`
- `config/profiles/live.yaml`

## Runtime Surface

The current bridge exposes:

- Publishers: `/joint_states`, `/battery`, `/imu`
- Optional publisher path: `/robot_state`
- Subscriber: `/cmd_vel`
- Services: `/start_simulation`, `/stop_simulation`, `/load_robot`
- Typed custom IDL is available for future non-JSON instruction, simulated circuit, and hardware recovery service surfaces under `agi_walker_msgs`.
- Typed compatibility topics/services: `/instruction_set`, `/simulated_circuit`, `/instruction_set/apply`, `/simulated_circuit/configure`, `/hardware/recovery`
- Behavior/navigation/perception topics: `/behavior_command`, `/navigation_goal`, `/perception_snapshot`
- Launch profile selection via `config_file` for local, replay, and live candidate runs.

## Basic Validation

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 service call /start_simulation std_srvs/srv/Trigger
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" --once
```

Non-ROS environments can still validate the bridge shape through the repository test suite:

```bash
python -m pytest tests/test_ros2_bridge_runtime.py tests/test_ros2_workspace.py -q
```

The fake-runtime suite also covers a bag-style replay manifest:

- `tests/fixtures/ros2_bridge_bag_replay.json`
- ordered `/cmd_vel`, `/simulated_circuit/json`, and `/instruction_set/json` application
- multi-node smoke reporting with expected node coverage and state coverage checks

Real ROS 2 Humble environments can also run the opt-in live smoke:

```bash
export AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

That smoke writes:

- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

## Current Status

This workspace has already been aligned with the current repository structure:

- `bridge_node.py` imports `GodotSimulationClient` from `agi_walker.core.api.comm.godot_client`
- `setup.py` exports the real `bridge_node` entrypoint
- `robot.launch.py` wraps `agi_walker.launch.py` instead of referencing removed nodes

It is still less battle-tested than the main CLI, Web Panel, and MCP paths, so validate it in your target ROS 2 environment before treating it as production-ready.

## Related Docs

- `docs/ros2/ROS2_QUICK_START.md`
- `docs/ros2/ROS2_INTEGRATION_DESIGN.md`
- `docs/ros2/ROS2_TYPED_IDL_MIGRATION.md`
