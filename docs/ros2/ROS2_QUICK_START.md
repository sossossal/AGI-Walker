# AGI-Walker ROS 2 Quick Start

This page describes the ROS 2 workspace that currently exists in the repository. It is a reference integration layer under `hardware/ros2_ws`, not part of the main CLI, Web, or MCP workflow.

This guide is now referenced by `extension_execution_plan.profiles[ros2_bridge_extension].runbook_entrypoints` and aligned with the same profile's `execution_template`. If the ROS2 bridge rollout, validation, or rollback flow changes, update both this page and the machine-readable execution plan.

## Current Workspace

The ROS 2 workspace contains two packages:

- `hardware/ros2_ws/src/agi_walker_msgs`
  - Robot model/state messages: `Part.msg`, `Connection.msg`, `RobotState.msg`
  - Instruction/circuit messages: `InstructionStep.msg`, `InstructionSet.msg`, `SimulatedCircuit.msg`
  - Hardware recovery messages: `HardwareFault.msg`, `HardwareRecoveryAction.msg`, `HardwareRecoveryStatus.msg`
  - Behavior/navigation/perception messages: `BehaviorCommand.msg`, `NavigationGoal.msg`, `PerceptionSnapshot.msg`
  - Services: `LoadRobot.srv`, `ApplyInstructionSet.srv`, `ConfigureSimulatedCircuit.srv`, `HardwareRecovery.srv`
- `hardware/ros2_ws/src/agi_walker_ros2`
  - Bridge implementation: `agi_walker_ros2/bridge_node.py`
  - Launch files: `launch/agi_walker.launch.py`, `launch/robot.launch.py`
  - Parameters: `config/params.yaml`
  - URDF: `urdf/agi_walker.urdf`

## Recommended Environment

- Ubuntu 22.04 with ROS 2 Humble
- `colcon` and the standard ROS 2 Python toolchain
- A Godot simulation endpoint reachable by the bridge node

## Build The Workspace

```bash
cd hardware/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## What Exists Today

The bridge node in `agi_walker_ros2/bridge_node.py` currently defines:

- Publishers: `/joint_states`, `/robot_state` (optional), `/battery`, `/imu`
- Subscriber: `/cmd_vel`
- Services: `/start_simulation`, `/stop_simulation`, `/load_robot` (optional)
- Custom typed IDL for instruction-set, simulated-circuit and hardware-recovery contracts under `agi_walker_msgs`
- Typed compatibility topics/services: `/instruction_set`, `/simulated_circuit`, `/instruction_set/apply`, `/simulated_circuit/configure`, `/hardware/recovery`
- Behavior/navigation/perception topics: `/behavior_command`, `/navigation_goal`, `/perception_snapshot`
- Parameters: Godot host and port, publish rates, control gains, PID gains

The minimal bridge entry is:

```bash
ros2 launch agi_walker_ros2 agi_walker.launch.py
```

There is also a wrapper launch that starts the bridge and optionally adds `robot_state_publisher` and RViz:

```bash
ros2 launch agi_walker_ros2 robot.launch.py
```

Use `config_file` to switch profiles without editing the launch file:

```bash
ros2 launch agi_walker_ros2 agi_walker.launch.py \
  config_file:=install/agi_walker_ros2/share/agi_walker_ros2/config/profiles/replay.yaml
```

Checked-in profiles:

- `config/profiles/local.yaml`
- `config/profiles/replay.yaml`
- `config/profiles/live.yaml`

## Known Drift You Should Expect

This ROS 2 workspace is not fully aligned yet. The main issues are:

- `tf_rate` is declared as a parameter, but the current bridge implementation does not publish TF on a timer.
- The bridge still relies on repo-root path injection when run directly from a source checkout.
- The ROS 2 workspace is still lightly tested compared with the main CLI, Web, and MCP paths.

Because of that, treat this workspace as a secondary integration path rather than the repository's most battle-tested runtime surface.

## Practical Validation

The basic validation loop is:

```bash
ros2 topic list
ros2 topic echo /joint_states
ros2 service call /start_simulation std_srvs/srv/Trigger
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.1}}" --once
```

For environments without ROS 2 installed, the repository now includes a fake-runtime bridge regression path:

```bash
python -m pytest tests/test_ros2_bridge_runtime.py tests/test_ros2_workspace.py -q
```

This path validates:

- replay payload shape for bridge input data
- bag-style replay manifest shape and ordered topic application
- multi-node replay smoke report generation
- JointState and RobotState publication mapping
- `/cmd_vel` to Godot parameter translation
- basic start/stop service flow under a fake ROS 2 runtime

For a real ROS 2 Humble runtime, there is now an explicit live smoke path:

```bash
export AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"
```

This path requires:

- ROS 2 Humble Python runtime (`rclpy`, `sensor_msgs`, `geometry_msgs`, `std_srvs`, `tf2_ros`)
- a local Python environment that can import the repo

It uses a repository-local mock Godot TCP server and writes:

- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`

If the runtime is missing, the smoke still writes a structured `skipped` report
that names the missing Python modules instead of failing with a bare
`pytest.importorskip` message.

## Recommendation

If you need a production ROS 2 workflow, run both layers:

- fake-runtime mapping tests for fast regression
- live ROS 2 smoke in the target Humble environment for startup, topic, service, and TCP bridge validation

For downstream migration from JSON topics/services to typed IDL, follow:

- `docs/ros2/ROS2_TYPED_IDL_MIGRATION.md`
