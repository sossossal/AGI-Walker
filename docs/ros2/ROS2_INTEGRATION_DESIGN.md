# AGI-Walker ROS 2 Integration Design

This document records the ROS 2 integration that currently exists in the repository. It reflects the checked-in code under `hardware/ros2_ws`, including known gaps and historical drift.

## Scope

The ROS 2 layer is a bridge between AGI-Walker simulation data and standard ROS 2 interfaces. Its purpose is to expose robot state, accept motion commands, and wrap a small set of simulator control services.

It is not the primary runtime path of the project today. The main supported entry points remain the Python CLI, the Web panel, and the MCP server.

## Workspace Layout

### `agi_walker_msgs`

Custom interface package under `hardware/ros2_ws/src/agi_walker_msgs`:

- `msg/Part.msg`
- `msg/Connection.msg`
- `msg/RobotState.msg`
- `srv/LoadRobot.srv`

These files define the repository-specific contracts used by the bridge when custom messages are available.

### `agi_walker_ros2`

Bridge package under `hardware/ros2_ws/src/agi_walker_ros2`:

- Runtime node: `agi_walker_ros2/bridge_node.py`
- Launch files: `launch/agi_walker.launch.py`, `launch/robot.launch.py`
- Parameters: `config/params.yaml`
- Robot description: `urdf/agi_walker.urdf`

## Runtime Design

### Bridge Node

`AGIWalkerROS2Bridge` is the core node. It creates a ROS 2 facade around a Godot TCP client and stores the latest simulator payload in memory for periodic publication.

Internal flow:

1. Declare ROS parameters.
2. Create a `GodotSimulationClient`.
3. Register a callback that stores incoming simulator data in `latest_data`.
4. Publish ROS messages from timers.
5. Forward ROS commands back to the Godot client.

To keep this bridge testable outside a ROS 2 runtime, the node now also exposes small pure-data helpers in `bridge_node.py`:

- `cmd_vel_to_godot_params()`
- `joint_state_fields_from_latest_data()`
- `robot_state_fields_from_latest_data()`
- `validate_ros2_bridge_replay_payload()`
- `load_ros2_bridge_replay_payload()`

### Published Interfaces

The current bridge code publishes:

- `/joint_states` as `sensor_msgs/msg/JointState`
- `/robot_state` as `agi_walker_msgs/msg/RobotState` when custom messages are available
- `/battery` as `sensor_msgs/msg/BatteryState`
- `/imu` as `sensor_msgs/msg/Imu`

### Subscribed Interfaces

The current bridge code subscribes to:

- `/cmd_vel` as `geometry_msgs/msg/Twist`

The callback maps `Twist` into Godot parameters:

- `cmd_linear_x`
- `cmd_linear_y`
- `cmd_angular_z`

### Services

The current bridge code exposes:

- `/start_simulation` as `std_srvs/srv/Trigger`
- `/stop_simulation` as `std_srvs/srv/Trigger`
- `/load_robot` as `agi_walker_msgs/srv/LoadRobot` when custom interfaces are available

### Parameters

`bridge_node.py` declares these parameter groups:

- Connection: `godot_host`, `godot_port`, `reconnect_timeout`
- Publish rates: `joint_state_rate`, `robot_state_rate`, `tf_rate`
- Control gains: `motor_power_multiplier`, `joint_stiffness`, `joint_damping`, `balance_gain`
- PID gains: `pid_kp`, `pid_ki`, `pid_kd`

`config/params.yaml` provides defaults for the same bridge node namespace.

## Launch Design

### Intended Launch Entry

`launch/agi_walker.launch.py` is the intended launch file for the current bridge. It:

- Declares `godot_host`, `godot_port`, and `use_sim_time`
- Loads `config/params.yaml`
- Starts a node named `agi_walker_bridge`

### Wrapper Launch Entry

`launch/robot.launch.py` now acts as a wrapper around `agi_walker.launch.py`. It:

- reuses the bridge launch instead of duplicating bridge node setup
- optionally starts `robot_state_publisher` with the packaged URDF
- optionally starts RViz without depending on a missing custom config

## Known Drift

The ROS 2 layer has multiple inconsistencies that should be understood before extension work:

- `tf_rate` is declared, and a `TransformBroadcaster` is created, but there is no active TF publishing timer in the current implementation.
- The bridge still injects the repository root into `sys.path` when running from source.
- The ROS 2 workspace is not covered by the same test depth as the main CLI, Web, and MCP paths.

That said, the bridge is no longer purely text-checked. The repository now includes a fake-runtime regression test:

- `tests/test_ros2_bridge_runtime.py`

It does not require `rclpy` or built ROS 2 message packages. Instead, it verifies bridge replay payload validation, JointState/RobotState publication mapping, `/cmd_vel` parameter translation, and basic service flow using stubbed ROS 2 modules.

The repository now also includes an opt-in live smoke test:

- `tests/test_ros2_bridge_smoke.py`

This test requires a real ROS 2 Python runtime, but still uses a repository-local mock Godot TCP server. It validates:

- bridge startup under real `rclpy`
- `/start_simulation` and `/stop_simulation` service availability
- `/joint_states` publication after simulator telemetry
- `/cmd_vel` forwarding into Godot `update_params`
- structured smoke diagnostics under `test_env/ros2_bridge_smoke/`

## Design Guidance

If this integration is revived, the cleanup order should be:

1. Decide whether TF publishing is required and either implement it or remove the unused parameter.
2. Reduce or remove the source-checkout `sys.path` injection.
3. Keep the new live smoke opt-in and archive its diagnostics in target Humble environments.
4. Expand coverage beyond bridge startup into launch-file level validation and optional TF publication.

## Status

The ROS 2 workspace is now internally more consistent, with both fake-runtime and opt-in live smoke coverage. It is still a secondary integration path, but the remaining gap is now environment execution and artifact collection rather than missing test structure.
