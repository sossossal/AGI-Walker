from pathlib import Path

ROS2_SETUP = Path("hardware/ros2_ws/src/agi_walker_ros2/setup.py")
BRIDGE_LAUNCH = Path("hardware/ros2_ws/src/agi_walker_ros2/launch/agi_walker.launch.py")
ROBOT_LAUNCH = Path("hardware/ros2_ws/src/agi_walker_ros2/launch/robot.launch.py")
BRIDGE_NODE = Path(
    "hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/bridge_node.py"
)
MSG_PACKAGE_CMAKE = Path("hardware/ros2_ws/src/agi_walker_msgs/CMakeLists.txt")
MSG_DIR = Path("hardware/ros2_ws/src/agi_walker_msgs/msg")
SRV_DIR = Path("hardware/ros2_ws/src/agi_walker_msgs/srv")
ROS2_TYPED_IDL_MIGRATION = Path("docs/ros2/ROS2_TYPED_IDL_MIGRATION.md")


def test_ros2_setup_exports_bridge_node_and_packages_config() -> None:
    content = ROS2_SETUP.read_text(encoding="utf-8")

    assert "bridge_node = agi_walker_ros2.bridge_node:main" in content
    assert 'glob("config/*.yaml")' in content
    assert 'glob("config/profiles/*.yaml")' in content
    assert "robot_node = agi_walker_ros2.robot_node:main" not in content
    assert "state_publisher = agi_walker_ros2.state_publisher:main" not in content
    assert "cmd_relay = agi_walker_ros2.cmd_relay:main" not in content


def test_robot_launch_wraps_bridge_launch_without_stale_runtime_patterns() -> None:
    bridge_content = BRIDGE_LAUNCH.read_text(encoding="utf-8")
    content = ROBOT_LAUNCH.read_text(encoding="utf-8")

    assert '"config_file"' in bridge_content
    assert 'LaunchConfiguration("config_file")' in bridge_content
    assert "IncludeLaunchDescription" in content
    assert "agi_walker.launch.py" in content
    assert '"config_file": config_file' in content
    assert 'executable="robot_state_publisher"' in content
    assert 'executable="rviz2"' in content
    assert 'executable="robot_node"' not in content
    assert "perform(None)" not in content


def test_bridge_node_uses_current_godot_client_import() -> None:
    content = BRIDGE_NODE.read_text(encoding="utf-8")

    assert (
        "from agi_walker.core.api.comm.godot_client import GodotSimulationClient"
        in content
    )
    assert (
        "from python_api.comm.godot_client import GodotSimulationClient" not in content
    )


def test_ros2_custom_interfaces_include_instruction_and_recovery_idl() -> None:
    cmake = MSG_PACKAGE_CMAKE.read_text(encoding="utf-8")
    expected_messages = [
        "InstructionStep.msg",
        "InstructionSet.msg",
        "SimulatedCircuit.msg",
        "HardwareFault.msg",
        "HardwareRecoveryAction.msg",
        "HardwareRecoveryStatus.msg",
        "BehaviorCommand.msg",
        "NavigationGoal.msg",
        "PerceptionSnapshot.msg",
    ]
    expected_services = [
        "ApplyInstructionSet.srv",
        "ConfigureSimulatedCircuit.srv",
        "HardwareRecovery.srv",
    ]

    for filename in expected_messages:
        assert MSG_DIR.joinpath(filename).exists()
        assert f'"msg/{filename}"' in cmake
    for filename in expected_services:
        assert SRV_DIR.joinpath(filename).exists()
        assert f'"srv/{filename}"' in cmake

    instruction_set = MSG_DIR.joinpath("InstructionSet.msg").read_text(
        encoding="utf-8"
    )
    recovery_service = SRV_DIR.joinpath("HardwareRecovery.srv").read_text(
        encoding="utf-8"
    )
    assert "agi_walker_msgs/InstructionStep[] steps" in instruction_set
    assert "agi_walker_msgs/HardwareRecoveryStatus recovery_status" in recovery_service


def test_ros2_launch_profiles_are_packaged() -> None:
    profile_dir = Path("hardware/ros2_ws/src/agi_walker_ros2/config/profiles")
    profiles = {
        path.name: path.read_text(encoding="utf-8")
        for path in profile_dir.glob("*.yaml")
    }

    assert {"local.yaml", "replay.yaml", "live.yaml"}.issubset(profiles)
    assert "debug: true" in profiles["local.yaml"]
    assert "verbose: true" in profiles["replay.yaml"]
    assert "reconnect_timeout: 10.0" in profiles["live.yaml"]


def test_ros2_typed_idl_migration_runbook_covers_legacy_mapping() -> None:
    content = ROS2_TYPED_IDL_MIGRATION.read_text(encoding="utf-8")

    assert "/instruction_set/json" in content
    assert "/instruction_set`" in content
    assert "agi_walker_msgs/msg/InstructionSet" in content
    assert "/simulated_circuit/json" in content
    assert "/simulated_circuit`" in content
    assert "agi_walker_msgs/msg/SimulatedCircuit" in content
    assert "/hardware/recovery_plan" in content
    assert "/hardware/recovery`" in content
    assert "agi_walker_msgs/srv/HardwareRecovery" in content
    assert "Single Writer Cutover" in content
    assert "Deprecation Policy" in content
    assert "Rollback" in content
