from pathlib import Path

ROS2_SETUP = Path("hardware/ros2_ws/src/agi_walker_ros2/setup.py")
ROBOT_LAUNCH = Path("hardware/ros2_ws/src/agi_walker_ros2/launch/robot.launch.py")
BRIDGE_NODE = Path(
    "hardware/ros2_ws/src/agi_walker_ros2/agi_walker_ros2/bridge_node.py"
)


def test_ros2_setup_exports_bridge_node_and_packages_config() -> None:
    content = ROS2_SETUP.read_text(encoding="utf-8")

    assert "bridge_node = agi_walker_ros2.bridge_node:main" in content
    assert 'glob("config/*.yaml")' in content
    assert "robot_node = agi_walker_ros2.robot_node:main" not in content
    assert "state_publisher = agi_walker_ros2.state_publisher:main" not in content
    assert "cmd_relay = agi_walker_ros2.cmd_relay:main" not in content


def test_robot_launch_wraps_bridge_launch_without_stale_runtime_patterns() -> None:
    content = ROBOT_LAUNCH.read_text(encoding="utf-8")

    assert "IncludeLaunchDescription" in content
    assert "agi_walker.launch.py" in content
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
