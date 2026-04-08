from pathlib import Path


ZENOH_GODOT_BRIDGE = Path("openneuro/cortex/zenoh_godot_bridge.py")
GUI_TEST_GUIDE = Path("tests/GUI_TEST_GUIDE.md")
QUADRUPED_README = Path("examples/quadruped/README.md")
ROBOT_CONFIGURATOR_GUI = Path("tools/robot_configurator_gui.py")
CUSTOM_PARTS_DEMO = Path("examples/custom_parts_demo.py")
ROS2_ROBOT_NODE = Path("agi_walker/core/api/ros2_robot_node.py")
PRECISION_ADJUSTMENT_DEMO = Path("examples/precision_adjustment_demo.py")


def test_openneuro_bridge_uses_current_godot_client_import() -> None:
    content = ZENOH_GODOT_BRIDGE.read_text(encoding="utf-8")

    assert "from python_api.comm.godot_client import GodotSimulationClient" not in content
    assert (
        "from agi_walker.core.api.comm.godot_client import GodotSimulationClient"
        in content
    )


def test_gui_test_guide_uses_current_mock_server_import() -> None:
    content = GUI_TEST_GUIDE.read_text(encoding="utf-8")

    assert "from python_api.godot_client import MockGodotServer" not in content
    assert (
        "from agi_walker.core.api.comm.godot_client import MockGodotServer"
        in content
    )


def test_quadruped_readme_uses_current_gait_controller_import() -> None:
    content = QUADRUPED_README.read_text(encoding="utf-8")

    assert "from python_api.gait_generator import GaitController" not in content
    assert (
        "from agi_walker.core.api.control.gait_generator import GaitController"
        in content
    )


def test_robot_configurator_gui_uses_current_godot_client_import() -> None:
    content = ROBOT_CONFIGURATOR_GUI.read_text(encoding="utf-8")

    assert "from python_api.comm.godot_client import GodotSimulationClient" not in content
    assert (
        "from agi_walker.core.api.comm.godot_client import GodotSimulationClient"
        in content
    )


def test_custom_parts_demo_does_not_reference_removed_python_api_parts_library() -> None:
    content = CUSTOM_PARTS_DEMO.read_text(encoding="utf-8")

    assert "../python_api/parts_library.json" not in content
    assert "pm = PartsManager()" in content


def test_ros2_robot_node_does_not_suggest_removed_python_api_entrypoint() -> None:
    content = ROS2_ROBOT_NODE.read_text(encoding="utf-8")

    assert "python python_api/ros2_robot_node.py" not in content


def test_precision_adjustment_demo_does_not_suggest_removed_python_api_entrypoint() -> None:
    content = PRECISION_ADJUSTMENT_DEMO.read_text(encoding="utf-8")

    assert "python python_api/precision_adjuster.py" not in content
