from pathlib import Path


ZENOH_GODOT_BRIDGE = Path("openneuro/cortex/zenoh_godot_bridge.py")
GUI_TEST_GUIDE = Path("tests/GUI_TEST_GUIDE.md")
QUADRUPED_README = Path("examples/quadruped/README.md")
ROBOT_CONFIGURATOR_GUI = Path("tools/robot_configurator_gui.py")


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
