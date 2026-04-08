from pathlib import Path


DOCKERFILE = Path("Dockerfile")
CONTRIBUTING = Path("CONTRIBUTING.md")
URDF_BATCH_CONVERT = Path("agi_walker/skills/urdf-generator/scripts/batch_convert.py")
PARAMETER_BATCH_OPTIMIZE = Path(
    "agi_walker/skills/parameter-optimizer/scripts/batch_optimize.py"
)
QUICK_START_SH = Path("scripts/quick_start.sh")
QUICK_START_BAT = Path("scripts/quick_start.bat")
INSTALL_SH = Path("scripts/install.sh")
INSTALL_BAT = Path("scripts/install.bat")
ROOT_RELEASE_NOTES = Path("RELEASE_NOTES.md")
ROS2_WORKSPACE_README = Path("hardware/ros2_ws/README.md")
CORE_API_README = Path("agi_walker/core/api/README.md")
GODOT_STUDIO_SETUP = Path("godot_studio_agent/setup.bat")
GODOT_STUDIO_MAIN = Path("godot_studio_agent/main.py")


def test_dockerfile_does_not_reference_removed_source_layout() -> None:
    content = DOCKERFILE.read_text(encoding="utf-8")

    assert "COPY requirements.txt ." not in content
    assert "COPY python_api/ ./python_api/" not in content
    assert "COPY python_controller/ ./python_controller/" not in content
    assert "COPY pyproject.toml ." in content
    assert "COPY agi_walker/ ./agi_walker/" in content
    assert "COPY web_panel/ ./web_panel/" in content
    assert "RUN pip install --no-cache-dir ." in content


def test_contributing_uses_current_source_directories() -> None:
    content = CONTRIBUTING.read_text(encoding="utf-8")

    assert "flake8 python_api/ python_controller/" not in content
    assert "black --check python_api/ python_controller/" not in content
    assert "flake8 agi_walker/ web_panel/ tests/" in content
    assert "black --check agi_walker/ web_panel/ tests/" in content


def test_skill_batch_scripts_do_not_use_brittle_repo_root_traversal() -> None:
    urdf_content = URDF_BATCH_CONVERT.read_text(encoding="utf-8")
    optimizer_content = PARAMETER_BATCH_OPTIMIZE.read_text(encoding="utf-8")

    hardcoded_root = "sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))"

    assert hardcoded_root not in urdf_content
    assert hardcoded_root not in optimizer_content
    assert "Path(__file__).resolve().parents" in urdf_content
    assert "Path(__file__).resolve().parents" in optimizer_content


def test_root_scripts_install_from_pyproject_instead_of_missing_requirements_files() -> None:
    quick_start_sh = QUICK_START_SH.read_text(encoding="utf-8")
    quick_start_bat = QUICK_START_BAT.read_text(encoding="utf-8")
    install_sh = INSTALL_SH.read_text(encoding="utf-8")
    install_bat = INSTALL_BAT.read_text(encoding="utf-8")

    assert "requirements.txt" not in install_sh
    assert "requirements.txt" not in install_bat
    assert "requirements-dev.txt" not in quick_start_sh
    assert "requirements.txt" not in quick_start_bat
    assert 'pip install -q -e ".[dev]"' in quick_start_sh
    assert 'python -m pip install -q -e ".[dev]"' in quick_start_bat
    assert "pip install -e ." in install_sh
    assert "python -m pip install -e ." in install_bat
    assert '"pyproject.toml"' in quick_start_sh


def test_active_docs_do_not_reintroduce_removed_root_requirements_or_python_api_layout() -> None:
    release_notes = ROOT_RELEASE_NOTES.read_text(encoding="utf-8")
    ros2_readme = ROS2_WORKSPACE_README.read_text(encoding="utf-8")
    core_api_readme = CORE_API_README.read_text(encoding="utf-8")

    assert "pip install -r requirements.txt" not in release_notes
    assert "pip install -r requirements.txt" not in ros2_readme
    assert "cd python_api" not in core_api_readme
    assert "pip install -r requirements.txt" not in core_api_readme
    assert 'pip install -e ".[dev]"' in release_notes
    assert "pip install -e ." in ros2_readme
    assert "pip install -e ." in core_api_readme
    assert "from agi_walker.core.api.godot_robot_env import GodotRobotEnv" in core_api_readme


def test_godot_studio_agent_setup_targets_real_cli_entrypoint() -> None:
    setup_content = GODOT_STUDIO_SETUP.read_text(encoding="utf-8")
    main_content = GODOT_STUDIO_MAIN.read_text(encoding="utf-8")

    assert "cd api_server" not in setup_content
    assert "http://localhost:8000/ui" not in setup_content
    assert "python main.py" in setup_content
    assert "from agent_system import GodotStudioRouter" in main_content
    assert "def main() -> int:" in main_content
