"""
AGI-Walker Skills System Tests
Converts the old script-style tests to proper pytest functions.
"""

import pytest
import sys
import os
from pathlib import Path
import importlib.util
import shutil

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.skills_loader import get_skills_loader


@pytest.fixture(scope="module")
def skills_loader():
    """Fixture to provide a loaded skills loader"""
    return get_skills_loader()


@pytest.fixture(scope="module")
def robot_modeling_skill(skills_loader):
    """Fixture to load the robot-modeling skill module"""
    skill_file = (
        PROJECT_ROOT / "agi_walker" / "skills" / "robot-modeling" / "__init__.py"
    )
    spec = importlib.util.spec_from_file_location("robot_modeling_skill", skill_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def param_opt_skill(skills_loader):
    """Fixture to load the parameter-optimizer skill module"""
    pytest.importorskip("scipy")
    skill_file = (
        PROJECT_ROOT / "agi_walker" / "skills" / "parameter-optimizer" / "__init__.py"
    )
    spec = importlib.util.spec_from_file_location(
        "parameter_optimizer_skill", skill_file
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def urdf_gen_skill(skills_loader):
    """Fixture to load the urdf-generator skill module"""
    skill_file = (
        PROJECT_ROOT / "agi_walker" / "skills" / "urdf-generator" / "__init__.py"
    )
    spec = importlib.util.spec_from_file_location("urdf_generator_skill", skill_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestSkillsLoader:
    def test_loader_initialization(self, skills_loader):
        assert len(skills_loader) >= 3

    def test_list_skills(self, skills_loader):
        skills = skills_loader.get_skills_list()
        assert len(skills) >= 3

    def test_get_skill(self, skills_loader):
        skill = skills_loader.get_skill("robot-modeling")
        assert skill is not None
        assert skill.name == "robot-modeling"

    def test_search_skills(self, skills_loader):
        # Assuming "机器人" description exists in some skill
        results = skills_loader.search_skills("机器人")
        assert len(results) > 0

    def test_get_categories(self, skills_loader):
        categories = skills_loader.get_categories()
        assert len(categories) >= 3


class TestRobotModeling:
    def test_builder_basic(self, robot_modeling_skill):
        robot = (
            robot_modeling_skill.RobotBuilder("test_builder")
            .add_torso(height=0.5, mass=5.0)
            .build()
        )
        assert robot.name == "test_builder"
        assert len(robot.parts) > 0

    def test_list_templates(self, robot_modeling_skill):
        templates = robot_modeling_skill.list_templates()
        assert len(templates) >= 2

    def test_load_template(self, robot_modeling_skill):
        template = robot_modeling_skill.load_template("biped_basic")
        assert template.name == "biped_basic"


class TestParamOptimizer:
    def test_optimize_mass(self, robot_modeling_skill, param_opt_skill):
        robot = robot_modeling_skill.load_template("biped_basic")
        result = param_opt_skill.optimize_mass_distribution(
            robot.to_dict(),
            target_com_height=0.22,
            max_iterations=5,  # Reduced iterations for speed
        )
        assert result.success
        assert result.iterations > 0


class TestURDFGenerator:
    @pytest.fixture
    def workspace(self):
        """Temp workspace for file generation"""
        temp_dir = PROJECT_ROOT / "tests" / "temp_output"
        temp_dir.mkdir(exist_ok=True)
        yield temp_dir
        shutil.rmtree(temp_dir)

    def test_urdf_generation(self, robot_modeling_skill, urdf_gen_skill, workspace):
        robot = robot_modeling_skill.load_template("biped_basic")

        test_json = workspace / "test_config.json"
        test_urdf = workspace / "test_output.urdf"

        robot.save(str(test_json))

        urdf_gen_skill.convert_to_urdf(str(test_json), str(test_urdf))

        assert test_urdf.exists()
        assert test_urdf.stat().st_size > 0

    def test_urdf_validation(self, robot_modeling_skill, urdf_gen_skill, workspace):
        robot = robot_modeling_skill.load_template("biped_basic")
        test_json = workspace / "test_val.json"
        test_urdf = workspace / "test_val.urdf"

        robot.save(str(test_json))
        urdf_gen_skill.convert_to_urdf(str(test_json), str(test_urdf))

        is_valid = urdf_gen_skill.validate_urdf(str(test_urdf))
        assert is_valid


@pytest.mark.integration
def test_full_workflow(robot_modeling_skill, param_opt_skill, urdf_gen_skill):
    # Temp file paths
    # Using a known location or tempdir
    test_config = PROJECT_ROOT / "configs" / "workflow_test_artifact.json"
    test_urdf = PROJECT_ROOT / "exports" / "workflow_test_artifact.urdf"

    try:
        # 1. Model
        robot = (
            robot_modeling_skill.RobotBuilder("workflow_test")
            .add_torso(height=0.5, mass=5.0)
            .add_leg_pair(thigh_length=0.3, shin_length=0.3)
            .build()
        )

        # 2. Optimize
        result = param_opt_skill.optimize_mass_distribution(
            robot.to_dict(), target_com_height=0.22, max_iterations=5
        )
        assert result.success

        # 3. Save
        robot.save(str(test_config))
        assert test_config.exists()

        # 4. Convert
        urdf_gen_skill.convert_to_urdf(str(test_config), str(test_urdf))
        assert test_urdf.exists()

    finally:
        # Cleanup
        if test_config.exists():
            test_config.unlink()
        if test_urdf.exists():
            test_urdf.unlink()


@pytest.mark.integration
def test_cli_tools():
    """Test CLI commands"""
    import subprocess

    # helper to run cli command
    def run_cli(*args):
        env = os.environ.copy()
        env["PYTHONIOENCODING"] = "utf-8"
        result = subprocess.run(
            [sys.executable, "-m", "agi_walker.cli", "skills"] + list(args),
            capture_output=True,
            text=True,
            encoding="utf-8",
            cwd=PROJECT_ROOT,
            env=env,
        )
        if result.returncode != 0:
            print(f"CLI Error: {result.stderr}")
        return result

    # 1. list
    result = run_cli("list")
    assert result.returncode == 0
    assert "Skills" in result.stdout

    # 2. info
    result = run_cli("info", "robot-modeling")
    assert result.returncode == 0
    assert "robot-modeling" in result.stdout

    # 3. search
    result = run_cli("search", "机器人")
    assert result.returncode == 0
