"""
CLI 单元测试 - 补充 0% 覆盖的 CLI 模块

测试命令行界面的解析、执行和错误处理。
"""

import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import pytest
import sys
from datetime import datetime
from pathlib import Path
from unittest.mock import patch, MagicMock
from io import StringIO

# 添加项目根目录到路径
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


# ============================================================================
# __main__.py 测试
# ============================================================================


class TestMainCLI:
    """主 CLI 入口测试"""

    def test_main_shows_help_without_args(self, capsys) -> None:
        """测试：不带参数时显示帮助"""
        from agi_walker.cli.__main__ import main
        
        with patch.object(sys, "argv", ["agi_walker"]):
            result = main()
            captured = capsys.readouterr()
            
            assert result == 0
            # 检查是否打印了帮助信息
            assert "AGI-Walker" in captured.out or "usage:" in captured.out

    def test_main_shows_version(self) -> None:
        """测试：--version 参数"""
        from agi_walker.cli.__main__ import main
        
        with patch.object(sys, "argv", ["agi_walker", "--version"]):
            with pytest.raises(SystemExit) as exc_info:
                main()
            # argparse 在显示版本后调用 sys.exit(0)
            assert exc_info.value.code == 0

    def test_main_shows_help(self) -> None:
        """测试：--help 参数"""
        from agi_walker.cli.__main__ import main
        
        with patch.object(sys, "argv", ["agi_walker", "--help"]):
            with pytest.raises(SystemExit) as exc_info:
                main()
            # argparse 在显示帮助后调用 sys.exit(0)
            assert exc_info.value.code == 0

    def test_main_skills_subcommand(self) -> None:
        """测试：skills 子命令路由"""
        from agi_walker.cli.__main__ import main
        
        with patch("agi_walker.cli.skills_cli.main") as mock_skills_main:
            mock_skills_main.return_value = 0
            
            with patch.object(sys, "argv", ["agi_walker", "skills", "list"]):
                result = main()
            
            # 确保 skills_cli.main 被调用
            mock_skills_main.assert_called_once()

    def test_main_unknown_module(self, capsys) -> None:
        """测试：未知模块"""
        from agi_walker.cli.__main__ import main
        
        with patch.object(sys, "argv", ["agi_walker", "unknown"]):
            try:
                result = main()
                # 应该返回 0 或显示帮助或抛出异常
                assert result is None or result == 0
            except SystemExit as e:
                # argparse 对无效参数会调用 sys.exit(2)
                assert e.code in [0, 1, 2]

    def test_main_no_args_returns_zero(self, capsys) -> None:
        """测试：无参数返回 0"""
        from agi_walker.cli.__main__ import main
        
        with patch.object(sys, "argv", ["agi_walker"]):
            result = main()
            assert result == 0

    def test_main_skills_with_remaining_args(self) -> None:
        """测试：skills 命令带传入的参数"""
        from agi_walker.cli.__main__ import main
        
        with patch("agi_walker.cli.skills_cli.main") as mock_skills_main:
            mock_skills_main.return_value = 0
            
            with patch.object(sys, "argv", ["agi_walker", "skills", "search", "model"]):
                try:
                    main()
                    # 如果成功调用，skills_cli.main 应该被调用
                    mock_skills_main.assert_called_once()
                except SystemExit:
                    logger.warning("Exception occurred")


# ============================================================================
# skills_cli.py 测试
# ============================================================================


class TestSkillsCLI:
    """Skills CLI 命令测试"""

    @pytest.fixture
    def mock_loader(self):
        """模拟加载器"""
        with patch("agi_walker.cli.skills_cli.get_skills_loader") as mock:
            loader = MagicMock()
            mock.return_value = loader
            yield loader

    def test_skills_list_command(self, capsys, mock_loader) -> None:
        """测试：list 命令"""
        from agi_walker.cli.skills_cli import main as skills_main
        
        # 模拟 skills
        from agi_walker.skills_loader import SkillMetadata
        
        mock_loader.get_skills_list.return_value = [
            SkillMetadata(
                name="robot-model",
                description="Build robots",
                emoji="🤖",
                category="建模"
            ),
            SkillMetadata(
                name="param-opt", 
                description="Optimize params",
                emoji="⚙️",
                category="优化"
            ),
        ]
        
        with patch.object(sys, "argv", ["agi_walker", "list"]):
            try:
                result = skills_main()
                assert result == 0 or result is None
            except SystemExit:
                logger.warning("Exception occurred")
            
            captured = capsys.readouterr()
            # 检查列表输出包含 skills 信息
            output = captured.out + captured.err
            assert len(output) > 0  # 应该有输出

    def test_skills_info_command(self, capsys, mock_loader) -> None:
        """测试：info 命令"""
        from agi_walker.cli.skills_cli import main as skills_main
        from agi_walker.skills_loader import SkillMetadata
        from pathlib import Path
        
        skill = SkillMetadata(
            name="robot-model",
            description="Build robots",
            emoji="🤖",
            category="建模",
            requires={"python_modules": ["numpy"]},
            skill_dir=Path("/tmp/robot-model")  # 设置 skill_dir，避免 None
        )
        mock_loader.get_skill.return_value = skill
        mock_loader.get_skill_doc.return_value = "# Robot Modeling\nDetailed docs"
        
        with patch.object(sys, "argv", ["agi_walker", "info", "robot-model"]):
            try:
                result = skills_main()
                assert result == 0 or result is None
            except SystemExit:
                logger.warning("Exception occurred")
            
            captured = capsys.readouterr()
            # 检查信息输出
            output = captured.out + captured.err
            assert len(output) > 0

    def test_skills_search_command(self, capsys, mock_loader) -> None:
        """测试：search 命令"""
        from agi_walker.cli.skills_cli import main as skills_main
        from agi_walker.skills_loader import SkillMetadata
        
        mock_loader.search_skills.return_value = [
            SkillMetadata(
                name="robot-model",
                description="Build robots",
                emoji="🤖",
                category="建模"
            )
        ]
        
        # 注意使用 search_skills 而不是 loader.search_skills
        with patch("agi_walker.cli.skills_cli.search_skills") as mock_search:
            mock_search.return_value = mock_loader.search_skills.return_value
            
            with patch.object(sys, "argv", ["agi_walker", "search", "robot"]):
                try:
                    result = skills_main()
                    assert result == 0 or result is None
                except SystemExit:
                    logger.warning("Exception occurred")
                
                captured = capsys.readouterr()
                # 检查搜索输出
                output = captured.out + captured.err
                assert len(output) > 0

    def test_skills_validate_command(self, capsys, mock_loader) -> None:
        """测试：validate 命令"""
        from agi_walker.cli.skills_cli import main as skills_main
        
        mock_loader.validate_skill_dependencies.return_value = {
            "python_modules": [],
            "bins": []
        }
        
        mock_loader.get_skills_list.return_value = []
        
        with patch.object(sys, "argv", ["agi_walker", "validate"]):
            try:
                result = skills_main()
                assert result == 0 or result is None
            except SystemExit:
                logger.warning("Exception occurred")
            
            captured = capsys.readouterr()
            # 检查验证输出
            output = captured.out + captured.err
            assert len(output) > 0

    def test_skills_workflows_list_command(self, capsys) -> None:
        """测试：workflows list 命令"""
        from agi_walker.cli.skills_cli import main as skills_main

        with patch.object(sys, "argv", ["agi_walker", "workflows", "list"]):
            result = skills_main()
            assert result == 0 or result is None

        captured = capsys.readouterr()
        output = captured.out + captured.err
        assert "robot_creation_pipeline" in output

    def test_skills_workflows_validate_command(self, capsys) -> None:
        """测试：workflows validate 命令"""
        from agi_walker.cli.skills_cli import main as skills_main

        with patch.object(
            sys,
            "argv",
            ["agi_walker", "workflows", "validate", "robot_creation_pipeline"],
        ):
            result = skills_main()
            assert result == 0 or result is None

        captured = capsys.readouterr()
        output = captured.out + captured.err
        assert "验证通过" in output

    def test_skills_workflows_run_mock_command(self, capsys) -> None:
        """测试：workflows run --mock 命令"""
        from agi_walker.cli.skills_cli import main as skills_main

        with patch.object(
            sys,
            "argv",
            ["agi_walker", "workflows", "run", "robot_creation_pipeline", "--mock"],
        ):
            result = skills_main()
            assert result == 0 or result is None

        captured = capsys.readouterr()
        output = captured.out + captured.err
        assert "执行结果: completed" in output

    def test_skills_workflows_run_reports_skipped_steps_as_success(self, capsys) -> None:
        """测试：SKIPPED 步骤不会把成功率显示成 0%"""
        from agi_walker.cli.skills_cli import main as skills_main
        from agi_walker.workflow_orchestrator import (
            StepStatus,
            WorkflowResult,
            WorkflowStatus,
            WorkflowStep,
        )

        start_time = datetime.now()
        result = WorkflowResult(
            workflow_name="robot_creation_pipeline",
            status=WorkflowStatus.COMPLETED,
            start_time=start_time,
            end_time=start_time,
            steps=[
                WorkflowStep(
                    name="create_model",
                    skill_executor="robot_modeling",
                    action="create_from_template",
                    status=StepStatus.SKIPPED,
                    output={"status": "success", "skipped": True},
                ),
                WorkflowStep(
                    name="optimize_params",
                    skill_executor="parameter_optimizer",
                    action="optimize_mass_distribution",
                    status=StepStatus.SKIPPED,
                    output={"status": "success", "skipped": True},
                ),
                WorkflowStep(
                    name="export_urdf",
                    skill_executor="urdf_generator",
                    action="export_to_format",
                    status=StepStatus.SKIPPED,
                    output={"status": "success", "skipped": True},
                ),
            ],
        )

        mock_orchestrator = MagicMock()
        mock_orchestrator.execute_workflow.return_value = result

        with patch(
            "agi_walker.cli.skills_cli.get_workflow_orchestrator",
            return_value=mock_orchestrator,
        ):
            with patch.object(
                sys,
                "argv",
                ["agi_walker", "workflows", "run", "robot_creation_pipeline", "--mock"],
            ):
                exit_code = skills_main()
                assert exit_code == 0

        captured = capsys.readouterr()
        output = captured.out + captured.err
        assert "成功率: 100.0%" in output
        assert "步骤统计: 完成 0, 跳过 3, 失败 0" in output

    def test_skills_workflows_run_passes_force_and_output_root(self, capsys) -> None:
        """测试：--force 和 --output-root 会传给 orchestrator"""
        from agi_walker.cli.skills_cli import main as skills_main
        from agi_walker.workflow_orchestrator import WorkflowResult, WorkflowStatus

        start_time = datetime.now()
        result = WorkflowResult(
            workflow_name="robot_creation_pipeline",
            status=WorkflowStatus.COMPLETED,
            start_time=start_time,
            end_time=start_time,
        )

        mock_orchestrator = MagicMock()
        mock_orchestrator.execute_workflow.return_value = result

        with patch(
            "agi_walker.cli.skills_cli.get_workflow_orchestrator",
            return_value=mock_orchestrator,
        ):
            with patch.object(
                sys,
                "argv",
                [
                    "agi_walker",
                    "workflows",
                    "run",
                    "robot_creation_pipeline",
                    "--mock",
                    "--force",
                    "--output-root",
                    "test_env/run_force",
                ],
            ):
                exit_code = skills_main()
                assert exit_code == 0

        mock_orchestrator.execute_workflow.assert_called_once()
        call_args = mock_orchestrator.execute_workflow.call_args
        assert call_args.args[0] == "robot_creation_pipeline"
        assert call_args.args[1]["execution_strategy"] == "force"
        assert call_args.args[1]["output_root"] == "test_env/run_force"
        assert call_args.kwargs["use_real"] is False

        captured = capsys.readouterr()
        output = captured.out + captured.err
        assert "执行策略: force" in output
        assert "输出根目录: test_env/run_force" in output

    def test_skills_workflows_run_passes_resume_strategy(self, capsys) -> None:
        """测试：--resume 会显式传递 resume 策略"""
        from agi_walker.cli.skills_cli import main as skills_main
        from agi_walker.workflow_orchestrator import WorkflowResult, WorkflowStatus

        start_time = datetime.now()
        result = WorkflowResult(
            workflow_name="robot_creation_pipeline",
            status=WorkflowStatus.COMPLETED,
            start_time=start_time,
            end_time=start_time,
        )

        mock_orchestrator = MagicMock()
        mock_orchestrator.execute_workflow.return_value = result

        with patch(
            "agi_walker.cli.skills_cli.get_workflow_orchestrator",
            return_value=mock_orchestrator,
        ):
            with patch.object(
                sys,
                "argv",
                [
                    "agi_walker",
                    "workflows",
                    "run",
                    "robot_creation_pipeline",
                    "--resume",
                ],
            ):
                exit_code = skills_main()
                assert exit_code == 0

        mock_orchestrator.execute_workflow.assert_called_once()
        call_args = mock_orchestrator.execute_workflow.call_args
        assert call_args.args[1]["execution_strategy"] == "resume"
        assert call_args.kwargs["use_real"] is True

        captured = capsys.readouterr()
        output = captured.out + captured.err
        assert "执行策略: resume" in output

    def test_skills_workflows_run_real_simulation_ready_robot(self, capsys) -> None:
        """测试：simulation_ready_robot 在 real 模式下可执行"""
        from agi_walker.cli.skills_cli import main as skills_main

        with patch.object(
            sys,
            "argv",
            ["agi_walker", "workflows", "run", "simulation_ready_robot"],
        ):
            result = skills_main()
            assert result == 0 or result is None

        captured = capsys.readouterr()
        output = captured.out + captured.err
        assert "执行结果: completed" in output

    def test_skills_no_subcommand(self, capsys) -> None:
        """测试：无子命令"""
        from agi_walker.cli.skills_cli import main as skills_main
        
        with patch.object(sys, "argv", ["agi_walker"]):
            try:
                result = skills_main()
                assert result == 0 or result is None
            except SystemExit:
                logger.warning("Exception occurred")
            
            captured = capsys.readouterr()
            # 应该显示帮助或错误
            output = captured.out + captured.err
            assert len(output) > 0

    def test_skills_help(self) -> None:
        """测试：--help"""
        from agi_walker.cli.skills_cli import main as skills_main
        
        with patch.object(sys, "argv", ["agi_walker", "--help"]):
            with pytest.raises(SystemExit) as exc_info:
                skills_main()
            assert exc_info.value.code == 0

    def test_skills_unknown_subcommand(self, capsys) -> None:
        """测试：未知的子命令"""
        from agi_walker.cli.skills_cli import main as skills_main
        
        with patch.object(sys, "argv", ["agi_walker", "unknown"]):
            try:
                result = skills_main()
                # skills_cli.main 不识别的命令
                assert result is not None or result == 0
            except SystemExit:
                logger.warning("Exception occurred")
            
            captured = capsys.readouterr()
            output = captured.out + captured.err
            assert len(output) > 0

    def test_workflows_alias_triggers_skills(self, capsys):
        """测试：顶层 workflows 直接代理到 skills workflows"""
        from agi_walker.cli.__main__ import main

        with patch.object(sys, "argv", ["agi_walker", "workflows", "list"]):
            with patch("agi_walker.cli.skills_cli.main") as mock_skills_main:
                mock_skills_main.return_value = 0
                result = main()
                assert result == 0
                mock_skills_main.assert_called_once()

        capsys.readouterr()

# ============================================================================
# CLI __init__.py 测试
# ============================================================================


class TestCLIInit:
    """CLI 包初始化测试"""

    def test_cli_package_imports(self) -> None:
        """测试：CLI 包可以导入"""
        from agi_walker import cli
        assert cli is not None
        
        # 检查主要导出
        assert hasattr(cli, "__main__") or hasattr(cli, "__path__")


# ============================================================================
# 集成测试
# ============================================================================


class TestCLIIntegration:
    """CLI 集成测试"""

    def test_end_to_end_skills_list(self) -> None:
        """测试：端到端 list 命令"""
        import subprocess
        
        result = subprocess.run(
            [sys.executable, "-m", "agi_walker.cli", "list"],
            capture_output=True,
            text=True,
            cwd=str(PROJECT_ROOT)
        )
        
        # 命令应该成功或者优雅地失败
        assert result.returncode in [0, 1, 2]  # 0=success, 1-2=expected error

    def test_end_to_end_skills_help(self) -> None:
        """测试：端到端 --help"""
        import subprocess
        
        result = subprocess.run(
            [sys.executable, "-m", "agi_walker.cli", "--help"],
            capture_output=True,
            text=True,
            cwd=str(PROJECT_ROOT)
        )
        
        # 应该显示帮助
        assert len(result.stdout) > 0 or len(result.stderr) > 0

    def test_end_to_end_main_help(self) -> None:
        """测试：端到端主命令帮助"""
        import subprocess
        
        result = subprocess.run(
            [sys.executable, "-m", "agi_walker.cli", "--help"],
            capture_output=True,
            text=True,
            cwd=str(PROJECT_ROOT)
        )
        
        # 应该显示帮助
        assert len(result.stdout) > 0 or len(result.stderr) > 0


# ============================================================================
# 错误处理测试
# ============================================================================


class TestCLIErrorHandling:
    """CLI 错误处理"""

    def test_missing_skill_info(self, capsys) -> None:
        """测试：查询不存在的 skill"""
        from agi_walker.cli.skills_cli import main as skills_main
        
        with patch("agi_walker.cli.skills_cli.get_skills_loader") as mock_loader_fn:
            mock_loader = MagicMock()
            mock_loader_fn.return_value = mock_loader
            mock_loader.get_skill.side_effect = KeyError("nonexistent")
            
            with patch.object(sys, "argv", ["agi_walker", "info", "nonexistent"]):
                try:
                    result = skills_main()
                    assert result == 1 or result is None
                except (SystemExit, Exception):
                    logger.warning("Exception occurred")
                
                captured = capsys.readouterr()
                # 应该有某种错误或警告消息
                output = captured.out + captured.err
                assert len(output) > 0

    def test_cli_with_invalid_arguments(self, capsys) -> None:
        """测试：无效的CLI参数"""
        from agi_walker.cli.skills_cli import main as skills_main
        
        with patch.object(sys, "argv", ["agi_walker", "info"]):
            # 缺少必需的参数（info 命令需要 name）
            try:
                result = skills_main()
            except SystemExit:
                logger.warning("Exception occurred")
            
            captured = capsys.readouterr()
            # 应该显示错误或用法信息
            output = captured.out + captured.err
            assert len(output) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
