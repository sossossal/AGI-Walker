import shutil
from pathlib import Path
from uuid import uuid4

from agi_walker.workflow_orchestrator import (
    StepStatus,
    WorkflowOrchestrator,
    WorkflowStatus,
)


def _build_robot_creation_steps(tmp_path: Path) -> list[dict]:
    created_robot = tmp_path / "created_robot.json"
    optimized_robot = tmp_path / "optimized_robot.json"
    exported_robot = tmp_path / "robot.urdf"

    return [
        {
            "name": "create_model",
            "skill_executor": "robot_modeling",
            "action": "create_from_template",
            "inputs": {
                "template": "biped_basic",
                "output_file": str(created_robot),
            },
        },
        {
            "name": "optimize_params",
            "skill_executor": "parameter_optimizer",
            "action": "optimize_mass_distribution",
            "inputs": {
                "robot_config": "{create_model.output_file}",
                "output_file": str(optimized_robot),
                "target_com_height": 0.4,
            },
        },
        {
            "name": "export_urdf",
            "skill_executor": "urdf_generator",
            "action": "export_to_format",
            "inputs": {
                "robot_config": "{optimize_params.output_file}",
                "output_format": "urdf",
                "output_file": str(exported_robot),
            },
        },
    ]


def _build_simulation_ready_steps(tmp_path: Path) -> list[dict]:
    sim_robot = tmp_path / "sim_robot.json"
    validated_robot = tmp_path / "validated_robot.json"
    exported_robot = tmp_path / "robot_sim.sdf"

    return [
        {
            "name": "load_model",
            "skill_executor": "robot_modeling",
            "action": "load_config",
            "inputs": {
                "config_file": "configs/tutorial_01_biped.json",
                "output_file": str(sim_robot),
            },
        },
        {
            "name": "validate_physics",
            "skill_executor": "parameter_optimizer",
            "action": "validate_physics",
            "inputs": {
                "robot_config": "{load_model.output_file}",
                "output_file": str(validated_robot),
            },
        },
        {
            "name": "export_for_sim",
            "skill_executor": "urdf_generator",
            "action": "export_to_format",
            "inputs": {
                "robot_config": "{validate_physics.output_file}",
                "output_format": "sdf",
                "output_file": str(exported_robot),
            },
        },
    ]


def _create_orchestrator_with_workflow(
    workflow_name: str, steps: list[dict], description: str
) -> WorkflowOrchestrator:
    orchestrator = WorkflowOrchestrator()
    assert orchestrator.create_custom_workflow(
        workflow_name, steps, description=description
    )
    return orchestrator


def _make_test_dir(prefix: str) -> Path:
    test_dir = Path("test_env") / "workflow_smoke" / f"{prefix}_{uuid4().hex}"
    test_dir.mkdir(parents=True, exist_ok=False)
    return test_dir


def test_robot_creation_pipeline_completes_with_mock_executors() -> None:
    tmp_path = _make_test_dir("robot_creation_mock")
    try:
        orchestrator = _create_orchestrator_with_workflow(
            "robot_creation_pipeline_smoke_mock",
            _build_robot_creation_steps(tmp_path),
            "Mock smoke test for robot creation workflow",
        )

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline_smoke_mock",
            parameters={"execution_strategy": "force"},
            use_real=False,
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert len(result.steps) == 3
        assert all(step.status == StepStatus.COMPLETED for step in result.steps)
        assert result.completed_steps == 3
        assert result.skipped_steps == 0
        assert result.failed_steps == 0
        assert result.success_rate == 100.0
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_robot_creation_pipeline_completes_with_real_executors() -> None:
    tmp_path = _make_test_dir("robot_creation_real")
    try:
        orchestrator = _create_orchestrator_with_workflow(
            "robot_creation_pipeline_smoke_real",
            _build_robot_creation_steps(tmp_path),
            "Real smoke test for robot creation workflow",
        )

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline_smoke_real",
            parameters={"execution_strategy": "force"},
            use_real=True,
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert len(result.steps) == 3
        assert all(step.status == StepStatus.COMPLETED for step in result.steps)
        assert result.completed_steps == 3
        assert result.skipped_steps == 0
        assert result.failed_steps == 0
        assert result.success_rate == 100.0
        assert (tmp_path / "created_robot.json").exists()
        assert (tmp_path / "optimized_robot.json").exists()
        assert (tmp_path / "robot.urdf").exists()
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_simulation_ready_robot_completes_with_mock_executors() -> None:
    tmp_path = _make_test_dir("simulation_ready_mock")
    try:
        orchestrator = _create_orchestrator_with_workflow(
            "simulation_ready_robot_smoke_mock",
            _build_simulation_ready_steps(tmp_path),
            "Mock smoke test for simulation-ready workflow",
        )

        result = orchestrator.execute_workflow(
            "simulation_ready_robot_smoke_mock",
            parameters={"execution_strategy": "force"},
            use_real=False,
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert len(result.steps) == 3
        assert all(step.status == StepStatus.COMPLETED for step in result.steps)
        assert result.completed_steps == 3
        assert result.skipped_steps == 0
        assert result.failed_steps == 0
        assert result.success_rate == 100.0
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_simulation_ready_robot_completes_with_real_executors() -> None:
    tmp_path = _make_test_dir("simulation_ready_real")
    try:
        orchestrator = _create_orchestrator_with_workflow(
            "simulation_ready_robot_smoke_real",
            _build_simulation_ready_steps(tmp_path),
            "Real smoke test for simulation-ready workflow",
        )

        result = orchestrator.execute_workflow(
            "simulation_ready_robot_smoke_real",
            parameters={"execution_strategy": "force"},
            use_real=True,
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert len(result.steps) == 3
        assert all(step.status == StepStatus.COMPLETED for step in result.steps)
        assert result.completed_steps == 3
        assert result.skipped_steps == 0
        assert result.failed_steps == 0
        assert result.success_rate == 100.0
        assert (tmp_path / "sim_robot.json").exists()
        assert (tmp_path / "validated_robot.json").exists()
        assert (tmp_path / "robot_sim.sdf").exists()
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_skipped_steps_are_counted_as_success() -> None:
    tmp_path = _make_test_dir("robot_creation_skipped")
    try:
        created_robot = tmp_path / "created_robot.json"
        optimized_robot = tmp_path / "optimized_robot.json"
        exported_robot = tmp_path / "robot.urdf"

        created_robot.write_text('{"template": "biped_basic"}', encoding="utf-8")
        optimized_robot.write_text('{"robot_name": "smoke_bot"}', encoding="utf-8")
        exported_robot.write_text("<robot name='smoke_bot' />", encoding="utf-8")

        orchestrator = _create_orchestrator_with_workflow(
            "robot_creation_pipeline_smoke_skipped",
            _build_robot_creation_steps(tmp_path),
            "Skip smoke test for robot creation workflow",
        )

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline_smoke_skipped",
            use_real=False,
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert len(result.steps) == 3
        assert all(step.status == StepStatus.SKIPPED for step in result.steps)
        assert result.completed_steps == 0
        assert result.skipped_steps == 3
        assert result.failed_steps == 0
        assert result.successful_steps == 3
        assert result.success_rate == 100.0
        assert all(step.output.get("skipped") is True for step in result.steps)
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_force_strategy_ignores_existing_outputs() -> None:
    tmp_path = _make_test_dir("robot_creation_force")
    try:
        created_robot = tmp_path / "created_robot.json"
        optimized_robot = tmp_path / "optimized_robot.json"
        exported_robot = tmp_path / "robot.urdf"

        created_robot.write_text('{"template": "stale"}', encoding="utf-8")
        optimized_robot.write_text('{"robot_name": "stale_bot"}', encoding="utf-8")
        exported_robot.write_text("<robot name='stale_bot' />", encoding="utf-8")

        orchestrator = _create_orchestrator_with_workflow(
            "robot_creation_pipeline_smoke_force",
            _build_robot_creation_steps(tmp_path),
            "Force smoke test for robot creation workflow",
        )

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline_smoke_force",
            parameters={"execution_strategy": "force"},
            use_real=False,
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert all(step.status == StepStatus.COMPLETED for step in result.steps)
        assert result.completed_steps == 3
        assert result.skipped_steps == 0
        assert result.success_rate == 100.0
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_output_root_rewrites_relative_outputs_for_built_in_workflow() -> None:
    tmp_path = _make_test_dir("robot_creation_output_root")
    try:
        orchestrator = WorkflowOrchestrator()

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline",
            parameters={
                "execution_strategy": "force",
                "output_root": str(tmp_path),
            },
            use_real=True,
        )

        created_robot = tmp_path / ".output" / "created_robot.json"
        optimized_robot = tmp_path / ".output" / "optimized_robot.json"
        exported_robot = tmp_path / "exports" / "robot.urdf"

        assert result.status == WorkflowStatus.COMPLETED
        assert created_robot.exists()
        assert optimized_robot.exists()
        assert exported_robot.exists()
        assert result.steps[0].output["output_file"] == str(created_robot)
        assert result.steps[1].output["output_file"] == str(optimized_robot)
        assert result.steps[2].output["output_file"] == str(exported_robot)
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_invalid_execution_strategy_returns_failed_result() -> None:
    orchestrator = WorkflowOrchestrator()

    result = orchestrator.execute_workflow(
        "robot_creation_pipeline",
        parameters={"execution_strategy": "invalid-strategy"},
        use_real=False,
    )

    assert result.status == WorkflowStatus.FAILED
    assert "Invalid execution_strategy" in (result.error_message or "")


def test_progress_callback_emits_step_level_updates() -> None:
    tmp_path = _make_test_dir("robot_creation_progress_callback")
    events = []
    try:
        orchestrator = _create_orchestrator_with_workflow(
            "robot_creation_pipeline_progress_callback",
            _build_robot_creation_steps(tmp_path),
            "Progress callback smoke test for robot creation workflow",
        )

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline_progress_callback",
            parameters={"execution_strategy": "force"},
            use_real=False,
            progress_callback=lambda payload: events.append(payload),
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert [event["event"] for event in events] == [
            "workflow_started",
            "step_started",
            "step_finished",
            "step_started",
            "step_finished",
            "step_started",
            "step_finished",
            "workflow_finished",
        ]
        assert events[0]["total_steps"] == 3
        assert events[1]["current_step"]["name"] == "create_model"
        assert events[1]["current_step"]["status"] == "running"
        assert events[2]["current_step"]["status"] == "completed"
        assert events[2]["completed_steps"] == 1
        assert events[-1]["completed_steps"] == 3
        assert events[-1]["workflow_status"] == "completed"
        assert len(events[-1]["steps"]) == 3
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)
