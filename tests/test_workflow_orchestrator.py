import json
import shutil
from datetime import datetime
from pathlib import Path
from uuid import uuid4

from agi_walker.core.api.workflow_contracts import (
    WORKFLOW_CONTRACT_VERSION,
    validate_export_result,
    validate_optimization_result,
    validate_robot_config,
    validate_workflow_step_artifact,
)
from agi_walker.workflow_orchestrator import (
    StepStatus,
    WorkflowOrchestrator,
    WorkflowResult,
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
        output_root = tmp_path.resolve()
        orchestrator = _create_orchestrator_with_workflow(
            "robot_creation_pipeline_smoke_real",
            _build_robot_creation_steps(output_root),
            "Real smoke test for robot creation workflow",
        )

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline_smoke_real",
            parameters={
                "execution_strategy": "force",
                "output_root": str(output_root),
            },
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
        assert all(
            Path(step.artifact_path).resolve().is_relative_to(output_root)
            for step in result.steps
            if step.artifact_path
        )
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
        output_root = tmp_path.resolve()
        orchestrator = _create_orchestrator_with_workflow(
            "simulation_ready_robot_smoke_real",
            _build_simulation_ready_steps(output_root),
            "Real smoke test for simulation-ready workflow",
        )

        result = orchestrator.execute_workflow(
            "simulation_ready_robot_smoke_real",
            parameters={
                "execution_strategy": "force",
                "output_root": str(output_root),
            },
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
        assert all(
            Path(step.artifact_path).resolve().is_relative_to(output_root)
            for step in result.steps
            if step.artifact_path
        )
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_workflow_result_to_dict_preserves_log_path() -> None:
    result = WorkflowResult(
        workflow_name="robot_creation_pipeline",
        status=WorkflowStatus.COMPLETED,
        start_time=datetime(2026, 4, 8, 10, 0, 0),
        end_time=datetime(2026, 4, 8, 10, 0, 2),
        log_path="test_env/workflow_log.json",
    )

    payload = result.to_dict()

    assert payload["log_path"] == "test_env/workflow_log.json"


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


def test_real_workflow_artifacts_follow_phase_one_contract() -> None:
    tmp_path = _make_test_dir("robot_creation_artifact_contract")
    try:
        workflow_output_root = tmp_path.resolve()
        orchestrator = _create_orchestrator_with_workflow(
            "robot_creation_pipeline_artifact_contract",
            _build_robot_creation_steps(workflow_output_root),
            "Artifact contract smoke test for robot creation workflow",
        )

        result = orchestrator.execute_workflow(
            "robot_creation_pipeline_artifact_contract",
            parameters={
                "execution_strategy": "force",
                "output_root": str(workflow_output_root),
            },
            use_real=True,
        )

        assert result.status == WorkflowStatus.COMPLETED
        assert result.to_dict()["schema_version"] == WORKFLOW_CONTRACT_VERSION
        assert len(result.to_dict()["artifacts"]) == 3

        artifact_payloads = []
        for step in result.steps:
            assert step.artifact_path is not None
            artifact_path = Path(step.artifact_path)
            assert artifact_path.exists()
            payload = json.loads(artifact_path.read_text(encoding="utf-8"))
            assert validate_workflow_step_artifact(payload) == []
            assert payload["workflow"] == "robot_creation_pipeline_artifact_contract"
            assert payload["mode"] == "real"
            artifact_payloads.append(payload)

        created_robot = json.loads(
            (workflow_output_root / "created_robot.json").read_text(encoding="utf-8")
        )
        optimized_robot = json.loads(
            (workflow_output_root / "optimized_robot.json").read_text(encoding="utf-8")
        )

        assert validate_robot_config(created_robot) == []
        assert validate_optimization_result(optimized_robot) == []
        assert validate_export_result(artifact_payloads[-1]["output"]) == []
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_validate_workflow_rejects_invalid_definitions() -> None:
    orchestrator = WorkflowOrchestrator()
    assert orchestrator.validate_workflow("robot_creation_pipeline") == (True, "Valid")

    assert orchestrator.create_custom_workflow(
        "invalid_workflow",
        [
            {
                "name": "broken",
                "skill_executor": "robot_modeling",
                "action": "unsupported_action",
                "inputs": {"robot_config": "{future.output_file}"},
            }
        ],
        description="Invalid workflow contract test",
    )

    is_valid, message = orchestrator.validate_workflow("invalid_workflow")

    assert is_valid is False
    assert "is not supported" in message
    assert "missing or future step" in message


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
