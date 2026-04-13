from agi_walker.core.api.workflow_contracts import (
    WORKFLOW_CONTRACT_VERSION,
    build_workflow_step_artifact,
    validate_export_result,
    validate_optimization_result,
    validate_part_spec,
    validate_robot_config,
    validate_workflow_definition,
    validate_workflow_step_artifact,
)


def test_workflow_step_artifact_contract_accepts_canonical_payload() -> None:
    artifact = build_workflow_step_artifact(
        workflow="robot_creation_pipeline",
        step="create_model",
        executor="robot_modeling",
        action="create_from_template",
        status="completed",
        mode="real",
        inputs={"template": "biped_basic"},
        output={"status": "success", "output_file": "robot.json"},
        artifact_index=1,
        attempts=1,
        duration_seconds=0.25,
        created_at="2026-04-11T00:00:00",
    )

    assert artifact["schema_version"] == WORKFLOW_CONTRACT_VERSION
    assert validate_workflow_step_artifact(artifact) == []


def test_workflow_step_artifact_contract_reports_missing_fields() -> None:
    errors = validate_workflow_step_artifact(
        {
            "schema_version": WORKFLOW_CONTRACT_VERSION,
            "artifact_type": "workflow_step",
            "workflow": "robot_creation_pipeline",
        }
    )

    assert any("missing required fields" in error for error in errors)
    assert any("status must be" in error for error in errors)


def test_workflow_definition_contract_rejects_invalid_references_and_actions() -> None:
    errors = validate_workflow_definition(
        "bad_workflow",
        {
            "name": "bad_workflow",
            "steps": [
                {
                    "name": "create_model",
                    "skill_executor": "robot_modeling",
                    "action": "fly",
                    "inputs": {"robot_config": "{future.output_file}"},
                }
            ],
        },
        {"robot_modeling": {"create_from_template", "load_config"}},
    )

    assert any("is not supported" in error for error in errors)
    assert any("missing or future step" in error for error in errors)


def test_robot_config_part_optimization_and_export_contracts() -> None:
    assert (
        validate_robot_config(
            {
                "name": "test_robot",
                "parts": [{"id": "torso_1", "type": "torso", "params": {"mass": 8.0}}],
                "connections": [
                    {
                        "from": "torso_1",
                        "to": "shin_1",
                        "joint_type": "revolute",
                    }
                ],
                "metadata": {},
            }
        )
        == []
    )
    assert (
        validate_part_spec(
            {
                "id": "go_m8010",
                "category": "actuators",
                "name": "Unitree Go-M8010 Style",
                "weight_kg": 0.53,
                "cost_usd": 350.0,
                "specs": {"max_torque_nm": 23.7},
            }
        )
        == []
    )
    assert (
        validate_optimization_result(
            {
                "success": True,
                "iterations": 4,
                "mass_distribution": {"torso_1": 7.5},
                "com_position": [0.0, 0.0, 0.4],
                "com_error": 0.01,
            }
        )
        == []
    )
    assert (
        validate_export_result(
            {
                "status": "success",
                "action": "export_to_format",
                "output_file": "robot.urdf",
                "format": "urdf",
                "file_size": 100,
                "output_generated": True,
            }
        )
        == []
    )
