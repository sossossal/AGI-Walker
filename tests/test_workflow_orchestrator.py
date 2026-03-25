from agi_walker.workflow_orchestrator import get_workflow_orchestrator


def test_robot_creation_pipeline_completes_with_mock_executors():
    orchestrator = get_workflow_orchestrator()

    result = orchestrator.execute_workflow("robot_creation_pipeline", use_real=False)

    assert result.status.value == "completed"
    assert len(result.steps) == 3
    assert all(step.status.value == "completed" for step in result.steps)


def test_robot_creation_pipeline_completes_with_real_executors():
    orchestrator = get_workflow_orchestrator()

    result = orchestrator.execute_workflow("robot_creation_pipeline", use_real=True)

    assert result.status.value == "completed"
    assert len(result.steps) == 3
    assert all(step.status.value == "completed" for step in result.steps)


def test_simulation_ready_robot_completes_with_mock_executors():
    orchestrator = get_workflow_orchestrator()

    result = orchestrator.execute_workflow("simulation_ready_robot", use_real=False)

    assert result.status.value == "completed"
    assert len(result.steps) == 3
    assert all(step.status.value == "completed" for step in result.steps)


def test_simulation_ready_robot_completes_with_real_executors():
    orchestrator = get_workflow_orchestrator()

    result = orchestrator.execute_workflow("simulation_ready_robot", use_real=True)

    assert result.status.value == "completed"
    assert len(result.steps) == 3
    assert all(step.status.value == "completed" for step in result.steps)
