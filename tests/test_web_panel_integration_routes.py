import importlib
import json
import os
import shutil
import time
import uuid
from datetime import datetime, timedelta
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

from agi_walker.workflow_orchestrator import (
    StepStatus,
    WorkflowResult,
    WorkflowStatus,
    WorkflowStep,
)
import web_panel.server
import web_panel.workflows_api
from web_panel.auth_api import get_current_user
from web_panel.command_parser import CommandParser
from web_panel.database import Base, engine


def _run_coro(coro):
    return web_panel.workflows_api._run_async(coro)


async def _reset_persistence_state() -> None:
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
        await conn.run_sync(Base.metadata.create_all)


def _store_run_record(record, user_id=None):
    return _run_coro(web_panel.workflows_api._store_run_record(record, user_id=user_id))


def _update_run_record(run_id, **changes):
    return _run_coro(web_panel.workflows_api._update_run_record(run_id, **changes))


def _get_run_record(run_id):
    return _run_coro(web_panel.workflows_api._get_run_record(run_id))


def _finalize_run_from_result(run_id, result_dict):
    return _run_coro(web_panel.workflows_api._finalize_run_from_result(run_id, result_dict))


def _mark_run_terminal(run_id, status, **changes):
    return _run_coro(web_panel.workflows_api._mark_run_terminal(run_id, status, **changes))


@pytest.fixture()
def client() -> TestClient:
    fake_user = SimpleNamespace(
        id=None,
        username="integration-user",
        is_admin=False,
        created_at=datetime(2026, 4, 2, 12, 0, 0),
    )

    async def _override_current_user():
        return fake_user

    web_panel.server.app.dependency_overrides[get_current_user] = _override_current_user
    with TestClient(web_panel.server.app) as test_client:
        yield test_client
    web_panel.server.app.dependency_overrides.pop(get_current_user, None)


@pytest.fixture(autouse=True)
def reset_server_state(monkeypatch: pytest.MonkeyPatch):
    _run_coro(_reset_persistence_state())
    archive_root = Path("test_env") / "web_panel_archives" / uuid.uuid4().hex
    archive_root.mkdir(parents=True, exist_ok=False)
    monkeypatch.setattr(web_panel.workflows_api, "WORKFLOW_ARCHIVE_ROOT", archive_root)
    web_panel.server.tasks_db.clear()
    web_panel.server.active_connections.clear()
    web_panel.workflows_api.execution_history.clear()
    web_panel.workflows_api.active_run_processes.clear()
    web_panel.workflows_api.run_event_history.clear()
    web_panel.workflows_api.run_event_counters.clear()
    yield
    web_panel.server.tasks_db.clear()
    web_panel.server.active_connections.clear()
    web_panel.workflows_api.execution_history.clear()
    web_panel.workflows_api.active_run_processes.clear()
    web_panel.workflows_api.run_event_history.clear()
    web_panel.workflows_api.run_event_counters.clear()
    _run_coro(_reset_persistence_state())
    web_panel.server.app.dependency_overrides.pop(get_current_user, None)
    shutil.rmtree(archive_root, ignore_errors=True)


class FakeWorkflowOrchestrator:
    def __init__(
        self,
        result: WorkflowResult,
        workflows: dict[str, dict] | None = None,
    ) -> None:
        self.result = result
        self.workflows = workflows or {
            "robot_creation_pipeline": {
                "name": "robot_creation_pipeline",
                "description": "Create a robot and export it",
                "steps": [
                    {"name": "create_model"},
                    {"name": "export_urdf"},
                ],
            }
        }
        self.last_execute_call: dict | None = None

    def list_workflows(self) -> list[str]:
        return list(self.workflows)

    def get_workflow(self, name: str) -> dict | None:
        return self.workflows.get(name)

    def execute_workflow(
        self,
        name: str,
        parameters: dict | None = None,
        use_real: bool | None = None,
    ) -> WorkflowResult:
        self.last_execute_call = {
            "name": name,
            "parameters": parameters or {},
            "use_real": use_real,
        }
        return self.result


class FakeProcess:
    def __init__(self, *, pid: int = 9999, exit_code: int = 0) -> None:
        self.pid = pid
        self._exit_code = exit_code
        self._terminated = False
        self._killed = False

    def poll(self) -> int | None:
        if self._terminated or self._killed:
            return self._exit_code
        return None

    def terminate(self) -> None:
        self._terminated = True

    def wait(self, timeout: float | None = None) -> int:
        return self._exit_code

    def kill(self) -> None:
        self._killed = True


def _make_test_dir(prefix: str) -> Path:
    test_dir = Path("test_env") / "web_panel_routes" / f"{prefix}_{uuid.uuid4().hex}"
    test_dir.mkdir(parents=True, exist_ok=False)
    return test_dir


def _build_completed_workflow_result(tmp_path: Path) -> WorkflowResult:
    output_dir = tmp_path / ".output"
    exports_dir = tmp_path / "exports"
    output_dir.mkdir(parents=True, exist_ok=True)
    exports_dir.mkdir(parents=True, exist_ok=True)

    created_robot = output_dir / "created_robot.json"
    robot_urdf = exports_dir / "robot.urdf"
    log_path = output_dir / "workflow_log_robot_creation_pipeline.json"

    created_robot.write_text(
        json.dumps(
            {
                "robot_name": "web_panel_bot",
                "parts": [{"id": "torso", "type": "body"}],
                "connections": [],
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    robot_urdf.write_text("<robot name='web_panel_bot' />", encoding="utf-8")
    log_path.write_text(
        json.dumps({"workflow_name": "robot_creation_pipeline"}, indent=2),
        encoding="utf-8",
    )

    start_time = datetime(2026, 3, 30, 19, 45, 0)
    first_end = start_time + timedelta(seconds=1)
    second_end = start_time + timedelta(seconds=2)

    return WorkflowResult(
        workflow_name="robot_creation_pipeline",
        status=WorkflowStatus.COMPLETED,
        steps=[
            WorkflowStep(
                name="create_model",
                skill_executor="robot_modeling",
                action="create_from_template",
                status=StepStatus.COMPLETED,
                output={"output_file": str(created_robot)},
                start_time=start_time,
                end_time=first_end,
            ),
            WorkflowStep(
                name="export_urdf",
                skill_executor="urdf_generator",
                action="export_to_format",
                status=StepStatus.COMPLETED,
                output={"output_file": str(robot_urdf)},
                start_time=first_end,
                end_time=second_end,
            ),
        ],
        start_time=start_time,
        end_time=second_end,
        log_path=str(log_path),
    )


def _build_failed_workflow_result() -> WorkflowResult:
    start_time = datetime(2026, 3, 30, 19, 50, 0)
    end_time = start_time + timedelta(seconds=2)

    return WorkflowResult(
        workflow_name="robot_creation_pipeline",
        status=WorkflowStatus.FAILED,
        steps=[
            WorkflowStep(
                name="create_model",
                skill_executor="robot_modeling",
                action="create_from_template",
                status=StepStatus.COMPLETED,
                output={"output_file": "test_env/failed/created_robot.json"},
                start_time=start_time,
                end_time=start_time + timedelta(seconds=1),
            ),
            WorkflowStep(
                name="optimize_params",
                skill_executor="parameter_optimizer",
                action="optimize_mass_distribution",
                status=StepStatus.FAILED,
                error="mass distribution optimizer diverged",
                output={"status": "error", "error": "mass distribution optimizer diverged"},
                start_time=start_time + timedelta(seconds=1),
                end_time=end_time,
            ),
        ],
        error_message="Workflow failed at step 'optimize_params': mass distribution optimizer diverged",
        start_time=start_time,
        end_time=end_time,
    )


def _seed_archived_run(
    *,
    workflow_name: str = "robot_creation_pipeline",
    mode: str = "real",
    execution_strategy: str = "force",
    status: str = "completed",
    output_root: str,
    started_at: str,
    finished_at: str,
    completed_steps: int = 2,
    skipped_steps: int = 0,
    failed_steps: int = 0,
    success_rate: float = 100.0,
    status_detail: str | None = None,
    message: str | None = None,
    failed_step_name: str | None = None,
    failed_step_error: str | None = None,
    diagnostic_summary: str | None = None,
) -> str:
    run_id = uuid.uuid4().hex
    request = web_panel.workflows_api.WorkflowRunRequest(
        use_real=mode == "real",
        execution_strategy=execution_strategy,
        parameters={
            "execution_strategy": execution_strategy,
            "output_root": output_root,
        },
    )
    record = web_panel.workflows_api._build_initial_run_record(
        run_id,
        workflow_name,
        request,
        output_root,
    )
    _store_run_record(record)

    started_dt = datetime.fromisoformat(started_at)
    finished_dt = datetime.fromisoformat(finished_at)
    _update_run_record(
        run_id,
        status=status,
        started_at=started_at,
        finished_at=finished_at,
        duration=(finished_dt - started_dt).total_seconds(),
        total_steps=max(2, completed_steps + skipped_steps + failed_steps),
        completed_steps=completed_steps,
        skipped_steps=skipped_steps,
        failed_steps=failed_steps,
        current_step_name=None,
        current_step_index=max(2, completed_steps + skipped_steps + failed_steps),
        progress_updated_at=finished_at,
        success_rate=success_rate,
        status_detail=status_detail or f"Workflow finished with status '{status}'.",
        message=message,
        failed_step_name=failed_step_name,
        failed_step_error=failed_step_error,
        diagnostic_summary=diagnostic_summary,
    )
    return run_id


def test_tasks_crud_round_trip(client: TestClient):
    task_name = f"task-{uuid.uuid4().hex[:8]}"

    create_response = client.post(
        "/api/tasks",
        json={
            "name": task_name,
            "description": "integration test task",
            "priority": "high",
        },
    )
    assert create_response.status_code == 200
    created = create_response.json()
    assert created["status"] == "success"
    task_id = created["task_id"]

    get_response = client.get(f"/api/tasks/{task_id}")
    assert get_response.status_code == 200
    assert get_response.json()["task"]["name"] == task_name

    update_response = client.put(
        f"/api/tasks/{task_id}",
        json={"status": "running", "description": "updated"},
    )
    assert update_response.status_code == 200
    updated = update_response.json()
    assert updated["status"] == "success"
    assert updated["task"]["status"] == "running"
    assert updated["task"]["description"] == "updated"

    delete_response = client.delete(f"/api/tasks/{task_id}")
    assert delete_response.status_code == 200
    assert delete_response.json()["status"] == "success"

    missing_response = client.get(f"/api/tasks/{task_id}")
    assert missing_response.status_code == 404


def test_workflow_routes_list_execute_and_download_artifacts(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    tmp_path = _make_test_dir("workflow_routes")
    try:
        result = _build_completed_workflow_result(tmp_path)
        orchestrator = FakeWorkflowOrchestrator(result)
        monkeypatch.setattr(web_panel.workflows_api, "get_workflow_orchestrator", lambda: orchestrator)
        submitted = {}

        async def fake_start_background_run(run_id, workflow_name, request, output_root):
            submitted["run_id"] = run_id
            submitted["workflow_name"] = workflow_name
            submitted["request"] = request
            submitted["output_root"] = output_root
            await web_panel.workflows_api._update_run_record(
                run_id,
                status="running",
                started_at=result.start_time.isoformat(),
                status_detail="Worker process started.",
                worker_pid=4321,
            )
            await web_panel.workflows_api._finalize_run_from_result(run_id, result.to_dict())
            return await web_panel.workflows_api._get_run_record(run_id)

        monkeypatch.setattr(web_panel.workflows_api, "_start_background_run", fake_start_background_run)

        workflows_response = client.get("/api/workflows/")
        assert workflows_response.status_code == 200
        workflows = workflows_response.json()
        assert workflows == [
            {
                "name": "robot_creation_pipeline",
                "description": "Create a robot and export it",
                "steps_count": 2,
                "step_names": ["create_model", "export_urdf"],
            }
        ]

        workflow_detail_response = client.get("/api/workflows/robot_creation_pipeline")
        assert workflow_detail_response.status_code == 200
        assert workflow_detail_response.json()["workflow"]["name"] == "robot_creation_pipeline"

        requested_output_root = tmp_path / "manual_run"
        run_response = client.post(
            "/api/workflows/robot_creation_pipeline/run",
            json={
                "use_real": False,
                "execution_strategy": "resume",
                "output_root": str(requested_output_root),
                "timeout_seconds": 45,
                "parameters": {"template": "web_panel_biped"},
            },
        )

        assert run_response.status_code == 202
        body = run_response.json()
        assert body["status"] == "accepted"
        assert body["workflow"] == "robot_creation_pipeline"
        run = body["run"]
        assert body["message"].startswith("Workflow started in background")
        assert run["workflow_name"] == "robot_creation_pipeline"
        assert run["status"] == "completed"
        assert run["mode"] == "mock"
        assert run["execution_strategy"] == "resume"
        assert run["output_root"] == str(requested_output_root)
        assert run["timeout_seconds"] == 45
        assert run["total_steps"] == 2
        assert run["completed_steps"] == 2
        assert run["skipped_steps"] == 0
        assert run["failed_steps"] == 0
        assert run["success_rate"] == 100.0
        assert run["last_event"] == "workflow_finished"
        assert run["steps_snapshot"][0]["status"] == "completed"
        assert len(run["artifacts"]) == 2
        assert run["log_download_url"] == f"/api/workflows/runs/{run['run_id']}/log"
        assert run["live_log_download_url"] == f"/api/workflows/runs/{run['run_id']}/live-log"

        assert submitted["workflow_name"] == "robot_creation_pipeline"
        assert submitted["output_root"] == str(requested_output_root)
        assert submitted["request"].use_real is False
        assert submitted["request"].timeout_seconds == 45
        assert submitted["request"].parameters == {
            "template": "web_panel_biped",
            "execution_strategy": "resume",
            "output_root": str(requested_output_root),
        }

        runs_response = client.get("/api/workflows/runs")
        assert runs_response.status_code == 200
        assert runs_response.json()["runs"][0]["run_id"] == run["run_id"]

        run_status_response = client.get(f"/api/workflows/runs/{run['run_id']}/status")
        assert run_status_response.status_code == 200
        assert run_status_response.json()["run"]["status"] == "completed"

        run_detail_response = client.get(f"/api/workflows/runs/{run['run_id']}")
        assert run_detail_response.status_code == 200
        assert run_detail_response.json()["run"]["artifacts"][0]["name"] == "created_robot.json"
        assert run_detail_response.json()["run"]["artifacts"][0]["artifact_type"] == "robot_config"
        assert run_detail_response.json()["run"]["artifacts"][0]["godot_load_supported"] is True
        assert run_detail_response.json()["run"]["artifacts"][1]["artifact_type"] == "urdf"
        assert run_detail_response.json()["run"]["artifacts"][1]["preview_mode"] == "web_urdf"

        artifact_response = client.get(run["artifacts"][0]["download_url"])
        assert artifact_response.status_code == 200
        assert artifact_response.content == (
            tmp_path / ".output" / "created_robot.json"
        ).read_bytes()

        log_response = client.get(run["log_download_url"])
        assert log_response.status_code == 200
        assert log_response.content == (
            tmp_path / ".output" / "workflow_log_robot_creation_pipeline.json"
        ).read_bytes()
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_workflow_artifact_godot_load_uses_legacy_controller(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    tmp_path = _make_test_dir("workflow_artifact_legacy_load")
    try:
        result = _build_completed_workflow_result(tmp_path)
        request = web_panel.workflows_api.WorkflowRunRequest(
            use_real=True,
            execution_strategy="force",
            parameters={"execution_strategy": "force", "output_root": str(tmp_path)},
        )
        run_id = uuid.uuid4().hex
        record = web_panel.workflows_api._build_initial_run_record(
            run_id,
            "robot_creation_pipeline",
            request,
            str(tmp_path),
        )
        _store_run_record(record)
        _finalize_run_from_result(run_id, result.to_dict())

        recorded: dict[str, object] = {}
        monkeypatch.setattr(
            web_panel.server.godot_controller,
            "is_connected",
            lambda session_id=None: False,
        )
        monkeypatch.setattr(
            web_panel.server.godot_controller,
            "connect",
            lambda host, port, session_id=None: recorded.update(
                {"host": host, "port": port, "connect_session_id": session_id}
            )
            or True,
        )
        monkeypatch.setattr(
            web_panel.server.godot_controller,
            "load_robot",
            lambda parts, connections, session_id=None: recorded.update(
                {
                    "parts": parts,
                    "connections": connections,
                    "load_session_id": session_id,
                }
            )
            or True,
        )

        response = client.post(
            f"/api/workflows/runs/{run_id}/artifacts/0/godot-load",
            json={
                "transport_mode": "legacy_controller",
                "session_id": "design-tab-1",
                "host": "127.0.0.1",
                "port": 9999,
            },
        )

        assert response.status_code == 200
        payload = response.json()
        assert payload["status"] == "success"
        assert payload["artifact"]["artifact_type"] == "robot_config"
        assert payload["transport"]["transport_mode"] == "legacy_controller"
        assert payload["transport"]["session_id"] == "design-tab-1"
        assert payload["godot_delivery"]["transport_status_url"].endswith(
            "/api/godot/status?session_id=design-tab-1"
        )
        assert payload["godot_delivery"]["artifact_retry_url"].endswith(
            f"/api/workflows/runs/{run_id}/artifacts/0/godot-load"
        )
        assert payload["robot_config_summary"] == {
            "parts_count": 1,
            "connections_count": 0,
        }
        assert recorded == {
            "host": "127.0.0.1",
            "port": 9999,
            "connect_session_id": "design-tab-1",
            "parts": [{"id": "torso", "type": "body"}],
            "connections": [],
            "load_session_id": "design-tab-1",
        }
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_workflow_artifact_godot_load_uses_session_bridge(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    tmp_path = _make_test_dir("workflow_artifact_session_bridge")
    try:
        result = _build_completed_workflow_result(tmp_path)
        request = web_panel.workflows_api.WorkflowRunRequest(
            use_real=True,
            execution_strategy="force",
            parameters={"execution_strategy": "force", "output_root": str(tmp_path)},
        )
        run_id = uuid.uuid4().hex
        record = web_panel.workflows_api._build_initial_run_record(
            run_id,
            "robot_creation_pipeline",
            request,
            str(tmp_path),
        )
        _store_run_record(record)
        _finalize_run_from_result(run_id, result.to_dict())

        observed: dict[str, object] = {}

        class FakeBridge:
            def __init__(self) -> None:
                self._running = False

            def is_running(self) -> bool:
                return self._running

            def launch(self, scene: str, godot_exe: str = "", headless: bool = False) -> dict[str, object]:
                observed["launch"] = {"scene": scene, "godot_exe": godot_exe, "headless": headless}
                self._running = True
                return {"status": "launched", "pid": 4321}

            def is_connected(self) -> bool:
                return False

            async def wait_until_connected(self, timeout_seconds: float = 10.0) -> bool:
                observed["wait_until_connected"] = timeout_seconds
                return True

            async def send_load_robot(self, config: dict[str, object]) -> dict[str, object]:
                observed["robot_config"] = config
                return {"status": "success"}

            async def wait_until_schema(self, timeout_seconds: float = 5.0) -> dict[str, object]:
                observed["wait_until_schema"] = timeout_seconds
                return {"sensors": {}, "actuators": {}}

            def get_process_diagnostics(self) -> dict[str, object]:
                return {"running": self._running}

        fake_bridge = FakeBridge()

        class FakeSessionManager:
            def get_or_create(self, session_id):
                observed["session_id"] = session_id
                return fake_bridge

        fake_manager = FakeSessionManager()
        client.app.state.godot_session_manager = fake_manager

        response = client.post(
            f"/api/workflows/runs/{run_id}/artifacts/0/godot-load",
            json={
                "transport_mode": "session_bridge",
                "session_id": "workflow-preview",
                "launch_if_needed": True,
                "headless": True,
            },
        )

        assert response.status_code == 200
        payload = response.json()
        assert payload["status"] == "success"
        assert payload["transport"]["transport_mode"] == "session_bridge"
        assert payload["transport"]["schema_available"] is True
        assert payload["transport"]["launch_result"]["status"] == "launched"
        assert payload["godot_delivery"]["schema_keys"] == ["actuators", "sensors"]
        assert payload["godot_delivery"]["transport_status_url"].endswith(
            "/api/godot/workflow-preview/status"
        )
        assert payload["godot_delivery"]["delivery_target"] == "session bridge workflow-preview"
        assert observed["session_id"] == "workflow-preview"
        assert observed["launch"] == {
            "scene": "demo_generated_biped.tscn",
            "godot_exe": "",
            "headless": True,
        }
        assert observed["robot_config"] == {
            "robot_name": "web_panel_bot",
            "parts": [{"id": "torso", "type": "body"}],
            "connections": [],
        }
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_workflow_godot_sync_uses_recommended_artifact_and_persists_delivery(
    client: TestClient,
) -> None:
    tmp_path = _make_test_dir("workflow_godot_sync_recommended")
    try:
        result = _build_completed_workflow_result(tmp_path)
        request = web_panel.workflows_api.WorkflowRunRequest(
            use_real=True,
            execution_strategy="force",
            parameters={"execution_strategy": "force", "output_root": str(tmp_path)},
        )
        run_id = uuid.uuid4().hex
        record = web_panel.workflows_api._build_initial_run_record(
            run_id,
            "robot_creation_pipeline",
            request,
            str(tmp_path),
        )
        _store_run_record(record)
        _finalize_run_from_result(run_id, result.to_dict())

        observed: dict[str, object] = {}

        class FakeBridge:
            def is_running(self) -> bool:
                return True

            def launch(self, scene: str, godot_exe: str = "", headless: bool = False) -> dict[str, object]:
                observed["launch"] = {"scene": scene, "godot_exe": godot_exe, "headless": headless}
                return {"status": "already_running", "pid": 9876}

            def is_connected(self) -> bool:
                return True

            async def wait_until_connected(self, timeout_seconds: float = 10.0) -> bool:
                observed["wait_until_connected"] = timeout_seconds
                return True

            async def send_load_robot(self, config: dict[str, object]) -> dict[str, object]:
                observed["robot_config"] = config
                return {"status": "success"}

            async def wait_until_schema(self, timeout_seconds: float = 5.0) -> dict[str, object]:
                observed["wait_until_schema"] = timeout_seconds
                return {"sensors": {}, "actuators": {}}

            def get_process_diagnostics(self) -> dict[str, object]:
                return {"running": True}

        class FakeSessionManager:
            def get_or_create(self, session_id):
                observed["session_id"] = session_id
                return FakeBridge()

        client.app.state.godot_session_manager = FakeSessionManager()

        response = client.post(
            f"/api/workflows/runs/{run_id}/godot-sync",
            json={
                "session_id": "official-session",
            },
        )

        assert response.status_code == 200
        payload = response.json()
        assert payload["status"] == "success"
        assert payload["artifact"]["artifact_type"] == "robot_config"
        assert payload["godot_delivery"]["status"] == "success"
        assert payload["godot_delivery"]["auto_selected"] is True
        assert payload["godot_delivery"]["transport_mode"] == "session_bridge"
        assert payload["godot_delivery"]["recommended_sync_url"].endswith(
            f"/api/workflows/runs/{run_id}/godot-sync"
        )
        assert observed["session_id"] == "official-session"
        assert observed["robot_config"] == {
            "robot_name": "web_panel_bot",
            "parts": [{"id": "torso", "type": "body"}],
            "connections": [],
        }

        run_detail = client.get(f"/api/workflows/runs/{run_id}").json()["run"]
        assert run_detail["godot_delivery"]["session_id"] == "official-session"
        assert run_detail["godot_delivery"]["transport_status_url"].endswith(
            "/api/godot/official-session/status"
        )
        assert run_detail["recommended_godot_sync_url"].endswith("/godot-sync")
        assert run_detail["preferred_godot_transport_mode"] == "session_bridge"
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_workflow_godot_sync_persists_failure_stage_and_retry_hint(
    client: TestClient,
) -> None:
    tmp_path = _make_test_dir("workflow_godot_sync_failure")
    try:
        result = _build_completed_workflow_result(tmp_path)
        request = web_panel.workflows_api.WorkflowRunRequest(
            use_real=True,
            execution_strategy="force",
            parameters={"execution_strategy": "force", "output_root": str(tmp_path)},
        )
        run_id = uuid.uuid4().hex
        record = web_panel.workflows_api._build_initial_run_record(
            run_id,
            "robot_creation_pipeline",
            request,
            str(tmp_path),
        )
        _store_run_record(record)
        _finalize_run_from_result(run_id, result.to_dict())

        class FakeBridge:
            def is_running(self) -> bool:
                return True

            def is_connected(self) -> bool:
                return False

            async def wait_until_connected(self, timeout_seconds: float = 10.0) -> bool:
                return False

            def get_process_diagnostics(self) -> dict[str, object]:
                return {"running": True, "tcp_connected": False}

        class FakeSessionManager:
            def get_or_create(self, session_id):
                return FakeBridge()

        client.app.state.godot_session_manager = FakeSessionManager()

        response = client.post(
            f"/api/workflows/runs/{run_id}/godot-sync",
            json={"session_id": "broken-session"},
        )

        assert response.status_code == 504
        assert "did not become ready in time" in response.json()["detail"]

        run_detail = client.get(f"/api/workflows/runs/{run_id}").json()["run"]
        assert run_detail["godot_delivery"]["status"] == "error"
        assert run_detail["godot_delivery"]["failure_stage"] == "tcp_connect"
        assert "TCP" in run_detail["godot_delivery"]["retry_hint"]
        assert run_detail["godot_delivery"]["transport_status_url"].endswith(
            "/api/godot/broken-session/status"
        )
        assert run_detail["godot_delivery"]["artifact_retry_url"].endswith(
            f"/api/workflows/runs/{run_id}/artifacts/0/godot-load"
        )
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_workflow_run_rejects_invalid_execution_strategy(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    tmp_path = _make_test_dir("workflow_invalid_strategy")
    try:
        result = _build_completed_workflow_result(tmp_path)
        orchestrator = FakeWorkflowOrchestrator(result)
        monkeypatch.setattr(web_panel.workflows_api, "get_workflow_orchestrator", lambda: orchestrator)

        response = client.post(
            "/api/workflows/robot_creation_pipeline/run",
            json={"execution_strategy": "invalid"},
        )

        assert response.status_code == 400
        assert "Invalid execution_strategy" in response.json()["detail"]
        assert orchestrator.last_execute_call is None
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_runs_endpoint_supports_archive_scope_and_filters(client: TestClient) -> None:
    failed_run_id = uuid.uuid4().hex
    failed_output_root = "test_env/archive_filters/failed"
    failed_request = web_panel.workflows_api.WorkflowRunRequest(
        use_real=True,
        execution_strategy="force",
        parameters={
            "execution_strategy": "force",
            "output_root": failed_output_root,
        },
    )
    failed_record = web_panel.workflows_api._build_initial_run_record(
        failed_run_id,
        "robot_creation_pipeline",
        failed_request,
        failed_output_root,
    )
    _store_run_record(failed_record)
    _update_run_record(
        failed_run_id,
        status="failed",
        started_at="2026-03-29T08:00:00",
        finished_at="2026-03-29T08:05:00",
        duration=300.0,
        total_steps=3,
        completed_steps=1,
        skipped_steps=0,
        failed_steps=1,
        current_step_name=None,
        current_step_index=3,
        progress_updated_at="2026-03-29T08:05:00",
        status_detail="Workflow failed while optimizing mass distribution.",
        message="hip mass optimizer diverged",
        failed_step_name="optimize_params",
        failed_step_error="hip mass optimizer diverged",
        diagnostic_summary="hip mass optimizer diverged",
    )

    completed_run_id = uuid.uuid4().hex
    completed_output_root = "test_env/archive_filters/completed"
    completed_request = web_panel.workflows_api.WorkflowRunRequest(
        use_real=False,
        execution_strategy="resume",
        parameters={
            "execution_strategy": "resume",
            "output_root": completed_output_root,
        },
    )
    completed_record = web_panel.workflows_api._build_initial_run_record(
        completed_run_id,
        "robot_creation_pipeline",
        completed_request,
        completed_output_root,
    )
    _store_run_record(completed_record)
    _update_run_record(
        completed_run_id,
        status="completed",
        started_at="2026-03-30T09:00:00",
        finished_at="2026-03-30T09:02:30",
        duration=150.0,
        total_steps=2,
        completed_steps=2,
        skipped_steps=0,
        failed_steps=0,
        current_step_name=None,
        current_step_index=2,
        progress_updated_at="2026-03-30T09:02:30",
        success_rate=100.0,
        status_detail="Workflow finished successfully.",
    )

    web_panel.workflows_api.execution_history.clear()
    web_panel.workflows_api.active_run_processes.clear()
    web_panel.workflows_api.run_event_history.clear()
    web_panel.workflows_api.run_event_counters.clear()

    response = client.get(
        "/api/workflows/runs",
        params={
            "scope": "archive",
            "workflow_name": "robot_creation_pipeline",
            "status": "failed",
            "mode": "real",
            "text": "hip mass",
            "date_from": "2026-03-29",
            "date_to": "2026-03-29",
            "only_failures": "true",
            "limit": "20",
        },
    )

    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "success"
    assert body["scope"] == "archive"
    assert body["count"] == 1
    assert body["filters"]["only_failures"] is True
    assert body["runs"][0]["run_id"] == failed_run_id
    assert body["runs"][0]["status"] == "failed"
    assert body["runs"][0]["run_source"] == "archive"
    assert body["runs"][0]["diagnostic_summary"] == "hip mass optimizer diverged"

    combined_response = client.get(
        "/api/workflows/runs",
        params={
            "scope": "all",
            "mode": "mock",
            "limit": "20",
        },
    )
    assert combined_response.status_code == 200
    combined_runs = combined_response.json()["runs"]
    assert len(combined_runs) == 1
    assert combined_runs[0]["run_id"] == completed_run_id
    assert combined_runs[0]["run_source"] == "archive"

    detail_response = client.get(f"/api/workflows/runs/{failed_run_id}")
    assert detail_response.status_code == 200
    assert detail_response.json()["run"]["run_source"] == "archive"


def test_runs_endpoint_rejects_invalid_scope(client: TestClient) -> None:
    response = client.get("/api/workflows/runs", params={"scope": "invalid"})

    assert response.status_code == 400
    assert "Invalid scope" in response.json()["detail"]


def test_runs_endpoint_supports_pagination_metadata(client: TestClient) -> None:
    first_run_id = _seed_archived_run(
        mode="real",
        execution_strategy="force",
        status="completed",
        output_root="test_env/pagination/first",
        started_at="2026-03-28T08:00:00",
        finished_at="2026-03-28T08:03:00",
    )
    second_run_id = _seed_archived_run(
        mode="mock",
        execution_strategy="resume",
        status="completed",
        output_root="test_env/pagination/second",
        started_at="2026-03-29T09:00:00",
        finished_at="2026-03-29T09:02:00",
    )
    third_run_id = _seed_archived_run(
        mode="real",
        execution_strategy="force",
        status="failed",
        output_root="test_env/pagination/third",
        started_at="2026-03-30T10:00:00",
        finished_at="2026-03-30T10:01:00",
        completed_steps=1,
        failed_steps=1,
        success_rate=50.0,
        diagnostic_summary="third run failed",
    )

    web_panel.workflows_api.execution_history.clear()
    web_panel.workflows_api.active_run_processes.clear()
    web_panel.workflows_api.run_event_history.clear()
    web_panel.workflows_api.run_event_counters.clear()

    response = client.get(
        "/api/workflows/runs",
        params={
            "scope": "archive",
            "page": "2",
            "page_size": "2",
        },
    )

    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "success"
    assert body["count"] == 1
    assert body["total_count"] == 3
    assert body["page"] == 2
    assert body["page_size"] == 2
    assert body["total_pages"] == 2
    assert body["offset"] == 2
    assert body["has_previous_page"] is True
    assert body["has_next_page"] is False
    assert body["archive_retention_policy"]["max_runs"] == web_panel.workflows_api.ARCHIVE_RETENTION_MAX_RUNS
    assert body["runs"][0]["run_id"] == first_run_id
    assert {run["run_id"] for run in body["runs"]}.isdisjoint({second_run_id, third_run_id})


def test_archive_retention_policy_prunes_old_and_excess_archives(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(web_panel.workflows_api, "ARCHIVE_RETENTION_MAX_RUNS", 0)
    monkeypatch.setattr(web_panel.workflows_api, "ARCHIVE_RETENTION_MAX_AGE_DAYS", 0)

    oldest_run_id = _seed_archived_run(
        output_root="test_env/retention/oldest",
        started_at="2026-03-20T08:00:00",
        finished_at="2026-03-20T08:05:00",
    )
    older_run_id = _seed_archived_run(
        output_root="test_env/retention/older",
        started_at="2026-03-21T08:00:00",
        finished_at="2026-03-21T08:05:00",
    )
    newer_run_id = _seed_archived_run(
        output_root="test_env/retention/newer",
        started_at="2026-03-22T08:00:00",
        finished_at="2026-03-22T08:05:00",
    )
    newest_run_id = _seed_archived_run(
        output_root="test_env/retention/newest",
        started_at="2026-03-23T08:00:00",
        finished_at="2026-03-23T08:05:00",
    )

    old_timestamp = time.time() - (3 * 24 * 60 * 60)
    older_timestamp = time.time() - (3 * 60 * 60)
    newer_timestamp = time.time() - (2 * 60 * 60)
    newest_timestamp = time.time() - (1 * 60 * 60)

    os.utime(web_panel.workflows_api._archive_run_path(oldest_run_id), (old_timestamp, old_timestamp))
    os.utime(web_panel.workflows_api._archive_run_path(older_run_id), (older_timestamp, older_timestamp))
    os.utime(web_panel.workflows_api._archive_run_path(newer_run_id), (newer_timestamp, newer_timestamp))
    os.utime(web_panel.workflows_api._archive_run_path(newest_run_id), (newest_timestamp, newest_timestamp))

    monkeypatch.setattr(web_panel.workflows_api, "ARCHIVE_RETENTION_MAX_RUNS", 2)
    monkeypatch.setattr(web_panel.workflows_api, "ARCHIVE_RETENTION_MAX_AGE_DAYS", 1)
    result = web_panel.workflows_api._enforce_archive_retention_policy()

    remaining_run_ids = {
        path.stem
        for path in web_panel.workflows_api.WORKFLOW_ARCHIVE_ROOT.glob("*.json")
    }
    assert oldest_run_id in result["removed_run_ids"]
    assert older_run_id in result["removed_run_ids"]
    assert remaining_run_ids == {newer_run_id, newest_run_id}
    assert result["remaining_files"] == 2


def test_workflow_routes_support_status_polling_and_cancel_request(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    tmp_path = _make_test_dir("workflow_running_status")
    try:
        result = _build_completed_workflow_result(tmp_path)
        orchestrator = FakeWorkflowOrchestrator(result)
        monkeypatch.setattr(web_panel.workflows_api, "get_workflow_orchestrator", lambda: orchestrator)

        async def fake_start_background_run(run_id, workflow_name, request, output_root):
            return await web_panel.workflows_api._update_run_record(
                run_id,
                status="running",
                started_at=datetime(2026, 3, 30, 20, 0, 0).isoformat(),
                status_detail="Worker process started.",
                worker_pid=9988,
            )

        monkeypatch.setattr(web_panel.workflows_api, "_start_background_run", fake_start_background_run)

        run_response = client.post(
            "/api/workflows/robot_creation_pipeline/run",
            json={"execution_strategy": "force"},
        )

        assert run_response.status_code == 202
        run = run_response.json()["run"]
        assert run["status"] == "running"
        assert run["cancel_requested"] is False

        status_response = client.get(f"/api/workflows/runs/{run['run_id']}/status")
        assert status_response.status_code == 200
        assert status_response.json()["run"] == {
            "run_id": run["run_id"],
            "run_source": "active",
            "workflow_name": "robot_creation_pipeline",
            "status": "running",
            "status_detail": "Worker process started.",
            "cancel_requested": False,
            "started_at": "2026-03-30T20:00:00",
            "finished_at": None,
            "duration": 0.0,
            "success_rate": 0.0,
            "total_steps": None,
            "completed_steps": 0,
            "skipped_steps": 0,
            "failed_steps": 0,
            "current_step_name": None,
            "current_step_index": None,
            "last_event": None,
            "progress_updated_at": None,
            "steps_snapshot": [],
            "live_log_tail": [],
            "live_log_line_count": 0,
            "live_log_updated_at": None,
            "failed_step_name": None,
            "failed_step_error": None,
            "step_errors": [],
            "worker_error_message": None,
            "diagnostic_summary": None,
            "worker_pid": 9988,
            "godot_delivery": None,
            "preferred_godot_transport_mode": "session_bridge",
            "recommended_godot_sync_url": f"/api/workflows/runs/{run['run_id']}/godot-sync",
        }

        cancel_response = client.post(f"/api/workflows/runs/{run['run_id']}/cancel")
        assert cancel_response.status_code == 200
        cancelled = cancel_response.json()
        assert cancelled["status"] == "success"
        assert cancelled["message"] == "Cancellation requested."
        assert cancelled["run"]["cancel_requested"] is True
        assert "Cancellation requested" in cancelled["run"]["status_detail"]

        status_after_cancel = client.get(f"/api/workflows/runs/{run['run_id']}/status")
        assert status_after_cancel.status_code == 200
        assert status_after_cancel.json()["run"]["cancel_requested"] is True
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_run_events_endpoint_replays_backlog_and_closes_on_terminal(
    client: TestClient,
) -> None:
    run_id = uuid.uuid4().hex
    request = web_panel.workflows_api.WorkflowRunRequest(
        use_real=False,
        execution_strategy="force",
        parameters={"execution_strategy": "force", "output_root": "test_env/events"},
    )
    record = web_panel.workflows_api._build_initial_run_record(
        run_id,
        "robot_creation_pipeline",
        request,
        "test_env/events",
    )
    _store_run_record(record)
    _update_run_record(
        run_id,
        event_type="run_started",
        status="running",
        started_at="2026-03-30T20:02:00",
        status_detail="Worker process started.",
        worker_pid=8877,
    )
    _mark_run_terminal(
        run_id,
        "completed",
        status_detail="Workflow finished successfully.",
        exit_reason="test_terminal",
    )

    with client.stream(
        "GET",
        f"/api/workflows/runs/{run_id}/events",
        headers={"Last-Event-ID": "1"},
    ) as response:
        assert response.status_code == 200
        body = "".join(response.iter_text())

    assert "event: workflow_run_event" in body
    assert '"event_type": "run_started"' in body
    assert '"event_type": "run_terminal"' in body
    assert '"status": "completed"' in body


def test_finalize_failed_result_extracts_step_diagnostics() -> None:
    run_id = uuid.uuid4().hex
    request = web_panel.workflows_api.WorkflowRunRequest(
        use_real=False,
        execution_strategy="force",
        parameters={"execution_strategy": "force", "output_root": "test_env/failed_result"},
    )
    record = web_panel.workflows_api._build_initial_run_record(
        run_id,
        "robot_creation_pipeline",
        request,
        "test_env/failed_result",
    )
    _store_run_record(record)

    failed_result = _build_failed_workflow_result()
    run = _finalize_run_from_result(run_id, failed_result.to_dict())

    assert run["status"] == "failed"
    assert run["failed_step_name"] == "optimize_params"
    assert run["failed_step_error"] == "mass distribution optimizer diverged"
    assert run["diagnostic_summary"] == "mass distribution optimizer diverged"
    assert run["step_errors"] == [
        {
            "name": "optimize_params",
            "status": "failed",
            "error": "mass distribution optimizer diverged",
            "executor": "parameter_optimizer",
            "action": "optimize_mass_distribution",
            "output_file": None,
        }
    ]


def test_monitor_marks_cancelled_run_terminal() -> None:
    run_id = uuid.uuid4().hex
    request = web_panel.workflows_api.WorkflowRunRequest(
        use_real=False,
        execution_strategy="force",
        parameters={"execution_strategy": "force", "output_root": "test_env/cancelled"},
    )
    record = web_panel.workflows_api._build_initial_run_record(
        run_id,
        "robot_creation_pipeline",
        request,
        "test_env/cancelled",
    )
    _store_run_record(record)
    _update_run_record(
        run_id,
        status="running",
        started_at=datetime(2026, 3, 30, 20, 5, 0).isoformat(),
        cancel_requested=True,
    )
    web_panel.workflows_api.active_run_processes[run_id] = {
        "process": FakeProcess(exit_code=15),
        "result_path": Path("test_env") / "missing_cancelled_result.json",
        "progress_path": Path("test_env") / "missing_cancelled_progress.json",
        "live_log_path": Path("test_env") / "missing_cancelled_live.log",
        "live_log_offset": 0,
        "timeout_seconds": None,
        "started_monotonic": time.monotonic(),
        "monitor_thread": None,
    }

    web_panel.workflows_api._monitor_background_run(run_id)

    run = _get_run_record(run_id)
    assert run["status"] == "cancelled"
    assert run["exit_reason"] == "cancelled_by_user"
    assert run["message"] == "Cancellation requested from Web workflow control plane."
    assert run_id not in web_panel.workflows_api.active_run_processes


def test_monitor_marks_timed_out_run_terminal() -> None:
    run_id = uuid.uuid4().hex
    request = web_panel.workflows_api.WorkflowRunRequest(
        use_real=False,
        execution_strategy="force",
        timeout_seconds=0.1,
        parameters={"execution_strategy": "force", "output_root": "test_env/timed_out"},
    )
    record = web_panel.workflows_api._build_initial_run_record(
        run_id,
        "robot_creation_pipeline",
        request,
        "test_env/timed_out",
    )
    _store_run_record(record)
    _update_run_record(
        run_id,
        status="running",
        started_at=datetime(2026, 3, 30, 20, 6, 0).isoformat(),
    )
    web_panel.workflows_api.active_run_processes[run_id] = {
        "process": FakeProcess(exit_code=24),
        "result_path": Path("test_env") / "missing_timed_out_result.json",
        "progress_path": Path("test_env") / "missing_timed_out_progress.json",
        "live_log_path": Path("test_env") / "missing_timed_out_live.log",
        "live_log_offset": 0,
        "timeout_seconds": 0.1,
        "started_monotonic": time.monotonic() - 1.0,
        "monitor_thread": None,
    }

    web_panel.workflows_api._monitor_background_run(run_id)

    run = _get_run_record(run_id)
    assert run["status"] == "timed_out"
    assert run["exit_reason"] == "timeout"
    assert "timed out" in (run["message"] or "")
    assert run_id not in web_panel.workflows_api.active_run_processes


def test_monitor_preserves_worker_error_traceback() -> None:
    tmp_path = _make_test_dir("workflow_worker_error_traceback")
    try:
        run_id = uuid.uuid4().hex
        request = web_panel.workflows_api.WorkflowRunRequest(
            use_real=False,
            execution_strategy="force",
            parameters={"execution_strategy": "force", "output_root": str(tmp_path)},
        )
        record = web_panel.workflows_api._build_initial_run_record(
            run_id,
            "robot_creation_pipeline",
            request,
            str(tmp_path),
        )
        _store_run_record(record)
        _update_run_record(
            run_id,
            status="running",
            started_at=datetime(2026, 3, 30, 20, 8, 0).isoformat(),
        )
        result_path = tmp_path / "worker_error_result.json"
        result_path.write_text(
            json.dumps(
                {
                    "status": "error",
                    "error_message": "executor import failed",
                    "traceback": "Traceback (most recent call last):\\nRuntimeError: executor import failed",
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        process = FakeProcess(exit_code=1)
        process.terminate()
        web_panel.workflows_api.active_run_processes[run_id] = {
            "process": process,
            "result_path": result_path,
            "progress_path": tmp_path / "missing_progress.json",
            "live_log_path": tmp_path / "missing_live.log",
            "live_log_offset": 0,
            "timeout_seconds": None,
            "started_monotonic": time.monotonic(),
            "monitor_thread": None,
        }

        web_panel.workflows_api._monitor_background_run(run_id)

        run = _get_run_record(run_id)
        assert run["status"] == "failed"
        assert run["worker_error_message"] == "executor import failed"
        assert "RuntimeError: executor import failed" in (run["worker_traceback"] or "")
        assert run["diagnostic_summary"] == "executor import failed"
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_monitor_merges_worker_progress_snapshot() -> None:
    tmp_path = _make_test_dir("workflow_progress_snapshot")
    try:
        run_id = uuid.uuid4().hex
        progress_path = tmp_path / "progress.json"
        result_path = tmp_path / "result.json"
        live_log_path = tmp_path / "live.log"
        request = web_panel.workflows_api.WorkflowRunRequest(
            use_real=False,
            execution_strategy="force",
            parameters={"execution_strategy": "force", "output_root": str(tmp_path)},
        )
        record = web_panel.workflows_api._build_initial_run_record(
            run_id,
            "robot_creation_pipeline",
            request,
            str(tmp_path),
        )
        _store_run_record(record)
        _update_run_record(
            run_id,
            status="running",
            started_at=datetime(2026, 3, 30, 20, 10, 0).isoformat(),
        )
        progress_path.write_text(
            json.dumps(
                {
                    "event": "step_started",
                    "workflow_name": "robot_creation_pipeline",
                    "workflow_status": "running",
                    "timestamp": "2026-03-30T20:10:05",
                    "started_at": "2026-03-30T20:10:00",
                    "finished_at": None,
                    "step_index": 2,
                    "total_steps": 3,
                    "completed_steps": 1,
                    "skipped_steps": 0,
                    "failed_steps": 0,
                    "success_rate": 33.3333333333,
                    "current_step": {
                        "name": "optimize_params",
                        "executor": "parameter_optimizer",
                        "action": "optimize_mass_distribution",
                        "status": "running",
                        "duration": 0.0,
                        "error": None,
                        "start_time": "2026-03-30T20:10:05",
                        "end_time": None,
                        "output_file": None,
                        "output_keys": [],
                    },
                    "steps": [
                        {
                            "name": "create_model",
                            "executor": "robot_modeling",
                            "action": "create_from_template",
                            "status": "completed",
                            "duration": 1.0,
                            "error": None,
                            "start_time": "2026-03-30T20:10:00",
                            "end_time": "2026-03-30T20:10:01",
                            "output_file": str(tmp_path / "created_robot.json"),
                            "output_keys": ["output_file"],
                        },
                        {
                            "name": "optimize_params",
                            "executor": "parameter_optimizer",
                            "action": "optimize_mass_distribution",
                            "status": "running",
                            "duration": 0.0,
                            "error": None,
                            "start_time": "2026-03-30T20:10:05",
                            "end_time": None,
                            "output_file": None,
                            "output_keys": [],
                        },
                    ],
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        process = FakeProcess(exit_code=0)
        process.terminate()
        live_log_path.write_text(
            "[2026-03-30T20:10:05] Step 2/3: optimize_params started.\n"
            "[2026-03-30T20:10:06] Workflow finished with status=completed.\n",
            encoding="utf-8",
        )
        web_panel.workflows_api.active_run_processes[run_id] = {
            "process": process,
            "result_path": result_path,
            "progress_path": progress_path,
            "live_log_path": live_log_path,
            "live_log_offset": 0,
            "timeout_seconds": None,
            "started_monotonic": time.monotonic() - 0.05,
            "monitor_thread": None,
        }
        result_path.write_text(
            json.dumps(
                {
                    "status": "ok",
                    "result": _build_completed_workflow_result(tmp_path).to_dict(),
                },
                indent=2,
            ),
            encoding="utf-8",
        )

        web_panel.workflows_api._monitor_background_run(run_id)

        run = _get_run_record(run_id)
        assert run["total_steps"] == 2
        assert run["last_event"] == "workflow_finished"
        assert run["steps_snapshot"][0]["name"] == "create_model"
        assert run["steps_snapshot"][1]["name"] == "export_urdf"
        assert run["live_log_line_count"] == 2
        assert "optimize_params started" in run["live_log_tail"][0]
        assert run["status"] == "completed"
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_live_log_endpoint_returns_incremental_text(client: TestClient) -> None:
    tmp_path = _make_test_dir("workflow_live_log_endpoint")
    try:
        run_id = uuid.uuid4().hex
        request = web_panel.workflows_api.WorkflowRunRequest(
            use_real=False,
            execution_strategy="force",
            parameters={"execution_strategy": "force", "output_root": str(tmp_path)},
        )
        record = web_panel.workflows_api._build_initial_run_record(
            run_id,
            "robot_creation_pipeline",
            request,
            str(tmp_path),
        )
        live_log_path = Path(record["live_log_path"])
        live_log_path.parent.mkdir(parents=True, exist_ok=True)
        live_log_path.write_text(
            "[2026-03-30T20:20:00] Workflow execution started with 3 steps.\n",
            encoding="utf-8",
        )
        _store_run_record(record)

        response = client.get(f"/api/workflows/runs/{run_id}/live-log")

        assert response.status_code == 200
        assert "Workflow execution started" in response.text
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)


def test_parse_command_endpoint_uses_regex_fallback(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(CommandParser, "_llm_parse", lambda self, prompt: None)

    response = client.post(
        "/api/agent/parse-command",
        json={"command": "创建一个双足机器人 高度0.62 质量7.5 叫walkerbot"},
    )

    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["config"]["metadata"]["type"] == "biped"
    assert data["config"]["metadata"]["parser"] == "Regex"
    assert data["config"]["skills_params"]["torso_height"] == 0.62
    assert data["config"]["skills_params"]["torso_mass"] == 7.5


def test_legacy_godot_routes_reflect_mocked_controller_state(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "connect",
        lambda host, port, session_id=None: True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "disconnect",
        lambda session_id=None: None,
    )
    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: True)
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "load_robot",
        lambda parts, connections, session_id=None: True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "start_simulation",
        lambda physics, session_id=None: True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "stop_simulation",
        lambda session_id=None: True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "update_params",
        lambda params, session_id=None: True,
    )
    monkeypatch.setattr(web_panel.server.godot_controller.client, "running", True)

    connect_response = client.post("/api/godot/connect", json={"host": "127.0.0.1", "port": 9999})
    assert connect_response.status_code == 200
    assert connect_response.json()["status"] == "connected"

    status_response = client.get("/api/godot/status")
    assert status_response.status_code == 200
    assert status_response.json() == {"connected": True, "client_running": True}

    load_response = client.post("/api/godot/load-robot", json={"parts": [], "connections": []})
    assert load_response.status_code == 200
    assert load_response.json()["status"] == "success"

    start_response = client.post("/api/godot/start", json={"physics": {"gravity": 9.81}})
    assert start_response.status_code == 200
    assert start_response.json()["status"] == "started"

    update_response = client.post("/api/godot/update-params", json={"params": {"mass": 7.5}})
    assert update_response.status_code == 200
    assert update_response.json()["status"] == "updated"

    stop_response = client.post("/api/godot/stop")
    assert stop_response.status_code == 200
    assert stop_response.json()["status"] == "stopped"

    disconnect_response = client.post("/api/godot/disconnect")
    assert disconnect_response.status_code == 200
    assert disconnect_response.json()["status"] == "disconnected"


def test_legacy_godot_load_robot_requires_connection(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
):
    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: False)

    response = client.post("/api/godot/load-robot", json={"parts": [], "connections": []})

    assert response.status_code == 400
    assert response.json()["detail"] == "Godot not connected"


def test_legacy_controller_forwards_physics_to_client(
    monkeypatch: pytest.MonkeyPatch,
):
    recorded = {}

    monkeypatch.setattr(
        web_panel.server.godot_controller.client,
        "start_simulation",
        lambda robot_config, physics_config=None: recorded.update(
            {
                "robot_config": robot_config,
                "physics_config": physics_config,
            }
        )
        or True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "cached_robot_config",
        {"parts": [{"id": "hip_left"}], "connections": []},
        raising=False,
    )

    result = web_panel.server.godot_controller.start_simulation(
        {"gravity": 1.62, "timestep": 0.02}
    )

    assert result is True
    assert recorded == {
        "robot_config": {"parts": [{"id": "hip_left"}], "connections": []},
        "physics_config": {"gravity": 1.62, "timestep": 0.02},
    }


def test_legacy_controller_broadcasts_canonical_messages() -> None:
    messages = []
    controller = web_panel.server.godot_controller

    controller.set_broadcast_callback(
        lambda session_id, message: messages.append((session_id, message))
    )
    controller.client.data_callback({"battery": 88.0})

    assert messages[-1][0] == controller.legacy_session_id
    assert messages[-1][1]["type"] == "telemetry.update"
    assert messages[-1][1]["payload"] == {"data": {"battery": 88.0}}
    assert messages[-1][1]["status"] == "push"


def test_legacy_routes_forward_session_id_query_to_controller(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    recorded = {}
    session_id = "design-tab-1"

    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "connect",
        lambda host, port, session_id=None: recorded.setdefault("connect", session_id)
        or True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "disconnect",
        lambda session_id=None: recorded.setdefault("disconnect", session_id),
    )
    monkeypatch.setattr(web_panel.server.godot_controller, "is_connected", lambda: True)
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "load_robot",
        lambda parts, connections, session_id=None: recorded.setdefault(
            "load_robot",
            session_id,
        )
        or True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "start_simulation",
        lambda physics, session_id=None: recorded.setdefault("start", session_id)
        or True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "stop_simulation",
        lambda session_id=None: recorded.setdefault("stop", session_id) or True,
    )
    monkeypatch.setattr(
        web_panel.server.godot_controller,
        "update_params",
        lambda params, session_id=None: recorded.setdefault("update", session_id)
        or True,
    )

    client.post(
        f"/api/godot/connect?session_id={session_id}",
        json={"host": "127.0.0.1", "port": 9999},
    )
    client.post(
        f"/api/godot/load-robot?session_id={session_id}",
        json={"parts": [], "connections": []},
    )
    client.post(
        f"/api/godot/start?session_id={session_id}",
        json={"physics": {"gravity": 9.81}},
    )
    client.post(
        f"/api/godot/update-params?session_id={session_id}",
        json={"params": {"mass": 7.5}},
    )
    client.post(f"/api/godot/stop?session_id={session_id}")
    client.post(f"/api/godot/disconnect?session_id={session_id}")

    assert recorded == {
        "connect": session_id,
        "load_robot": session_id,
        "start": session_id,
        "update": session_id,
        "stop": session_id,
        "disconnect": session_id,
    }


def test_workflows_api_loads_runtime_env_file_on_reload(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    tmp_path = _make_test_dir("workflow_env_file_reload")
    try:
        env_file = tmp_path / "web_panel.env"
        env_file.write_text(
            "\n".join(
                [
                    "AGI_WALKER_WEB_RUNS_PAGE_SIZE=7",
                    "AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE=25",
                    "AGI_WALKER_WEB_ARCHIVE_MAX_RUNS=17",
                    "AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS=5",
                ]
            ),
            encoding="utf-8",
        )

        with monkeypatch.context() as ctx:
            ctx.setenv("AGI_WALKER_WEB_ENV_FILE", str(env_file))
            ctx.delenv("AGI_WALKER_WEB_RUNS_PAGE_SIZE", raising=False)
            ctx.delenv("AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE", raising=False)
            ctx.delenv("AGI_WALKER_WEB_ARCHIVE_MAX_RUNS", raising=False)
            ctx.delenv("AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS", raising=False)

            reloaded = importlib.reload(web_panel.workflows_api)

            assert reloaded.WEB_PANEL_ENV_FILE == env_file
            assert reloaded.DEFAULT_RUNS_PAGE_SIZE == 7
            assert reloaded.MAX_RUNS_PAGE_SIZE == 25
            assert reloaded.ARCHIVE_RETENTION_MAX_RUNS == 17
            assert reloaded.ARCHIVE_RETENTION_MAX_AGE_DAYS == 5
            assert reloaded._get_archive_retention_policy()["env_file"] == str(env_file)

        restored = importlib.reload(web_panel.workflows_api)
        assert restored.DEFAULT_RUNS_PAGE_SIZE == 20
        assert restored.MAX_RUNS_PAGE_SIZE == 100
        assert restored.ARCHIVE_RETENTION_MAX_RUNS == 200
        assert restored.ARCHIVE_RETENTION_MAX_AGE_DAYS == 30
    finally:
        shutil.rmtree(tmp_path, ignore_errors=True)
