import uuid
from datetime import datetime

import pytest
from fastapi.testclient import TestClient

import web_panel.server
import web_panel.workflows_api
from web_panel.database import Base, engine


async def _reset_auth_test_db() -> None:
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
        await conn.run_sync(Base.metadata.create_all)


def _run_coro(coro):
    return web_panel.workflows_api._run_async(coro)


@pytest.fixture(autouse=True)
def reset_auth_state():
    _run_coro(_reset_auth_test_db())
    web_panel.server.app.dependency_overrides.clear()
    yield
    web_panel.server.app.dependency_overrides.clear()
    _run_coro(_reset_auth_test_db())


@pytest.fixture()
def client() -> TestClient:
    with TestClient(web_panel.server.app) as test_client:
        yield test_client


def _register_and_login(client: TestClient, username: str, password: str) -> str:
    register_response = client.post(
        "/api/auth/register",
        data={"username": username, "password": password},
    )
    assert register_response.status_code == 200

    login_response = client.post(
        "/api/auth/login",
        data={"username": username, "password": password},
    )
    assert login_response.status_code == 200
    return login_response.json()["access_token"]


def test_auth_register_login_and_me_round_trip(client: TestClient) -> None:
    token = _register_and_login(client, f"user_{uuid.uuid4().hex[:8]}", "pass-1234")

    me_response = client.get(
        "/api/auth/me",
        headers={"Authorization": f"Bearer {token}"},
    )

    assert me_response.status_code == 200
    payload = me_response.json()
    assert payload["username"].startswith("user_")
    assert payload["is_admin"] is False
    assert payload["created_at"] is not None


def test_protected_workflow_route_requires_authentication(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    orchestrator = type(
        "AuthWorkflowOrchestrator",
        (),
        {
            "list_workflows": lambda self: ["robot_creation_pipeline"],
            "get_workflow": lambda self, name: {
                "name": name,
                "description": "auth test",
                "steps": [],
            },
        },
    )()
    monkeypatch.setattr(
        web_panel.workflows_api, "get_workflow_orchestrator", lambda: orchestrator
    )

    response = client.post(
        "/api/workflows/robot_creation_pipeline/run",
        json={"execution_strategy": "force"},
    )

    assert response.status_code == 401
    assert response.json()["detail"] == "Not authenticated"


def test_protected_workflow_route_accepts_valid_bearer_token(
    client: TestClient,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    token = _register_and_login(client, f"runner_{uuid.uuid4().hex[:8]}", "pass-1234")

    orchestrator = type(
        "AuthWorkflowOrchestrator",
        (),
        {
            "list_workflows": lambda self: ["robot_creation_pipeline"],
            "get_workflow": lambda self, name: {
                "name": name,
                "description": "auth test",
                "steps": [],
            },
        },
    )()
    monkeypatch.setattr(
        web_panel.workflows_api, "get_workflow_orchestrator", lambda: orchestrator
    )

    async def fake_start_background_run(run_id, workflow_name, request, output_root):
        return await web_panel.workflows_api._update_run_record(
            run_id,
            status="running",
            started_at=datetime(2026, 4, 2, 23, 0, 0).isoformat(),
            status_detail="Workflow queued for authenticated user.",
        )

    monkeypatch.setattr(
        web_panel.workflows_api, "_start_background_run", fake_start_background_run
    )

    response = client.post(
        "/api/workflows/robot_creation_pipeline/run",
        json={"execution_strategy": "force"},
        headers={"Authorization": f"Bearer {token}"},
    )

    assert response.status_code == 202
    payload = response.json()
    assert payload["status"] == "accepted"
    assert payload["run"]["status"] == "running"
    assert payload["run"]["status_detail"] == "Workflow queued for authenticated user."
