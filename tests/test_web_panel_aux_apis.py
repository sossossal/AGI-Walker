from fastapi.testclient import TestClient

from web_panel.server import app
from web_panel.distributed_monitor import DistributedMonitor


client = TestClient(app)


class FakeGodotAgentBackend:
    def execute_command(self, command, context=None, project_path=None):
        return {"status": "success", "message": command, "project_path": project_path}

    def execute_pipeline(self, commands, context=None):
        return [
            {"status": "success", "success": True, "message": cmd} for cmd in commands
        ]

    def get_roles_info(self):
        return [{"name": "developer", "description": "dev", "capabilities": ["code"]}]

    def list_skills(self):
        return {
            "status": "success",
            "skills": [{"id": "biped_3d_robot", "source_kind": "legacy_skill"}],
            "compatibility_alias": False,
        }

    def apply_skill(self, skill_id):
        return {
            "status": "success",
            "data": {"id": skill_id, "source_kind": "legacy_skill"},
            "compatibility_alias": False,
        }

    def list_templates(self):
        return {
            "status": "success",
            "templates": [
                {"id": "ai/patrol.gd", "type": "template", "source_kind": "template"}
            ],
            "backend_mode": "fake",
        }

    def get_template(self, template_id):
        return {
            "status": "success",
            "data": {
                "id": template_id,
                "type": "template",
                "content": "extends Node\n",
            },
            "backend_mode": "fake",
        }

    def plan_command(self, command, context=None, project_path=None):
        return {
            "status": "awaiting_confirmation",
            "success": True,
            "prompt": command,
            "project_path": project_path,
            "steps": [{"name": "Structure", "role": "developer"}],
            "context": context or {},
        }

    def get_history(self, limit=20):
        return {
            "status": "success",
            "items": [{"task_id": "task-1", "prompt": "生成玩家移动脚本"}],
            "count": 1,
            "limit": limit,
        }

    def doctor(self, project_path=None):
        return {
            "status": "success",
            "ok": True,
            "backend": "fake",
            "checks": [{"name": "router", "passed": True, "message": "FakeBackend"}],
            "project_path": project_path,
        }

    def launch_editor(self, project_path=None, scene_path=None):
        return {
            "status": "success",
            "ok": True,
            "message": "Godot editor launching",
            "data": {
                "project_path": project_path,
                "scene_path": scene_path,
                "pid": 1234,
            },
        }


class FakeNightlyStatusProvider:
    def snapshot(self):
        return {
            "status": "healthy",
            "repo": "demo/agi-walker",
            "workflow_file": ".github/workflows/ci.yml",
            "tracked_jobs": ["smoke", "distributed-smoke", "godot-headless-smoke"],
            "latest_run": {
                "id": 42,
                "run_number": 77,
                "event": "schedule",
                "status": "completed",
                "conclusion": "success",
                "run_started_at": "2026-04-01T02:00:00",
            },
            "summary": {
                "tracked_jobs": 3,
                "passed_jobs": 3,
                "failed_jobs": 0,
                "running_jobs": 0,
                "missing_jobs": 0,
                "skipped_jobs": 0,
            },
            "jobs": {
                "smoke": {
                    "name": "smoke",
                    "present": True,
                    "status": "completed",
                    "conclusion": "success",
                },
                "distributed-smoke": {
                    "name": "distributed-smoke",
                    "present": True,
                    "status": "completed",
                    "conclusion": "success",
                },
                "godot-headless-smoke": {
                    "name": "godot-headless-smoke",
                    "present": True,
                    "status": "completed",
                    "conclusion": "success",
                },
            },
        }

    def dashboard(self, limit_runs=5):
        data = self.snapshot()
        data["job_catalog"] = {
            "smoke": {
                "artifact_name": "smoke-artifacts",
                "local_repro_command": "python tests/run_smoke_tests.py --output-root test_env/smoke_runs/manual_nightly_repro",
            },
            "distributed-smoke": {
                "artifact_name": "distributed-smoke-artifacts",
                "local_repro_command": "python tests/run_distributed_smoke.py --build",
            },
            "godot-headless-smoke": {
                "artifact_name": "godot-headless-smoke-artifacts",
                "local_repro_command": "python -m pytest tests/test_godot_headless_smoke.py -q -m integration",
            },
        }
        data["recent_runs"] = [
            {
                "id": 42,
                "run_number": 77,
                "event": "schedule",
                "status": "healthy",
                "workflow_status": "completed",
                "conclusion": "success",
                "created_at": "2026-04-01T02:00:00",
                "updated_at": "2026-04-01T02:10:00",
                "run_started_at": "2026-04-01T02:01:00",
                "html_url": "https://example.invalid/runs/77",
                "head_branch": "main",
                "summary": data["summary"],
                "jobs": data["jobs"],
            }
        ][:limit_runs]
        return data


def _install_fake_godot_agent_backend(monkeypatch):
    monkeypatch.setattr(
        "web_panel.agent_api.create_godot_agent_backend",
        lambda: FakeGodotAgentBackend(),
    )
    if hasattr(app.state, "godot_agent_backend"):
        delattr(app.state, "godot_agent_backend")


def _install_fake_nightly_status_provider():
    app.state.nightly_status_provider = FakeNightlyStatusProvider()


def test_core_panel_routes_smoke():
    _install_fake_nightly_status_provider()

    response = client.get("/")
    assert response.status_code == 200
    assert "text/html" in response.headers["content-type"]

    response = client.get("/static/nightly.html")
    assert response.status_code == 200
    assert "Nightly 运维页" in response.text

    response = client.get("/api/system/status")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "running"
    assert "tasks_count" in data
    assert "active_connections" in data
    assert "distributed_monitor" in data
    assert "godot_agent" in data
    assert "zenoh_available" in data["distributed_monitor"]
    assert "monitor_active" in data["distributed_monitor"]
    assert "endpoint" in data["distributed_monitor"]
    assert "backend_mode" in data["godot_agent"]
    assert "backend_class" in data["godot_agent"]
    assert "router_ready" in data["godot_agent"]
    assert "resource_mode" in data["godot_agent"]
    assert "roles_count" in data["godot_agent"]
    assert "templates_count" in data["godot_agent"]
    assert "project_path" in data["godot_agent"]
    assert "history_file" in data["godot_agent"]
    assert "nightly_regressions" in data
    assert data["nightly_regressions"]["status"] == "healthy"
    assert data["nightly_regressions"]["latest_run"]["event"] == "schedule"
    assert data["nightly_regressions"]["jobs"]["smoke"]["conclusion"] == "success"

    response = client.get("/api/distributed/status")
    assert response.status_code == 200
    data = response.json()
    assert "actors" in data
    assert "monitor" in data
    assert "zenoh_available" in data["monitor"]
    assert "endpoint" in data["monitor"]

    response = client.get("/api/godot/capabilities")
    assert response.status_code == 200
    data = response.json()
    assert data["preferred_mode"] == "session_bridge"
    assert "legacy_controller" in data["modes"]
    assert "session_bridge" in data["modes"]


def test_tasks_api_smoke():
    response = client.get("/api/tasks")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert isinstance(data["tasks"], list)


def test_services_api_smoke():
    response = client.get("/api/skills/list")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"


def test_sim2real_api_smoke():
    response = client.post("/api/sim2real/analyze", json={"mock": True})
    assert response.status_code == 200
    data = response.json()
    assert data["status"] in {"success", "error"}


def test_parts_market_api_smoke():
    response = client.get("/api/parts/market")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert isinstance(data["parts"], list)


def test_godot_skills_endpoints_smoke():
    response = client.get("/api/godot_skills/list")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert any(skill.get("id") == "biped_3d_robot" for skill in data["skills"])
    assert data["compatibility_alias"] is False

    response = client.post(
        "/api/godot_skills/apply", json={"skill_id": "biped_3d_robot"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["data"]["id"] == "biped_3d_robot"
    assert data["compatibility_alias"] is False


def test_godot_agent_management_routes(monkeypatch):
    _install_fake_godot_agent_backend(monkeypatch)
    _install_fake_nightly_status_provider()

    response = client.get("/api/system/status")
    assert response.status_code == 200
    status_data = response.json()
    assert status_data["godot_agent"]["status"] == "ready"
    assert status_data["godot_agent"]["backend_mode"] in {
        "legacy",
        "godot-agent",
        "fake",
    }
    assert status_data["godot_agent"]["roles_count"] >= 0
    assert status_data["nightly_regressions"]["summary"]["passed_jobs"] == 3

    response = client.post(
        "/api/godot-agent/plan",
        json={
            "command": "生成玩家移动脚本",
            "context": {"mode": "preview"},
            "godot_project_path": "D:/tmp/project",
        },
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "awaiting_confirmation"
    assert data["project_path"] == "D:/tmp/project"
    assert data["steps"][0]["role"] == "developer"

    response = client.get("/api/godot-agent/history", params={"limit": 5})
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["count"] == 1
    assert data["items"][0]["task_id"] == "task-1"

    response = client.get(
        "/api/godot-agent/doctor", params={"godot_project_path": "D:/tmp/project"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["ok"] is True
    assert data["project_path"] == "D:/tmp/project"

    response = client.post(
        "/api/godot-agent/launch",
        json={
            "godot_project_path": "D:/tmp/project",
            "scene_path": "res://scenes/Main.tscn",
        },
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["data"]["scene_path"] == "res://scenes/Main.tscn"


def test_godot_agent_template_routes(monkeypatch):
    _install_fake_godot_agent_backend(monkeypatch)

    response = client.get("/api/godot-agent/templates")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["templates"][0]["id"] == "ai/patrol.gd"
    assert data["templates"][0]["source_kind"] == "template"

    response = client.get("/api/godot-agent/templates/ai/patrol.gd")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["data"]["id"] == "ai/patrol.gd"
    assert data["data"]["type"] == "template"


def test_nightly_regression_dashboard_route():
    _install_fake_nightly_status_provider()

    response = client.get("/api/nightly/regressions", params={"limit": 3})
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "healthy"
    assert data["latest_run"]["run_number"] == 77
    assert len(data["recent_runs"]) == 1
    assert data["jobs"]["smoke"]["conclusion"] == "success"
    assert data["job_catalog"]["smoke"]["artifact_name"] == "smoke-artifacts"


def test_distributed_monitor_prunes_stale_actors() -> None:
    monitor = DistributedMonitor()
    monitor.actor_ttl_seconds = 5.0
    monitor.actors = {
        "fresh": {
            "id": "fresh",
            "status": "active",
            "last_seen": "2026-04-01T10:00:10",
            "data": {},
        },
        "stale": {
            "id": "stale",
            "status": "active",
            "last_seen": "2026-04-01T09:59:00",
            "data": {},
        },
    }

    class FakeDateTime:
        @staticmethod
        def now():
            from datetime import datetime

            return datetime.fromisoformat("2026-04-01T10:00:12")

        @staticmethod
        def fromisoformat(value):
            from datetime import datetime

            return datetime.fromisoformat(value)

    import web_panel.distributed_monitor as distributed_monitor_module

    original_datetime = distributed_monitor_module.datetime
    distributed_monitor_module.datetime = FakeDateTime
    try:
        snapshot = monitor.snapshot()
        capabilities = monitor.capabilities()
    finally:
        distributed_monitor_module.datetime = original_datetime

    assert "fresh" in snapshot
    assert "stale" not in snapshot
    assert capabilities["actors_count"] == 1
    assert capabilities["last_pruned_at"] is not None
    assert capabilities["actor_ttl_seconds"] == 5.0
