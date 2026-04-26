from fastapi.testclient import TestClient
from unittest.mock import patch

from web_panel.server import app
from web_panel.distributed_monitor import DistributedMonitor, _payload_to_bytes

client = TestClient(app)


class FakeZenohPayload:
    def __init__(self, value: bytes) -> None:
        self.value = value

    def to_bytes(self) -> bytes:
        return self.value


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
            "tracked_jobs": [
                "smoke",
                "distributed-smoke",
                "godot-headless-smoke",
                "ros2-bridge-smoke",
            ],
            "latest_run": {
                "id": 42,
                "run_number": 77,
                "event": "schedule",
                "status": "completed",
                "conclusion": "success",
                "run_started_at": "2026-04-01T02:00:00",
            },
            "summary": {
                "tracked_jobs": 4,
                "passed_jobs": 4,
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
                "ros2-bridge-smoke": {
                    "name": "ros2-bridge-smoke",
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
                "local_repro_command": 'AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live"',
            },
            "ros2-bridge-smoke": {
                "artifact_name": "ros2-bridge-smoke-artifacts",
                "local_repro_command": 'AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live"',
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
    assert "Release Control Plane" in response.text
    assert 'id="release-control-plane-status"' in response.text
    assert 'id="release-control-plane-counts"' in response.text
    assert 'id="release-control-plane-profiles"' in response.text
    assert 'id="release-control-plane-next-link"' in response.text
    assert 'id="release-control-plane-request-file-link"' in response.text
    assert 'id="release-next-link"' in response.text
    assert 'id="release-next-primary-link"' in response.text
    assert 'id="release-next-primary-json-link"' in response.text
    assert 'id="release-next-follow-up-link"' in response.text
    assert 'id="release-next-follow-up-json-link"' in response.text
    assert 'id="release-next-follow-up-download-link"' in response.text
    assert 'id="release-next-request-file-link"' in response.text
    assert "Release Closeout" in response.text
    assert 'id="release-closeout-status"' in response.text
    assert 'id="release-closeout-counts"' in response.text
    assert 'id="release-closeout-route"' in response.text
    assert 'id="release-closeout-next-link"' in response.text
    assert 'id="release-closeout-next-json-link"' in response.text
    assert 'id="release-closeout-command"' in response.text

    response = client.get("/static/nightly.html")
    assert response.status_code == 200
    assert "Nightly 运维页" in response.text

    response = client.get("/static/release-closeout.html")
    assert response.status_code == 200
    assert "Release Closeout 详情页" in response.text
    assert 'id="closeout-next-json-link"' in response.text
    assert 'id="closeout-plan-link"' in response.text
    assert 'id="closeout-plan-json-link"' in response.text

    response = client.get("/static/release-closeout-plan.html")
    assert response.status_code == 200
    assert "Release Closeout 执行计划" in response.text
    assert 'id="closeout-plan-json-link"' in response.text
    assert 'id="closeout-plan-next-link"' in response.text
    assert 'id="closeout-plan-next-json-link"' in response.text
    assert 'id="closeout-plan-next-stage-link"' in response.text

    response = client.get("/static/release-control-plane.html")
    assert response.status_code == 200
    assert "Release Control Plane 详情页" in response.text
    assert 'id="next-action-json-link"' in response.text
    assert 'id="action-template-select"' in response.text
    assert 'id="action-template-json-link"' in response.text
    assert 'id="action-request-file-link"' in response.text
    assert 'id="action-request-file-download"' in response.text
    assert 'id="action-request-file-copy"' in response.text

    response = client.get("/static/release-next.html")
    assert response.status_code == 200
    assert "Release Next 详情页" in response.text
    assert 'id="release-next-json-link"' in response.text
    assert 'id="release-next-primary-json-link"' in response.text
    assert 'id="release-next-follow-up-json-link"' in response.text
    assert 'id="release-next-request-file-json-link"' in response.text
    assert 'id="release-next-follow-up-link"' in response.text
    assert 'id="release-next-follow-up-download-link"' in response.text
    assert 'id="release-next-primary-link"' in response.text

    response = client.get("/static/instruction-control.html")
    assert response.status_code == 200
    assert "Instruction Control Console" in response.text
    assert 'id="send-instruction-button"' in response.text
    assert 'id="send-circuit-button"' in response.text
    assert 'id="refresh-status-button"' in response.text
    assert 'id="connect-ws-button"' in response.text
    assert 'id="history-operator"' in response.text
    assert 'id="history-tag"' in response.text
    assert 'id="history-note"' in response.text
    assert 'id="instruction-response"' in response.text
    assert 'id="instruction-runtime-response"' in response.text
    assert 'id="circuit-config-response"' in response.text
    assert 'id="session-status-response"' in response.text
    assert 'id="instruction-ws-status"' in response.text
    assert 'id="runtime-sequence-name"' in response.text
    assert 'id="runtime-step-count"' in response.text
    assert 'id="runtime-circuit-transport"' in response.text
    assert 'id="runtime-feedback-nodes"' in response.text
    assert "/static/operator-history.html" in response.text

    response = client.get("/static/operator-history.html")
    assert response.status_code == 200
    assert "Operator History Console" in response.text
    assert 'id="instruction-history-list"' in response.text
    assert 'id="instruction-history-empty"' in response.text
    assert 'id="clear-history-button"' in response.text
    assert 'id="apply-history-filters-button"' in response.text
    assert 'id="history-summary-response"' in response.text
    assert 'id="history-total-entries"' in response.text
    assert 'id="history-session-count"' in response.text
    assert 'id="history-kind-summary"' in response.text
    assert 'id="history-route-summary"' in response.text
    assert 'id="history-prev-button"' in response.text
    assert 'id="history-next-button"' in response.text
    assert 'id="history-page-status"' in response.text
    assert 'id="history-kind-filter"' in response.text
    assert 'id="history-route-mode-filter"' in response.text
    assert 'id="history-all-sessions-filter"' in response.text
    assert 'id="history-created-after-filter"' in response.text
    assert 'id="history-created-before-filter"' in response.text
    assert 'id="history-session-query-filter"' in response.text
    assert 'id="history-operator-filter"' in response.text
    assert 'id="history-tag-filter"' in response.text
    assert 'id="history-note-filter"' in response.text
    assert 'id="history-note-exact-filter"' in response.text
    assert 'id="history-sort-by-filter"' in response.text
    assert 'id="history-sort-order-filter"' in response.text
    assert 'id="export-history-json-button"' in response.text
    assert 'id="export-history-csv-button"' in response.text
    assert "/static/operator-history-timeline.html" in response.text
    assert "/api/godot/${encodeURIComponent(currentSessionId())}/history" in response.text
    assert "/api/godot/history" in response.text
    assert "/api/godot/history/summary" in response.text
    assert "/api/godot/${encodeURIComponent(currentSessionId())}/history/replay" in response.text
    assert "/api/godot/history/export?format=" in response.text

    response = client.get("/static/operator-history-timeline.html")
    assert response.status_code == 200
    assert "Operator History Timeline" in response.text
    assert 'id="timeline-session-query"' in response.text
    assert 'id="timeline-operator-filter"' in response.text
    assert 'id="timeline-tag-filter"' in response.text
    assert 'id="timeline-note-filter"' in response.text
    assert 'id="timeline-note-exact-filter"' in response.text
    assert 'id="timeline-kind-filter"' in response.text
    assert 'id="timeline-route-mode-filter"' in response.text
    assert 'id="timeline-created-after"' in response.text
    assert 'id="timeline-created-before"' in response.text
    assert 'id="timeline-sort-order"' in response.text
    assert 'id="timeline-apply-button"' in response.text
    assert 'id="timeline-export-json-button"' in response.text
    assert 'id="timeline-export-csv-button"' in response.text
    assert 'id="timeline-clear-compare-button"' in response.text
    assert 'id="timeline-summary-response"' in response.text
    assert 'id="timeline-groups"' in response.text
    assert 'id="timeline-compare-left"' in response.text
    assert 'id="timeline-compare-right"' in response.text
    assert 'id="timeline-compare-diff"' in response.text
    assert "/api/godot/history/summary" in response.text
    assert "/api/godot/history/export?format=json" in response.text

    assert "🧭 Control Plane 详情" in client.get("/").text
    assert "📦 Release 收口页" in client.get("/").text
    assert "🧩 Release 下一步" in client.get("/").text
    assert "🎮 Instruction Console" in client.get("/").text
    assert "🗂️ Operator History" in client.get("/").text
    assert "🕒 Operator Timeline" in client.get("/").text

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
    assert data["distributed_monitor"]["schema_version"] == "1.0"
    assert data["distributed_monitor"]["subscription"] == "ag/*/obs"
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
    assert data["release_control_plane"]["route"] == "/api/release/control-plane"
    assert data["release_control_plane"]["actions_count"] >= 1
    assert data["release_control_plane"]["policy_profiles_count"] >= 1
    assert data["release_control_plane"]["request_templates_count"] >= 1
    assert data["release_control_plane"]["next_action"]
    assert data["release_control_plane"]["next_action_route"].startswith(
        "/static/release-control-plane.html?action="
    )
    assert data["release_control_plane"]["next_action_request_file_route"].startswith(
        "/api/release/control-plane/request-file?action="
    )
    assert data["release_control_plane"][
        "next_action_request_file_download_route"
    ].startswith("/api/release/control-plane/request-file?action=")
    assert data["release_control_plane"]["next_action_request_file_name"].startswith(
        "release_ops."
    )
    assert data["release_control_plane"]["release_closeout_status"] in {
        "action_required",
        "blocked",
        "ready",
        "missing",
    }
    assert data["release_next"]["route"] == "/api/release/next"
    assert data["release_next"]["portal_route"] == "/static/release-next.html"
    assert data["release_next"]["primary_route"] == "/api/release/next/primary"
    assert data["release_next"]["primary_payload_route"] == "/api/release/next/primary"
    assert data["release_next"]["primary_kind"] in {
        "closeout_component",
        "control_plane_action",
    }
    assert data["release_next"]["primary_portal_route"].startswith("/static/")
    assert data["release_next"]["primary_api_route"].startswith("/api/release/")
    assert data["release_next"]["primary_next_route"].startswith("/api/release/")
    assert data["release_next"]["primary_follow_up_kind"] in {"command", "request_file", "json"}
    assert data["release_next"]["primary_follow_up_label"]
    assert data["release_next"]["primary_follow_up_payload_route"] == "/api/release/next/follow-up"
    assert data["release_next"]["primary_follow_up_text"]
    assert "primary_follow_up_download_route" in data["release_next"]
    assert data["release_next"]["request_file_route"] == "/api/release/next/request-file"
    assert data["release_next"]["request_file_payload_route"] == "/api/release/next/request-file"
    assert data["release_next"]["request_file_status"] in {"success", "missing"}
    assert "request_file_name" in data["release_next"] or data["release_next"]["request_file_status"] == "missing"
    assert "request_file_download_route" in data["release_next"]
    assert data["release_closeout"]["route"] == "/api/release/closeout"
    assert data["release_closeout"]["status"] in {
        "action_required",
        "blocked",
        "ready",
        "missing",
    }
    assert "action_items_count" in data["release_closeout"]
    assert isinstance(data["release_closeout"]["top_action_items"], list)
    assert data["release_closeout"]["next_component_route"].startswith(
        "/static/release-closeout.html?component="
    )
    assert data["release_closeout"]["next_component_api_route"].startswith(
        "/api/release/closeout/component?component="
    )
    assert data["release_closeout"]["next_component_next_route"] == (
        "/api/release/closeout/next"
    )
    assert "blocked_components" in data["release_closeout"]
    assert "waiting_external_input_components" in data["release_closeout"]
    assert "ready_to_run_components" in data["release_closeout"]
    assert "missing_components" in data["release_closeout"]
    assert data["capability_matrix"]["artifact_type"] == "capability_matrix"
    assert data["capability_matrix"]["route"] == "/api/capabilities/matrix"
    assert data["capability_matrix"]["summary"]["total_domains"] == 5

    response = client.get("/api/distributed/status")
    assert response.status_code == 200
    data = response.json()
    assert data["schema_version"] == "1.0"
    assert "actors" in data
    assert "actor_ids" in data
    assert "actors_count" in data
    assert "monitor" in data
    assert "zenoh_available" in data["monitor"]
    assert "endpoint" in data["monitor"]

    response = client.get("/api/godot/capabilities")
    assert response.status_code == 200
    data = response.json()
    assert data["preferred_mode"] == "session_bridge"
    assert "legacy_controller" in data["modes"]
    assert "session_bridge" in data["modes"]

    response = client.get("/api/capabilities/matrix")
    assert response.status_code == 200
    data = response.json()
    assert data["artifact_type"] == "capability_matrix"
    assert data["summary"]["total_domains"] == 5
    assert any(domain["id"] == "distributed_runtime" for domain in data["domains"])

    response = client.get("/api/release/control-plane")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/control-plane"
    assert "control_plane_surface" in data
    assert data["actions_count"] >= 1
    assert data["policy_profiles_count"] >= 1
    assert data["request_templates_count"] >= 1
    assert data["next_action"]
    assert data["next_action_request_file_route"].startswith(
        "/api/release/control-plane/request-file?action="
    )
    assert data["next_action_request_file_name"].startswith("release_ops.")
    assert any(
        item.get("action") == "external_mainline_execution"
        for item in data["actions"]
    )
    assert any(
        item.get("action") == "external_mainline_execution"
        for item in data["request_templates"]
    )

    response = client.get("/api/release/control-plane/surface")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/control-plane/surface"
    assert "control_plane_surface" in data
    assert data["source"] in {"release_manifest", "release_ops_execution_report"}

    response = client.get("/api/release/next")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/next"
    assert data["portal_route"] == "/static/release-next.html"
    assert data["primary_route"] == "/api/release/next/primary"
    assert data["primary_payload_route"] == "/api/release/next/primary"
    assert data["primary_payload"]["route"] == "/api/release/next/primary"
    assert data["follow_up_payload"]["route"] == "/api/release/next/follow-up"
    assert data["request_file_payload"]["route"] == "/api/release/next/request-file"
    assert data["request_file_payload"]["release_next_follow_up_route"] == "/api/release/next/follow-up"
    assert data["control_plane_next"]["route"] == "/api/release/control-plane/next"
    assert data["release_closeout_next"]["route"] == "/api/release/closeout/next"
    assert data["primary_kind"] in {"closeout_component", "control_plane_action"}
    assert data["primary_name"]
    assert data["primary_next_route"].startswith("/api/release/")

    response = client.get("/api/release/next/primary")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/next/primary"
    assert data["primary_payload_route"] == "/api/release/next/primary"
    assert data["release_next_route"] == "/api/release/next"
    assert data["release_next_portal_route"] == "/static/release-next.html"
    assert data["primary_kind"] in {"closeout_component", "control_plane_action"}
    assert data["primary_name"]
    assert data["primary_follow_up_kind"] in {"command", "request_file", "json"}
    assert data["primary_follow_up_label"]
    assert data["primary_follow_up_text"]

    response = client.get("/api/release/next/follow-up")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/next/follow-up"
    assert data["release_next_route"] == "/api/release/next"
    assert data["release_next_primary_route"] == "/api/release/next/primary"
    assert data["follow_up_kind"] in {"command", "request_file", "json"}
    assert data["follow_up_label"]
    assert data["follow_up_text"]

    response = client.get("/api/release/next/request-file")
    assert response.status_code == 200
    data = response.json()
    assert data["route"] == "/api/release/next/request-file"
    assert data["release_next_route"] == "/api/release/next"
    assert data["release_next_primary_route"] == "/api/release/next/primary"
    assert "request_file_download_route" in data

    with patch(
        "web_panel.core_api.build_release_next_request_file_payload",
        return_value={
            "status": "success",
            "route": "/api/release/next/request-file",
            "release_next_route": "/api/release/next",
            "release_next_primary_route": "/api/release/next/primary",
            "release_next_follow_up_route": "/api/release/next/follow-up",
            "release_next_portal_route": "/static/release-next.html",
            "request_file_name": "release_ops.release_readiness.request.json",
            "content_type": "application/json",
            "request_file_pretty_json": '{\n  "action": "release_readiness"\n}',
            "request_file_download_route": "/api/release/next/request-file?download=1",
        },
    ):
        response = client.get("/api/release/next/request-file?download=1")
    assert response.status_code == 200
    assert response.headers["content-type"].startswith("application/json")
    assert (
        response.headers["content-disposition"]
        == 'attachment; filename=release_ops.release_readiness.request.json'
    )
    assert '"action": "release_readiness"' in response.text

    response = client.get("/api/release/closeout")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/closeout"
    assert data["release_closeout"]["status"] in {"action_required", "blocked", "ready"}
    assert isinstance(data["release_closeout"]["action_items"], list)
    assert "external_mainline" in data["release_closeout"]
    assert "vulnerability_exception_review" in data["release_closeout"]
    assert "worktree_release_blocker" in data["release_closeout"]
    assert data["release_closeout"]["action_items"][0]["component_route"].startswith(
        "/static/release-closeout.html?component="
    )
    assert data["release_closeout"]["action_items"][0]["component_api_route"].startswith(
        "/api/release/closeout/component?component="
    )
    assert data["release_closeout"]["next_component_next_route"] == (
        "/api/release/closeout/next"
    )

    response = client.get("/api/release/closeout/plan")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/closeout/plan"
    assert data["portal_route"] == "/static/release-closeout-plan.html"
    assert data["plan_status"] in {"action_required", "blocked", "ready"}
    assert data["next_route"] == "/api/release/closeout/plan/next"
    assert data["next_stage_route"].startswith("/static/release-closeout-plan.html?stage=")
    assert data["next_stage_api_route"].startswith("/api/release/closeout/plan/stage?stage=")
    assert data["next_stage_next_route"] == "/api/release/closeout/plan/next"
    assert isinstance(data["stages"], list)
    assert len(data["stages"]) >= 1
    assert data["stages"][0]["stage_route"].startswith("/static/release-closeout-plan.html?stage=")
    assert data["stages"][0]["stage_api_route"].startswith("/api/release/closeout/plan/stage?stage=")

    response = client.get(
        "/api/release/closeout/plan/stage",
        params={"stage": "customer_external_bindings_inputs"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/closeout/plan/stage"
    assert data["stage"] == "customer_external_bindings_inputs"
    assert data["stage_api_route"].endswith(
        "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs"
    )
    assert data["stage_payload"]["id"] == "customer_external_bindings_inputs"

    response = client.get("/api/release/closeout/plan/next")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/closeout/plan/next"
    assert data["next_stage_id"]
    assert data["next_stage_api_route"].startswith("/api/release/closeout/plan/stage?stage=")
    assert data["stage_payload"]["id"] == data["next_stage_id"]

    response = client.get("/api/release/closeout/next")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/closeout/next"
    assert data["next_component"]
    assert data["component_route"].endswith(
        f"/static/release-closeout.html?component={data['next_component']}"
    )
    assert data["component_api_route"].endswith(
        f"/api/release/closeout/component?component={data['next_component']}"
    )
    assert data["action_item"]["component"] == data["next_component"]

    response = client.get(
        "/api/release/closeout/component",
        params={"component": "external_mainline"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/closeout/component"
    assert data["component"] == "external_mainline"
    assert data["component_route"].endswith(
        "/static/release-closeout.html?component=external_mainline"
    )
    assert data["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )
    assert data["action_item"]["component"] == "external_mainline"
    assert data["action_item"]["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )

    response = client.get("/api/release/control-plane/catalog")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/control-plane/catalog"
    assert data["actions_count"] >= 1
    assert data["policy_profiles_count"] >= 1
    assert any(
        item["portal_route"].startswith("/static/release-control-plane.html?action=")
        for item in data["actions"]
    )

    response = client.get(
        "/api/release/control-plane/request-templates",
        params={"action": "external_mainline_execution"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/control-plane/request-templates"
    assert data["request_templates_count"] == 1
    assert data["request_templates"][0]["action"] == "external_mainline_execution"
    assert data["request_templates"][0]["action_route"].endswith(
        "/api/release/control-plane/action?action=external_mainline_execution"
    )
    assert data["request_templates"][0]["request_template_route"].endswith(
        "/api/release/control-plane/request-templates?action=external_mainline_execution"
    )
    assert data["request_templates"][0]["portal_route"].endswith(
        "/static/release-control-plane.html?action=external_mainline_execution"
    )
    assert data["request_templates"][0]["request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution"
    )
    assert data["request_templates"][0]["request_file_download_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution&download=1"
    )
    assert data["request_templates"][0]["request_file_name"] == (
        "release_ops.external_mainline_execution.request.json"
    )

    response = client.get(
        "/api/release/control-plane/action",
        params={"action": "external_mainline_execution"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/control-plane/action"
    assert data["action"] == "external_mainline_execution"
    assert data["action_definition"]["action"] == "external_mainline_execution"
    assert data["request_template"]["action"] == "external_mainline_execution"
    assert data["action_route"].endswith(
        "/api/release/control-plane/action?action=external_mainline_execution"
    )
    assert data["portal_route"].endswith(
        "/static/release-control-plane.html?action=external_mainline_execution"
    )
    assert data["request_template_route"].endswith(
        "/api/release/control-plane/request-templates?action=external_mainline_execution"
    )
    assert data["request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution"
    )
    assert data["request_file_download_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution&download=1"
    )
    assert data["request_file_name"] == (
        "release_ops.external_mainline_execution.request.json"
    )

    response = client.get("/api/release/control-plane/next")
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/control-plane/next"
    assert data["next_action"]
    assert data["action_definition"]["action"] == data["next_action"]
    assert data["request_template"]["action"] == data["next_action"]
    assert data["request_file_route"].endswith(
        f"/api/release/control-plane/request-file?action={data['next_action']}"
    )
    assert data["request_file_download_route"].endswith(
        f"/api/release/control-plane/request-file?action={data['next_action']}&download=1"
    )
    assert data["request_file_payload"]["request_file_name"] == data["request_file_name"]

    response = client.get(
        "/api/release/control-plane/request-file",
        params={"action": "external_mainline_execution"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "success"
    assert data["route"] == "/api/release/control-plane/request-file"
    assert data["action"] == "external_mainline_execution"
    assert data["request_file_route"].endswith(
        "/api/release/control-plane/request-file?action=external_mainline_execution"
    )
    assert data["request_file_name"] == (
        "release_ops.external_mainline_execution.request.json"
    )
    assert data["request_file"]["project_root"] == "."

    response = client.get(
        "/api/release/control-plane/request-file",
        params={"action": "external_mainline_execution", "download": "true"},
    )
    assert response.status_code == 200
    assert "application/json" in response.headers["content-type"]
    assert (
        response.headers["content-disposition"]
        == 'attachment; filename="release_ops.external_mainline_execution.request.json"'
    )
    assert '"project_root": "."' in response.text

    response = client.get(
        "/api/release/control-plane/request-templates",
        params={"action": "missing_action"},
    )
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "error"
    assert data["action"] == "missing_action"


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
    assert status_data["nightly_regressions"]["summary"]["passed_jobs"] == 4

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


def test_distributed_monitor_payload_to_bytes_accepts_zenoh_and_mock_payloads() -> None:
    assert _payload_to_bytes(b"raw") == b"raw"
    assert _payload_to_bytes("raw") == b"raw"
    assert _payload_to_bytes(FakeZenohPayload(b"raw")) == b"raw"
