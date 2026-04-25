import asyncio
import json
from pathlib import Path

from agi_walker.core.api.mcp_tools import MCPToolProvider
from agi_walker.core.api.release_control_plane import (
    build_release_next_request_file_payload,
)
from agi_walker.core.api.release_contracts import (
    build_release_evidence_report,
    write_release_evidence_report,
)
from agi_walker.skills_loader import SkillMetadata


class FakeWorkflowResult:
    def __init__(self, status: str = "completed") -> None:
        self.status = type("Status", (), {"value": status})()

    def to_dict(self):
        return {"status": self.status.value, "steps": []}


class FakeOrchestrator:
    def list_workflows(self):
        return ["robot_creation_pipeline"]

    def get_workflow(self, name: str):
        if name != "robot_creation_pipeline":
            return None
        return {
            "name": name,
            "description": "Create a robot",
            "steps": [{"name": "create_model"}],
        }

    def execute_workflow(self, name: str, parameters=None, use_real=None):
        assert name == "robot_creation_pipeline"
        assert parameters == {"output_root": "tmp"}
        assert use_real is False
        return FakeWorkflowResult()

    def execute_task_graph(self, graph):
        return FakeWorkflowResult()


class FakePlanner:
    def plan(self, instruction: str):
        return {"instruction": instruction}


class FakeMonitor:
    def get_hw_stats(self):
        return {"cpu_percent": 10.0, "memory_percent": 20.0}


class FakeKnowledgeBase:
    def get_stats(self):
        return {"knowledge_count": 2, "experience_count": 1}

    def retrieve_experience(self, sensor_data, top_k=1):
        return [
            type(
                "Experience",
                (),
                {
                    "id": "exp-1",
                    "scenario": "recovery",
                    "outcome": "success",
                    "state_pattern": sensor_data["sensors"]["imu"]["orient"],
                    "action_ref": [0.5, 0.5],
                    "source_file": "traj.json",
                },
            )()
        ][:top_k]


class FakeSkillsLoader:
    def __init__(self) -> None:
        self.skill = SkillMetadata(
            name="robot-modeling",
            description="Build robots",
            category="建模",
            emoji="🤖",
            skill_dir=Path("agi_walker/skills/robot-modeling"),
        )

    def get_skills_list(self):
        return [self.skill]

    def get_skill(self, name: str):
        if name == self.skill.name:
            return self.skill
        return None

    def get_skill_doc(self, name: str):
        assert name == self.skill.name
        return "## Robot Modeling"


class FakeGodotBackend:
    default_project_path = "D:/tmp/project"
    history_file = Path("D:/tmp/history.json")
    agent_dir = Path("D:/tmp/godot-agent")
    router = object()

    def get_roles_info(self):
        return [{"name": "developer"}]

    def list_templates(self):
        return {
            "status": "success",
            "backend_mode": "fake",
            "templates": [{"id": "ai/patrol.gd", "type": "template"}],
        }

    def plan_command(self, command: str, context=None, project_path=None):
        return {
            "status": "awaiting_confirmation",
            "command": command,
            "context": context or {},
            "project_path": project_path,
        }

    def doctor(self, project_path=None):
        return {"status": "success", "ok": True, "project_path": project_path}

    def get_history(self, limit=20):
        return {"status": "success", "count": 1, "limit": limit, "items": []}


def build_provider() -> MCPToolProvider:
    return MCPToolProvider(
        orchestrator=FakeOrchestrator(),
        planner=FakePlanner(),
        monitor=FakeMonitor(),
        knowledge_base=FakeKnowledgeBase(),
        skills_loader=FakeSkillsLoader(),
        godot_backend_factory=lambda: FakeGodotBackend(),
    )


def test_execute_mission_returns_structured_payload() -> None:
    provider = build_provider()

    result = asyncio.run(provider.execute_mission("inspect area"))

    assert result["status"] == "success"
    assert result["instruction"] == "inspect area"
    assert result["result"]["status"] == "completed"


def test_workflow_and_skill_queries_are_exposed() -> None:
    provider = build_provider()

    workflows = provider.list_workflows()
    workflow = provider.get_workflow("robot_creation_pipeline")
    executed = provider.execute_workflow(
        "robot_creation_pipeline",
        parameters={"output_root": "tmp"},
        use_real=False,
    )
    skills = provider.list_skills()
    skill = provider.get_skill("robot-modeling", include_doc=True)

    assert workflows["count"] == 1
    assert workflow["workflow"]["name"] == "robot_creation_pipeline"
    assert executed["result"]["status"] == "completed"
    assert skills["skills"][0]["name"] == "robot-modeling"
    assert skill["skill"]["doc"] == "## Robot Modeling"


def test_godot_and_telemetry_queries_are_exposed() -> None:
    provider = build_provider()

    telemetry = provider.get_telemetry()
    rag = provider.query_rag([0.1, 0.2, 0.3], top_k=1)
    capability_matrix = provider.get_capability_matrix()
    status = provider.get_godot_agent_status()
    templates = provider.list_godot_templates()
    plan = provider.plan_godot_command(
        "生成玩家移动脚本",
        context={"mode": "preview"},
        project_path="D:/tmp/project",
    )
    doctor = provider.doctor_godot_agent("D:/tmp/project")
    history = provider.get_godot_history(limit=5)

    assert telemetry["telemetry"]["rag"]["knowledge_count"] == 2
    assert rag["matches"][0]["id"] == "exp-1"
    assert capability_matrix["artifact_type"] == "capability_matrix"
    assert capability_matrix["summary"]["total_domains"] == 5
    assert status["backend_mode"] == "fake"
    assert status["templates_count"] == 1
    assert templates["templates"][0]["id"] == "ai/patrol.gd"
    assert plan["project_path"] == "D:/tmp/project"
    assert doctor["ok"] is True
    assert history["limit"] == 5


def test_release_control_plane_surface_query_exposes_canonical_surface(
    tmp_path: Path,
) -> None:
    provider = build_provider()
    report_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "release_ops_execution_report.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="release_ops_execution",
            status="passed",
            summary="release op stable_promotion_checklist completed via control plane.",
            command="release_ops::stable_promotion_checklist",
            generated_at="2026-04-21T10:00:00+00:00",
            metrics={
                "action": "stable_promotion_checklist",
                "policy_level": "local_safe_refresh",
                "policy_profile": "local_safe_refresh",
                "request_type": "StablePromotionChecklistRequest",
                "event_count": 3,
            },
            control_plane_session={
                "engagement_id": "mcp-session",
                "window_id": "mcp-window",
                "change_ticket": "CHG-MCP",
                "channel": "ops-cli",
            },
            control_plane_event_stream={
                "path": "test_env/release_ops/mcp.jsonl",
                "event_count": 3,
            },
        ),
        report_path,
    )

    payload = provider.get_release_control_plane_surface(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/surface"
    assert payload["source"] == "release_ops_execution_report"
    assert payload["control_plane_surface"]["status"] == "passed"
    assert payload["control_plane_surface"]["event_count"] == 3
    assert (
        payload["control_plane_surface"]["release_ops_execution"]["status"] == "passed"
    )
    assert (
        payload["control_plane_surface"]["control_plane_event_stream"]["event_count"]
        == 3
    )


def test_release_ops_catalog_query_exposes_actions_and_policy_profiles() -> None:
    provider = build_provider()

    payload = provider.get_release_ops_catalog()

    assert payload["status"] == "success"
    assert payload["actions_count"] >= 1
    assert payload["policy_profiles_count"] >= 1
    assert any(item["action"] == "external_mainline_execution" for item in payload["actions"])
    assert any(
        item["policy_profile"] == "requires_attestation"
        for item in payload["policy_profiles"]
    )


def test_release_ops_request_templates_query_exposes_defaults() -> None:
    provider = build_provider()

    payload = provider.get_release_ops_request_templates(
        action="external_mainline_execution"
    )

    assert payload["status"] == "success"
    assert payload["request_templates_count"] == 1
    template = payload["request_templates"][0]
    assert template["action"] == "external_mainline_execution"
    assert template["policy_level"] == "requires_attestation"
    assert template["default_policy_profile"] == "local_safe_refresh"
    assert "project_root" in template["required_fields"]
    assert template["action_route"].endswith(
        "action?action=external_mainline_execution"
    )
    assert template["request_template_route"].endswith(
        "request-templates?action=external_mainline_execution"
    )
    assert template["portal_route"].endswith(
        "/static/release-control-plane.html?action=external_mainline_execution"
    )
    assert template["request_file_route"].endswith(
        "request-file?action=external_mainline_execution"
    )
    assert template["request_file_download_route"].endswith(
        "request-file?action=external_mainline_execution&download=1"
    )
    assert template["request_file_name"] == (
        "release_ops.external_mainline_execution.request.json"
    )
    assert template["request_template"]["output"].endswith(
        "external_mainline_execution_plan.json"
    )


def test_release_control_plane_action_query_exposes_single_action_detail() -> None:
    provider = build_provider()

    payload = provider.get_release_control_plane_action(
        action="external_mainline_execution"
    )

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/action"
    assert payload["action"] == "external_mainline_execution"
    assert payload["action_definition"]["policy_level"] == "requires_attestation"
    assert payload["request_template"]["action"] == "external_mainline_execution"
    assert payload["action_route"].endswith(
        "action?action=external_mainline_execution"
    )
    assert payload["portal_route"].endswith(
        "/static/release-control-plane.html?action=external_mainline_execution"
    )
    assert payload["request_template_route"].endswith(
        "request-templates?action=external_mainline_execution"
    )
    assert payload["request_file_route"].endswith(
        "request-file?action=external_mainline_execution"
    )
    assert payload["request_file_download_route"].endswith(
        "request-file?action=external_mainline_execution&download=1"
    )
    assert payload["request_file_name"] == (
        "release_ops.external_mainline_execution.request.json"
    )


def test_release_control_plane_next_query_exposes_recommended_action() -> None:
    provider = build_provider()

    payload = provider.get_release_control_plane_next()

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/next"
    assert payload["next_action"] == payload["action_definition"]["action"]
    assert payload["request_template"]["action"] == payload["next_action"]
    assert payload["request_file_route"].endswith(
        f"request-file?action={payload['next_action']}"
    )
    assert payload["request_file_download_route"].endswith(
        f"request-file?action={payload['next_action']}&download=1"
    )
    assert payload["request_file_name"] == (
        f"release_ops.{payload['next_action']}.request.json"
    )
    assert payload["request_file_payload"]["request_file_name"] == (
        f"release_ops.{payload['next_action']}.request.json"
    )


def test_release_control_plane_request_file_query_exposes_scaffold() -> None:
    provider = build_provider()

    payload = provider.get_release_control_plane_request_file(
        action="external_mainline_execution"
    )

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/control-plane/request-file"
    assert payload["action"] == "external_mainline_execution"
    assert payload["request_file_route"].endswith(
        "request-file?action=external_mainline_execution"
    )
    assert payload["request_file_download_route"].endswith(
        "request-file?action=external_mainline_execution&download=1"
    )
    assert payload["request_file_name"] == (
        "release_ops.external_mainline_execution.request.json"
    )
    assert payload["content_type"] == "application/json"
    assert payload["request_file"]["output"].endswith(
        "external_mainline_execution_plan.json"
    )
    assert '"project_root": "."' in payload["request_file_pretty_json"]


def test_release_control_plane_index_query_aggregates_surface_and_catalog(
    tmp_path: Path,
) -> None:
    provider = build_provider()
    report_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "release_ops_execution_report.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="release_ops_execution",
            status="passed",
            summary="release op stable_promotion_checklist completed via control plane.",
            command="release_ops::stable_promotion_checklist",
            generated_at="2026-04-22T10:00:00+00:00",
            metrics={
                "action": "stable_promotion_checklist",
                "policy_level": "local_safe_refresh",
                "policy_profile": "local_safe_refresh",
                "request_type": "StablePromotionChecklistRequest",
                "event_count": 3,
            },
            control_plane_session={
                "engagement_id": "mcp-index-session",
                "window_id": "mcp-index-window",
                "change_ticket": "CHG-MCP-INDEX",
                "channel": "ops-cli",
            },
            control_plane_event_stream={
                "path": "test_env/release_ops/mcp-index.jsonl",
                "event_count": 3,
            },
        ),
        report_path,
    )

    payload = provider.get_release_control_plane_index(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["control_plane_surface_source"] == "release_ops_execution_report"
    assert payload["control_plane_surface"]["status"] == "passed"
    assert payload["control_plane_surface"]["event_count"] == 3
    assert payload["release_closeout"]["status"] in {
        "action_required",
        "blocked",
        "ready",
    }
    assert payload["actions_count"] >= 1
    assert payload["policy_profiles_count"] >= 1
    assert payload["request_templates_count"] >= 1
    assert payload["next_action"]
    assert payload["next_action_request_file_route"].startswith(
        "/api/release/control-plane/request-file?action="
    )
    assert payload["next_action_request_file_download_route"].startswith(
        "/api/release/control-plane/request-file?action="
    )
    assert payload["next_action_request_file_name"].startswith("release_ops.")
    assert any(item["action"] == "external_mainline_execution" for item in payload["actions"])
    assert any(
        item["action"] == "external_mainline_execution"
        for item in payload["request_templates"]
    )


def test_release_next_query_aggregates_control_plane_and_closeout(tmp_path: Path) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan waiting for customer input.",
                "completed_steps": 0,
                "ready_to_run_steps": 0,
                "waiting_external_input_steps": 1,
                "blocked_steps": 0,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_next(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/next"
    assert payload["portal_route"] == "/static/release-next.html"
    assert payload["control_plane_next"]["route"] == "/api/release/control-plane/next"
    assert payload["release_closeout_next"]["route"] == "/api/release/closeout/next"
    assert payload["primary_kind"] == "closeout_component"
    assert payload["primary_name"] == "external_mainline"
    assert payload["primary_next_route"] == "/api/release/closeout/next"
    assert payload["primary_payload"]["route"] == "/api/release/next/primary"
    assert payload["follow_up_payload"]["route"] == "/api/release/next/follow-up"
    assert payload["primary_portal_route"].endswith(
        "/static/release-closeout.html?component=external_mainline"
    )


def test_release_next_primary_query_returns_primary_entry(tmp_path: Path) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan waiting for customer input.",
                "completed_steps": 0,
                "ready_to_run_steps": 0,
                "waiting_external_input_steps": 1,
                "blocked_steps": 0,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_next_primary(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/next/primary"
    assert payload["primary_payload_route"] == "/api/release/next/primary"
    assert payload["release_next_route"] == "/api/release/next"
    assert payload["release_next_portal_route"] == "/static/release-next.html"
    assert payload["primary_kind"] == "closeout_component"
    assert payload["primary_name"] == "external_mainline"
    assert payload["primary_follow_up_kind"] == "command"
    assert payload["primary_follow_up_label"] == "建议命令"
    assert payload["primary_follow_up_route"] == "/api/release/closeout/next"
    assert "run_external_mainline_execution_plan.py" in payload["primary_follow_up_text"]


def test_release_next_follow_up_query_returns_normalized_follow_up(tmp_path: Path) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan waiting for customer input.",
                "completed_steps": 0,
                "ready_to_run_steps": 0,
                "waiting_external_input_steps": 1,
                "blocked_steps": 0,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_next_follow_up(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/next/follow-up"
    assert payload["release_next_route"] == "/api/release/next"
    assert payload["release_next_primary_route"] == "/api/release/next/primary"
    assert payload["follow_up_kind"] == "command"
    assert payload["follow_up_label"] == "建议命令"
    assert payload["follow_up_route"] == "/api/release/closeout/next"


def test_release_next_request_file_query_returns_missing_for_closeout_primary(
    tmp_path: Path,
) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan waiting for customer input.",
                "completed_steps": 0,
                "ready_to_run_steps": 0,
                "waiting_external_input_steps": 1,
                "blocked_steps": 0,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_next_request_file(project_root=str(tmp_path))

    assert payload["status"] == "missing"
    assert payload["route"] == "/api/release/next/request-file"
    assert payload["release_next_route"] == "/api/release/next"
    assert payload["release_next_primary_route"] == "/api/release/next/primary"


def test_release_next_request_file_query_returns_scaffold_for_control_plane_primary(
    tmp_path: Path,
) -> None:
    payload = build_release_next_request_file_payload(
        payload={
            "status": "success",
            "route": "/api/release/next",
            "portal_route": "/static/release-next.html",
            "primary_kind": "control_plane_action",
            "primary_name": "release_readiness",
            "primary_status": "local_safe_refresh",
            "primary_request_file_route": "/api/release/control-plane/request-file?action=release_readiness",
            "primary_request_file_download_route": "/api/release/control-plane/request-file?action=release_readiness&download=1",
            "primary_request_file_name": "release_ops.release_readiness.request.json",
        }
    )

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/next/request-file"
    assert payload["release_next_route"] == "/api/release/next"
    assert payload["release_next_primary_route"] == "/api/release/next/primary"
    assert payload["action"] == "release_readiness"
    assert payload["request_file_route"] == "/api/release/next/request-file"
    assert payload["request_file_download_route"] == "/api/release/next/request-file?download=1"
    assert payload["request_file_name"] == "release_ops.release_readiness.request.json"


def test_release_closeout_query_aggregates_remaining_blockers(tmp_path: Path) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan ready.",
                "completed_steps": 0,
                "ready_to_run_steps": 1,
                "waiting_external_input_steps": 1,
                "blocked_steps": 0,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    preflight_path = (
        tmp_path / "test_env" / "release_evidence" / "security_release_preflight_report.json"
    )
    preflight_path.parent.mkdir(parents=True, exist_ok=True)
    write_release_evidence_report(
        build_release_evidence_report(
            evidence_name="security_release_preflight",
            status="passed",
            summary="31 active exceptions are in review window.",
            command="python tools/run_security_release_preflight.py",
            metrics={
                "vulnerability_exception_review_status": "review_due",
                "vulnerability_exception_review_due": 31,
                "vulnerability_exception_review_candidate_count": 31,
            },
        ),
        preflight_path,
    )

    readiness_path = (
        tmp_path / "test_env" / "release_readiness_ready" / "release_readiness_report.json"
    )
    readiness_path.parent.mkdir(parents=True, exist_ok=True)
    readiness_path.write_text(
        json.dumps(
            {
                "worktree_release_blocker": {
                    "status": "blocked",
                    "summary": "clean_worktree=false, total_paths=2.",
                    "clean_worktree": False,
                    "total_paths": 2,
                    "tracked_review_candidate_count": 1,
                }
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_closeout(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout"
    assert payload["release_closeout"]["status"] == "blocked"
    assert len(payload["release_closeout"]["action_items"]) == 3
    assert payload["release_closeout"]["next_component"] == "external_mainline"
    assert payload["release_closeout"]["next_component_next_route"] == (
        "/api/release/closeout/next"
    )
    assert payload["release_closeout"]["external_mainline"]["status"] == "waiting_external_input"
    assert payload["release_closeout"]["external_mainline"]["command"]
    assert payload["release_closeout"]["action_items"][0]["component_route"].endswith(
        "/static/release-closeout.html?component=external_mainline"
    )
    assert payload["release_closeout"]["action_items"][0]["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )
    assert (
        payload["release_closeout"]["vulnerability_exception_review"]["status"]
        == "waiting_external_input"
    )
    assert payload["release_closeout"]["vulnerability_exception_review"]["command"]


def test_release_closeout_next_query_exposes_recommended_component(tmp_path: Path) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan waiting for customer input.",
                "completed_steps": 0,
                "ready_to_run_steps": 0,
                "waiting_external_input_steps": 1,
                "blocked_steps": 0,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_closeout_next(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout/next"
    assert payload["next_component"] == "external_mainline"
    assert payload["component_route"].endswith(
        "/static/release-closeout.html?component=external_mainline"
    )
    assert payload["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )
    assert payload["action_item"]["component"] == "external_mainline"
    assert payload["closeout_component"]["status"] == "waiting_external_input"
    assert payload["component_payload"]["component"] == "external_mainline"
    assert payload["component_payload"]["action_item"]["component"] == "external_mainline"


def test_release_closeout_plan_query_exposes_staged_runbook(tmp_path: Path) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan waiting for customer input.",
                "completed_steps": 0,
                "ready_to_run_steps": 0,
                "waiting_external_input_steps": 3,
                "blocked_steps": 0,
                "steps": [
                    {
                        "id": "customer_external_bindings_closure",
                        "status": "waiting_external_input",
                        "summary": "customer bindings still need confirmation inputs.",
                        "blocking_inputs": [
                            "confirmed_by",
                            "confirmation_ticket",
                        ],
                        "command": "python tools/run_customer_external_bindings_closure.py ...",
                    },
                    {
                        "id": "vulnerability_exception_replacement",
                        "status": "waiting_external_input",
                        "summary": "review evidence still needs replacement inputs.",
                        "blocking_inputs": ["最新 upstream fix 版本或重算后的 scanner 结果"],
                        "command": "python tools/build_vulnerability_exception_review_report.py ...",
                    },
                    {
                        "id": "industrial_delivery_live_evidence",
                        "status": "waiting_external_input",
                        "summary": "industrial live evidence still needs real environment inputs.",
                        "blocking_inputs": ["真实客户环境标识"],
                        "command": "python tools/run_release_rehearsal.py ...",
                    },
                ],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    inputs_path = tmp_path / "deployment" / "external_mainline.inputs.json"
    inputs_path.parent.mkdir(parents=True, exist_ok=True)
    inputs_path.write_text(
        json.dumps(
            {
                "customer_external_bindings": {
                    "config": "deployment/customer_delivery.external_bindings.customer.json",
                    "overrides_file": "deployment/customer_delivery.external_bindings.customer.overrides.json",
                    "sections": ["approval_identity", "archive_target", "due_trigger"],
                }
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_closeout_plan(project_root=str(tmp_path))

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout/plan"
    assert payload["portal_route"] == "/static/release-closeout-plan.html"
    assert payload["plan_status"] == "action_required"
    assert payload["next_stage_id"] == "customer_external_bindings_inputs"
    assert payload["next_stage_route"].endswith(
        "/static/release-closeout-plan.html?stage=customer_external_bindings_inputs"
    )
    assert len(payload["stages"]) == 5
    assert payload["stages"][0]["input_files"][0] == "deployment/external_mainline.inputs.json"
    assert payload["stages"][0]["commands"][0]["command"].startswith(
        "python tools/build_customer_external_bindings_config.py"
    )
    assert payload["stages"][0]["stage_api_route"].endswith(
        "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs"
    )
    assert payload["next_stage_next_route"] == "/api/release/closeout/plan/next"


def test_release_closeout_plan_stage_and_next_queries_expose_canonical_stage_routes(
    tmp_path: Path,
) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan waiting for customer input.",
                "completed_steps": 0,
                "ready_to_run_steps": 0,
                "waiting_external_input_steps": 3,
                "blocked_steps": 0,
                "steps": [
                    {
                        "id": "customer_external_bindings_closure",
                        "status": "waiting_external_input",
                        "summary": "customer bindings still need confirmation inputs.",
                        "blocking_inputs": ["confirmed_by", "confirmation_ticket"],
                    }
                ],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    stage_payload = provider.get_release_closeout_plan_stage(
        "customer_external_bindings_inputs",
        project_root=str(tmp_path),
    )
    next_payload = provider.get_release_closeout_plan_next(project_root=str(tmp_path))

    assert stage_payload["status"] == "success"
    assert stage_payload["route"] == "/api/release/closeout/plan/stage"
    assert stage_payload["stage"] == "customer_external_bindings_inputs"
    assert stage_payload["stage_api_route"].endswith(
        "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs"
    )
    assert stage_payload["plan_next_route"] == "/api/release/closeout/plan/next"
    assert stage_payload["stage_payload"]["component"] == "external_mainline"

    assert next_payload["status"] == "success"
    assert next_payload["route"] == "/api/release/closeout/plan/next"
    assert next_payload["next_stage_id"] == "customer_external_bindings_inputs"
    assert next_payload["next_stage_api_route"].endswith(
        "/api/release/closeout/plan/stage?stage=customer_external_bindings_inputs"
    )
    assert next_payload["stage_payload"]["id"] == "customer_external_bindings_inputs"


def test_release_closeout_component_query_exposes_single_component(
    tmp_path: Path,
) -> None:
    provider = build_provider()

    plan_path = (
        tmp_path
        / "test_env"
        / "release_evidence"
        / "operations"
        / "external_mainline_execution_plan.json"
    )
    plan_path.parent.mkdir(parents=True, exist_ok=True)
    plan_path.write_text(
        json.dumps(
            {
                "status": "ready",
                "summary": "External mainline execution plan ready.",
                "completed_steps": 0,
                "ready_to_run_steps": 1,
                "waiting_external_input_steps": 1,
                "blocked_steps": 0,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    payload = provider.get_release_closeout_component(
        component="external_mainline",
        project_root=str(tmp_path),
    )

    assert payload["status"] == "success"
    assert payload["route"] == "/api/release/closeout/component"
    assert payload["component"] == "external_mainline"
    assert payload["closeout_component"]["status"] == "waiting_external_input"
    assert payload["action_item"]["component"] == "external_mainline"
    assert payload["component_route"].endswith(
        "/static/release-closeout.html?component=external_mainline"
    )
    assert payload["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )
    assert payload["action_item"]["component_api_route"].endswith(
        "/api/release/closeout/component?component=external_mainline"
    )
