import asyncio
from pathlib import Path

from agi_walker.core.api.mcp_tools import MCPToolProvider
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
    assert status["backend_mode"] == "fake"
    assert status["templates_count"] == 1
    assert templates["templates"][0]["id"] == "ai/patrol.gd"
    assert plan["project_path"] == "D:/tmp/project"
    assert doctor["ok"] is True
    assert history["limit"] == 5
