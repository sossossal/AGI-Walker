import asyncio
import json

from agi_walker.mcp.server import (
    build_initialization_options,
    build_tool_list,
    call_tool,
    create_server,
    get_tool_definitions,
)


class FakeProvider:
    async def execute_mission(self, instruction: str):
        return {"status": "success", "instruction": instruction}

    def get_telemetry(self):
        return {"status": "success", "telemetry": {"cpu_percent": 12.5}}

    def query_rag(self, orient, top_k=1):
        return {"status": "success", "query": {"orient": orient, "top_k": top_k}}

    def list_workflows(self):
        return {"status": "success", "count": 1, "workflows": [{"name": "wf"}]}

    def get_workflow(self, name: str):
        return {"status": "success", "workflow": {"name": name}}

    def execute_workflow(self, name: str, parameters=None, use_real=None):
        return {"status": "success", "workflow_name": name, "parameters": parameters}

    def list_skills(self):
        return {
            "status": "success",
            "count": 1,
            "skills": [{"name": "robot-modeling"}],
        }

    def get_skill(self, name: str, include_doc: bool = False):
        return {
            "status": "success",
            "skill": {"name": name, "include_doc": include_doc},
        }

    def get_godot_agent_status(self):
        return {"status": "ready", "backend_mode": "fake"}

    def list_godot_templates(self):
        return {"status": "success", "templates": [{"id": "ai/patrol.gd"}]}

    def plan_godot_command(self, command: str, context=None, project_path=None):
        return {
            "status": "awaiting_confirmation",
            "command": command,
            "project_path": project_path,
        }

    def doctor_godot_agent(self, project_path=None):
        return {"status": "success", "ok": True, "project_path": project_path}

    def get_godot_history(self, limit=20):
        return {"status": "success", "count": 1, "limit": limit}

    def get_capability_matrix(self):
        return {
            "schema_version": "1.0",
            "artifact_type": "capability_matrix",
            "summary": {"total_domains": 5},
        }


def test_build_tool_list_contains_extended_surface() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())
    tools = build_tool_list(tool_definitions)
    names = {tool.name for tool in tools}

    assert "mission_execute" in names
    assert "workflow_execute" in names
    assert "skills_list" in names
    assert "godot_agent_status" in names
    assert "capability_matrix_get" in names
    assert len(names) == 14


def test_call_tool_returns_json_text_content() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(
        call_tool(
            tool_definitions,
            "godot_agent_plan",
            {"command": "生成巡逻脚本", "project_path": "D:/tmp/project"},
        )
    )

    assert len(result) == 1
    payload = json.loads(result[0].text)
    assert payload["status"] == "awaiting_confirmation"
    assert payload["project_path"] == "D:/tmp/project"


def test_capability_matrix_tool_returns_contract_payload() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    result = asyncio.run(call_tool(tool_definitions, "capability_matrix_get", {}))

    payload = json.loads(result[0].text)
    assert payload["artifact_type"] == "capability_matrix"
    assert payload["summary"]["total_domains"] == 5


def test_call_tool_rejects_unknown_name() -> None:
    tool_definitions = get_tool_definitions(FakeProvider())

    try:
        asyncio.run(call_tool(tool_definitions, "missing_tool", {}))
    except ValueError as exc:
        assert "Unknown tool" in str(exc)
    else:
        raise AssertionError("Expected ValueError for unknown tool")


def test_build_initialization_options_exposes_tools_capability() -> None:
    server = create_server(FakeProvider())

    options = build_initialization_options(server)

    assert options.server_name == "agi-walker-control"
    assert options.server_version == "3.0.0"
    assert options.capabilities.tools is not None
