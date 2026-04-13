import asyncio
import inspect
import json
import logging
import sys
from dataclasses import dataclass
from typing import Any, Awaitable, Callable, Dict

import mcp.types as types
from mcp.server import NotificationOptions, Server
from mcp.server.models import InitializationOptions
from mcp.server.stdio import stdio_server

from agi_walker.core.api.mcp_tools import MCPToolProvider

logging.basicConfig(level=logging.INFO, stream=sys.stderr)
logger = logging.getLogger("agi-walker-mcp")

SERVER_NAME = "agi-walker-control"
SERVER_VERSION = "3.0.0"

ToolHandler = Callable[[Dict[str, Any]], Any | Awaitable[Any]]


@dataclass(frozen=True)
class ToolDefinition:
    name: str
    description: str
    input_schema: Dict[str, Any]
    handler: ToolHandler


def _render_payload(payload: Any) -> str:
    return json.dumps(payload, indent=2, ensure_ascii=False, default=str)


def get_tool_definitions(
    provider: MCPToolProvider | None = None,
) -> Dict[str, ToolDefinition]:
    active_provider = provider or MCPToolProvider()
    return {
        "mission_execute": ToolDefinition(
            name="mission_execute",
            description="根据自然语言指令规划并执行机器人任务图。",
            input_schema={
                "type": "object",
                "properties": {
                    "instruction": {"type": "string", "description": "语义任务指令"}
                },
                "required": ["instruction"],
            },
            handler=lambda args: active_provider.execute_mission(
                args.get("instruction", "")
            ),
        ),
        "robot_telemetry": ToolDefinition(
            name="robot_telemetry",
            description="获取机器人实时硬件负载、温度以及 RAG 统计。",
            input_schema={"type": "object", "properties": {}},
            handler=lambda args: active_provider.get_telemetry(),
        ),
        "rag_query": ToolDefinition(
            name="rag_query",
            description="根据当前姿态检索最匹配的历史成功经验。",
            input_schema={
                "type": "object",
                "properties": {
                    "orient": {
                        "type": "array",
                        "items": {"type": "number"},
                        "description": "当前姿态 [roll, pitch, yaw]",
                    },
                    "top_k": {
                        "type": "integer",
                        "description": "返回经验条目数量",
                        "default": 1,
                    },
                },
                "required": ["orient"],
            },
            handler=lambda args: active_provider.query_rag(
                args.get("orient", [0, 0, 0]),
                top_k=int(args.get("top_k", 1)),
            ),
        ),
        "workflows_list": ToolDefinition(
            name="workflows_list",
            description="列出当前已注册的 AGI-Walker workflows。",
            input_schema={"type": "object", "properties": {}},
            handler=lambda args: active_provider.list_workflows(),
        ),
        "workflow_get": ToolDefinition(
            name="workflow_get",
            description="获取指定 workflow 的定义。",
            input_schema={
                "type": "object",
                "properties": {"name": {"type": "string"}},
                "required": ["name"],
            },
            handler=lambda args: active_provider.get_workflow(args.get("name", "")),
        ),
        "workflow_execute": ToolDefinition(
            name="workflow_execute",
            description="执行指定 workflow。",
            input_schema={
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "parameters": {"type": "object"},
                    "use_real": {"type": "boolean"},
                },
                "required": ["name"],
            },
            handler=lambda args: active_provider.execute_workflow(
                args.get("name", ""),
                parameters=args.get("parameters"),
                use_real=args.get("use_real"),
            ),
        ),
        "skills_list": ToolDefinition(
            name="skills_list",
            description="列出 AGI-Walker skills 及其元数据。",
            input_schema={"type": "object", "properties": {}},
            handler=lambda args: active_provider.list_skills(),
        ),
        "skill_get": ToolDefinition(
            name="skill_get",
            description="获取单个 skill 的详细信息，可选返回完整文档正文。",
            input_schema={
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "include_doc": {"type": "boolean", "default": False},
                },
                "required": ["name"],
            },
            handler=lambda args: active_provider.get_skill(
                args.get("name", ""),
                include_doc=bool(args.get("include_doc", False)),
            ),
        ),
        "godot_agent_status": ToolDefinition(
            name="godot_agent_status",
            description="获取当前 Godot Agent backend 的状态摘要。",
            input_schema={"type": "object", "properties": {}},
            handler=lambda args: active_provider.get_godot_agent_status(),
        ),
        "godot_agent_templates": ToolDefinition(
            name="godot_agent_templates",
            description="列出当前 Godot Agent backend 暴露的模板。",
            input_schema={"type": "object", "properties": {}},
            handler=lambda args: active_provider.list_godot_templates(),
        ),
        "godot_agent_plan": ToolDefinition(
            name="godot_agent_plan",
            description="为 Godot Agent 生成计划，不直接执行。",
            input_schema={
                "type": "object",
                "properties": {
                    "command": {"type": "string"},
                    "context": {"type": "object"},
                    "project_path": {"type": "string"},
                },
                "required": ["command"],
            },
            handler=lambda args: active_provider.plan_godot_command(
                args.get("command", ""),
                context=args.get("context"),
                project_path=args.get("project_path"),
            ),
        ),
        "godot_agent_doctor": ToolDefinition(
            name="godot_agent_doctor",
            description="运行 Godot Agent backend 环境自检。",
            input_schema={
                "type": "object",
                "properties": {"project_path": {"type": "string"}},
            },
            handler=lambda args: active_provider.doctor_godot_agent(
                project_path=args.get("project_path")
            ),
        ),
        "godot_agent_history": ToolDefinition(
            name="godot_agent_history",
            description="读取 Godot Agent 最近任务历史。",
            input_schema={
                "type": "object",
                "properties": {"limit": {"type": "integer", "default": 20}},
            },
            handler=lambda args: active_provider.get_godot_history(
                limit=int(args.get("limit", 20))
            ),
        ),
        "capability_matrix_get": ToolDefinition(
            name="capability_matrix_get",
            description="读取发布面 capability matrix 及其契约版本。",
            input_schema={"type": "object", "properties": {}},
            handler=lambda args: active_provider.get_capability_matrix(),
        ),
    }


def build_tool_list(tool_definitions: Dict[str, ToolDefinition]) -> list[types.Tool]:
    return [
        types.Tool(
            name=definition.name,
            description=definition.description,
            inputSchema=definition.input_schema,
        )
        for definition in tool_definitions.values()
    ]


async def call_tool(
    tool_definitions: Dict[str, ToolDefinition],
    name: str,
    arguments: Dict[str, Any] | None,
) -> list[types.TextContent]:
    definition = tool_definitions.get(name)
    if definition is None:
        raise ValueError(f"Unknown tool: {name}")

    payload = definition.handler(arguments or {})
    if inspect.isawaitable(payload):
        payload = await payload
    return [types.TextContent(type="text", text=_render_payload(payload))]


def create_server(provider: MCPToolProvider | None = None) -> Server:
    server = Server(SERVER_NAME)
    tool_definitions = get_tool_definitions(provider)

    @server.list_tools()
    async def handle_list_tools() -> list[types.Tool]:
        return build_tool_list(tool_definitions)

    @server.call_tool()
    async def handle_call_tool(
        name: str, arguments: dict | None
    ) -> list[types.TextContent]:
        return await call_tool(tool_definitions, name, arguments)

    return server


def build_initialization_options(server: Server) -> InitializationOptions:
    return InitializationOptions(
        server_name=SERVER_NAME,
        server_version=SERVER_VERSION,
        capabilities=server.get_capabilities(
            notification_options=NotificationOptions(),
            experimental_capabilities={},
        ),
    )


async def main() -> None:
    server = create_server()
    async with stdio_server() as (read_stream, write_stream):
        await server.run(
            read_stream,
            write_stream,
            build_initialization_options(server),
        )


def run() -> None:
    asyncio.run(main())


if __name__ == "__main__":
    run()
