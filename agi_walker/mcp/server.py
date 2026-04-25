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
        "release_control_plane_surface_get": ToolDefinition(
            name="release_control_plane_surface_get",
            description="读取 canonical release/control-plane surface，优先来自 release_manifest，缺失时回退到 canonical release_ops wrapper。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 manifest/report 路径的项目根目录。",
                    },
                    "manifest_path": {
                        "type": "string",
                        "description": "可选 manifest 路径；提供时优先从该 release_manifest 读取 control_plane_surface。",
                    },
                    "release_ops_execution_report_path": {
                        "type": "string",
                        "description": "可选 release_ops_execution_report 路径；当 manifest 缺失时作为 fallback。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_control_plane_surface(
                project_root=args.get("project_root"),
                manifest_path=args.get("manifest_path"),
                release_ops_execution_report_path=args.get(
                    "release_ops_execution_report_path"
                ),
            ),
        ),
        "release_closeout_get": ToolDefinition(
            name="release_closeout_get",
            description="读取剩余 release closeout 问题的统一只读聚合面，汇总 external-mainline、vulnerability exception review 与 worktree blocker。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 artifact 路径的项目根目录。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径；当 readiness 内没有嵌套 blocker 时作为 fallback。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_closeout(
                project_root=args.get("project_root"),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_closeout_component_get": ToolDefinition(
            name="release_closeout_component_get",
            description="读取单个 release closeout component 的只读聚合详情，返回 component payload 与对应 action item。",
            input_schema={
                "type": "object",
                "properties": {
                    "component": {
                        "type": "string",
                        "description": "closeout component 名称。",
                    },
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 artifact 路径的项目根目录。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
                "required": ["component"],
            },
            handler=lambda args: active_provider.get_release_closeout_component(
                args.get("component", ""),
                project_root=args.get("project_root"),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_closeout_next_get": ToolDefinition(
            name="release_closeout_next_get",
            description="读取推荐下一步 release closeout component 的只读聚合详情，直接返回 component payload 与建议命令。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 artifact 路径的项目根目录。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_closeout_next(
                project_root=args.get("project_root"),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_closeout_plan_get": ToolDefinition(
            name="release_closeout_plan_get",
            description="读取剩余 external closeout 输入的分阶段执行计划，返回阶段、输入文件、建议命令和完成标准。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 artifact 路径的项目根目录。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "external_mainline_inputs_path": {
                        "type": "string",
                        "description": "可选 external_mainline.inputs.json 路径。",
                    },
                    "external_mainline_input_checklist_report_path": {
                        "type": "string",
                        "description": "可选 external_mainline_input_checklist_report.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_closeout_plan(
                project_root=args.get("project_root"),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                external_mainline_inputs_path=args.get(
                    "external_mainline_inputs_path"
                ),
                external_mainline_input_checklist_report_path=args.get(
                    "external_mainline_input_checklist_report_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_closeout_plan_stage_get": ToolDefinition(
            name="release_closeout_plan_stage_get",
            description="读取单个 release closeout plan stage 的只读聚合详情，直接返回选中 stage 的 payload。",
            input_schema={
                "type": "object",
                "properties": {
                    "stage": {
                        "type": "string",
                        "description": "closeout plan stage 名称。",
                    },
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 artifact 路径的项目根目录。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "external_mainline_inputs_path": {
                        "type": "string",
                        "description": "可选 external_mainline.inputs.json 路径。",
                    },
                    "external_mainline_input_checklist_report_path": {
                        "type": "string",
                        "description": "可选 external_mainline_input_checklist_report.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
                "required": ["stage"],
            },
            handler=lambda args: active_provider.get_release_closeout_plan_stage(
                args.get("stage", ""),
                project_root=args.get("project_root"),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                external_mainline_inputs_path=args.get("external_mainline_inputs_path"),
                external_mainline_input_checklist_report_path=args.get(
                    "external_mainline_input_checklist_report_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_closeout_plan_next_get": ToolDefinition(
            name="release_closeout_plan_next_get",
            description="读取推荐下一步 release closeout plan stage 的只读聚合详情，直接返回选中 stage payload 与建议命令。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 artifact 路径的项目根目录。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "external_mainline_inputs_path": {
                        "type": "string",
                        "description": "可选 external_mainline.inputs.json 路径。",
                    },
                    "external_mainline_input_checklist_report_path": {
                        "type": "string",
                        "description": "可选 external_mainline_input_checklist_report.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_closeout_plan_next(
                project_root=args.get("project_root"),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                external_mainline_inputs_path=args.get("external_mainline_inputs_path"),
                external_mainline_input_checklist_report_path=args.get(
                    "external_mainline_input_checklist_report_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_ops_catalog_get": ToolDefinition(
            name="release_ops_catalog_get",
            description="读取 release_ops control plane 的只读 action catalog 与 policy profiles。",
            input_schema={"type": "object", "properties": {}},
            handler=lambda args: active_provider.get_release_ops_catalog(),
        ),
        "release_ops_request_templates_get": ToolDefinition(
            name="release_ops_request_templates_get",
            description="读取 release_ops action 的只读 request template defaults，可选按 action 过滤。",
            input_schema={
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "description": "可选 release_ops action 名称；提供时只返回该 action 的 request template。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_ops_request_templates(
                action=args.get("action")
            ),
        ),
        "release_control_plane_next_get": ToolDefinition(
            name="release_control_plane_next_get",
            description="读取推荐下一步 release/control-plane action 的只读聚合详情，直接返回 action schema 与 request-file scaffold。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 manifest/report 路径的项目根目录。",
                    },
                    "manifest_path": {
                        "type": "string",
                        "description": "可选 manifest 路径；提供时优先从该 release_manifest 读取 control_plane_surface。",
                    },
                    "release_ops_execution_report_path": {
                        "type": "string",
                        "description": "可选 release_ops_execution_report 路径；当 manifest 缺失时作为 fallback。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_control_plane_next(
                project_root=args.get("project_root"),
                manifest_path=args.get("manifest_path"),
                release_ops_execution_report_path=args.get(
                    "release_ops_execution_report_path"
                ),
            ),
        ),
        "release_control_plane_request_file_get": ToolDefinition(
            name="release_control_plane_request_file_get",
            description="读取单个 release/control-plane action 的只读 request-file scaffold，可直接用于生成 request-file 草稿。",
            input_schema={
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "description": "release_ops action 名称。",
                    },
                },
                "required": ["action"],
            },
            handler=lambda args: active_provider.get_release_control_plane_request_file(
                args.get("action", "")
            ),
        ),
        "release_control_plane_action_get": ToolDefinition(
            name="release_control_plane_action_get",
            description="读取单个 release/control-plane action 的只读聚合详情，包含 catalog entry 与 request template。",
            input_schema={
                "type": "object",
                "properties": {
                    "action": {
                        "type": "string",
                        "description": "release_ops action 名称。",
                    },
                },
                "required": ["action"],
            },
            handler=lambda args: active_provider.get_release_control_plane_action(
                args.get("action", "")
            ),
        ),
        "release_control_plane_index_get": ToolDefinition(
            name="release_control_plane_index_get",
            description="一次读取 canonical control-plane surface 与 release_ops catalog 的只读聚合入口。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 manifest/report 路径的项目根目录。",
                    },
                    "manifest_path": {
                        "type": "string",
                        "description": "可选 manifest 路径；提供时优先从该 release_manifest 读取 control_plane_surface。",
                    },
                    "release_ops_execution_report_path": {
                        "type": "string",
                        "description": "可选 release_ops_execution_report 路径；当 manifest 缺失时作为 fallback。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_control_plane_index(
                project_root=args.get("project_root"),
                manifest_path=args.get("manifest_path"),
                release_ops_execution_report_path=args.get(
                    "release_ops_execution_report_path"
                ),
            ),
        ),
        "release_next_get": ToolDefinition(
            name="release_next_get",
            description="一次读取统一 release 下一步入口，聚合 control-plane next 与 closeout next。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 manifest/report 路径的项目根目录。",
                    },
                    "manifest_path": {
                        "type": "string",
                        "description": "可选 manifest 路径；提供时优先从该 release_manifest 读取 control_plane_surface。",
                    },
                    "release_ops_execution_report_path": {
                        "type": "string",
                        "description": "可选 release_ops_execution_report 路径；当 manifest 缺失时作为 fallback。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_next(
                project_root=args.get("project_root"),
                manifest_path=args.get("manifest_path"),
                release_ops_execution_report_path=args.get(
                    "release_ops_execution_report_path"
                ),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_next_primary_get": ToolDefinition(
            name="release_next_primary_get",
            description="读取统一 release 下一步里的主推荐入口，直接返回 primary kind/name/status 与对应 Portal/API 深链。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 manifest/report 路径的项目根目录。",
                    },
                    "manifest_path": {
                        "type": "string",
                        "description": "可选 manifest 路径；提供时优先从该 release_manifest 读取 control_plane_surface。",
                    },
                    "release_ops_execution_report_path": {
                        "type": "string",
                        "description": "可选 release_ops_execution_report 路径；当 manifest 缺失时作为 fallback。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_next_primary(
                project_root=args.get("project_root"),
                manifest_path=args.get("manifest_path"),
                release_ops_execution_report_path=args.get(
                    "release_ops_execution_report_path"
                ),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_next_follow_up_get": ToolDefinition(
            name="release_next_follow_up_get",
            description="读取统一 release 下一步里的规范化 follow-up 入口，直接返回 request-file / 建议命令 / 下一步 JSON 的归一化结果。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 manifest/report 路径的项目根目录。",
                    },
                    "manifest_path": {
                        "type": "string",
                        "description": "可选 manifest 路径；提供时优先从该 release_manifest 读取 control_plane_surface。",
                    },
                    "release_ops_execution_report_path": {
                        "type": "string",
                        "description": "可选 release_ops_execution_report 路径；当 manifest 缺失时作为 fallback。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_next_follow_up(
                project_root=args.get("project_root"),
                manifest_path=args.get("manifest_path"),
                release_ops_execution_report_path=args.get(
                    "release_ops_execution_report_path"
                ),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
        ),
        "release_next_request_file_get": ToolDefinition(
            name="release_next_request_file_get",
            description="读取统一 release 下一步里的 request-file 导出入口；仅当当前主推荐项为 control-plane request-file 时返回草稿内容。",
            input_schema={
                "type": "object",
                "properties": {
                    "project_root": {
                        "type": "string",
                        "description": "用于解析默认 manifest/report 路径的项目根目录。",
                    },
                    "manifest_path": {
                        "type": "string",
                        "description": "可选 manifest 路径；提供时优先从该 release_manifest 读取 control_plane_surface。",
                    },
                    "release_ops_execution_report_path": {
                        "type": "string",
                        "description": "可选 release_ops_execution_report 路径；当 manifest 缺失时作为 fallback。",
                    },
                    "external_mainline_execution_plan_path": {
                        "type": "string",
                        "description": "可选 external_mainline_execution_plan.json 路径。",
                    },
                    "security_release_preflight_report_path": {
                        "type": "string",
                        "description": "可选 security_release_preflight_report.json 路径。",
                    },
                    "vulnerability_exception_review_report_path": {
                        "type": "string",
                        "description": "可选 vulnerability_exception_review_report.json 路径。",
                    },
                    "release_readiness_report_path": {
                        "type": "string",
                        "description": "可选 release_readiness_report.json 路径。",
                    },
                    "worktree_release_blocker_report_path": {
                        "type": "string",
                        "description": "可选 worktree_release_blocker_report.json 路径。",
                    },
                },
            },
            handler=lambda args: active_provider.get_release_next_request_file(
                project_root=args.get("project_root"),
                manifest_path=args.get("manifest_path"),
                release_ops_execution_report_path=args.get(
                    "release_ops_execution_report_path"
                ),
                external_mainline_execution_plan_path=args.get(
                    "external_mainline_execution_plan_path"
                ),
                security_release_preflight_report_path=args.get(
                    "security_release_preflight_report_path"
                ),
                vulnerability_exception_review_report_path=args.get(
                    "vulnerability_exception_review_report_path"
                ),
                release_readiness_report_path=args.get(
                    "release_readiness_report_path"
                ),
                worktree_release_blocker_report_path=args.get(
                    "worktree_release_blocker_report_path"
                ),
            ),
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
