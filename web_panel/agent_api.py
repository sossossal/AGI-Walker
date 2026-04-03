import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import pydantic
from fastapi import APIRouter
from fastapi import FastAPI

from agi_walker.integrations.godot_agent import (
    GodotAgentBackend,
    LegacyGodotAgentAdapter,
    ModernGodotAgentAdapter,
    create_godot_agent_backend,
)

logger = logging.getLogger(__name__)


class CommandRequest(pydantic.BaseModel):
    command: str


class GodotAgentCommandRequest(pydantic.BaseModel):
    command: str
    context: Optional[Dict[str, Any]] = None
    godot_project_path: Optional[str] = None


class GodotAgentPipelineRequest(pydantic.BaseModel):
    commands: List[str]
    context: Optional[Dict[str, Any]] = None


class GodotSkillApplyRequest(pydantic.BaseModel):
    skill_id: str


class GodotTemplateFetchRequest(pydantic.BaseModel):
    template_id: str


class GodotAgentPlanRequest(pydantic.BaseModel):
    command: str
    context: Optional[Dict[str, Any]] = None
    godot_project_path: Optional[str] = None


class GodotAgentLaunchRequest(pydantic.BaseModel):
    godot_project_path: Optional[str] = None
    scene_path: Optional[str] = None


def parse_command(req: CommandRequest) -> Dict[str, Any]:
    from web_panel.command_parser import CommandParser

    try:
        parser = CommandParser()
        config = parser.parse(req.command)
        return {"status": "success", "config": config}
    except Exception as exc:
        logger.info("Command parse error: %s", exc)
        return {"status": "error", "message": str(exc)}


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent

def get_godot_agent_backend(app: FastAPI) -> GodotAgentBackend:
    if not hasattr(app.state, "godot_agent_backend"):
        app.state.godot_agent_backend = create_godot_agent_backend()
    return app.state.godot_agent_backend


def get_godot_agent_status(app: FastAPI) -> Dict[str, Any]:
    configured_backend = os.getenv("AGI_WALKER_GODOT_AGENT_BACKEND", "legacy").strip().lower()
    configured_agent_dir = os.getenv("AGI_WALKER_GODOT_AGENT_DIR")
    backend = get_godot_agent_backend(app)

    backend_mode = configured_backend
    resource_mode = "skills"
    if isinstance(backend, ModernGodotAgentAdapter):
        backend_mode = "godot-agent"
        resource_mode = "templates"
    elif isinstance(backend, LegacyGodotAgentAdapter):
        backend_mode = "legacy"

    router_attr_exists = hasattr(backend, "router")
    router = getattr(backend, "router", None)
    backend_ready = (not router_attr_exists) or (router is not None)
    roles = backend.get_roles_info() if backend_ready else []
    templates_result = backend.list_templates() if backend_ready else {"templates": []}
    templates = templates_result.get("templates", []) if isinstance(templates_result, dict) else []

    resolved_agent_dir = str(getattr(backend, "agent_dir", configured_agent_dir or ""))
    resolved_project_path = getattr(backend, "default_project_path", None)
    resolved_history_file = getattr(backend, "history_file", None)
    return {
        "backend_mode": backend_mode,
        "configured_backend": configured_backend,
        "backend_class": type(backend).__name__,
        "agent_dir": resolved_agent_dir,
        "project_path": str(resolved_project_path) if resolved_project_path else None,
        "history_file": str(resolved_history_file) if resolved_history_file else None,
        "router_ready": backend_ready,
        "status": "ready" if backend_ready else "degraded",
        "resource_mode": resource_mode,
        "roles_count": len(roles),
        "templates_count": len(templates),
    }

def execute_godot_agent_command(
    app: FastAPI, req: GodotAgentCommandRequest, timestamp: str
) -> Dict[str, Any]:
    backend = get_godot_agent_backend(app)
    result = backend.execute_command(req.command, req.context, project_path=req.godot_project_path)
    if isinstance(result, dict):
        result["timestamp"] = timestamp
    return result

def execute_godot_agent_pipeline(
    app: FastAPI, req: GodotAgentPipelineRequest
) -> Dict[str, Any]:
    backend = get_godot_agent_backend(app)
    results = backend.execute_pipeline(req.commands, req.context)
    if isinstance(results, dict) and results.get("status") == "error":
        return {"success": False, "message": results.get("message")}
    return {
        "success": all(res.get("success") for res in results),
        "steps": len(results),
        "results": results,
    }

def get_godot_agent_roles(app: FastAPI) -> Dict[str, Any]:
    return {"roles": get_godot_agent_backend(app).get_roles_info()}

def list_godot_skills(app: FastAPI) -> Dict[str, Any]:
    return get_godot_agent_backend(app).list_skills()

def apply_godot_skill(app: FastAPI, req: GodotSkillApplyRequest) -> Dict[str, Any]:
    return get_godot_agent_backend(app).apply_skill(req.skill_id)

def list_godot_templates(app: FastAPI) -> Dict[str, Any]:
    return get_godot_agent_backend(app).list_templates()

def get_godot_template(app: FastAPI, template_id: str) -> Dict[str, Any]:
    return get_godot_agent_backend(app).get_template(template_id)

def plan_godot_agent_command(
    app: FastAPI, req: GodotAgentPlanRequest
) -> Dict[str, Any]:
    return get_godot_agent_backend(app).plan_command(
        req.command,
        req.context,
        project_path=req.godot_project_path,
    )

def get_godot_agent_history(app: FastAPI, limit: int = 20) -> Dict[str, Any]:
    return get_godot_agent_backend(app).get_history(limit=limit)

def doctor_godot_agent(app: FastAPI, godot_project_path: Optional[str] = None) -> Dict[str, Any]:
    return get_godot_agent_backend(app).doctor(project_path=godot_project_path)

def launch_godot_agent_editor(
    app: FastAPI, req: GodotAgentLaunchRequest
) -> Dict[str, Any]:
    return get_godot_agent_backend(app).launch_editor(
        project_path=req.godot_project_path,
        scene_path=req.scene_path,
    )


def build_router(app: FastAPI) -> APIRouter:
    router = APIRouter()

    @router.post("/api/agent/parse-command")
    async def parse_command_route(req: CommandRequest):
        """解析自然语言指令并返回 Robot Config"""
        return parse_command(req)

    @router.post("/execute")
    async def execute_godot_agent_command_route(req: GodotAgentCommandRequest):
        """执行 Godot Studio Agent 命令 (供 Godot 插件调用)"""
        from datetime import datetime

        return execute_godot_agent_command(app, req, datetime.now().isoformat())

    @router.post("/pipeline")
    async def execute_godot_agent_pipeline_route(req: GodotAgentPipelineRequest):
        """执行多步骤命令流水线"""
        return execute_godot_agent_pipeline(app, req)

    @router.get("/roles")
    async def get_godot_agent_roles_route():
        """获取所有可用角色信息"""
        return get_godot_agent_roles(app)

    @router.get("/api/godot_skills/list")
    async def list_godot_skills_route():
        """获取所有可用的 Godot 技能"""
        return list_godot_skills(app)

    @router.post("/api/godot_skills/apply")
    async def apply_godot_skill_route(req: GodotSkillApplyRequest):
        """获取完整单个神盾局技能配置"""
        return apply_godot_skill(app, req)

    @router.get("/api/godot-agent/templates")
    async def list_godot_templates_route():
        """列出当前 backend 暴露的模板资源"""
        return list_godot_templates(app)

    @router.get("/api/godot-agent/templates/{template_id:path}")
    async def get_godot_template_route(template_id: str):
        """获取单个模板详情"""
        return get_godot_template(app, template_id)

    @router.post("/api/godot-agent/plan")
    async def plan_godot_agent_route(req: GodotAgentPlanRequest):
        """生成 Godot Agent 任务计划"""
        return plan_godot_agent_command(app, req)

    @router.get("/api/godot-agent/history")
    async def get_godot_agent_history_route(limit: int = 20):
        """获取 Godot Agent 最近任务历史"""
        return get_godot_agent_history(app, limit=limit)

    @router.get("/api/godot-agent/doctor")
    async def doctor_godot_agent_route(godot_project_path: Optional[str] = None):
        """运行 Godot Agent 环境自检"""
        return doctor_godot_agent(app, godot_project_path=godot_project_path)

    @router.post("/api/godot-agent/launch")
    async def launch_godot_agent_route(req: GodotAgentLaunchRequest):
        """请求 Godot Agent 启动编辑器"""
        return launch_godot_agent_editor(app, req)

    return router
