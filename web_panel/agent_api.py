import logging
import os
from typing import Any, Dict, Optional

import pydantic
from pydantic import Field
from fastapi import APIRouter
from fastapi import FastAPI

from agi_walker.integrations.godot_agent.godot_agent_adapter import (
    ModernGodotAgentAdapter,
)
from agi_walker.integrations.godot_agent.factory import create_godot_agent_backend
from web_panel.command_parser import CommandParser

logger = logging.getLogger(__name__)


class GodotAgentLaunchRequest(pydantic.BaseModel):
    project_path: Optional[str] = Field(default=None, alias="godot_project_path")
    scene_path: Optional[str] = None


class GodotSkillApplyRequest(pydantic.BaseModel):
    skill_id: str


class AgentCommandParseRequest(pydantic.BaseModel):
    command: str


def get_godot_agent_backend(
    app: FastAPI, backend_type: Optional[str] = None
) -> Any:
    cached = getattr(app.state, "godot_agent_backend", None)
    if cached is not None:
        return cached

    if backend_type is None:
        backend = create_godot_agent_backend()
    else:
        backend = create_godot_agent_backend(backend_name=backend_type)
    app.state.godot_agent_backend = backend
    return backend


def get_godot_agent_status(app: FastAPI) -> Dict[str, Any]:
    """获取 Godot Agent 后端状态的核心实现 (V3.0 隔离识别版)"""
    configured_backend = (
        os.getenv("AGI_WALKER_GODOT_AGENT_BACKEND", "legacy").strip().lower()
    )
    backend = get_godot_agent_backend(app)

    # 统一识别逻辑
    backend_class = type(backend).__name__
    is_fake = backend_class.lower().startswith("fake")
    is_modern = isinstance(backend, ModernGodotAgentAdapter) or configured_backend in {
        "godot-agent",
        "modern",
    }

    if is_fake:
        backend_mode = "fake"
    elif is_modern:
        backend_mode = "godot-agent"
    else:
        backend_mode = "legacy"
    resource_mode = "templates" if is_modern else "skills"

    router = getattr(backend, "router", None)
    roles_count = 0
    templates_count = 0
    try:
        roles_count = len(backend.get_roles_info())
    except Exception:
        roles_count = 0
    try:
        templates_count = len((backend.list_templates() or {}).get("templates", []))
    except Exception:
        templates_count = 0

    default_project_path = getattr(backend, "default_project_path", None)
    history_file = getattr(backend, "history_file", None)

    return {
        "status": "ready",
        "backend_mode": backend_mode,
        "backend_class": backend_class,
        "router_ready": router is not None or is_fake,
        "resource_mode": resource_mode,
        "roles_count": roles_count,
        "templates_count": templates_count,
        "agent_dir": os.getenv("AGI_WALKER_GODOT_AGENT_DIR"),
        "project_path": os.getenv("AGI_WALKER_GODOT_PROJECT_PATH")
        or default_project_path,
        "history_file": str(history_file) if history_file else None,
    }


def build_router(app: FastAPI) -> APIRouter:
    router = APIRouter()

    @router.get("/api/godot-agent/status")
    async def get_godot_agent_status_route():
        return get_godot_agent_status(app)

    @router.get("/api/godot-agent/templates")
    async def list_godot_agent_templates_route():
        # V3.0 FIX: Do NOT pass FastAPI instance 'app' as the first positional argument
        backend = create_godot_agent_backend()
        return backend.list_templates()

    @router.get("/api/godot-agent/templates/{template_id:path}")
    async def get_godot_agent_template_route(template_id: str):
        backend = create_godot_agent_backend()
        return backend.get_template(template_id)

    @router.post("/api/godot-agent/plan")
    async def plan_godot_agent_command_route(req: Dict[str, Any]):
        backend = create_godot_agent_backend()
        return backend.plan_command(
            req.get("command", ""),
            context=req.get("context"),
            project_path=req.get("godot_project_path"),
        )

    @router.get("/api/godot-agent/doctor")
    async def doctor_godot_agent_route(godot_project_path: Optional[str] = None):
        backend = create_godot_agent_backend()
        return backend.doctor(project_path=godot_project_path)

    @router.get("/api/godot-agent/history")
    async def get_godot_agent_history_route(limit: int = 20):
        backend = create_godot_agent_backend()
        return backend.get_history(limit=limit)

    @router.post("/api/agent/parse-command")
    async def parse_agent_command_route(req: AgentCommandParseRequest):
        parser = CommandParser()
        config = parser.parse(req.command)
        return {"status": "success", "config": config}

    @router.get("/api/godot_skills/list")
    async def list_godot_skills_route():
        backend = create_godot_agent_backend()
        result = backend.list_skills()
        if isinstance(backend, ModernGodotAgentAdapter) and isinstance(result, dict):
            result["compatibility_alias"] = True
        return result

    @router.post("/api/godot_skills/apply")
    async def apply_godot_skill_route(req: GodotSkillApplyRequest):
        backend = create_godot_agent_backend()
        return backend.apply_skill(req.skill_id)

    @router.post("/api/godot-agent/launch")
    async def launch_godot_agent_route(req: GodotAgentLaunchRequest):
        backend = create_godot_agent_backend()
        return backend.launch_editor(req.project_path, req.scene_path)

    return router
