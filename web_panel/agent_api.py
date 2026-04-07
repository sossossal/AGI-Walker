import json
import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import pydantic
from fastapi import APIRouter
from fastapi import FastAPI

from agi_walker.integrations.godot_agent.adapter import LegacyGodotAgentAdapter
from agi_walker.integrations.godot_agent.godot_agent_adapter import ModernGodotAgentAdapter
from agi_walker.integrations.godot_agent.factory import get_godot_agent_backend
from agi_walker.integrations.godot_agent.result_translation import translate_task_result

logger = logging.getLogger(__name__)

class GodotAgentLaunchRequest(pydantic.BaseModel):
    project_path: Optional[str] = None
    scene_path: Optional[str] = None

class GodotSkillApplyRequest(pydantic.BaseModel):
    skill_id: str

def get_godot_agent_status(app: FastAPI) -> Dict[str, Any]:
    """获取 Godot Agent 后端状态的核心实现 (V3.0 隔离识别版)"""
    configured_backend = os.getenv("AGI_WALKER_GODOT_AGENT_BACKEND", "legacy").strip().lower()
    backend = get_godot_agent_backend(backend_type=configured_backend)

    # 统一识别逻辑
    is_modern = isinstance(backend, ModernGodotAgentAdapter) or configured_backend in {"godot-agent", "modern"}
    
    backend_mode = "godot-agent" if is_modern else "legacy"
    resource_mode = "templates" if is_modern else "skills"
    
    return {
        "backend_mode": backend_mode,
        "resource_mode": resource_mode,
        "agent_dir": os.getenv("AGI_WALKER_GODOT_AGENT_DIR"),
        "project_path": os.getenv("AGI_WALKER_GODOT_PROJECT_PATH")
    }

def build_router(app: FastAPI) -> APIRouter:
    router = APIRouter()

    @router.get("/api/godot-agent/status")
    async def get_godot_agent_status_route():
        return get_godot_agent_status(app)

    @router.get("/api/godot-agent/templates")
    async def list_godot_agent_templates_route():
        # V3.0 FIX: Do NOT pass FastAPI instance 'app' as the first positional argument
        backend = get_godot_agent_backend()
        return backend.list_templates()

    @router.get("/api/godot-agent/templates/{template_id:path}")
    async def get_godot_agent_template_route(template_id: str):
        backend = get_godot_agent_backend()
        return backend.get_template(template_id)

    @router.post("/api/godot-agent/plan")
    async def plan_godot_agent_command_route(req: Dict[str, Any]):
        backend = get_godot_agent_backend()
        return backend.plan_command(req.get("command", ""), context=req.get("context"))

    @router.get("/api/godot-agent/doctor")
    async def doctor_godot_agent_route():
        backend = get_godot_agent_backend()
        return backend.doctor()

    @router.get("/api/godot-agent/history")
    async def get_godot_agent_history_route(limit: int = 20):
        backend = get_godot_agent_backend()
        return backend.get_history(limit=limit)

    @router.get("/api/godot_skills/list")
    async def list_godot_skills_route():
        backend = get_godot_agent_backend()
        result = backend.list_skills()
        if isinstance(backend, ModernGodotAgentAdapter) and isinstance(result, dict):
            result["compatibility_alias"] = True
        return result

    @router.post("/api/godot_skills/apply")
    async def apply_godot_skill_route(req: GodotSkillApplyRequest):
        backend = get_godot_agent_backend()
        return backend.apply_skill(req.skill_id)

    @router.post("/api/godot-agent/launch")
    async def launch_godot_agent_route(req: GodotAgentLaunchRequest):
        backend = get_godot_agent_backend()
        return backend.launch_editor(req.project_path, req.scene_path)

    return router
