import glob
import json
import logging
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

import pydantic
from fastapi import APIRouter
from fastapi import FastAPI

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


def _skills_dir() -> Path:
    return _repo_root() / "godot_studio_agent" / "agent_system" / "godot_skills"


def get_godot_agent_router(app: FastAPI):
    agent_dir = str(_repo_root() / "godot_studio_agent")
    if agent_dir not in sys.path:
        sys.path.insert(0, agent_dir)

    if not hasattr(app.state, "godot_agent_router"):
        from agent_system.router import GodotStudioRouter

        app.state.godot_agent_router = GodotStudioRouter()
    return app.state.godot_agent_router


def execute_godot_agent_command(
    app: FastAPI, req: GodotAgentCommandRequest, timestamp: str
) -> Dict[str, Any]:
    try:
        router = get_godot_agent_router(app)
        if req.godot_project_path:
            router.godot_cli.project_path = req.godot_project_path
        result = router.execute(req.command, req.context)
        result["timestamp"] = timestamp
        return result
    except Exception as exc:
        logger.info("Agent Execute Error: %s", exc)
        return {"status": "error", "message": str(exc), "data": {"code": ""}}


def execute_godot_agent_pipeline(
    app: FastAPI, req: GodotAgentPipelineRequest
) -> Dict[str, Any]:
    try:
        router = get_godot_agent_router(app)
        results = router.execute_pipeline(req.commands)
        return {
            "success": all(result.get("success") for result in results),
            "steps": len(results),
            "results": results,
        }
    except Exception as exc:
        return {"success": False, "message": str(exc)}


def get_godot_agent_roles(app: FastAPI) -> Dict[str, Any]:
    try:
        return {"roles": get_godot_agent_router(app).get_roles_info()}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}


def list_godot_skills() -> Dict[str, Any]:
    try:
        skills = []
        for skill_file in glob.glob(str(_skills_dir() / "*.json")):
            with open(skill_file, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            skills.append(
                {
                    "id": data.get("id"),
                    "name": data.get("name"),
                    "description": data.get("description"),
                }
            )
        return {"status": "success", "skills": skills}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}


def apply_godot_skill(req: GodotSkillApplyRequest) -> Dict[str, Any]:
    try:
        skill_file = _skills_dir() / f"{req.skill_id}.json"
        if not skill_file.exists():
            return {"status": "error", "message": "Skill not found"}
        with open(skill_file, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        return {"status": "success", "data": data}
    except Exception as exc:
        return {"status": "error", "message": str(exc)}


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
        return list_godot_skills()

    @router.post("/api/godot_skills/apply")
    async def apply_godot_skill_route(req: GodotSkillApplyRequest):
        """获取完整单个神盾局技能配置"""
        return apply_godot_skill(req)

    return router
