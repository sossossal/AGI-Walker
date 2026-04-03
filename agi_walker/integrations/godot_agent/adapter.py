import glob
import json
import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional

from agi_walker.integrations.godot_agent.backend import GodotAgentBackend
from agi_walker.integrations.godot_agent.loader import load_router_class

logger = logging.getLogger(__name__)


class LegacyGodotAgentAdapter(GodotAgentBackend):
    """
    负责将纯净的 Backend 契约转译为本地文件系统中 `godot-studio-agent` 原有生态调用的适配器。
    采用“鸭子装载”模式，一旦 godot_studio_agent 可被路径解析，则启动内部路由。
    """

    def __init__(self, agent_dir: str):
        self.agent_dir = Path(agent_dir)
        if agent_dir not in sys.path:
            sys.path.insert(0, agent_dir)

        # 记录技能路径
        self.skills_dir = self.agent_dir / "agent_system" / "godot_skills"
        self.router = None

        try:
            router_class = load_router_class(agent_dir, "GodotStudioRouter")
            config_path = os.path.join(agent_dir, "config.yaml")
            self.router = router_class(config_path=config_path)
            logger.info("Successfully bound to legacy Godot Studio Agent.")
        except Exception as exc:
            logger.warning("Failed to loaded GodotStudioRouter! %s", exc)

    def execute_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        project_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not self.router:
            return {
                "status": "error",
                "message": "Backend engine missing / GodotStudioRouter not imported",
            }

        try:
            if project_path and hasattr(self.router, "godot_cli"):
                self.router.godot_cli.project_path = project_path

            # 新架构会返回强类型 Task, 也可以平稳降级旧架构的 Dict
            result = self.router.execute(command, context)
            if hasattr(result, "to_dict"):
                return result.to_dict()
            return result
        except Exception as exc:
            return {"status": "error", "message": str(exc), "data": {"code": ""}}

    def execute_pipeline(
        self, commands: List[str], context: Optional[Dict[str, Any]] = None
    ) -> List[Dict[str, Any]]:
        if not self.router:
            return [
                {
                    "status": "error",
                    "message": "Backend engine missing",
                    "success": False,
                }
            ]

        try:
            if hasattr(self.router, "execute_pipeline"):
                results = self.router.execute_pipeline(commands, context=context)
                if isinstance(results, list):
                    for item in results:
                        if isinstance(item, dict) and "success" not in item:
                            item["success"] = item.get("status") == "success"
                return results

            results = []
            current_context = dict(context or {})
            for cmd in commands:
                task = self.router.execute(cmd, current_context)

                if hasattr(task, "to_dict"):
                    res_dict = task.to_dict()
                else:
                    res_dict = task

                res_dict["success"] = res_dict.get("status") == "success"
                results.append(res_dict)

                if not res_dict.get("success"):
                    break
            return results
        except Exception as exc:
            return [{"success": False, "message": str(exc), "status": "error"}]

    def get_roles_info(self) -> List[Dict[str, Any]]:
        if not self.router:
            return []
        try:
            return self.router.get_roles_info()
        except Exception:
            return []

    def list_skills(self) -> Dict[str, Any]:
        try:
            skills = []
            for skill_file in glob.glob(str(self.skills_dir / "*.json")):
                with open(skill_file, "r", encoding="utf-8") as handle:
                    data = json.load(handle)
                skills.append(
                    {
                        "id": data.get("id"),
                        "name": data.get("name"),
                        "description": data.get("description"),
                        "source_kind": "legacy_skill",
                        "compatibility_mode": "native_skill",
                    }
                )
            return {
                "status": "success",
                "skills": skills,
                "compatibility_alias": False,
                "source_kind": "legacy_skill",
            }
        except Exception as exc:
            return {"status": "error", "message": str(exc)}

    def apply_skill(self, skill_id: str) -> Dict[str, Any]:
        try:
            skill_file = self.skills_dir / f"{skill_id}.json"
            if not skill_file.exists():
                return {"status": "error", "message": "Skill not found"}
            with open(skill_file, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            return {
                "status": "success",
                "data": {
                    **data,
                    "source_kind": "legacy_skill",
                    "compatibility_mode": "native_skill",
                },
                "compatibility_alias": False,
            }
        except Exception as exc:
            return {"status": "error", "message": str(exc)}

    def list_templates(self) -> Dict[str, Any]:
        skills_result = self.list_skills()
        if skills_result.get("status") != "success":
            return {
                "status": "error",
                "message": skills_result.get("message", "Failed to list legacy skills"),
            }

        templates = []
        for skill in skills_result.get("skills", []):
            templates.append(
                {
                    **skill,
                    "type": "legacy_skill_projection",
                    "source_kind": "legacy_skill",
                    "compatibility_mode": "skill_projection",
                }
            )
        return {
            "status": "success",
            "templates": templates,
            "backend_mode": "legacy",
            "compatibility_alias": True,
        }

    def get_template(self, template_id: str) -> Dict[str, Any]:
        skill_result = self.apply_skill(template_id)
        if skill_result.get("status") != "success":
            return skill_result

        data = dict(skill_result.get("data") or {})
        data["type"] = "legacy_skill_projection"
        data["source_kind"] = "legacy_skill"
        data["compatibility_mode"] = "skill_projection"
        return {
            "status": "success",
            "data": data,
            "backend_mode": "legacy",
            "compatibility_alias": True,
        }

    def plan_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        project_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not self.router:
            return {
                "status": "error",
                "message": "Backend engine missing / GodotStudioRouter not imported",
            }

        try:
            if project_path and hasattr(self.router, "godot_cli"):
                self.router.godot_cli.project_path = project_path

            if hasattr(self.router, "_analyze_prompt"):
                matches = self.router._analyze_prompt(command)
                steps = []
                for index, match in enumerate(matches, start=1):
                    role = getattr(self.router, "_roles", {}).get(match.role_name)
                    steps.append(
                        {
                            "name": f"Step {index}",
                            "description": (
                                role.get_description() if role else match.role_name
                            ),
                            "role": match.role_name,
                            "confidence": match.confidence,
                            "matched_keywords": match.matched_keywords,
                        }
                    )
                if steps:
                    return {
                        "status": "planned",
                        "success": True,
                        "prompt": command,
                        "message": "Legacy backend generated a heuristic plan",
                        "steps": steps,
                        "context": dict(context or {}),
                    }

            return {
                "status": "error",
                "success": False,
                "message": "Legacy backend does not support planning for this command",
                "prompt": command,
                "steps": [],
                "context": dict(context or {}),
            }
        except Exception as exc:
            return {"status": "error", "success": False, "message": str(exc)}

    def get_history(self, limit: int = 20) -> Dict[str, Any]:
        if not self.router:
            return {
                "status": "error",
                "items": [],
                "count": 0,
                "message": "Backend engine missing",
            }
        try:
            items = self.router.get_history(limit=limit)
            return {"status": "success", "items": items, "count": len(items)}
        except Exception as exc:
            return {"status": "error", "items": [], "count": 0, "message": str(exc)}

    def doctor(self, project_path: Optional[str] = None) -> Dict[str, Any]:
        if not self.router:
            return {
                "status": "error",
                "ok": False,
                "checks": [],
                "message": "Backend engine missing",
            }
        try:
            if project_path and hasattr(self.router, "godot_cli"):
                self.router.godot_cli.project_path = project_path

            godot_cli = getattr(self.router, "godot_cli", None)
            checks = []
            executable = getattr(godot_cli, "executable", None)
            checks.append(
                {
                    "name": "godot_executable",
                    "passed": bool(executable),
                    "message": executable or "Godot executable not found",
                }
            )
            project_value = project_path or getattr(godot_cli, "project_path", None)
            checks.append(
                {
                    "name": "project_path",
                    "passed": bool(project_value),
                    "message": project_value or "Project path not configured",
                }
            )
            checks.append(
                {
                    "name": "router",
                    "passed": True,
                    "message": type(self.router).__name__,
                }
            )
            ok = all(item["passed"] for item in checks)
            return {
                "status": "success" if ok else "error",
                "ok": ok,
                "checks": checks,
                "backend": "legacy",
            }
        except Exception as exc:
            return {"status": "error", "ok": False, "checks": [], "message": str(exc)}

    def launch_editor(
        self,
        project_path: Optional[str] = None,
        scene_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not self.router or not hasattr(self.router, "godot_cli"):
            return {"status": "error", "ok": False, "message": "Backend engine missing"}

        godot_cli = self.router.godot_cli
        if project_path:
            godot_cli.project_path = project_path

        if not getattr(godot_cli, "executable", None):
            return {
                "status": "error",
                "ok": False,
                "message": "Godot executable not configured",
            }
        if not getattr(godot_cli, "project_path", None):
            return {
                "status": "error",
                "ok": False,
                "message": "Godot project path not configured",
            }

        project_root = Path(godot_cli.project_path)
        if not project_root.exists():
            return {
                "status": "error",
                "ok": False,
                "message": f"Project path does not exist: {project_root}",
            }

        try:
            command = [godot_cli.executable, "-e", "--path", str(project_root)]
            if scene_path:
                command.append(scene_path)
            process = subprocess.Popen(
                command,
                cwd=str(project_root),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return {
                "status": "success",
                "ok": True,
                "message": "Godot editor launching",
                "data": {
                    "pid": process.pid,
                    "command": command,
                    "project_path": str(project_root),
                    "scene_path": scene_path,
                },
            }
        except Exception as exc:
            return {"status": "error", "ok": False, "message": str(exc)}
