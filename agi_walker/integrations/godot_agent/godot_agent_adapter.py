import logging
import os
import threading
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
from typing import Any, Dict, List, Optional

from agi_walker.integrations.godot_agent.backend import GodotAgentBackend
from agi_walker.integrations.godot_agent.loader import (
    load_agent_attribute,
    load_router_class,
)
from agi_walker.integrations.godot_agent.result_translation import (
    build_roles_info,
    translate_task_result,
)

logger = logging.getLogger(__name__)


class ModernGodotAgentAdapter(GodotAgentBackend):
    """
    `godot-agent` 项目的正式后端适配器。
    只暴露 AGI-Walker 已经约定的稳定接口，不把整套外部 API Server 直接并入核心。
    """

    def __init__(
        self,
        agent_dir: str,
        project_path: Optional[str] = None,
        history_file: Optional[str] = None,
    ):
        self.agent_dir = Path(agent_dir).resolve()
        self.config_path = self.agent_dir / "config.yaml"
        self.templates_dir = self.agent_dir / "agent_system" / "templates"
        self.default_project_path = self._resolve_default_project_path(project_path)
        self.history_file = self._resolve_history_file(history_file)
        self.router = None

        try:
            router_class = load_router_class(str(self.agent_dir), "GodotAgentRouter")
            try:
                self.router = router_class(
                    config_path=str(self.config_path),
                    godot_project_path=self.default_project_path,
                    history_file=str(self.history_file),
                )
            except TypeError:
                self.router = router_class(config_path=str(self.config_path))
            self._sync_router_project_path(self.default_project_path)
            logger.info("Successfully bound to modern godot-agent backend.")
        except Exception as exc:
            logger.warning(
                "Failed to load GodotAgentRouter from %s: %s", agent_dir, exc
            )

    @staticmethod
    def _repo_root() -> Path:
        return Path(__file__).resolve().parents[3]

    def _resolve_default_project_path(
        self, project_path: Optional[str]
    ) -> Optional[str]:
        candidate = project_path or os.getenv("AGI_WALKER_GODOT_PROJECT_PATH")
        if candidate:
            return str(Path(candidate).expanduser().resolve())

        repo_default = self._repo_root() / "godot_project"
        if repo_default.exists():
            return str(repo_default.resolve())
        return None

    def _resolve_history_file(self, history_file: Optional[str]) -> Path:
        candidate = history_file or os.getenv("AGI_WALKER_GODOT_AGENT_HISTORY_FILE")
        if candidate:
            resolved = Path(candidate).expanduser()
        else:
            resolved = (
                self._repo_root()
                / ".output"
                / "godot_agent_backend"
                / "task_history.json"
            )

        resolved.parent.mkdir(parents=True, exist_ok=True)
        return resolved.resolve()

    @staticmethod
    def _normalize_path(project_path: Optional[str]) -> Optional[str]:
        if not project_path:
            return None
        return str(Path(project_path).expanduser().resolve())

    def _rebuild_router_index_service(self, project_path: str) -> None:
        if not self.router:
            return

        index_service_class = load_agent_attribute(
            str(self.agent_dir),
            "agent_system.tools.index_service",
            "ProjectIndexService",
        )
        new_index_service = index_service_class(project_path)
        self.router.index_service = new_index_service

        for role in getattr(self.router, "roles", {}).values():
            if hasattr(role, "index_service"):
                role.index_service = new_index_service

        threading.Thread(target=new_index_service.rebuild, daemon=True).start()

    def _sync_router_project_path(self, project_path: Optional[str]) -> None:
        if not self.router:
            return

        normalized_path = self._normalize_path(project_path)

        if hasattr(self.router, "project_path"):
            self.router.project_path = normalized_path

        godot_cli = getattr(self.router, "godot_cli", None)
        if godot_cli is not None and hasattr(godot_cli, "project_path"):
            godot_cli.project_path = normalized_path

        if not normalized_path:
            return

        current_index_service = getattr(self.router, "index_service", None)
        current_index_path = getattr(current_index_service, "project_path", None)
        current_index_path_str = None
        if current_index_path is not None:
            current_index_path_str = str(
                Path(str(current_index_path)).expanduser().resolve()
            )

        if current_index_path_str != normalized_path:
            self._rebuild_router_index_service(normalized_path)

    def execute_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        project_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not self.router:
            return {
                "status": "error",
                "message": "Backend engine missing / GodotAgentRouter not imported",
            }

        try:
            self._sync_router_project_path(project_path or self.default_project_path)

            result = self.router.execute(command, context=context, confirm=True)
            return translate_task_result(result)
        except Exception as exc:
            return {"status": "error", "message": str(exc), "success": False}

    def execute_pipeline(
        self,
        commands: List[str],
        context: Optional[Dict[str, Any]] = None,
    ) -> List[Dict[str, Any]]:
        if not self.router:
            return [
                {
                    "status": "error",
                    "message": "Backend engine missing",
                    "success": False,
                }
            ]

        results: List[Dict[str, Any]] = []
        current_context = dict(context or {})
        try:
            self._sync_router_project_path(self.default_project_path)
            for command in commands:
                result = translate_task_result(
                    self.router.execute(command, context=current_context, confirm=True)
                )
                results.append(result)
                if not result.get("success"):
                    break
                current_context.update(result.get("context") or {})
            return results
        except Exception as exc:
            return [{"status": "error", "message": str(exc), "success": False}]

    def get_roles_info(self) -> List[Dict[str, Any]]:
        if not self.router:
            return []
        try:
            return build_roles_info(self.router)
        except Exception:
            return []

    def list_skills(self) -> Dict[str, Any]:
        templates_result = self.list_templates()
        skills = (
            templates_result.get("templates", [])
            if isinstance(templates_result, dict)
            else []
        )

        return {
            "status": "success",
            "skills": list(skills),
            "compatibility_alias": True,
            "source_kind": "template",
            "backend_mode": "godot-agent",
        }

    def apply_skill(self, skill_id: str) -> Dict[str, Any]:
        template_result = self.get_template(skill_id)
        if template_result.get("status") != "success":
            return template_result
        return {
            "status": "success",
            "data": dict(template_result.get("data") or {}),
            "compatibility_alias": True,
            "source_kind": "template",
            "backend_mode": "godot-agent",
        }

    def list_templates(self) -> Dict[str, Any]:
        try:
            templates = []
            if not self.templates_dir.exists():
                return {
                    "status": "success",
                    "templates": templates,
                    "backend_mode": "godot-agent",
                }

            for template_path in sorted(self.templates_dir.rglob("*")):
                if not template_path.is_file():
                    continue
                relative_path = template_path.relative_to(self.templates_dir).as_posix()
                templates.append(
                    {
                        "id": relative_path,
                        "name": template_path.stem,
                        "description": f"Template {relative_path}",
                        "category": template_path.parent.name,
                        "type": "template",
                        "source_kind": "template",
                        "compatibility_mode": "native_template",
                    }
                )
            return {
                "status": "success",
                "templates": templates,
                "backend_mode": "godot-agent",
            }
        except Exception as exc:
            return {"status": "error", "message": str(exc)}

    def get_template(self, template_id: str) -> Dict[str, Any]:
        try:
            candidate = (self.templates_dir / template_id).resolve()
            if (
                self.templates_dir.resolve() not in candidate.parents
                and candidate != self.templates_dir.resolve()
            ):
                return {
                    "status": "error",
                    "message": "Template path escapes template root",
                }
            if not candidate.exists() or not candidate.is_file():
                return {"status": "error", "message": "Template not found"}

            relative_path = candidate.relative_to(
                self.templates_dir.resolve()
            ).as_posix()
            return {
                "status": "success",
                "data": {
                    "id": relative_path,
                    "name": candidate.stem,
                    "category": candidate.parent.name,
                    "type": "template",
                    "source_kind": "template",
                    "compatibility_mode": "native_template",
                    "content": candidate.read_text(encoding="utf-8"),
                },
                "backend_mode": "godot-agent",
            }
        except Exception as exc:
            return {"status": "error", "message": str(exc)}

    def plan_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        project_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not self.router:
            return {
                "status": "error",
                "message": "Backend engine missing / GodotAgentRouter not imported",
            }

        try:
            self._sync_router_project_path(project_path or self.default_project_path)
            result = translate_task_result(self.router.plan(command, context=context))
            result["success"] = result.get("status") != "failed"
            return result
        except Exception as exc:
            return {"status": "error", "message": str(exc), "success": False}

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
            normalized_project_path = project_path or self.default_project_path
            self._sync_router_project_path(normalized_project_path)

            with redirect_stdout(StringIO()):
                load_router_class(str(self.agent_dir), "GodotAgentRouter")

            godot_cli = getattr(self.router, "godot_cli", None)
            executable = getattr(godot_cli, "executable", None)
            effective_project_path = normalized_project_path or getattr(
                godot_cli, "project_path", None
            )

            checks = [
                {
                    "name": "config_path",
                    "passed": self.config_path.exists(),
                    "message": str(self.config_path),
                },
                {
                    "name": "router",
                    "passed": True,
                    "message": type(self.router).__name__,
                },
                {
                    "name": "templates_dir",
                    "passed": self.templates_dir.exists(),
                    "message": str(self.templates_dir),
                },
                {
                    "name": "history_file_parent",
                    "passed": self.history_file.parent.exists(),
                    "message": str(self.history_file),
                },
                {
                    "name": "project_path",
                    "passed": bool(effective_project_path),
                    "message": effective_project_path or "Project path not configured",
                },
                {
                    "name": "godot_executable",
                    "passed": bool(executable),
                    "message": executable or "Godot executable not found",
                },
            ]

            ok = all(item["passed"] for item in checks)

            return {
                "status": "success" if ok else "error",
                "ok": ok,
                "checks": checks,
                "backend": "godot-agent",
                "config_path": str(self.config_path),
                "project_path": effective_project_path,
                "history_file": str(self.history_file),
            }
        except Exception as exc:
            return {"status": "error", "ok": False, "checks": [], "message": str(exc)}

    def launch_editor(
        self,
        project_path: Optional[str] = None,
        scene_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        if not self.router:
            return {
                "status": "error",
                "ok": False,
                "message": "Backend engine missing / GodotAgentRouter not imported",
            }

        try:
            self._sync_router_project_path(project_path or self.default_project_path)

            result = self.router.godot_cli.launch_editor(scene_path=scene_path)
            return {
                "status": "success" if result.success else "error",
                "ok": result.success,
                "message": result.message,
                "data": result.data,
                "error": result.error,
            }
        except Exception as exc:
            return {"status": "error", "ok": False, "message": str(exc)}
