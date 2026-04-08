import logging
import os
from typing import Any, Callable, Dict, List, Optional

from agi_walker.core.api.simple_planner import SimplePlanner
from agi_walker.core.controllers.load_monitor import SystemMonitor
from agi_walker.core.controllers.rag_knowledge_base import PhysicsKnowledgeBase
from agi_walker.integrations.godot_agent.factory import create_godot_agent_backend
from agi_walker.integrations.godot_agent.godot_agent_adapter import (
    ModernGodotAgentAdapter,
)
from agi_walker.skills_loader import SkillMetadata, get_skills_loader
from agi_walker.workflow_orchestrator import get_workflow_orchestrator

logger = logging.getLogger("AGI-Walker-MCP")


class MCPToolProvider:
    """
    AGI-Walker V3.0 MCP Tool Bridge.
    Exposes project capabilities as structured tools for MCP clients.
    """

    def __init__(
        self,
        orchestrator: Any = None,
        planner: Optional[SimplePlanner] = None,
        monitor: Optional[SystemMonitor] = None,
        knowledge_base: Optional[PhysicsKnowledgeBase] = None,
        skills_loader: Any = None,
        godot_backend_factory: Optional[Callable[[], Any]] = None,
    ):
        self.orchestrator = orchestrator or get_workflow_orchestrator()
        self.planner = planner or SimplePlanner()
        self.monitor = monitor or SystemMonitor()
        self.kb = knowledge_base or PhysicsKnowledgeBase()
        self.skills_loader = skills_loader
        self.godot_backend_factory = godot_backend_factory or create_godot_agent_backend
        self._godot_backend: Any = None

    def _error(self, message: str, **extra: Any) -> Dict[str, Any]:
        return {"status": "error", "message": message, **extra}

    def _get_skills_loader(self):
        if self.skills_loader is None:
            self.skills_loader = get_skills_loader()
        return self.skills_loader

    def _get_godot_backend(self):
        if self._godot_backend is None:
            self._godot_backend = self.godot_backend_factory()
        return self._godot_backend

    @staticmethod
    def _model_dump(model: Any) -> Dict[str, Any]:
        if hasattr(model, "model_dump"):
            return model.model_dump()
        return dict(model)

    def _serialize_skill(
        self, skill: SkillMetadata, include_doc: bool = False
    ) -> Dict[str, Any]:
        payload = {
            "name": skill.name,
            "description": skill.description,
            "category": skill.category,
            "emoji": skill.emoji,
            "requires": skill.requires,
            "inputs": [self._model_dump(item) for item in skill.inputs],
            "outputs": [self._model_dump(item) for item in skill.outputs],
            "skill_dir": str(skill.skill_dir) if skill.skill_dir else None,
        }
        if include_doc:
            payload["doc"] = self._get_skills_loader().get_skill_doc(skill.name)
        return payload

    @staticmethod
    def _configured_godot_backend_name() -> str:
        return os.getenv("AGI_WALKER_GODOT_AGENT_BACKEND", "legacy").strip().lower()

    async def execute_mission(self, instruction: str) -> Dict[str, Any]:
        normalized_instruction = instruction.strip()
        if not normalized_instruction:
            return self._error("instruction is required")

        logger.info("MCP Request: Execute mission '%s'", normalized_instruction)
        try:
            graph = self.planner.plan(normalized_instruction)
            result = self.orchestrator.execute_task_graph(graph)
            return {
                "status": "success",
                "instruction": normalized_instruction,
                "result": result.to_dict(),
            }
        except Exception as exc:
            logger.exception("Mission execution failed")
            return self._error(str(exc), instruction=normalized_instruction)

    def get_telemetry(self) -> Dict[str, Any]:
        stats = self.monitor.get_hw_stats()
        stats["rag"] = self.kb.get_stats()
        return {"status": "success", "telemetry": stats}

    def query_rag(self, orient: List[float], top_k: int = 1) -> Dict[str, Any]:
        sensor_data = {"sensors": {"imu": {"orient": orient}}}
        experiences = self.kb.retrieve_experience(sensor_data, top_k=top_k)
        return {
            "status": "success",
            "query": {"orient": orient, "top_k": top_k},
            "matches": [
                {
                    "id": item.id,
                    "scenario": item.scenario,
                    "outcome": item.outcome,
                    "state_pattern": item.state_pattern,
                    "action_ref": item.action_ref,
                    "source_file": item.source_file,
                }
                for item in experiences
            ],
        }

    def list_available_workflows(self) -> List[str]:
        return self.orchestrator.list_workflows()

    def list_workflows(self) -> Dict[str, Any]:
        workflows = []
        for name in self.orchestrator.list_workflows():
            workflow = self.orchestrator.get_workflow(name) or {}
            workflows.append(
                {
                    "name": name,
                    "description": workflow.get("description", ""),
                    "steps_count": len(workflow.get("steps", [])),
                }
            )
        return {"status": "success", "count": len(workflows), "workflows": workflows}

    def get_workflow(self, name: str) -> Dict[str, Any]:
        workflow = self.orchestrator.get_workflow(name)
        if workflow is None:
            return self._error(f"workflow '{name}' not found", workflow_name=name)
        return {"status": "success", "workflow": workflow}

    def execute_workflow(
        self,
        name: str,
        parameters: Optional[Dict[str, Any]] = None,
        use_real: Optional[bool] = None,
    ) -> Dict[str, Any]:
        result = self.orchestrator.execute_workflow(
            name,
            parameters=parameters,
            use_real=use_real,
        )
        return {
            "status": "success" if result.status.value == "completed" else "error",
            "workflow_name": name,
            "result": result.to_dict(),
        }

    def list_skills(self) -> Dict[str, Any]:
        skills = [
            self._serialize_skill(skill)
            for skill in self._get_skills_loader().get_skills_list()
        ]
        return {"status": "success", "count": len(skills), "skills": skills}

    def get_skill(self, name: str, include_doc: bool = False) -> Dict[str, Any]:
        skill = self._get_skills_loader().get_skill(name)
        if skill is None:
            return self._error(f"skill '{name}' not found", skill_name=name)
        return {
            "status": "success",
            "skill": self._serialize_skill(skill, include_doc=include_doc),
        }

    def get_godot_agent_status(self) -> Dict[str, Any]:
        try:
            backend = self._get_godot_backend()
        except Exception as exc:
            return self._error(
                str(exc),
                backend_mode=self._configured_godot_backend_name(),
            )

        configured_backend = self._configured_godot_backend_name()
        is_modern = isinstance(
            backend, ModernGodotAgentAdapter
        ) or configured_backend in {
            "godot-agent",
            "modern",
        }
        roles = backend.get_roles_info()
        templates_result = backend.list_templates()
        templates = (
            templates_result.get("templates", [])
            if isinstance(templates_result, dict)
            else []
        )
        templates_status = (
            templates_result.get("status", "error")
            if isinstance(templates_result, dict)
            else "error"
        )
        return {
            "status": "ready" if templates_status == "success" else "degraded",
            "backend_mode": (
                templates_result.get("backend_mode")
                if isinstance(templates_result, dict)
                and templates_result.get("backend_mode")
                else ("godot-agent" if is_modern else "legacy")
            ),
            "backend_class": backend.__class__.__name__,
            "resource_mode": "templates" if is_modern else "skills",
            "router_ready": getattr(backend, "router", None) is not None,
            "roles_count": len(roles),
            "templates_count": len(templates),
            "agent_dir": (
                str(getattr(backend, "agent_dir"))
                if getattr(backend, "agent_dir", None)
                else os.getenv("AGI_WALKER_GODOT_AGENT_DIR")
            ),
            "project_path": getattr(backend, "default_project_path", None)
            or os.getenv("AGI_WALKER_GODOT_PROJECT_PATH"),
            "history_file": (
                str(getattr(backend, "history_file"))
                if getattr(backend, "history_file", None)
                else None
            ),
        }

    def list_godot_templates(self) -> Dict[str, Any]:
        try:
            return self._get_godot_backend().list_templates()
        except Exception as exc:
            return self._error(str(exc))

    def plan_godot_command(
        self,
        command: str,
        context: Optional[Dict[str, Any]] = None,
        project_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        try:
            return self._get_godot_backend().plan_command(
                command,
                context=context,
                project_path=project_path,
            )
        except Exception as exc:
            return self._error(str(exc))

    def doctor_godot_agent(self, project_path: Optional[str] = None) -> Dict[str, Any]:
        try:
            return self._get_godot_backend().doctor(project_path=project_path)
        except Exception as exc:
            return self._error(str(exc))

    def get_godot_history(self, limit: int = 20) -> Dict[str, Any]:
        try:
            return self._get_godot_backend().get_history(limit=limit)
        except Exception as exc:
            return self._error(str(exc), count=0, items=[])
