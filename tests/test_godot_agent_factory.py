import sys
import types
import shutil
from pathlib import Path
from uuid import uuid4

from fastapi import FastAPI

from agi_walker.integrations.godot_agent.factory import create_godot_agent_backend
from agi_walker.integrations.godot_agent.godot_agent_adapter import (
    ModernGodotAgentAdapter,
)
from web_panel.agent_api import get_godot_agent_backend


ROUTER_STUB = """
from pathlib import Path


class FakeTask:
    def __init__(self, prompt, context=None):
        self.prompt = prompt
        self._context = dict(context or {})

    def to_dict(self):
        return {
            "status": "success",
            "message": f"ok: {self.prompt}",
            "steps": [],
            "logs": [f"ran: {self.prompt}"],
            "artifacts": [],
            "context": {**self._context, "last_prompt": self.prompt},
        }


class FakeIndexService:
    rebuild_calls = []

    def __init__(self, project_path):
        self.project_path = Path(project_path).resolve()

    def rebuild(self, force=False):
        self.__class__.rebuild_calls.append((str(self.project_path), force))


class FakeRole:
    def __init__(self, index_service):
        self.index_service = index_service


class GodotAgentRouter:
    def __init__(self, config_path=None, godot_project_path=None, history_file=None):
        self.config_path = config_path
        self.project_path = godot_project_path
        self.history_file = history_file
        self.godot_cli = type("FakeCli", (), {"project_path": godot_project_path})()
        self.index_service = FakeIndexService(godot_project_path or ".")
        self.roles = {"developer": FakeRole(self.index_service)}

    def execute(self, prompt, context=None, confirm=True):
        return FakeTask(prompt, context=context)

    def get_available_roles(self):
        return ["developer", "tester"]

    def get_role_info(self, role_name):
        return {
            "name": role_name,
            "description": f"{role_name} role",
            "capabilities": [f"{role_name}_capability"],
        }
""".strip()


class FakeTask:
    def __init__(self, prompt, context=None):
        self.prompt = prompt
        self._context = dict(context or {})

    def to_dict(self):
        return {
            "status": "success",
            "message": f"ok: {self.prompt}",
            "steps": [],
            "logs": [f"ran: {self.prompt}"],
            "artifacts": [],
            "context": {**self._context, "last_prompt": self.prompt},
        }


class FakeRouter:
    def __init__(self, config_path=None):
        self.config_path = config_path
        self.project_path = None
        self.godot_cli = types.SimpleNamespace(project_path=None)

    def execute(self, prompt, context=None, confirm=True):
        return FakeTask(prompt, context=context)

    def get_available_roles(self):
        return ["developer", "tester"]

    def get_role_info(self, role_name):
        return {
            "name": role_name,
            "description": f"{role_name} role",
            "capabilities": [f"{role_name}_capability"],
        }


def _install_fake_modern_agent(monkeypatch):
    fake_router_module = types.ModuleType("agent_system.router")
    fake_router_module.GodotAgentRouter = FakeRouter
    monkeypatch.setitem(sys.modules, "agent_system.router", fake_router_module)


def _make_agent_dir(prefix: str) -> Path:
    agent_dir = Path.cwd() / f"{prefix}_{uuid4().hex}"
    agent_dir.mkdir(parents=True, exist_ok=False)
    return agent_dir


def _write_modern_agent_stub(agent_dir: Path) -> None:
    package_dir = agent_dir / "agent_system"
    package_dir.mkdir(parents=True, exist_ok=True)
    (package_dir / "__init__.py").write_text("", encoding="utf-8")
    (package_dir / "router.py").write_text(ROUTER_STUB, encoding="utf-8")
    tools_dir = package_dir / "tools"
    tools_dir.mkdir(parents=True, exist_ok=True)
    (tools_dir / "__init__.py").write_text("", encoding="utf-8")
    (tools_dir / "index_service.py").write_text(
        "from agent_system.router import FakeIndexService as ProjectIndexService\n",
        encoding="utf-8",
    )


def test_factory_selects_modern_backend(monkeypatch):
    agent_dir = _make_agent_dir("godot_agent_factory")
    try:
        _write_modern_agent_stub(agent_dir)
        (agent_dir / "agent_system" / "templates" / "ai").mkdir(parents=True)
        (agent_dir / "config.yaml").write_text("godot: {}\n", encoding="utf-8")
        (agent_dir / "agent_system" / "templates" / "ai" / "patrol.gd").write_text(
            "extends Node\n", encoding="utf-8"
        )
        project_dir = agent_dir / "sample_project"
        project_dir.mkdir()
        history_file = agent_dir / "runtime" / "task_history.json"

        backend = create_godot_agent_backend(
            "godot-agent",
            str(agent_dir),
            project_path=str(project_dir),
            history_file=str(history_file),
        )

        assert isinstance(backend, ModernGodotAgentAdapter)
        assert backend.router is not None
        assert backend.default_project_path == str(project_dir.resolve())
        assert backend.router.project_path == str(project_dir.resolve())
        assert backend.router.godot_cli.project_path == str(project_dir.resolve())
        assert backend.router.history_file == str(history_file.resolve())
        assert backend.history_file == history_file.resolve()
        assert backend.list_templates()["templates"][0]["id"] == "ai/patrol.gd"
        assert backend.list_skills()["skills"][0]["id"] == "ai/patrol.gd"
        assert backend.list_skills()["compatibility_alias"] is True
        assert backend.list_templates()["templates"][0]["id"] == "ai/patrol.gd"
    finally:
        shutil.rmtree(agent_dir, ignore_errors=True)


def test_modern_adapter_pipeline_and_roles(monkeypatch):
    agent_dir = _make_agent_dir("godot_agent_adapter")
    try:
        _write_modern_agent_stub(agent_dir)
        (agent_dir / "agent_system" / "templates" / "ai").mkdir(parents=True)
        (agent_dir / "config.yaml").write_text("godot: {}\n", encoding="utf-8")
        (agent_dir / "agent_system" / "templates" / "ai" / "chase.gd").write_text(
            "extends Node\n", encoding="utf-8"
        )
        project_dir = agent_dir / "project_a"
        project_dir.mkdir()
        switched_project_dir = agent_dir / "project_b"
        switched_project_dir.mkdir()

        adapter = ModernGodotAgentAdapter(str(agent_dir), project_path=str(project_dir))
        results = adapter.execute_pipeline(
            ["step one", "step two"], context={"foo": "bar"}
        )

        assert len(results) == 2
        assert all(item["success"] is True for item in results)
        assert results[-1]["context"]["last_prompt"] == "step two"
        assert adapter.router.project_path == str(project_dir.resolve())
        assert str(adapter.router.index_service.project_path) == str(
            project_dir.resolve()
        )

        adapter.execute_command(
            "switch project", project_path=str(switched_project_dir)
        )
        assert adapter.router.project_path == str(switched_project_dir.resolve())
        assert adapter.router.godot_cli.project_path == str(
            switched_project_dir.resolve()
        )
        assert str(adapter.router.index_service.project_path) == str(
            switched_project_dir.resolve()
        )
        assert str(adapter.router.roles["developer"].index_service.project_path) == str(
            switched_project_dir.resolve()
        )

        roles = adapter.get_roles_info()
        assert [role["name"] for role in roles] == ["developer", "tester"]

        templates = adapter.list_templates()
        assert templates["templates"][0]["type"] == "template"

        template = adapter.get_template("ai/chase.gd")
        assert template["status"] == "success"
        assert template["data"]["id"] == "ai/chase.gd"
        assert template["data"]["source_kind"] == "template"

        applied = adapter.apply_skill("ai/chase.gd")
        assert applied["status"] == "success"
        assert applied["data"]["id"] == "ai/chase.gd"
        assert applied["compatibility_alias"] is True
        assert applied["compatibility_alias"] is True

        template = adapter.get_template("ai/chase.gd")
        assert template["status"] == "success"
        assert template["data"]["source_kind"] == "template"
    finally:
        shutil.rmtree(agent_dir, ignore_errors=True)


def test_modern_adapter_doctor_degrades_to_builtin_checks():
    agent_dir = _make_agent_dir("godot_agent_doctor")
    try:
        _write_modern_agent_stub(agent_dir)
        (agent_dir / "agent_system" / "templates" / "ai").mkdir(parents=True)
        (agent_dir / "config.yaml").write_text("godot: {}\n", encoding="utf-8")
        project_dir = agent_dir / "project_a"
        project_dir.mkdir()

        adapter = ModernGodotAgentAdapter(str(agent_dir), project_path=str(project_dir))
        doctor = adapter.doctor()

        assert doctor["status"] == "error"
        assert doctor["ok"] is False
        assert doctor["backend"] == "godot-agent"
        assert doctor["project_path"] == str(project_dir.resolve())
        assert doctor["config_path"] == str((agent_dir / "config.yaml").resolve())
        assert doctor["history_file"]
        assert any(check["name"] == "router" and check["passed"] for check in doctor["checks"])
        assert any(
            check["name"] == "godot_executable" and check["passed"] is False
            for check in doctor["checks"]
        )
    finally:
        shutil.rmtree(agent_dir, ignore_errors=True)


def test_agent_api_uses_factory(monkeypatch):
    fake_backend = object()
    monkeypatch.setattr(
        "web_panel.agent_api.create_godot_agent_backend",
        lambda: fake_backend,
    )

    app = FastAPI()
    resolved = get_godot_agent_backend(app)

    assert resolved is fake_backend
    assert get_godot_agent_backend(app) is fake_backend
