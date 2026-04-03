import os
from pathlib import Path
from typing import Optional

from agi_walker.integrations.godot_agent.adapter import LegacyGodotAgentAdapter
from agi_walker.integrations.godot_agent.backend import GodotAgentBackend
from agi_walker.integrations.godot_agent.godot_agent_adapter import (
    ModernGodotAgentAdapter,
)


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def _default_backend_name() -> str:
    return os.getenv("AGI_WALKER_GODOT_AGENT_BACKEND", "legacy").strip().lower()


def _default_agent_dir(backend_name: str) -> Path:
    repo_root = _repo_root()
    if backend_name == "legacy":
        return repo_root / "godot_studio_agent"
    return repo_root.parent / "godot-agent"


def _default_project_path() -> Optional[str]:
    configured = os.getenv("AGI_WALKER_GODOT_PROJECT_PATH")
    if configured:
        return str(Path(configured).expanduser().resolve())

    repo_default = _repo_root() / "godot_project"
    if repo_default.exists():
        return str(repo_default.resolve())
    return None


def _default_history_file() -> Optional[str]:
    configured = os.getenv("AGI_WALKER_GODOT_AGENT_HISTORY_FILE")
    if configured:
        return str(Path(configured).expanduser().resolve())

    repo_default = (
        _repo_root() / ".output" / "godot_agent_backend" / "task_history.json"
    )
    repo_default.parent.mkdir(parents=True, exist_ok=True)
    return str(repo_default.resolve())


def create_godot_agent_backend(
    backend_name: Optional[str] = None,
    agent_dir: Optional[str] = None,
    project_path: Optional[str] = None,
    history_file: Optional[str] = None,
) -> GodotAgentBackend:
    resolved_backend = (backend_name or _default_backend_name()).strip().lower()
    resolved_agent_dir = (
        agent_dir
        or os.getenv("AGI_WALKER_GODOT_AGENT_DIR")
        or str(_default_agent_dir(resolved_backend))
    )
    resolved_project_path = project_path or _default_project_path()
    resolved_history_file = history_file or _default_history_file()

    if resolved_backend == "legacy":
        return LegacyGodotAgentAdapter(resolved_agent_dir)
    if resolved_backend in {"godot-agent", "modern"}:
        return ModernGodotAgentAdapter(
            resolved_agent_dir,
            project_path=resolved_project_path,
            history_file=resolved_history_file,
        )

    raise ValueError(
        f"Unsupported Godot Agent backend '{resolved_backend}'. "
        "Expected one of: legacy, godot-agent, modern."
    )
