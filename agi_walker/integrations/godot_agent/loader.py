import importlib
import sys
from pathlib import Path
from types import ModuleType


def _module_belongs_to_dir(module: ModuleType, agent_dir: Path) -> bool:
    module_file = getattr(module, "__file__", None)
    if not module_file:
        return True

    try:
        module_path = Path(module_file).resolve()
        resolved_dir = agent_dir.resolve()
    except OSError:
        return False

    return module_path == resolved_dir or resolved_dir in module_path.parents


def _purge_agent_system_modules() -> None:
    module_names = [
        name
        for name in sys.modules
        if name == "agent_system" or name.startswith("agent_system.")
    ]
    for name in module_names:
        sys.modules.pop(name, None)


def _prepare_agent_imports(agent_dir: str) -> None:
    resolved_dir = Path(agent_dir).resolve()
    agent_dir_str = str(resolved_dir)

    if agent_dir_str not in sys.path:
        sys.path.insert(0, agent_dir_str)

    loaded_module = sys.modules.get("agent_system")
    if loaded_module and not _module_belongs_to_dir(loaded_module, resolved_dir):
        _purge_agent_system_modules()

    loaded_router_module = sys.modules.get("agent_system.router")
    if loaded_router_module and not _module_belongs_to_dir(
        loaded_router_module, resolved_dir
    ):
        _purge_agent_system_modules()


def _module_exists_under_agent_dir(agent_dir: str, module_name: str) -> bool:
    resolved_dir = Path(agent_dir).resolve()
    relative_module_path = Path(*module_name.split("."))
    module_file = resolved_dir / relative_module_path.with_suffix(".py")
    package_init = resolved_dir / relative_module_path / "__init__.py"
    return module_file.exists() or package_init.exists()


def load_agent_attribute(agent_dir: str, module_name: str, attribute_name: str):
    _prepare_agent_imports(agent_dir)

    preloaded_module = sys.modules.get(module_name)
    if preloaded_module is not None:
        try:
            return getattr(preloaded_module, attribute_name)
        except AttributeError:
            pass

    if not _module_exists_under_agent_dir(agent_dir, module_name):
        raise ImportError(
            f"Module '{module_name}' not found under '{Path(agent_dir).resolve()}'"
        )

    module = importlib.import_module(module_name)
    try:
        return getattr(module, attribute_name)
    except AttributeError as exc:
        raise ImportError(
            f"Attribute '{attribute_name}' not found in module '{module_name}'"
        ) from exc


def load_router_class(agent_dir: str, router_class_name: str):
    resolved_dir = Path(agent_dir).resolve()
    _prepare_agent_imports(agent_dir)

    preloaded_router_module = sys.modules.get("agent_system.router")
    if preloaded_router_module is not None:
        try:
            return getattr(preloaded_router_module, router_class_name)
        except AttributeError:
            pass

    if not _module_exists_under_agent_dir(agent_dir, "agent_system.router"):
        raise ImportError(f"Router module not found in {resolved_dir}")

    router_module = importlib.import_module("agent_system.router")
    try:
        return getattr(router_module, router_class_name)
    except AttributeError as exc:
        raise ImportError(
            f"Router class '{router_class_name}' not found in {resolved_dir}"
        ) from exc
