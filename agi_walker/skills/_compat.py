"""
Compatibility helpers for importing skill implementations from kebab-case paths.
"""

import logging
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys
from types import ModuleType
from typing import Dict

logger = logging.getLogger(__name__)


_MODULE_CACHE: Dict[str, ModuleType] = {}


def load_skill_module(module_name: str, skill_dir_name: str) -> ModuleType:
    """Load a skill module from a kebab-case skill directory."""
    cached = _MODULE_CACHE.get(module_name)
    if cached is not None:
        return cached

    skill_init = Path(__file__).resolve().parent / skill_dir_name / "__init__.py"
    spec = spec_from_file_location(module_name, skill_init)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load skill module '{module_name}' from {skill_init}")

    module = module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    _MODULE_CACHE[module_name] = module
    return module
