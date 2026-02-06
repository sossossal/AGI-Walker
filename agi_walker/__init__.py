"""
AGI-Walker - Parametric Robot Design and AI Training Platform
"""

from .skills_loader import (
    SkillMetadata,
    SkillsLoader,
    get_skills_loader,
    list_skills,
    search_skills,
    get_skill_doc,
)

__version__ = "3.0.0"
__all__ = [
    "SkillMetadata",
    "SkillsLoader",
    "get_skills_loader",
    "list_skills",
    "search_skills",
    "get_skill_doc",
]
