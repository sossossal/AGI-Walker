"""
AGI-Walker - Parametric Robot Design and AI Training Platform
"""

import logging
logger = logging.getLogger(__name__)
from .skills_loader import (
    SkillMetadata,
    SkillsLoader,
    get_skills_loader,
    list_skills,
    search_skills,
    get_skill_doc,
)
from .skills.robot_modeling import RobotBuilder, RobotConfig, load_template, list_templates
from .skills.parameter_optimizer import optimize_mass_distribution, tune_pid_controller
from .skills.urdf_generator import convert_to_urdf, convert_to_sdf, validate_urdf

__version__ = "3.0.0"
__all__ = [
    "SkillMetadata",
    "SkillsLoader",
    "get_skills_loader",
    "list_skills",
    "search_skills",
    "get_skill_doc",
    "RobotConfig",
    "RobotBuilder",
    "load_template",
    "list_templates",
    "optimize_mass_distribution",
    "tune_pid_controller",
    "convert_to_urdf",
    "convert_to_sdf",
    "validate_urdf",
]
