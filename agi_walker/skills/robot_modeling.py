"""
Compatibility wrapper for the robot-modeling skill.
"""

import logging
logger = logging.getLogger(__name__)
from agi_walker.skills._compat import load_skill_module


_module = load_skill_module(__name__, "robot-modeling")

RobotConfig = _module.RobotConfig
RobotBuilder = _module.RobotBuilder
load_template = _module.load_template
list_templates = _module.list_templates

__all__ = ["RobotConfig", "RobotBuilder", "load_template", "list_templates"]
