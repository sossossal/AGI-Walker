"""
Compatibility wrapper for the urdf-generator skill.
"""

import logging
from agi_walker.skills._compat import load_skill_module

logger = logging.getLogger(__name__)


_module = load_skill_module(__name__, "urdf-generator")

URDFLink = _module.URDFLink
URDFJoint = _module.URDFJoint
URDFGenerator = _module.URDFGenerator
convert_to_urdf = _module.convert_to_urdf
convert_to_sdf = _module.convert_to_sdf
validate_urdf = _module.validate_urdf

__all__ = [
    "URDFLink",
    "URDFJoint",
    "URDFGenerator",
    "convert_to_urdf",
    "convert_to_sdf",
    "validate_urdf",
]
