from enum import Enum
from typing import Any, Dict, List, Optional
from pydantic import BaseModel, Field


class ParameterType(str, Enum):
    STRING = "string"
    NUMBER = "number"
    BOOLEAN = "boolean"
    DICT = "dict"
    LIST = "list"
    FILE_PATH = "file_path"


class SkillParameter(BaseModel):
    """Definition of a single input or output parameter."""

    name: str
    type: ParameterType
    description: str
    default: Optional[Any] = None
    required: bool = True


class SkillContract(BaseModel):
    """The formal interface definition for a Skill."""

    name: str
    version: str
    description: str
    category: str
    emoji: str = "🧩"
    inputs: List[SkillParameter] = Field(default_factory=list)
    outputs: List[SkillParameter] = Field(default_factory=list)

    def validate_inputs(self, params: Dict[str, Any]):
        """Validation logic would go here."""
        pass


class SkillCatalog(BaseModel):
    """A collection of all available skills for the visual editor."""

    skills: List[SkillContract] = Field(default_factory=list)
