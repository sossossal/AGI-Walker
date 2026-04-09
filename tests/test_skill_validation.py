"""
Unit tests for Skill validation logic in SkillsLoader.
Tests specific ValueError messages for invalid inputs.
"""

import pytest
from pathlib import Path
import tempfile
import shutil
import logging

from agi_walker.skills_loader import SkillsLoader


logger = logging.getLogger(__name__)


@pytest.fixture
def temp_skills_dir():
    """Create a temporary skills directory"""
    temp_dir = tempfile.mkdtemp()
    yield Path(temp_dir)
    shutil.rmtree(temp_dir)


def create_skill_file(skills_dir, name, content):
    """Helper to create a skill file"""
    skill_dir = skills_dir / name
    skill_dir.mkdir(parents=True, exist_ok=True)
    md_file = skill_dir / "SKILL.md"
    md_file.write_text(content, encoding="utf-8")
    return md_file


def test_validation_description_type(temp_skills_dir) -> None:
    """Test description type validation (must be string)"""
    # Create a loader instance
    loader = SkillsLoader(str(temp_skills_dir))

    # Create invalid skill file
    md_file = create_skill_file(
        temp_skills_dir,
        "bad-desc-list",
        """---
name: bad-desc-list
description: ["not", "string"]
---
""",
    )

    # Assert ValueError is raised with specific message
    with pytest.raises(ValueError, match="description必须是字符串"):
        loader.parse_skill_metadata(md_file)


def test_validation_description_empty(temp_skills_dir) -> None:
    """Test description content validation (cannot be empty)"""
    loader = SkillsLoader(str(temp_skills_dir))

    # Case 1: Empty string
    md_file = create_skill_file(
        temp_skills_dir,
        "empty-desc",
        """---
name: empty-desc
description: ""
---
""",
    )
    with pytest.raises(ValueError, match="description不能抛为空"):
        loader.parse_skill_metadata(md_file)

    # Case 2: Whitespace only
    md_file2 = create_skill_file(
        temp_skills_dir,
        "white-desc",
        """---
name: white-desc
description: "   "
---
""",
    )
    with pytest.raises(ValueError, match="description不能抛为空"):
        loader.parse_skill_metadata(md_file2)


def test_validation_requires_element_type(temp_skills_dir) -> None:
    """Test requires list element validation (must be strings)"""
    loader = SkillsLoader(str(temp_skills_dir))

    md_file = create_skill_file(
        temp_skills_dir,
        "bad-req-elem",
        """---
name: bad-req-elem
description: "valid"
metadata:
  agi_walker:
    requires:
      python_modules: ["numpy", 123]
---
""",
    )
    # regex match for "requires.python_modules 元素必须是字符串"
    with pytest.raises(ValueError, match="requires.*元素必须是字符串"):
        loader.parse_skill_metadata(md_file)


def test_validation_requires_is_list(temp_skills_dir) -> None:
    """Test requires field validation (must be list)"""
    loader = SkillsLoader(str(temp_skills_dir))

    md_file = create_skill_file(
        temp_skills_dir,
        "bad-req-type",
        """---
name: bad-req-type
description: "valid"
metadata:
  agi_walker:
    requires:
      python_modules: "not-a-list"
---
""",
    )
    with pytest.raises(ValueError, match="requires.*必须是列表"):
        loader.parse_skill_metadata(md_file)
