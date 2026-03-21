"""
Skills 加载器单元测试
"""

import pytest
from pathlib import Path
import tempfile
import shutil

from agi_walker.skills_loader import SkillsLoader, SkillMetadata


@pytest.fixture
def temp_skills_dir():
    """创建临时 skills 目录"""
    temp_dir = tempfile.mkdtemp()
    yield Path(temp_dir)
    shutil.rmtree(temp_dir)


@pytest.fixture
def sample_skill(temp_skills_dir):
    """创建示例 skill"""
    skill_dir = temp_skills_dir / "test-skill"
    skill_dir.mkdir()
    
    skill_md = skill_dir / "SKILL.md"
    skill_md.write_text("""---
name: test-skill
description: "测试 skill"
metadata:
  agi_walker:
    emoji: "🧪"
    category: "测试"
    requires:
      python_modules: ["numpy"]
---

# Test Skill

这是测试文档。
""", encoding="utf-8")
    
    return skill_dir


def test_skills_loader_init(temp_skills_dir):
    """测试加载器初始化"""
    loader = SkillsLoader(str(temp_skills_dir))
    assert loader.skills_dir == temp_skills_dir
    assert len(loader.skills) == 0


def test_parse_skill_metadata(sample_skill, temp_skills_dir):
    """测试解析 skill metadata"""
    loader = SkillsLoader(str(temp_skills_dir))
    assert len(loader.skills) == 1
    
    skill = loader.get_skill("test-skill")
    assert skill is not None
    assert skill.name == "test-skill"
    assert skill.description == "测试 skill"
    assert skill.emoji == "🧪"
    assert skill.category == "测试"
    assert "python_modules" in skill.requires
    assert "numpy" in skill.requires["python_modules"]


def test_get_skill_doc(sample_skill, temp_skills_dir):
    """测试获取 skill 文档"""
    loader = SkillsLoader(str(temp_skills_dir))
    doc = loader.get_skill_doc("test-skill")
    
    assert "# Test Skill" in doc
    assert "这是测试文档" in doc
    assert "---" not in doc  # frontmatter 应该被移除


def test_search_skills(temp_skills_dir):
    """测试 skill 搜索"""
    # 创建多个 skills
    for i in range(3):
        skill_dir = temp_skills_dir / f"skill-{i}"
        skill_dir.mkdir()
        (skill_dir / "SKILL.md").write_text(f"""---
name: skill-{i}
description: "描述 {i}"
---
""", encoding="utf-8")
    
    loader = SkillsLoader(str(temp_skills_dir))
    
    # 搜索名称
    results = loader.search_skills("skill-1")
    assert len(results) == 1
    assert results[0].name == "skill-1"
    
    # 搜索描述
    results = loader.search_skills("描述")
    assert len(results) == 3


def test_get_skill_by_category(temp_skills_dir):
    """测试按类别获取 skills"""
    # 创建不同类别的 skills
    categories = ["建模", "优化", "建模"]
    for i, cat in enumerate(categories):
        skill_dir = temp_skills_dir / f"skill-{i}"
        skill_dir.mkdir()
        (skill_dir / "SKILL.md").write_text(f"""---
name: skill-{i}
description: "描述"
metadata:
  agi_walker:
    category: "{cat}"
---
""", encoding="utf-8")
    
    loader = SkillsLoader(str(temp_skills_dir))
    
    modeling_skills = loader.get_skill_by_category("建模")
    assert len(modeling_skills) == 2
    
    opt_skills = loader.get_skill_by_category("优化")
    assert len(opt_skills) == 1


def test_invalid_skill_md(temp_skills_dir):
    """测试无效的 SKILL.md"""
    skill_dir = temp_skills_dir / "invalid-skill"
    skill_dir.mkdir()
    
    # 缺少 frontmatter
    (skill_dir / "SKILL.md").write_text("# No Frontmatter", encoding="utf-8")
    
    loader = SkillsLoader(str(temp_skills_dir))
    assert len(loader.skills) == 0


def test_missing_required_fields(temp_skills_dir):
    """测试缺少必需字段"""
    skill_dir = temp_skills_dir / "bad-skill"
    skill_dir.mkdir()
    
    # 缺少 description
    (skill_dir / "SKILL.md").write_text("""---
name: bad-skill
---
""", encoding="utf-8")
    
    loader = SkillsLoader(str(temp_skills_dir))
    assert len(loader.skills) == 0


def test_invalid_metadata_type(temp_skills_dir):
    """测试 metadata 类型错误时会跳过 skill"""
    skill_dir = temp_skills_dir / "bad-metadata-skill"
    skill_dir.mkdir()

    # metadata 必须是对象,不能是列表
    (skill_dir / "SKILL.md").write_text("""---
name: bad-metadata-skill
description: "描述"
metadata:
  - invalid
---
""", encoding="utf-8")

    loader = SkillsLoader(str(temp_skills_dir))
    assert loader.get_skill("bad-metadata-skill") is None
    assert len(loader.skills) == 0


def test_invalid_requires_item_type(temp_skills_dir):
    """测试 requires 子字段类型错误时会跳过 skill"""
    skill_dir = temp_skills_dir / "bad-requires-skill"
    skill_dir.mkdir()

    # requires.python_modules 必须是字符串列表
    (skill_dir / "SKILL.md").write_text("""---
name: bad-requires-skill
description: "描述"
metadata:
  agi_walker:
    requires:
      python_modules: numpy
---
""", encoding="utf-8")

    loader = SkillsLoader(str(temp_skills_dir))
    assert loader.get_skill("bad-requires-skill") is None
    assert len(loader.skills) == 0


def test_invalid_name_type(temp_skills_dir):
    """测试 name 类型错误时会跳过 skill"""
    skill_dir = temp_skills_dir / "bad-name-skill"
    skill_dir.mkdir()

    # name 必须是非空字符串
    (skill_dir / "SKILL.md").write_text("""---
name: [invalid]
description: "描述"
---
""", encoding="utf-8")

    loader = SkillsLoader(str(temp_skills_dir))
    assert loader.get_skill("bad-name-skill") is None
    assert len(loader.skills) == 0


def test_skill_display_name():
    """测试 skill 显示名称"""
    skill = SkillMetadata(
        name="test",
        description="Test",
        emoji="🧪"
    )
    assert skill.display_name == "🧪 test"


def test_get_all_categories(temp_skills_dir):
    """测试获取所有类别"""
    categories = ["建模", "优化", "转换", "建模"]
    for i, cat in enumerate(categories):
        skill_dir = temp_skills_dir / f"skill-{i}"
        skill_dir.mkdir()
        (skill_dir / "SKILL.md").write_text(f"""---
name: skill-{i}
description: "描述"
metadata:
  agi_walker:
    category: "{cat}"
---
""", encoding="utf-8")
    
    loader = SkillsLoader(str(temp_skills_dir))
    all_cats = loader.get_all_categories()
    
    assert len(all_cats) == 3  # 去重后
    assert "建模" in all_cats
    assert "优化" in all_cats
    assert "转换" in all_cats
