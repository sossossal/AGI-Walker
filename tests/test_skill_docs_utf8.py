from pathlib import Path

import yaml


SKILLS_DIR = Path("agi_walker/skills")


def test_skill_docs_are_strict_utf8_and_valid_frontmatter() -> None:
    for skill_md in SKILLS_DIR.glob("*/SKILL.md"):
        content = skill_md.read_text(encoding="utf-8")
        assert content.startswith("---"), f"{skill_md} 缺少 frontmatter"

        parts = content.split("---", 2)
        assert len(parts) == 3, f"{skill_md} frontmatter 格式错误"

        frontmatter = yaml.safe_load(parts[1])
        assert isinstance(frontmatter, dict), f"{skill_md} frontmatter 不是对象"
        assert frontmatter.get("name"), f"{skill_md} 缺少 name"
        assert frontmatter.get("description"), f"{skill_md} 缺少 description"
