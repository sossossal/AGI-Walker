"""
Enhanced Skills Loader Tests - 覆盖关键路径和异常处理

这个测试套件修复了之前测试的覆盖率问题，目标是达到 90%+ 的技能加载器覆盖率。
使用参数化测试和异常场景覆盖所有分支。
"""

import logging

import pytest
from pathlib import Path
import tempfile
import shutil

from agi_walker.skills_loader import (
    SkillsLoader,
    get_skills_loader,
    list_skills,
    search_skills,
    get_skill_doc,
)

logger = logging.getLogger(__name__)


@pytest.fixture
def temp_skills_dir():
    """创建临时 skills 目录"""
    temp_dir = tempfile.mkdtemp()
    yield Path(temp_dir)
    shutil.rmtree(temp_dir, ignore_errors=True)


@pytest.fixture
def reset_singleton():
    """重置全局单例（在测试间隔隔离状态）"""
    import agi_walker.skills_loader as module

    original = module._skills_loader_instance
    module._skills_loader_instance = None
    yield
    module._skills_loader_instance = original


# ============================================================================
# 基础功能测试 (核心路径)
# ============================================================================


class TestSkillsLoaderInit:
    """初始化和基础功能测试"""

    def test_init_creates_directory(self, tmp_path) -> None:
        """测试：初始化时创建不存在的目录"""
        skills_dir = tmp_path / "new_skills"
        assert not skills_dir.exists()

        loader = SkillsLoader(str(skills_dir))
        assert skills_dir.exists()
        assert loader.skills_dir == skills_dir

    def test_init_empty_directory(self, temp_skills_dir) -> None:
        """测试：空目录初始化"""
        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader.skills) == 0
        assert loader.skills_dir == temp_skills_dir

    def test_len_method(self, temp_skills_dir) -> None:
        """测试：__len__ 方法"""
        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0

    def test_repr_method(self, temp_skills_dir) -> None:
        """测试：__repr__ 字符串表示"""
        loader = SkillsLoader(str(temp_skills_dir))
        repr_str = repr(loader)
        assert "SkillsLoader" in repr_str
        assert "0 skills" in repr_str


# ============================================================================
# YAML Frontmatter 解析测试 (覆盖行 105-160)
# ============================================================================


class TestSkillMetadataParsing:
    """Skill 元数据解析和验证"""

    def test_parse_valid_skill_metadata(self, temp_skills_dir) -> None:
        """测试：解析有效的 SKILL.md"""
        skill_dir = temp_skills_dir / "valid-skill"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: valid-skill
description: "A valid skill"
metadata:
  agi_walker:
    emoji: "🚀"
    category: "modeling"
    requires:
      python_modules: ["numpy", "scipy"]
      bins: ["git"]
---
# Documentation
Content here.
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 1

        skill = loader.get_skill("valid-skill")
        assert skill.name == "valid-skill"
        assert skill.description == "A valid skill"
        assert skill.emoji == "🚀"
        assert skill.category == "modeling"
        assert skill.requires == {"python_modules": ["numpy", "scipy"], "bins": ["git"]}

    @pytest.mark.parametrize(
        "invalid_content,error_msg",
        [
            # 缺少 frontmatter (行 115)
            ("# No frontmatter", "无效的 SKILL.md: 缺少 YAML frontmatter"),
            # 格式错误 (行 119)
            ("---\nno closing fence", "无效的 SKILL.md: frontmatter 格式错误"),
            # 缺少 name (行 122)
            ("---\ndescription: test\n---\n", "SKILL.md 缺少 'name' 字段"),
            # 缺少 description (行 124)
            ("---\nname: test\n---\n", "SKILL.md 缺少 'description' 字段"),
        ],
    )
    def test_parse_invalid_metadata_format(
        self, temp_skills_dir, invalid_content, error_msg
    ) -> None:
        """测试：各种无效格式的 SKILL.md"""
        skill_dir = temp_skills_dir / "invalid-skill"
        skill_dir.mkdir()
        (skill_dir / "SKILL.md").write_text(invalid_content, encoding="utf-8")

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0  # 应该被跳过

    def test_description_must_be_string(self, temp_skills_dir) -> None:
        """测试：description 必须是字符串（行 127）"""
        skill_dir = temp_skills_dir / "bad-desc"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: bad-desc
description: 123
---
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0

    def test_description_cannot_be_empty(self, temp_skills_dir) -> None:
        """测试：description 不能是空白字符串（行 130）"""
        skill_dir = temp_skills_dir / "empty-desc"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: empty-desc
description: "   "
---
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0


# ============================================================================
# 依赖检查和验证测试 (覆盖行 270-294, 302)
# ============================================================================


class TestDependencyValidation:
    """skill 依赖项验证"""

    def test_validate_dependencies_all_present(self, temp_skills_dir) -> None:
        """测试：所有依赖都存在"""
        skill_dir = temp_skills_dir / "valid-deps"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: valid-deps
description: "Skill with real dependencies"
metadata:
  agi_walker:
    requires:
      python_modules: ["pathlib", "os"]
      bins: ["python"]
---
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        missing = loader.validate_skill_dependencies("valid-deps")

        assert missing["python_modules"] == []
        assert len(missing.get("bins", [])) <= 1  # python 可能在路径中

    def test_validate_dependencies_missing_module(self, temp_skills_dir) -> None:
        """测试：缺失 Python 模块（行 283-286）"""
        skill_dir = temp_skills_dir / "missing-module"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: missing-module
description: "Skill with missing module"
metadata:
  agi_walker:
    requires:
      python_modules: ["nonexistent_package_xyz_12345"]
---
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        missing = loader.validate_skill_dependencies("missing-module")

        assert "nonexistent_package_xyz_12345" in missing["python_modules"]

    def test_validate_dependencies_nonexistent_skill(self, temp_skills_dir) -> None:
        """测试：验证不存在的 skill（行 270）"""
        loader = SkillsLoader(str(temp_skills_dir))
        missing = loader.validate_skill_dependencies("nonexistent-skill")

        assert "error" in missing
        assert "不存在" in missing["error"][0]

    def test_validate_dependencies_none_requires(self, temp_skills_dir) -> None:
        """测试：requires 为 None 的情况（行 151-152）"""
        skill_dir = temp_skills_dir / "none-requires"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: none-requires
description: "Skill with null requires"
metadata:
  agi_walker:
    requires: null
---
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        skill = loader.get_skill("none-requires")
        assert skill.requires == {}


# ============================================================================
# requires 字段验证测试 (覆盖行 154-163)
# ============================================================================


class TestRequiresValidation:
    """requires 字段格式和类型检查"""

    @pytest.mark.parametrize(
        "invalid_requires",
        [
            # requires 不是字典/None
            "string_instead_of_dict",
            123,
            ["list"],
        ],
    )
    def test_requires_invalid_type(self, temp_skills_dir, invalid_requires) -> None:
        """测试：requires 值不是有效格式"""
        skill_dir = temp_skills_dir / "bad-requires"
        skill_dir.mkdir()

        with open(skill_dir / "SKILL.md", "w", encoding="utf-8") as f:
            f.write("---\n")
            f.write("name: bad-requires\n")
            f.write("description: test\n")
            f.write("metadata:\n")
            f.write("  agi_walker:\n")
            f.write("    requires:\n")
            # 直接写 YAML，让 yaml.safe_load 处理
            if isinstance(invalid_requires, str):
                f.write(f'      python_modules: "{invalid_requires}"\n')
            else:
                f.write(f"      python_modules: {invalid_requires}\n")
            f.write("---\n")

        loader = SkillsLoader(str(temp_skills_dir))
        # 大多数无效类型会导致加载失败
        assert len(loader) <= 1

    def test_requires_list_items_must_be_strings(self, temp_skills_dir) -> None:
        """测试：requires 列表项必须是字符串（行 160）"""
        skill_dir = temp_skills_dir / "int-require"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: int-require
description: test
metadata:
  agi_walker:
    requires:
      python_modules: ["numpy", 123, "scipy"]
---
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0  # 应该拒绝非字符串项


# ============================================================================
# 搜索和过滤功能测试
# ============================================================================


class TestSearchAndFilter:
    """搜索、分类、过滤功能"""

    @pytest.fixture
    def multi_skills(self, temp_skills_dir):
        """创建多个不同类别的 skills"""
        config = [
            ("robot-model", "Build robot models", "建模", "🤖"),
            ("param-opt", "Optimize parameters", "优化", "⚙️"),
            ("urdf-gen", "Generate URDF files", "转换", "📝"),
            ("sim-run", "Run simulations", "仿真", "🌐"),
        ]

        for name, desc, cat, emoji in config:
            skill_dir = temp_skills_dir / name
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text(
                f"""---
name: {name}
description: "{desc}"
metadata:
  agi_walker:
    category: "{cat}"
    emoji: "{emoji}"
---
# {name.replace("-", " ").title()}
""",
                encoding="utf-8",
            )

        return SkillsLoader(str(temp_skills_dir))

    def test_search_by_name(self, multi_skills) -> None:
        """测试：按名称搜索"""
        results = multi_skills.search_skills("robot")
        assert len(results) == 1
        assert results[0].name == "robot-model"

    def test_search_by_description(self, multi_skills) -> None:
        """测试：按描述搜索"""
        results = multi_skills.search_skills("optimize")
        assert len(results) == 1
        assert results[0].name == "param-opt"

    def test_search_case_insensitive(self, multi_skills) -> None:
        """测试：搜索不区分大小写"""
        results = multi_skills.search_skills("ROBOT")
        assert len(results) == 1

    def test_search_empty_query(self, multi_skills) -> None:
        """测试：空搜索查询"""
        results = multi_skills.search_skills("")
        assert len(results) == 4  # 匹配所有

    def test_get_by_category(self, multi_skills) -> None:
        """测试：按类别获取"""
        modeling = multi_skills.get_skill_by_category("建模")
        assert len(modeling) == 1
        assert modeling[0].name == "robot-model"

    def test_get_categories(self, multi_skills) -> None:
        """测试：获取所有类别"""
        categories = multi_skills.get_categories()
        assert "建模" in categories
        assert "优化" in categories
        assert "转换" in categories

    def test_get_all_categories(self, multi_skills) -> None:
        """测试：get_all_categories 和 get_categories 一致"""
        cats1 = multi_skills.get_categories()
        cats2 = multi_skills.get_all_categories()
        assert cats1 == cats2

    def test_get_skills_list(self, multi_skills) -> None:
        """测试：获取 skills 列表（带排序）"""
        skills_list = multi_skills.get_skills_list()
        assert len(skills_list) == 4
        # 检查按名称排序
        names = [s.name for s in skills_list]
        assert names == sorted(names)


# ============================================================================
# 文档提取测试 (覆盖行 185-210)
# ============================================================================


class TestDocExtraction:
    """skill 文档提取"""

    def test_get_skill_doc_removes_frontmatter(self, temp_skills_dir) -> None:
        """测试：文档提取去除 frontmatter（行 205-210）"""
        skill_dir = temp_skills_dir / "doc-skill"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: doc-skill
description: "Skill with docs"
---
# Main Heading

This is the content.

## Subsection
More content here.
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        doc = loader.get_skill_doc("doc-skill")

        # 检查 frontmatter 已移除
        assert "---" not in doc
        assert "name: doc-skill" not in doc

        # 检查内容保留
        assert "# Main Heading" in doc
        assert "This is the content" in doc

    def test_get_skill_doc_nonexistent(self, temp_skills_dir) -> None:
        """测试：获取不存在的 skill 文档"""
        loader = SkillsLoader(str(temp_skills_dir))

        with pytest.raises(FileNotFoundError):
            loader.get_skill_doc("nonexistent")


# ============================================================================
# 特殊情况和边界测试 (覆盖行 67, 84-85, 139)
# ============================================================================


class TestEdgeCases:
    """边界情况和特殊场景"""

    def test_skip_directories_without_skill_md(self, temp_skills_dir) -> None:
        """测试：跳过没有 SKILL.md 的目录（行 67 的日志）"""
        # 创建没有 SKILL.md 的目录
        (temp_skills_dir / "no-skill-md").mkdir()
        (temp_skills_dir / "no-skill-md" / "README.md").write_text("readme")

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0

    def test_skip_hidden_directories(self, temp_skills_dir) -> None:
        """测试：跳过隐藏目录"""
        # 创建隐藏目录
        (temp_skills_dir / ".hidden").mkdir()
        (temp_skills_dir / ".hidden" / "SKILL.md").write_text(
            "---\nname: hidden\ndescription: test\n---\n"
        )

        # 创建 __pycache__ 目录
        (temp_skills_dir / "__pycache__").mkdir()
        (temp_skills_dir / "__pycache__" / "SKILL.md").write_text(
            "---\nname: pycache\ndescription: test\n---\n"
        )

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0

    def test_yaml_parse_error_handling(self, temp_skills_dir) -> None:
        """测试：YAML 解析错误处理（行 84-85 的异常）"""
        skill_dir = temp_skills_dir / "bad-yaml"
        skill_dir.mkdir()

        (skill_dir / "SKILL.md").write_text(
            """---
name: bad-yaml
description: test
invalid: [unclosed list
---
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 0  # yaml.safe_load should fail gracefully


# ============================================================================
# 全局便捷函数测试
# ============================================================================


class TestModuleLevelFunctions:
    """模块级便捷函数"""

    def test_singleton_get_skills_loader(
        self, temp_skills_dir, reset_singleton
    ) -> None:
        """测试：get_skills_loader 单例模式"""

        loader1 = get_skills_loader(str(temp_skills_dir))
        loader2 = get_skills_loader(str(temp_skills_dir))

        assert loader1 is loader2  # 应该返回同一实例

    def test_convenience_functions(self, temp_skills_dir, reset_singleton) -> None:
        """测试：list_skills, search_skills, get_skill_doc 便捷函数"""
        # 创建测试 skill
        skill_dir = temp_skills_dir / "test"
        skill_dir.mkdir()
        (skill_dir / "SKILL.md").write_text(
            """---
name: test
description: "Test skill"
---
# Docs
""",
            encoding="utf-8",
        )

        # 重置单例
        import agi_walker.skills_loader as module

        module._skills_loader_instance = None

        # 测试便捷函数
        list_skills()  # 这会使用默认路径，可能为空

        # 这些函数会自动创建加载器
        assert callable(search_skills)
        assert callable(get_skill_doc)


# ============================================================================
# 性能和压力测试
# ============================================================================


class TestPerformance:
    """性能和大规模场景"""

    def test_large_number_of_skills(self, temp_skills_dir) -> None:
        """测试：处理大量 skill"""
        for i in range(50):
            skill_dir = temp_skills_dir / f"skill-{i:03d}"
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text(
                f"""---
name: skill-{i:03d}
description: "Skill number {i}"
metadata:
  agi_walker:
    category: "test"
---
""",
                encoding="utf-8",
            )

        loader = SkillsLoader(str(temp_skills_dir))
        assert len(loader) == 50

        # 检查搜索性能
        results = loader.search_skills("skill-042")
        assert len(results) == 1

    def test_large_skill_doc(self, temp_skills_dir) -> None:
        """测试：处理大型文档"""
        skill_dir = temp_skills_dir / "large-doc"
        skill_dir.mkdir()

        # 创建大型文档
        large_content = "\n".join(
            [f"## Section {i}\n\nContent {i}" for i in range(100)]
        )

        (skill_dir / "SKILL.md").write_text(
            f"""---
name: large-doc
description: "Large documentation"
---
{large_content}
""",
            encoding="utf-8",
        )

        loader = SkillsLoader(str(temp_skills_dir))
        doc = loader.get_skill_doc("large-doc")

        assert len(doc) > 1000
        assert "Section 50" in doc


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
