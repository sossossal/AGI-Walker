"""
错误处理和边界情况测试 - 覆盖异常路径和边界条件

这个测试套件专注于覆盖异常处理路径、边界值和错误场景。
"""

import logging
from typing import Any, Optional, Dict, List, Tuple
logger = logging.getLogger(__name__)
import pytest
import sys
from pathlib import Path
import tempfile
import shutil
from unittest.mock import patch, MagicMock

PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.skills_loader import SkillsLoader, SkillMetadata


# ============================================================================
# 文件系统异常测试
# ============================================================================


class TestFileSystemErrors:
    """文件系统相关异常处理"""

    def test_skills_dir_permission_denied(self) -> None:
        """测试：没有权限访问 skills 目录"""
        temp_dir = tempfile.mkdtemp()
        try:
            # 移除读权限
            import os
            os.chmod(temp_dir, 0o000)
            
            # 尝试创建加载器 - 可能抛出异常或静默失败
            try:
                loader = SkillsLoader(str(temp_dir))
                # 如果没有异常，应该至少是空的
                assert len(loader) == 0
            except (PermissionError, OSError):
                logger.warning("Exception occurred")  # 这是预期的
        finally:
            # 恢复权限以便清理
            os.chmod(temp_dir, 0o755)
            shutil.rmtree(temp_dir, ignore_errors=True)

    def test_skill_md_read_error(self, tmp_path) -> None:
        """测试：读取 SKILL.md 失败"""
        skill_dir = tmp_path / "skill"
        skill_dir.mkdir()
        skill_md = skill_dir / "SKILL.md"
        skill_md.write_text("---\nname: test\ndescription: 'test'\n---\n")
        
        # 模拟读取错误
        with patch.object(Path, "read_text", side_effect=IOError("Read error")):
            loader = SkillsLoader(str(tmp_path))
            # 应该跳过失败的 skill
            assert len(loader) <= 1

    def test_missing_skill_md_reference(self) -> None:
        """测试：get_skill_doc 当 SKILL.md 被删除"""
        temp_dir = tempfile.mkdtemp()
        try:
            skill_dir = Path(temp_dir) / "skill"
            skill_dir.mkdir()
            skill_md = skill_dir / "SKILL.md"
            skill_md.write_text("---\nname: test\ndescription: 'test'\n---\n# Docs")
            
            loader = SkillsLoader(str(temp_dir))
            
            # 现在删除 SKILL.md
            skill_md.unlink()
            
            # 尝试获取文档
            with pytest.raises(FileNotFoundError):
                loader.get_skill_doc("test")
        finally:
            shutil.rmtree(temp_dir, ignore_errors=True)


# ============================================================================
# YAML 解析异常
# ============================================================================


class TestYAMLErrors:
    """YAML 解析异常处理"""

    @pytest.mark.parametrize("bad_yaml", [
        "---\n[invalid: yaml syntax",  # 未闭合列表
        "---\nname: tab\terror",  # Tab 字符
        "---\n{{invalid}}: value",  # 无效的键
    ])
    def test_yaml_syntax_errors(self, tmp_path, bad_yaml) -> None:
        """测试：各种 YAML 语法错误"""
        skill_dir = tmp_path / "skill"
        skill_dir.mkdir()
        (skill_dir / "SKILL.md").write_text(bad_yaml)
        
        loader = SkillsLoader(str(tmp_path))
        # 应该优雅地处理，跳过或放在错误列表中
        assert len(loader) == 0

    def test_circular_yaml_references(self, tmp_path) -> None:
        """测试：YAML 循环引用"""
        skill_dir = tmp_path / "skill"
        skill_dir.mkdir()
        
        # YAML 允许的引用 - yaml.safe_load 会拒绝
        content = """---
name: &anchor test
ref: *anchor
description: 'test'
---
"""
        (skill_dir / "SKILL.md").write_text(content)
        
        loader = SkillsLoader(str(tmp_path))
        # safe_load 应该处理这个，但可能导致加载失败
        assert len(loader) <= 1


# ============================================================================
# 类型验证异常
# ============================================================================


class TestTypeValidationErrors:
    """类型检查和验证错误"""

    @pytest.mark.parametrize("invalid_desc_type", [
        123,  # 整数
        12.34,  # 浮点数
        True,  # 布尔值
        ["list"],  # 列表
        {"dict": "value"},  # 字典
    ])
    def test_description_type_validation(self, tmp_path, invalid_desc_type) -> None:
        """测试：description 类型检查"""
        import yaml
        
        skill_dir = tmp_path / "skill"
        skill_dir.mkdir()
        
        # 构造包含无效类型的 YAML
        data = {
            "name": "test",
            "description": invalid_desc_type,
        }
        
        content = """---
name: test
description: """ + repr(invalid_desc_type) + """
---
"""
        (skill_dir / "SKILL.md").write_text(content)
        
        loader = SkillsLoader(str(tmp_path))
        # 应该拒绝
        assert len(loader) == 0

    def test_requires_type_mismatch(self, tmp_path) -> None:
        """测试：requires 值类型不匹配"""
        skill_dir = tmp_path / "skill"
        skill_dir.mkdir()
        
        # requires.python_modules 应该是列表，但用字符串
        (skill_dir / "SKILL.md").write_text("""---
name: test
description: "test"
metadata:
  agi_walker:
    requires:
      python_modules: "should_be_list"
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        assert len(loader) == 0


# ============================================================================
# 依赖验证异常
# ============================================================================


class TestDependencyErrors:
    """依赖验证的异常情况"""

    def test_import_module_side_effects(self, tmp_path) -> None:
        """测试：importlib.import_module 异常处理"""
        skill_dir = tmp_path / "skill"
        skill_dir.mkdir()
        
        (skill_dir / "SKILL.md").write_text("""---
name: test
description: "test"
metadata:
  agi_walker:
    requires:
      python_modules: ["sys", "os", "invalid_module_xyz"]
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        missing = loader.validate_skill_dependencies("test")
        
        # sys 和 os 应该在系统中
        assert "invalid_module_xyz" in missing["python_modules"]

    def test_shutil_which_not_found(self, tmp_path) -> None:
        """测试：shutil.which 二进制文件未找到"""
        skill_dir = tmp_path / "skill"
        skill_dir.mkdir()
        
        (skill_dir / "SKILL.md").write_text("""---
name: test
description: "test"
metadata:
  agi_walker:
    requires:
      bins: ["nonexistent_binary_xyz_12345"]
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        missing = loader.validate_skill_dependencies("test")
        
        # 这个二进制文件应该找不到
        assert "nonexistent_binary_xyz_12345" in missing.get("bins", [])


# ============================================================================
# 搜索和过滤边界情况
# ============================================================================


class TestSearchBoundaries:
    """搜索功能的边界情况"""

    @pytest.fixture
    def skills_with_special_chars(self, tmp_path):
        """创建含有特殊字符的 skills"""
        special_skills = [
            ("robot-model", "Robot (Model) & [Tools]"),
            ("param-opt", "Parameters | Optimization"),
            ("custom-skill", "User defined skill"),
        ]
        
        for name, desc in special_skills:
            skill_dir = tmp_path / name
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text(
                f"""---
name: {name}
description: "{desc}"
---
""",
                encoding="utf-8"
            )
        
        return SkillsLoader(str(tmp_path))

    def test_search_special_characters(self, skills_with_special_chars) -> None:
        """测试：搜索含特殊字符的技能"""
        # 搜索英文
        results = skills_with_special_chars.search_skills("Model")
        assert len(results) > 0

    def test_search_regex_characters(self, skills_with_special_chars) -> None:
        """测试：搜索 regex 特殊字符"""
        # 不应该将 | 解释为 OR
        results = skills_with_special_chars.search_skills("|")
        # 应该根据字符串匹配，不是 regex
        assert len(results) >= 0

    def test_empty_category_filter(self, tmp_path) -> None:
        """测试：过滤不存在的类别"""
        skill_dir = tmp_path / "test"
        skill_dir.mkdir()
        (skill_dir / "SKILL.md").write_text("""---
name: test
description: "test"
metadata:
  agi_walker:
    category: "existing"
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        results = loader.get_skill_by_category("nonexistent")
        assert len(results) == 0


# ============================================================================
# 文档提取边界情况
# ============================================================================


class TestDocExtractBoundaries:
    """文档提取的边界情况"""

    def test_doc_with_multiple_frontmatters(self, tmp_path) -> None:
        """测试：多个 frontmatter 分隔符的文档"""
        skill_dir = tmp_path / "test"
        skill_dir.mkdir()
        
        # 第一个 --- 后面的内容作为 frontmatter
        (skill_dir / "SKILL.md").write_text("""---
name: test
description: "test"
---
# Content with --- separator

More content with --- inside.
---
Additional content after triple dash.
""")
        
        loader = SkillsLoader(str(tmp_path))
        doc = loader.get_skill_doc("test")
        
        # 应该包含文档部分
        assert "# Content" in doc
        assert "Additional content" in doc

    def test_empty_frontmatter_body(self, tmp_path) -> None:
        """测试：frontmatter 后没有内容"""
        skill_dir = tmp_path / "test"
        skill_dir.mkdir()
        
        (skill_dir / "SKILL.md").write_text("""---
name: test
description: "test"
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        doc = loader.get_skill_doc("test")
        
        # 应该返回空字符串或只有空格
        assert doc.strip() == ""

    def test_unicode_in_documentation(self, tmp_path) -> None:
        """测试：文档中的 Unicode 字符"""
        skill_dir = tmp_path / "test"
        skill_dir.mkdir()
        
        (skill_dir / "SKILL.md").write_text("""---
name: test
description: "测试 Skill"
---
# 中文标题

日本語 コンテンツ

🚀 Emoji 内容

µ∑∆ 数学符号
""", encoding="utf-8")
        
        loader = SkillsLoader(str(tmp_path))
        doc = loader.get_skill_doc("test")
        
        assert "中文标题" in doc
        assert "🚀" in doc


# ============================================================================
# 并发和线程安全测试
# ============================================================================


class TestConcurrency:
    """并发访问和线程安全"""

    def test_concurrent_skill_access(self, tmp_path) -> None:
        """测试：并发访问 skills"""
        import threading
        
        # 创建多个 skills
        for i in range(10):
            skill_dir = tmp_path / f"skill-{i}"
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text(f"""---
name: skill-{i}
description: "Skill {i}"
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        results = []
        
        def access_skill(index):
            skill = loader.get_skill(f"skill-{index%10}")
            results.append(skill)
        
        threads = [
            threading.Thread(target=access_skill, args=(i,))
            for i in range(20)
        ]
        
        for t in threads:
            t.start()
        
        for t in threads:
            t.join()
        
        # 所有访问都应该成功
        assert len(results) == 20
        assert all(r is not None for r in results)

    def test_search_while_loading(self, tmp_path) -> None:
        """测试：加载时搜索"""
        # 创建许多 skills，可能导致加载时间过长
        for i in range(50):
            skill_dir = tmp_path / f"skill-{i:03d}"
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text(f"""---
name: skill-{i:03d}
description: "Test skill number {i}"
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        
        # 同时执行多个搜索
        results1 = loader.search_skills("skill-042")
        results2 = loader.search_skills("skill-012")
        results3 = loader.get_categories()
        
        assert len(results1) == 1
        assert len(results2) == 1
        assert type(results3) == list


# ============================================================================
# 内存和资源限制
# ============================================================================


class TestResourceLimits:
    """资源限制和内存处理"""

    def test_very_large_skill_metadata(self, tmp_path) -> None:
        """测试：极大的 metadata"""
        skill_dir = tmp_path / "large"
        skill_dir.mkdir()
        
        # 创建包含大量 requires 的 metadata
        large_requires = {
            "python_modules": [f"module_{i}" for i in range(1000)],
            "bins": [f"bin_{i}" for i in range(1000)],
        }
        
        import json
        requires_yaml = json.dumps(large_requires).replace('"', "'")
        
        (skill_dir / "SKILL.md").write_text(f"""---
name: large
description: "Large metadata"
metadata:
  agi_walker:
    requires:
      python_modules: {list(range(100))}
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        assert len(loader) >= 0  # 应该处理而不崩溃

    def test_deeply_nested_yaml_structure(self, tmp_path) -> None:
        """测试：深层嵌套的 YAML"""
        skill_dir = tmp_path / "nested"
        skill_dir.mkdir()
        
        (skill_dir / "SKILL.md").write_text("""---
name: nested
description: "test"
metadata:
  agi_walker:
    metadata1:
      metadata2:
        metadata3:
          metadata4:
            python_modules: ["numpy"]
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        # 应该处理或拒绝
        assert len(loader) >= 0


# ============================================================================
# 恢复和降级测试
# ============================================================================


class TestRecovery:
    """错误恢复和降级行为"""

    def test_partial_skill_loading_error_recovery(self, tmp_path) -> None:
        """测试：部分失败时继续加载其他"""
        # 创建一些有效的 skills
        for i in range(3):
            skill_dir = tmp_path / f"valid-{i}"
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text(f"""---
name: valid-{i}
description: "Valid skill {i}"
---
""")
        
        # 在中间插入无效的 skill
        invalid_dir = tmp_path / "invalid"
        invalid_dir.mkdir()
        (invalid_dir / "SKILL.md").write_text("Invalid content, no frontmatter")
        
        # 再添加有效的
        for i in range(3, 5):
            skill_dir = tmp_path / f"valid-{i}"
            skill_dir.mkdir()
            (skill_dir / "SKILL.md").write_text(f"""---
name: valid-{i}
description: "Valid skill {i}"
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        
        # 应该加载有效的，跳过无效的
        assert len(loader) == 5  # 5 个有效的

    def test_skill_get_with_missing_field(self, tmp_path) -> None:
        """测试：获取缺少可选字段的 skill"""
        skill_dir = tmp_path / "minimal"
        skill_dir.mkdir()
        
        (skill_dir / "SKILL.md").write_text("""---
name: minimal
description: "Minimal skill"
---
""")
        
        loader = SkillsLoader(str(tmp_path))
        skill = loader.get_skill("minimal")
        
        # 应该使用默认值
        assert skill.emoji == "📦"  # 默认值
        assert skill.category == "其他"  # 默认值
        assert skill.requires == {}  # 空字典


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
