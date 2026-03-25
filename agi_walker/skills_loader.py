"""
AGI-Walker Skills 加载器

基于 Moltbot Skills 系统设计,提供模块化的知识包加载和管理功能。

Skills 是包含 SKILL.md 文件的目录,提供特定领域的专业知识和工具。
加载器实现渐进式披露机制,仅在需要时加载完整文档,避免上下文污染。
"""

import logging
import yaml
from pathlib import Path
from typing import Dict, List, Optional
from dataclasses import dataclass, field

# 配置日志
logger = logging.getLogger(__name__)


@dataclass
class SkillMetadata:
    """Skill 元数据

    Attributes:
        name: Skill 名称 (kebab-case)
        description: 功能描述和触发条件
        category: 分类 (建模/优化/转换/仿真/数据生成)
        emoji: 图标 emoji
        requires: 依赖项 (python_modules/bins/files)
        skill_dir: Skill 目录路径
    """

    name: str
    description: str
    category: str = "其他"
    emoji: str = "📦"
    requires: Dict[str, List[str]] = field(default_factory=dict)
    skill_dir: Optional[Path] = None

    @property
    def display_name(self) -> str:
        """获取显示名称 (emoji + name)"""
        return f"{self.emoji} {self.name}"


class SkillsLoader:
    """Skills 加载器

    负责扫描、解析和管理所有 skills。
    实现渐进式加载:
    1. 启动时加载所有 metadata (轻量)
    2. 触发时加载 SKILL.md body (中度)
    3. 按需加载 references/scripts (重度)
    """

    def __init__(self, skills_dir: str = "agi_walker/skills"):
        """初始化 Skills 加载器

        Args:
            skills_dir: Skills 根目录路径
        """
        self.skills_dir = Path(skills_dir)
        self.skills: Dict[str, SkillMetadata] = {}

        # 确保 skills 目录存在
        if not self.skills_dir.exists():
            self.skills_dir.mkdir(parents=True, exist_ok=True)

        # 加载所有 skills
        self.load_all_skills()

    def load_all_skills(self) -> None:
        """扫描并加载所有 skills 的 metadata"""
        try:
            skill_paths = list(self.skills_dir.iterdir())
        except OSError as e:
            logger.error(f"无法访问 skills 目录 {self.skills_dir}: {e}")
            return

        for skill_path in skill_paths:
            if not skill_path.is_dir():
                continue

            # 跳过特殊目录
            if skill_path.name.startswith((".", "__")):
                continue

            skill_md = skill_path / "SKILL.md"
            if not skill_md.exists():
                logger.warning(f"Skill 目录 {skill_path.name} 缺少 SKILL.md")
                continue

            try:
                metadata = self.parse_skill_metadata(skill_md)
                metadata.skill_dir = skill_path
                self.skills[metadata.name] = metadata
            except Exception as e:
                logger.error(f"加载 skill {skill_path.name} 失败: {e}")

    def parse_skill_metadata(self, skill_md: Path) -> SkillMetadata:
        """解析 SKILL.md 的 YAML frontmatter

        Args:
            skill_md: SKILL.md 文件路径

        Returns:
            SkillMetadata 对象

        Raises:
            ValueError: 如果 YAML frontmatter 格式无效
        """
        content = skill_md.read_text(encoding="utf-8")

        # 检查是否以 --- 开头
        if not content.startswith("---"):
            raise ValueError("无效的 SKILL.md: 缺少 YAML frontmatter")

        # 提取 YAML frontmatter (在两个 --- 之间)
        parts = content.split("---", 2)
        if len(parts) < 3:
            raise ValueError("无效的 SKILL.md: frontmatter 格式错误")

        yaml_content = parts[1]
        data = yaml.safe_load(yaml_content)

        # 提取必需字段
        if "name" not in data:
            raise ValueError("SKILL.md 缺少 'name' 字段")
        if "description" not in data:
            raise ValueError("SKILL.md 缺少 'description' 字段")

        # 验证 description
        desc = data["description"]
        if not isinstance(desc, str):
            raise ValueError(f"description必须是字符串, 收到: {type(desc)}")
        if not desc.strip():
            raise ValueError("description不能抛为空")

        # 提取可选的 metadata
        metadata = data.get("metadata", {}).get("agi_walker", {})

        # 验证并规范化 requires
        requires = metadata.get("requires", {})
        if requires is None:
            requires = {}

        normalized_requires = {}
        for key, val in requires.items():
            if val is None:
                normalized_requires[key] = []
                continue

            if not isinstance(val, list):
                # 尝试转换或报错? 这里严格要求 list
                raise ValueError(f"requires.{key} 必须是列表")

            for item in val:
                if not isinstance(item, str):
                    raise ValueError(f"requires.{key} 元素必须是字符串, 收到: {item}")

            normalized_requires[key] = val

        return SkillMetadata(
            name=data["name"],
            description=data["description"],
            emoji=metadata.get("emoji", "📦"),
            category=metadata.get("category", "其他"),
            requires=normalized_requires,
        )

    def get_skill(self, name: str) -> Optional[SkillMetadata]:
        """获取指定名称的 skill

        Args:
            name: Skill 名称

        Returns:
            SkillMetadata 或 None (如果不存在)
        """
        return self.skills.get(name)

    def get_skill_by_category(self, category: str) -> List[SkillMetadata]:
        """按类别获取 skills

        Args:
            category: 类别名称

        Returns:
            SkillMetadata 列表
        """
        return [skill for skill in self.skills.values() if skill.category == category]

    def search_skills(self, query: str) -> List[SkillMetadata]:
        """搜索 skills

        在 name 和 description 中进行模糊搜索。

        Args:
            query: 搜索关键词

        Returns:
            匹配的 SkillMetadata 列表
        """
        query = query.lower()
        return [
            skill
            for skill in self.skills.values()
            if query in skill.name.lower() or query in skill.description.lower()
        ]

    def get_categories(self) -> List[str]:
        """获取所有分类"""
        categories = set(skill.category for skill in self.skills.values())
        return sorted(categories)

    def get_skill_doc(self, skill_name: str) -> str:
        """获取 skill 的完整文档

        从 SKILL.md 中提取 body 部分 (去除 frontmatter)。

        Args:
            skill_name: Skill 名称

        Returns:
            Markdown 格式的文档内容

        Raises:
            FileNotFoundError: 如果 skill 不存在
        """
        skill = self.skills.get(skill_name)
        if not skill or not skill.skill_dir:
            raise FileNotFoundError(f"Skill '{skill_name}' 不存在")

        skill_md = skill.skill_dir / "SKILL.md"
        if not skill_md.exists():
            raise FileNotFoundError(f"Skill '{skill_name}' 的 SKILL.md 文件不存在")

        content = skill_md.read_text(encoding="utf-8")

        # 移除 frontmatter,返回纯文档
        parts = content.split("---", 2)
        return parts[2].strip() if len(parts) > 2 else content

    def get_all_categories(self) -> List[str]:
        """获取所有类别列表

        Returns:
            类别名称列表 (按字母排序)
        """
        categories = set(skill.category for skill in self.skills.values())
        return sorted(categories)

    def get_skills_list(self) -> List[SkillMetadata]:
        """获取所有 skills 列表

        Returns:
            SkillMetadata 列表 (按名称排序)
        """
        return sorted(self.skills.values(), key=lambda s: s.name)

    def validate_skill_dependencies(self, skill_name: str) -> Dict[str, List[str]]:
        """验证 skill 的依赖项

        检查所需的 Python 模块、二进制工具和物理文件是否可用。

        Args:
            skill_name: Skill 名称

        Returns:
            Dict包含缺失的依赖项:
            {
                "python_modules": ["missing_module1", ...],
                "bins": ["missing_bin1", ...],
                "files": ["missing_file1", ...]
            }
        """
        skill = self.skills.get(skill_name)
        if not skill:
            return {"error": [f"Skill '{skill_name}' 不存在"]}

        missing = {"python_modules": [], "bins": [], "files": []}

        # 检查 Python 模块
        if "python_modules" in skill.requires:
            import importlib

            for module in skill.requires["python_modules"]:
                try:
                    importlib.import_module(module)
                except ImportError:
                    missing["python_modules"].append(module)

        # 检查二进制工具
        if "bins" in skill.requires:
            import shutil

            for bin_name in skill.requires["bins"]:
                if not shutil.which(bin_name):
                    missing["bins"].append(bin_name)
                    
        # 检查物理文件
        if "files" in skill.requires:
            project_root = Path(__file__).parent.parent
            for file_path in skill.requires["files"]:
                full_path = project_root / file_path
                if not full_path.exists():
                    missing["files"].append(file_path)

        return missing

    def __len__(self) -> int:
        """返回加载的 skills 数量"""
        return len(self.skills)

    def __repr__(self) -> str:
        """字符串表示"""
        return f"SkillsLoader({len(self)} skills loaded)"


# 全局单例实例
_skills_loader_instance: Optional[SkillsLoader] = None


def get_skills_loader(skills_dir: str = "agi_walker/skills") -> SkillsLoader:
    """获取全局 Skills 加载器实例 (单例模式)

    Args:
        skills_dir: Skills 根目录路径

    Returns:
        SkillsLoader 实例
    """
    global _skills_loader_instance
    requested_dir = Path(skills_dir)
    if (
        _skills_loader_instance is None
        or _skills_loader_instance.skills_dir != requested_dir
    ):
        _skills_loader_instance = SkillsLoader(skills_dir)
    return _skills_loader_instance


# 便捷函数
def list_skills() -> List[SkillMetadata]:
    """列出所有 skills"""
    return get_skills_loader().get_skills_list()


def search_skills(query: str) -> List[SkillMetadata]:
    """搜索 skills"""
    return get_skills_loader().search_skills(query)


def get_skill_doc(skill_name: str) -> str:
    """获取 skill 文档"""
    return get_skills_loader().get_skill_doc(skill_name)
