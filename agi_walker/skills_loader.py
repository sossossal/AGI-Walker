"""
AGI-Walker Skills 加载器

基于 MoltBot Skills 系统设计,提供模块化的知识包加载和管理功能。

Skills 是包含 SKILL.md 文件的目录,提供特定领域的专业知识和工具。
加载器实现渐进式披露机制,仅在需要时加载完整文档,避免上下文污染。
"""

import yaml
from pathlib import Path
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field


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
        for skill_path in self.skills_dir.iterdir():
            if not skill_path.is_dir():
                continue
            
            # 跳过特殊目录
            if skill_path.name.startswith(('.', '__')):
                continue
            
            skill_md = skill_path / "SKILL.md"
            if not skill_md.exists():
                print(f"警告: Skill 目录 {skill_path.name} 缺少 SKILL.md")
                continue
            
            try:
                metadata = self.parse_skill_metadata(skill_md)
                metadata.skill_dir = skill_path
                self.skills[metadata.name] = metadata
            except Exception as e:
                print(f"错误: 加载 skill {skill_path.name} 失败: {e}")
    
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
            raise ValueError(f"无效的 SKILL.md: 缺少 YAML frontmatter")
        
        # 提取 YAML frontmatter (在两个 --- 之间)
        parts = content.split("---", 2)
        if len(parts) < 3:
            raise ValueError(f"无效的 SKILL.md: frontmatter 格式错误")
        
        yaml_content = parts[1]
        data = yaml.safe_load(yaml_content)
        if not isinstance(data, dict):
            raise ValueError("无效的 SKILL.md: frontmatter 必须是 YAML 映射")
        
        # 提取必需字段
        if "name" not in data:
            raise ValueError("SKILL.md 缺少 'name' 字段")
        if "description" not in data:
            raise ValueError("SKILL.md 缺少 'description' 字段")

        name = data["name"]
        description = data["description"]
        if not isinstance(name, str) or not name.strip():
            raise ValueError("SKILL.md 的 'name' 字段必须是非空字符串")
        if not isinstance(description, str) or not description.strip():
            raise ValueError("SKILL.md 的 'description' 字段必须是非空字符串")
        
        # 提取可选的 metadata
        raw_metadata = data.get("metadata") or {}
        if not isinstance(raw_metadata, dict):
            raise ValueError("SKILL.md 的 'metadata' 字段必须是对象")

        metadata = raw_metadata.get("agi_walker", {}) or {}
        if not isinstance(metadata, dict):
            raise ValueError("SKILL.md 的 'metadata.agi_walker' 字段必须是对象")

        requires = metadata.get("requires", {}) or {}
        if not isinstance(requires, dict):
            raise ValueError("SKILL.md 的 'metadata.agi_walker.requires' 字段必须是对象")

        normalized_requires = self._normalize_requires(requires)
        
        return SkillMetadata(
            name=name,
            description=description,
            emoji=metadata.get("emoji", "📦"),
            category=metadata.get("category", "其他"),
            requires=normalized_requires
        )
    

    def _normalize_requires(self, requires: Dict[str, Any]) -> Dict[str, List[str]]:
        """规范化 requires 配置并进行类型校验。"""
        normalized: Dict[str, List[str]] = {}
        for key, value in requires.items():
            if value is None:
                normalized[key] = []
                continue

            if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
                raise ValueError(
                    f"SKILL.md 的 'metadata.agi_walker.requires.{key}' 必须是字符串列表"
                )
            normalized[key] = value

        return normalized

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
        return [
            skill for skill in self.skills.values()
            if skill.category == category
        ]
    
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
            skill for skill in self.skills.values()
            if query in skill.name.lower() or query in skill.description.lower()
        ]
    
    def get_categories(self) -> List[str]:
        """获取所有分类（兼容旧接口，结果按名称排序）。"""
        return self.get_all_categories()
    
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
        
        检查所需的 Python 模块和二进制工具是否可用。
        
        Args:
            skill_name: Skill 名称
            
        Returns:
            Dict包含缺失的依赖项:
            {
                "python_modules": ["missing_module1", ...],
                "bins": ["missing_bin1", ...]
            }
        """
        skill = self.skills.get(skill_name)
        if not skill:
            return {"error": [f"Skill '{skill_name}' 不存在"]}
        
        missing = {"python_modules": [], "bins": []}
        
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
    if _skills_loader_instance is None:
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
