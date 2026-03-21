#!/usr/bin/env python3
"""
AGI-Walker Skills 端到端验证与索引生成工具
用于 CI 质量门禁阶段，扫描所有 Skills 的一致性和资产完整性。
"""

import os
import sys
from pathlib import Path

# 把项目根目录加入到 sys.path
project_root = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(project_root))

from agi_walker.skills_loader import get_skills_loader, SkillMetadata


def generate_markdown_index(skills: list[SkillMetadata]) -> str:
    """生成最新的 Markdown 索引表"""
    lines = [
        "# 🤖 AGI-Walker Skills 技能索引手册",
        "\n> 本文档由 CI Pipeline 自动生成，保持与代码库同步。如需增加技能，请查阅开发指南。\n",
        "## 技能分类全览\n"
    ]
    
    # 按照类别聚类
    categories = {}
    for skill in skills:
        if skill.category not in categories:
            categories[skill.category] = []
        categories[skill.category].append(skill)
        
    for cat in sorted(categories.keys()):
        lines.append(f"### 🏷️ 类别: {cat}\n")
        lines.append("| 图标 | 名称 | 描述 | 核心依赖 |")
        lines.append("|--|--|--|--|")
        for sk in sorted(categories[cat], key=lambda x: x.name):
            reqs = []
            if sk.requires.get("python_modules"):
                reqs.append(f"📦 Py: {','.join(sk.requires['python_modules'])}")
            if sk.requires.get("bins"):
                reqs.append(f"⚙️ Bin: {','.join(sk.requires['bins'])}")
            if sk.requires.get("files"):
                reqs.append(f"📄 File: {len(sk.requires['files'])} 项")
            
            req_str = "<br>".join(reqs) if reqs else "无特殊依赖"
            lines.append(f"| {sk.emoji} | **`{sk.name}`** | {sk.description} | {req_str} |")
        lines.append("")
        
    return "\n".join(lines)


def main():
    if sys.stdout.encoding.lower() != 'utf-8':
        try:
            sys.stdout.reconfigure(encoding='utf-8')
        except AttributeError:
            pass
            
    print("===============================================")
    print("🚀 AGI-Walker Skills 端到端质量验证工具")
    print("===============================================")
    
    loader = get_skills_loader()
    skills = loader.get_skills_list()
    
    if not skills:
        print("❌ 未扫描到任何 Skills，流程中止！")
        sys.exit(1)
        
    print(f"✅ 共扫描到 {len(skills)} 个 Skills，正在逐一校验...")
    
    has_error = False
    
    for skill in skills:
        print(f"\n🔍 校验 Skill: {skill.display_name}")
        missing = loader.validate_skill_dependencies(skill.name)
        
        if "error" in missing:
            print(f"  ❌ 严重内部错误: {missing['error'][0]}")
            has_error = True
            continue
            
        is_clean = True
        for dep_type, misses in missing.items():
            if misses:
                is_clean = False
                print(f"  ❌ 缺失 {dep_type}: {', '.join(misses)}")
                has_error = True
                
        if is_clean:
            print("  🟢 依赖全部就绪")
            
    print("\n===============================================")
    if has_error:
        print("💥 验证失败！有部分 Skill 标称的依赖或物理文件不存在。由于这是门禁环节，请修复依赖项或下架该 Skill。")
        sys.exit(1)

    # 验证全部通过后，向物理文件吐出最新的索引手册
    docs_dir = project_root / "docs"
    docs_dir.mkdir(exist_ok=True)
    index_file = docs_dir / "SKILLS_INDEX.md"
    
    markdown_content = generate_markdown_index(skills)
    index_file.write_text(markdown_content, encoding="utf-8")
    print(f"📝 自动索引生成成功: {index_file.relative_to(project_root)}")
    print("✨ 所有步骤顺利完结！")


if __name__ == "__main__":
    main()
