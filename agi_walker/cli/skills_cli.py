#!/usr/bin/env python
"""
AGI-Walker CLI - Skills管理命令行工具

提供完整的Skills系统命令行接口。
"""

import sys
import argparse
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from agi_walker.skills_loader import (
    get_skills_loader,
    list_skills,
    search_skills,
    get_skill_doc
)


def cmd_list(args):
    """列出所有skills"""
    loader = get_skills_loader()
    skills = loader.get_skills_list()
    
    if args.category:
        skills = [s for s in skills if s.category == args.category]
    
    if not skills:
        print("未找到任何skills")
        return
    
    print(f"\n可用 Skills ({len(skills)} 个):\n")
    
    if args.verbose:
        for skill in skills:
            print(f"{skill.display_name}")
            print(f"  名称: {skill.name}")
            print(f"  分类: {skill.category}")
            print(f"  描述: {skill.description[:100]}...")
            if skill.requires:
                print(f"  依赖: {skill.requires}")
            print(f"  路径: {skill.skill_dir}")
            print()
    else:
        # 按分类分组
        by_category = {}
        for skill in skills:
            cat = skill.category
            if cat not in by_category:
                by_category[cat] = []
            by_category[cat].append(skill)
        
        for category, cat_skills in sorted(by_category.items()):
            print(f"【{category}】")
            for skill in cat_skills:
                print(f"  {skill.display_name}")
                print(f"    {skill.description[:80]}...")
            print()


def cmd_info(args):
    """显示skill详细信息"""
    loader = get_skills_loader()
    
    try:
        skill = loader.get_skill(args.name)
    except KeyError:
        print(f"错误: Skill '{args.name}' 不存在")
        print("\n使用 'agi_walker skills list' 查看所有可用skills")
        return 1
    
    print(f"\n{skill.display_name}")
    print("=" * 60)
    print(f"名称: {skill.name}")
    print(f"分类: {skill.category}")
    print(f"描述: {skill.description}")
    print(f"路径: {skill.skill_dir}")
    
    if skill.requires:
        print(f"\n依赖:")
        for dep_type, deps in skill.requires.items():
            print(f"  {dep_type}: {', '.join(deps)}")
    
    # 显示文档
    if args.doc:
        print(f"\n完整文档:")
        print("-" * 60)
        doc = get_skill_doc(args.name)
        print(doc)
    
    # 显示可用脚本
    scripts_dir = skill.skill_dir / "scripts"
    if scripts_dir.exists():
        scripts = list(scripts_dir.glob("*.py"))
        if scripts:
            print(f"\n可用脚本:")
            for script in scripts:
                print(f"  - {script.name}")
    
    # 显示参考文档
    refs_dir = skill.skill_dir / "references"
    if refs_dir.exists():
        refs = list(refs_dir.glob("*.md"))
        if refs:
            print(f"\n参考文档:")
            for ref in refs:
                print(f"  - {ref.name}")
    
    print()


def cmd_search(args):
    """搜索skills"""
    results = search_skills(args.query)
    
    if not results:
        print(f"未找到匹配 '{args.query}' 的skills")
        return
    
    print(f"\n搜索结果 ({len(results)} 个):\n")
    
    for skill in results:
        print(f"{skill.display_name}")
        print(f"  {skill.description[:100]}...")
        print()


def cmd_categories(args):
    """列出所有分类"""
    loader = get_skills_loader()
    categories = loader.get_categories()
    
    print(f"\nSkill 分类 ({len(categories)} 个):\n")
    
    for cat in sorted(categories):
        skills = loader.get_skill_by_category(cat)
        print(f"  {cat} ({len(skills)} 个skills)")


def cmd_validate(args):
    """验证skills配置"""
    loader = get_skills_loader()
    
    print("验证 Skills 配置...\n")
    
    all_valid = True
    for skill in loader.get_skills_list():
        # 检查依赖
        missing_deps = loader.validate_skill_dependencies(skill.name)
        
        if missing_deps:
            all_valid = False
            print(f"✗ {skill.name}:")
            for dep_type, deps in missing_deps.items():
                print(f"    缺失 {dep_type}: {', '.join(deps)}")
        else:
            if args.verbose:
                print(f"✓ {skill.name}")
    
    print()
    if all_valid:
        print("✓ 所有skills配置有效")
        return 0
    else:
        print("✗ 发现配置问题")
        return 1


def main():
    parser = argparse.ArgumentParser(
        description="AGI-Walker Skills 管理工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  agi_walker skills list                    列出所有skills
  agi_walker skills list -v                 详细列表
  agi_walker skills list --category 建模    按分类过滤
  agi_walker skills info robot-modeling     查看详情
  agi_walker skills info robot-modeling -d  查看完整文档
  agi_walker skills search 优化             搜索skills
  agi_walker skills validate                验证配置
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='可用命令')
    
    # list 命令
    list_parser = subparsers.add_parser('list', help='列出所有skills')
    list_parser.add_argument('-v', '--verbose', action='store_true', help='显示详细信息')
    list_parser.add_argument('--category', help='按分类过滤')
    
    # info 命令
    info_parser = subparsers.add_parser('info', help='显示skill详细信息')
    info_parser.add_argument('name', help='skill名称')
    info_parser.add_argument('-d', '--doc', action='store_true', help='显示完整文档')
    
    # search 命令
    search_parser = subparsers.add_parser('search', help='搜索skills')
    search_parser.add_argument('query', help='搜索关键词')
    
    # categories 命令
    cat_parser = subparsers.add_parser('categories', help='列出所有分类')
    
    # validate 命令
    val_parser = subparsers.add_parser('validate', help='验证skills配置')
    val_parser.add_argument('-v', '--verbose', action='store_true', help='显示所有验证结果')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return 0
    
    # 执行命令
    commands = {
        'list': cmd_list,
        'info': cmd_info,
        'search': cmd_search,
        'categories': cmd_categories,
        'validate': cmd_validate
    }
    
    return commands[args.command](args) or 0


if __name__ == '__main__':
    sys.exit(main())
