#!/usr/bin/env python
"""
AGI-Walker CLI - Skills管理命令行工具

提供完整的Skills系统命令行接口。
"""

import argparse
import logging
import sys
from pathlib import Path

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent))

from agi_walker.skills_loader import (  # noqa: E402
    get_skills_loader,
    search_skills,
    get_skill_doc,
)
from agi_walker.workflow_orchestrator import get_workflow_orchestrator  # noqa: E402

logger = logging.getLogger(__name__)


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
            print(f"{skill.name}")
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
            print(f"[{category}]")
            for skill in cat_skills:
                print(f"  {skill.name}")
                print(f"    {skill.description[:80]}...")
            print("")  # Empty line separator


def cmd_info(args):
    """显示skill详细信息"""
    loader = get_skills_loader()

    try:
        skill = loader.get_skill(args.name)
    except KeyError:
        skill = None
    if skill is None:
        print(f"错误: Skill '{args.name}' 不存在")
        print("\n使用 'agi_walker skills list' 查看所有可用skills")
        return 1

    print(f"\n{skill.name}")
    print("=" * 60)
    print(f"名称: {skill.name}")
    print(f"分类: {skill.category}")
    print(f"描述: {skill.description}")
    print(f"路径: {skill.skill_dir}")

    if skill.requires:
        print("\n依赖:")
        for dep_type, deps in skill.requires.items():
            print(f"  {dep_type}: {', '.join(deps)}")

    # 显示文档
    if args.doc:
        print("\n完整文档:")
        print("-" * 60)
        doc = get_skill_doc(args.name)
        print(doc)

    # 显示可用脚本
    scripts_dir = skill.skill_dir / "scripts"
    if scripts_dir.exists():
        scripts = list(scripts_dir.glob("*.py"))
        if scripts:
            print("\n可用脚本:")
            for script in scripts:
                print(f"  - {script.name}")

    # 显示参考文档
    refs_dir = skill.skill_dir / "references"
    if refs_dir.exists():
        refs = list(refs_dir.glob("*.md"))
        if refs:
            print("\n参考文档:")
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
        print(f"{skill.name}")
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
        has_missing = any(missing_deps.get(dep_type) for dep_type in missing_deps)

        if has_missing:
            all_valid = False
            print(f"[FAIL] {skill.name}:")
            for dep_type, deps in missing_deps.items():
                if deps:
                    print(f"    缺失 {dep_type}: {', '.join(deps)}")
        else:
            if args.verbose:
                print(f"[OK] {skill.name}")

    print()
    if all_valid:
        print("[OK] 所有skills配置有效")
        return 0
    else:
        print("[FAIL] 发现配置问题")
        return 1


def cmd_workflows_list(args):
    """列出所有工作流"""
    orchestrator = get_workflow_orchestrator()
    workflows = orchestrator.list_workflows()

    if not workflows:
        print("未找到任何工作流")
        return

    print(f"\n可用工作流 ({len(workflows)} 个):\n")

    for name in workflows:
        workflow = orchestrator.get_workflow(name)
        print(f"{name}")
        print(f"  {workflow.get('description', '无描述')}")
        steps = workflow.get("steps", [])
        print(f"  步骤数: {len(steps)}")
        print()


def cmd_workflows_run(args):
    """执行工作流"""
    orchestrator = get_workflow_orchestrator()

    # 解析参数
    parameters = {}
    if args.params:
        for param in args.params:
            if "=" not in param:
                print(f"错误: 参数格式无效 '{param}'，应为 key=value")
                return 1
            key, value = param.split("=", 1)
            # 简单类型转换
            if value.lower() in ("true", "false"):
                value = value.lower() == "true"
            elif value.isdigit():
                value = int(value)
            elif value.replace(".", "").isdigit():
                value = float(value)
            parameters[key] = value

    if getattr(args, "force", False):
        parameters["execution_strategy"] = "force"
    elif getattr(args, "resume", False):
        parameters["execution_strategy"] = "resume"

    if getattr(args, "output_root", None):
        parameters["output_root"] = args.output_root

    # 确定使用mock还是real executors
    use_mock = getattr(args, "mock", False)
    use_real = not use_mock  # 反向逻辑：如果--mock标志被设置，use_real=False
    execution_strategy = parameters.get("execution_strategy", "resume")

    print(f"执行工作流: {args.name}")
    print(f"执行器模式: {'mock' if use_mock else 'real'}")
    print(f"执行策略: {execution_strategy}")
    if "output_root" in parameters:
        print(f"输出根目录: {parameters['output_root']}")
    if parameters:
        print(f"参数: {parameters}")
    print()

    result = orchestrator.execute_workflow(args.name, parameters, use_real=use_real)

    # 显示结果
    print(f"\n执行结果: {result.status.value}")
    if result.duration:
        print(f"总耗时: {result.duration:.2f}s")
    print(f"成功率: {result.success_rate:.1f}%")
    if result.steps:
        print(
            f"步骤统计: 完成 {result.completed_steps}, "
            f"跳过 {result.skipped_steps}, 失败 {result.failed_steps}"
        )
    if result.error_message:
        print(f"错误: {result.error_message}")

    # 显示步骤详情
    if result.steps:
        print(f"\n步骤执行详情 ({len(result.steps)} 步):\n")
        for i, step in enumerate(result.steps, 1):
            status_label = {
                "completed": "[OK]",
                "failed": "[FAIL]",
                "running": "[RUNNING]",
                "pending": "[PENDING]",
                "skipped": "[SKIPPED]",
            }.get(step.status.value, "[?]")

            print(f"  {i}. {status_label} {step.name}")
            print(f"     Skill: {step.skill_executor}.{step.action}")
            if step.duration:
                print(f"     耗时: {step.duration:.2f}s")
            if step.error:
                print(f"     错误: {step.error}")
            if step.output:
                print(f"     输出键: {', '.join(step.output.keys())}")

    # 显示输出
    final_outputs = {step.name: step.output for step in result.steps if step.output}
    if final_outputs:
        print("\n最终输出:")
        for key, value in final_outputs.items():
            if isinstance(value, (dict, list)):
                print(f"  {key}: {type(value).__name__} ({len(value)} 项)")
            else:
                print(f"  {key}: {value}")

    return 0 if result.status.value == "completed" else 1


def cmd_workflows_validate(args):
    """验证工作流定义"""
    orchestrator = get_workflow_orchestrator()

    workflow = orchestrator.get_workflow(args.name)
    if not workflow:
        print(f"错误: 工作流 '{args.name}' 不存在")
        return 1

    is_valid, message = orchestrator.validate_workflow(args.name)

    if is_valid:
        print(f"[OK] 工作流 '{args.name}' 验证通过")
        return 0
    else:
        print(f"[FAIL] 工作流 '{args.name}' 验证失败:")
        print(f"  - {message}")
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

  agi_walker skills workflows list          列出所有工作流
  agi_walker skills workflows run robot_creation_pipeline  执行完整流水线
  agi_walker skills workflows run robot_creation_pipeline --force --output-root test_env/run1
  agi_walker skills workflows validate robot_creation_pipeline  验证工作流定义
        """,
    )

    subparsers = parser.add_subparsers(dest="command", help="可用命令")

    # list 命令
    list_parser = subparsers.add_parser("list", help="列出所有skills")
    list_parser.add_argument(
        "-v", "--verbose", action="store_true", help="显示详细信息"
    )
    list_parser.add_argument("--category", help="按分类过滤")

    # info 命令
    info_parser = subparsers.add_parser("info", help="显示skill详细信息")
    info_parser.add_argument("name", help="skill名称")
    info_parser.add_argument("-d", "--doc", action="store_true", help="显示完整文档")

    # search 命令
    search_parser = subparsers.add_parser("search", help="搜索skills")
    search_parser.add_argument("query", help="搜索关键词")

    # categories 命令
    subparsers.add_parser("categories", help="列出所有分类")

    # validate 命令
    val_parser = subparsers.add_parser("validate", help="验证skills配置")
    val_parser.add_argument(
        "-v", "--verbose", action="store_true", help="显示所有验证结果"
    )

    # workflows 子命令组
    workflows_parser = subparsers.add_parser("workflows", help="工作流管理")
    workflows_subparsers = workflows_parser.add_subparsers(
        dest="workflows_command", help="工作流子命令"
    )

    # workflows list
    workflows_subparsers.add_parser("list", help="列出所有工作流")

    # workflows run
    run_parser = workflows_subparsers.add_parser("run", help="执行工作流")
    run_parser.add_argument("name", help="工作流名称")
    run_parser.add_argument("--params", nargs="*", help="工作流参数 (key=value)")
    run_parser.add_argument(
        "--mock", action="store_true", help="使用mock executors而不是真实的skill调用"
    )
    strategy_group = run_parser.add_mutually_exclusive_group()
    strategy_group.add_argument(
        "--force", action="store_true", help="强制重跑所有步骤，即使产物已存在"
    )
    strategy_group.add_argument(
        "--resume", action="store_true", help="遇到已存在的产物时跳过对应步骤"
    )
    run_parser.add_argument("--output-root", help="将相对输出路径重定向到指定目录下")

    # workflows validate
    val_wf_parser = workflows_subparsers.add_parser("validate", help="验证工作流定义")
    val_wf_parser.add_argument("name", help="工作流名称")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return 0

    # 执行命令
    commands = {
        "list": cmd_list,
        "info": cmd_info,
        "search": cmd_search,
        "categories": cmd_categories,
        "validate": cmd_validate,
    }

    # 处理 workflows 子命令
    if args.command == "workflows":
        if not args.workflows_command:
            workflows_parser.print_help()
            return 0

        workflow_commands = {
            "list": cmd_workflows_list,
            "run": cmd_workflows_run,
            "validate": cmd_workflows_validate,
        }

        return workflow_commands[args.workflows_command](args) or 0

    return commands[args.command](args) or 0


if __name__ == "__main__":
    sys.exit(main())
