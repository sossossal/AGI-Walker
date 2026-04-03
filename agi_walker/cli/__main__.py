#!/usr/bin/env python
"""
AGI-Walker 主CLI入口

统一的命令行接口。
"""

import argparse
import logging
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    parser = argparse.ArgumentParser(
        prog="agi_walker",
        description="AGI-Walker - 智能机器人建模工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
可用命令:
  skills    Skills系统管理
  doctor    运行环境自检
  
示例:
  agi_walker skills list
  agi_walker doctor
        """,
    )

    parser.add_argument("--version", action="version", version="AGI-Walker 0.1.0")

    subparsers = parser.add_subparsers(dest="module", help="功能模块")

    # skills 模块
    subparsers.add_parser("skills", help="Skills系统管理", add_help=False)
    # workflows alias to help users get there faster
    subparsers.add_parser(
        "workflows", help="Alias for `skills workflows`", add_help=False
    )
    # doctor 模块
    subparsers.add_parser("doctor", help="运行环境自检")

    args, remaining = parser.parse_known_args()

    if args.module == "doctor":
        from agi_walker.utils.doctor import run_diagnostics, print_report

        results = run_diagnostics()
        print_report(results)
        return 0

    if args.module == "skills":
        # 调用skills CLI
        from agi_walker.cli.skills_cli import main as skills_main

        sys.argv = ["agi_walker skills"] + remaining
        return skills_main()

    if args.module == "workflows":
        from agi_walker.cli.skills_cli import main as skills_main

        sys.argv = ["agi_walker skills", "workflows"] + remaining
        return skills_main()

    if not args.module:
        parser.print_help()
        return 0

    return 0


if __name__ == "__main__":
    sys.exit(main())
