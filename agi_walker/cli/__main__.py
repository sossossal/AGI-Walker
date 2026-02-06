#!/usr/bin/env python
"""
AGI-Walker 主CLI入口

统一的命令行接口。
"""

import sys
import argparse
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    parser = argparse.ArgumentParser(
        prog='agi_walker',
        description='AGI-Walker - 智能机器人建模工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
可用命令:
  skills    Skills系统管理
  
示例:
  agi_walker skills list
  agi_walker skills info robot-modeling
  agi_walker skills search 优化
        """
    )
    
    parser.add_argument('--version', action='version', version='AGI-Walker 0.1.0')
    
    subparsers = parser.add_subparsers(dest='module', help='功能模块')
    
    # skills 模块
    skills_parser = subparsers.add_parser(
        'skills',
        help='Skills系统管理',
        add_help=False
    )
    
    args, remaining = parser.parse_known_args()
    
    if args.module == 'skills':
        # 调用skills CLI
        from agi_walker.cli.skills_cli import main as skills_main
        sys.argv = ['agi_walker skills'] + remaining
        return skills_main()
    
    if not args.module:
        parser.print_help()
        return 0
    
    return 0


if __name__ == '__main__':
    sys.exit(main())
