#!/usr/bin/env python
"""
批量转换工具

将多个AGI-Walker配置批量转换为URDF/SDF格式。
"""

import argparse
import logging
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def _add_repo_root_to_path() -> None:
    module_path = Path("agi_walker/skills/urdf_generator/__init__.py")

    for parent in Path(__file__).resolve().parents:
        if (parent / module_path).exists():
            sys.path.insert(0, str(parent))
            return


_add_repo_root_to_path()

from agi_walker.skills.urdf_generator import (
    convert_to_urdf,
    validate_urdf,
)  # noqa: E402

logger = logging.getLogger(__name__)


def main() -> None:
    parser = argparse.ArgumentParser(description="批量转换机器人配置为URDF/SDF")
    parser.add_argument("--input", required=True, help="输入目录或文件")
    parser.add_argument("--output", required=True, help="输出目录")
    parser.add_argument("--format", default="urdf", help="输出格式 (urdf/sdf)")
    parser.add_argument("--meshes", action="store_true", help="生成碰撞网格")
    parser.add_argument("--validate", action="store_true", help="验证生成的URDF")

    args = parser.parse_args()

    input_path = Path(args.input)
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 收集输入文件
    if input_path.is_file():
        input_files = [input_path]
    else:
        input_files = list(input_path.glob("*.json"))

    logger.info("=" * 60)
    logger.info(f"批量转换: {len(input_files)} 个配置文件")
    logger.info("=" * 60)
    logger.info(f"输入: {input_path}")
    logger.info(f"输出: {output_dir}")
    logger.info(f"格式: {args.format.upper()}")
    logger.info()

    success_count = 0

    for input_file in input_files:
        logger.info(f"处理: {input_file.name}")

        try:
            # 生成输出文件名
            output_file = output_dir / f"{input_file.stem}.{args.format}"

            # 转换
            if args.format == "urdf":
                convert_to_urdf(
                    str(input_file), str(output_file), generate_meshes=args.meshes
                )
            else:
                logger.warning("  警告: SDF转换尚未完全实现")
                convert_to_urdf(str(input_file), str(output_file))

            # 验证
            if args.validate and args.format == "urdf":
                if validate_urdf(str(output_file)):
                    success_count += 1
                else:
                    logger.error("  警告: 验证失败")
            else:
                success_count += 1

        except Exception as e:
            logger.info(f"  错误: {e}")

        logger.info()

    logger.info("=" * 60)
    logger.info(f"转换完成: {success_count}/{len(input_files)} 成功")
    logger.info("=" * 60)


if __name__ == "__main__":
    main()
