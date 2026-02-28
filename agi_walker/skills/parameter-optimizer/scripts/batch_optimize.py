#!/usr/bin/env python
"""
批量优化工具

支持同时优化质量分布和PID增益。
"""

import argparse
import json
from pathlib import Path
import sys

# 添加项目根目录到路径
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from agi_walker.skills.parameter_optimizer import (
    optimize_mass_distribution,
    batch_optimize_pid,
)


def main():
    parser = argparse.ArgumentParser(description="批量优化机器人参数")
    parser.add_argument("--config", required=True, help="机器人配置文件路径")
    parser.add_argument(
        "--optimize", required=True, help="优化项目,逗号分隔 (mass,pid,damping)"
    )
    parser.add_argument("--output", required=True, help="输出文件路径")
    parser.add_argument("--com-height", type=float, default=0.25, help="目标重心高度")
    parser.add_argument("--iterations", type=int, default=100, help="优化迭代次数")
    parser.add_argument("--method", default="gradient", help="优化方法")

    args = parser.parse_args()

    # 加载配置
    with open(args.config, "r", encoding="utf-8") as f:
        config = json.load(f)

    optimize_items = args.optimize.split(",")
    results = {"original_config": args.config, "optimizations": {}}

    print("=" * 60)
    print("批量参数优化")
    print("=" * 60)
    print(f"输入配置: {args.config}")
    print(f"优化项目: {', '.join(optimize_items)}")
    print()

    # 质量分布优化
    if "mass" in optimize_items:
        print("--- 质量分布优化 ---")
        mass_result = optimize_mass_distribution(
            config,
            target_com_height=args.com_height,
            max_iterations=args.iterations,
            method=args.method,
        )

        if mass_result.success:
            print("✓ 优化成功")
            print(f"  COM 误差: {mass_result.com_error:.6f} m")
            print(f"  迭代次数: {mass_result.iterations}")

            # 更新配置
            for part in config["parts"]:
                part_id = part["id"]
                if part_id in mass_result.mass_distribution:
                    part["params"]["mass"] = mass_result.mass_distribution[part_id]

            results["optimizations"]["mass"] = {
                "com_error": mass_result.com_error,
                "iterations": mass_result.iterations,
            }
        else:
            print("✗ 优化失败")
        print()

    # PID调优
    if "pid" in optimize_items:
        print("--- PID 增益调优 ---")

        # 提取所有关节
        joints = []
        for conn in config.get("connections", []):
            joint_type = conn.get("joint_type")
            if joint_type and joint_type != "fixed":
                joint_id = f"{conn['from']}_to_{conn['to']}"
                joints.append(joint_id)

        if joints:
            pid_results = batch_optimize_pid(config, joints, method="ziegler_nichols")

            # 存储结果
            if "pid_gains" not in config["metadata"]:
                config["metadata"]["pid_gains"] = {}

            for joint, gains in pid_results.items():
                config["metadata"]["pid_gains"][joint] = {
                    "kp": gains.kp,
                    "ki": gains.ki,
                    "kd": gains.kd,
                }

            results["optimizations"]["pid"] = {"joints_optimized": len(pid_results)}
            print(f"\n✓ 优化了 {len(pid_results)} 个关节")
        else:
            print("未找到可调优关节")
        print()

    # 保存结果
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

    print("=" * 60)
    print(f"✓ 优化完成,结果已保存到: {args.output}")
    print("=" * 60)

    # 保存优化报告
    report_path = output_path.parent / f"{output_path.stem}_report.json"
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2, ensure_ascii=False)

    print(f"优化报告: {report_path}")


if __name__ == "__main__":
    main()
