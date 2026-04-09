from __future__ import annotations

import argparse
import logging
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


logger = logging.getLogger(__name__)
PROJECT_ROOT = Path(__file__).resolve().parent.parent

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")


@dataclass(frozen=True)
class ScriptCheck:
    check_id: str
    name: str
    path: str
    desc: str
    args: tuple[str, ...] = ()
    optional: bool = False


CHECKS: tuple[ScriptCheck, ...] = (
    ScriptCheck(
        check_id="pipeline",
        name="Web generation pipeline",
        path="tests/verify_pipeline.py",
        desc="验证 /api/generate_robot 输出的 JSON 元数据和 URDF 结构",
    ),
    ScriptCheck(
        check_id="parts_store",
        name="Parts store API",
        path="tests/verify_parts_store.py",
        desc="验证零件市场查询、导入和重复导入处理",
    ),
    ScriptCheck(
        check_id="web_integration",
        name="Legacy web-godot integration",
        path="tests/verify_web_integration.py",
        desc="验证 parse-command 与 legacy Godot 路由链路",
    ),
    ScriptCheck(
        check_id="godot_api",
        name="Godot framed TCP API",
        path="tests/verify_godot_api.py",
        desc="验证长度前缀 Godot 协议与 MockGodotServer",
    ),
    ScriptCheck(
        check_id="dreamer_interface",
        name="Dreamer interface",
        path="tests/verify_dreamer_interface.py",
        desc="验证 DreamerEnv 的 reset/step framed TCP 契约",
    ),
    ScriptCheck(
        check_id="sim2real_score",
        name="Sim2Real score",
        path="tests/verify_sim2real_score.py",
        desc="验证 Sim2RealScorer 的排序和报告字段",
    ),
    ScriptCheck(
        check_id="mocked_evolution",
        name="Mocked evolution loop",
        path="tests/verify_mocked.py",
        desc="验证隔离工作目录下的进化管理器脚本链路",
    ),
    ScriptCheck(
        check_id="smoke",
        name="Smoke runner",
        path="tests/run_smoke_tests.py",
        desc="验证 CLI、workflow 和 Web panel 的高信号烟测",
        args=(
            "--output-root",
            "test_env/run_all_tests_smoke",
        ),
        optional=True,
    ),
    ScriptCheck(
        check_id="rl_minimal",
        name="Minimal RL",
        path="tests/verify_rl_minimal.py",
        desc="验证最小 PPO 训练回路",
        optional=True,
    ),
    ScriptCheck(
        check_id="d3rlpy",
        name="d3rlpy import",
        path="tests/verify_d3rlpy.py",
        desc="验证 d3rlpy 依赖是否可导入",
        optional=True,
    ),
)


def _configure_logging() -> None:
    logging.basicConfig(level=logging.INFO, format="%(message)s")


def _build_env() -> dict[str, str]:
    env = os.environ.copy()
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUTF8"] = "1"
    existing_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = (
        str(PROJECT_ROOT)
        if not existing_pythonpath
        else f"{PROJECT_ROOT}{os.pathsep}{existing_pythonpath}"
    )
    return env


def _resolve_checks(
    selected_ids: Sequence[str] | None,
    include_optional: bool,
) -> list[ScriptCheck]:
    check_map = {check.check_id: check for check in CHECKS}

    if selected_ids:
        unknown = sorted(set(selected_ids) - set(check_map))
        if unknown:
            raise ValueError(f"Unknown checks: {', '.join(unknown)}")
        return [check_map[check_id] for check_id in selected_ids]

    return [
        check for check in CHECKS if include_optional or not check.optional
    ]


def _summarize_output(stdout: str, stderr: str) -> str:
    combined_lines = [
        line
        for line in "\n".join(part for part in [stdout.strip(), stderr.strip()] if part).splitlines()
        if line.strip()
    ]
    if not combined_lines:
        return "(no output)"
    if len(combined_lines) <= 20:
        return "\n".join(combined_lines)
    kept_head = combined_lines[:12]
    kept_tail = combined_lines[-4:]
    omitted = len(combined_lines) - len(kept_head) - len(kept_tail)
    return "\n".join(
        kept_head
        + [f"... (skipped {omitted} lines) ..."]
        + kept_tail
    )


def _run_check(check: ScriptCheck, env: dict[str, str]) -> tuple[bool, float, str]:
    script_path = PROJECT_ROOT / check.path
    if not script_path.exists():
        return False, 0.0, f"Missing script: {script_path}"

    start_time = time.time()
    result = subprocess.run(
        [sys.executable, str(script_path), *check.args],
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=env,
    )
    duration = time.time() - start_time

    output = _summarize_output(result.stdout, result.stderr)
    if result.returncode != 0:
        output = f"exit code: {result.returncode}\n{output}"

    return result.returncode == 0, duration, output


def run_all_tests(
    selected_ids: Sequence[str] | None = None,
    include_optional: bool = False,
    fail_fast: bool = False,
) -> bool:
    env = _build_env()
    checks = _resolve_checks(selected_ids, include_optional)

    logger.info("AGI-Walker script verification bundle")
    logger.info("project root: %s", PROJECT_ROOT)
    logger.info("selected checks: %s", ", ".join(check.check_id for check in checks))
    logger.info("")

    failures: list[str] = []
    for index, check in enumerate(checks, start=1):
        logger.info("[%s/%s] %s", index, len(checks), check.name)
        logger.info("id: %s", check.check_id)
        logger.info("desc: %s", check.desc)
        ok, duration, output = _run_check(check, env)
        logger.info("duration: %.2fs", duration)
        logger.info(output)
        logger.info("status: %s", "PASS" if ok else "FAIL")
        logger.info("")

        if not ok:
            failures.append(check.check_id)
            if fail_fast:
                break

    if failures:
        logger.info("summary: FAIL")
        logger.info("failed checks: %s", ", ".join(failures))
        return False

    logger.info("summary: PASS")
    return True


def main(argv: Sequence[str] | None = None) -> int:
    _configure_logging()

    parser = argparse.ArgumentParser(
        description="Run the current AGI-Walker script verification bundle."
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List available check ids and exit.",
    )
    parser.add_argument(
        "--checks",
        nargs="+",
        help="Run only the specified check ids.",
    )
    parser.add_argument(
        "--include-optional",
        action="store_true",
        help="Include optional slow or dependency-sensitive checks.",
    )
    parser.add_argument(
        "--fail-fast",
        action="store_true",
        help="Stop at the first failing check.",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    if args.list:
        for check in CHECKS:
            tag = "optional" if check.optional else "core"
            logger.info("%s [%s] - %s", check.check_id, tag, check.desc)
        return 0

    try:
        ok = run_all_tests(
            selected_ids=args.checks,
            include_optional=args.include_optional,
            fail_fast=args.fail_fast,
        )
    except ValueError as exc:
        logger.error(str(exc))
        return 2

    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
