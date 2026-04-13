#!/usr/bin/env python
"""Build a non-destructive cleanup report for the current Git worktree."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from collections import Counter
from datetime import datetime, timezone
from fnmatch import fnmatch
from pathlib import Path
from typing import Any


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


_configure_stdio()
PROJECT_ROOT = _find_repo_root()

RUNTIME_PATTERNS = (
    ".agi_data/",
    "test_env/",
    ".pytest_cache/",
    ".ruff_cache/",
    "htmlcov/",
    "__pycache__/",
    "archive/",
    ".coverage",
    "coverage.xml",
    "agi_walker.db",
    "*.db",
)
GENERATED_PATTERNS = (
    "configs/generated/",
    "knowledge/test_index/",
    "godot_project/.godot_agent_index.json",
    "*.generated.json",
)
SOURCE_PATTERNS = (
    ".github/",
    "agi_walker/",
    "alembic/",
    "blender_scripts/",
    "deployment/",
    "docs/",
    "examples/",
    "godot_studio_agent/",
    "hardware/",
    "openneuro/",
    "parts_library/",
    "robot_models/",
    "scripts/",
    "tests/",
    "tools/",
    "web_panel/",
    "weights/",
    ".gitignore",
    "README.md",
    "RELEASE_NOTES.md",
    "pyproject.toml",
    "pytest.ini",
)

CATEGORY_METADATA = {
    "runtime_artifact": {
        "title": "运行时产物",
        "recommendation": "优先人工确认后清理本地产物；如果这些文件不该入库，再补 ignore 规则。",
    },
    "generated_artifact": {
        "title": "生成物候选",
        "recommendation": "确认是否需要保留为源码；如果只是生成结果，优先改成可重建并忽略。",
    },
    "manual_review": {
        "title": "源码/文档人工审查",
        "recommendation": "这些通常是真实改动，不要自动删除；需要按提交意图拆分或保留。",
    },
    "unknown": {
        "title": "未知分类",
        "recommendation": "路径未命中当前分类规则，需要人工判断后再处理。",
    },
}


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _run_git_status(source_root: Path) -> list[str]:
    result = subprocess.run(
        ["git", "status", "--short", "--untracked-files=all"],
        cwd=str(source_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or "git status --short failed"
        raise RuntimeError(message)
    return [line.rstrip() for line in result.stdout.splitlines() if line.strip()]


def _extract_path(raw_line: str) -> str:
    candidate = raw_line[3:].strip()
    if " -> " in candidate:
        candidate = candidate.split(" -> ", 1)[1].strip()
    return candidate.replace("\\", "/")


def _status_label(code: str) -> str:
    if code == "??":
        return "untracked"
    if code[0] != " " and code[1] != " ":
        return "staged_and_unstaged"
    if code[0] != " ":
        return "staged"
    return "modified"


def _matches_any(path: str, patterns: tuple[str, ...]) -> bool:
    wrapped_path = f"/{path}"
    for pattern in patterns:
        if pattern.endswith("/"):
            if path.startswith(pattern) or f"/{pattern}" in wrapped_path:
                return True
            continue
        if fnmatch(path, pattern):
            return True
    return False


def _categorize_path(path: str) -> str:
    if _matches_any(path, RUNTIME_PATTERNS):
        return "runtime_artifact"
    if _matches_any(path, GENERATED_PATTERNS):
        return "generated_artifact"
    if _matches_any(path, SOURCE_PATTERNS):
        return "manual_review"
    return "unknown"


def _build_path_entries(lines: list[str]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for raw_line in lines:
        status_code = raw_line[:2]
        path = _extract_path(raw_line)
        category = _categorize_path(path)
        entries.append(
            {
                "path": path,
                "status_code": status_code,
                "status_label": _status_label(status_code),
                "tracked": status_code != "??",
                "category": category,
                "recommendation": CATEGORY_METADATA[category]["recommendation"],
            }
        )
    return entries


def _build_category_summary(entries: list[dict[str, Any]]) -> list[dict[str, Any]]:
    grouped: dict[str, list[dict[str, Any]]] = {
        category: [item for item in entries if item["category"] == category]
        for category in CATEGORY_METADATA
    }
    summaries: list[dict[str, Any]] = []
    for category, items in grouped.items():
        if not items:
            continue
        tracked_count = sum(1 for item in items if item["tracked"])
        untracked_count = len(items) - tracked_count
        summaries.append(
            {
                "id": category,
                "title": CATEGORY_METADATA[category]["title"],
                "count": len(items),
                "tracked_count": tracked_count,
                "untracked_count": untracked_count,
                "sample_paths": [item["path"] for item in items[:5]],
                "recommendation": CATEGORY_METADATA[category]["recommendation"],
            }
        )
    return summaries


def _build_next_step_plan(entries: list[dict[str, Any]]) -> list[str]:
    counts = Counter(item["category"] for item in entries)
    if not entries:
        return ["工作区已 clean，可继续 stable promotion。"]

    plan: list[str] = []
    if counts["runtime_artifact"] > 0:
        plan.append(
            f"先审查并清理 {counts['runtime_artifact']} 个运行时产物，避免把本地状态带入 stable promotion。"
        )
    if counts["generated_artifact"] > 0:
        plan.append(
            f"再确认 {counts['generated_artifact']} 个生成物候选是否需要入库；如果不需要，应转成可重建并忽略。"
        )
    if counts["manual_review"] > 0:
        plan.append(
            f"最后人工拆分 {counts['manual_review']} 个源码/文档改动，决定哪些保留为真实提交。"
        )
    if counts["unknown"] > 0:
        plan.append(
            f"还需人工判断 {counts['unknown']} 个未知分类路径，再决定清理还是保留。"
        )
    return plan


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a non-destructive Git worktree cleanup report."
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Git repository root to inspect.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "worktree_cleanup"),
        help="Directory used to store the cleanup report.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional override path for the structured cleanup report.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    source_root = Path(args.source_root)
    report_path = (
        Path(args.report_file)
        if args.report_file
        else Path(args.output_root) / "worktree_cleanup_report.json"
    )
    output_root = Path(args.output_root)
    if args.report_file and output_root == PROJECT_ROOT / "test_env" / "worktree_cleanup":
        output_root = report_path.parent
    output_root.mkdir(parents=True, exist_ok=True)

    lines = _run_git_status(source_root)
    entries = _build_path_entries(lines)
    category_summary = _build_category_summary(entries)
    tracked_count = sum(1 for item in entries if item["tracked"])
    untracked_count = len(entries) - tracked_count
    payload = {
        "schema_version": "1.0",
        "artifact_type": "worktree_cleanup_report",
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "report_path": str(report_path),
        "clean_worktree": not entries,
        "total_paths": len(entries),
        "tracked_paths": tracked_count,
        "untracked_paths": untracked_count,
        "category_summary": category_summary,
        "paths": entries,
        "next_step_plan": _build_next_step_plan(entries),
    }
    written_report = _write_report(report_path, payload)

    print(f"worktree_cleanup_report_written={written_report}")
    print(f"worktree_cleanup_clean={str(not entries).lower()}")
    print(f"worktree_cleanup_total_paths={len(entries)}")
    print(f"worktree_cleanup_categories={len(category_summary)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
