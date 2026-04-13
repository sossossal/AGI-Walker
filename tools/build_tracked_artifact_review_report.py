#!/usr/bin/env python
"""Build a focused review report for tracked runtime/generated artifacts."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
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


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _run_git_capture(command: list[str], *, cwd: Path) -> str:
    result = subprocess.run(
        command,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip() or f"{' '.join(command)} failed"
        raise RuntimeError(message)
    return result.stdout


def _load_cleanup_report(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _resolve_cleanup_report_path(args: argparse.Namespace) -> Path:
    if args.cleanup_report:
        return Path(args.cleanup_report)
    return Path(args.output_root) / "worktree_cleanup_report.json"


def _tracked_artifact_candidates(cleanup_report: dict[str, Any]) -> list[dict[str, Any]]:
    return [
        item
        for item in cleanup_report.get("paths", [])
        if item.get("tracked") is True
        and item.get("category") in {"runtime_artifact", "generated_artifact"}
    ]


def _parse_numstat(output: str) -> tuple[int, int]:
    line = output.strip().splitlines()
    if not line:
        return 0, 0
    fields = line[-1].split("\t")
    if len(fields) < 2:
        return 0, 0
    added = 0 if fields[0] == "-" else int(fields[0])
    removed = 0 if fields[1] == "-" else int(fields[1])
    return added, removed


def _diff_preview(source_root: Path, path: str) -> list[str]:
    output = _run_git_capture(
        ["git", "diff", "--unified=0", "--", path],
        cwd=source_root,
    )
    preview: list[str] = []
    for line in output.splitlines():
        if line.startswith(("+++", "---", "@@")):
            preview.append(line)
            continue
        if line.startswith(("+", "-")) and not line.startswith(("+++", "---")):
            preview.append(line[:200])
        if len(preview) >= 12:
            break
    return preview


def _review_recommendation(path: str, category: str) -> dict[str, Any]:
    if category == "runtime_artifact":
        return {
            "action": "review_then_revert_or_rebaseline",
            "summary": "这是 tracked 的运行时产物，通常不应直接参与 stable promotion。",
            "suggested_command": f"git diff -- {path}",
        }
    return {
        "action": "review_tracking_policy",
        "summary": "这是 tracked 的生成索引，需明确决定继续跟踪还是转为可重建产物。",
        "suggested_command": f"git diff -- {path}",
    }


def _build_entries(source_root: Path, candidates: list[dict[str, Any]]) -> list[dict[str, Any]]:
    entries: list[dict[str, Any]] = []
    for item in candidates:
        path = str(item["path"])
        numstat = _run_git_capture(["git", "diff", "--numstat", "--", path], cwd=source_root)
        added, removed = _parse_numstat(numstat)
        recommendation = _review_recommendation(path, str(item["category"]))
        entries.append(
            {
                "path": path,
                "category": item["category"],
                "status_label": item.get("status_label"),
                "diff_lines_added": added,
                "diff_lines_removed": removed,
                "diff_preview": _diff_preview(source_root, path),
                "recommendation": recommendation,
            }
        )
    return entries


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a review report for tracked runtime/generated artifact candidates."
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Git repository root to inspect.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "tracked_artifact_review"),
        help="Directory used to store the review report.",
    )
    parser.add_argument(
        "--cleanup-report",
        default=None,
        help="Optional existing cleanup report path. Defaults to <output-root>/worktree_cleanup_report.json.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional override path for the structured review report.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    source_root = Path(args.source_root)
    output_root = Path(args.output_root)
    report_path = (
        Path(args.report_file)
        if args.report_file
        else output_root / "tracked_artifact_review_report.json"
    )
    cleanup_report_path = _resolve_cleanup_report_path(args)
    cleanup_report = _load_cleanup_report(cleanup_report_path)
    candidates = _tracked_artifact_candidates(cleanup_report)
    entries = _build_entries(source_root, candidates)
    payload = {
        "schema_version": "1.0",
        "artifact_type": "tracked_artifact_review_report",
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "cleanup_report_path": str(cleanup_report_path),
        "report_path": str(report_path),
        "tracked_candidate_count": len(entries),
        "entries": entries,
        "next_step_plan": [
            "先人工确认 tracked runtime artifacts 是否只是 smoke/golden 产物；若不是发布输入，应显式回退或迁出。",
            "再决定 tracked generated artifact 是否保留跟踪；若不需要，应改成可重建并停止跟踪。",
        ]
        if entries
        else ["当前没有 tracked runtime/generated artifact 候选。"],
    }
    written_report = _write_report(report_path, payload)
    print(f"tracked_artifact_review_report_written={written_report}")
    print(f"tracked_artifact_review_candidates={len(entries)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
