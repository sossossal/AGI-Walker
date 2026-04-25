#!/usr/bin/env python
"""Build a unified worktree release-blocker report."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


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
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    WorktreeReleaseBlockerRequest,
)
from agi_walker.ops.worktree import execute_worktree_release_blocker  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run worktree cleanup + tracked review and emit a unified release-blocker report."
    )
    parser.add_argument("--source-root", default=str(PROJECT_ROOT))
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "worktree_cleanup"),
    )
    parser.add_argument(
        "--cleanup-report",
        default=None,
        help="Optional cleanup report path. Defaults to <output-root>/worktree_cleanup_report.json.",
    )
    parser.add_argument(
        "--review-report",
        default=None,
        help="Optional tracked review report path. Defaults to <output-root>/tracked_artifact_review_report.json.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional unified blocker report path. Defaults to <output-root>/worktree_release_blocker_report.json.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    result = execute_worktree_release_blocker(
        WorktreeReleaseBlockerRequest(
            source_root=args.source_root,
            output_root=args.output_root,
            cleanup_report=args.cleanup_report,
            review_report=args.review_report,
            report_file=args.report_file,
        )
    )
    payload = result.payload
    print(f"worktree_release_blocker_report_written={result.report_path}")
    print(f"worktree_release_blocker_status={payload['status']}")
    print(
        "worktree_release_blocker_clean="
        f"{str(bool(payload.get('clean_worktree'))).lower()}"
    )
    print(
        "worktree_release_blocker_tracked_review_status="
        f"{payload.get('tracked_review_status', 'not_required')}"
    )
    print(
        "worktree_release_blocker_tracked_review_candidates="
        f"{int(payload.get('tracked_review_candidate_count', 0))}"
    )
    return 0 if payload.get("status") == "ready" or not payload.get("failed_steps") else 1


if __name__ == "__main__":
    raise SystemExit(main())
