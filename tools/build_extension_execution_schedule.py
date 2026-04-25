#!/usr/bin/env python
"""Build a customer-specific extension execution schedule artifact."""

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

from agi_walker.core.api.release_contracts import (  # noqa: E402
    build_extension_execution_schedule_artifact,
    default_extension_execution_instance_artifact_path,
    default_extension_execution_schedule_artifact_path,
    write_extension_execution_schedule_artifact,
)


def _resolve_project_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a customer-specific extension execution schedule artifact."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve artifact paths.",
    )
    parser.add_argument(
        "--output",
        default=default_extension_execution_schedule_artifact_path(),
        help="Output path for the generated extension execution schedule artifact.",
    )
    parser.add_argument(
        "--instance-artifact",
        default=default_extension_execution_instance_artifact_path(),
        help="Extension execution instance artifact used as the source for schedule materialization.",
    )
    parser.add_argument("--window-trigger-at", default=None)
    parser.add_argument("--signoff-due-at", default=None)
    parser.add_argument("--closure-archive-due-at", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root)
    output_path = _resolve_project_path(args.output, project_root)

    payload = build_extension_execution_schedule_artifact(
        project_root=project_root,
        artifact_path=args.output,
        instance_artifact_path=args.instance_artifact,
        window_trigger_at=args.window_trigger_at,
        signoff_due_at=args.signoff_due_at,
        closure_archive_due_at=args.closure_archive_due_at,
    )
    written_path = write_extension_execution_schedule_artifact(payload, output_path)

    print(f"extension_execution_schedule_written={written_path}")
    print(f"extension_execution_schedule_status={payload['status']}")
    print(
        "extension_execution_schedule_profiles="
        f"{payload['ready_profiles']}/{payload['actionable_profiles']}"
    )
    print(
        "extension_execution_schedule_window_trigger_at="
        f"{payload['window_trigger_at']}"
    )
    print(
        "extension_execution_schedule_closure_archive_due_at="
        f"{payload['closure_archive_due_at']}"
    )
    return 0 if payload["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
