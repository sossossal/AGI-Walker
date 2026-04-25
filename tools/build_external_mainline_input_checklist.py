#!/usr/bin/env python
"""Build a structured checklist of missing external-mainline inputs."""

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
    build_external_mainline_input_checklist_report,
    default_external_mainline_execution_plan_path,
    default_external_mainline_input_checklist_report_path,
    default_external_mainline_inputs_path,
    write_release_evidence_report,
)


def _resolve_project_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a structured checklist of missing external-mainline inputs."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve artifact paths.",
    )
    parser.add_argument(
        "--inputs-file",
        default=default_external_mainline_inputs_path(),
        help="Managed external-mainline inputs JSON path.",
    )
    parser.add_argument(
        "--external-mainline-execution-plan",
        default=default_external_mainline_execution_plan_path(),
        help="Optional plan path stored into the checklist metrics.",
    )
    parser.add_argument(
        "--output",
        default=default_external_mainline_input_checklist_report_path(),
        help="Output path for the structured checklist report.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root)
    output_path = _resolve_project_path(args.output, project_root)

    payload = build_external_mainline_input_checklist_report(
        project_root=project_root,
        inputs_file_path=args.inputs_file,
        external_mainline_execution_plan_path=args.external_mainline_execution_plan,
        output_path=args.output,
    )
    written_path = write_release_evidence_report(payload, output_path)
    print(f"external_mainline_input_checklist_report_written={written_path}")
    print(f"external_mainline_input_checklist_status={payload['status']}")
    metrics = payload.get("metrics", {})
    print(
        "external_mainline_input_checklist_missing_input_count="
        f"{metrics.get('missing_input_count', 0)}"
    )
    print(
        "external_mainline_input_checklist_waiting_steps="
        + ",".join(metrics.get("waiting_external_input_steps", []))
    )
    return 0 if payload["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
