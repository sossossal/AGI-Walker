#!/usr/bin/env python
"""Run a stable release rehearsal with a temporary Git source repo."""

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


def _format_external_mainline_execution_plan_summary(component: dict[str, object]) -> str:
    status = str(component.get("status") or "missing").strip() or "missing"
    counts: list[str] = []
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = component.get(field)
        if not isinstance(value, int) or value < 0:
            return status
        counts.append(str(value))
    return "/".join([status, *counts])


def _format_release_ops_execution_summary(component: dict[str, object]) -> str:
    status = str(component.get("status") or "missing").strip() or "missing"
    event_count = component.get("event_count")
    if isinstance(event_count, int) and event_count >= 0:
        return f"{status}/{event_count}"
    return status


_configure_stdio()
PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    ReleaseRehearsalRequest,
)
from agi_walker.ops.rehearsal import execute_release_rehearsal  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run a stable release rehearsal with a temporary Git source repo."
    )
    parser.add_argument(
        "--version",
        default="2026.04.12-rehearsal",
        help="Version used for the rehearsal manifest and matching tag.",
    )
    parser.add_argument(
        "--tag",
        default=None,
        help="Optional tag override. Defaults to the same value as --version.",
    )
    parser.add_argument(
        "--build-id",
        default="release-rehearsal",
        help="Stable build identifier written into the rehearsal manifest.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "release_rehearsal"),
        help="Workspace used for the temporary Git repo, reports, and manifest.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional override path for the structured rehearsal report.",
    )
    parser.add_argument(
        "--external-bindings-config-source",
        default=str(
            Path("deployment") / "customer_delivery.external_bindings.rehearsal.json"
        ),
        help="Managed external bindings config copied into the rehearsal workspace and passed to actuals generation.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    result = execute_release_rehearsal(
        ReleaseRehearsalRequest(
            version=args.version,
            tag=args.tag,
            build_id=args.build_id,
            output_root=args.output_root,
            report_file=args.report_file,
            external_bindings_config_source=args.external_bindings_config_source,
        )
    )

    print(f"release_rehearsal_written={result.report_path}")
    print(
        "industrial_delivery_rehearsal_report_written="
        f"{result.industrial_delivery_rehearsal_report_path}"
    )
    print(
        "industrial_delivery_rehearsal_status="
        f"{result.industrial_delivery_rehearsal_payload['status']}"
    )
    review_payload = result.industrial_delivery_rehearsal_payload.get(
        "vulnerability_exception_review",
        {},
    )
    review_status = str(review_payload.get("status") or "missing").strip() or "missing"
    review_candidate_count = review_payload.get("review_candidate_count")
    review_summary = (
        f"{review_status}/{review_candidate_count}"
        if isinstance(review_candidate_count, int) and review_candidate_count >= 0
        else review_status
    )
    print(f"industrial_delivery_vulnerability_exception_review={review_summary}")
    closure_payload = result.industrial_delivery_rehearsal_payload.get(
        "customer_external_bindings_closure",
        {},
    )
    closure_status = (
        str(closure_payload.get("status") or "missing").strip() or "missing"
    )
    print(f"industrial_delivery_customer_external_bindings_closure={closure_status}")
    external_mainline_payload = result.industrial_delivery_rehearsal_payload.get(
        "external_mainline_execution_plan",
        {},
    )
    external_mainline_summary = _format_external_mainline_execution_plan_summary(
        external_mainline_payload
        if isinstance(external_mainline_payload, dict)
        else {},
    )
    print(
        "industrial_delivery_external_mainline_execution_plan="
        f"{external_mainline_summary}"
    )
    release_ops_execution_payload = result.industrial_delivery_rehearsal_payload.get(
        "release_ops_execution",
        {},
    )
    print(
        "industrial_delivery_release_ops_execution="
        f"{_format_release_ops_execution_summary(release_ops_execution_payload if isinstance(release_ops_execution_payload, dict) else {})}"
    )
    rehearsal_control_plane_events = (
        result.payload.get("control_plane_event_stream", {})
        if isinstance(result.payload.get("control_plane_event_stream"), dict)
        else {}
    )
    if isinstance(rehearsal_control_plane_events.get("event_count"), int):
        print(
            "release_rehearsal_control_plane_events="
            f"{rehearsal_control_plane_events['event_count']}"
        )
    industrial_control_plane_events = (
        result.industrial_delivery_rehearsal_payload.get("control_plane_event_stream", {})
        if isinstance(
            result.industrial_delivery_rehearsal_payload.get(
                "control_plane_event_stream"
            ),
            dict,
        )
        else {}
    )
    if isinstance(industrial_control_plane_events.get("event_count"), int):
        print(
            "industrial_delivery_control_plane_events="
            f"{industrial_control_plane_events['event_count']}"
        )
    print(f"release_rehearsal_gate={result.gate_status}")
    print(f"release_rehearsal_tag={result.tag}")
    return 0 if result.payload.get("status") == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
