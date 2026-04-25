#!/usr/bin/env python
"""Build an industrial delivery rehearsal report from a release rehearsal report."""

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
    IndustrialDeliveryRehearsalReportRequest,
)
from agi_walker.ops.industrial_delivery import (  # noqa: E402
    execute_industrial_delivery_rehearsal_report,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build an industrial delivery rehearsal report from a release rehearsal report."
    )
    parser.add_argument(
        "--rehearsal-report",
        default=str(PROJECT_ROOT / "test_env" / "release_rehearsal" / "release_rehearsal_report.json"),
        help="Path to release_rehearsal_report.json.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional output path for industrial_delivery_rehearsal_report.json.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        result = execute_industrial_delivery_rehearsal_report(
            IndustrialDeliveryRehearsalReportRequest(
                rehearsal_report=args.rehearsal_report,
                output=args.output,
            )
        )
    except ValueError as exc:
        parser.error(str(exc))

    payload = result.payload
    stage_summary = payload["stage_summary"]
    review_payload = payload.get("vulnerability_exception_review", {})
    review_status = str(review_payload.get("status") or "missing").strip() or "missing"
    review_candidate_count = review_payload.get("review_candidate_count")
    review_summary = (
        f"{review_status}/{review_candidate_count}"
        if isinstance(review_candidate_count, int) and review_candidate_count >= 0
        else review_status
    )
    closure_payload = payload.get("customer_external_bindings_closure", {})
    closure_status = (
        str(closure_payload.get("status") or "missing").strip() or "missing"
    )
    external_mainline_payload = payload.get("external_mainline_execution_plan", {})
    external_mainline_status = (
        str(external_mainline_payload.get("status") or "missing").strip() or "missing"
    )
    external_mainline_counts: list[str] = []
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = external_mainline_payload.get(field)
        if not isinstance(value, int) or value < 0:
            external_mainline_counts = []
            break
        external_mainline_counts.append(str(value))
    external_mainline_summary = (
        "/".join([external_mainline_status, *external_mainline_counts])
        if external_mainline_counts
        else external_mainline_status
    )
    release_ops_execution_payload = payload.get("release_ops_execution", {})
    release_ops_execution_status = (
        str(release_ops_execution_payload.get("status") or "missing").strip() or "missing"
    )
    release_ops_execution_event_count = release_ops_execution_payload.get("event_count")
    release_ops_execution_summary = (
        f"{release_ops_execution_status}/{release_ops_execution_event_count}"
        if isinstance(release_ops_execution_event_count, int)
        and release_ops_execution_event_count >= 0
        else release_ops_execution_status
    )
    checklist_payload = payload.get("external_mainline_input_checklist", {})
    checklist_status = (
        str(checklist_payload.get("status") or "missing").strip() or "missing"
    )
    checklist_missing_inputs = checklist_payload.get("missing_input_count")
    checklist_counts: list[str] = []
    for field in [
        "waiting_external_input_steps",
        "ready_to_run_steps",
        "completed_steps",
    ]:
        value = checklist_payload.get(field)
        if isinstance(value, list):
            checklist_counts.append(
                str(len([item for item in value if isinstance(item, str) and item.strip()]))
            )
            continue
        checklist_counts = []
        break
    checklist_summary = (
        "/".join([checklist_status, str(checklist_missing_inputs), *checklist_counts])
        if isinstance(checklist_missing_inputs, int)
        and checklist_missing_inputs >= 0
        and checklist_counts
        else checklist_status
    )
    print(f"industrial_delivery_rehearsal_report_written={result.output_path}")
    print(f"industrial_delivery_rehearsal_status={payload['status']}")
    print(
        "industrial_delivery_rehearsal_stages="
        f"{stage_summary['passed']}/{stage_summary['total']}"
    )
    print(f"industrial_delivery_vulnerability_exception_review={review_summary}")
    print(f"industrial_delivery_customer_external_bindings_closure={closure_status}")
    print(
        "industrial_delivery_external_mainline_execution_plan="
        f"{external_mainline_summary}"
    )
    print(
        "industrial_delivery_release_ops_execution="
        f"{release_ops_execution_summary}"
    )
    print(
        "industrial_delivery_external_mainline_input_checklist="
        f"{checklist_summary}"
    )
    control_plane_event_stream = payload.get("control_plane_event_stream", {})
    if isinstance(control_plane_event_stream, dict) and isinstance(
        control_plane_event_stream.get("event_count"), int
    ):
        print(
            "industrial_delivery_control_plane_events="
            f"{control_plane_event_stream['event_count']}"
        )
    return 0 if payload["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
