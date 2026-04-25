#!/usr/bin/env python
"""Build a structured plan for the remaining external delivery mainline."""

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
    build_external_mainline_execution_plan_artifact,
    default_canonical_industrial_delivery_rehearsal_report_path,
    default_customer_external_bindings_closure_report_path,
    default_customer_external_bindings_config_path,
    default_external_mainline_execution_plan_path,
    default_vulnerability_exception_review_report_path,
    write_external_mainline_execution_plan_artifact,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a structured plan for the remaining external delivery mainline."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve default artifact paths.",
    )
    parser.add_argument(
        "--output",
        default=default_external_mainline_execution_plan_path(),
        help="Output path for external_mainline_execution_plan.json.",
    )
    parser.add_argument(
        "--customer-config",
        default=default_customer_external_bindings_config_path(),
        help="Customer-specific external bindings config inspected by the plan.",
    )
    parser.add_argument(
        "--customer-external-bindings-closure-report",
        default=default_customer_external_bindings_closure_report_path(),
        help="Existing closure report used to summarize customer external bindings progress.",
    )
    parser.add_argument(
        "--vulnerability-exception-review-report",
        default=default_vulnerability_exception_review_report_path(),
        help="Existing vulnerability exception review report used to summarize residual-risk follow-up.",
    )
    parser.add_argument(
        "--industrial-delivery-rehearsal-report",
        default=default_canonical_industrial_delivery_rehearsal_report_path(),
        help="Canonical industrial delivery rehearsal report used to summarize the live-evidence baseline.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    payload = build_external_mainline_execution_plan_artifact(
        project_root=args.project_root,
        customer_config_path=args.customer_config,
        customer_external_bindings_closure_report_path=args.customer_external_bindings_closure_report,
        vulnerability_exception_review_report_path=args.vulnerability_exception_review_report,
        industrial_delivery_rehearsal_report_path=args.industrial_delivery_rehearsal_report,
    )
    output_path = write_external_mainline_execution_plan_artifact(payload, args.output)

    print(f"external_mainline_execution_plan_written={output_path}")
    print(f"external_mainline_execution_plan_status={payload['status']}")
    print(
        "external_mainline_execution_plan_counts="
        f"completed:{payload['completed_steps']},"
        f"ready_to_run:{payload['ready_to_run_steps']},"
        f"waiting_external_input:{payload['waiting_external_input_steps']},"
        f"blocked:{payload['blocked_steps']}"
    )
    for item in payload["steps"]:
        print(
            "external_mainline_step_"
            f"{item['id']}={item['status']}"
        )
    return 0 if payload["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
