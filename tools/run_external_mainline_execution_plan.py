#!/usr/bin/env python
"""Refresh automatable external-mainline artifacts and emit the unified plan."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


def _configure_stdio() -> None:
    for stream_name in ("stdout", "stderr"):
        stream = getattr(sys, stream_name, None)
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")


_configure_stdio()
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_contracts import (  # noqa: E402
    EXTENSION_EXTERNAL_BINDING_SECTION_IDS,
    default_canonical_industrial_delivery_rehearsal_report_path,
    default_customer_external_bindings_closure_report_path,
    default_customer_external_bindings_config_path,
    default_external_mainline_input_checklist_report_path,
    default_external_mainline_execution_plan_path,
    default_external_mainline_inputs_path,
    default_vulnerability_exception_review_report_path,
)
from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    ExternalMainlineExecutionRequest,
)
from agi_walker.ops.external_mainline import (  # noqa: E402
    execute_external_mainline_execution,
)


def _run(command: list[str]) -> int:
    print(f"command={' '.join(command)}")
    result = subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.stdout.strip():
        print(result.stdout.strip())
    if result.stderr.strip():
        print(result.stderr.strip(), file=sys.stderr)
    return result.returncode


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Refresh the automatable parts of the remaining external delivery mainline "
            "and write external_mainline_execution_plan.json."
        )
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve default artifact paths.",
    )
    parser.add_argument(
        "--inputs-file",
        default=default_external_mainline_inputs_path(),
        help=(
            "Managed JSON file containing external-mainline runner inputs. "
            "Defaults to deployment/external_mainline.inputs.json."
        ),
    )
    parser.add_argument(
        "--skip-managed-inputs",
        action="store_true",
        help="Do not load or synchronize the managed external-mainline inputs file.",
    )
    parser.add_argument(
        "--output",
        default=default_external_mainline_execution_plan_path(),
        help="Output path for external_mainline_execution_plan.json.",
    )
    parser.add_argument(
        "--external-mainline-input-checklist-report",
        default=default_external_mainline_input_checklist_report_path(),
        help="Output path for the structured external-mainline input checklist report.",
    )
    parser.add_argument(
        "--customer-config",
        default=default_customer_external_bindings_config_path(),
        help="Customer-specific external bindings config passed through to the plan and closure runner.",
    )
    parser.add_argument(
        "--customer-external-bindings-closure-report",
        default=default_customer_external_bindings_closure_report_path(),
        help="Path used both for the closure runner output and final plan inspection.",
    )
    parser.add_argument(
        "--vulnerability-exception-review-report",
        default=default_vulnerability_exception_review_report_path(),
        help="Path used both for the review refresh output and final plan inspection.",
    )
    parser.add_argument(
        "--industrial-delivery-rehearsal-report",
        default=default_canonical_industrial_delivery_rehearsal_report_path(),
        help="Industrial rehearsal report inspected by the final plan.",
    )
    parser.add_argument(
        "--skip-vulnerability-exception-review-refresh",
        action="store_true",
        help="Do not rebuild vulnerability_exception_review_report.json before writing the plan.",
    )
    parser.add_argument(
        "--skip-customer-external-bindings-closure",
        action="store_true",
        help="Do not invoke the managed customer external bindings closure runner.",
    )
    parser.add_argument(
        "--customer-section",
        action="append",
        choices=list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS),
        default=None,
        help="Optional section subset passed through to run_customer_external_bindings_closure.py.",
    )
    parser.add_argument("--customer-confirmed-by", default=None)
    parser.add_argument("--customer-confirmed-at", default=None)
    parser.add_argument("--customer-confirmation-ticket", default=None)
    parser.add_argument("--customer-confirmation-notes", default=None)
    parser.add_argument("--customer-confirmation-evidence", default=None)
    parser.add_argument("--customer-overrides-file", default=None)
    parser.add_argument(
        "--customer-set",
        action="append",
        default=None,
        help="Optional section.field=value override passed through to the closure runner.",
    )
    parser.add_argument(
        "--skip-customer-collect-release-evidence",
        action="store_true",
        help="Pass --skip-collect-release-evidence through to run_customer_external_bindings_closure.py.",
    )
    parser.add_argument(
        "--refresh-industrial-rehearsal",
        action="store_true",
        help="Refresh the canonical industrial rehearsal baseline before writing the plan.",
    )
    parser.add_argument(
        "--industrial-rehearsal-version",
        default=None,
        help="Version/tag passed to tools/run_release_rehearsal.py when --refresh-industrial-rehearsal is used.",
    )
    parser.add_argument(
        "--industrial-rehearsal-build-id",
        default=None,
        help="Build id passed to tools/run_release_rehearsal.py when --refresh-industrial-rehearsal is used.",
    )
    parser.add_argument(
        "--industrial-rehearsal-output-root",
        default="test_env/release_rehearsal_industrial",
        help="Output root passed to tools/run_release_rehearsal.py when --refresh-industrial-rehearsal is used.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    request = ExternalMainlineExecutionRequest(
        project_root=args.project_root,
        inputs_file=args.inputs_file,
        skip_managed_inputs=args.skip_managed_inputs,
        output=args.output,
        external_mainline_input_checklist_report=args.external_mainline_input_checklist_report,
        customer_config=args.customer_config,
        customer_external_bindings_closure_report=args.customer_external_bindings_closure_report,
        vulnerability_exception_review_report=args.vulnerability_exception_review_report,
        industrial_delivery_rehearsal_report=args.industrial_delivery_rehearsal_report,
        skip_vulnerability_exception_review_refresh=args.skip_vulnerability_exception_review_refresh,
        skip_customer_external_bindings_closure=args.skip_customer_external_bindings_closure,
        customer_section=args.customer_section,
        customer_confirmed_by=args.customer_confirmed_by,
        customer_confirmed_at=args.customer_confirmed_at,
        customer_confirmation_ticket=args.customer_confirmation_ticket,
        customer_confirmation_notes=args.customer_confirmation_notes,
        customer_confirmation_evidence=args.customer_confirmation_evidence,
        customer_overrides_file=args.customer_overrides_file,
        customer_set=args.customer_set,
        skip_customer_collect_release_evidence=args.skip_customer_collect_release_evidence,
        refresh_industrial_rehearsal=args.refresh_industrial_rehearsal,
        industrial_rehearsal_version=args.industrial_rehearsal_version,
        industrial_rehearsal_build_id=args.industrial_rehearsal_build_id,
        industrial_rehearsal_output_root=args.industrial_rehearsal_output_root,
    )
    try:
        result = execute_external_mainline_execution(
            request,
            run_command=_run,
            python_executable=sys.executable,
        )
    except ValueError as exc:
        parser.error(str(exc))

    print(f"external_mainline_execution_plan_written={result.output_path}")
    print(f"external_mainline_execution_plan_status={result.payload['status']}")
    print(
        "external_mainline_input_checklist_report_written="
        f"{result.checklist_path}"
    )
    print(
        "external_mainline_input_checklist_status="
        f"{result.checklist_payload['status']}"
    )
    if result.resolved_inputs_path is not None:
        print(
            "external_mainline_execution_plan_inputs_file="
            f"{result.resolved_inputs_path}"
        )
    if result.managed_inputs_sync_status == "bootstrapped":
        print(
            "external_mainline_execution_plan_inputs_file_bootstrapped="
            f"{result.resolved_inputs_path}"
        )
    elif result.managed_inputs_sync_status == "refreshed":
        print(
            "external_mainline_execution_plan_inputs_file_refreshed="
            f"{result.resolved_inputs_path}"
        )
    print(
        "external_mainline_execution_plan_executed_steps="
        + ",".join(result.executed_steps)
    )
    print(
        "external_mainline_execution_plan_skipped_steps="
        + ",".join(result.skipped_steps)
    )
    print(
        "external_mainline_execution_plan_failures="
        f"{len(result.failures)}"
    )
    if result.industrial_live_evidence_inputs_ready is not None:
        print(
            "external_mainline_execution_plan_industrial_live_evidence_inputs_ready="
            f"{str(result.industrial_live_evidence_inputs_ready).lower()}"
        )
    for item in result.payload["steps"]:
        print(f"external_mainline_step_{item['id']}={item['status']}")

    return 1 if result.failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
