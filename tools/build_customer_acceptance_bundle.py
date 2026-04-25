#!/usr/bin/env python
"""Build a customer-facing acceptance bundle from a release manifest."""

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
    CustomerAcceptanceBundleRequest,
)
from agi_walker.ops.acceptance import execute_customer_acceptance_bundle  # noqa: E402


def _format_vulnerability_exception_review(report: object) -> str:
    if not isinstance(report, dict):
        return "missing"
    status = str(report.get("status") or "missing").strip() or "missing"
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    candidate_count = metrics.get("review_candidate_count")
    if not isinstance(candidate_count, int) or candidate_count < 0:
        candidate_count = metrics.get("review_due_exception_count")
    if isinstance(candidate_count, int) and candidate_count >= 0:
        return f"{status}/{candidate_count}"
    return status


def _format_external_mainline_execution_plan(report: object) -> str:
    if not isinstance(report, dict):
        return "missing"
    status = str(report.get("status") or "missing").strip() or "missing"
    counts: list[str] = []
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = report.get(field)
        if not isinstance(value, int) or value < 0:
            counts = []
            break
        counts.append(str(value))
    return "/".join([status, *counts]) if counts else status


def _format_external_mainline_input_checklist(report: object) -> str:
    if not isinstance(report, dict):
        return "missing"
    status = str(report.get("status") or "missing").strip() or "missing"
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    counts: list[str] = []
    metric_values: list[object] = [
        metrics.get("missing_input_count"),
        len(metrics.get("waiting_external_input_steps", []))
        if isinstance(metrics.get("waiting_external_input_steps"), list)
        else None,
        len(metrics.get("ready_to_run_steps", []))
        if isinstance(metrics.get("ready_to_run_steps"), list)
        else None,
        len(metrics.get("completed_steps", []))
        if isinstance(metrics.get("completed_steps"), list)
        else None,
    ]
    for value in metric_values:
        if not isinstance(value, int) or value < 0:
            counts = []
            break
        counts.append(str(value))
    return "/".join([status, *counts]) if counts else status


def _format_release_ops_execution(report: object) -> str:
    if not isinstance(report, dict):
        return "missing"
    status = str(report.get("status") or "missing").strip() or "missing"
    metrics = report.get("metrics") if isinstance(report.get("metrics"), dict) else {}
    event_count = metrics.get("event_count")
    if not isinstance(event_count, int) or event_count < 0:
        control_plane_event_stream = (
            report.get("control_plane_event_stream")
            if isinstance(report.get("control_plane_event_stream"), dict)
            else {}
        )
        event_count = control_plane_event_stream.get("event_count")
    if isinstance(event_count, int) and event_count >= 0:
        return f"{status}/{event_count}"
    return status


def _format_control_plane_events(payload: object) -> str | None:
    if not isinstance(payload, dict):
        return None
    event_stream = (
        payload.get("control_plane_event_stream")
        if isinstance(payload.get("control_plane_event_stream"), dict)
        else None
    )
    if not isinstance(event_stream, dict):
        return None
    event_count = event_stream.get("event_count")
    if not isinstance(event_count, int) or event_count < 0:
        return None
    return str(event_count)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a customer acceptance bundle from a release manifest."
    )
    parser.add_argument(
        "--manifest",
        default=str(PROJECT_ROOT / "test_env" / "release" / "release_manifest_stable.json"),
        help="Path to the source release manifest.",
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Root directory used to resolve documentation and report paths.",
    )
    parser.add_argument(
        "--output",
        default=str(
            PROJECT_ROOT / "test_env" / "release" / "customer_acceptance_bundle.json"
        ),
        help="Output path for the generated customer acceptance bundle.",
    )
    parser.add_argument(
        "--readiness-report",
        default=None,
        help="Optional override path for the channel-specific readiness report.",
    )
    parser.add_argument(
        "--promotion-checklist",
        default=None,
        help="Optional override path for the channel-specific promotion checklist.",
    )
    parser.add_argument(
        "--security-posture-report",
        default=None,
        help="Optional override path for security_posture_report.json.",
    )
    parser.add_argument(
        "--sbom-artifact",
        default=None,
        help="Optional override path for sbom.json.",
    )
    parser.add_argument(
        "--python-vuln-report",
        default=None,
        help="Optional override path for python_vuln_scan_report.json.",
    )
    parser.add_argument(
        "--container-vuln-report",
        default=None,
        help="Optional override path for container_vuln_scan_report.json.",
    )
    parser.add_argument(
        "--vulnerability-exception-report",
        default=None,
        help="Optional override path for vulnerability_exception_report.json.",
    )
    parser.add_argument(
        "--vulnerability-exception-review-report",
        default=None,
        help="Optional override path for vulnerability_exception_review_report.json.",
    )
    parser.add_argument(
        "--customer-external-bindings-closure-report",
        default=None,
        help="Optional override path for customer_external_bindings_closure_report.json.",
    )
    parser.add_argument(
        "--external-mainline-execution-plan",
        default=None,
        help="Optional override path for external_mainline_execution_plan.json.",
    )
    parser.add_argument(
        "--external-mainline-input-checklist-report",
        default=None,
        help="Optional override path for external_mainline_input_checklist_report.json.",
    )
    parser.add_argument(
        "--release-ops-execution-report",
        default=None,
        help="Optional override path for release_ops_execution_report.json.",
    )
    parser.add_argument(
        "--backup-restore-report",
        default=None,
        help="Optional override path for backup_restore_rehearsal_report.json.",
    )
    parser.add_argument(
        "--industrial-delivery-rehearsal-report",
        default=None,
        help="Optional override path for industrial_delivery_rehearsal_report.json.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        result = execute_customer_acceptance_bundle(
            CustomerAcceptanceBundleRequest(
                manifest=args.manifest,
                project_root=args.project_root,
                output=args.output,
                readiness_report=args.readiness_report,
                promotion_checklist=args.promotion_checklist,
                security_posture_report=args.security_posture_report,
                sbom_artifact=args.sbom_artifact,
                python_vuln_report=args.python_vuln_report,
                container_vuln_report=args.container_vuln_report,
                vulnerability_exception_report=args.vulnerability_exception_report,
                vulnerability_exception_review_report=args.vulnerability_exception_review_report,
                customer_external_bindings_closure_report=args.customer_external_bindings_closure_report,
                external_mainline_execution_plan=args.external_mainline_execution_plan,
                external_mainline_input_checklist_report=args.external_mainline_input_checklist_report,
                release_ops_execution_report=args.release_ops_execution_report,
                backup_restore_report=args.backup_restore_report,
                industrial_delivery_rehearsal_report=args.industrial_delivery_rehearsal_report,
            )
        )
    except ValueError as exc:
        parser.error(str(exc))

    bundle_payload = result.payload
    output_path = result.output_path

    required_docs = [
        item for item in bundle_payload["acceptance_documents"] if item["required"] is True
    ]
    ready_docs = [item for item in required_docs if item["exists"] is True]
    required_evidence = bundle_payload["required_evidence"]
    passed_required_evidence = [
        item for item in required_evidence if item.get("status") == "passed"
    ]

    print(f"customer_acceptance_bundle_written={output_path}")
    print(f"customer_acceptance_bundle_status={bundle_payload['bundle_status']}")
    print(
        "customer_acceptance_bundle_documents_ready="
        f"{len(ready_docs)}/{len(required_docs)}"
    )
    print(
        "customer_acceptance_bundle_required_evidence="
        f"{len(passed_required_evidence)}/{len(required_evidence)}"
    )
    acceptance_reports = bundle_payload["acceptance_reports"]
    present_reports = [item for item in acceptance_reports if item.get("exists") is True]
    print(
        "customer_acceptance_bundle_reports_present="
        f"{len(present_reports)}/{len(acceptance_reports)}"
    )
    security_posture_report = next(
        (
            item
            for item in acceptance_reports
            if item.get("name") == "security_posture_report"
        ),
        None,
    )
    if security_posture_report is not None:
        print(
            "customer_acceptance_bundle_security_posture="
            f"{security_posture_report.get('status') or 'missing'}"
        )
    print(
        "customer_acceptance_bundle_vulnerability_exception_review="
        f"{_format_vulnerability_exception_review(bundle_payload.get('vulnerability_exception_review'))}"
    )
    print(
        "customer_acceptance_bundle_external_mainline_execution_plan="
        f"{_format_external_mainline_execution_plan(bundle_payload.get('external_mainline_execution_plan'))}"
    )
    print(
        "customer_acceptance_bundle_external_mainline_input_checklist="
        f"{_format_external_mainline_input_checklist(bundle_payload.get('external_mainline_input_checklist'))}"
    )
    print(
        "customer_acceptance_bundle_release_ops_execution="
        f"{_format_release_ops_execution(bundle_payload.get('release_ops_execution'))}"
    )
    control_plane_events = _format_control_plane_events(bundle_payload)
    if control_plane_events is not None:
        print(f"customer_acceptance_bundle_control_plane_events={control_plane_events}")
    return 0 if bundle_payload["bundle_status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
