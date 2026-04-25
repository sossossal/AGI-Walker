#!/usr/bin/env python
"""Check industrial release readiness and emit next actions."""

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


PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_contracts import RELEASE_APPROVAL_STATUSES  # noqa: E402
from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    IndustrialReleaseReadinessRequest,
)
from agi_walker.ops.readiness import (  # noqa: E402
    execute_industrial_release_readiness,
    format_external_mainline_execution_plan_preview,
    format_external_mainline_input_checklist_preview,
    format_worktree_release_blocker_preview,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check industrial release readiness and print the next actions."
    )
    parser.add_argument(
        "--current-version",
        default=None,
        help="Current release version. Defaults to the version inferred from RELEASE_NOTES.md.",
    )
    parser.add_argument(
        "--industrial-version",
        default=None,
        help="Industrial target version. Defaults to current version with rc/dev suffix stripped.",
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve evidence artifact paths.",
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Repository root used to resolve Git release source metadata.",
    )
    parser.add_argument(
        "--changelog",
        default="RELEASE_NOTES.md",
        help="Changelog path used for metadata inference.",
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "industrial_release_readiness"),
        help="Directory used to store the industrial preview manifest and readiness report.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional override path for the structured industrial readiness report.",
    )
    parser.add_argument(
        "--approval-status",
        choices=sorted(RELEASE_APPROVAL_STATUSES - {"not_required"}),
        default=None,
        help="Optional approval status used for the industrial readiness preview.",
    )
    parser.add_argument("--approved-by", default=None)
    parser.add_argument("--approved-at", default=None)
    parser.add_argument("--commit-sha", default=None)
    parser.add_argument("--approval-notes", default=None)
    parser.add_argument(
        "--approval-manifest",
        default=None,
        help="Optional industrial release manifest used to import release_approval metadata.",
    )
    parser.add_argument(
        "--security-preflight-report",
        default=None,
        help="Optional structured security_release_preflight_report path. Defaults to test_env/release_evidence/security_release_preflight_report.json under --project-root.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    default_output_root = PROJECT_ROOT / "test_env" / "industrial_release_readiness"
    report_path = (
        Path(args.report_file)
        if args.report_file
        else Path(args.output_root) / "industrial_release_readiness_report.json"
    )
    output_root = Path(args.output_root)
    if args.report_file and output_root == default_output_root:
        output_root = report_path.parent
    try:
        result = execute_industrial_release_readiness(
            IndustrialReleaseReadinessRequest(
                current_version=args.current_version,
                industrial_version=args.industrial_version,
                project_root=args.project_root,
                source_root=args.source_root,
                changelog=args.changelog,
                output_root=str(output_root),
                report_file=str(report_path),
                approval_status=args.approval_status,
                approved_by=args.approved_by,
                approved_at=args.approved_at,
                commit_sha=args.commit_sha,
                approval_notes=args.approval_notes,
                approval_manifest=args.approval_manifest,
                security_preflight_report=args.security_preflight_report,
            )
        )
    except ValueError as exc:
        parser.error(str(exc))

    industrial_preview = result.industrial_preview
    print(f"industrial_release_readiness_written={result.report_path}")
    print(f"current_version={result.current_version}")
    print(f"industrial_version={result.industrial_version}")
    print(f"industrial_release_gate={industrial_preview['release_gate_status']}")
    print(
        "industrial_security_preflight="
        f"{industrial_preview['security_release_preflight']['status']}"
    )
    review_preview = industrial_preview["vulnerability_exception_review"]
    review_candidate_count = review_preview.get("review_candidate_count")
    review_status = review_preview.get("status")
    review_stdout = (
        f"{review_status}/{review_candidate_count}"
        if isinstance(review_candidate_count, int) and review_candidate_count >= 0
        else str(review_status)
    )
    print(f"industrial_vulnerability_exception_review={review_stdout}")
    print(
        "industrial_external_mainline_execution_plan="
        f"{format_external_mainline_execution_plan_preview(industrial_preview['external_mainline_execution_plan'])}"
    )
    print(
        "industrial_external_mainline_input_checklist="
        f"{format_external_mainline_input_checklist_preview(industrial_preview['external_mainline_input_checklist'])}"
    )
    print(
        "industrial_worktree_release_blocker="
        f"{format_worktree_release_blocker_preview(industrial_preview['worktree_release_blocker'])}"
    )
    print(
        "industrial_customer_delivery="
        f"{industrial_preview['customer_delivery_surface']['status']}"
    )
    print(
        "industrial_industrial_delivery="
        f"{industrial_preview['industrial_delivery_gate']['status']}"
    )
    print(
        "industrial_extension_execution_instance="
        f"{industrial_preview['extension_execution_instance']['status']}"
    )
    print(
        "industrial_extension_execution_schedule="
        f"{industrial_preview['extension_execution_schedule']['status']}"
    )
    print(
        "industrial_extension_execution_actuals="
        f"{industrial_preview['extension_execution_actuals']['status']}"
    )
    print(f"industrial_next_actions={len(industrial_preview['next_actions'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
