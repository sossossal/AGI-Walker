#!/usr/bin/env python
"""Check current rc/stable release readiness and emit next actions."""

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

from agi_walker.core.api.release_contracts import RELEASE_APPROVAL_STATUSES  # noqa: E402
from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    ReleaseReadinessRequest,
)
from agi_walker.ops.readiness import (  # noqa: E402
    execute_release_readiness,
    format_external_mainline_execution_plan_preview,
    format_external_mainline_input_checklist_preview,
    format_worktree_release_blocker_preview,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check rc/stable release readiness and print the next actions."
    )
    parser.add_argument(
        "--current-version",
        default=None,
        help="Current release version. Defaults to the version inferred from RELEASE_NOTES.md.",
    )
    parser.add_argument(
        "--stable-version",
        default=None,
        help="Stable target version. Defaults to current version with rc/dev suffix stripped.",
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
        default=str(PROJECT_ROOT / "test_env" / "release_readiness"),
        help="Directory used to store preview manifests and the readiness report.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional override path for the structured readiness report.",
    )
    parser.add_argument(
        "--approval-status",
        choices=sorted(RELEASE_APPROVAL_STATUSES - {"not_required"}),
        default=None,
        help="Optional approval status used for the stable readiness preview.",
    )
    parser.add_argument("--approved-by", default=None)
    parser.add_argument("--approved-at", default=None)
    parser.add_argument("--commit-sha", default=None)
    parser.add_argument("--approval-notes", default=None)
    parser.add_argument(
        "--approval-manifest",
        default=None,
        help="Optional stable release manifest used to import release_approval metadata.",
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
    default_output_root = PROJECT_ROOT / "test_env" / "release_readiness"
    report_path = (
        Path(args.report_file)
        if args.report_file
        else Path(args.output_root) / "release_readiness_report.json"
    )
    output_root = Path(args.output_root)
    if args.report_file and output_root == default_output_root:
        output_root = report_path.parent
    try:
        result = execute_release_readiness(
            ReleaseReadinessRequest(
                current_version=args.current_version,
                stable_version=args.stable_version,
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

    stable_preview = result.stable_preview
    rc_preview = result.rc_preview
    print(f"release_readiness_written={result.report_path}")
    print(f"current_version={result.current_version}")
    print(f"stable_version={result.stable_version}")
    print(f"rc_release_gate={rc_preview['release_gate_status']}")
    print(f"stable_release_gate={stable_preview['release_gate_status']}")
    print(
        "stable_security_preflight="
        f"{stable_preview['security_release_preflight']['status']}"
    )
    print(
        "stable_customer_delivery="
        f"{stable_preview['customer_delivery_surface']['status']}"
    )
    print(
        "stable_industrial_delivery="
        f"{stable_preview['industrial_delivery_gate']['status']}"
    )
    print(
        "stable_extension_execution_instance="
        f"{stable_preview['extension_execution_instance']['status']}"
    )
    print(
        "stable_extension_execution_schedule="
        f"{stable_preview['extension_execution_schedule']['status']}"
    )
    print(
        "stable_extension_execution_actuals="
        f"{stable_preview['extension_execution_actuals']['status']}"
    )
    print(
        "stable_external_mainline_execution_plan="
        f"{format_external_mainline_execution_plan_preview(stable_preview['external_mainline_execution_plan'])}"
    )
    print(
        "stable_external_mainline_input_checklist="
        f"{format_external_mainline_input_checklist_preview(stable_preview['external_mainline_input_checklist'])}"
    )
    print(
        "stable_worktree_release_blocker="
        f"{format_worktree_release_blocker_preview(stable_preview['worktree_release_blocker'])}"
    )
    review_preview = stable_preview["vulnerability_exception_review"]
    review_candidate_count = review_preview.get("review_candidate_count")
    review_status = review_preview.get("status")
    review_stdout = (
        f"{review_status}/{review_candidate_count}"
        if isinstance(review_candidate_count, int) and review_candidate_count >= 0
        else str(review_status)
    )
    print(f"stable_vulnerability_exception_review={review_stdout}")
    print(f"stable_next_actions={len(stable_preview['next_actions'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
