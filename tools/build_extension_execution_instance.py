#!/usr/bin/env python
"""Build a customer-specific extension execution instance artifact."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timedelta, timezone
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
    build_extension_execution_instance_artifact,
    default_extension_execution_instance_artifact_path,
    write_extension_execution_instance_artifact,
)
from agi_walker.core.api.security_posture_contracts import (  # noqa: E402
    validate_vulnerability_exception_report,
)


def _resolve_project_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def _default_window() -> tuple[str, str]:
    start = datetime.now(timezone.utc).replace(microsecond=0)
    end = (start + timedelta(hours=2)).replace(microsecond=0)
    return start.isoformat(), end.isoformat()


def _derive_exception_review_due_at(path: Path) -> str | None:
    if not path.is_file():
        return None
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    if validate_vulnerability_exception_report(payload):
        return None
    exceptions = payload.get("exceptions")
    if not isinstance(exceptions, list):
        return None
    expiries = sorted(
        str(item.get("expires_at")).strip()
        for item in exceptions
        if isinstance(item, dict)
        and isinstance(item.get("expires_at"), str)
        and item.get("expires_at").strip()
    )
    return expiries[0] if expiries else None


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a customer-specific extension execution instance artifact."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve artifact paths.",
    )
    parser.add_argument(
        "--output",
        default=default_extension_execution_instance_artifact_path(),
        help="Output path for the generated extension execution instance artifact.",
    )
    parser.add_argument(
        "--vulnerability-exception-report",
        default="test_env/release_evidence/security/vulnerability_exception_report.json",
        help="Structured vulnerability exception report used to derive exception_review_due_at when not supplied explicitly.",
    )
    parser.add_argument("--engagement-id", default="canonical-release-delivery")
    parser.add_argument("--customer-name", default="AGI-Walker Customer")
    parser.add_argument("--site-name", default="primary-site")
    parser.add_argument("--change-ticket", default="CHG-CANONICAL-RELEASE")
    parser.add_argument("--window-id", default="window-canonical-release")
    parser.add_argument("--window-start-at", default=None)
    parser.add_argument("--window-end-at", default=None)
    parser.add_argument(
        "--delivery-root",
        default="test_env/release_delivery/canonical_release_delivery",
    )
    parser.add_argument(
        "--closure-archive-root",
        default="test_env/release_delivery/canonical_release_delivery/closure_archive",
    )
    parser.add_argument("--exception-review-due-at", default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root)
    output_path = _resolve_project_path(args.output, project_root)
    exception_report_path = _resolve_project_path(
        args.vulnerability_exception_report,
        project_root,
    )
    default_start_at, default_end_at = _default_window()
    window_start_at = args.window_start_at or default_start_at
    window_end_at = args.window_end_at or default_end_at
    exception_review_due_at = (
        args.exception_review_due_at
        or _derive_exception_review_due_at(exception_report_path)
        or window_end_at
    )

    payload = build_extension_execution_instance_artifact(
        project_root=project_root,
        artifact_path=args.output,
        engagement_id=args.engagement_id,
        customer_name=args.customer_name,
        site_name=args.site_name,
        change_ticket=args.change_ticket,
        window_id=args.window_id,
        window_start_at=window_start_at,
        window_end_at=window_end_at,
        delivery_root=args.delivery_root,
        closure_archive_root=args.closure_archive_root,
        exception_review_due_at=exception_review_due_at,
    )
    written_path = write_extension_execution_instance_artifact(payload, output_path)

    print(f"extension_execution_instance_written={written_path}")
    print(f"extension_execution_instance_status={payload['status']}")
    print(
        "extension_execution_instance_profiles="
        f"{payload['ready_profiles']}/{payload['actionable_profiles']}"
    )
    print(
        "extension_execution_instance_exception_review_due_at="
        f"{payload['exception_review_due_at']}"
    )
    return 0 if payload["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
