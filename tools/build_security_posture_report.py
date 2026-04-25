#!/usr/bin/env python
"""Build a security posture report from SBOM, scan reports, and baseline docs."""

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

from agi_walker.core.api.security_posture_contracts import (  # noqa: E402
    build_security_posture_report,
    write_security_posture_report,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build AGI-Walker security posture report.")
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve docs and artifact paths.",
    )
    parser.add_argument(
        "--sbom",
        default=str(PROJECT_ROOT / "test_env" / "security" / "sbom.json"),
        help="SBOM artifact path.",
    )
    parser.add_argument(
        "--python-vuln-report",
        default=str(PROJECT_ROOT / "test_env" / "security" / "python_vuln_scan_report.json"),
        help="Structured python dependency vulnerability scan report path.",
    )
    parser.add_argument(
        "--container-vuln-report",
        default=str(PROJECT_ROOT / "test_env" / "security" / "container_vuln_scan_report.json"),
        help="Structured container vulnerability scan report path.",
    )
    parser.add_argument(
        "--backup-restore-report",
        default=str(PROJECT_ROOT / "test_env" / "security" / "backup_restore_rehearsal_report.json"),
        help="Structured backup and restore rehearsal report path.",
    )
    parser.add_argument(
        "--vulnerability-remediation-report",
        default=str(PROJECT_ROOT / "test_env" / "security" / "vulnerability_remediation_report.json"),
        help="Structured vulnerability remediation report path.",
    )
    parser.add_argument(
        "--vulnerability-exception-report",
        default=str(PROJECT_ROOT / "test_env" / "security" / "vulnerability_exception_report.json"),
        help="Structured vulnerability exception report path.",
    )
    parser.add_argument(
        "--output",
        default=str(PROJECT_ROOT / "test_env" / "security" / "security_posture_report.json"),
        help="Output path for the generated security posture report.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    payload = build_security_posture_report(
        project_root=args.project_root,
        sbom_path=args.sbom,
        python_vuln_report_path=args.python_vuln_report,
        container_vuln_report_path=args.container_vuln_report,
        backup_restore_rehearsal_report_path=args.backup_restore_report,
        vulnerability_remediation_report_path=args.vulnerability_remediation_report,
        vulnerability_exception_report_path=args.vulnerability_exception_report,
    )
    output_path = write_security_posture_report(payload, args.output)

    print(f"security_posture_report_written={output_path}")
    print(f"security_posture_status={payload['posture_status']}")
    print(f"security_posture_missing_docs={payload['missing_documents']}")
    print(
        "security_posture_missing_vuln_reports="
        f"{payload['missing_vulnerability_reports']}"
    )
    print(
        "security_posture_missing_backup_restore_reports="
        f"{payload['missing_backup_restore_rehearsal_reports']}"
    )
    print(
        "security_posture_unresolved_findings="
        f"{payload.get('unresolved_vulnerability_findings', 0)}"
    )
    exception_report = payload.get("vulnerability_exception_report", {})
    if isinstance(exception_report, dict):
        print(
            "security_posture_stale_exceptions="
            f"{exception_report.get('stale_exception_count', 0)}"
        )
        print(
            "security_posture_exception_review_due="
            f"{exception_report.get('review_due_exception_count', 0)}"
        )
        print(
            "security_posture_exception_review_status="
            f"{exception_report.get('review_status') or ''}"
        )
        print(
            "security_posture_exception_next_expiry="
            f"{exception_report.get('next_exception_expiry') or ''}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
