#!/usr/bin/env python
"""Run the formal security release preflight and emit a structured evidence report."""

from __future__ import annotations

import argparse
import json
import subprocess
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
    build_release_evidence_report,
    validate_release_evidence_report,
    write_release_evidence_report,
)
from agi_walker.core.api.security_posture_contracts import (  # noqa: E402
    validate_vulnerability_exception_burndown_report,
    validate_security_posture_report,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the formal security release preflight and emit a structured evidence report."
    )
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "release_evidence"),
        help="Output root passed to tools/collect_release_evidence.py.",
    )
    parser.add_argument(
        "--report-file",
        default=None,
        help="Optional output path for the structured security release preflight evidence report.",
    )
    parser.add_argument(
        "--security-posture-report",
        default=None,
        help="Optional explicit security_posture_report.json path. Defaults to <output-root>/security/security_posture_report.json.",
    )
    parser.add_argument(
        "--vulnerability-exception-review-report",
        default=None,
        help="Optional explicit vulnerability_exception_review_report.json path. Defaults to <output-root>/security/vulnerability_exception_review_report.json.",
    )
    parser.add_argument(
        "--vulnerability-exception-burndown-report",
        default=None,
        help="Optional explicit vulnerability_exception_burndown_report.json path. Defaults to <output-root>/security/vulnerability_exception_burndown_report.json.",
    )
    parser.add_argument(
        "--skip-collect",
        action="store_true",
        help="Do not execute collect_release_evidence.py; only evaluate the existing security posture report.",
    )
    parser.add_argument(
        "--security-only",
        action="store_true",
        help=(
            "Pass --security-only to collect_release_evidence.py so the preflight "
            "collects only security posture evidence and skips broad release gates."
        ),
    )
    parser.add_argument("--python-vuln-report-source", default=None)
    parser.add_argument("--python-vuln-raw-report", default=None)
    parser.add_argument(
        "--python-vuln-raw-format",
        default="pip-audit-json",
        choices=["pip-audit-json", "trivy-json"],
    )
    parser.add_argument(
        "--python-vuln-command",
        default="pip-audit --format json",
    )
    parser.add_argument("--run-python-vuln-scan", action="store_true")
    parser.add_argument("--python-vuln-raw-output", default=None)
    parser.add_argument("--container-vuln-report-source", default=None)
    parser.add_argument("--container-vuln-raw-report", default=None)
    parser.add_argument(
        "--container-vuln-raw-format",
        default="trivy-json",
        choices=["pip-audit-json", "trivy-json"],
    )
    parser.add_argument(
        "--container-vuln-command",
        default="trivy image --scanners vuln --timeout 15m --format json",
    )
    parser.add_argument("--run-container-vuln-scan", action="store_true")
    parser.add_argument("--container-image-ref", action="append", default=[])
    parser.add_argument("--container-vuln-raw-output-dir", default=None)
    parser.add_argument("--vulnerability-exception-report-source", default=None)
    parser.add_argument(
        "--vulnerability-exception-input-source",
        default=str(
            PROJECT_ROOT
            / "deployment"
            / "security"
            / "vulnerability_exceptions.input.json"
        ),
    )
    return parser


def _resolve_report_path(args: argparse.Namespace, output_root: Path) -> Path:
    if args.security_posture_report:
        return Path(args.security_posture_report)
    return output_root / "security" / "security_posture_report.json"


def _resolve_exception_review_report_path(
    args: argparse.Namespace, output_root: Path
) -> Path:
    if args.vulnerability_exception_review_report:
        return Path(args.vulnerability_exception_review_report)
    if args.security_posture_report:
        return Path(args.security_posture_report).parent / (
            "vulnerability_exception_review_report.json"
        )
    return output_root / "security" / "vulnerability_exception_review_report.json"


def _resolve_exception_burndown_report_path(
    args: argparse.Namespace, output_root: Path
) -> Path:
    if args.vulnerability_exception_burndown_report:
        return Path(args.vulnerability_exception_burndown_report)
    if args.security_posture_report:
        return Path(args.security_posture_report).parent / (
            "vulnerability_exception_burndown_report.json"
        )
    return output_root / "security" / "vulnerability_exception_burndown_report.json"


def _resolve_evidence_report_path(args: argparse.Namespace, output_root: Path) -> Path:
    if args.report_file:
        return Path(args.report_file)
    return output_root / "security_release_preflight_report.json"


def _run_collect_release_evidence(args: argparse.Namespace, output_root: Path) -> tuple[int, str]:
    command: list[str] = [
        sys.executable,
        "tools/collect_release_evidence.py",
        "--output-root",
        str(output_root),
    ]
    option_pairs = [
        ("--python-vuln-report-source", args.python_vuln_report_source),
        ("--python-vuln-raw-report", args.python_vuln_raw_report),
        ("--python-vuln-raw-format", args.python_vuln_raw_format),
        ("--python-vuln-command", args.python_vuln_command),
        ("--python-vuln-raw-output", args.python_vuln_raw_output),
        ("--container-vuln-report-source", args.container_vuln_report_source),
        ("--container-vuln-raw-report", args.container_vuln_raw_report),
        ("--container-vuln-raw-format", args.container_vuln_raw_format),
        ("--container-vuln-command", args.container_vuln_command),
        ("--container-vuln-raw-output-dir", args.container_vuln_raw_output_dir),
        (
            "--vulnerability-exception-report-source",
            args.vulnerability_exception_report_source,
        ),
        (
            "--vulnerability-exception-input-source",
            args.vulnerability_exception_input_source,
        ),
    ]
    for option, value in option_pairs:
        if value is not None:
            command.extend([option, value])
    if args.run_python_vuln_scan:
        command.append("--run-python-vuln-scan")
    if args.run_container_vuln_scan:
        command.append("--run-container-vuln-scan")
    if args.security_only:
        command.append("--security-only")
    for image_ref in args.container_image_ref:
        command.extend(["--container-image-ref", image_ref])

    result = subprocess.run(
        command,
        cwd=str(PROJECT_ROOT),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    combined_output = "\n".join(
        part for part in [result.stdout.strip(), result.stderr.strip()] if part
    )
    return result.returncode, combined_output


def _load_security_posture_report(path: Path) -> tuple[str, str, dict[str, object]]:
    if not path.is_file():
        return "blocked", f"security_posture_report is missing: {path}", {}

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return "blocked", f"security_posture_report is unreadable: {exc}", {}

    errors = validate_security_posture_report(payload)
    if errors:
        return "blocked", f"security_posture_report is invalid: {'; '.join(errors)}", {}

    posture_status = payload.get("posture_status")
    summary = str(payload.get("summary") or "").strip()
    exception_report = payload.get("vulnerability_exception_report", {})
    vulnerability_reports = payload.get("vulnerability_reports", [])
    if not isinstance(vulnerability_reports, list):
        vulnerability_reports = []
    blocked_report_names = [
        str(item.get("name"))
        for item in vulnerability_reports
        if isinstance(item, dict)
        and item.get("required") is True
        and item.get("status") == "blocked"
        and item.get("name")
    ]
    metrics: dict[str, object] = {
        "missing_vulnerability_reports": payload.get(
            "missing_vulnerability_reports", 0
        ),
        "blocked_vulnerability_reports": payload.get(
            "blocked_vulnerability_reports", 0
        ),
        "blocked_vulnerability_execution_reports": payload.get(
            "blocked_vulnerability_execution_reports", 0
        ),
        "blocked_vulnerability_finding_reports": payload.get(
            "blocked_vulnerability_finding_reports", 0
        ),
        "blocked_vulnerability_report_names": blocked_report_names,
        "accepted_vulnerability_findings": payload.get(
            "accepted_vulnerability_findings", 0
        ),
        "unresolved_vulnerability_findings": payload.get(
            "unresolved_vulnerability_findings", 0
        ),
    }
    if isinstance(exception_report, dict):
        metrics.update(
            {
                "active_vulnerability_exceptions": exception_report.get(
                    "active_exception_count", 0
                ),
                "expired_vulnerability_exceptions": exception_report.get(
                    "expired_exception_count", 0
                ),
                "stale_vulnerability_exceptions": exception_report.get(
                    "stale_exception_count", 0
                ),
                "stale_vulnerability_exception_ids": exception_report.get(
                    "stale_exception_ids", []
                ),
                "vulnerability_exception_review_due": exception_report.get(
                    "review_due_exception_count", 0
                ),
                "review_due_vulnerability_exception_ids": exception_report.get(
                    "review_due_exception_ids",
                    [],
                ),
                "review_due_vulnerability_exception_tickets": exception_report.get(
                    "review_due_exception_tickets",
                    [],
                ),
                "vulnerability_exception_review_status": exception_report.get(
                    "review_status"
                ),
                "vulnerability_exception_next_expiry": exception_report.get(
                    "next_exception_expiry"
                ),
                "expired_vulnerability_exception_ids": exception_report.get(
                    "expired_exception_ids",
                    [],
                ),
            }
        )
    if posture_status == "ready":
        return "passed", summary or "security posture is ready.", metrics
    return "blocked", summary or "security posture remains blocked.", metrics


def _load_vulnerability_exception_review_report(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {}

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {
            "vulnerability_exception_review_report_path": str(path),
            "vulnerability_exception_review_report_status": "invalid",
            "vulnerability_exception_review_report_summary": (
                f"vulnerability exception review report is unreadable: {exc}"
            ),
        }

    errors = validate_release_evidence_report(payload)
    if payload.get("evidence_name") != "vulnerability_exception_review":
        errors.append(
            "evidence_name must be 'vulnerability_exception_review'"
        )
    if errors:
        return {
            "vulnerability_exception_review_report_path": str(path),
            "vulnerability_exception_review_report_status": "invalid",
            "vulnerability_exception_review_report_summary": "; ".join(errors),
        }

    metrics = payload.get("metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
    return {
        "vulnerability_exception_review_report_path": str(path),
        "vulnerability_exception_review_report_status": payload.get("status"),
        "vulnerability_exception_review_report_summary": payload.get("summary"),
        "vulnerability_exception_review_follow_up_required": metrics.get(
            "review_follow_up_required"
        ),
        "review_due_vulnerability_exception_ids": metrics.get(
            "review_due_exception_ids",
            [],
        ),
        "review_due_vulnerability_exception_tickets": metrics.get(
            "review_due_exception_tickets",
            [],
        ),
        "expired_vulnerability_exception_ids": metrics.get(
            "expired_exception_ids",
            [],
        ),
        "vulnerability_exception_review_candidate_count": metrics.get(
            "review_candidate_count",
            0,
        ),
    }


def _load_vulnerability_exception_burndown_report(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {
            "vulnerability_exception_burndown_report_path": str(path),
            "vulnerability_exception_burndown_report_exists": False,
        }

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {
            "vulnerability_exception_burndown_report_path": str(path),
            "vulnerability_exception_burndown_report_exists": True,
            "vulnerability_exception_burndown_report_status": "invalid",
            "vulnerability_exception_burndown_report_summary": (
                f"vulnerability exception burn-down report is unreadable: {exc}"
            ),
        }

    errors = validate_vulnerability_exception_burndown_report(payload)
    if errors:
        return {
            "vulnerability_exception_burndown_report_path": str(path),
            "vulnerability_exception_burndown_report_exists": True,
            "vulnerability_exception_burndown_report_status": "invalid",
            "vulnerability_exception_burndown_report_summary": "; ".join(errors),
        }

    return {
        "vulnerability_exception_burndown_report_path": str(path),
        "vulnerability_exception_burndown_report_exists": True,
        "vulnerability_exception_burndown_report_status": payload.get("status"),
        "vulnerability_exception_burndown_report_summary": payload.get("summary"),
        "vulnerability_exception_burndown_active": payload.get(
            "active_exception_count", 0
        ),
        "vulnerability_exception_burndown_review_due": payload.get(
            "review_due_exception_count", 0
        ),
        "vulnerability_exception_burndown_expired": payload.get(
            "expired_exception_count", 0
        ),
        "vulnerability_exception_burndown_next_expiry": payload.get(
            "next_exception_expiry"
        ),
    }


def _coerce_non_negative_int(value: object) -> int:
    if isinstance(value, bool):
        return 0
    if isinstance(value, int) and value >= 0:
        return value
    return 0


def _review_gate_blocker(
    posture_metrics: dict[str, object],
    review_report_metrics: dict[str, object],
) -> str:
    if not review_report_metrics:
        return (
            "security release preflight failed because "
            "vulnerability_exception_review_report.json is missing."
        )

    review_report_status = review_report_metrics.get(
        "vulnerability_exception_review_report_status"
    )
    if review_report_status != "passed":
        return (
            "security release preflight failed because "
            "vulnerability_exception_review_report status is "
            f"{review_report_status or 'missing'}."
        )

    posture_review_due = _coerce_non_negative_int(
        posture_metrics.get("vulnerability_exception_review_due")
    )
    review_candidate_count = _coerce_non_negative_int(
        review_report_metrics.get("vulnerability_exception_review_candidate_count")
    )
    follow_up_required = bool(
        review_report_metrics.get(
            "vulnerability_exception_review_follow_up_required"
        )
    )
    if posture_review_due or review_candidate_count or follow_up_required:
        return (
            "security release preflight failed because vulnerability exceptions "
            "require review before release: "
            f"review_due={posture_review_due}, "
            f"review_candidates={review_candidate_count}."
        )

    return ""


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    security_posture_report_path = _resolve_report_path(args, output_root)
    vulnerability_exception_review_report_path = _resolve_exception_review_report_path(
        args,
        output_root,
    )
    vulnerability_exception_burndown_report_path = (
        _resolve_exception_burndown_report_path(args, output_root)
    )
    evidence_report_path = _resolve_evidence_report_path(args, output_root)

    collect_exit_code = 0
    collect_summary = "collect_release_evidence was not executed."
    if not args.skip_collect:
        collect_exit_code, collect_summary = _run_collect_release_evidence(args, output_root)

    status, summary, posture_metrics = _load_security_posture_report(
        security_posture_report_path
    )
    review_report_metrics = _load_vulnerability_exception_review_report(
        vulnerability_exception_review_report_path
    )
    burndown_report_metrics = _load_vulnerability_exception_burndown_report(
        vulnerability_exception_burndown_report_path
    )
    if collect_exit_code != 0:
        status = "blocked"
        summary = (
            "security release preflight failed because collect_release_evidence returned a non-zero exit code. "
            + collect_summary
        )
    elif status == "passed":
        review_blocker = _review_gate_blocker(posture_metrics, review_report_metrics)
        if review_blocker:
            status = "blocked"
            summary = review_blocker

    payload = build_release_evidence_report(
        evidence_name="security_release_preflight",
        status=status,
        summary=summary,
        command="python tools/run_security_release_preflight.py",
        metrics={
            "collect_exit_code": collect_exit_code,
            "security_posture_report_path": str(security_posture_report_path),
            **posture_metrics,
            **review_report_metrics,
            **burndown_report_metrics,
        },
    )
    written_report = write_release_evidence_report(payload, evidence_report_path)
    print(f"security_release_preflight_written={written_report}")
    print(f"security_release_preflight_status={payload['status']}")
    print(f"security_release_preflight_summary={payload['summary']}")
    if posture_metrics:
        print(
            "security_release_preflight_missing_vulnerability_reports="
            f"{posture_metrics.get('missing_vulnerability_reports', 0)}"
        )
        print(
            "security_release_preflight_blocked_vulnerability_reports="
            f"{posture_metrics.get('blocked_vulnerability_reports', 0)}"
        )
        print(
            "security_release_preflight_blocked_vulnerability_execution_reports="
            f"{posture_metrics.get('blocked_vulnerability_execution_reports', 0)}"
        )
        print(
            "security_release_preflight_blocked_vulnerability_finding_reports="
            f"{posture_metrics.get('blocked_vulnerability_finding_reports', 0)}"
        )
        blocked_report_names = posture_metrics.get(
            "blocked_vulnerability_report_names", []
        )
        if isinstance(blocked_report_names, list):
            print(
                "security_release_preflight_blocked_vulnerability_report_names="
                f"{','.join(str(item) for item in blocked_report_names)}"
            )
        print(
            "security_release_preflight_stale_exceptions="
            f"{posture_metrics.get('stale_vulnerability_exceptions', 0)}"
        )
        print(
            "security_release_preflight_exception_review_status="
            f"{posture_metrics.get('vulnerability_exception_review_status') or ''}"
        )
        print(
            "security_release_preflight_exception_next_expiry="
            f"{posture_metrics.get('vulnerability_exception_next_expiry') or ''}"
        )
    if review_report_metrics:
        print(
            "security_release_preflight_review_report_status="
            f"{review_report_metrics.get('vulnerability_exception_review_report_status') or ''}"
        )
        print(
            "security_release_preflight_review_candidates="
            f"{review_report_metrics.get('vulnerability_exception_review_candidate_count', 0)}"
        )
    if burndown_report_metrics.get("vulnerability_exception_burndown_report_exists"):
        print(
            "security_release_preflight_burndown_status="
            f"{burndown_report_metrics.get('vulnerability_exception_burndown_report_status') or ''}"
        )
        print(
            "security_release_preflight_burndown_active="
            f"{burndown_report_metrics.get('vulnerability_exception_burndown_active', 0)}"
        )
    return 0 if payload["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
