"""Build structured extension execution evidence reports."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


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
    build_customer_external_bindings_confirmation_report,
    build_customer_delivery_surface,
    build_extension_execution_evidence,
    build_extension_execution_plan,
    build_release_evidence_report,
    default_extension_execution_actuals_artifact_path,
    default_extension_execution_evidence_reports,
    write_release_evidence_report,
)
from agi_walker.core.api.security_posture_contracts import (  # noqa: E402
    validate_vulnerability_exception_report,
)


def _resolve_project_path(path: str | Path, project_root: Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return project_root / candidate


def _resolve_git_commit(source_root: Path) -> str | None:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=str(source_root),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.returncode != 0:
        return None
    value = result.stdout.strip()
    return value or None


def _actionable_profiles(plan: dict[str, Any]) -> list[dict[str, Any]]:
    profiles = plan.get("profiles")
    if not isinstance(profiles, list):
        return []
    return [
        dict(item)
        for item in profiles
        if isinstance(item, dict) and item.get("actionable") is True
    ]


def _artifact_paths(
    profiles: list[dict[str, Any]],
    list_name: str,
    artifact_key: str,
) -> tuple[int, list[str]]:
    total = 0
    paths: list[str] = []
    for profile in profiles:
        template = profile.get("execution_template", {})
        if not isinstance(template, dict):
            continue
        items = template.get(list_name)
        if not isinstance(items, list):
            continue
        for item in items:
            if not isinstance(item, dict):
                continue
            total += 1
            value = item.get(artifact_key)
            if isinstance(value, str) and value.strip():
                paths.append(value.strip())
    return total, list(dict.fromkeys(paths))


def _load_exception_metrics(path: Path) -> tuple[bool, dict[str, Any], str | None]:
    if not path.is_file():
        return False, {}, f"vulnerability exception report is missing: {path}"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return False, {}, f"vulnerability exception report could not be parsed: {exc}"
    errors = validate_vulnerability_exception_report(payload)
    if errors:
        return False, {}, "; ".join(errors)
    exceptions = payload.get("exceptions", [])
    next_expiry = None
    if isinstance(exceptions, list):
        expiries = sorted(
            str(item.get("expires_at")).strip()
            for item in exceptions
            if isinstance(item, dict)
            and isinstance(item.get("expires_at"), str)
            and item.get("expires_at").strip()
        )
        if expiries:
            next_expiry = expiries[0]
    metrics = {
        "active_exception_count": int(payload.get("active_exception_count") or 0),
        "expired_exception_count": int(payload.get("expired_exception_count") or 0),
        "next_exception_expiry": next_expiry,
    }
    return True, metrics, None


def build_extension_execution_evidence_reports(
    *,
    project_root: Path,
    output_root: Path,
    source_root: Path,
    vulnerability_exception_report_path: Path,
    actuals_artifact_path: Path,
    generated_at: str | None = None,
) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
    customer_delivery_surface = build_customer_delivery_surface(project_root=project_root)
    extension_support_surface = dict(customer_delivery_surface.get("extension_support_surface", {}))
    plan = build_extension_execution_plan(extension_support_surface)
    actionable_profiles = _actionable_profiles(plan)
    actionable_count = len(actionable_profiles)
    source_commit_sha = _resolve_git_commit(source_root)
    started_at = generated_at or datetime.now().isoformat()
    command = (
        "python tools/build_extension_execution_evidence.py "
        f"--project-root {project_root} --output-root {output_root} "
        f"--vulnerability-exception-report {vulnerability_exception_report_path} "
        f"--actuals-artifact {actuals_artifact_path}"
    )

    on_call_total, on_call_paths = _artifact_paths(
        actionable_profiles,
        "on_call_handoff_records",
        "required_artifact",
    )
    exception_total, exception_paths = _artifact_paths(
        actionable_profiles,
        "exception_review_steps",
        "required_artifact",
    )
    closure_total, closure_paths = _artifact_paths(
        actionable_profiles,
        "escalation_closure_steps",
        "required_artifact",
    )
    exception_report_ready, exception_metrics, exception_error = _load_exception_metrics(
        vulnerability_exception_report_path
    )

    payloads: dict[str, dict[str, Any]] = {}

    if plan.get("status") != "ready":
        on_call_status = "blocked"
        on_call_summary = (
            "extension_on_call_rehearsal evidence blocked: extension_execution_plan is not ready."
        )
    elif actionable_count == 0:
        on_call_status = "passed"
        on_call_summary = (
            "extension_on_call_rehearsal evidence passed: no actionable extension profiles declared."
        )
    elif on_call_total == 0:
        on_call_status = "blocked"
        on_call_summary = (
            "extension_on_call_rehearsal evidence blocked: no on-call handoff records were declared."
        )
    else:
        on_call_status = "passed"
        on_call_summary = (
            "extension_on_call_rehearsal evidence passed: "
            f"{actionable_count} actionable profiles, "
            f"{on_call_total} handoff record(s), "
            f"{len(on_call_paths)} unique artifact path(s)."
        )
    payloads["extension_on_call_rehearsal"] = build_release_evidence_report(
        evidence_name="extension_on_call_rehearsal",
        status=on_call_status,
        summary=on_call_summary,
        command=command,
        generated_at=started_at,
        metrics={
            "declared_profiles": int(plan.get("declared_profiles") or 0),
            "actionable_profiles": actionable_count,
            "handoff_records": on_call_total,
            "unique_artifact_paths": len(on_call_paths),
            "artifact_paths": on_call_paths,
        },
        source_commit_sha=source_commit_sha,
    )

    if plan.get("status") != "ready":
        exception_status = "blocked"
        exception_summary = (
            "extension_exception_review_schedule evidence blocked: extension_execution_plan is not ready."
        )
    elif actionable_count == 0:
        exception_status = "passed"
        exception_summary = (
            "extension_exception_review_schedule evidence passed: no actionable extension profiles declared."
        )
    elif exception_total == 0:
        exception_status = "blocked"
        exception_summary = (
            "extension_exception_review_schedule evidence blocked: no exception review steps were declared."
        )
    elif not exception_report_ready:
        exception_status = "blocked"
        exception_summary = (
            "extension_exception_review_schedule evidence blocked: "
            + (exception_error or "vulnerability exception report is unavailable.")
        )
    else:
        exception_status = "passed"
        next_expiry = exception_metrics.get("next_exception_expiry")
        exception_summary = (
            "extension_exception_review_schedule evidence passed: "
            f"{actionable_count} actionable profiles, "
            f"{exception_total} review step(s), "
            f"active_exceptions={exception_metrics.get('active_exception_count', 0)}"
            + (
                f", next_expiry={next_expiry}."
                if isinstance(next_expiry, str) and next_expiry
                else "."
            )
        )
    payloads["extension_exception_review_schedule"] = build_release_evidence_report(
        evidence_name="extension_exception_review_schedule",
        status=exception_status,
        summary=exception_summary,
        command=command,
        generated_at=started_at,
        metrics={
            "declared_profiles": int(plan.get("declared_profiles") or 0),
            "actionable_profiles": actionable_count,
            "review_steps": exception_total,
            "unique_artifact_paths": len(exception_paths),
            "artifact_paths": exception_paths,
            **exception_metrics,
        },
        source_commit_sha=source_commit_sha,
    )

    if plan.get("status") != "ready":
        closure_status = "blocked"
        closure_summary = (
            "extension_escalation_closure evidence blocked: extension_execution_plan is not ready."
        )
    elif actionable_count == 0:
        closure_status = "passed"
        closure_summary = (
            "extension_escalation_closure evidence passed: no actionable extension profiles declared."
        )
    elif closure_total == 0:
        closure_status = "blocked"
        closure_summary = (
            "extension_escalation_closure evidence blocked: no escalation closure steps were declared."
        )
    else:
        closure_status = "passed"
        closure_summary = (
            "extension_escalation_closure evidence passed: "
            f"{actionable_count} actionable profiles, "
            f"{closure_total} closure step(s), "
            f"{len(closure_paths)} unique artifact path(s)."
        )
    payloads["extension_escalation_closure"] = build_release_evidence_report(
        evidence_name="extension_escalation_closure",
        status=closure_status,
        summary=closure_summary,
        command=command,
        generated_at=started_at,
        metrics={
            "declared_profiles": int(plan.get("declared_profiles") or 0),
            "actionable_profiles": actionable_count,
            "closure_steps": closure_total,
            "unique_artifact_paths": len(closure_paths),
            "artifact_paths": closure_paths,
        },
        source_commit_sha=source_commit_sha,
    )
    payloads["customer_external_bindings_confirmation"] = (
        build_customer_external_bindings_confirmation_report(
            project_root=project_root,
            actuals_artifact_path=str(actuals_artifact_path),
            output_path=str(
                output_root / "customer_external_bindings_confirmation_report.json"
            ),
            generated_at=started_at,
        )
    )

    evidence_surface = build_extension_execution_evidence(
        project_root=project_root,
        extension_support_surface=extension_support_surface,
        reports=[
            {
                "name": name,
                "path": str(output_root / next(
                    item["path"].split("/")[-1]
                    for item in default_extension_execution_evidence_reports()
                    if item["name"] == name
                )),
                "required": False,
                "exists": True,
                "status": payload["status"],
                "summary": payload["summary"],
                "artifact_type": payload["artifact_type"],
                "metrics": payload["metrics"],
            }
            for name, payload in payloads.items()
        ],
    )
    return payloads, evidence_surface


def write_extension_execution_evidence_reports(
    *,
    project_root: Path,
    output_root: Path,
    payloads: dict[str, dict[str, Any]],
) -> dict[str, Path]:
    report_paths = {
        str(item["name"]): _resolve_project_path(item["path"], project_root)
        for item in default_extension_execution_evidence_reports()
    }
    written: dict[str, Path] = {}
    for name, payload in payloads.items():
        path = report_paths.get(name, output_root / f"{name}_report.json")
        written[name] = write_release_evidence_report(payload, path)
    return written


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build structured extension execution evidence reports."
    )
    parser.add_argument(
        "--project-root",
        default=str(PROJECT_ROOT),
        help="Project root used to resolve customer delivery docs and artifact paths.",
    )
    parser.add_argument(
        "--source-root",
        default=str(PROJECT_ROOT),
        help="Repository root used to resolve Git commit metadata.",
    )
    parser.add_argument(
        "--output-root",
        default="test_env/release_evidence/operations",
        help="Output directory for the generated reports.",
    )
    parser.add_argument(
        "--vulnerability-exception-report",
        default="test_env/release_evidence/security/vulnerability_exception_report.json",
        help="Structured vulnerability exception report used to derive review schedule metrics.",
    )
    parser.add_argument(
        "--actuals-artifact",
        default=default_extension_execution_actuals_artifact_path(),
        help="Structured extension_execution_actuals artifact used to attest customer external bindings confirmation.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    project_root = Path(args.project_root)
    source_root = Path(args.source_root)
    output_root = _resolve_project_path(args.output_root, project_root)
    vulnerability_exception_report = _resolve_project_path(
        args.vulnerability_exception_report,
        project_root,
    )
    actuals_artifact = _resolve_project_path(args.actuals_artifact, project_root)

    payloads, evidence_surface = build_extension_execution_evidence_reports(
        project_root=project_root,
        output_root=output_root,
        source_root=source_root,
        vulnerability_exception_report_path=vulnerability_exception_report,
        actuals_artifact_path=actuals_artifact,
    )
    written_reports = write_extension_execution_evidence_reports(
        project_root=project_root,
        output_root=output_root,
        payloads=payloads,
    )

    for name, path in written_reports.items():
        print(f"extension_execution_report_written[{name}]={path}")
    print(
        "extension_execution_evidence_reports="
        f"{evidence_surface['ready_reports']}/{evidence_surface['required_reports']}"
    )
    print(
        "extension_execution_evidence_status="
        f"{evidence_surface['status']}"
    )
    return 0 if evidence_surface["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
