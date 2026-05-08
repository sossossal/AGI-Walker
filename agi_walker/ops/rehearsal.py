"""Deterministic orchestration for stable release rehearsal."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from uuid import uuid4


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (
            candidate / "agi_walker"
        ).exists():
            return candidate
    return current.parent


PROJECT_ROOT = _find_repo_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agi_walker.core.api.release_contracts import (  # noqa: E402
    EXTENSION_EXTERNAL_BINDING_SECTION_IDS,
    build_external_mainline_execution_plan_artifact,
    build_external_mainline_input_checklist_report,
    build_run_customer_external_bindings_closure_command,
    build_industrial_delivery_rehearsal_report_artifact,
    build_extension_execution_instance_artifact,
    build_extension_execution_schedule_artifact,
    build_extension_execution_plan,
    build_release_evidence_report,
    build_vulnerability_exception_review_report,
    default_customer_external_bindings_closure_report_path,
    default_customer_external_bindings_confirmation_report_path,
    default_external_mainline_execution_plan_path,
    default_external_mainline_input_checklist_report_path,
    default_release_ops_execution_report_path,
    validate_release_manifest_artifact,
    write_external_mainline_execution_plan_artifact,
    write_industrial_delivery_rehearsal_report_artifact,
    write_extension_execution_instance_artifact,
    write_extension_execution_schedule_artifact,
    write_release_evidence_report,
)
from agi_walker.core.api.security_posture_contracts import (  # noqa: E402
    build_backup_restore_rehearsal_report,
    build_sbom_artifact,
    build_security_posture_report,
    build_vulnerability_exception_report,
    build_vulnerability_remediation_report,
    build_vulnerability_scan_report,
    write_backup_restore_rehearsal_report,
    write_sbom_artifact,
    write_security_posture_report,
    write_vulnerability_exception_report,
    write_vulnerability_remediation_report,
    write_vulnerability_scan_report,
)
from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    ReleaseRehearsalRequest,
    ReleaseRehearsalResult,
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _path_for_contract(path: str | Path, *, project_root: Path) -> str:
    resolved_path = Path(path).resolve()
    resolved_project_root = project_root.resolve()
    try:
        return str(resolved_path.relative_to(resolved_project_root))
    except ValueError:
        return str(resolved_path)


def _resolve_source_path(path: str | Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return PROJECT_ROOT / candidate


def _load_repo_tool_module(module_name: str, relative_path: str):
    module_path = (PROJECT_ROOT / relative_path).resolve()
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"could not load tool module from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run_command(command: list[str], *, cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )


def _run_checked_command(
    command: list[str],
    *,
    cwd: Path,
    failure_message: str,
) -> tuple[str, str]:
    result = _run_command(command, cwd=cwd)
    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    if result.returncode != 0:
        raise RuntimeError(stderr or stdout or failure_message)
    return stdout, stderr


def _load_json_payload(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _summarize_security_release_preflight(
    payload: dict[str, Any],
    *,
    report_path: Path,
) -> dict[str, Any]:
    return {
        "status": payload.get("status"),
        "summary": payload.get("summary"),
        "metrics": payload.get("metrics", {}),
        "report_path": str(report_path),
    }


def _summarize_vulnerability_exception_review(
    payload: dict[str, Any],
    *,
    report_path: Path,
) -> dict[str, Any]:
    metrics = payload.get("metrics", {})
    review_candidate_count = (
        metrics.get("review_candidate_count") if isinstance(metrics, dict) else None
    )
    return {
        "status": payload.get("status"),
        "summary": payload.get("summary"),
        "review_candidate_count": review_candidate_count,
        "report_path": str(report_path),
    }


def _summarize_industrial_manifest(
    payload: dict[str, Any],
    *,
    manifest_path: Path,
) -> dict[str, Any]:
    return {
        "status": payload.get("release_gate_status"),
        "customer_delivery_status": payload.get("customer_delivery_surface", {}).get(
            "status"
        ),
        "industrial_delivery_status": payload.get("industrial_delivery_gate", {}).get(
            "status"
        ),
        "manifest_path": str(manifest_path),
    }


def _summarize_industrial_release_readiness(
    payload: dict[str, Any],
    *,
    report_path: Path,
) -> dict[str, Any]:
    return {
        "status": payload.get("industrial_release_gate"),
        "summary": payload.get("summary"),
        "next_step_count": len(payload.get("next_step_plan", [])),
        "report_path": str(report_path),
    }


def _summarize_industrial_promotion_checklist(
    payload: dict[str, Any],
    *,
    report_path: Path,
) -> dict[str, Any]:
    return {
        "status": "ready" if payload.get("ready_to_promote") else "blocked",
        "summary": payload.get("summary"),
        "blocking_steps": payload.get("blocking_steps"),
        "report_path": str(report_path),
    }


def _resolve_customer_acceptance_report(
    payload: dict[str, Any],
    *,
    report_name: str,
) -> dict[str, Any]:
    inline_report = payload.get(report_name)
    if isinstance(inline_report, dict):
        return dict(inline_report)

    acceptance_reports = payload.get("acceptance_reports", [])
    for item in acceptance_reports:
        if isinstance(item, dict) and item.get("name") == report_name:
            return dict(item)
    return {}


def _summarize_bundle_vulnerability_exception_review(
    payload: dict[str, Any],
    *,
    bundle_path: Path,
) -> dict[str, Any]:
    report = _resolve_customer_acceptance_report(
        payload,
        report_name="vulnerability_exception_review",
    )
    if not report:
        return {}

    summary: dict[str, Any] = {
        "status": report.get("status"),
        "summary": report.get("summary"),
        "bundle_path": str(bundle_path),
    }
    report_path = (
        report.get("resolved_report_path")
        or report.get("report_path")
        or report.get("path")
    )
    if isinstance(report_path, str) and report_path.strip():
        summary["report_path"] = report_path.strip()
    metrics = report.get("metrics")
    if isinstance(metrics, dict):
        review_candidate_count = metrics.get("review_candidate_count")
        if not isinstance(review_candidate_count, int) or review_candidate_count < 0:
            review_candidate_count = metrics.get("review_due_exception_count")
        if isinstance(review_candidate_count, int) and review_candidate_count >= 0:
            summary["review_candidate_count"] = review_candidate_count
    return summary


def _summarize_external_mainline_input_checklist(
    payload: dict[str, Any],
    *,
    bundle_path: Path,
) -> dict[str, Any]:
    report = _resolve_customer_acceptance_report(
        payload,
        report_name="external_mainline_input_checklist",
    )
    if not report:
        return {}

    summary: dict[str, Any] = {
        "status": report.get("status"),
        "summary": report.get("summary"),
        "bundle_path": str(bundle_path),
    }
    report_path = (
        report.get("resolved_report_path")
        or report.get("report_path")
        or report.get("path")
    )
    if isinstance(report_path, str) and report_path.strip():
        summary["report_path"] = report_path.strip()
    control_plane_session = report.get("control_plane_session")
    if isinstance(control_plane_session, dict):
        summary["control_plane_session"] = dict(control_plane_session)
    control_plane_event_stream = report.get("control_plane_event_stream")
    if isinstance(control_plane_event_stream, dict):
        summary["control_plane_event_stream"] = dict(control_plane_event_stream)
    metrics = report.get("metrics")
    if isinstance(metrics, dict):
        missing_input_count = metrics.get("missing_input_count")
        if isinstance(missing_input_count, int) and missing_input_count >= 0:
            summary["missing_input_count"] = missing_input_count
        for field in [
            "waiting_external_input_steps",
            "ready_to_run_steps",
            "completed_steps",
        ]:
            value = metrics.get(field)
            if isinstance(value, list):
                summary[field] = [
                    str(item).strip()
                    for item in value
                    if isinstance(item, str) and item.strip()
                ]
    return summary


def _aggregate_control_plane_surface(
    *components: Mapping[str, Any] | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    session: dict[str, Any] = {}
    event_stream: dict[str, Any] = {}
    for component in components:
        if not isinstance(component, Mapping):
            continue
        if not session:
            candidate_session = component.get("control_plane_session")
            if isinstance(candidate_session, dict) and candidate_session:
                session = dict(candidate_session)
        if not event_stream:
            candidate_event_stream = component.get("control_plane_event_stream")
            if isinstance(candidate_event_stream, dict) and candidate_event_stream:
                event_stream = dict(candidate_event_stream)
        if session and event_stream:
            break
    return session, event_stream


def _summarize_customer_acceptance_bundle(
    payload: dict[str, Any],
    *,
    bundle_path: Path,
) -> dict[str, Any]:
    acceptance_reports = payload.get("acceptance_reports", [])
    present_reports = [
        item
        for item in acceptance_reports
        if isinstance(item, dict) and item.get("exists") is True
    ]
    summary: dict[str, Any] = {
        "status": payload.get("bundle_status"),
        "summary": payload.get("summary"),
        "reports_present": len(present_reports),
        "reports_total": len(acceptance_reports),
        "bundle_path": str(bundle_path),
    }
    vulnerability_exception_review = _summarize_bundle_vulnerability_exception_review(
        payload,
        bundle_path=bundle_path,
    )
    if vulnerability_exception_review:
        summary["vulnerability_exception_review"] = vulnerability_exception_review
    external_mainline_execution_plan = _summarize_external_mainline_execution_plan(
        payload,
        bundle_path=bundle_path,
    )
    if external_mainline_execution_plan.get("status"):
        summary["external_mainline_execution_plan"] = external_mainline_execution_plan
    external_mainline_input_checklist = _summarize_external_mainline_input_checklist(
        payload,
        bundle_path=bundle_path,
    )
    if external_mainline_input_checklist.get("status"):
        summary["external_mainline_input_checklist"] = external_mainline_input_checklist
    release_ops_execution = _summarize_bundle_release_ops_execution(
        payload,
        bundle_path=bundle_path,
    )
    if release_ops_execution.get("status"):
        summary["release_ops_execution"] = release_ops_execution
    return summary


def _summarize_customer_external_bindings_closure(
    payload: dict[str, Any],
    *,
    bundle_path: Path,
) -> dict[str, Any]:
    acceptance_reports = payload.get("acceptance_reports", [])
    for item in acceptance_reports:
        if not isinstance(item, dict):
            continue
        if item.get("name") != "customer_external_bindings_closure":
            continue
        report_path = item.get("resolved_report_path") or item.get("path")
        summary = (
            str(item.get("summary")).strip()
            if isinstance(item.get("summary"), str) and item.get("summary").strip()
            else "customer external bindings closure summary is unavailable."
        )
        status = (
            str(item.get("status")).strip()
            if isinstance(item.get("status"), str) and item.get("status").strip()
            else "blocked"
        )
        payload_summary = {
            "status": status,
            "summary": summary,
            "bundle_path": str(bundle_path),
        }
        if isinstance(report_path, str) and report_path.strip():
            payload_summary["report_path"] = report_path.strip()
        metrics = item.get("metrics")
        if isinstance(metrics, dict):
            failed_steps = metrics.get("failed_steps")
            if isinstance(failed_steps, list):
                payload_summary["failed_steps"] = [
                    str(step).strip()
                    for step in failed_steps
                    if isinstance(step, str) and step.strip()
                ]
        return payload_summary
    return {
        "status": "blocked",
        "summary": (
            "customer external bindings closure is missing from the "
            "industrial customer acceptance bundle."
        ),
        "bundle_path": str(bundle_path),
    }


def _summarize_external_mainline_execution_plan(
    payload: dict[str, Any],
    *,
    bundle_path: Path,
) -> dict[str, Any]:
    item = _resolve_customer_acceptance_report(
        payload,
        report_name="external_mainline_execution_plan",
    )
    if item:
        report_path = (
            item.get("resolved_report_path")
            or item.get("report_path")
            or item.get("path")
        )
        summary = (
            str(item.get("summary")).strip()
            if isinstance(item.get("summary"), str) and item.get("summary").strip()
            else "external mainline execution plan summary is unavailable."
        )
        status = (
            str(item.get("status")).strip()
            if isinstance(item.get("status"), str) and item.get("status").strip()
            else "blocked"
        )
        payload_summary: dict[str, Any] = {
            "status": status,
            "summary": summary,
            "bundle_path": str(bundle_path),
        }
        if isinstance(report_path, str) and report_path.strip():
            payload_summary["report_path"] = report_path.strip()
        for field in [
            "completed_steps",
            "ready_to_run_steps",
            "waiting_external_input_steps",
            "blocked_steps",
        ]:
            value = item.get(field)
            if isinstance(value, int) and value >= 0:
                payload_summary[field] = value
        control_plane_session = item.get("control_plane_session")
        if isinstance(control_plane_session, dict) and control_plane_session:
            payload_summary["control_plane_session"] = dict(control_plane_session)
        control_plane_event_stream = item.get("control_plane_event_stream")
        if isinstance(control_plane_event_stream, dict) and control_plane_event_stream:
            payload_summary["control_plane_event_stream"] = dict(
                control_plane_event_stream
            )
        return payload_summary
    return {
        "status": "blocked",
        "summary": (
            "external mainline execution plan is missing from the "
            "industrial customer acceptance bundle."
        ),
        "bundle_path": str(bundle_path),
    }


def _summarize_bundle_release_ops_execution(
    payload: dict[str, Any],
    *,
    bundle_path: Path,
) -> dict[str, Any]:
    report = _resolve_customer_acceptance_report(
        payload,
        report_name="release_ops_execution",
    )
    if not report:
        return {}

    summary: dict[str, Any] = {
        "status": report.get("status"),
        "summary": report.get("summary"),
        "bundle_path": str(bundle_path),
    }
    report_path = (
        report.get("resolved_report_path")
        or report.get("report_path")
        or report.get("path")
    )
    if isinstance(report_path, str) and report_path.strip():
        summary["report_path"] = report_path.strip()
    control_plane_session = report.get("control_plane_session")
    if isinstance(control_plane_session, dict) and control_plane_session:
        summary["control_plane_session"] = dict(control_plane_session)
    control_plane_event_stream = report.get("control_plane_event_stream")
    if isinstance(control_plane_event_stream, dict) and control_plane_event_stream:
        summary["control_plane_event_stream"] = dict(control_plane_event_stream)
    metrics = report.get("metrics")
    if isinstance(metrics, dict):
        event_count = metrics.get("event_count")
        if isinstance(event_count, int) and event_count >= 0:
            summary["event_count"] = event_count
        for field in ["action", "policy_level", "policy_profile", "request_type"]:
            value = metrics.get(field)
            if isinstance(value, str) and value.strip():
                summary[field] = value.strip()
    return summary


def _format_external_mainline_execution_plan_summary(component: dict[str, Any]) -> str:
    status = str(component.get("status") or "missing").strip() or "missing"
    counts: list[str] = []
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = component.get(field)
        if not isinstance(value, int) or value < 0:
            return status
        counts.append(str(value))
    return "/".join([status, *counts])


def _init_git_repo(source_root: Path, *, version: str, tag: str) -> dict[str, Any]:
    source_root.mkdir(parents=True, exist_ok=True)
    (source_root / "README.md").write_text(
        f"# Release rehearsal for {version}\n",
        encoding="utf-8",
    )
    commands = [
        ["git", "init"],
        ["git", "config", "user.name", "AGI-Walker Release Rehearsal"],
        ["git", "config", "user.email", "release-rehearsal@example.com"],
        ["git", "add", "README.md"],
        ["git", "commit", "-m", "seed rehearsal repo"],
        ["git", "tag", tag],
    ]
    for command in commands:
        result = _run_command(command, cwd=source_root)
        if result.returncode != 0:
            raise RuntimeError(
                result.stderr.strip() or result.stdout.strip() or "git command failed"
            )

    commit_sha = _run_command(
        ["git", "rev-parse", "HEAD"], cwd=source_root
    ).stdout.strip()
    short_commit_sha = _run_command(
        ["git", "rev-parse", "--short=12", "HEAD"],
        cwd=source_root,
    ).stdout.strip()
    return {
        "source_root": str(source_root),
        "commit_sha": commit_sha,
        "short_commit_sha": short_commit_sha,
        "tag": tag,
    }


def _seed_live_evidence(project_root: Path) -> list[dict[str, str]]:
    reports = {
        project_root
        / "test_env"
        / "distributed_smoke"
        / "distributed_smoke_report.json": {
            "schema_version": "1.0",
            "status": "passed",
            "actor_id": "release-rehearsal-actor",
            "checks": [
                {"name": "compose_build", "status": "pass"},
                {"name": "compose_up", "status": "pass"},
                {"name": "web_panel_status", "status": "pass"},
            ],
        },
        project_root
        / "test_env"
        / "godot_headless_smoke"
        / "headless_smoke_report.json": {
            "status": "passed",
            "message": "Seeded Godot headless live smoke evidence for release rehearsal.",
        },
        project_root
        / "test_env"
        / "ros2_bridge_smoke"
        / "ros2_bridge_smoke_report.json": {
            "status": "passed",
            "message": "Seeded ROS2 bridge live smoke evidence for release rehearsal.",
        },
    }
    seeded_paths: list[dict[str, str]] = []
    for path, payload in reports.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        seeded_paths.append({"name": path.name, "path": str(path)})
    return seeded_paths


def _seed_customer_delivery_docs(project_root: Path) -> list[dict[str, str]]:
    docs = {
        project_root / "README.md": "# Release Rehearsal Acceptance Bundle\n",
        project_root / "docs" / "CURRENT_STATUS.md": "# Current Status\n",
        project_root / "docs" / "guides" / "RELEASE_GUIDE.md": "# Release Guide\n",
        project_root
        / "docs"
        / "guides"
        / "DEPLOYMENT_MATRIX.md": "# Deployment Matrix\n",
        project_root
        / "docs"
        / "guides"
        / "CUSTOMER_INSTALLATION_GUIDE.md": "# Customer Installation Guide\n",
        project_root / "docs" / "guides" / "SUPPORT_MATRIX.md": "# Support Matrix\n",
        project_root
        / "docs"
        / "guides"
        / "CAPACITY_AND_SCALE.md": "# Capacity And Scale\n",
        project_root
        / "docs"
        / "guides"
        / "CUSTOMER_ACCEPTANCE_CHECKLIST.md": "# Customer Acceptance Checklist\n",
        project_root
        / "docs"
        / "guides"
        / "KNOWN_LIMITATIONS.md": "# Known Limitations\n",
        project_root / "PRODUCTION_DEPLOYMENT_RUNBOOK.md": "# Production Runbook\n",
        project_root
        / "docs"
        / "guides"
        / "SECURITY_BASELINE.md": "# Security Baseline\n",
        project_root
        / "docs"
        / "guides"
        / "AUDIT_TRAIL_POLICY.md": "# Audit Trail Policy\n",
        project_root
        / "docs"
        / "guides"
        / "BACKUP_RESTORE_RUNBOOK.md": "# Backup Restore Runbook\n",
        project_root
        / "docs"
        / "guides"
        / "INCIDENT_RESPONSE_MATRIX.md": "# Incident Response Matrix\n",
    }
    seeded_paths: list[dict[str, str]] = []
    for path, content in docs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
        seeded_paths.append({"name": path.name, "path": str(path)})
    return seeded_paths


def _seed_required_release_evidence(project_root: Path) -> list[dict[str, str]]:
    report_root = project_root / "test_env" / "release_evidence"
    report_root.mkdir(parents=True, exist_ok=True)
    reports = {
        report_root
        / "non_live_gate_report.json": build_release_evidence_report(
            evidence_name="non_live_gate",
            status="passed",
            summary="non_live_gate pytest evidence passed: 856 passed, 4 skipped, 3 deselected.",
            command='python -m pytest -m "not live" -q',
            generated_at=_now_iso(),
            metrics={"passed": 856, "skipped": 4, "deselected": 3},
            source_commit_sha="release-rehearsal-non-live",
        ),
        report_root
        / "release_contracts_and_capability_matrix_report.json": build_release_evidence_report(
            evidence_name="release_contracts_and_capability_matrix",
            status="passed",
            summary="release_contracts_and_capability_matrix pytest evidence passed: 50 passed.",
            command="python -m pytest tests/test_release_contracts.py tests/test_release_artifact_builder.py tests/test_release_readiness.py tests/test_stable_promotion_checklist.py tests/test_active_path_references.py -q",
            generated_at=_now_iso(),
            metrics={"passed": 50},
            source_commit_sha="release-rehearsal-contracts",
        ),
    }
    seeded_paths: list[dict[str, str]] = []
    for path, payload in reports.items():
        write_release_evidence_report(payload, path)
        seeded_paths.append({"name": path.name, "path": str(path)})
    return seeded_paths


def _seed_backup_restore_rehearsal(project_root: Path) -> dict[str, str]:
    project_root = project_root.resolve()
    security_root = project_root / "test_env" / "release_evidence" / "security"
    source_runtime_root = security_root / "backup_restore" / "source_runtime"
    source_config_root = security_root / "backup_restore" / "source_config"
    backup_snapshot_root = security_root / "backup_restore" / "backup_snapshot"
    restored_runtime_root = security_root / "backup_restore" / "restored_runtime"
    restored_config_root = security_root / "backup_restore" / "restored_config"

    runtime_files = {
        source_runtime_root / "db" / "state.json": '{"status":"ok"}\n',
        source_runtime_root / "workflow_runs" / "run.json": '{"id":"rehearsal"}\n',
        source_runtime_root
        / "workflow_archive"
        / "archive.json": '{"archived":true}\n',
        source_runtime_root / "backups" / "manifest.json": '{"backup":"ok"}\n',
    }
    config_files = {
        source_config_root / "compose.env": "AGI_WALKER_WEB_PORT=8000\n",
        source_config_root / "web_panel.env": "AGI_WALKER_SECRET_KEY=rehearsal\n",
    }
    for path, content in {**runtime_files, **config_files}.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    runtime_copy_targets = {
        backup_snapshot_root
        / "db"
        / "state.json": runtime_files[source_runtime_root / "db" / "state.json"],
        backup_snapshot_root
        / "workflow_runs"
        / "run.json": runtime_files[source_runtime_root / "workflow_runs" / "run.json"],
        backup_snapshot_root
        / "workflow_archive"
        / "archive.json": runtime_files[
            source_runtime_root / "workflow_archive" / "archive.json"
        ],
        backup_snapshot_root
        / "backups"
        / "manifest.json": runtime_files[
            source_runtime_root / "backups" / "manifest.json"
        ],
        restored_runtime_root
        / "db"
        / "state.json": runtime_files[source_runtime_root / "db" / "state.json"],
        restored_runtime_root
        / "workflow_runs"
        / "run.json": runtime_files[source_runtime_root / "workflow_runs" / "run.json"],
        restored_runtime_root
        / "workflow_archive"
        / "archive.json": runtime_files[
            source_runtime_root / "workflow_archive" / "archive.json"
        ],
        restored_runtime_root
        / "backups"
        / "manifest.json": runtime_files[
            source_runtime_root / "backups" / "manifest.json"
        ],
        backup_snapshot_root
        / "compose.env": config_files[source_config_root / "compose.env"],
        backup_snapshot_root
        / "web_panel.env": config_files[source_config_root / "web_panel.env"],
        restored_config_root
        / "compose.env": config_files[source_config_root / "compose.env"],
        restored_config_root
        / "web_panel.env": config_files[source_config_root / "web_panel.env"],
    }
    for path, content in runtime_copy_targets.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    report_path = security_root / "backup_restore_rehearsal_report.json"
    payload = build_backup_restore_rehearsal_report(
        project_root=project_root,
        actor="release-rehearsal",
        source_runtime_root=_path_for_contract(
            source_runtime_root,
            project_root=project_root,
        ),
        source_config_root=_path_for_contract(
            source_config_root,
            project_root=project_root,
        ),
        backup_snapshot_root=_path_for_contract(
            backup_snapshot_root,
            project_root=project_root,
        ),
        restored_runtime_root=_path_for_contract(
            restored_runtime_root,
            project_root=project_root,
        ),
        restored_config_root=_path_for_contract(
            restored_config_root,
            project_root=project_root,
        ),
        rehearsal_duration_seconds=1.0,
    )
    write_backup_restore_rehearsal_report(payload, report_path)
    return {"name": report_path.name, "path": str(report_path)}


def _seed_security_evidence(project_root: Path) -> list[dict[str, str]]:
    project_root = project_root.resolve()
    security_root = project_root / "test_env" / "release_evidence" / "security"
    security_root.mkdir(parents=True, exist_ok=True)

    sbom_path = security_root / "sbom.json"
    write_sbom_artifact(build_sbom_artifact(project_root=PROJECT_ROOT), sbom_path)

    python_report_path = security_root / "python_vuln_scan_report.json"
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="python_dependencies",
            target="pyproject.toml",
            status="passed",
            summary="python dependency scan passed for release rehearsal.",
            command="pip-audit --format json",
            scanner="pip-audit",
            finding_count=0,
            affected_component_count=0,
        ),
        python_report_path,
    )

    container_raw_root = security_root / "container_vuln_scan_report_raw"
    container_raw_root.mkdir(parents=True, exist_ok=True)
    (container_raw_root / "deployment-web-panel-distributed.json").write_text(
        json.dumps(
            {
                "ArtifactName": "deployment-web-panel-distributed",
                "Results": [
                    {
                        "Target": "debian:stable",
                        "Vulnerabilities": [
                            {
                                "PkgName": "libsystemd0",
                                "InstalledVersion": "257.8-1",
                                "FixedVersion": "",
                                "VulnerabilityID": "CVE-REHEARSAL-1",
                                "Severity": "HIGH",
                            }
                        ],
                    }
                ],
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    container_report_path = security_root / "container_vuln_scan_report.json"
    write_vulnerability_scan_report(
        build_vulnerability_scan_report(
            scan_name="container_images",
            target="deployment/docker-compose.yml",
            status="blocked",
            summary="trivy reported 1 finding(s) across 1 affected component(s) in 1 image(s).",
            command="trivy image --scanners vuln --timeout 15m --format json",
            scanner="trivy",
            report_format="trivy-json",
            raw_report_path=_path_for_contract(
                container_raw_root,
                project_root=project_root,
            ),
            finding_count=1,
            affected_component_count=1,
        ),
        container_report_path,
    )

    exception_report_path = security_root / "vulnerability_exception_report.json"
    write_vulnerability_exception_report(
        build_vulnerability_exception_report(
            project_root=project_root,
            generated_at=_now_iso(),
            exceptions=[
                {
                    "id": "release-rehearsal-container-exception",
                    "scope": "container_images",
                    "component": "libsystemd0",
                    "image_refs": ["deployment-web-panel-distributed"],
                    "vulnerability_ids": ["CVE-REHEARSAL-1"],
                    "justification": "Rehearsal exception proving industrial delivery can close approved residual image findings.",
                    "approved_by": "security-reviewer",
                    "approved_at": _now_iso(),
                    "expires_at": "2026-05-15T00:00:00+00:00",
                }
            ],
        ),
        exception_report_path,
    )
    exception_review_report_path = (
        security_root / "vulnerability_exception_review_report.json"
    )
    write_release_evidence_report(
        build_vulnerability_exception_review_report(
            project_root=project_root,
            exception_report_path=_path_for_contract(
                exception_report_path,
                project_root=project_root,
            ),
            output_path=_path_for_contract(
                exception_review_report_path,
                project_root=project_root,
            ),
            generated_at=_now_iso(),
        ),
        exception_review_report_path,
    )

    remediation_path = security_root / "vulnerability_remediation_report.json"
    write_vulnerability_remediation_report(
        build_vulnerability_remediation_report(
            project_root=project_root,
            python_vuln_report_path=_path_for_contract(
                python_report_path,
                project_root=project_root,
            ),
            container_vuln_report_path=_path_for_contract(
                container_report_path,
                project_root=project_root,
            ),
            vulnerability_exception_report_path=_path_for_contract(
                exception_report_path,
                project_root=project_root,
            ),
        ),
        remediation_path,
    )

    backup_restore_report = _seed_backup_restore_rehearsal(project_root)
    backup_restore_path = Path(backup_restore_report["path"])

    posture_path = security_root / "security_posture_report.json"
    write_security_posture_report(
        build_security_posture_report(
            project_root=project_root,
            sbom_path=_path_for_contract(sbom_path, project_root=project_root),
            python_vuln_report_path=_path_for_contract(
                python_report_path,
                project_root=project_root,
            ),
            container_vuln_report_path=_path_for_contract(
                container_report_path,
                project_root=project_root,
            ),
            backup_restore_rehearsal_report_path=_path_for_contract(
                backup_restore_path,
                project_root=project_root,
            ),
            vulnerability_remediation_report_path=_path_for_contract(
                remediation_path,
                project_root=project_root,
            ),
            vulnerability_exception_report_path=_path_for_contract(
                exception_report_path,
                project_root=project_root,
            ),
        ),
        posture_path,
    )

    preflight_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "security_release_preflight_report.json"
    )
    _run_checked_command(
        [
            sys.executable,
            str(PROJECT_ROOT / "tools" / "run_security_release_preflight.py"),
            "--output-root",
            str(project_root / "test_env" / "release_evidence"),
            "--skip-collect",
            "--security-posture-report",
            str(posture_path),
            "--report-file",
            str(preflight_path),
        ],
        cwd=PROJECT_ROOT,
        failure_message="security release preflight build failed",
    )

    return [
        {"name": sbom_path.name, "path": str(sbom_path)},
        {"name": python_report_path.name, "path": str(python_report_path)},
        {"name": container_report_path.name, "path": str(container_report_path)},
        {"name": exception_report_path.name, "path": str(exception_report_path)},
        {
            "name": exception_review_report_path.name,
            "path": str(exception_review_report_path),
        },
        {"name": remediation_path.name, "path": str(remediation_path)},
        backup_restore_report,
        {"name": posture_path.name, "path": str(posture_path)},
        {"name": preflight_path.name, "path": str(preflight_path)},
    ]


def _seed_extension_execution_evidence(project_root: Path) -> list[dict[str, str]]:
    tool_module = _load_repo_tool_module(
        "_agi_walker_tool_build_extension_execution_evidence",
        "tools/build_extension_execution_evidence.py",
    )

    project_root = project_root.resolve()
    output_root = project_root / "test_env" / "release_evidence" / "operations"
    payloads, _ = tool_module.build_extension_execution_evidence_reports(
        project_root=project_root,
        output_root=output_root,
        source_root=project_root,
        vulnerability_exception_report_path=project_root
        / "test_env"
        / "release_evidence"
        / "security"
        / "vulnerability_exception_report.json",
        actuals_artifact_path=project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_actuals.json",
        generated_at=_now_iso(),
    )
    written = tool_module.write_extension_execution_evidence_reports(
        project_root=project_root,
        output_root=output_root,
        payloads=payloads,
    )
    return [{"name": path.name, "path": str(path)} for path in written.values()]


def _seed_customer_external_bindings_closure_report(
    project_root: Path,
    *,
    external_bindings_config_path: str | None,
) -> dict[str, str]:
    project_root = project_root.resolve()
    output_path = (
        project_root / default_customer_external_bindings_closure_report_path()
    )
    confirmation_report_path = (
        project_root / default_customer_external_bindings_confirmation_report_path()
    )
    instance_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    schedule_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_schedule.json"
    )
    actuals_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_actuals.json"
    )
    resolved_config_path = (
        str(external_bindings_config_path).strip()
        if external_bindings_config_path and str(external_bindings_config_path).strip()
        else "deployment/customer_delivery.external_bindings.rehearsal.json"
    )
    command = build_run_customer_external_bindings_closure_command(
        config_path=resolved_config_path,
        instance_artifact_path=_path_for_contract(
            instance_path, project_root=project_root
        ),
        schedule_artifact_path=_path_for_contract(
            schedule_path, project_root=project_root
        ),
        actuals_artifact_path=_path_for_contract(
            actuals_path, project_root=project_root
        ),
        confirmation_report_output_path=_path_for_contract(
            confirmation_report_path,
            project_root=project_root,
        ),
        closure_report_output_path=default_customer_external_bindings_closure_report_path(),
        sections=list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS),
        skip_collect_release_evidence=True,
    )
    payload = build_release_evidence_report(
        evidence_name="customer_external_bindings_closure",
        status="passed",
        summary=(
            "customer external bindings closure passed: rehearsal config and "
            "ready actuals/confirmation evidence are available for approval_identity, "
            "archive_target, due_trigger."
        ),
        command=command,
        metrics={
            "project_root": str(project_root),
            "config_path": resolved_config_path,
            "config_exists": (project_root / resolved_config_path).is_file(),
            "instance_artifact_path": _path_for_contract(
                instance_path, project_root=project_root
            ),
            "instance_exists": instance_path.is_file(),
            "schedule_artifact_path": _path_for_contract(
                schedule_path, project_root=project_root
            ),
            "schedule_exists": schedule_path.is_file(),
            "actuals_output_path": _path_for_contract(
                actuals_path, project_root=project_root
            ),
            "actuals_exists": actuals_path.is_file(),
            "confirmation_report_output_path": _path_for_contract(
                confirmation_report_path,
                project_root=project_root,
            ),
            "confirmation_report_exists": confirmation_report_path.is_file(),
            "closure_report_output_path": default_customer_external_bindings_closure_report_path(),
            "selected_sections": list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS),
            "confirmed_by": "release-rehearsal",
            "confirmation_ticket": "CHG-REHEARSAL-INDUSTRIAL",
            "generated_instance": False,
            "generated_schedule": False,
            "generated_config": False,
            "collect_release_evidence": False,
            "failure_count": 0,
            "failed_steps": [],
            "overrides_file": None,
        },
        source_commit_sha="release-rehearsal",
    )
    written = write_release_evidence_report(payload, output_path)
    return {"name": written.name, "path": str(written)}


def _seed_external_mainline_execution_plan(
    project_root: Path, *, version: str, build_id: str
) -> dict[str, str]:
    project_root = project_root.resolve()
    output_path = project_root / default_external_mainline_execution_plan_path()
    payload = build_external_mainline_execution_plan_artifact(
        project_root=project_root,
        control_plane_session={
            "engagement_id": build_id,
            "window_id": f"{version}-external-mainline",
            "change_ticket": f"{build_id}-change",
            "channel": "industrial",
        },
        control_plane_event_stream={
            "path": "test_env/release_ops/release_rehearsal_external_mainline.jsonl",
            "event_count": 3,
        },
    )
    written = write_external_mainline_execution_plan_artifact(payload, output_path)
    return {"name": written.name, "path": str(written)}


def _seed_external_mainline_input_checklist_report(
    project_root: Path,
    *,
    version: str,
    build_id: str,
) -> dict[str, str]:
    project_root = project_root.resolve()
    output_path = project_root / default_external_mainline_input_checklist_report_path()
    payload = build_external_mainline_input_checklist_report(
        project_root=project_root,
        output_path=default_external_mainline_input_checklist_report_path(),
        external_mainline_execution_plan_path=default_external_mainline_execution_plan_path(),
        control_plane_session={
            "engagement_id": build_id,
            "window_id": f"{version}-external-mainline-inputs",
            "change_ticket": f"{build_id}-change",
            "channel": "industrial",
        },
        control_plane_event_stream={
            "path": "test_env/release_ops/release_rehearsal_external_mainline.jsonl",
            "event_count": 3,
        },
    )
    written = write_release_evidence_report(payload, output_path)
    return {"name": written.name, "path": str(written)}


def _seed_release_ops_execution_report(
    project_root: Path,
    *,
    version: str,
    build_id: str,
) -> dict[str, str]:
    project_root = project_root.resolve()
    output_path = project_root / default_release_ops_execution_report_path()
    payload = build_release_evidence_report(
        evidence_name="release_ops_execution",
        status="passed",
        summary=(
            "release op external_mainline_execution completed via rehearsal "
            "control plane evidence wrapper."
        ),
        command=(
            "python tools/run_release_ops.py external_mainline_execution "
            "--policy-profile requires_attestation "
            "--event-stream-file test_env/release_ops/"
            "release_rehearsal_external_mainline.jsonl "
            "--evidence-report-file "
            f"{default_release_ops_execution_report_path()}"
        ),
        metrics={
            "action": "external_mainline_execution",
            "policy_level": "requires_attestation",
            "policy_profile": "requires_attestation",
            "request_type": "ExternalMainlineExecutionRequest",
            "status": "ready",
            "event_count": 3,
            "output_path": default_external_mainline_execution_plan_path(),
        },
        source_commit_sha="release-rehearsal",
        control_plane_session={
            "engagement_id": build_id,
            "window_id": f"{version}-external-mainline-release-ops",
            "change_ticket": f"{build_id}-change",
            "channel": "industrial",
        },
        control_plane_event_stream={
            "path": "test_env/release_ops/release_rehearsal_external_mainline.jsonl",
            "event_count": 3,
        },
    )
    written = write_release_evidence_report(payload, output_path)
    return {"name": written.name, "path": str(written)}


def _seed_extension_execution_instance(project_root: Path) -> dict[str, str]:
    project_root = project_root.resolve()
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_instance.json"
    )
    payload = build_extension_execution_instance_artifact(
        project_root=project_root,
        artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
        engagement_id="release-rehearsal-industrial",
        customer_name="AGI-Walker Rehearsal Customer",
        site_name="sandbox-site",
        change_ticket="CHG-REHEARSAL-INDUSTRIAL",
        window_id="window-release-rehearsal",
        window_start_at=_now_iso(),
        window_end_at=_now_iso(),
        delivery_root="test_env/release_delivery/release_rehearsal",
        closure_archive_root="test_env/release_delivery/release_rehearsal/closure_archive",
        exception_review_due_at="2026-05-15T00:00:00+00:00",
    )
    written = write_extension_execution_instance_artifact(payload, output_path)
    return {"name": written.name, "path": str(written)}


def _seed_extension_execution_schedule(project_root: Path) -> dict[str, str]:
    project_root = project_root.resolve()
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_schedule.json"
    )
    payload = build_extension_execution_schedule_artifact(
        project_root=project_root,
        artifact_path="test_env/release_evidence/operations/extension_execution_schedule.json",
        instance_artifact_path="test_env/release_evidence/operations/extension_execution_instance.json",
    )
    written = write_extension_execution_schedule_artifact(payload, output_path)
    return {"name": written.name, "path": str(written)}


def _seed_extension_external_bindings_config(
    project_root: Path,
    *,
    config_source: str | Path,
) -> dict[str, str]:
    project_root = project_root.resolve()
    source_path = _resolve_source_path(config_source)
    if not source_path.is_file():
        raise RuntimeError(f"external bindings config is missing: {source_path}")
    try:
        payload = json.loads(source_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise RuntimeError(
            f"external bindings config is not valid JSON: {source_path}: {exc}"
        ) from exc
    if not isinstance(payload, dict):
        raise RuntimeError("external bindings config must be a JSON object")

    try:
        relative_path = source_path.resolve().relative_to(PROJECT_ROOT.resolve())
    except ValueError:
        relative_path = Path("deployment") / source_path.name
    output_path = project_root / relative_path
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return {
        "name": output_path.name,
        "path": str(output_path),
        "contract_path": relative_path.as_posix(),
    }


def _seed_extension_execution_actuals(
    project_root: Path,
    *,
    external_bindings_config_path: str | None = None,
) -> dict[str, str]:
    project_root = project_root.resolve()
    output_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "operations"
        / "extension_execution_actuals.json"
    )
    command = [
        sys.executable,
        str(PROJECT_ROOT / "tools" / "build_extension_execution_actuals.py"),
        "--project-root",
        str(project_root),
        "--output",
        "test_env/release_evidence/operations/extension_execution_actuals.json",
        "--schedule-artifact",
        "test_env/release_evidence/operations/extension_execution_schedule.json",
    ]
    if external_bindings_config_path:
        command.extend(
            [
                "--external-bindings-config",
                external_bindings_config_path,
            ]
        )
    result = _run_command(command, cwd=PROJECT_ROOT)
    if result.returncode != 0:
        raise RuntimeError(
            result.stderr.strip()
            or result.stdout.strip()
            or "extension execution actuals build failed"
        )
    return {"name": output_path.name, "path": str(output_path)}


def _seed_clean_checkout_evidence(
    project_root: Path, *, version: str, tag: str
) -> dict[str, str]:
    report_path = (
        project_root
        / "test_env"
        / "release_evidence"
        / "clean_checkout_smoke_report.json"
    )
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(
            {
                "schema_version": "1.0",
                "artifact_type": "clean_checkout_smoke_report",
                "status": "passed",
                "generated_at": _now_iso(),
                "source_root": str(project_root),
                "checkout_root": str(
                    project_root / "test_env" / "release_rehearsal_checkout"
                ),
                "output_root": str(project_root / "test_env" / "release_rehearsal"),
                "version": version,
                "tag": tag,
                "runs": 2,
                "command_template": [
                    sys.executable,
                    "tests/run_smoke_tests.py",
                    "--output-root",
                    "{run_output_root}",
                ],
                "checkout_commit_sha": "release-rehearsal-clean-checkout",
                "seeded_evidence_paths": [
                    "test_env/release_evidence/non_live_gate_report.json",
                    "test_env/release_evidence/release_contracts_and_capability_matrix_report.json",
                ],
                "checks": [
                    {
                        "name": "release_rehearsal_seed",
                        "status": "pass",
                        "detail": "seeded clean checkout smoke evidence for stable rehearsal",
                    }
                ],
                "run_reports": [
                    {
                        "run_index": 1,
                        "status": "passed",
                        "command": [sys.executable, "tests/run_smoke_tests.py"],
                        "run_output_root": "run_01",
                        "exit_code": 0,
                        "stdout_path": "logs/run_01.stdout.txt",
                        "stderr_path": "logs/run_01.stderr.txt",
                        "worktree_clean": True,
                        "dirty_paths": [],
                    },
                    {
                        "run_index": 2,
                        "status": "passed",
                        "command": [sys.executable, "tests/run_smoke_tests.py"],
                        "run_output_root": "run_02",
                        "exit_code": 0,
                        "stdout_path": "logs/run_02.stdout.txt",
                        "stderr_path": "logs/run_02.stderr.txt",
                        "worktree_clean": True,
                        "dirty_paths": [],
                    },
                ],
                "failure_reason": None,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return {"name": report_path.name, "path": str(report_path)}


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def execute_release_rehearsal(
    request: ReleaseRehearsalRequest,
) -> ReleaseRehearsalResult:
    output_root = _resolve_source_path(request.output_root).resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = (
        _resolve_source_path(request.report_file).resolve()
        if request.report_file
        else output_root / "release_rehearsal_report.json"
    )
    industrial_delivery_rehearsal_report_path = (
        output_root / "industrial_delivery_rehearsal_report.json"
    )
    source_root = output_root / f"git_source_{uuid4().hex[:8]}"
    manifest_path = output_root / "release_manifest.json"
    tag = request.tag or request.version

    checks: list[dict[str, str]] = []
    status = "failed"
    builder_stdout = ""
    builder_stderr = ""
    git_source: dict[str, Any] | None = None
    evidence_paths: list[dict[str, str]] = []
    document_paths: list[dict[str, str]] = []
    release_gate_status = "blocked"
    customer_delivery_status = "blocked"
    industrial_delivery_status = "blocked"
    external_bindings_config: dict[str, str] | None = None
    extension_execution_plan: dict[str, Any] = {}
    extension_execution_evidence: dict[str, Any] = {}
    extension_execution_instance: dict[str, Any] = {}
    extension_execution_schedule: dict[str, Any] = {}
    extension_execution_actuals: dict[str, Any] = {}
    security_release_preflight: dict[str, Any] = {}
    vulnerability_exception_review: dict[str, Any] = {}
    customer_external_bindings_closure: dict[str, Any] = {}
    external_mainline_execution_plan: dict[str, Any] = {}
    external_mainline_input_checklist: dict[str, Any] = {}
    release_ops_execution: dict[str, Any] = {}
    industrial_manifest: dict[str, Any] = {}
    industrial_release_readiness: dict[str, Any] = {}
    industrial_promotion_checklist: dict[str, Any] = {}
    industrial_customer_acceptance_bundle: dict[str, Any] = {}
    industrial_delivery_artifact_paths: list[dict[str, str]] = []
    delivery_rehearsal_stages: list[dict[str, Any]] = []

    try:
        git_source = _init_git_repo(source_root, version=request.version, tag=tag)
        checks.append(
            {
                "name": "git_repo_init",
                "status": "pass",
                "detail": f"temporary Git repo initialized at {source_root}",
            }
        )
        checks.append(
            {
                "name": "version_tag_created",
                "status": "pass",
                "detail": f"created matching tag {tag}",
            }
        )

        document_paths = _seed_customer_delivery_docs(output_root)
        external_bindings_config = _seed_extension_external_bindings_config(
            output_root,
            config_source=request.external_bindings_config_source,
        )
        document_paths.append(external_bindings_config)
        checks.append(
            {
                "name": "customer_delivery_docs_seeded",
                "status": "pass",
                "detail": "seeded the required customer delivery document set and managed external bindings config for rehearsal",
            }
        )

        evidence_paths = _seed_live_evidence(output_root)
        evidence_paths = [
            _seed_clean_checkout_evidence(
                output_root, version=request.version, tag=tag
            ),
            *_seed_required_release_evidence(output_root),
            *evidence_paths,
            *_seed_security_evidence(output_root),
            _seed_extension_execution_instance(output_root),
            _seed_extension_execution_schedule(output_root),
            _seed_extension_execution_actuals(
                output_root,
                external_bindings_config_path=(
                    external_bindings_config.get("contract_path")
                    if external_bindings_config is not None
                    else None
                ),
            ),
            *_seed_extension_execution_evidence(output_root),
            _seed_customer_external_bindings_closure_report(
                output_root,
                external_bindings_config_path=(
                    external_bindings_config.get("contract_path")
                    if external_bindings_config is not None
                    else None
                ),
            ),
            _seed_external_mainline_execution_plan(
                output_root,
                version=request.version,
                build_id=request.build_id,
            ),
            _seed_external_mainline_input_checklist_report(
                output_root,
                version=request.version,
                build_id=request.build_id,
            ),
            _seed_release_ops_execution_report(
                output_root,
                version=request.version,
                build_id=request.build_id,
            ),
        ]
        checks.append(
            {
                "name": "live_evidence_seeded",
                "status": "pass",
                "detail": "seeded required evidence, live evidence, security/industrial closure reports, external mainline execution plan, release-ops execution evidence, customer-specific extension execution instance/schedule/actuals artifacts, and extension execution evidence/closure reports including customer external bindings confirmation",
            }
        )

        builder = _run_command(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "build_release_artifact.py"),
                "--version",
                request.version,
                "--channel",
                "stable",
                "--build-id",
                request.build_id,
                "--release-summary",
                "Stable release rehearsal validation.",
                "--project-root",
                str(output_root),
                "--source-root",
                str(source_root),
                "--approval-status",
                "approved",
                "--approved-by",
                "release-manager",
                "--approved-at",
                _now_iso(),
                "--approval-notes",
                "stable rehearsal signoff",
                "--output",
                str(manifest_path),
            ],
            cwd=PROJECT_ROOT,
        )
        builder_stdout = builder.stdout.strip()
        builder_stderr = builder.stderr.strip()
        if builder.returncode != 0:
            raise RuntimeError(
                builder_stderr or builder_stdout or "release builder failed"
            )
        checks.append(
            {
                "name": "release_manifest_built",
                "status": "pass",
                "detail": f"built stable manifest at {manifest_path}",
            }
        )

        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        validation_errors = validate_release_manifest_artifact(payload)
        if validation_errors:
            raise RuntimeError("; ".join(validation_errors))
        checks.append(
            {
                "name": "release_manifest_validated",
                "status": "pass",
                "detail": "manifest matches release contract",
            }
        )

        if payload.get("release_gate_status") != "ready":
            raise RuntimeError(
                f"expected release_gate_status='ready', got {payload.get('release_gate_status')!r}"
            )
        release_gate_status = str(payload.get("release_gate_status") or "blocked")
        checks.append(
            {
                "name": "stable_gate_ready",
                "status": "pass",
                "detail": "stable rehearsal reached release_gate_status=ready",
            }
        )

        customer_delivery_status = str(
            payload.get("customer_delivery_surface", {}).get("status") or "blocked"
        )
        if customer_delivery_status != "ready":
            raise RuntimeError(
                "expected customer_delivery_surface.status='ready', got "
                f"{customer_delivery_status!r}"
            )
        checks.append(
            {
                "name": "customer_delivery_ready",
                "status": "pass",
                "detail": "rehearsal manifest reached customer_delivery_surface.status=ready",
            }
        )

        industrial_delivery_status = str(
            payload.get("industrial_delivery_gate", {}).get("status") or "blocked"
        )
        if industrial_delivery_status != "ready":
            raise RuntimeError(
                "expected industrial_delivery_gate.status='ready', got "
                f"{industrial_delivery_status!r}"
            )
        checks.append(
            {
                "name": "industrial_delivery_ready",
                "status": "pass",
                "detail": "rehearsal manifest reached industrial_delivery_gate.status=ready",
            }
        )

        extension_execution_plan = build_extension_execution_plan(
            payload.get("customer_delivery_surface", {}).get(
                "extension_support_surface",
                {},
            )
            if isinstance(payload.get("customer_delivery_surface"), dict)
            else {}
        )
        if extension_execution_plan.get("status") != "ready":
            raise RuntimeError(
                "expected extension_execution_plan.status='ready', got "
                f"{extension_execution_plan.get('status')!r}"
            )
        checks.append(
            {
                "name": "extension_execution_plan_ready",
                "status": "pass",
                "detail": "rehearsal manifest attached a ready extension execution plan with watch actions, on-call handoff records, exception review steps, escalation closure evidence, and rollback evidence duties",
            }
        )
        extension_execution_evidence = dict(
            payload.get("extension_execution_evidence", {})
            if isinstance(payload.get("extension_execution_evidence"), dict)
            else {}
        )
        if extension_execution_evidence.get("status") != "ready":
            raise RuntimeError(
                "expected extension_execution_evidence.status='ready', got "
                f"{extension_execution_evidence.get('status')!r}"
            )
        checks.append(
            {
                "name": "extension_execution_evidence_ready",
                "status": "pass",
                "detail": "rehearsal manifest attached ready extension execution evidence with on-call rehearsal, exception review scheduling, and escalation closure reports",
            }
        )
        extension_execution_instance = dict(
            payload.get("extension_execution_instance", {})
            if isinstance(payload.get("extension_execution_instance"), dict)
            else {}
        )
        if extension_execution_instance.get("status") != "ready":
            raise RuntimeError(
                "expected extension_execution_instance.status='ready', got "
                f"{extension_execution_instance.get('status')!r}"
            )
        checks.append(
            {
                "name": "extension_execution_instance_ready",
                "status": "pass",
                "detail": "rehearsal manifest attached a ready customer-specific extension execution instance with delivery window, exception review due date, and closure archive targets",
            }
        )
        extension_execution_schedule = dict(
            payload.get("extension_execution_schedule", {})
            if isinstance(payload.get("extension_execution_schedule"), dict)
            else {}
        )
        if extension_execution_schedule.get("status") != "ready":
            raise RuntimeError(
                "expected extension_execution_schedule.status='ready', got "
                f"{extension_execution_schedule.get('status')!r}"
            )
        checks.append(
            {
                "name": "extension_execution_schedule_ready",
                "status": "pass",
                "detail": "rehearsal manifest attached a ready extension execution schedule with window triggers, signoff checkpoints, residual-risk review records, and closure archive manifests",
            }
        )
        extension_execution_actuals = dict(
            payload.get("extension_execution_actuals", {})
            if isinstance(payload.get("extension_execution_actuals"), dict)
            else {}
        )
        if extension_execution_actuals.get("status") != "ready":
            raise RuntimeError(
                "expected extension_execution_actuals.status='ready', got "
                f"{extension_execution_actuals.get('status')!r}"
            )
        actuals_detail = (
            "rehearsal manifest attached ready execution actuals with window trigger "
            "records, signoff records, residual-risk review records, and closure manifests"
        )
        external_bindings_status = str(
            extension_execution_actuals.get("external_bindings_status") or ""
        ).strip()
        if external_bindings_status:
            actuals_detail += f"; external bindings status={external_bindings_status}"
        checks.append(
            {
                "name": "extension_execution_actuals_ready",
                "status": "pass",
                "detail": actuals_detail,
            }
        )

        security_preflight_report_path = (
            output_root
            / "test_env"
            / "release_evidence"
            / "security_release_preflight_report.json"
        )
        security_preflight_payload = _load_json_payload(security_preflight_report_path)
        security_release_preflight = _summarize_security_release_preflight(
            security_preflight_payload,
            report_path=security_preflight_report_path,
        )
        if security_release_preflight.get("status") != "passed":
            raise RuntimeError(
                "expected security_release_preflight.status='passed', got "
                f"{security_release_preflight.get('status')!r}"
            )
        checks.append(
            {
                "name": "security_release_preflight_ready",
                "status": "pass",
                "detail": "rehearsal security release preflight passed with the seeded security posture report",
            }
        )
        vulnerability_exception_review_report_path = (
            output_root
            / "test_env"
            / "release_evidence"
            / "security"
            / "vulnerability_exception_review_report.json"
        )
        vulnerability_exception_review_payload = _load_json_payload(
            vulnerability_exception_review_report_path
        )
        vulnerability_exception_review = _summarize_vulnerability_exception_review(
            vulnerability_exception_review_payload,
            report_path=vulnerability_exception_review_report_path,
        )
        if vulnerability_exception_review.get("status") != "passed":
            raise RuntimeError(
                "expected vulnerability_exception_review.status='passed', got "
                f"{vulnerability_exception_review.get('status')!r}"
            )
        checks.append(
            {
                "name": "vulnerability_exception_review_ready",
                "status": "pass",
                "detail": (
                    "rehearsal vulnerability exception review passed with the seeded "
                    "residual-risk review report"
                ),
            }
        )

        industrial_manifest_path = output_root / "release_manifest_industrial.json"
        _run_checked_command(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "build_release_artifact.py"),
                "--version",
                request.version,
                "--channel",
                "industrial",
                "--build-id",
                f"{request.build_id}-industrial",
                "--release-summary",
                "Industrial release rehearsal validation.",
                "--project-root",
                str(output_root),
                "--source-root",
                str(source_root),
                "--approval-status",
                "approved",
                "--approved-by",
                "release-manager",
                "--approved-at",
                _now_iso(),
                "--approval-notes",
                "industrial rehearsal signoff",
                "--output",
                str(industrial_manifest_path),
            ],
            cwd=PROJECT_ROOT,
            failure_message="industrial release manifest build failed",
        )
        industrial_manifest_payload = _load_json_payload(industrial_manifest_path)
        industrial_manifest_errors = validate_release_manifest_artifact(
            industrial_manifest_payload
        )
        if industrial_manifest_errors:
            raise RuntimeError("; ".join(industrial_manifest_errors))
        industrial_manifest = _summarize_industrial_manifest(
            industrial_manifest_payload,
            manifest_path=industrial_manifest_path,
        )
        if industrial_manifest.get("status") != "ready":
            raise RuntimeError(
                "expected industrial release_gate_status='ready', got "
                f"{industrial_manifest.get('status')!r}"
            )
        industrial_delivery_artifact_paths.append(
            {
                "name": industrial_manifest_path.name,
                "path": str(industrial_manifest_path),
            }
        )
        checks.append(
            {
                "name": "industrial_manifest_built",
                "status": "pass",
                "detail": "rehearsal built a ready industrial release manifest from the same seeded evidence set",
            }
        )

        industrial_readiness_root = (
            output_root / "test_env" / "industrial_release_readiness_ready"
        )
        industrial_readiness_report_path = (
            industrial_readiness_root / "industrial_release_readiness_report.json"
        )
        _run_checked_command(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "check_industrial_release_readiness.py"),
                "--project-root",
                str(output_root),
                "--source-root",
                str(source_root),
                "--current-version",
                request.version,
                "--approval-manifest",
                str(industrial_manifest_path),
                "--security-preflight-report",
                str(security_preflight_report_path),
                "--output-root",
                str(industrial_readiness_root),
            ],
            cwd=PROJECT_ROOT,
            failure_message="industrial release readiness build failed",
        )
        industrial_readiness_payload = _load_json_payload(
            industrial_readiness_report_path
        )
        industrial_release_readiness = _summarize_industrial_release_readiness(
            industrial_readiness_payload,
            report_path=industrial_readiness_report_path,
        )
        if industrial_release_readiness.get("status") != "ready":
            raise RuntimeError(
                "expected industrial_release_gate='ready', got "
                f"{industrial_release_readiness.get('status')!r}"
            )
        industrial_delivery_artifact_paths.append(
            {
                "name": industrial_readiness_report_path.name,
                "path": str(industrial_readiness_report_path),
            }
        )
        checks.append(
            {
                "name": "industrial_release_readiness_ready",
                "status": "pass",
                "detail": "rehearsal generated an approved industrial release readiness report with zero remaining next steps",
            }
        )

        industrial_promotion_root = (
            output_root / "test_env" / "industrial_promotion_ready"
        )
        industrial_promotion_report_path = (
            industrial_promotion_root / "industrial_promotion_checklist.json"
        )
        _run_checked_command(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "build_industrial_promotion_checklist.py"),
                "--project-root",
                str(output_root),
                "--source-root",
                str(source_root),
                "--current-version",
                request.version,
                "--approval-manifest",
                str(industrial_manifest_path),
                "--security-preflight-report",
                str(security_preflight_report_path),
                "--output-root",
                str(industrial_promotion_root),
            ],
            cwd=PROJECT_ROOT,
            failure_message="industrial promotion checklist build failed",
        )
        industrial_promotion_payload = _load_json_payload(
            industrial_promotion_report_path
        )
        industrial_promotion_checklist = _summarize_industrial_promotion_checklist(
            industrial_promotion_payload,
            report_path=industrial_promotion_report_path,
        )
        if industrial_promotion_checklist.get("status") != "ready":
            raise RuntimeError(
                "expected industrial promotion checklist to be ready, got "
                f"{industrial_promotion_checklist.get('status')!r}"
            )
        industrial_delivery_artifact_paths.append(
            {
                "name": industrial_promotion_report_path.name,
                "path": str(industrial_promotion_report_path),
            }
        )
        checks.append(
            {
                "name": "industrial_promotion_ready",
                "status": "pass",
                "detail": "rehearsal generated a ready industrial promotion checklist with zero blocking steps",
            }
        )

        industrial_bundle_path = (
            output_root
            / "test_env"
            / "release"
            / "customer_acceptance_bundle_industrial.json"
        )
        _run_checked_command(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "build_customer_acceptance_bundle.py"),
                "--manifest",
                str(industrial_manifest_path),
                "--project-root",
                str(output_root),
                "--output",
                str(industrial_bundle_path),
                "--readiness-report",
                str(industrial_readiness_report_path),
                "--promotion-checklist",
                str(industrial_promotion_report_path),
                "--security-posture-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "security_posture_report.json"
                ),
                "--sbom-artifact",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "sbom.json"
                ),
                "--python-vuln-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "python_vuln_scan_report.json"
                ),
                "--container-vuln-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "container_vuln_scan_report.json"
                ),
                "--vulnerability-exception-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "vulnerability_exception_report.json"
                ),
                "--backup-restore-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "backup_restore_rehearsal_report.json"
                ),
            ],
            cwd=PROJECT_ROOT,
            failure_message="industrial customer acceptance bundle build failed",
        )
        industrial_bundle_payload = _load_json_payload(industrial_bundle_path)
        industrial_customer_acceptance_bundle = _summarize_customer_acceptance_bundle(
            industrial_bundle_payload,
            bundle_path=industrial_bundle_path,
        )
        customer_external_bindings_closure = (
            _summarize_customer_external_bindings_closure(
                industrial_bundle_payload,
                bundle_path=industrial_bundle_path,
            )
        )
        external_mainline_execution_plan = _summarize_external_mainline_execution_plan(
            industrial_bundle_payload,
            bundle_path=industrial_bundle_path,
        )
        release_ops_execution = _summarize_bundle_release_ops_execution(
            industrial_bundle_payload,
            bundle_path=industrial_bundle_path,
        )
        if industrial_customer_acceptance_bundle.get("status") != "ready":
            raise RuntimeError(
                "expected industrial customer acceptance bundle to be ready, got "
                f"{industrial_customer_acceptance_bundle.get('status')!r}"
            )
        industrial_delivery_artifact_paths.append(
            {"name": industrial_bundle_path.name, "path": str(industrial_bundle_path)}
        )
        checks.append(
            {
                "name": "industrial_customer_acceptance_bundle_ready",
                "status": "pass",
                "detail": "rehearsal generated a ready industrial customer acceptance bundle with industrial readiness and promotion reports attached",
            }
        )

        clean_checkout_report_path = (
            output_root
            / "test_env"
            / "release_evidence"
            / "clean_checkout_smoke_report.json"
        )
        clean_checkout_payload = _load_json_payload(clean_checkout_report_path)
        distributed_smoke_report_path = (
            output_root
            / "test_env"
            / "distributed_smoke"
            / "distributed_smoke_report.json"
        )
        distributed_smoke_payload = _load_json_payload(distributed_smoke_report_path)
        godot_headless_report_path = (
            output_root
            / "test_env"
            / "godot_headless_smoke"
            / "headless_smoke_report.json"
        )
        godot_headless_payload = _load_json_payload(godot_headless_report_path)
        ros2_bridge_report_path = (
            output_root
            / "test_env"
            / "ros2_bridge_smoke"
            / "ros2_bridge_smoke_report.json"
        )
        ros2_bridge_payload = _load_json_payload(ros2_bridge_report_path)
        backup_restore_report_path = (
            output_root
            / "test_env"
            / "release_evidence"
            / "security"
            / "backup_restore_rehearsal_report.json"
        )
        backup_restore_payload = _load_json_payload(backup_restore_report_path)
        extension_execution_actuals_path = (
            output_root
            / "test_env"
            / "release_evidence"
            / "operations"
            / "extension_execution_actuals.json"
        )

        delivery_rehearsal_stages = [
            {
                "id": "new_environment_install",
                "status": (
                    "pass"
                    if customer_delivery_status == "ready"
                    and industrial_customer_acceptance_bundle.get("status") == "ready"
                    else "fail"
                ),
                "summary": (
                    "Customer installation docs and the industrial acceptance bundle are ready for a fresh environment install."
                ),
                "artifact_paths": [
                    str(industrial_bundle_path),
                    str(
                        output_root
                        / "docs"
                        / "guides"
                        / "CUSTOMER_INSTALLATION_GUIDE.md"
                    ),
                    str(output_root / "docs" / "guides" / "DEPLOYMENT_MATRIX.md"),
                    str(output_root / "docs" / "guides" / "SUPPORT_MATRIX.md"),
                ],
            },
            {
                "id": "smoke",
                "status": (
                    "pass"
                    if clean_checkout_payload.get("status") == "passed"
                    and distributed_smoke_payload.get("status") == "passed"
                    else "fail"
                ),
                "summary": "Clean checkout smoke and distributed smoke evidence are both present and passing.",
                "artifact_paths": [
                    str(clean_checkout_report_path),
                    str(distributed_smoke_report_path),
                ],
            },
            {
                "id": "live_evidence",
                "status": (
                    "pass"
                    if security_release_preflight.get("status") == "passed"
                    and extension_execution_evidence.get("status") == "ready"
                    and godot_headless_payload.get("status") == "passed"
                    and ros2_bridge_payload.get("status") == "passed"
                    else "fail"
                ),
                "summary": "Live smoke, extension execution evidence, and security preflight are all ready for industrial delivery attestation.",
                "artifact_paths": [
                    str(godot_headless_report_path),
                    str(ros2_bridge_report_path),
                    str(security_preflight_report_path),
                    str(
                        output_root
                        / "test_env"
                        / "release_evidence"
                        / "operations"
                        / "extension_on_call_rehearsal_report.json"
                    ),
                    str(
                        output_root
                        / "test_env"
                        / "release_evidence"
                        / "operations"
                        / "extension_exception_review_schedule_report.json"
                    ),
                    str(
                        output_root
                        / "test_env"
                        / "release_evidence"
                        / "operations"
                        / "extension_escalation_closure_report.json"
                    ),
                ],
            },
            {
                "id": "upgrade",
                "status": (
                    "pass"
                    if industrial_manifest.get("status") == "ready"
                    and industrial_release_readiness.get("status") == "ready"
                    else "fail"
                ),
                "summary": "Industrial manifest generation and industrial readiness preview both closed for the rehearsal upgrade window.",
                "artifact_paths": [
                    str(industrial_manifest_path),
                    str(industrial_readiness_report_path),
                ],
            },
            {
                "id": "rollback",
                "status": (
                    "pass"
                    if industrial_promotion_checklist.get("status") == "ready"
                    and extension_execution_actuals.get("status") == "ready"
                    else "fail"
                ),
                "summary": "Rollback ownership, archive duties, and promotion prerequisites are closed for the rehearsal window.",
                "artifact_paths": [
                    str(industrial_promotion_report_path),
                    str(extension_execution_actuals_path),
                ],
            },
            {
                "id": "backup_restore",
                "status": (
                    "pass"
                    if backup_restore_payload.get("status") == "passed"
                    else "fail"
                ),
                "summary": "Backup and restore rehearsal passed and remains attached to the industrial acceptance surface.",
                "artifact_paths": [
                    str(backup_restore_report_path),
                    str(industrial_bundle_path),
                ],
            },
        ]
        provisional_release_rehearsal_report = {
            "status": "passed",
            "version": request.version,
            "tag": tag,
            "source_root": str(source_root),
            "git_source": git_source or {},
            "release_gate_status": release_gate_status,
            "customer_delivery_status": customer_delivery_status,
            "industrial_delivery_status": industrial_delivery_status,
            "security_release_preflight": security_release_preflight,
            "vulnerability_exception_review": vulnerability_exception_review,
            "customer_external_bindings_closure": customer_external_bindings_closure,
            "external_mainline_execution_plan": external_mainline_execution_plan,
            "external_mainline_input_checklist": external_mainline_input_checklist,
            "release_ops_execution": release_ops_execution,
            "industrial_manifest": industrial_manifest,
            "industrial_release_readiness": industrial_release_readiness,
            "industrial_promotion_checklist": industrial_promotion_checklist,
            "industrial_customer_acceptance_bundle": industrial_customer_acceptance_bundle,
            "extension_execution_plan": extension_execution_plan,
            "extension_execution_evidence": extension_execution_evidence,
            "extension_execution_instance": extension_execution_instance,
            "extension_execution_schedule": extension_execution_schedule,
            "extension_execution_actuals": extension_execution_actuals,
            "industrial_delivery_artifact_paths": industrial_delivery_artifact_paths,
            "delivery_rehearsal_stages": delivery_rehearsal_stages,
        }
        provisional_control_plane_session, provisional_control_plane_event_stream = (
            _aggregate_control_plane_surface(
                release_ops_execution,
                external_mainline_execution_plan,
                external_mainline_input_checklist,
            )
        )
        if provisional_control_plane_session:
            provisional_release_rehearsal_report["control_plane_session"] = (
                provisional_control_plane_session
            )
        if provisional_control_plane_event_stream:
            provisional_release_rehearsal_report["control_plane_event_stream"] = (
                provisional_control_plane_event_stream
            )
        provisional_industrial_delivery_report = (
            build_industrial_delivery_rehearsal_report_artifact(
                release_rehearsal_report=provisional_release_rehearsal_report,
                release_rehearsal_report_path=report_path,
            )
        )
        write_industrial_delivery_rehearsal_report_artifact(
            provisional_industrial_delivery_report,
            industrial_delivery_rehearsal_report_path,
        )
        industrial_delivery_artifact_paths.append(
            {
                "name": industrial_delivery_rehearsal_report_path.name,
                "path": str(industrial_delivery_rehearsal_report_path),
            }
        )
        _run_checked_command(
            [
                sys.executable,
                str(PROJECT_ROOT / "tools" / "build_customer_acceptance_bundle.py"),
                "--manifest",
                str(industrial_manifest_path),
                "--project-root",
                str(output_root),
                "--output",
                str(industrial_bundle_path),
                "--readiness-report",
                str(industrial_readiness_report_path),
                "--promotion-checklist",
                str(industrial_promotion_report_path),
                "--security-posture-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "security_posture_report.json"
                ),
                "--sbom-artifact",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "sbom.json"
                ),
                "--python-vuln-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "python_vuln_scan_report.json"
                ),
                "--container-vuln-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "container_vuln_scan_report.json"
                ),
                "--vulnerability-exception-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "vulnerability_exception_report.json"
                ),
                "--backup-restore-report",
                str(
                    output_root
                    / "test_env"
                    / "release_evidence"
                    / "security"
                    / "backup_restore_rehearsal_report.json"
                ),
                "--industrial-delivery-rehearsal-report",
                str(industrial_delivery_rehearsal_report_path),
            ],
            cwd=PROJECT_ROOT,
            failure_message="industrial customer acceptance bundle rebuild failed",
        )
        industrial_bundle_payload = _load_json_payload(industrial_bundle_path)
        industrial_customer_acceptance_bundle = _summarize_customer_acceptance_bundle(
            industrial_bundle_payload,
            bundle_path=industrial_bundle_path,
        )
        customer_external_bindings_closure = (
            _summarize_customer_external_bindings_closure(
                industrial_bundle_payload,
                bundle_path=industrial_bundle_path,
            )
        )
        external_mainline_execution_plan = _summarize_external_mainline_execution_plan(
            industrial_bundle_payload,
            bundle_path=industrial_bundle_path,
        )
        external_mainline_input_checklist = (
            _summarize_external_mainline_input_checklist(
                industrial_bundle_payload,
                bundle_path=industrial_bundle_path,
            )
        )
        release_ops_execution = _summarize_bundle_release_ops_execution(
            industrial_bundle_payload,
            bundle_path=industrial_bundle_path,
        )
        checks.append(
            {
                "name": "industrial_delivery_rehearsal_stages_ready",
                "status": "pass",
                "detail": "rehearsal report now records new_environment_install, smoke, live_evidence, upgrade, rollback, and backup_restore stages with explicit artifact paths",
            }
        )
        status = "passed"
    except Exception as exc:
        checks.append(
            {
                "name": "release_rehearsal",
                "status": "fail",
                "detail": str(exc),
            }
        )
        status = "failed"

    control_plane_session, control_plane_event_stream = (
        _aggregate_control_plane_surface(
            release_ops_execution,
            external_mainline_execution_plan,
            external_mainline_input_checklist,
        )
    )
    report = {
        "schema_version": "1.0",
        "artifact_type": "release_rehearsal_report",
        "status": status,
        "version": request.version,
        "tag": tag,
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "manifest_path": str(manifest_path),
        "report_path": str(report_path),
        "git_source": git_source,
        "release_gate_status": release_gate_status,
        "customer_delivery_status": customer_delivery_status,
        "industrial_delivery_status": industrial_delivery_status,
        "extension_execution_plan": extension_execution_plan,
        "extension_execution_evidence": extension_execution_evidence,
        "extension_execution_instance": extension_execution_instance,
        "extension_execution_schedule": extension_execution_schedule,
        "extension_execution_actuals": extension_execution_actuals,
        "security_release_preflight": security_release_preflight,
        "vulnerability_exception_review": vulnerability_exception_review,
        "customer_external_bindings_closure": customer_external_bindings_closure,
        "external_mainline_execution_plan": external_mainline_execution_plan,
        "external_mainline_input_checklist": external_mainline_input_checklist,
        "release_ops_execution": release_ops_execution,
        "industrial_manifest": industrial_manifest,
        "industrial_release_readiness": industrial_release_readiness,
        "industrial_promotion_checklist": industrial_promotion_checklist,
        "industrial_customer_acceptance_bundle": industrial_customer_acceptance_bundle,
        "industrial_delivery_artifact_paths": industrial_delivery_artifact_paths,
        "delivery_rehearsal_stages": delivery_rehearsal_stages,
        "document_paths": document_paths,
        "evidence_paths": evidence_paths,
        "checks": checks,
        "builder_stdout": builder_stdout,
        "builder_stderr": builder_stderr,
    }
    if control_plane_session:
        report["control_plane_session"] = control_plane_session
    if control_plane_event_stream:
        report["control_plane_event_stream"] = control_plane_event_stream
    written_report = _write_report(report_path, report)
    industrial_delivery_rehearsal_report = (
        build_industrial_delivery_rehearsal_report_artifact(
            release_rehearsal_report=report,
            release_rehearsal_report_path=written_report,
        )
    )
    written_industrial_delivery_report = (
        write_industrial_delivery_rehearsal_report_artifact(
            industrial_delivery_rehearsal_report,
            industrial_delivery_rehearsal_report_path,
        )
    )
    return ReleaseRehearsalResult(
        payload=report,
        report_path=written_report,
        industrial_delivery_rehearsal_payload=industrial_delivery_rehearsal_report,
        industrial_delivery_rehearsal_report_path=written_industrial_delivery_report,
        tag=tag,
        gate_status="ready" if status == "passed" else "blocked",
    )


__all__ = ["execute_release_rehearsal"]
