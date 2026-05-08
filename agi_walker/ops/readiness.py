"""Deterministic orchestration for stable and industrial release readiness."""

from __future__ import annotations

import json
import re
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agi_walker.core.api.release_contracts import (
    build_release_manifest_artifact,
    build_run_customer_external_bindings_closure_command,
    build_run_external_mainline_execution_plan_command,
    build_run_worktree_release_blocker_command,
    build_vulnerability_exception_review_report_command,
    default_customer_external_bindings_closure_report_path,
    default_external_mainline_execution_plan_path,
    default_external_mainline_input_checklist_report_path,
    resolve_customer_external_bindings_config_path,
    validate_external_mainline_execution_plan_artifact,
    validate_release_evidence_report,
    validate_release_manifest_artifact,
    write_release_manifest_artifact,
)
from agi_walker.core.api.release_ops_contracts import (
    IndustrialReleaseReadinessRequest,
    IndustrialReleaseReadinessResult,
    ReleaseReadinessRequest,
    ReleaseReadinessResult,
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _resolve_project_path(path: str, project_root: str) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return Path(project_root) / candidate


def _extract_release_notes_metadata(changelog_path: Path) -> dict[str, str]:
    if not changelog_path.is_file():
        return {}

    content = changelog_path.read_text(encoding="utf-8")
    title = ""
    summary = ""
    version = ""
    for line in content.splitlines():
        stripped = line.strip()
        if not title and stripped.startswith("# "):
            title = stripped[2:].strip()
            match = re.match(r"AGI-Walker\s+(.+?)\s+Release Notes$", title)
            if match:
                version = match.group(1).strip()
        if not summary and stripped.startswith("发布摘要："):
            summary = stripped.split("：", 1)[1].strip()
        if not summary and stripped.startswith("Release Summary:"):
            summary = stripped.split(":", 1)[1].strip()
        if title and summary and version:
            break

    metadata: dict[str, str] = {}
    if title:
        metadata["title"] = title
    if summary:
        metadata["release_summary"] = summary
    if version:
        metadata["version"] = version
    return metadata


def _derive_stable_version(current_version: str) -> str:
    stripped = re.sub(r"-(?:rc\d+|dev[\w.-]*)$", "", current_version)
    return stripped or current_version


def _default_build_id(channel: str, version: str) -> str:
    normalized_version = re.sub(r"[^A-Za-z0-9._-]+", "-", version)
    return f"{channel}-readiness-{normalized_version}"


def _default_security_preflight_command() -> str:
    return (
        "python tools/run_security_release_preflight.py "
        "--output-root test_env/release_evidence "
        "--report-file test_env/release_evidence/security_release_preflight_report.json"
    )


def _default_vulnerability_exception_review_command() -> str:
    return build_vulnerability_exception_review_report_command()


def _resolve_security_preflight_path(
    security_preflight_report: str | None,
    *,
    project_root: str,
) -> Path:
    if security_preflight_report:
        return _resolve_project_path(security_preflight_report, project_root)
    return (
        Path(project_root)
        / "test_env"
        / "release_evidence"
        / "security_release_preflight_report.json"
    )


def _build_release_approval_payload(
    *,
    approval_status: str | None,
    approved_by: str | None,
    approved_at: str | None,
    commit_sha: str | None,
    approval_notes: str | None,
) -> dict[str, str | None] | None:
    if not any(
        [
            approval_status,
            approved_by,
            approved_at,
            commit_sha,
            approval_notes,
        ]
    ):
        return None

    return {
        "status": approval_status,
        "approved_by": approved_by,
        "approved_at": approved_at,
        "commit_sha": commit_sha,
        "notes": approval_notes,
    }


def _load_approval_manifest(
    path: str,
    *,
    project_root: str,
    expected_channel: str,
    expected_version: str,
    version_flag: str,
) -> tuple[dict[str, Any], Path]:
    manifest_path = _resolve_project_path(path, project_root)
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    if payload.get("artifact_type") != "release_manifest":
        raise ValueError(
            f"--approval-manifest must point to a release_manifest artifact: {manifest_path}"
        )
    if payload.get("channel") != expected_channel:
        raise ValueError(
            f"--approval-manifest must point to a {expected_channel} manifest: {manifest_path}"
        )
    if payload.get("version") != expected_version:
        raise ValueError(
            f"--approval-manifest version does not match --{version_flag}: "
            f"{payload.get('version')} != {expected_version}"
        )
    if not isinstance(payload.get("release_approval"), dict):
        raise ValueError(
            f"--approval-manifest is missing a valid release_approval object: {manifest_path}"
        )
    return payload, manifest_path


def _resolve_release_approval(
    *,
    project_root: str,
    expected_channel: str,
    expected_version: str,
    version_flag: str,
    approval_manifest: str | None,
    approval_status: str | None,
    approved_by: str | None,
    approved_at: str | None,
    commit_sha: str | None,
    approval_notes: str | None,
) -> tuple[dict[str, str | None] | None, str | None]:
    explicit = _build_release_approval_payload(
        approval_status=approval_status,
        approved_by=approved_by,
        approved_at=approved_at,
        commit_sha=commit_sha,
        approval_notes=approval_notes,
    )
    if not approval_manifest:
        return explicit, None

    manifest_payload, manifest_path = _load_approval_manifest(
        approval_manifest,
        project_root=project_root,
        expected_channel=expected_channel,
        expected_version=expected_version,
        version_flag=version_flag,
    )
    approval = dict(manifest_payload["release_approval"])
    if explicit is not None:
        approval.update(
            {key: value for key, value in explicit.items() if value is not None}
        )
    return approval, str(manifest_path)


def _load_security_preflight_preview(report_path: Path) -> dict[str, Any]:
    preview = {
        "name": "security_release_preflight",
        "status": "missing",
        "summary": f"security release preflight report is missing: {report_path}",
        "command": _default_security_preflight_command(),
        "report_path": str(report_path),
        "required_for_stable": True,
    }
    if not report_path.is_file():
        return preview

    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - defensive
        preview["status"] = "blocked"
        preview["summary"] = f"security release preflight report is unreadable: {exc}"
        return preview

    errors = validate_release_evidence_report(payload)
    if payload.get("evidence_name") != "security_release_preflight":
        errors.append(
            "security release preflight report must use evidence_name='security_release_preflight'"
        )
    if errors:
        preview["status"] = "blocked"
        preview["summary"] = (
            "security release preflight report is invalid: " + "; ".join(errors)
        )
        return preview

    preview["status"] = payload.get("status") or "blocked"
    preview["summary"] = (
        payload.get("summary") or "security release preflight has no summary."
    )
    preview["command"] = payload.get("command") or _default_security_preflight_command()
    preview["generated_at"] = payload.get("generated_at")
    preview["metrics"] = payload.get("metrics", {})
    return preview


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _coerce_non_negative_int(value: Any) -> int:
    return value if isinstance(value, int) and value >= 0 else 0


def _coerce_non_empty_strings(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item).strip() for item in value if _is_non_empty_string(item)]


def _build_stale_vulnerability_exception_action(
    security_preflight: Mapping[str, Any],
) -> str | None:
    metrics = security_preflight.get("metrics")
    if not isinstance(metrics, Mapping):
        return None
    stale_exception_count = _coerce_non_negative_int(
        metrics.get("stale_vulnerability_exceptions")
    )
    if stale_exception_count == 0:
        return None
    stale_exception_ids = _coerce_non_empty_strings(
        metrics.get("stale_vulnerability_exception_ids")
    )
    stale_preview = ", ".join(stale_exception_ids[:3])
    if stale_exception_count > 3:
        stale_preview += ", ..."
    return (
        "替换已失效的 no-fix vulnerability exceptions: "
        "更新 deployment/security/vulnerability_exceptions.input.json，"
        f"stale={stale_exception_count}"
        + (f"（例如 {stale_preview}）" if stale_preview else "")
        + f"，然后重跑 {_default_security_preflight_command()}"
    )


def _build_vulnerability_exception_review_action(
    security_preflight: Mapping[str, Any],
) -> str | None:
    if security_preflight.get("status") != "passed":
        return None
    metrics = security_preflight.get("metrics")
    if not isinstance(metrics, Mapping):
        return None
    review_status = str(
        metrics.get("vulnerability_exception_review_status") or ""
    ).strip()
    review_due = _coerce_non_negative_int(
        metrics.get("vulnerability_exception_review_due")
    )
    expired = _coerce_non_negative_int(metrics.get("expired_vulnerability_exceptions"))
    review_report_status = str(
        metrics.get("vulnerability_exception_review_report_status") or ""
    ).strip()
    review_candidate_count = _coerce_non_negative_int(
        metrics.get("vulnerability_exception_review_candidate_count")
    )
    review_report_path = str(
        metrics.get("vulnerability_exception_review_report_path") or ""
    ).strip()
    review_due_ids = _coerce_non_empty_strings(
        metrics.get("review_due_vulnerability_exception_ids")
    )
    review_due_tickets = _coerce_non_empty_strings(
        metrics.get("review_due_vulnerability_exception_tickets")
    )
    expired_ids = _coerce_non_empty_strings(
        metrics.get("expired_vulnerability_exception_ids")
    )
    next_expiry = str(metrics.get("vulnerability_exception_next_expiry") or "").strip()
    review_command = _default_vulnerability_exception_review_command()
    review_path_suffix = (
        f"（report={review_report_path}）" if review_report_path else ""
    )
    review_status_suffix = f"status={review_report_status or 'unknown'}，candidates={review_candidate_count}"
    review_due_preview = ", ".join(review_due_ids[:3])
    if len(review_due_ids) > 3:
        review_due_preview += ", ..."
    review_ticket_preview = ", ".join(review_due_tickets[:3])
    if len(review_due_tickets) > 3:
        review_ticket_preview += ", ..."
    expired_preview = ", ".join(expired_ids[:3])
    if len(expired_ids) > 3:
        expired_preview += ", ..."
    if review_status == "expired" or expired > 0:
        return (
            "先复核独立 vulnerability exception review report: "
            f"{review_command}，{review_status_suffix}{review_path_suffix}；"
            "然后立即刷新或移除已过期的 vulnerability exceptions，"
            "更新 deployment/security/vulnerability_exceptions.input.json，"
            + (f"expired={expired}，" if expired else "")
            + (f"例如 {expired_preview}，" if expired_preview else "")
            + f"最后重跑 {_default_security_preflight_command()}"
        )
    if review_status == "review_due" and review_due > 0:
        return (
            "先复核独立 vulnerability exception review report: "
            f"{review_command}，{review_status_suffix}{review_path_suffix}；"
            "然后在 "
            f"{next_expiry or '当前 exception expiry deadline'} 前更新 deployment/security/vulnerability_exceptions.input.json，"
            + (f"review_due={review_due}，" if review_due else "")
            + (f"例如 {review_due_preview}，" if review_due_preview else "")
            + (f"tickets={review_ticket_preview}，" if review_ticket_preview else "")
            + f"最后重跑 {_default_security_preflight_command()}"
        )
    return None


def _load_customer_external_bindings_closure_preview(
    project_root: str,
) -> dict[str, Any] | None:
    report_path = _resolve_project_path(
        default_customer_external_bindings_closure_report_path(),
        project_root,
    )
    if not report_path.is_file():
        return None
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - defensive
        return {
            "status": "blocked",
            "summary": f"customer external bindings closure report is unreadable: {exc}",
            "report_path": str(report_path),
            "failed_steps": [],
        }
    errors = validate_release_evidence_report(payload)
    if payload.get("evidence_name") != "customer_external_bindings_closure":
        errors.append(
            "customer external bindings closure report must use "
            "evidence_name='customer_external_bindings_closure'"
        )
    if errors:
        return {
            "status": "blocked",
            "summary": (
                "customer external bindings closure report is invalid: "
                + "; ".join(errors)
            ),
            "report_path": str(report_path),
            "failed_steps": [],
        }
    metrics = payload.get("metrics", {})
    failed_steps = (
        _coerce_non_empty_strings(metrics.get("failed_steps"))
        if isinstance(metrics, Mapping)
        else []
    )
    return {
        "status": payload.get("status") or "blocked",
        "summary": payload.get("summary")
        or "customer external bindings closure report has no summary.",
        "report_path": str(report_path),
        "failed_steps": failed_steps,
    }


def _build_external_bindings_follow_up_action(
    project_root: str,
    extension_execution_actuals: Mapping[str, Any],
    extension_execution_instance: Mapping[str, Any],
) -> str | None:
    if (
        extension_execution_actuals.get("external_bindings_follow_up_required")
        is not True
    ):
        return None
    status = str(
        extension_execution_actuals.get("external_bindings_status") or ""
    ).strip()
    if status not in {"placeholder", "partial"}:
        return None

    config_path = None
    external_bindings = extension_execution_actuals.get("external_bindings")
    if isinstance(external_bindings, Mapping):
        candidate = external_bindings.get("config_path")
        if _is_non_empty_string(candidate):
            config_path = str(candidate).strip()

    customer_config_path = resolve_customer_external_bindings_config_path(config_path)
    placeholder_sections = _coerce_non_empty_strings(
        extension_execution_actuals.get("external_bindings_placeholder_sections")
    )
    missing_sections = _coerce_non_empty_strings(
        extension_execution_actuals.get("external_bindings_missing_sections")
    )
    draft_sections = _coerce_non_empty_strings(
        extension_execution_actuals.get("external_bindings_draft_sections")
    )
    unconfirmed_sections = _coerce_non_empty_strings(
        extension_execution_actuals.get("external_bindings_unconfirmed_sections")
    )
    confirmation_missing_sections = _coerce_non_empty_strings(
        extension_execution_actuals.get(
            "external_bindings_confirmation_missing_sections"
        )
    )
    is_existing_customer_config = _is_non_empty_string(
        config_path
    ) and config_path not in {
        "deployment/customer_delivery.external_bindings.json",
        "deployment/customer_delivery.external_bindings.rehearsal.json",
    }

    section_details: list[str] = []
    if placeholder_sections:
        section_details.append("placeholder=" + ", ".join(placeholder_sections))
    if missing_sections:
        section_details.append("missing=" + ", ".join(missing_sections))
    if draft_sections:
        section_details.append("draft=" + ", ".join(draft_sections))
    if unconfirmed_sections:
        section_details.append("unconfirmed=" + ", ".join(unconfirmed_sections))
    if confirmation_missing_sections:
        section_details.append(
            "confirmation_missing=" + ", ".join(confirmation_missing_sections)
        )

    confirm_sections = (
        draft_sections
        or confirmation_missing_sections
        or unconfirmed_sections
        or placeholder_sections
        or missing_sections
    )
    closure_command = build_run_customer_external_bindings_closure_command(
        config_path=customer_config_path,
        instance_artifact_path=extension_execution_instance.get("artifact_path"),
        actuals_artifact_path=extension_execution_actuals.get("artifact_path"),
        sections=confirm_sections,
    )
    action = (
        "将 extension_execution_actuals 的 external bindings 收口到真实客户系统映射: "
    )
    if is_existing_customer_config:
        action += (
            "先把现有 customer-specific config 中的真实客户系统元数据补齐，再运行以下闭环命令: "
            + closure_command
        )
    else:
        action += (
            "当前还没有 customer-specific config。直接运行以下闭环命令会先生成 draft config；"
            "如果你同时提供 `--set section.field=value` 覆盖，它会继续 confirm / rebuild / collect，"
            "否则会在生成后阻塞等待你补齐真实客户系统元数据: " + closure_command
        )
    if _is_non_empty_string(
        extension_execution_actuals.get("external_bindings_summary")
    ):
        action += "。当前状态: " + str(
            extension_execution_actuals.get("external_bindings_summary")
        )
    if section_details:
        action += "。待处理 sections: " + "; ".join(section_details)
    closure_preview = _load_customer_external_bindings_closure_preview(project_root)
    if closure_preview is not None:
        action += "。最近一次 closure report: " + str(closure_preview.get("summary"))
        failed_steps = _coerce_non_empty_strings(closure_preview.get("failed_steps"))
        if failed_steps:
            action += "。failed_steps=" + ", ".join(failed_steps)
        report_path = closure_preview.get("report_path")
        if _is_non_empty_string(report_path):
            action += f"。report={report_path}"
    return action


def _build_external_mainline_execution_plan_preview(
    *,
    project_root: str,
) -> dict[str, Any]:
    report_path = _resolve_project_path(
        default_external_mainline_execution_plan_path(),
        project_root,
    )
    preview: dict[str, Any] = {
        "status": "missing",
        "report_path": str(report_path),
        "summary": f"external mainline execution plan is missing: {report_path}",
    }
    if not report_path.is_file():
        return preview
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - defensive
        preview["status"] = "blocked"
        preview["summary"] = f"external mainline execution plan is unreadable: {exc}"
        return preview
    errors = validate_external_mainline_execution_plan_artifact(payload)
    if errors:
        preview["status"] = "blocked"
        preview["summary"] = (
            "external mainline execution plan is invalid: " + "; ".join(errors)
        )
        return preview
    preview["status"] = str(payload.get("status") or "missing").strip() or "missing"
    counts: list[str] = []
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = payload.get(field)
        if isinstance(value, int) and value >= 0:
            preview[field] = value
            counts.append(f"{field}={value}")
    preview["summary"] = (
        "external mainline execution plan "
        f"{preview['status']}: " + ", ".join(counts + [f"report={report_path}"]) + "."
    )
    return preview


def _build_external_mainline_input_checklist_preview(
    *,
    project_root: str,
) -> dict[str, Any]:
    report_path = _resolve_project_path(
        default_external_mainline_input_checklist_report_path(),
        project_root,
    )
    preview: dict[str, Any] = {
        "status": "missing",
        "report_path": str(report_path),
        "summary": f"external mainline input checklist is missing: {report_path}",
    }
    if not report_path.is_file():
        return preview
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - defensive
        preview["status"] = "blocked"
        preview["summary"] = f"external mainline input checklist is unreadable: {exc}"
        return preview
    errors = validate_release_evidence_report(payload)
    if payload.get("evidence_name") != "external_mainline_input_checklist":
        errors.append(
            "external mainline input checklist must use "
            "evidence_name='external_mainline_input_checklist'"
        )
    if errors:
        preview["status"] = "blocked"
        preview["summary"] = (
            "external mainline input checklist is invalid: " + "; ".join(errors)
        )
        return preview
    preview["status"] = str(payload.get("status") or "missing").strip() or "missing"
    control_plane_session = payload.get("control_plane_session")
    if isinstance(control_plane_session, Mapping):
        preview["control_plane_session"] = dict(control_plane_session)
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if isinstance(control_plane_event_stream, Mapping):
        preview["control_plane_event_stream"] = dict(control_plane_event_stream)
    metrics = (
        payload.get("metrics") if isinstance(payload.get("metrics"), Mapping) else {}
    )
    missing_input_count = metrics.get("missing_input_count")
    if isinstance(missing_input_count, int) and missing_input_count >= 0:
        preview["missing_input_count"] = missing_input_count
    counts: list[str] = []
    for source_field, preview_field in {
        "waiting_external_input_steps": "waiting_external_input_steps",
        "ready_to_run_steps": "ready_to_run_steps",
        "completed_steps": "completed_steps",
    }.items():
        value = metrics.get(source_field)
        if isinstance(value, list):
            preview[preview_field] = len(
                [str(item).strip() for item in value if _is_non_empty_string(item)]
            )
        count = preview.get(preview_field)
        if isinstance(count, int) and count >= 0:
            counts.append(f"{preview_field}={count}")
    if "missing_input_count" in preview:
        counts.insert(0, f"missing_input_count={preview['missing_input_count']}")
    preview["summary"] = (
        "external mainline input checklist "
        f"{preview['status']}: " + ", ".join(counts + [f"report={report_path}"]) + "."
    )
    return preview


def _build_worktree_release_blocker_preview(
    *,
    release_source: Mapping[str, Any],
    output_root: Path,
) -> dict[str, Any]:
    report_path = output_root / "worktree_release_blocker_report.json"
    clean_worktree = release_source.get("worktree_clean") is True
    preview: dict[str, Any] = {
        "status": "ready" if clean_worktree else "blocked",
        "report_path": str(report_path),
        "clean_worktree": clean_worktree,
        "summary": (
            "worktree release blocker ready: clean_worktree=true, total_paths=0, tracked_review=not_required."
            if clean_worktree
            else "worktree release blocker report is missing."
        ),
    }
    if clean_worktree:
        preview["total_paths"] = 0
        preview["tracked_review_candidate_count"] = 0
    else:
        worktree_status_summary = release_source.get("worktree_status_summary")
        if isinstance(worktree_status_summary, str) and worktree_status_summary.strip():
            preview["summary"] = (
                "worktree release blocker blocked: "
                f"{worktree_status_summary.strip()}."
            )
    if not report_path.is_file():
        return preview
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:  # pragma: no cover - defensive
        preview["status"] = "blocked"
        preview["summary"] = f"worktree release blocker report is unreadable: {exc}"
        return preview
    required_fields = {
        "artifact_type",
        "status",
        "summary",
        "clean_worktree",
        "total_paths",
        "tracked_review_candidate_count",
    }
    if (
        not isinstance(payload, dict)
        or payload.get("artifact_type") != "worktree_release_blocker_report"
        or not required_fields.issubset(payload)
    ):
        preview["status"] = "blocked"
        preview["summary"] = "worktree release blocker report is invalid."
        return preview
    preview["status"] = str(payload.get("status") or "blocked").strip() or "blocked"
    preview["summary"] = (
        str(payload.get("summary")).strip()
        if isinstance(payload.get("summary"), str) and payload.get("summary").strip()
        else "worktree release blocker summary is unavailable."
    )
    preview["clean_worktree"] = payload.get("clean_worktree") is True
    total_paths = payload.get("total_paths")
    if isinstance(total_paths, int) and total_paths >= 0:
        preview["total_paths"] = total_paths
    tracked_review_candidate_count = payload.get("tracked_review_candidate_count")
    if (
        isinstance(tracked_review_candidate_count, int)
        and tracked_review_candidate_count >= 0
    ):
        preview["tracked_review_candidate_count"] = tracked_review_candidate_count
    tracked_review_status = payload.get("tracked_review_status")
    if isinstance(tracked_review_status, str) and tracked_review_status.strip():
        preview["tracked_review_status"] = tracked_review_status.strip()
    control_plane_session = payload.get("control_plane_session")
    if isinstance(control_plane_session, Mapping):
        preview["control_plane_session"] = dict(control_plane_session)
    control_plane_event_stream = payload.get("control_plane_event_stream")
    if isinstance(control_plane_event_stream, Mapping):
        preview["control_plane_event_stream"] = dict(control_plane_event_stream)
    failed_steps = payload.get("failed_steps")
    if isinstance(failed_steps, list):
        preview["failed_steps"] = [
            str(item).strip()
            for item in failed_steps
            if isinstance(item, str) and item.strip()
        ]
    return preview


def _build_vulnerability_exception_review_preview(
    security_preflight: Mapping[str, Any],
) -> dict[str, Any]:
    metrics = security_preflight.get("metrics")
    preview: dict[str, Any] = {
        "status": "missing",
        "summary": (
            "vulnerability exception review report state is unavailable from "
            "security release preflight metrics."
        ),
    }
    if not isinstance(metrics, Mapping):
        return preview
    review_status = str(
        metrics.get("vulnerability_exception_review_report_status") or ""
    ).strip()
    review_candidate_count = metrics.get(
        "vulnerability_exception_review_candidate_count"
    )
    review_report_path = str(
        metrics.get("vulnerability_exception_review_report_path") or ""
    ).strip()
    if review_status:
        preview["status"] = review_status
    if isinstance(review_candidate_count, int) and review_candidate_count >= 0:
        preview["review_candidate_count"] = review_candidate_count
    if review_report_path:
        preview["report_path"] = review_report_path
    preview_summary = f"vulnerability exception review {preview['status']}"
    if "review_candidate_count" in preview:
        preview_summary += f": candidates={preview['review_candidate_count']}"
    if review_report_path:
        preview_summary += f", report={review_report_path}"
    preview["summary"] = preview_summary + "."
    return preview


def format_external_mainline_execution_plan_preview(
    preview: Mapping[str, Any],
) -> str:
    status = str(preview.get("status") or "missing").strip() or "missing"
    counts: list[str] = []
    for field in [
        "completed_steps",
        "ready_to_run_steps",
        "waiting_external_input_steps",
        "blocked_steps",
    ]:
        value = preview.get(field)
        if not isinstance(value, int) or value < 0:
            counts = []
            break
        counts.append(str(value))
    return "/".join([status, *counts]) if counts else status


def format_external_mainline_input_checklist_preview(
    preview: Mapping[str, Any],
) -> str:
    status = str(preview.get("status") or "missing").strip() or "missing"
    counts: list[str] = []
    for field in [
        "missing_input_count",
        "waiting_external_input_steps",
        "ready_to_run_steps",
        "completed_steps",
    ]:
        value = preview.get(field)
        if not isinstance(value, int) or value < 0:
            counts = []
            break
        counts.append(str(value))
    return "/".join([status, *counts]) if counts else status


def format_worktree_release_blocker_preview(preview: Mapping[str, Any]) -> str:
    status = str(preview.get("status") or "missing").strip() or "missing"
    total_paths = preview.get("total_paths")
    tracked_review_candidate_count = preview.get("tracked_review_candidate_count")
    if (
        isinstance(total_paths, int)
        and total_paths >= 0
        and isinstance(tracked_review_candidate_count, int)
        and tracked_review_candidate_count >= 0
    ):
        return f"{status}/{total_paths}/{tracked_review_candidate_count}"
    return status


def _build_channel_readiness_summary(
    *,
    label: str,
    preview: Mapping[str, Any],
) -> str:
    gate = preview["release_gate_status"]
    security_preflight = preview["security_release_preflight"]
    vulnerability_exception_review = preview.get("vulnerability_exception_review", {})
    external_mainline_execution_plan = preview.get(
        "external_mainline_execution_plan", {}
    )
    external_mainline_input_checklist = preview.get(
        "external_mainline_input_checklist", {}
    )
    worktree_release_blocker = preview.get("worktree_release_blocker", {})
    next_actions = preview["next_actions"]
    review_status = str(
        vulnerability_exception_review.get("status") or "missing"
    ).strip()
    review_candidate_count = vulnerability_exception_review.get(
        "review_candidate_count"
    )
    review_summary = (
        f"{review_status}/{review_candidate_count}"
        if isinstance(review_candidate_count, int) and review_candidate_count >= 0
        else review_status
    )
    external_mainline_summary = format_external_mainline_execution_plan_preview(
        external_mainline_execution_plan
    )
    checklist_summary = format_external_mainline_input_checklist_preview(
        external_mainline_input_checklist
    )
    worktree_summary = format_worktree_release_blocker_preview(worktree_release_blocker)
    checklist_status = str(
        external_mainline_input_checklist.get("status") or ""
    ).strip()
    checklist_suffix = (
        f", external_mainline_input_checklist={checklist_summary}"
        if checklist_status and checklist_status != "missing"
        else ""
    )
    if gate == "ready" and security_preflight.get("status") == "passed":
        return (
            f"{label} release readiness is ready: "
            f"exception_review={review_summary}, "
            f"external_mainline={external_mainline_summary}, "
            f"worktree={worktree_summary}"
            f"{checklist_suffix}."
        )
    return (
        f"{label} release readiness remains blocked: "
        f"release_gate={gate}, "
        f"security_preflight={security_preflight.get('status')}, "
        f"exception_review={review_summary}, "
        f"external_mainline={external_mainline_summary}, "
        f"worktree={worktree_summary}"
        f"{checklist_suffix}, "
        f"next_actions={len(next_actions)}."
    )


def _build_preview(
    *,
    channel: str,
    version: str,
    build_id: str,
    project_root: str,
    source_root: str,
    changelog: str,
    changelog_title: str,
    release_summary: str,
    output_root: Path,
    source_root_hint: str,
    security_preflight: Mapping[str, Any],
    release_approval: dict[str, str | None] | None = None,
) -> dict[str, Any]:
    payload = build_release_manifest_artifact(
        build_id=build_id,
        version=version,
        channel=channel,
        release_summary=release_summary,
        changelog_path=changelog,
        changelog_title=changelog_title,
        release_approval=release_approval,
        project_root=project_root,
        source_root=source_root,
    )
    manifest_path = write_release_manifest_artifact(
        payload,
        output_root / f"{channel}_release_manifest.json",
    )
    validation_errors = validate_release_manifest_artifact(payload)
    worktree_release_blocker = _build_worktree_release_blocker_preview(
        release_source=payload["release_source"],
        output_root=output_root,
    )
    return {
        "channel": channel,
        "version": version,
        "build_id": build_id,
        "manifest_path": str(manifest_path),
        "release_gate_status": payload["release_gate_status"],
        "release_gate": payload["release_gate"],
        "release_policy": payload["release_policy"],
        "release_source": payload["release_source"],
        "validation_errors": validation_errors,
        "security_release_preflight": dict(security_preflight),
        "customer_delivery_surface": dict(payload.get("customer_delivery_surface", {})),
        "industrial_delivery_gate": dict(payload.get("industrial_delivery_gate", {})),
        "extension_execution_evidence": dict(
            payload.get("extension_execution_evidence", {})
        ),
        "extension_execution_instance": dict(
            payload.get("extension_execution_instance", {})
        ),
        "extension_execution_schedule": dict(
            payload.get("extension_execution_schedule", {})
        ),
        "extension_execution_actuals": dict(
            payload.get("extension_execution_actuals", {})
        ),
        "worktree_release_blocker": worktree_release_blocker,
        "external_mainline_execution_plan": _build_external_mainline_execution_plan_preview(
            project_root=project_root
        ),
        "next_actions": _build_next_actions(
            payload,
            project_root=project_root,
            source_root_hint=source_root_hint,
            output_root=output_root,
            security_preflight=security_preflight,
            worktree_release_blocker=worktree_release_blocker,
            customer_delivery_surface=payload.get("customer_delivery_surface", {}),
            industrial_delivery_gate=payload.get("industrial_delivery_gate", {}),
            extension_execution_evidence=payload.get(
                "extension_execution_evidence", {}
            ),
            extension_execution_instance=payload.get(
                "extension_execution_instance", {}
            ),
            extension_execution_schedule=payload.get(
                "extension_execution_schedule", {}
            ),
            extension_execution_actuals=payload.get("extension_execution_actuals", {}),
        ),
    }


def _build_next_actions(
    payload: dict[str, Any],
    *,
    project_root: str,
    source_root_hint: str,
    output_root: Path,
    security_preflight: Mapping[str, Any],
    worktree_release_blocker: Mapping[str, Any],
    customer_delivery_surface: Mapping[str, Any],
    industrial_delivery_gate: Mapping[str, Any],
    extension_execution_evidence: Mapping[str, Any],
    extension_execution_instance: Mapping[str, Any],
    extension_execution_schedule: Mapping[str, Any],
    extension_execution_actuals: Mapping[str, Any],
) -> list[str]:
    actions: list[str] = []
    channel = payload["channel"]
    channel_label = "stable" if channel == "stable" else channel
    version = payload["version"]
    gate = payload["release_gate"]
    source = payload["release_source"]

    for item in payload.get("test_evidence", []):
        if item.get("status") == "passed":
            continue
        command = item.get("command")
        if item.get("status") == "blocked":
            actions.append(f"修复或重跑 {item.get('name')}: {command}")
        elif item.get("status") == "opt_in":
            actions.append(f"补齐可选 live 证据 {item.get('name')}: {command}")

    if gate.get("diagnostic_ready_domains", 0) > 0:
        for domain in payload.get("capability_matrix", {}).get("domains", []):
            if domain.get("status") == "diagnostic_ready":
                actions.append(
                    f"关闭诊断态域 {domain.get('id')}: {domain.get('summary')}"
                )

    if (
        gate.get("release_approval_required", 0) > 0
        and gate.get("release_approval_ready", 0) == 0
    ):
        actions.append(
            f"补齐 {channel_label} 签核: "
            f"python tools/build_release_artifact.py --version {version} --channel {channel} "
            f'--build-id {payload["build_id"]} --approval-status approved --approved-by release-manager '
            f'--approved-at {_now_iso()} --approval-notes "{channel_label} signoff"'
        )

    if (
        gate.get("release_source_required", 0) > 0
        and gate.get("release_source_ready", 0) == 0
    ):
        actions.append("确保在目标 Git 检出上执行，并让签核 commit 与当前 HEAD 一致。")

    if (
        gate.get("release_worktree_required", 0) > 0
        and gate.get("release_worktree_ready", 0) == 0
    ):
        summary = worktree_release_blocker.get("summary")
        actions.append(
            "先运行 worktree release blocker runner: "
            + build_run_worktree_release_blocker_command(
                source_root=source_root_hint,
                output_root=output_root,
            )
            + (
                f"。当前状态: {summary}"
                if isinstance(summary, str) and summary.strip()
                else ""
            )
        )

    if (
        gate.get("release_version_tag_required", 0) > 0
        and gate.get("release_version_tag_ready", 0) == 0
    ):
        head = source.get("commit_sha") or "HEAD"
        actions.append(f"为当前 HEAD 创建匹配版本 tag: git tag {version} {head}")

    stale_action = _build_stale_vulnerability_exception_action(security_preflight)
    if stale_action:
        actions.append(stale_action)

    if (
        channel in {"stable", "industrial"}
        and security_preflight.get("status") != "passed"
    ):
        summary = security_preflight.get("summary")
        command = (
            security_preflight.get("command") or _default_security_preflight_command()
        )
        actions.append(
            "补齐 security release preflight: "
            f"{command}"
            + (f"。当前状态: {summary}" if _is_non_empty_string(summary) else "")
        )

    review_action = _build_vulnerability_exception_review_action(security_preflight)
    if review_action:
        actions.append(review_action)

    if customer_delivery_surface.get("status") != "ready":
        missing_documents = [
            str(item).strip()
            for item in customer_delivery_surface.get("missing_required_documents", [])
            if _is_non_empty_string(item)
        ]
        actions.append(
            "补齐客户交付文档集: "
            + (
                ", ".join(missing_documents)
                if missing_documents
                else "README、状态页和 Phase E 文档需要重新核对。"
            )
        )

    if industrial_delivery_gate.get("status") != "ready":
        missing_requirements = [
            str(item).strip()
            for item in industrial_delivery_gate.get("missing_requirements", [])
            if _is_non_empty_string(item)
        ]
        actions.append(
            "补齐 industrial delivery gate: "
            + (
                ", ".join(missing_requirements)
                if missing_requirements
                else "部署包文档、结构化 evidence 或安全产物需要重新核对。"
            )
        )

    if extension_execution_instance.get("status") != "ready":
        missing_profiles = [
            str(item).strip()
            for item in extension_execution_instance.get("missing_profiles", [])
            if _is_non_empty_string(item)
        ]
        actions.append(
            "生成客户实例化扩展执行面: "
            "python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json"
            + (
                "。缺失 profile: " + ", ".join(missing_profiles)
                if missing_profiles
                else ""
            )
        )

    if extension_execution_schedule.get("status") != "ready":
        missing_profiles = [
            str(item).strip()
            for item in extension_execution_schedule.get("missing_profiles", [])
            if _is_non_empty_string(item)
        ]
        actions.append(
            "生成客户窗口排程与 closure 归档面: "
            "python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json"
            + (
                "。缺失 profile: " + ", ".join(missing_profiles)
                if missing_profiles
                else ""
            )
        )

    if extension_execution_actuals.get("status") != "ready":
        missing_profiles = [
            str(item).strip()
            for item in extension_execution_actuals.get("missing_profiles", [])
            if _is_non_empty_string(item)
        ]
        actions.append(
            "生成客户窗口执行留痕与 closure manifest: "
            "python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json"
            + (
                "。缺失 profile: " + ", ".join(missing_profiles)
                if missing_profiles
                else ""
            )
        )

    external_bindings_action = _build_external_bindings_follow_up_action(
        project_root,
        extension_execution_actuals,
        extension_execution_instance,
    )
    if external_bindings_action:
        actions.append(external_bindings_action)

    if extension_execution_evidence.get("status") != "ready":
        missing_reports = [
            str(item).strip()
            for item in extension_execution_evidence.get("missing_reports", [])
            if _is_non_empty_string(item)
        ]
        actions.append(
            "生成扩展执行留痕与外部绑定确认报告: "
            "python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations"
            + ("。缺失项: " + ", ".join(missing_reports) if missing_reports else "")
        )

    if (
        not actions
        and payload.get("release_gate_status") == "ready"
        and (channel != "stable" or security_preflight.get("status") == "passed")
    ):
        actions.append(
            f'门禁已就绪，可生成最终 manifest: python tools/build_release_artifact.py --version {version} --channel {channel} --build-id {payload["build_id"]}'
        )
    if channel in {"stable", "industrial"}:
        actions.append(
            "刷新外部主线计划并自动执行可安全部分: "
            + build_run_external_mainline_execution_plan_command()
            + "。这条 runner 会重建 vulnerability exception review，"
            "在提供 customer confirmation 参数后继续执行 external bindings closure，"
            "并把剩余真实 industrial 环境留痕固化到 "
            "external_mainline_execution_plan.json。"
        )

    return actions


def _combine_next_step_plan(previews: list[dict[str, Any]]) -> list[str]:
    plan: list[str] = []
    for preview in previews:
        if preview["channel"] == "stable":
            plan.extend(preview["next_actions"])
            break
    if not plan:
        for preview in previews:
            plan.extend(preview["next_actions"])
    return plan


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def execute_release_readiness(
    request: ReleaseReadinessRequest,
) -> ReleaseReadinessResult:
    changelog_path = _resolve_project_path(request.changelog, request.project_root)
    metadata = _extract_release_notes_metadata(changelog_path)
    current_version = request.current_version or metadata.get("version")
    if not current_version:
        raise ValueError(
            "--current-version is required when the changelog cannot provide a version."
        )
    stable_version = request.stable_version or _derive_stable_version(current_version)
    changelog_title = metadata.get("title", "AGI-Walker Release Notes")
    summary = metadata.get("release_summary", "Release readiness preview.")
    security_preflight = _load_security_preflight_preview(
        _resolve_security_preflight_path(
            request.security_preflight_report,
            project_root=request.project_root,
        )
    )
    release_approval, approval_manifest_path = _resolve_release_approval(
        project_root=request.project_root,
        expected_channel="stable",
        expected_version=stable_version,
        version_flag="stable-version",
        approval_manifest=request.approval_manifest,
        approval_status=request.approval_status,
        approved_by=request.approved_by,
        approved_at=request.approved_at,
        commit_sha=request.commit_sha,
        approval_notes=request.approval_notes,
    )

    output_root = Path(request.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_file = (
        Path(request.report_file)
        if request.report_file
        else output_root / "release_readiness_report.json"
    )
    previews = [
        _build_preview(
            channel="rc",
            version=current_version,
            build_id=_default_build_id("rc", current_version),
            project_root=request.project_root,
            source_root=request.source_root,
            changelog=request.changelog,
            changelog_title=changelog_title,
            release_summary=summary,
            output_root=output_root,
            source_root_hint=request.source_root,
            security_preflight=security_preflight,
        ),
        _build_preview(
            channel="stable",
            version=stable_version,
            build_id=_default_build_id("stable", stable_version),
            project_root=request.project_root,
            source_root=request.source_root,
            changelog=request.changelog,
            changelog_title=changelog_title,
            release_summary=summary,
            output_root=output_root,
            source_root_hint=request.source_root,
            security_preflight=security_preflight,
            release_approval=release_approval,
        ),
    ]
    next_step_plan = _combine_next_step_plan(previews)
    stable_preview = next(item for item in previews if item["channel"] == "stable")
    rc_preview = next(item for item in previews if item["channel"] == "rc")
    stable_preview["vulnerability_exception_review"] = (
        _build_vulnerability_exception_review_preview(security_preflight)
    )
    stable_preview["external_mainline_input_checklist"] = (
        _build_external_mainline_input_checklist_preview(
            project_root=request.project_root
        )
    )

    payload = {
        "schema_version": "1.0",
        "artifact_type": "release_readiness_report",
        "generated_at": _now_iso(),
        "project_root": request.project_root,
        "source_root": request.source_root,
        "current_version": current_version,
        "stable_version": stable_version,
        "approval_manifest_path": approval_manifest_path,
        "security_preflight_report_path": security_preflight["report_path"],
        "report_path": str(report_file),
        "stable_release_gate": stable_preview["release_gate_status"],
        "customer_delivery_surface": stable_preview["customer_delivery_surface"],
        "industrial_delivery_gate": stable_preview["industrial_delivery_gate"],
        "vulnerability_exception_review": stable_preview[
            "vulnerability_exception_review"
        ],
        "external_mainline_input_checklist": stable_preview[
            "external_mainline_input_checklist"
        ],
        "worktree_release_blocker": stable_preview["worktree_release_blocker"],
        "external_mainline_execution_plan": stable_preview[
            "external_mainline_execution_plan"
        ],
        "extension_execution_evidence": stable_preview.get(
            "extension_execution_evidence", {}
        ),
        "extension_execution_instance": stable_preview.get(
            "extension_execution_instance", {}
        ),
        "extension_execution_schedule": stable_preview.get(
            "extension_execution_schedule", {}
        ),
        "extension_execution_actuals": stable_preview.get(
            "extension_execution_actuals", {}
        ),
        "summary": _build_channel_readiness_summary(
            label="stable", preview=stable_preview
        ),
        "previews": previews,
        "next_step_plan": next_step_plan,
    }
    report_path = _write_report(report_file, payload)
    return ReleaseReadinessResult(
        payload=payload,
        report_path=report_path,
        current_version=current_version,
        stable_version=stable_version,
        rc_preview=rc_preview,
        stable_preview=stable_preview,
    )


def execute_industrial_release_readiness(
    request: IndustrialReleaseReadinessRequest,
) -> IndustrialReleaseReadinessResult:
    changelog_path = _resolve_project_path(request.changelog, request.project_root)
    metadata = _extract_release_notes_metadata(changelog_path)
    current_version = request.current_version or metadata.get("version")
    if not current_version:
        raise ValueError(
            "--current-version is required when the changelog cannot provide a version."
        )
    industrial_version = request.industrial_version or _derive_stable_version(
        current_version
    )
    changelog_title = metadata.get("title", "AGI-Walker Release Notes")
    summary = metadata.get("release_summary", "Industrial release readiness preview.")
    security_preflight = _load_security_preflight_preview(
        _resolve_security_preflight_path(
            request.security_preflight_report,
            project_root=request.project_root,
        )
    )
    release_approval, approval_manifest_path = _resolve_release_approval(
        project_root=request.project_root,
        expected_channel="industrial",
        expected_version=industrial_version,
        version_flag="industrial-version",
        approval_manifest=request.approval_manifest,
        approval_status=request.approval_status,
        approved_by=request.approved_by,
        approved_at=request.approved_at,
        commit_sha=request.commit_sha,
        approval_notes=request.approval_notes,
    )

    output_root = Path(request.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    report_file = (
        Path(request.report_file)
        if request.report_file
        else output_root / "industrial_release_readiness_report.json"
    )
    industrial_preview = _build_preview(
        channel="industrial",
        version=industrial_version,
        build_id=_default_build_id("industrial", industrial_version),
        project_root=request.project_root,
        source_root=request.source_root,
        changelog=request.changelog,
        changelog_title=changelog_title,
        release_summary=summary,
        output_root=output_root,
        source_root_hint=request.source_root,
        security_preflight=security_preflight,
        release_approval=release_approval,
    )
    industrial_preview["vulnerability_exception_review"] = (
        _build_vulnerability_exception_review_preview(security_preflight)
    )
    industrial_preview["external_mainline_input_checklist"] = (
        _build_external_mainline_input_checklist_preview(
            project_root=request.project_root
        )
    )

    payload = {
        "schema_version": "1.0",
        "artifact_type": "industrial_release_readiness_report",
        "generated_at": _now_iso(),
        "project_root": request.project_root,
        "source_root": request.source_root,
        "current_version": current_version,
        "industrial_version": industrial_version,
        "approval_manifest_path": approval_manifest_path,
        "security_preflight_report_path": security_preflight["report_path"],
        "report_path": str(report_file),
        "industrial_release_gate": industrial_preview["release_gate_status"],
        "customer_delivery_surface": industrial_preview["customer_delivery_surface"],
        "industrial_delivery_gate": industrial_preview["industrial_delivery_gate"],
        "vulnerability_exception_review": industrial_preview[
            "vulnerability_exception_review"
        ],
        "external_mainline_input_checklist": industrial_preview[
            "external_mainline_input_checklist"
        ],
        "external_mainline_execution_plan": industrial_preview[
            "external_mainline_execution_plan"
        ],
        "worktree_release_blocker": industrial_preview["worktree_release_blocker"],
        "extension_execution_evidence": industrial_preview.get(
            "extension_execution_evidence", {}
        ),
        "extension_execution_instance": industrial_preview.get(
            "extension_execution_instance", {}
        ),
        "extension_execution_schedule": industrial_preview.get(
            "extension_execution_schedule", {}
        ),
        "extension_execution_actuals": industrial_preview.get(
            "extension_execution_actuals", {}
        ),
        "summary": _build_channel_readiness_summary(
            label="industrial",
            preview=industrial_preview,
        ),
        "previews": [industrial_preview],
        "next_step_plan": industrial_preview["next_actions"],
    }
    report_path = _write_report(report_file, payload)
    return IndustrialReleaseReadinessResult(
        payload=payload,
        report_path=report_path,
        current_version=current_version,
        industrial_version=industrial_version,
        industrial_preview=industrial_preview,
    )


__all__ = [
    "execute_industrial_release_readiness",
    "execute_release_readiness",
    "format_external_mainline_execution_plan_preview",
    "format_external_mainline_input_checklist_preview",
    "format_worktree_release_blocker_preview",
]
