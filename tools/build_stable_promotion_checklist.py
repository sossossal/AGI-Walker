#!/usr/bin/env python
"""Build a structured stable-promotion checklist for the current HEAD."""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import datetime, timezone
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
    EXTENSION_EXTERNAL_BINDING_SECTION_IDS,
    build_external_mainline_input_checklist_command,
    build_run_external_mainline_execution_plan_command,
    build_run_customer_external_bindings_closure_command,
    build_run_worktree_release_blocker_command,
    build_vulnerability_exception_review_report_command,
    default_customer_external_bindings_closure_report_path,
    resolve_customer_external_bindings_config_path,
    validate_release_evidence_report,
)
from agi_walker.core.api.release_ops_contracts import (  # noqa: E402
    ReleaseReadinessRequest,
)
from agi_walker.ops.readiness import (  # noqa: E402
    execute_release_readiness,
    format_external_mainline_execution_plan_preview as _format_readiness_external_mainline_execution_plan_preview,
    format_external_mainline_input_checklist_preview as _format_readiness_external_mainline_input_checklist_preview,
    format_worktree_release_blocker_preview as _format_readiness_worktree_release_blocker_preview,
)


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _default_stable_build_id(version: str) -> str:
    normalized = re.sub(r"[^A-Za-z0-9._-]+", "-", version)
    return f"build-{normalized}-stable"


def _default_vulnerability_exception_review_command() -> str:
    return build_vulnerability_exception_review_report_command()


def _build_final_command(
    *,
    version: str,
    approval: dict[str, Any],
) -> str:
    approved_by = approval.get("approved_by") or "<approved-by>"
    approved_at = approval.get("approved_at") or "<approved-at-iso8601>"
    approval_notes = approval.get("notes") or "stable signoff"
    return (
        "python tools/build_release_artifact.py "
        f"--version {version} "
        "--channel stable "
        f"--build-id {_default_stable_build_id(version)} "
        "--approval-status approved "
        f"--approved-by {approved_by} "
        f"--approved-at {approved_at} "
        f'--approval-notes "{approval_notes}" '
        "--output test_env/release/release_manifest_stable.json"
    )


def _run_readiness_command(
    *,
    readiness_report_path: Path,
    current_version: str | None,
    stable_version: str | None,
    project_root: str,
    source_root: str,
    changelog: str,
    security_preflight_report: str | None,
    approval_args: dict[str, str | None],
    approval_manifest: str | None,
) -> tuple[dict[str, Any], str, str]:
    result = execute_release_readiness(
        ReleaseReadinessRequest(
            current_version=current_version,
            stable_version=stable_version,
            project_root=project_root,
            source_root=source_root,
            changelog=changelog,
            output_root=str(readiness_report_path.parent),
            report_file=str(readiness_report_path),
            approval_status=approval_args.get("--approval-status"),
            approved_by=approval_args.get("--approved-by"),
            approved_at=approval_args.get("--approved-at"),
            commit_sha=approval_args.get("--commit-sha"),
            approval_notes=approval_args.get("--approval-notes"),
            approval_manifest=approval_manifest,
            security_preflight_report=security_preflight_report,
        )
    )
    stable_preview = result.stable_preview
    review_preview = stable_preview["vulnerability_exception_review"]
    review_candidate_count = review_preview.get("review_candidate_count")
    review_status = review_preview.get("status")
    review_stdout = (
        f"{review_status}/{review_candidate_count}"
        if isinstance(review_candidate_count, int) and review_candidate_count >= 0
        else str(review_status)
    )
    stdout_lines = [
        f"release_readiness_written={result.report_path}",
        f"current_version={result.current_version}",
        f"stable_version={result.stable_version}",
        f"rc_release_gate={result.rc_preview['release_gate_status']}",
        f"stable_release_gate={stable_preview['release_gate_status']}",
        "stable_security_preflight="
        f"{stable_preview['security_release_preflight']['status']}",
        "stable_customer_delivery="
        f"{stable_preview['customer_delivery_surface']['status']}",
        "stable_industrial_delivery="
        f"{stable_preview['industrial_delivery_gate']['status']}",
        "stable_extension_execution_instance="
        f"{stable_preview['extension_execution_instance']['status']}",
        "stable_extension_execution_schedule="
        f"{stable_preview['extension_execution_schedule']['status']}",
        "stable_extension_execution_actuals="
        f"{stable_preview['extension_execution_actuals']['status']}",
        "stable_external_mainline_execution_plan="
        f"{_format_readiness_external_mainline_execution_plan_preview(stable_preview['external_mainline_execution_plan'])}",
        "stable_external_mainline_input_checklist="
        f"{_format_readiness_external_mainline_input_checklist_preview(stable_preview['external_mainline_input_checklist'])}",
        "stable_worktree_release_blocker="
        f"{_format_readiness_worktree_release_blocker_preview(stable_preview['worktree_release_blocker'])}",
        f"stable_vulnerability_exception_review={review_stdout}",
        f"stable_next_actions={len(stable_preview['next_actions'])}",
    ]
    return result.payload, "\n".join(stdout_lines), ""


def _load_readiness_report(
    *,
    readiness_report_path: Path,
    refresh_readiness: bool,
    current_version: str | None,
    stable_version: str | None,
    project_root: str,
    source_root: str,
    changelog: str,
    security_preflight_report: str | None,
    approval_args: dict[str, str | None],
    approval_manifest: str | None,
) -> tuple[dict[str, Any], str, str]:
    if readiness_report_path.is_file() and not refresh_readiness:
        payload = json.loads(readiness_report_path.read_text(encoding="utf-8"))
        return payload, "", ""
    return _run_readiness_command(
        readiness_report_path=readiness_report_path,
        current_version=current_version,
        stable_version=stable_version,
        project_root=project_root,
        source_root=source_root,
        changelog=changelog,
        security_preflight_report=security_preflight_report,
        approval_args=approval_args,
        approval_manifest=approval_manifest,
    )


def _stable_preview(readiness_payload: dict[str, Any]) -> dict[str, Any]:
    return next(item for item in readiness_payload["previews"] if item["channel"] == "stable")


def _load_manifest(path: str) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _matches_ready_stable_manifest(
    approval_manifest: dict[str, Any] | None,
    *,
    stable_preview: dict[str, Any],
) -> bool:
    if not isinstance(approval_manifest, dict):
        return False
    if approval_manifest.get("artifact_type") != "release_manifest":
        return False
    if approval_manifest.get("channel") != "stable":
        return False
    if approval_manifest.get("release_gate_status") != "ready":
        return False
    if approval_manifest.get("version") != stable_preview.get("version"):
        return False
    manifest_source = approval_manifest.get("release_source", {})
    preview_source = stable_preview.get("release_source", {})
    return (
        manifest_source.get("commit_sha") == preview_source.get("commit_sha")
        and manifest_source.get("matched_version_tag")
        == preview_source.get("matched_version_tag")
    )


def _evidence_step_title(name: str) -> str:
    titles = {
        "distributed_runtime_live": "补齐 distributed live 证据",
        "godot_headless_live": "补齐 Godot headless live 证据",
        "ros2_bridge_live": "补齐 ROS2 bridge live 证据",
    }
    return titles.get(name, f"补齐证据 {name}")


def _build_evidence_steps(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []
    for item in manifest.get("test_evidence", []):
        if item.get("status") == "passed":
            continue
        status = "pending"
        steps.append(
            {
                "id": f"evidence:{item['name']}",
                "category": "evidence",
                "title": _evidence_step_title(str(item["name"])),
                "status": status,
                "required": True,
                "blocking": True,
                "summary": item.get("summary"),
                "command": item.get("command"),
                "ready_to_run": bool(item.get("command")),
                "depends_on": [],
            }
        )
    return steps


def _build_security_preflight_step(stable_preview: dict[str, Any]) -> dict[str, Any]:
    security_preflight = stable_preview.get("security_release_preflight", {})
    status = security_preflight.get("status")
    summary = security_preflight.get("summary")
    command = security_preflight.get("command")
    is_done = status == "passed"
    return {
        "id": "security_release_preflight",
        "category": "security",
        "title": "补齐 security release preflight",
        "status": "done" if is_done else "pending",
        "required": True,
        "blocking": not is_done,
        "summary": (
            summary
            if isinstance(summary, str) and summary.strip()
            else "security release preflight status is unavailable."
        ),
        "command": None if is_done else command,
        "ready_to_run": bool(command) and not is_done,
        "depends_on": [],
    }


def _coerce_non_negative_int(value: Any) -> int:
    return value if isinstance(value, int) and value >= 0 else 0


def _coerce_non_empty_strings(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [
        str(item).strip()
        for item in value
        if isinstance(item, str) and item.strip()
    ]


def _build_vulnerability_exception_review_step(
    stable_preview: dict[str, Any],
) -> dict[str, Any]:
    security_preflight = stable_preview.get("security_release_preflight", {})
    preflight_status = security_preflight.get("status")
    metrics = security_preflight.get("metrics", {})
    if not isinstance(metrics, dict):
        metrics = {}
    review_status = str(metrics.get("vulnerability_exception_review_status") or "").strip()
    review_due = _coerce_non_negative_int(metrics.get("vulnerability_exception_review_due"))
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
    stale_exception_count = _coerce_non_negative_int(
        metrics.get("stale_vulnerability_exceptions")
    )
    stale_exception_ids = _coerce_non_empty_strings(
        metrics.get("stale_vulnerability_exception_ids")
    )
    next_expiry = str(metrics.get("vulnerability_exception_next_expiry") or "").strip()
    command = _default_vulnerability_exception_review_command()
    stale_preview = ", ".join(stale_exception_ids[:3])
    if stale_exception_count > 3:
        stale_preview += ", ..."
    review_due_preview = ", ".join(review_due_ids[:3])
    if len(review_due_ids) > 3:
        review_due_preview += ", ..."
    expired_preview = ", ".join(expired_ids[:3])
    if len(expired_ids) > 3:
        expired_preview += ", ..."
    review_ticket_preview = ", ".join(review_due_tickets[:3])
    if len(review_due_tickets) > 3:
        review_ticket_preview += ", ..."
    report_suffix = (
        f" review_report={review_report_status or 'unknown'}/{review_candidate_count}"
        + (f" ({review_report_path})" if review_report_path else ".")
    )

    if stale_exception_count > 0:
        return {
            "id": "vulnerability_exception_review",
            "category": "security",
            "title": "替换已失效的 no-fix vulnerability exceptions",
            "status": "pending",
            "required": False,
            "blocking": False,
            "summary": (
                f"{stale_exception_count} 条 active no-fix exception 已失效，"
                "因为匹配 findings 现在带 fix versions。"
                + (
                    f" 例如: {stale_preview}."
                    if stale_preview
                    else ""
                )
                + " 先重建并复核独立 review report，再替换 exception 并重跑 security release preflight。"
                + report_suffix
            ),
            "command": command,
            "ready_to_run": bool(command),
            "depends_on": [],
        }

    if preflight_status != "passed":
        return {
            "id": "vulnerability_exception_review",
            "category": "security",
            "title": "复核 vulnerability exception 到期窗口",
            "status": "pending",
            "required": False,
            "blocking": False,
            "summary": "先让 security release preflight 通过，再确认 exception 到期复核窗口。",
            "command": None,
            "ready_to_run": False,
            "depends_on": ["security_release_preflight"],
        }
    if review_status == "expired" or expired > 0:
        return {
            "id": "vulnerability_exception_review",
            "category": "security",
            "title": "处理已过期的 vulnerability exceptions",
            "status": "pending",
            "required": False,
            "blocking": False,
            "summary": (
                "security release preflight 已检测到过期 exception；"
                f"expired={expired}，需要先更新 deployment/security/vulnerability_exceptions.input.json。"
                + (
                    f" 例如: {expired_preview}."
                    if expired_preview
                    else ""
                )
                + " 先重建独立 review report，再刷新 exception 并重跑 security release preflight。"
                + report_suffix
            ),
            "command": command,
            "ready_to_run": bool(command),
            "depends_on": ["security_release_preflight"],
        }
    if review_status == "review_due" and review_due > 0:
        return {
            "id": "vulnerability_exception_review",
            "category": "security",
            "title": "复核即将到期的 vulnerability exceptions",
            "status": "pending",
            "required": False,
            "blocking": False,
            "summary": (
                f"{review_due} 条 active exception 已进入复核窗口，"
                f"最早到期时间为 {next_expiry or 'unknown'}。"
                + (
                    f" 例如: {review_due_preview}."
                    if review_due_preview
                    else ""
                )
                + (
                    f" tickets={review_ticket_preview}."
                    if review_ticket_preview
                    else ""
                )
                + " 先重建独立 review report，再更新 exception 输入并重跑 security release preflight。"
                + report_suffix
            ),
            "command": command,
            "ready_to_run": bool(command),
            "depends_on": ["security_release_preflight"],
        }
    return {
        "id": "vulnerability_exception_review",
        "category": "security",
        "title": "复核 vulnerability exception 到期窗口",
        "status": "done",
        "required": False,
        "blocking": False,
        "summary": (
            f"当前 active exception 尚未进入复核窗口，下一次到期时间为 {next_expiry}。"
            if review_status == "tracked" and next_expiry
            else "当前没有进入复核窗口的 vulnerability exceptions。"
        ),
        "command": None,
        "ready_to_run": False,
        "depends_on": ["security_release_preflight"],
    }


def _format_external_mainline_execution_plan_preview(
    preview: dict[str, Any],
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


def _format_external_mainline_input_checklist_preview(
    preview: dict[str, Any],
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


def _build_worktree_release_blocker_preview(
    stable_preview: dict[str, Any],
    *,
    output_root: Path,
) -> dict[str, Any]:
    preview = (
        dict(stable_preview.get("worktree_release_blocker", {}))
        if isinstance(stable_preview.get("worktree_release_blocker"), dict)
        else {}
    )
    if isinstance(preview.get("status"), str) and isinstance(preview.get("summary"), str):
        return preview

    source = (
        dict(stable_preview.get("release_source", {}))
        if isinstance(stable_preview.get("release_source"), dict)
        else {}
    )
    clean_worktree = source.get("worktree_clean") is True
    preview = {
        "status": "ready" if clean_worktree else "blocked",
        "clean_worktree": clean_worktree,
        "report_path": str(output_root / "worktree_release_blocker_report.json"),
        "summary": (
            "worktree release blocker ready: clean_worktree=true, total_paths=0, tracked_review=not_required."
            if clean_worktree
            else (
                f"worktree release blocker blocked: {source.get('worktree_status_summary')}."
                if isinstance(source.get("worktree_status_summary"), str)
                and source.get("worktree_status_summary", "").strip()
                else "worktree release blocker is blocked."
            )
        ),
    }
    if clean_worktree:
        preview["total_paths"] = 0
        preview["tracked_review_candidate_count"] = 0
    return preview


def _format_worktree_release_blocker_preview(preview: dict[str, Any]) -> str:
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


def _build_external_mainline_execution_plan_step(
    stable_preview: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    preview = (
        dict(stable_preview.get("external_mainline_execution_plan", {}))
        if isinstance(stable_preview.get("external_mainline_execution_plan"), dict)
        else {}
    )
    status = str(preview.get("status") or "missing").strip() or "missing"
    completed_steps = _coerce_non_negative_int(preview.get("completed_steps"))
    ready_to_run_steps = _coerce_non_negative_int(preview.get("ready_to_run_steps"))
    waiting_external_input_steps = _coerce_non_negative_int(
        preview.get("waiting_external_input_steps")
    )
    blocked_steps = _coerce_non_negative_int(preview.get("blocked_steps"))
    report_path = str(preview.get("report_path") or "").strip()
    summary = str(preview.get("summary") or "").strip()
    if not summary:
        summary = "external mainline execution plan status is unavailable."
    command = build_run_external_mainline_execution_plan_command()
    is_done = (
        status == "ready"
        and ready_to_run_steps == 0
        and waiting_external_input_steps == 0
        and blocked_steps == 0
    )
    if not is_done:
        summary += (
            " Run run_external_mainline_execution_plan.py to refresh the managed plan "
            "and auto-execute the safe portion of the remaining external mainline."
        )
    if report_path:
        summary += f" report={report_path}"
    preview.setdefault("completed_steps", completed_steps)
    preview.setdefault("ready_to_run_steps", ready_to_run_steps)
    preview.setdefault("waiting_external_input_steps", waiting_external_input_steps)
    preview.setdefault("blocked_steps", blocked_steps)
    preview.setdefault("status", status)
    if report_path:
        preview.setdefault("report_path", report_path)
    preview["summary"] = summary
    return preview, {
        "id": "external_mainline_execution_plan",
        "category": "external_mainline",
        "title": "刷新剩余外部主线计划并自动执行可安全部分",
        "status": "done" if is_done else "pending",
        "required": False,
        "blocking": False,
        "summary": summary,
        "command": None if is_done else command,
        "ready_to_run": (not is_done) and bool(command),
        "depends_on": [],
    }


def _build_external_mainline_input_checklist_step(
    stable_preview: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any] | None]:
    preview = (
        dict(stable_preview.get("external_mainline_input_checklist", {}))
        if isinstance(stable_preview.get("external_mainline_input_checklist"), dict)
        else {}
    )
    status = str(preview.get("status") or "missing").strip() or "missing"
    if status == "missing":
        return preview, None
    missing_input_count = _coerce_non_negative_int(preview.get("missing_input_count"))
    waiting_external_input_steps = _coerce_non_negative_int(
        preview.get("waiting_external_input_steps")
    )
    ready_to_run_steps = _coerce_non_negative_int(preview.get("ready_to_run_steps"))
    completed_steps = _coerce_non_negative_int(preview.get("completed_steps"))
    report_path = str(preview.get("report_path") or "").strip()
    summary = str(preview.get("summary") or "").strip()
    if not summary:
        summary = "external mainline input checklist status is unavailable."
    command = build_external_mainline_input_checklist_command()
    is_done = status == "passed"
    if not is_done:
        summary += (
            " Run build_external_mainline_input_checklist.py to refresh the structured "
            "missing-input checklist before continuing the external mainline."
        )
    if report_path:
        summary += f" report={report_path}"
    preview.setdefault("status", status)
    preview.setdefault("missing_input_count", missing_input_count)
    preview.setdefault("waiting_external_input_steps", waiting_external_input_steps)
    preview.setdefault("ready_to_run_steps", ready_to_run_steps)
    preview.setdefault("completed_steps", completed_steps)
    if report_path:
        preview.setdefault("report_path", report_path)
    preview["summary"] = summary
    return preview, {
        "id": "external_mainline_input_checklist",
        "category": "external_mainline_input_checklist",
        "title": "刷新剩余外部主线输入缺口清单",
        "status": "done" if is_done else "pending",
        "required": False,
        "blocking": False,
        "summary": summary,
        "command": None if is_done else command,
        "ready_to_run": (not is_done) and bool(command),
        "depends_on": [],
    }


def _build_customer_delivery_step(stable_preview: dict[str, Any]) -> dict[str, Any]:
    customer_delivery_surface = stable_preview.get("customer_delivery_surface", {})
    status = customer_delivery_surface.get("status")
    is_done = status == "ready"
    summary = customer_delivery_surface.get("summary")
    if not isinstance(summary, str) or not summary.strip():
        summary = "Customer delivery surface status is unavailable."
    return {
        "id": "customer_delivery_surface",
        "category": "customer_delivery",
        "title": "确认 Phase E 客户交付文档集齐备",
        "status": "done" if is_done else "pending",
        "required": False,
        "blocking": False,
        "summary": summary,
        "command": None,
        "ready_to_run": False,
        "depends_on": [],
    }


def _build_industrial_delivery_step(stable_preview: dict[str, Any]) -> dict[str, Any]:
    industrial_delivery_gate = stable_preview.get("industrial_delivery_gate", {})
    status = industrial_delivery_gate.get("status")
    is_done = status == "ready"
    summary = industrial_delivery_gate.get("summary")
    if not isinstance(summary, str) or not summary.strip():
        summary = "Industrial delivery gate status is unavailable."
    return {
        "id": "industrial_delivery_gate",
        "category": "industrial_delivery",
        "title": "确认 industrial delivery gate 已闭合",
        "status": "done" if is_done else "pending",
        "required": False,
        "blocking": False,
        "summary": summary,
        "command": None,
        "ready_to_run": False,
        "depends_on": [],
    }


def _build_extension_execution_actuals_step(
    stable_preview: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    actuals = (
        dict(stable_preview.get("extension_execution_actuals", {}))
        if isinstance(stable_preview.get("extension_execution_actuals"), dict)
        else {}
    )
    status = actuals.get("status")
    is_done = status == "ready"
    summary = actuals.get("summary")
    if not isinstance(summary, str) or not summary.strip():
        summary = "Extension execution actuals status is unavailable."
    return actuals, {
        "id": "extension_execution_actuals",
        "category": "extension_execution_actuals",
        "title": "附带客户窗口执行留痕与 closure manifest",
        "status": "done" if is_done else "pending",
        "required": False,
        "blocking": False,
        "summary": summary,
        "command": (
            None
            if is_done
            else "python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json"
        ),
        "ready_to_run": not is_done,
        "depends_on": [],
    }


def _build_extension_external_bindings_step(
    *, project_root: str, actuals: dict[str, Any]
) -> dict[str, Any] | None:
    status = str(actuals.get("external_bindings_status") or "").strip()
    if status not in {"missing", "placeholder", "partial", "ready"}:
        return None

    config_path = None
    external_bindings = actuals.get("external_bindings")
    if isinstance(external_bindings, dict):
        candidate = external_bindings.get("config_path")
        if isinstance(candidate, str) and candidate.strip():
            config_path = candidate.strip()

    command = None
    summary = actuals.get("external_bindings_summary") or actuals.get("summary")
    draft_sections = _coerce_non_empty_strings(
        actuals.get("external_bindings_draft_sections")
    )
    unconfirmed_sections = _coerce_non_empty_strings(
        actuals.get("external_bindings_unconfirmed_sections")
    )
    confirmation_missing_sections = _coerce_non_empty_strings(
        actuals.get("external_bindings_confirmation_missing_sections")
    )
    placeholder_sections = _coerce_non_empty_strings(
        actuals.get("external_bindings_placeholder_sections")
    )
    is_existing_customer_config = bool(config_path) and config_path not in {
        "deployment/customer_delivery.external_bindings.json",
        "deployment/customer_delivery.external_bindings.rehearsal.json",
    }
    if status in {"missing", "placeholder", "partial"}:
        customer_config_path = resolve_customer_external_bindings_config_path(config_path)
        closure_sections = (
            draft_sections
            or confirmation_missing_sections
            or unconfirmed_sections
            or placeholder_sections
            or list(EXTENSION_EXTERNAL_BINDING_SECTION_IDS)
        )
        command = build_run_customer_external_bindings_closure_command(
            config_path=customer_config_path,
            sections=closure_sections,
        )
        if is_existing_customer_config:
            summary = (
                f"{summary} Complete the real customer metadata in the existing config, then "
                "run run_customer_external_bindings_closure.py to confirm the affected sections, "
                "rebuild actuals, and refresh release evidence."
            )
        else:
            summary = (
                f"{summary} Run run_customer_external_bindings_closure.py to generate the draft "
                "customer-specific bindings config. Supply `--set` overrides in the same command "
                "or rerun after editing the generated config so the runner can confirm the "
                "sections, rebuild actuals, and refresh release evidence."
            )
        closure_preview = _load_customer_external_bindings_closure_preview(project_root)
        if closure_preview is not None:
            summary += " Latest closure report: " + str(closure_preview.get("summary"))
            failed_steps = _coerce_non_empty_strings(
                closure_preview.get("failed_steps")
            )
            if failed_steps:
                summary += " failed_steps=" + ", ".join(failed_steps)
            report_path = closure_preview.get("report_path")
            if isinstance(report_path, str) and report_path.strip():
                summary += f" report={report_path}"

    return {
        "id": "extension_external_bindings",
        "category": "extension_external_bindings",
        "title": "将客户外部审批/归档/到期触发绑定切到真实系统",
        "status": "done" if status == "ready" else "pending",
        "required": False,
        "blocking": False,
        "summary": summary,
        "command": command,
        "ready_to_run": bool(command),
        "depends_on": ["extension_execution_actuals"],
    }


def _load_customer_external_bindings_closure_preview(
    project_root: str,
) -> dict[str, Any] | None:
    report_path = Path(project_root) / default_customer_external_bindings_closure_report_path()
    if not report_path.is_file():
        return None
    try:
        payload = json.loads(report_path.read_text(encoding="utf-8"))
    except Exception as exc:
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
    return {
        "status": payload.get("status") or "blocked",
        "summary": payload.get("summary")
        or "customer external bindings closure report has no summary.",
        "report_path": str(report_path),
        "failed_steps": _coerce_non_empty_strings(metrics.get("failed_steps"))
        if isinstance(metrics, dict)
        else [],
    }


def _build_domain_steps(manifest: dict[str, Any]) -> list[dict[str, Any]]:
    steps: list[dict[str, Any]] = []
    for domain in manifest.get("capability_matrix", {}).get("domains", []):
        if domain.get("status") != "diagnostic_ready":
            continue
        steps.append(
            {
                "id": f"domain:{domain['id']}",
                "category": "domain",
                "title": f"关闭发布面 {domain['id']} 的诊断态",
                "status": "pending",
                "required": True,
                "blocking": True,
                "summary": domain.get("summary"),
                "command": None,
                "ready_to_run": False,
                "depends_on": [],
            }
        )
    return steps


def _build_prerequisite_steps(
    *,
    manifest: dict[str, Any],
    stable_preview: dict[str, Any],
    worktree_release_blocker: dict[str, Any],
    source_root: str,
    output_root: Path,
) -> list[dict[str, Any]]:
    approval = manifest["release_approval"]
    source = stable_preview["release_source"]
    version = stable_preview["version"]
    steps: list[dict[str, Any]] = []

    approval_done = manifest["release_gate"]["release_approval_ready"] == 1
    steps.append(
        {
            "id": "stable_approval",
            "category": "approval",
            "title": "补齐 stable 签核元数据",
            "status": "done" if approval_done else "pending",
            "required": True,
            "blocking": not approval_done,
            "summary": (
                f"签核已完成: {approval.get('approved_by')} @ {approval.get('approved_at')}"
                if approval_done
                else "stable 通道要求 approved_by、approved_at、commit_sha 三项齐全。"
            ),
            "command": None,
            "ready_to_run": False,
            "depends_on": [],
        }
    )

    source_done = manifest["release_gate"]["release_source_ready"] == 1
    source_summary = (
        f"当前 HEAD 已绑定签核 SHA: {source.get('short_commit_sha')}"
        if source_done
        else (
            f"签核 commit 需与当前 HEAD 对齐: {source.get('commit_sha')}"
            if source.get("resolved_from_git")
            else "当前 source_root 未解析到有效 Git HEAD。"
        )
    )
    steps.append(
        {
            "id": "git_source_binding",
            "category": "git",
            "title": "确认 stable 签核绑定当前 Git HEAD",
            "status": "done" if source_done else "pending",
            "required": True,
            "blocking": not source_done,
            "summary": source_summary,
            "command": None,
            "ready_to_run": False,
            "depends_on": ["stable_approval"],
        }
    )

    worktree_done = manifest["release_gate"]["release_worktree_ready"] == 1
    worktree_summary = source.get("worktree_status_summary")
    steps.append(
        {
            "id": "clean_worktree",
            "category": "git",
            "title": "清理当前工作区并保持 release source 为 clean worktree",
            "status": "done" if worktree_done else "pending",
            "required": True,
            "blocking": not worktree_done,
            "summary": (
                "当前工作区已 clean。"
                if worktree_done
                else (
                    str(worktree_release_blocker.get("summary")).strip()
                    if isinstance(worktree_release_blocker.get("summary"), str)
                    and str(worktree_release_blocker.get("summary")).strip()
                    else (
                        f"当前工作区未清理: {worktree_summary}"
                        if isinstance(worktree_summary, str) and worktree_summary.strip()
                        else "当前工作区未清理。"
                    )
                )
            ),
            "command": (
                None
                if worktree_done
                else build_run_worktree_release_blocker_command(
                    source_root=source_root,
                    output_root=output_root,
                )
            ),
            "ready_to_run": not worktree_done,
            "depends_on": [],
        }
    )

    tag_done = manifest["release_gate"]["release_version_tag_ready"] == 1
    steps.append(
        {
            "id": "version_tag",
            "category": "git",
            "title": "为当前 HEAD 创建匹配 stable 版本 tag",
            "status": "done" if tag_done else "pending",
            "required": True,
            "blocking": not tag_done,
            "summary": (
                f"已匹配 tag: {source.get('matched_version_tag')}"
                if tag_done
                else f"当前 HEAD 尚未匹配 {version} 或 v{version}。"
            ),
            "command": None if tag_done else f"git tag {version} {source.get('commit_sha')}",
            "ready_to_run": not tag_done,
            "depends_on": [],
        }
    )
    return steps


def _build_final_step(
    *,
    manifest: dict[str, Any],
    stable_preview: dict[str, Any],
    prerequisite_steps: list[dict[str, Any]],
    approval_manifest: dict[str, Any] | None,
    approval_manifest_path: str | None,
) -> dict[str, Any]:
    if _matches_ready_stable_manifest(approval_manifest, stable_preview=stable_preview):
        return {
            "id": "build_stable_manifest",
            "category": "release",
            "title": "生成 stable release manifest",
            "status": "done",
            "required": True,
            "blocking": False,
            "summary": (
                "已存在与当前 HEAD 匹配的 ready stable manifest。"
                + (
                    f" 来源: {approval_manifest_path}"
                    if approval_manifest_path
                    else ""
                )
            ),
            "command": None,
            "ready_to_run": False,
            "depends_on": [],
        }

    pending_blockers = [
        step["id"] for step in prerequisite_steps if step["blocking"] and step["status"] != "done"
    ]
    command = _build_final_command(
        version=stable_preview["version"],
        approval=manifest["release_approval"],
    )
    return {
        "id": "build_stable_manifest",
        "category": "release",
        "title": "生成 stable release manifest",
        "status": "pending",
        "required": True,
        "blocking": False,
        "summary": (
            "所有前置项已闭合，可直接执行 stable builder。"
            if not pending_blockers
            else "前置项未闭合，暂不应执行最终 stable builder。"
        ),
        "command": command,
        "ready_to_run": not pending_blockers,
        "depends_on": pending_blockers,
    }


def _build_summary(
    *,
    blocking_steps: int,
    stable_gate: str,
    worktree_release_blocker: dict[str, Any],
    external_mainline_execution_plan: dict[str, Any],
    external_mainline_input_checklist: dict[str, Any],
) -> str:
    external_mainline_summary = _format_external_mainline_execution_plan_preview(
        external_mainline_execution_plan
    )
    checklist_summary = _format_external_mainline_input_checklist_preview(
        external_mainline_input_checklist
    )
    worktree_summary = _format_worktree_release_blocker_preview(
        worktree_release_blocker
    )
    checklist_suffix = (
        f", external_mainline_input_checklist={checklist_summary}"
        if str(external_mainline_input_checklist.get("status") or "").strip()
        and str(external_mainline_input_checklist.get("status") or "").strip()
        != "missing"
        else ""
    )
    if blocking_steps == 0:
        return (
            "stable promotion 前置项已闭合，最终 manifest 生成命令可执行。 "
            f"external_mainline={external_mainline_summary}, "
            f"worktree={worktree_summary}"
            f"{checklist_suffix}."
        )
    return (
        f"stable promotion 仍有 {blocking_steps} 个阻塞前置项。 "
        f"external_mainline={external_mainline_summary}, "
        f"worktree={worktree_summary}"
        f"{checklist_suffix}."
    )


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build a stable promotion checklist for the current HEAD."
    )
    parser.add_argument("--current-version", default=None)
    parser.add_argument("--stable-version", default=None)
    parser.add_argument("--project-root", default=str(PROJECT_ROOT))
    parser.add_argument("--source-root", default=str(PROJECT_ROOT))
    parser.add_argument("--changelog", default="RELEASE_NOTES.md")
    parser.add_argument(
        "--output-root",
        default=str(PROJECT_ROOT / "test_env" / "stable_promotion"),
    )
    parser.add_argument("--readiness-report", default=None)
    parser.add_argument("--refresh-readiness", action="store_true")
    parser.add_argument("--report-file", default=None)
    parser.add_argument("--approval-status", default=None)
    parser.add_argument("--approved-by", default=None)
    parser.add_argument("--approved-at", default=None)
    parser.add_argument("--commit-sha", default=None)
    parser.add_argument("--approval-notes", default=None)
    parser.add_argument("--security-preflight-report", default=None)
    parser.add_argument(
        "--approval-manifest",
        default=None,
        help="Optional ready stable release manifest used to import approval metadata.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    from agi_walker.core.api.release_ops_contracts import StablePromotionChecklistRequest
    from agi_walker.ops.promotion import execute_stable_promotion_checklist

    parser = build_parser()
    args = parser.parse_args(argv)
    result = execute_stable_promotion_checklist(
        StablePromotionChecklistRequest(
            current_version=args.current_version,
            stable_version=args.stable_version,
            project_root=args.project_root,
            source_root=args.source_root,
            changelog=args.changelog,
            output_root=args.output_root,
            readiness_report=args.readiness_report,
            refresh_readiness=args.refresh_readiness,
            report_file=args.report_file,
            approval_status=args.approval_status,
            approved_by=args.approved_by,
            approved_at=args.approved_at,
            commit_sha=args.commit_sha,
            approval_notes=args.approval_notes,
            security_preflight_report=args.security_preflight_report,
            approval_manifest=args.approval_manifest,
        )
    )

    payload = result.payload
    print(f"stable_promotion_checklist_written={result.report_path}")
    print(f"stable_promotion_gate={result.stable_preview['release_gate_status']}")
    print(f"stable_promotion_blocking_steps={payload['blocking_steps']}")
    print(
        "stable_extension_execution_actuals="
        f"{payload['extension_execution_actuals'].get('status') or 'missing'}"
    )
    print(
        "stable_external_mainline_execution_plan="
        f"{_format_external_mainline_execution_plan_preview(payload['external_mainline_execution_plan'])}"
    )
    print(
        "stable_external_mainline_input_checklist="
        f"{_format_external_mainline_input_checklist_preview(payload['external_mainline_input_checklist'])}"
    )
    print(
        "stable_worktree_release_blocker="
        f"{_format_worktree_release_blocker_preview(payload['worktree_release_blocker'])}"
    )
    print(
        "stable_promotion_ready_to_promote="
        f"{str(payload['ready_to_promote']).lower()}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
