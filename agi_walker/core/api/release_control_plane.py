"""Shared read-only helpers for release/control-plane discovery surfaces."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any
from urllib.parse import quote

from agi_walker.core.api.release_contracts import (
    build_confirm_customer_external_bindings_command,
    build_customer_external_bindings_config_command,
    build_external_mainline_input_checklist_command,
    build_run_external_mainline_execution_plan_command,
    build_run_customer_external_bindings_closure_command,
    build_run_worktree_release_blocker_command,
    build_vulnerability_exception_review_report_command,
    default_customer_external_bindings_config_path,
    default_external_mainline_execution_plan_path,
    default_external_mainline_input_checklist_report_path,
    default_external_mainline_inputs_path,
    default_vulnerability_exception_review_report_path,
    read_release_control_plane_surface,
)
from agi_walker.ops import (
    list_release_ops_actions,
    list_release_ops_policy_profiles,
    list_release_ops_request_templates,
)

DEFAULT_RELEASE_CONTROL_PLANE_ROUTE = "/api/release/control-plane"
DEFAULT_RELEASE_CONTROL_PLANE_SURFACE_ROUTE = "/api/release/control-plane/surface"
DEFAULT_RELEASE_CONTROL_PLANE_CATALOG_ROUTE = "/api/release/control-plane/catalog"
DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE = "/api/release/control-plane/action"
DEFAULT_RELEASE_CONTROL_PLANE_NEXT_ROUTE = "/api/release/control-plane/next"
DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE = (
    "/api/release/control-plane/request-templates"
)
DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE = (
    "/api/release/control-plane/request-file"
)
DEFAULT_RELEASE_NEXT_ROUTE = "/api/release/next"
DEFAULT_RELEASE_NEXT_PORTAL_ROUTE = "/static/release-next.html"
DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE = "/api/release/next/primary"
DEFAULT_RELEASE_NEXT_FOLLOW_UP_ROUTE = "/api/release/next/follow-up"
DEFAULT_RELEASE_NEXT_REQUEST_FILE_ROUTE = "/api/release/next/request-file"
DEFAULT_RELEASE_CLOSEOUT_ROUTE = "/api/release/closeout"
DEFAULT_RELEASE_CLOSEOUT_NEXT_ROUTE = "/api/release/closeout/next"
DEFAULT_RELEASE_CLOSEOUT_COMPONENT_ROUTE = "/api/release/closeout/component"
DEFAULT_RELEASE_CLOSEOUT_PLAN_ROUTE = "/api/release/closeout/plan"
DEFAULT_RELEASE_CLOSEOUT_PLAN_STAGE_ROUTE = "/api/release/closeout/plan/stage"
DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE = "/api/release/closeout/plan/next"
DEFAULT_RELEASE_CLOSEOUT_PLAN_PORTAL_ROUTE = "/static/release-closeout-plan.html"
DEFAULT_RELEASE_READINESS_REPORT_PATH = (
    "test_env/release_readiness_ready/release_readiness_report.json"
)
DEFAULT_SECURITY_RELEASE_PREFLIGHT_REPORT_PATH = (
    "test_env/release_evidence/security_release_preflight_report.json"
)
DEFAULT_WORKTREE_RELEASE_BLOCKER_REPORT_PATH = (
    "test_env/worktree_release_blocker/worktree_release_blocker_report.json"
)


def _resolve_release_surface_path(
    project_root: str | Path | None,
    artifact_path: str | Path,
) -> Path:
    root = Path(project_root) if project_root else Path.cwd()
    candidate = Path(artifact_path)
    if candidate.is_absolute():
        return candidate
    return root / candidate


def _load_json_object(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    if not isinstance(payload, dict):
        return {}
    return payload


def _build_external_mainline_closeout_component(
    payload: dict[str, Any],
    *,
    path: Path,
) -> dict[str, Any]:
    if not payload:
        return {
            "status": "missing",
            "summary": f"external_mainline_execution_plan is missing: {path}",
            "path": str(path),
            "command": None,
            "ready_to_run_steps": 0,
            "waiting_external_input_steps": 0,
            "blocked_steps": 0,
            "completed_steps": 0,
        }
    blocked_steps = int(payload.get("blocked_steps", 0) or 0)
    waiting_steps = int(payload.get("waiting_external_input_steps", 0) or 0)
    ready_steps = int(payload.get("ready_to_run_steps", 0) or 0)
    completed_steps = int(payload.get("completed_steps", 0) or 0)
    if blocked_steps > 0:
        status = "blocked"
    elif waiting_steps > 0:
        status = "waiting_external_input"
    elif ready_steps > 0:
        status = "ready_to_run"
    elif completed_steps > 0:
        status = "completed"
    else:
        status = str(payload.get("status") or "missing")
    next_step = {}
    for item in payload.get("steps", []):
        if not isinstance(item, dict):
            continue
        if str(item.get("status") or "").strip() in {
            "blocked",
            "waiting_external_input",
            "ready_to_run",
        }:
            next_step = item
            break
    return {
        "status": status,
        "summary": str(payload.get("summary") or "").strip()
        or f"external_mainline_execution_plan loaded from {path}",
        "path": str(path),
        "command": str(next_step.get("command") or "").strip()
        or build_run_external_mainline_execution_plan_command(output_path=str(path)),
        "next_step_id": str(next_step.get("id") or "").strip() or None,
        "next_step_status": str(next_step.get("status") or "").strip() or None,
        "blocking_inputs": (
            list(next_step.get("blocking_inputs", []))
            if isinstance(next_step.get("blocking_inputs"), list)
            else []
        ),
        "ready_to_run_steps": ready_steps,
        "waiting_external_input_steps": waiting_steps,
        "blocked_steps": blocked_steps,
        "completed_steps": completed_steps,
    }


def _build_vulnerability_exception_closeout_component(
    preflight_payload: dict[str, Any],
    *,
    preflight_path: Path,
    review_payload: dict[str, Any],
    review_path: Path,
) -> dict[str, Any]:
    metrics = (
        preflight_payload.get("metrics")
        if isinstance(preflight_payload.get("metrics"), dict)
        else {}
    )
    review_status = str(
        metrics.get("vulnerability_exception_review_status")
        or review_payload.get("status")
        or "missing"
    ).strip()
    review_candidate_count = int(
        metrics.get("vulnerability_exception_review_candidate_count")
        or review_payload.get("metrics", {}).get("review_candidate_count")
        or 0
    )
    review_due_count = int(
        metrics.get("vulnerability_exception_review_due")
        or review_payload.get("metrics", {}).get("review_due_exception_count")
        or 0
    )
    stale_exception_count = int(
        metrics.get("stale_vulnerability_exceptions")
        or review_payload.get("metrics", {}).get("stale_exception_count")
        or 0
    )
    next_exception_expiry = (
        metrics.get("next_exception_expiry")
        or review_payload.get("metrics", {}).get("next_exception_expiry")
        or ""
    )
    if stale_exception_count > 0 or review_status == "expired":
        status = "blocked"
    elif (
        review_status in {"review_due", "waiting_external_input"}
        or review_due_count > 0
        or review_candidate_count > 0
    ):
        status = "waiting_external_input"
    elif preflight_payload or review_payload:
        status = "completed"
    else:
        status = "missing"
    summary = (
        str(preflight_payload.get("summary") or "").strip()
        or str(review_payload.get("summary") or "").strip()
    )
    if not summary:
        summary = (
            f"vulnerability exception closeout {status}: "
            f"review_candidate_count={review_candidate_count}, "
            f"review_due_exception_count={review_due_count}, "
            f"stale_exception_count={stale_exception_count}."
        )
    blocking_inputs: list[str] = []
    if status in {"waiting_external_input", "blocked"}:
        if review_due_count > 0 or review_candidate_count > 0:
            blocking_inputs.append(
                "更新 vulnerability exception review / replacement 结论"
            )
        if stale_exception_count > 0:
            blocking_inputs.append("替换已失效的 no-fix exceptions")
        if next_exception_expiry:
            blocking_inputs.append(
                f"在 {str(next_exception_expiry).strip()} 前完成 review / replacement"
            )
    return {
        "status": status,
        "summary": summary,
        "security_release_preflight_path": str(preflight_path),
        "review_report_path": str(review_path),
        "command": build_vulnerability_exception_review_report_command(
            output_path=str(review_path)
        ),
        "blocking_inputs": blocking_inputs,
        "review_status": review_status,
        "review_candidate_count": review_candidate_count,
        "review_due_exception_count": review_due_count,
        "stale_exception_count": stale_exception_count,
        "next_exception_expiry": str(next_exception_expiry).strip(),
    }


def _build_worktree_closeout_component(
    readiness_payload: dict[str, Any],
    *,
    readiness_path: Path,
    fallback_payload: dict[str, Any],
    fallback_path: Path,
) -> dict[str, Any]:
    worktree_payload = (
        readiness_payload.get("worktree_release_blocker")
        if isinstance(readiness_payload.get("worktree_release_blocker"), dict)
        else fallback_payload
    )
    report_path = (
        readiness_path if worktree_payload is not fallback_payload else fallback_path
    )
    if not isinstance(worktree_payload, dict) or not worktree_payload:
        return {
            "status": "missing",
            "summary": f"worktree release blocker report is missing: {fallback_path}",
            "path": str(fallback_path),
            "command": build_run_worktree_release_blocker_command(),
            "clean_worktree": False,
            "total_paths": 0,
            "tracked_review_candidate_count": 0,
        }
    status = str(worktree_payload.get("status") or "missing").strip()
    return {
        "status": status,
        "summary": str(worktree_payload.get("summary") or "").strip()
        or f"worktree release blocker loaded from {report_path}",
        "path": str(report_path),
        "command": build_run_worktree_release_blocker_command(),
        "blocking_inputs": ["clean_worktree"] if status == "blocked" else [],
        "clean_worktree": worktree_payload.get("clean_worktree"),
        "total_paths": int(worktree_payload.get("total_paths", 0) or 0),
        "tracked_review_candidate_count": int(
            worktree_payload.get("tracked_review_candidate_count", 0) or 0
        ),
    }


def build_release_closeout_payload(
    *,
    project_root: str | Path | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    external_mainline_path = _resolve_release_surface_path(
        project_root,
        external_mainline_execution_plan_path
        or default_external_mainline_execution_plan_path(),
    )
    security_preflight_path = _resolve_release_surface_path(
        project_root,
        security_release_preflight_report_path
        or DEFAULT_SECURITY_RELEASE_PREFLIGHT_REPORT_PATH,
    )
    vulnerability_review_path = _resolve_release_surface_path(
        project_root,
        vulnerability_exception_review_report_path
        or default_vulnerability_exception_review_report_path(),
    )
    readiness_path = _resolve_release_surface_path(
        project_root,
        release_readiness_report_path or DEFAULT_RELEASE_READINESS_REPORT_PATH,
    )
    worktree_path = _resolve_release_surface_path(
        project_root,
        worktree_release_blocker_report_path
        or DEFAULT_WORKTREE_RELEASE_BLOCKER_REPORT_PATH,
    )

    external_mainline = _build_external_mainline_closeout_component(
        _load_json_object(external_mainline_path),
        path=external_mainline_path,
    )
    vulnerability_exception_review = _build_vulnerability_exception_closeout_component(
        _load_json_object(security_preflight_path),
        preflight_path=security_preflight_path,
        review_payload=_load_json_object(vulnerability_review_path),
        review_path=vulnerability_review_path,
    )
    worktree_release_blocker = _build_worktree_closeout_component(
        _load_json_object(readiness_path),
        readiness_path=readiness_path,
        fallback_payload=_load_json_object(worktree_path),
        fallback_path=worktree_path,
    )

    components = [
        external_mainline,
        vulnerability_exception_review,
        worktree_release_blocker,
    ]
    blocked_components = sum(
        1 for item in components if item.get("status") == "blocked"
    )
    waiting_external_input_components = sum(
        1 for item in components if item.get("status") == "waiting_external_input"
    )
    ready_to_run_components = sum(
        1 for item in components if item.get("status") == "ready_to_run"
    )
    missing_components = sum(
        1 for item in components if item.get("status") == "missing"
    )
    if blocked_components > 0:
        closeout_status = "blocked"
    elif (
        waiting_external_input_components > 0
        or ready_to_run_components > 0
        or missing_components > 0
    ):
        closeout_status = "action_required"
    else:
        closeout_status = "ready"
    closeout_summary = (
        "Release closeout "
        f"{closeout_status}: "
        f"external_mainline={external_mainline.get('status')}, "
        f"exception_review={vulnerability_exception_review.get('status')}, "
        f"worktree={worktree_release_blocker.get('status')}."
    )
    action_items = []
    for component_name, component_payload in [
        ("external_mainline", external_mainline),
        ("vulnerability_exception_review", vulnerability_exception_review),
        ("worktree_release_blocker", worktree_release_blocker),
    ]:
        action_items.append(
            {
                "component": component_name,
                "status": component_payload.get("status"),
                "summary": component_payload.get("summary"),
                "command": component_payload.get("command"),
                "blocking_inputs": component_payload.get("blocking_inputs", []),
                "component_route": f"/static/release-closeout.html?component={quote(component_name)}",
                "component_api_route": f"{DEFAULT_RELEASE_CLOSEOUT_COMPONENT_ROUTE}?component={quote(component_name)}",
            }
        )
    next_item = (
        action_items[0] if action_items and isinstance(action_items[0], dict) else {}
    )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CLOSEOUT_ROUTE,
        "release_closeout": {
            "status": closeout_status,
            "summary": closeout_summary,
            "blocked_components": blocked_components,
            "waiting_external_input_components": waiting_external_input_components,
            "ready_to_run_components": ready_to_run_components,
            "missing_components": missing_components,
            "action_items": action_items,
            "next_component": next_item.get("component"),
            "next_component_status": next_item.get("status"),
            "next_command": next_item.get("command"),
            "next_component_route": next_item.get("component_route"),
            "next_component_api_route": next_item.get("component_api_route"),
            "next_component_next_route": DEFAULT_RELEASE_CLOSEOUT_NEXT_ROUTE,
            "external_mainline": external_mainline,
            "vulnerability_exception_review": vulnerability_exception_review,
            "worktree_release_blocker": worktree_release_blocker,
        },
    }


def build_release_closeout_component_payload(
    *,
    component: str,
    project_root: str | Path | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    payload = build_release_closeout_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    closeout = payload.get("release_closeout", {})
    component_payload = closeout.get(component)
    if not isinstance(component_payload, dict):
        raise ValueError(f"unsupported release closeout component: {component}")
    action_item = next(
        (
            item
            for item in closeout.get("action_items", [])
            if isinstance(item, dict) and item.get("component") == component
        ),
        {},
    )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CLOSEOUT_COMPONENT_ROUTE,
        "closeout_route": DEFAULT_RELEASE_CLOSEOUT_ROUTE,
        "component": component,
        "component_route": f"/static/release-closeout.html?component={quote(component)}",
        "component_api_route": (
            f"{DEFAULT_RELEASE_CLOSEOUT_COMPONENT_ROUTE}?component={quote(component)}"
        ),
        "action_item": action_item,
        "closeout_component": component_payload,
    }


def build_release_closeout_next_payload(
    *,
    project_root: str | Path | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    payload = build_release_closeout_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    closeout = (
        payload.get("release_closeout")
        if isinstance(payload.get("release_closeout"), dict)
        else {}
    )
    action_items = (
        closeout.get("action_items")
        if isinstance(closeout.get("action_items"), list)
        else []
    )
    next_item = (
        action_items[0] if action_items and isinstance(action_items[0], dict) else {}
    )
    component = str(next_item.get("component") or "").strip()
    if not component:
        return {
            "status": "missing",
            "route": DEFAULT_RELEASE_CLOSEOUT_NEXT_ROUTE,
            "closeout_route": payload.get("route"),
            "message": "No recommended next closeout component is available.",
        }
    component_payload = build_release_closeout_component_payload(
        component=component,
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CLOSEOUT_NEXT_ROUTE,
        "closeout_route": payload.get("route"),
        "next_component": component,
        "next_component_status": next_item.get("status"),
        "next_command": next_item.get("command"),
        "component_route": component_payload.get("component_route"),
        "component_api_route": component_payload.get("component_api_route"),
        "action_item": component_payload.get("action_item", {}),
        "closeout_component": component_payload.get("closeout_component", {}),
        "component_payload": component_payload,
    }


def build_release_closeout_plan_payload(
    *,
    project_root: str | Path | None = None,
    external_mainline_execution_plan_path: str | None = None,
    external_mainline_inputs_path: str | None = None,
    external_mainline_input_checklist_report_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    closeout_payload = build_release_closeout_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    resolved_inputs_path = _resolve_release_surface_path(
        project_root,
        external_mainline_inputs_path or default_external_mainline_inputs_path(),
    )
    resolved_checklist_path = _resolve_release_surface_path(
        project_root,
        external_mainline_input_checklist_report_path
        or default_external_mainline_input_checklist_report_path(),
    )
    inputs_payload = _load_json_object(resolved_inputs_path)
    checklist_payload = _load_json_object(resolved_checklist_path)
    closeout = (
        closeout_payload.get("release_closeout")
        if isinstance(closeout_payload.get("release_closeout"), dict)
        else {}
    )
    external_mainline = (
        closeout.get("external_mainline")
        if isinstance(closeout.get("external_mainline"), dict)
        else {}
    )
    vulnerability_exception_review = (
        closeout.get("vulnerability_exception_review")
        if isinstance(closeout.get("vulnerability_exception_review"), dict)
        else {}
    )
    worktree_release_blocker = (
        closeout.get("worktree_release_blocker")
        if isinstance(closeout.get("worktree_release_blocker"), dict)
        else {}
    )
    plan_payload = _load_json_object(
        _resolve_release_surface_path(
            project_root,
            external_mainline_execution_plan_path
            or default_external_mainline_execution_plan_path(),
        )
    )
    plan_steps = {
        str(item.get("id") or "").strip(): item
        for item in plan_payload.get("steps", [])
        if isinstance(item, dict)
    }
    checklist_metrics = (
        checklist_payload.get("metrics")
        if isinstance(checklist_payload.get("metrics"), dict)
        else {}
    )
    customer_inputs = (
        dict(inputs_payload.get("customer_external_bindings"))
        if isinstance(inputs_payload.get("customer_external_bindings"), dict)
        else {}
    )
    industrial_inputs = (
        dict(inputs_payload.get("industrial_live_evidence"))
        if isinstance(inputs_payload.get("industrial_live_evidence"), dict)
        else {}
    )
    customer_sections = customer_inputs.get("sections")
    if not isinstance(customer_sections, list) or not customer_sections:
        customer_sections = ["approval_identity", "archive_target", "due_trigger"]
    customer_config_path = str(
        customer_inputs.get("config")
        or default_customer_external_bindings_config_path()
    )
    customer_overrides_path = str(
        customer_inputs.get("overrides_file")
        or "deployment/customer_delivery.external_bindings.customer.overrides.json"
    )
    customer_step = (
        plan_steps.get("customer_external_bindings_closure")
        if isinstance(plan_steps.get("customer_external_bindings_closure"), dict)
        else {}
    )
    vulnerability_step = (
        plan_steps.get("vulnerability_exception_replacement")
        if isinstance(plan_steps.get("vulnerability_exception_replacement"), dict)
        else {}
    )
    industrial_step = (
        plan_steps.get("industrial_delivery_live_evidence")
        if isinstance(plan_steps.get("industrial_delivery_live_evidence"), dict)
        else {}
    )

    def _stage(
        stage_id: str,
        *,
        label: str,
        status: str,
        component: str,
        summary: str,
        required_inputs: list[str],
        input_files: list[str],
        commands: list[dict[str, str]],
        done_when: list[str],
    ) -> dict[str, Any]:
        return {
            "id": stage_id,
            "label": label,
            "status": status,
            "component": component,
            "summary": summary,
            "required_inputs": required_inputs,
            "input_files": input_files,
            "commands": commands,
            "done_when": done_when,
            "stage_route": f"{DEFAULT_RELEASE_CLOSEOUT_PLAN_PORTAL_ROUTE}?stage={quote(stage_id)}",
            "stage_api_route": f"{DEFAULT_RELEASE_CLOSEOUT_PLAN_STAGE_ROUTE}?stage={quote(stage_id)}",
        }

    customer_stage = _stage(
        "customer_external_bindings_inputs",
        label="Close Customer External Bindings Inputs",
        status=str(
            customer_step.get("status") or external_mainline.get("status") or "missing"
        ),
        component="external_mainline",
        summary=str(
            customer_step.get("summary")
            or external_mainline.get("summary")
            or "customer external bindings inputs are still incomplete."
        ),
        required_inputs=[
            str(item)
            for item in (customer_step.get("blocking_inputs") or [])
            if isinstance(item, str) and item.strip()
        ],
        input_files=[
            default_external_mainline_inputs_path(),
            customer_config_path,
            customer_overrides_path,
        ],
        commands=[
            {
                "label": "Generate customer-specific config",
                "command": build_customer_external_bindings_config_command(
                    output_path=customer_config_path
                ),
            },
            {
                "label": "Confirm customer bindings",
                "command": build_confirm_customer_external_bindings_command(
                    config_path=customer_config_path,
                    sections=customer_sections,
                ),
            },
            {
                "label": "Run managed closure chain",
                "command": build_run_customer_external_bindings_closure_command(
                    config_path=customer_config_path,
                    sections=customer_sections,
                ),
            },
        ],
        done_when=[
            "customer external bindings config no longer requires draft/confirmation inputs",
            "customer_external_bindings_closure_report.json exists and reports status=passed",
        ],
    )
    vulnerability_stage = _stage(
        "vulnerability_exception_replacement_evidence",
        label="Collect Vulnerability Replacement Evidence",
        status=str(
            vulnerability_step.get("status")
            or vulnerability_exception_review.get("status")
            or "missing"
        ),
        component="vulnerability_exception_review",
        summary=str(
            vulnerability_step.get("summary")
            or vulnerability_exception_review.get("summary")
            or "vulnerability exception replacement evidence is still incomplete."
        ),
        required_inputs=[
            str(item)
            for item in (vulnerability_step.get("blocking_inputs") or [])
            if isinstance(item, str) and item.strip()
        ],
        input_files=[
            "deployment/security/vulnerability_exceptions.input.json",
            default_vulnerability_exception_review_report_path(),
            DEFAULT_SECURITY_RELEASE_PREFLIGHT_REPORT_PATH,
        ],
        commands=[
            {
                "label": "Refresh exception review report",
                "command": build_vulnerability_exception_review_report_command(
                    output_path=default_vulnerability_exception_review_report_path()
                ),
            }
        ],
        done_when=[
            "latest scanner/fix evidence is reflected in vulnerability_exceptions.input.json",
            "vulnerability_exception_replacement no longer waits on review/replacement evidence",
        ],
    )
    industrial_stage = _stage(
        "industrial_live_evidence_inputs",
        label="Collect Real Industrial Live-Evidence Inputs",
        status=str(
            industrial_step.get("status")
            or external_mainline.get("status")
            or "missing"
        ),
        component="external_mainline",
        summary=str(
            industrial_step.get("summary")
            or "industrial live evidence inputs are still incomplete."
        ),
        required_inputs=[
            str(item)
            for item in (industrial_step.get("blocking_inputs") or [])
            if isinstance(item, str) and item.strip()
        ],
        input_files=[
            default_external_mainline_inputs_path(),
            str(
                industrial_inputs.get("evidence_output_root")
                or "test_env/industrial_live_evidence"
            ),
            "test_env/release_rehearsal_industrial/industrial_delivery_rehearsal_report.json",
        ],
        commands=[
            {
                "label": "Refresh industrial rehearsal baseline",
                "command": str(industrial_step.get("command") or ""),
            }
        ],
        done_when=[
            "external_mainline.inputs.json contains real environment/access/install/upgrade/rollback/backup/closure fields",
            "industrial_delivery_live_evidence no longer waits on real environment inputs",
        ],
    )

    upstream_statuses = {
        customer_stage["status"],
        vulnerability_stage["status"],
        industrial_stage["status"],
    }
    if upstream_statuses == {"completed"}:
        rebuild_status = "ready_to_run"
    elif "blocked" in upstream_statuses:
        rebuild_status = "blocked"
    else:
        rebuild_status = "pending"
    rebuild_stage = _stage(
        "canonical_external_mainline_rebuild",
        label="Rebuild Canonical External Mainline Evidence",
        status=rebuild_status,
        component="external_mainline",
        summary=(
            "rebuild canonical external-mainline plan/checklist/evidence once all upstream inputs are closed."
            if rebuild_status != "completed"
            else "canonical external-mainline artifacts already reflect closed upstream inputs."
        ),
        required_inputs=(
            [] if rebuild_status == "ready_to_run" else ["complete stages 1-3 first"]
        ),
        input_files=[
            default_external_mainline_execution_plan_path(),
            default_external_mainline_input_checklist_report_path(),
            default_external_mainline_inputs_path(),
        ],
        commands=[
            {
                "label": "Run external mainline runner",
                "command": build_run_external_mainline_execution_plan_command(
                    output_path=default_external_mainline_execution_plan_path(),
                    inputs_file=default_external_mainline_inputs_path(),
                ),
            },
            {
                "label": "Rebuild external-mainline checklist",
                "command": build_external_mainline_input_checklist_command(
                    output_path=default_external_mainline_input_checklist_report_path(),
                    inputs_file=default_external_mainline_inputs_path(),
                    external_mainline_execution_plan_path=default_external_mainline_execution_plan_path(),
                ),
            },
            {
                "label": "Collect canonical release evidence",
                "command": "python tools/collect_release_evidence.py",
            },
        ],
        done_when=[
            "external_mainline_execution_plan.waiting_external_input_steps == 0",
            "external_mainline_input_checklist.missing_input_count == 0",
        ],
    )

    missing_input_count = int(checklist_metrics.get("missing_input_count") or 0)
    if (
        closeout.get("status") == "ready"
        and missing_input_count == 0
        and worktree_release_blocker.get("status") == "ready"
    ):
        acceptance_status = "completed"
    elif rebuild_stage["status"] == "ready_to_run":
        acceptance_status = "pending"
    elif rebuild_stage["status"] == "blocked":
        acceptance_status = "blocked"
    else:
        acceptance_status = "pending"
    acceptance_stage = _stage(
        "release_closeout_acceptance_review",
        label="Review Final Closeout Acceptance",
        status=acceptance_status,
        component="release_closeout",
        summary=str(
            closeout.get("summary")
            or "review release_closeout and release_next surfaces after canonical rebuild."
        ),
        required_inputs=(
            []
            if acceptance_status == "completed"
            else [
                "close all upstream external-input stages and rerun canonical closeout"
            ]
        ),
        input_files=[
            default_external_mainline_execution_plan_path(),
            default_external_mainline_input_checklist_report_path(),
            DEFAULT_RELEASE_READINESS_REPORT_PATH,
        ],
        commands=[
            {
                "label": "Review release closeout JSON",
                "command": "GET /api/release/closeout",
            },
            {
                "label": "Review unified next step",
                "command": "GET /api/release/next",
            },
        ],
        done_when=[
            "release_closeout.status == ready",
            "release_next no longer points to unresolved external-input blockers",
        ],
    )

    stages = [
        customer_stage,
        vulnerability_stage,
        industrial_stage,
        rebuild_stage,
        acceptance_stage,
    ]
    completed_stages = sum(1 for item in stages if item.get("status") == "completed")
    blocked_stages = sum(1 for item in stages if item.get("status") == "blocked")
    pending_stages = len(stages) - completed_stages
    if blocked_stages > 0:
        plan_status = "blocked"
    elif pending_stages > 0:
        plan_status = "action_required"
    else:
        plan_status = "ready"
    next_stage = next(
        (item for item in stages if item.get("status") != "completed"),
        stages[-1] if stages else {},
    )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CLOSEOUT_PLAN_ROUTE,
        "portal_route": DEFAULT_RELEASE_CLOSEOUT_PLAN_PORTAL_ROUTE,
        "next_route": DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE,
        "closeout_route": closeout_payload.get("route"),
        "external_mainline_inputs_path": str(
            Path(default_external_mainline_inputs_path()).as_posix()
        ),
        "plan_status": plan_status,
        "summary": (
            f"release_closeout_plan {plan_status}: "
            f"completed={completed_stages}, pending={pending_stages}, blocked={blocked_stages}."
        ),
        "completed_stages": completed_stages,
        "pending_stages": pending_stages,
        "blocked_stages": blocked_stages,
        "missing_input_count": missing_input_count,
        "stages": stages,
        "next_stage_id": next_stage.get("id"),
        "next_stage_label": next_stage.get("label"),
        "next_stage_status": next_stage.get("status"),
        "next_stage_route": next_stage.get("stage_route"),
        "next_stage_api_route": next_stage.get("stage_api_route"),
        "next_stage_next_route": DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE,
        "next_stage_component": next_stage.get("component"),
        "next_stage_commands": next_stage.get("commands", []),
        "release_closeout": closeout,
    }


def build_release_closeout_plan_stage_payload(
    stage: str,
    *,
    project_root: str | Path | None = None,
    external_mainline_execution_plan_path: str | None = None,
    external_mainline_inputs_path: str | None = None,
    external_mainline_input_checklist_report_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    stage_id = str(stage or "").strip()
    if not stage_id:
        raise ValueError("stage is required")
    payload = build_release_closeout_plan_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        external_mainline_inputs_path=external_mainline_inputs_path,
        external_mainline_input_checklist_report_path=external_mainline_input_checklist_report_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    stage_payload = next(
        (
            item
            for item in payload.get("stages", [])
            if isinstance(item, dict) and item.get("id") == stage_id
        ),
        None,
    )
    if not isinstance(stage_payload, dict):
        raise ValueError(f"unknown closeout plan stage: {stage_id}")
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CLOSEOUT_PLAN_STAGE_ROUTE,
        "plan_route": payload.get("route"),
        "plan_portal_route": payload.get("portal_route"),
        "plan_next_route": payload.get(
            "next_route", DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE
        ),
        "plan_status": payload.get("plan_status"),
        "stage": stage_id,
        "is_next_stage": payload.get("next_stage_id") == stage_id,
        "stage_label": stage_payload.get("label"),
        "stage_status": stage_payload.get("status"),
        "stage_component": stage_payload.get("component"),
        "stage_route": stage_payload.get("stage_route"),
        "stage_api_route": stage_payload.get("stage_api_route"),
        "stage_payload": stage_payload,
        "release_closeout": payload.get("release_closeout", {}),
    }


def build_release_closeout_plan_next_payload(
    *,
    project_root: str | Path | None = None,
    external_mainline_execution_plan_path: str | None = None,
    external_mainline_inputs_path: str | None = None,
    external_mainline_input_checklist_report_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    payload = build_release_closeout_plan_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        external_mainline_inputs_path=external_mainline_inputs_path,
        external_mainline_input_checklist_report_path=external_mainline_input_checklist_report_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    stage_id = str(payload.get("next_stage_id") or "").strip()
    if not stage_id:
        return {
            "status": "error",
            "route": DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE,
            "message": "No recommended next closeout plan stage is available.",
            "plan_route": payload.get("route"),
        }
    stage_payload = build_release_closeout_plan_stage_payload(
        stage_id,
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        external_mainline_inputs_path=external_mainline_inputs_path,
        external_mainline_input_checklist_report_path=external_mainline_input_checklist_report_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE,
        "plan_route": payload.get("route"),
        "plan_portal_route": payload.get("portal_route"),
        "plan_status": payload.get("plan_status"),
        "next_stage_id": stage_id,
        "next_stage_label": payload.get("next_stage_label"),
        "next_stage_status": payload.get("next_stage_status"),
        "next_stage_component": payload.get("next_stage_component"),
        "next_stage_route": payload.get("next_stage_route"),
        "next_stage_api_route": payload.get("next_stage_api_route"),
        "next_stage_commands": payload.get("next_stage_commands", []),
        "stage_payload": stage_payload.get("stage_payload", {}),
        "release_closeout": payload.get("release_closeout", {}),
    }


def build_release_closeout_summary(
    payload: dict[str, Any] | None = None,
    *,
    project_root: str | Path | None = None,
) -> dict[str, Any]:
    if payload is None:
        payload = build_release_closeout_payload(project_root=project_root)
    closeout = payload.get("release_closeout", {})
    summary = {
        "route": DEFAULT_RELEASE_CLOSEOUT_ROUTE,
        "status": closeout.get("status", "missing"),
        "blocked_components": closeout.get("blocked_components", 0),
        "waiting_external_input_components": closeout.get(
            "waiting_external_input_components", 0
        ),
        "ready_to_run_components": closeout.get("ready_to_run_components", 0),
        "missing_components": closeout.get("missing_components", 0),
        "action_items_count": len(
            closeout.get("action_items", [])
            if isinstance(closeout.get("action_items"), list)
            else []
        ),
        "top_action_items": [
            {
                "component": item.get("component"),
                "status": item.get("status"),
                "command": item.get("command"),
                "component_route": item.get("component_route"),
                "component_api_route": item.get("component_api_route"),
            }
            for item in (
                closeout.get("action_items", [])
                if isinstance(closeout.get("action_items"), list)
                else []
            )[:2]
            if isinstance(item, dict)
        ],
    }
    for field in [
        "next_component",
        "next_component_status",
        "next_command",
        "next_component_route",
        "next_component_api_route",
        "next_component_next_route",
    ]:
        value = closeout.get(field)
        if value:
            summary[field] = value
    summary["plan_route"] = DEFAULT_RELEASE_CLOSEOUT_PLAN_ROUTE
    summary["plan_portal_route"] = DEFAULT_RELEASE_CLOSEOUT_PLAN_PORTAL_ROUTE
    summary["plan_next_route"] = DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE
    return summary


def build_release_control_plane_surface_payload(
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
) -> dict[str, Any]:
    resolved_root = Path(project_root) if project_root else Path.cwd()
    surface_payload = read_release_control_plane_surface(
        project_root=resolved_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
    )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CONTROL_PLANE_SURFACE_ROUTE,
        **surface_payload,
    }


def build_release_control_plane_request_file_payload(*, action: str) -> dict[str, Any]:
    action_payload = build_release_control_plane_action_payload(action=action)
    request_template = (
        action_payload.get("request_template")
        if isinstance(action_payload.get("request_template"), dict)
        else {}
    )
    request_file = (
        request_template.get("request_template")
        if isinstance(request_template.get("request_template"), dict)
        else {}
    )
    request_file_name = f"release_ops.{action}.request.json"
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE,
        "action": action,
        "action_route": action_payload.get("action_route"),
        "portal_route": action_payload.get("portal_route"),
        "request_template_route": action_payload.get("request_template_route"),
        "request_file_route": (
            f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action)}"
        ),
        "request_file_download_route": (
            f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action)}&download=1"
        ),
        "request_file_name": request_file_name,
        "content_type": "application/json",
        "request_file": request_file,
        "request_file_pretty_json": json.dumps(
            request_file, ensure_ascii=False, indent=2
        )
        + "\n",
    }


def build_release_ops_catalog_payload() -> dict[str, Any]:
    actions = []
    for item in list_release_ops_actions():
        action_name = str(item.get("action") or "").strip()
        actions.append(
            {
                **item,
                "action_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE}?action={quote(action_name)}"
                    if action_name
                    else DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE
                ),
                "request_template_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE}?action={quote(action_name)}"
                    if action_name
                    else DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE
                ),
                "request_file_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action_name)}"
                    if action_name
                    else DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE
                ),
                "request_file_download_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action_name)}&download=1"
                    if action_name
                    else f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?download=1"
                ),
                "request_file_name": (
                    f"release_ops.{action_name}.request.json"
                    if action_name
                    else "release_ops.request.json"
                ),
                "portal_route": (
                    f"/static/release-control-plane.html?action={quote(action_name)}"
                    if action_name
                    else "/static/release-control-plane.html"
                ),
            }
        )
    policy_profiles = list_release_ops_policy_profiles()
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CONTROL_PLANE_CATALOG_ROUTE,
        "actions_count": len(actions),
        "policy_profiles_count": len(policy_profiles),
        "actions": actions,
        "policy_profiles": policy_profiles,
    }


def build_release_ops_request_templates_payload(
    *, action: str | None = None
) -> dict[str, Any]:
    request_templates = []
    for item in list_release_ops_request_templates(action=action):
        action_name = str(item.get("action") or "").strip()
        request_templates.append(
            {
                **item,
                "action_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE}?action={quote(action_name)}"
                    if action_name
                    else DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE
                ),
                "request_template_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE}?action={quote(action_name)}"
                    if action_name
                    else DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE
                ),
                "request_file_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action_name)}"
                    if action_name
                    else DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE
                ),
                "request_file_download_route": (
                    f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action_name)}&download=1"
                    if action_name
                    else f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?download=1"
                ),
                "request_file_name": (
                    f"release_ops.{action_name}.request.json"
                    if action_name
                    else "release_ops.request.json"
                ),
                "portal_route": (
                    f"/static/release-control-plane.html?action={quote(action_name)}"
                    if action_name
                    else "/static/release-control-plane.html"
                ),
            }
        )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE,
        "request_templates_count": len(request_templates),
        "request_templates": request_templates,
    }


def build_release_control_plane_action_payload(*, action: str) -> dict[str, Any]:
    actions = build_release_ops_catalog_payload().get("actions", [])
    action_definition = next(
        (item for item in actions if item.get("action") == action),
        None,
    )
    if action_definition is None:
        raise ValueError(f"unsupported release op action: {action}")
    request_templates = build_release_ops_request_templates_payload(action=action).get(
        "request_templates", []
    )
    request_template = request_templates[0] if request_templates else {}
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE,
        "action": action,
        "action_route": f"{DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE}?action={quote(action)}",
        "portal_route": f"/static/release-control-plane.html?action={quote(action)}",
        "action_definition": action_definition,
        "request_template": request_template,
        "request_template_route": (
            f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE}?action={quote(action)}"
        ),
        "request_file_route": (
            f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action)}"
        ),
        "request_file_download_route": (
            f"{DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE}?action={quote(action)}&download=1"
        ),
        "request_file_name": f"release_ops.{action}.request.json",
    }


def build_release_control_plane_next_payload(
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
) -> dict[str, Any]:
    index_payload = build_release_control_plane_index_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
    )
    action = str(index_payload.get("next_action") or "").strip()
    if not action:
        return {
            "status": "missing",
            "route": DEFAULT_RELEASE_CONTROL_PLANE_NEXT_ROUTE,
            "control_plane_index_route": index_payload.get("route"),
            "control_plane_surface_route": index_payload.get(
                "control_plane_surface_route"
            ),
            "release_closeout_route": index_payload.get("release_closeout_route"),
            "control_plane_surface_source": index_payload.get(
                "control_plane_surface_source"
            ),
            "message": "No recommended next action is available.",
        }
    action_payload = build_release_control_plane_action_payload(action=action)
    request_file_payload = build_release_control_plane_request_file_payload(
        action=action
    )
    request_template = (
        action_payload.get("request_template")
        if isinstance(action_payload.get("request_template"), dict)
        else {}
    )
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CONTROL_PLANE_NEXT_ROUTE,
        "control_plane_index_route": index_payload.get("route"),
        "control_plane_surface_route": index_payload.get("control_plane_surface_route"),
        "release_closeout_route": index_payload.get("release_closeout_route"),
        "control_plane_surface_source": index_payload.get(
            "control_plane_surface_source"
        ),
        "next_action": action,
        "next_action_default_policy_profile": index_payload.get(
            "next_action_default_policy_profile"
        ),
        "action_route": action_payload.get("action_route"),
        "portal_route": action_payload.get("portal_route"),
        "request_template_route": action_payload.get("request_template_route"),
        "request_file_route": request_file_payload.get("request_file_route"),
        "request_file_download_route": request_file_payload.get(
            "request_file_download_route"
        ),
        "request_file_name": request_file_payload.get("request_file_name"),
        "action_definition": action_payload.get("action_definition", {}),
        "request_template": request_template,
        "request_file": request_file_payload.get("request_file", {}),
        "request_file_pretty_json": request_file_payload.get(
            "request_file_pretty_json"
        ),
        "content_type": request_file_payload.get("content_type"),
        "action_payload": action_payload,
        "request_file_payload": request_file_payload,
    }


def build_release_control_plane_index_payload(
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
) -> dict[str, Any]:
    surface_payload = build_release_control_plane_surface_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
    )
    closeout_payload = build_release_closeout_payload(project_root=project_root)
    catalog_payload = build_release_ops_catalog_payload()
    request_templates_payload = build_release_ops_request_templates_payload()
    next_action = {}
    request_templates = (
        request_templates_payload.get("request_templates")
        if isinstance(request_templates_payload.get("request_templates"), list)
        else []
    )
    actions = (
        catalog_payload.get("actions")
        if isinstance(catalog_payload.get("actions"), list)
        else []
    )
    if request_templates:
        next_action = (
            request_templates[0] if isinstance(request_templates[0], dict) else {}
        )
    elif actions:
        next_action = actions[0] if isinstance(actions[0], dict) else {}
    return {
        "status": "success",
        "route": DEFAULT_RELEASE_CONTROL_PLANE_ROUTE,
        "control_plane_surface_source": surface_payload.get("source"),
        "control_plane_surface_route": surface_payload.get("route"),
        "manifest_path": surface_payload.get("manifest_path"),
        "release_ops_execution_report_path": surface_payload.get(
            "release_ops_execution_report_path"
        ),
        "release_closeout_route": closeout_payload.get("route"),
        "control_plane_surface": surface_payload.get("control_plane_surface", {}),
        "release_closeout": closeout_payload.get("release_closeout", {}),
        "actions_count": catalog_payload.get("actions_count", 0),
        "policy_profiles_count": catalog_payload.get("policy_profiles_count", 0),
        "request_templates_count": request_templates_payload.get(
            "request_templates_count", 0
        ),
        "actions": catalog_payload.get("actions", []),
        "policy_profiles": catalog_payload.get("policy_profiles", []),
        "request_templates": request_templates_payload.get("request_templates", []),
        "next_action": next_action.get("action"),
        "next_action_default_policy_profile": next_action.get("default_policy_profile")
        or next_action.get("policy_level"),
        "next_action_route": next_action.get("portal_route"),
        "next_action_request_route": next_action.get("action_route"),
        "next_action_request_file_route": next_action.get("request_file_route"),
        "next_action_request_file_download_route": next_action.get(
            "request_file_download_route"
        ),
        "next_action_request_file_name": next_action.get("request_file_name"),
    }


def build_release_control_plane_index_summary(
    payload: dict[str, Any] | None = None,
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
) -> dict[str, Any]:
    if payload is None:
        payload = build_release_control_plane_index_payload(
            project_root=project_root,
            manifest_path=manifest_path,
            release_ops_execution_report_path=release_ops_execution_report_path,
        )
    surface = payload.get("control_plane_surface", {})
    release_closeout = (
        payload.get("release_closeout")
        if isinstance(payload.get("release_closeout"), dict)
        else {}
    )
    summary = {
        "route": DEFAULT_RELEASE_CONTROL_PLANE_ROUTE,
        "status": surface.get("status", "unknown"),
        "source": payload.get("control_plane_surface_source"),
        "release_closeout_status": release_closeout.get("status", "missing"),
        "actions_count": payload.get("actions_count", 0),
        "policy_profiles_count": payload.get("policy_profiles_count", 0),
        "request_templates_count": payload.get("request_templates_count", 0),
    }
    if isinstance(release_closeout, dict):
        for field in [
            "blocked_components",
            "waiting_external_input_components",
            "ready_to_run_components",
            "missing_components",
        ]:
            value = release_closeout.get(field)
            if isinstance(value, int):
                summary[f"release_closeout_{field}"] = value
    if isinstance(surface.get("event_count"), int):
        summary["event_count"] = surface["event_count"]
    release_ops_execution = surface.get("release_ops_execution")
    if isinstance(release_ops_execution, dict):
        if isinstance(release_ops_execution.get("status"), str):
            summary["release_ops_execution_status"] = release_ops_execution["status"]
        if isinstance(release_ops_execution.get("event_count"), int):
            summary["release_ops_execution_event_count"] = release_ops_execution[
                "event_count"
            ]
    request_templates = (
        payload.get("request_templates")
        if isinstance(payload.get("request_templates"), list)
        else []
    )
    next_action = request_templates[0] if request_templates else {}
    if isinstance(next_action, dict) and isinstance(next_action.get("action"), str):
        summary["next_action"] = next_action.get("action")
        summary["next_action_default_policy_profile"] = next_action.get(
            "default_policy_profile"
        ) or next_action.get("policy_level")
        summary["next_action_request_route"] = next_action.get("action_route")
        summary["next_action_route"] = next_action.get("portal_route")
        summary["next_action_request_file_route"] = next_action.get(
            "request_file_route"
        )
        summary["next_action_request_file_download_route"] = next_action.get(
            "request_file_download_route"
        )
        summary["next_action_request_file_name"] = next_action.get("request_file_name")
    return summary


def build_release_next_payload(
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    control_plane_next = build_release_control_plane_next_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
    )
    closeout_next = build_release_closeout_next_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    primary: dict[str, Any] = {}
    if closeout_next.get("status") == "success":
        primary = {
            "primary_kind": "closeout_component",
            "primary_name": closeout_next.get("next_component"),
            "primary_status": closeout_next.get("next_component_status"),
            "primary_portal_route": closeout_next.get("component_route"),
            "primary_api_route": closeout_next.get("component_api_route"),
            "primary_next_route": closeout_next.get("route"),
            "primary_command": closeout_next.get("next_command"),
        }
    elif control_plane_next.get("status") == "success":
        primary = {
            "primary_kind": "control_plane_action",
            "primary_name": control_plane_next.get("next_action"),
            "primary_status": control_plane_next.get(
                "next_action_default_policy_profile"
            )
            or (
                control_plane_next.get("action_definition", {}).get("policy_level")
                if isinstance(control_plane_next.get("action_definition"), dict)
                else None
            ),
            "primary_portal_route": control_plane_next.get("portal_route"),
            "primary_api_route": control_plane_next.get("action_route"),
            "primary_next_route": control_plane_next.get("route"),
            "primary_request_file_route": control_plane_next.get("request_file_route"),
            "primary_request_file_download_route": control_plane_next.get(
                "request_file_download_route"
            ),
            "primary_request_file_name": control_plane_next.get("request_file_name"),
        }
    payload = {
        "status": "success",
        "route": DEFAULT_RELEASE_NEXT_ROUTE,
        "portal_route": DEFAULT_RELEASE_NEXT_PORTAL_ROUTE,
        "primary_route": DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE,
        "primary_payload_route": DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE,
        "control_plane_next": control_plane_next,
        "release_closeout_next": closeout_next,
        **primary,
    }
    payload["primary_payload"] = build_release_next_primary_payload(payload=payload)
    payload["follow_up_payload"] = build_release_next_follow_up_payload(payload=payload)
    payload["request_file_payload"] = build_release_next_request_file_payload(
        payload=payload
    )
    return payload


def build_release_next_primary_payload(
    payload: dict[str, Any] | None = None,
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    if payload is None:
        payload = build_release_next_payload(
            project_root=project_root,
            manifest_path=manifest_path,
            release_ops_execution_report_path=release_ops_execution_report_path,
            external_mainline_execution_plan_path=external_mainline_execution_plan_path,
            security_release_preflight_report_path=security_release_preflight_report_path,
            vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
            release_readiness_report_path=release_readiness_report_path,
            worktree_release_blocker_report_path=worktree_release_blocker_report_path,
        )
    primary_payload = {
        "route": DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE,
        "primary_payload_route": DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE,
        "release_next_route": DEFAULT_RELEASE_NEXT_ROUTE,
        "release_next_portal_route": DEFAULT_RELEASE_NEXT_PORTAL_ROUTE,
        "primary_follow_up_payload_route": DEFAULT_RELEASE_NEXT_FOLLOW_UP_ROUTE,
        "primary_follow_up_download_route": None,
    }
    primary_name = payload.get("primary_name")
    if not primary_name:
        return {
            "status": "missing",
            **primary_payload,
        }
    for field in [
        "primary_kind",
        "primary_name",
        "primary_status",
        "primary_portal_route",
        "primary_api_route",
        "primary_next_route",
        "primary_command",
        "primary_request_file_route",
        "primary_request_file_download_route",
        "primary_request_file_name",
        "primary_follow_up_kind",
        "primary_follow_up_label",
        "primary_follow_up_route",
        "primary_follow_up_download_route",
        "primary_follow_up_text",
    ]:
        value = payload.get(field)
        if value:
            primary_payload[field] = value
    if primary_payload.get("primary_request_file_route"):
        primary_payload["primary_follow_up_kind"] = "request_file"
        primary_payload["primary_follow_up_label"] = "request-file 草稿"
        primary_payload["primary_follow_up_route"] = primary_payload.get(
            "primary_request_file_route"
        )
        if primary_payload.get("primary_request_file_download_route"):
            primary_payload["primary_follow_up_download_route"] = primary_payload.get(
                "primary_request_file_download_route"
            )
        primary_payload["primary_follow_up_text"] = primary_payload.get(
            "primary_request_file_name"
        ) or primary_payload.get("primary_request_file_route")
    elif primary_payload.get("primary_command"):
        primary_payload["primary_follow_up_kind"] = "command"
        primary_payload["primary_follow_up_label"] = "建议命令"
        if primary_payload.get("primary_next_route"):
            primary_payload["primary_follow_up_route"] = primary_payload.get(
                "primary_next_route"
            )
        primary_payload["primary_follow_up_text"] = primary_payload.get(
            "primary_command"
        )
    elif primary_payload.get("primary_next_route"):
        primary_payload["primary_follow_up_kind"] = "json"
        primary_payload["primary_follow_up_label"] = "下一步 JSON"
        primary_payload["primary_follow_up_route"] = primary_payload.get(
            "primary_next_route"
        )
        primary_payload["primary_follow_up_text"] = primary_payload.get(
            "primary_next_route"
        )
    return {
        "status": "success",
        **primary_payload,
    }


def build_release_next_follow_up_payload(
    payload: dict[str, Any] | None = None,
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    primary_payload = build_release_next_primary_payload(
        payload=payload,
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )
    follow_up_payload = {
        "route": DEFAULT_RELEASE_NEXT_FOLLOW_UP_ROUTE,
        "release_next_route": DEFAULT_RELEASE_NEXT_ROUTE,
        "release_next_primary_route": DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE,
        "release_next_portal_route": DEFAULT_RELEASE_NEXT_PORTAL_ROUTE,
        "follow_up_download_route": None,
    }
    if primary_payload.get("status") != "success":
        return {
            "status": "missing",
            **follow_up_payload,
        }
    for field in [
        "primary_kind",
        "primary_name",
        "primary_status",
        "primary_portal_route",
        "primary_api_route",
        "primary_next_route",
    ]:
        value = primary_payload.get(field)
        if value:
            follow_up_payload[field] = value
    follow_up_payload["follow_up_kind"] = primary_payload.get("primary_follow_up_kind")
    follow_up_payload["follow_up_label"] = primary_payload.get(
        "primary_follow_up_label"
    )
    follow_up_payload["follow_up_route"] = primary_payload.get(
        "primary_follow_up_route"
    )
    follow_up_payload["follow_up_download_route"] = primary_payload.get(
        "primary_follow_up_download_route"
    )
    follow_up_payload["follow_up_text"] = primary_payload.get("primary_follow_up_text")
    return {
        "status": "success",
        **follow_up_payload,
    }


def build_release_next_request_file_payload(
    payload: dict[str, Any] | None = None,
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    if payload is None:
        payload = build_release_next_payload(
            project_root=project_root,
            manifest_path=manifest_path,
            release_ops_execution_report_path=release_ops_execution_report_path,
            external_mainline_execution_plan_path=external_mainline_execution_plan_path,
            security_release_preflight_report_path=security_release_preflight_report_path,
            vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
            release_readiness_report_path=release_readiness_report_path,
            worktree_release_blocker_report_path=worktree_release_blocker_report_path,
        )
    request_file_payload = {
        "status": "missing",
        "route": DEFAULT_RELEASE_NEXT_REQUEST_FILE_ROUTE,
        "release_next_route": DEFAULT_RELEASE_NEXT_ROUTE,
        "release_next_primary_route": DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE,
        "release_next_follow_up_route": DEFAULT_RELEASE_NEXT_FOLLOW_UP_ROUTE,
        "release_next_portal_route": DEFAULT_RELEASE_NEXT_PORTAL_ROUTE,
        "request_file_download_route": None,
    }
    primary_payload = (
        payload.get("primary_payload")
        if isinstance(payload.get("primary_payload"), dict)
        else build_release_next_primary_payload(payload=payload)
    )
    if primary_payload.get("status") != "success":
        return request_file_payload
    for field in ["primary_kind", "primary_name", "primary_status"]:
        value = primary_payload.get(field)
        if value:
            request_file_payload[field] = value
    if (
        primary_payload.get("primary_kind") != "control_plane_action"
        or not str(primary_payload.get("primary_name") or "").strip()
        or primary_payload.get("primary_follow_up_kind") != "request_file"
    ):
        return request_file_payload
    action = str(primary_payload.get("primary_name") or "").strip()
    control_plane_request_file = build_release_control_plane_request_file_payload(
        action=action
    )
    request_file_payload.update(
        {
            "status": "success",
            "action": action,
            "action_route": control_plane_request_file.get("action_route"),
            "portal_route": control_plane_request_file.get("portal_route"),
            "request_template_route": control_plane_request_file.get(
                "request_template_route"
            ),
            "request_file_route": DEFAULT_RELEASE_NEXT_REQUEST_FILE_ROUTE,
            "request_file_download_route": (
                f"{DEFAULT_RELEASE_NEXT_REQUEST_FILE_ROUTE}?download=1"
            ),
            "request_file_name": control_plane_request_file.get("request_file_name"),
            "content_type": control_plane_request_file.get("content_type"),
            "request_file": control_plane_request_file.get("request_file"),
            "request_file_pretty_json": control_plane_request_file.get(
                "request_file_pretty_json"
            ),
            "source_request_file_route": control_plane_request_file.get(
                "request_file_route"
            ),
            "source_request_file_download_route": control_plane_request_file.get(
                "request_file_download_route"
            ),
        }
    )
    return request_file_payload


def build_release_next_summary(
    payload: dict[str, Any] | None = None,
    *,
    project_root: str | Path | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> dict[str, Any]:
    if payload is None:
        payload = build_release_next_payload(
            project_root=project_root,
            manifest_path=manifest_path,
            release_ops_execution_report_path=release_ops_execution_report_path,
            external_mainline_execution_plan_path=external_mainline_execution_plan_path,
            security_release_preflight_report_path=security_release_preflight_report_path,
            vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
            release_readiness_report_path=release_readiness_report_path,
            worktree_release_blocker_report_path=worktree_release_blocker_report_path,
        )
    summary = {
        "route": DEFAULT_RELEASE_NEXT_ROUTE,
        "portal_route": DEFAULT_RELEASE_NEXT_PORTAL_ROUTE,
        "status": payload.get("status", "missing"),
    }
    primary_payload = build_release_next_primary_payload(payload)
    for field in [
        "primary_route",
        "primary_payload_route",
        "primary_kind",
        "primary_name",
        "primary_status",
        "primary_portal_route",
        "primary_api_route",
        "primary_next_route",
        "primary_command",
        "primary_request_file_route",
        "primary_request_file_download_route",
        "primary_request_file_name",
        "primary_follow_up_kind",
        "primary_follow_up_label",
        "primary_follow_up_route",
        "primary_follow_up_payload_route",
        "primary_follow_up_download_route",
        "primary_follow_up_text",
    ]:
        value = payload.get(field)
        if not value:
            value = primary_payload.get(field)
        if value:
            summary[field] = value
    if (
        "primary_follow_up_download_route" in payload
        or "primary_follow_up_download_route" in primary_payload
    ):
        summary["primary_follow_up_download_route"] = payload.get(
            "primary_follow_up_download_route",
            primary_payload.get("primary_follow_up_download_route"),
        )
    request_file_payload = (
        payload.get("request_file_payload")
        if isinstance(payload.get("request_file_payload"), dict)
        else build_release_next_request_file_payload(payload=payload)
    )
    summary["request_file_route"] = DEFAULT_RELEASE_NEXT_REQUEST_FILE_ROUTE
    summary["request_file_payload_route"] = DEFAULT_RELEASE_NEXT_REQUEST_FILE_ROUTE
    summary["request_file_status"] = request_file_payload.get("status", "missing")
    if "request_file_name" in request_file_payload:
        summary["request_file_name"] = request_file_payload.get("request_file_name")
    summary["request_file_download_route"] = request_file_payload.get(
        "request_file_download_route"
    )
    return summary


__all__ = [
    "DEFAULT_RELEASE_CONTROL_PLANE_ROUTE",
    "DEFAULT_RELEASE_CONTROL_PLANE_SURFACE_ROUTE",
    "DEFAULT_RELEASE_CONTROL_PLANE_CATALOG_ROUTE",
    "DEFAULT_RELEASE_CONTROL_PLANE_ACTION_ROUTE",
    "DEFAULT_RELEASE_CONTROL_PLANE_NEXT_ROUTE",
    "DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_TEMPLATES_ROUTE",
    "DEFAULT_RELEASE_CONTROL_PLANE_REQUEST_FILE_ROUTE",
    "DEFAULT_RELEASE_NEXT_ROUTE",
    "DEFAULT_RELEASE_CLOSEOUT_ROUTE",
    "DEFAULT_RELEASE_CLOSEOUT_NEXT_ROUTE",
    "DEFAULT_RELEASE_CLOSEOUT_COMPONENT_ROUTE",
    "DEFAULT_RELEASE_CLOSEOUT_PLAN_ROUTE",
    "DEFAULT_RELEASE_CLOSEOUT_PLAN_STAGE_ROUTE",
    "DEFAULT_RELEASE_CLOSEOUT_PLAN_NEXT_ROUTE",
    "DEFAULT_RELEASE_CLOSEOUT_PLAN_PORTAL_ROUTE",
    "build_release_closeout_payload",
    "build_release_closeout_component_payload",
    "build_release_closeout_next_payload",
    "build_release_closeout_plan_payload",
    "build_release_closeout_plan_stage_payload",
    "build_release_closeout_plan_next_payload",
    "build_release_closeout_summary",
    "build_release_control_plane_surface_payload",
    "build_release_control_plane_request_file_payload",
    "build_release_ops_catalog_payload",
    "build_release_ops_request_templates_payload",
    "build_release_control_plane_action_payload",
    "build_release_control_plane_next_payload",
    "build_release_control_plane_index_payload",
    "build_release_control_plane_index_summary",
    "build_release_next_payload",
    "build_release_next_primary_payload",
    "build_release_next_follow_up_payload",
    "build_release_next_request_file_payload",
    "build_release_next_summary",
    "DEFAULT_RELEASE_NEXT_FOLLOW_UP_ROUTE",
    "DEFAULT_RELEASE_NEXT_REQUEST_FILE_ROUTE",
    "DEFAULT_RELEASE_NEXT_PORTAL_ROUTE",
    "DEFAULT_RELEASE_NEXT_PRIMARY_ROUTE",
]
