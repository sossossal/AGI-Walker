import os
from datetime import datetime
from typing import Any, Callable, Dict, List

from fastapi import APIRouter
from fastapi import WebSocket
from fastapi.responses import FileResponse, Response
from agi_walker.core.api.capability_matrix import (
    build_capability_matrix_artifact,
    build_capability_matrix_summary,
)
from agi_walker.core.api.release_control_plane import (
    build_release_closeout_component_payload,
    build_release_closeout_plan_next_payload,
    build_release_closeout_plan_payload,
    build_release_closeout_plan_stage_payload,
    build_release_closeout_next_payload,
    build_release_closeout_payload,
    build_release_control_plane_action_payload,
    build_release_control_plane_next_payload,
    build_release_control_plane_request_file_payload,
    build_release_next_summary,
    build_release_next_payload,
    build_release_next_follow_up_payload,
    build_release_next_primary_payload,
    build_release_next_request_file_payload,
    build_release_closeout_summary,
    build_release_control_plane_index_summary,
    build_release_control_plane_surface_payload,
    build_release_ops_catalog_payload,
    build_release_ops_request_templates_payload,
)
from agi_walker.core.api.workflow_contracts import WORKFLOW_CONTRACT_VERSION
from agi_walker.skills_loader import get_skills_loader

DEFAULT_GODOT_SESSION_ID = "default"


def root_response(server_dir: str) -> FileResponse:
    html_path = os.path.join(server_dir, "static", "index.html")
    return FileResponse(html_path, media_type="text/html")


def system_status(
    tasks_db: Dict[str, Dict[str, Any]],
    active_connections: Dict[str, List[WebSocket]],
    distributed_monitor=None,
    godot_agent_status_provider: Callable[[], Dict[str, Any]] | None = None,
    nightly_status_provider: Callable[[], Dict[str, Any]] | None = None,
    release_control_plane_provider: Callable[[], Dict[str, Any]] | None = None,
    release_next_provider: Callable[[], Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    status = {
        "status": "running",
        "tasks_count": len(tasks_db),
        "active_connections": len(active_connections),
        "timestamp": datetime.now().isoformat(),
    }
    if distributed_monitor is not None:
        status["distributed_monitor"] = distributed_monitor.capabilities()
    if godot_agent_status_provider is not None:
        status["godot_agent"] = godot_agent_status_provider()
    if nightly_status_provider is not None:
        status["nightly_regressions"] = nightly_status_provider()
    if release_control_plane_provider is not None:
        release_control_plane_payload = release_control_plane_provider()
        status["release_control_plane"] = build_release_control_plane_index_summary(
            release_control_plane_payload
        )
        release_closeout_payload = (
            {"release_closeout": release_control_plane_payload.get("release_closeout")}
            if isinstance(release_control_plane_payload.get("release_closeout"), dict)
            else None
        )
        status["release_closeout"] = build_release_closeout_summary(
            release_closeout_payload
        )
    if release_next_provider is not None:
        status["release_next"] = build_release_next_summary(
            release_next_provider()
        )
    status["capability_matrix"] = build_capability_matrix_summary()
    return status


def godot_capabilities() -> Dict[str, Any]:
    return {
        "default_session_id": DEFAULT_GODOT_SESSION_ID,
        "preferred_mode": "session_bridge",
        "modes": {
            "legacy_controller": {
                "description": (
                    "Compatibility transport for direct connect/load/start/stop/update flows. "
                    "Commands may be scoped to a websocket session via the session_id query parameter."
                ),
                "routes": [
                    "/api/godot/connect",
                    "/api/godot/disconnect",
                    "/api/godot/status",
                    "/api/godot/load-robot",
                    "/api/godot/start",
                    "/api/godot/stop",
                    "/api/godot/update-params",
                ],
                "websocket_protocol": [
                    "simulation.start",
                    "simulation.stop",
                    "config.load_robot",
                    "params.update",
                    "ping",
                ],
                "push_messages": [
                    "telemetry.update",
                    "simulation.status",
                    "simulation.error",
                    "connection.status",
                ],
                "session_query_param": "session_id",
                "status": "compatibility_only",
            },
            "session_bridge": {
                "description": "Preferred official transport: session-isolated Godot process + TCP bridge for telemetry/control loops.",
                "routes": [
                    "/api/godot/{session_id}/launch",
                    "/api/godot/{session_id}/stop",
                    "/api/godot/{session_id}/status",
                    "/api/godot/{session_id}/control",
                    "/api/godot/{session_id}/instruction-set",
                    "/api/godot/{session_id}/simulated-circuit",
                    "/ws/{session_id}",
                ],
                "tcp_commands": [
                    "reset",
                    "step",
                    "get_schema",
                    "load_robot",
                    "instruction_set",
                    "configure_simulated_circuit",
                ],
                "status": "preferred",
                "status_schema_version": "1.0",
                "session_states": [
                    "disconnected",
                    "launching",
                    "connected",
                    "schema_ready",
                    "running",
                    "failed",
                ],
            },
            "workflow_bridge": {
                "description": "Official workflow-to-Godot bridge. Recommended to target session_bridge unless legacy compatibility is required.",
                "routes": [
                    "/api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load",
                    "/api/workflows/runs/{run_id}/godot-sync",
                ],
                "transport_modes": [
                    "session_bridge",
                    "legacy_controller",
                ],
                "preferred_transport_mode": "session_bridge",
                "artifact_contract_version": WORKFLOW_CONTRACT_VERSION,
            },
        },
        "note": (
            "Legacy controller remains available for compatibility, but session_bridge is now the preferred path. "
            "Workflow artifacts can be forwarded into either transport via the workflow bridge route, "
            "and /api/workflows/runs/{run_id}/godot-sync provides the recommended auto-selection flow. "
            "Frontend clients should treat canonical websocket pushes as telemetry.update, "
            "simulation.status, simulation.error, and connection.status."
        ),
    }


def distributed_status(distributed_monitor) -> Dict[str, Any]:
    actors = distributed_monitor.snapshot()
    return {
        "schema_version": "1.0",
        "actors": actors,
        "actor_ids": sorted(actors),
        "actors_count": len(actors),
        "monitor": distributed_monitor.capabilities(),
    }


def capability_matrix() -> Dict[str, Any]:
    return build_capability_matrix_artifact()


def release_control_plane_index(
    release_control_plane_provider: Callable[[], Dict[str, Any]] | None = None,
) -> Dict[str, Any]:
    if release_control_plane_provider is None:
        return {
            "status": "not_configured",
            "message": "Release control-plane provider unavailable.",
        }
    return release_control_plane_provider()


def release_control_plane_surface(
    *,
    project_root: str | os.PathLike[str] | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_control_plane_surface_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
    )


def release_control_plane_catalog() -> Dict[str, Any]:
    return build_release_ops_catalog_payload()


def release_control_plane_request_templates(
    action: str | None = None,
) -> Dict[str, Any]:
    try:
        return build_release_ops_request_templates_payload(action=action)
    except ValueError as exc:
        return {
            "status": "error",
            "message": str(exc),
            "action": action,
        }


def release_control_plane_action(action: str) -> Dict[str, Any]:
    try:
        return build_release_control_plane_action_payload(action=action)
    except ValueError as exc:
        return {
            "status": "error",
            "message": str(exc),
            "action": action,
        }


def release_control_plane_next(
    *,
    project_root: str | os.PathLike[str] | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_control_plane_next_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
    )


def release_control_plane_request_file(action: str) -> Dict[str, Any]:
    try:
        return build_release_control_plane_request_file_payload(action=action)
    except ValueError as exc:
        return {
            "status": "error",
            "message": str(exc),
            "action": action,
        }


def release_closeout(
    *,
    project_root: str | os.PathLike[str] | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_closeout_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def release_closeout_component(
    component: str,
    *,
    project_root: str | os.PathLike[str] | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    try:
        return build_release_closeout_component_payload(
            component=component,
            project_root=project_root,
            external_mainline_execution_plan_path=external_mainline_execution_plan_path,
            security_release_preflight_report_path=security_release_preflight_report_path,
            vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
            release_readiness_report_path=release_readiness_report_path,
            worktree_release_blocker_report_path=worktree_release_blocker_report_path,
        )
    except ValueError as exc:
        return {
            "status": "error",
            "message": str(exc),
            "component": component,
        }


def release_closeout_next(
    *,
    project_root: str | os.PathLike[str] | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_closeout_next_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def release_closeout_plan(
    *,
    project_root: str | os.PathLike[str] | None = None,
    external_mainline_execution_plan_path: str | None = None,
    external_mainline_inputs_path: str | None = None,
    external_mainline_input_checklist_report_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_closeout_plan_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        external_mainline_inputs_path=external_mainline_inputs_path,
        external_mainline_input_checklist_report_path=external_mainline_input_checklist_report_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def release_closeout_plan_stage(
    stage: str,
    *,
    project_root: str | os.PathLike[str] | None = None,
    external_mainline_execution_plan_path: str | None = None,
    external_mainline_inputs_path: str | None = None,
    external_mainline_input_checklist_report_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    try:
        return build_release_closeout_plan_stage_payload(
            stage=stage,
            project_root=project_root,
            external_mainline_execution_plan_path=external_mainline_execution_plan_path,
            external_mainline_inputs_path=external_mainline_inputs_path,
            external_mainline_input_checklist_report_path=external_mainline_input_checklist_report_path,
            security_release_preflight_report_path=security_release_preflight_report_path,
            vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
            release_readiness_report_path=release_readiness_report_path,
            worktree_release_blocker_report_path=worktree_release_blocker_report_path,
        )
    except ValueError as exc:
        return {
            "status": "error",
            "message": str(exc),
            "stage": stage,
        }


def release_closeout_plan_next(
    *,
    project_root: str | os.PathLike[str] | None = None,
    external_mainline_execution_plan_path: str | None = None,
    external_mainline_inputs_path: str | None = None,
    external_mainline_input_checklist_report_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_closeout_plan_next_payload(
        project_root=project_root,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        external_mainline_inputs_path=external_mainline_inputs_path,
        external_mainline_input_checklist_report_path=external_mainline_input_checklist_report_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def release_next(
    *,
    project_root: str | os.PathLike[str] | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_next_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def release_next_primary(
    project_root: str | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_next_primary_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def release_next_follow_up(
    project_root: str | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_next_follow_up_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def release_next_request_file(
    project_root: str | None = None,
    manifest_path: str | None = None,
    release_ops_execution_report_path: str | None = None,
    external_mainline_execution_plan_path: str | None = None,
    security_release_preflight_report_path: str | None = None,
    vulnerability_exception_review_report_path: str | None = None,
    release_readiness_report_path: str | None = None,
    worktree_release_blocker_report_path: str | None = None,
) -> Dict[str, Any]:
    return build_release_next_request_file_payload(
        project_root=project_root,
        manifest_path=manifest_path,
        release_ops_execution_report_path=release_ops_execution_report_path,
        external_mainline_execution_plan_path=external_mainline_execution_plan_path,
        security_release_preflight_report_path=security_release_preflight_report_path,
        vulnerability_exception_review_report_path=vulnerability_exception_review_report_path,
        release_readiness_report_path=release_readiness_report_path,
        worktree_release_blocker_report_path=worktree_release_blocker_report_path,
    )


def build_router(
    server_dir: str,
    tasks_db: Dict[str, Dict[str, Any]],
    active_connections: Dict[str, List[WebSocket]],
    distributed_monitor,
    godot_agent_status_provider: Callable[[], Dict[str, Any]] | None = None,
    nightly_status_provider: Callable[[], Dict[str, Any]] | None = None,
    nightly_dashboard_provider: Callable[[int], Dict[str, Any]] | None = None,
    release_control_plane_provider: Callable[[], Dict[str, Any]] | None = None,
    release_control_plane_surface_provider: Callable[[], Dict[str, Any]] | None = None,
    release_control_plane_next_provider: Callable[[], Dict[str, Any]] | None = None,
    release_closeout_provider: Callable[[], Dict[str, Any]] | None = None,
    release_closeout_next_provider: Callable[[], Dict[str, Any]] | None = None,
    release_next_provider: Callable[[], Dict[str, Any]] | None = None,
) -> APIRouter:
    router = APIRouter()

    @router.get("/")
    async def root():
        """主页（使用 FileResponse 避免编码风险）"""
        return root_response(server_dir)

    @router.get("/api/system/status")
    async def get_system_status():
        """获取系统状态"""
        return system_status(
            tasks_db,
            active_connections,
            distributed_monitor,
            godot_agent_status_provider,
            nightly_status_provider,
            release_control_plane_provider,
            release_next_provider,
        )

    @router.get("/api/godot/capabilities")
    async def get_godot_capabilities():
        """Describe the currently supported Godot integration modes."""
        return godot_capabilities()

    @router.get("/api/nightly/regressions")
    async def get_nightly_regressions(limit: int = 5):
        """Get recent nightly specialized regression runs for the ops view."""
        if nightly_dashboard_provider is None:
            return {
                "status": "not_configured",
                "message": "Nightly regression provider unavailable.",
                "recent_runs": [],
                "job_catalog": {},
            }
        return nightly_dashboard_provider(limit)

    @router.get("/api/distributed/status")
    async def get_distributed_status():
        """Get snapshot of distributed actors"""
        return distributed_status(distributed_monitor)

    @router.get("/api/capabilities/matrix")
    async def get_capability_matrix():
        """Get the release-facing capability matrix."""
        return capability_matrix()

    @router.get("/api/release/control-plane")
    async def get_release_control_plane():
        """Get the read-only release/control-plane overview for Portal clients."""
        return release_control_plane_index(release_control_plane_provider)

    @router.get("/api/release/control-plane/surface")
    async def get_release_control_plane_surface():
        """Get the canonical release/control-plane surface for Portal clients."""
        if release_control_plane_surface_provider is not None:
            return release_control_plane_surface_provider()
        return release_control_plane_surface()

    @router.get("/api/release/control-plane/catalog")
    async def get_release_control_plane_catalog():
        """Get the read-only release/control-plane catalog for Portal clients."""
        return release_control_plane_catalog()

    @router.get("/api/release/control-plane/request-templates")
    async def get_release_control_plane_request_templates(action: str | None = None):
        """Get read-only release/control-plane request templates for Portal clients."""
        return release_control_plane_request_templates(action=action)

    @router.get("/api/release/control-plane/action")
    async def get_release_control_plane_action(action: str):
        """Get one release/control-plane action summary for Portal clients."""
        return release_control_plane_action(action=action)

    @router.get("/api/release/control-plane/next")
    async def get_release_control_plane_next():
        """Get the recommended next release/control-plane action summary."""
        if release_control_plane_next_provider is not None:
            return release_control_plane_next_provider()
        return release_control_plane_next()

    @router.get("/api/release/control-plane/request-file")
    async def get_release_control_plane_request_file(
        action: str,
        download: bool = False,
    ):
        """Get one read-only release/control-plane request-file scaffold."""
        payload = release_control_plane_request_file(action=action)
        if download and payload.get("status") == "success":
            filename = str(payload.get("request_file_name") or "release_ops.request.json")
            return Response(
                content=str(payload.get("request_file_pretty_json") or ""),
                media_type=str(payload.get("content_type") or "application/json"),
                headers={
                    "Content-Disposition": f'attachment; filename="{filename}"',
                },
            )
        return payload

    @router.get("/api/release/closeout")
    async def get_release_closeout():
        """Get the unified release closeout surface for Portal clients."""
        if release_closeout_provider is not None:
            return release_closeout_provider()
        return release_closeout()

    @router.get("/api/release/closeout/next")
    async def get_release_closeout_next():
        """Get the recommended next release closeout component summary."""
        if release_closeout_next_provider is not None:
            return release_closeout_next_provider()
        return release_closeout_next()

    @router.get("/api/release/closeout/plan")
    async def get_release_closeout_plan():
        """Get the staged execution plan for the remaining external closeout inputs."""
        return release_closeout_plan()

    @router.get("/api/release/closeout/plan/stage")
    async def get_release_closeout_plan_stage(stage: str):
        """Get one closeout-plan stage payload for Portal clients."""
        return release_closeout_plan_stage(stage)

    @router.get("/api/release/closeout/plan/next")
    async def get_release_closeout_plan_next():
        """Get the recommended next closeout-plan stage payload."""
        return release_closeout_plan_next()

    @router.get("/api/release/closeout/component")
    async def get_release_closeout_component(component: str):
        """Get one release closeout component summary for Portal clients."""
        return release_closeout_component(component)

    @router.get("/api/release/next")
    async def get_release_next():
        """Get the unified recommended next release step."""
        if release_next_provider is not None:
            return release_next_provider()
        return release_next()

    @router.get("/api/release/next/primary")
    async def get_release_next_primary():
        """Get the primary recommended next release entry."""
        if release_next_provider is not None:
            return build_release_next_primary_payload(release_next_provider())
        return release_next_primary()

    @router.get("/api/release/next/follow-up")
    async def get_release_next_follow_up():
        """Get the normalized follow-up entry for the current primary next release item."""
        if release_next_provider is not None:
            return build_release_next_follow_up_payload(release_next_provider())
        return release_next_follow_up()

    @router.get("/api/release/next/request-file")
    async def get_release_next_request_file(download: int = 0):
        """Get the unified request-file scaffold for the current primary next release item."""
        payload = (
            build_release_next_request_file_payload(release_next_provider())
            if release_next_provider is not None
            else release_next_request_file()
        )
        if download and payload.get("status") == "success":
            from fastapi.responses import Response

            return Response(
                content=str(payload.get("request_file_pretty_json") or ""),
                media_type=str(payload.get("content_type") or "application/json"),
                headers={
                    "Content-Disposition": f"attachment; filename={payload.get('request_file_name') or 'release-next.request.json'}"
                },
            )
        return payload

    @router.get("/api/skills/catalog")
    async def get_skill_catalog():
        """获取所有可用 Skill 的正式契约定义 (V2.5 No-code 支持)"""
        loader = get_skills_loader()
        return loader.get_catalog()

    return router
