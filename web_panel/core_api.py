import os
from datetime import datetime
from typing import Any, Callable, Dict, List

from fastapi import APIRouter
from fastapi import WebSocket
from fastapi.responses import FileResponse
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
                    "/ws/{session_id}",
                ],
                "tcp_commands": [
                    "reset",
                    "step",
                    "get_schema",
                    "load_robot",
                ],
                "status": "preferred",
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
    return {
        "actors": distributed_monitor.snapshot(),
        "monitor": distributed_monitor.capabilities(),
    }


def build_router(
    server_dir: str,
    tasks_db: Dict[str, Dict[str, Any]],
    active_connections: Dict[str, List[WebSocket]],
    distributed_monitor,
    godot_agent_status_provider: Callable[[], Dict[str, Any]] | None = None,
    nightly_status_provider: Callable[[], Dict[str, Any]] | None = None,
    nightly_dashboard_provider: Callable[[int], Dict[str, Any]] | None = None,
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

    @router.get("/api/skills/catalog")
    async def get_skill_catalog():
        """获取所有可用 Skill 的正式契约定义 (V2.5 No-code 支持)"""
        loader = get_skills_loader()
        return loader.get_catalog()

    return router
