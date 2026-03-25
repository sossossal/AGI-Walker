import os
from datetime import datetime
from typing import Any, Dict, List

from fastapi import APIRouter
from fastapi import WebSocket
from fastapi.responses import FileResponse


DEFAULT_GODOT_SESSION_ID = "default"


def root_response(server_dir: str) -> FileResponse:
    html_path = os.path.join(server_dir, "static", "index.html")
    return FileResponse(html_path, media_type="text/html")


def system_status(
    tasks_db: Dict[str, Dict[str, Any]],
    active_connections: Dict[str, List[WebSocket]],
) -> Dict[str, Any]:
    return {
        "status": "running",
        "tasks_count": len(tasks_db),
        "active_connections": len(active_connections),
        "timestamp": datetime.now().isoformat(),
    }


def godot_capabilities() -> Dict[str, Any]:
    return {
        "default_session_id": DEFAULT_GODOT_SESSION_ID,
        "modes": {
            "legacy_controller": {
                "description": "Direct controller client for connect/load/start/stop/update flows.",
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
            },
            "session_bridge": {
                "description": "Session-isolated Godot process + TCP bridge for telemetry/control loops.",
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
                ],
            },
        },
        "note": (
            "The session bridge does not yet replace the legacy controller flow. "
            "The two modes currently serve different transport semantics."
        ),
    }


def distributed_status(distributed_monitor) -> Dict[str, Any]:
    return {"actors": distributed_monitor.snapshot()}


def build_router(
    server_dir: str,
    tasks_db: Dict[str, Dict[str, Any]],
    active_connections: Dict[str, List[WebSocket]],
    distributed_monitor,
) -> APIRouter:
    router = APIRouter()

    @router.get("/")
    async def root():
        """主页（使用 FileResponse 避免编码风险）"""
        return root_response(server_dir)

    @router.get("/api/system/status")
    async def get_system_status():
        """获取系统状态"""
        return system_status(tasks_db, active_connections)

    @router.get("/api/godot/capabilities")
    async def get_godot_capabilities():
        """Describe the currently supported Godot integration modes."""
        return godot_capabilities()

    @router.get("/api/distributed/status")
    async def get_distributed_status():
        """Get snapshot of distributed actors"""
        return distributed_status(distributed_monitor)

    return router
