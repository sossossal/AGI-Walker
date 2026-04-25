"""
AGI-Walker Web 控制面板
基于 FastAPI 的 Web 服务器

集成了 Web-Godot WebSocket 协议 (v1.0)，用于实时模拟控制和数据交互。
"""

# ruff: noqa: E402

import logging

import sys
from pathlib import Path
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from typing import List, Dict, Any
import asyncio
import uvicorn
import os
from contextlib import asynccontextmanager

logger = logging.getLogger(__name__)
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from web_panel.godot_session_bridge import (
    GodotSessionManager,
    build_router as build_godot_session_router,
    telemetry_loop,
)
from web_panel.godot_legacy_api import (
    build_router as build_godot_legacy_router,
)
from web_panel.tasks_api import (
    build_router as build_tasks_router,
)
from web_panel.distributed_monitor import DistributedMonitor
from web_panel.services_api import (
    build_router as build_services_router,
)
from web_panel.parts_sim2real_api import (
    build_router as build_parts_sim2real_router,
)
from web_panel.agent_api import (
    build_router as build_agent_router,
    get_godot_agent_status,
)
from web_panel.websocket_api import (
    broadcast_all as broadcast_all_service,
    broadcast_session as broadcast_session_service,
    handle_websocket,
)
from web_panel.core_api import (
    build_router as build_core_router,
)
from web_panel.nightly_status import (
    NightlyStatusProvider,
    get_nightly_regression_dashboard,
    get_nightly_regression_status,
)
from web_panel.workflows_api import (
    build_router as build_workflows_router,
)
from web_panel.auth_api import (
    build_router as build_auth_router,
)
from agi_walker.core.api.release_control_plane import (
    build_release_closeout_next_payload,
    build_release_closeout_payload,
    build_release_control_plane_index_payload,
    build_release_control_plane_next_payload,
    build_release_next_payload,
    build_release_control_plane_surface_payload,
)

# Godot integration currently has two transport modes:
# 1. Legacy controller mode: command-oriented connect/load/start/stop/update flow.
# 2. Session bridge mode: launch/process management + TCP telemetry/control for RL-style loops.

from web_panel.logging_config import setup_logging
from prometheus_fastapi_instrumentator import Instrumentator

# Initialize structured logging
setup_logging()

app = FastAPI(title="AGI-Walker Control Panel", version="1.0.0")

# Initialize Prometheus metrics
Instrumentator().instrument(app).expose(app)

# Mount Static Files
app.mount("/static", StaticFiles(directory="web_panel/static"), name="static")
os.makedirs("robots", exist_ok=True)
app.mount("/robots", StaticFiles(directory="robots"), name="robots")
_docs_build_dir = Path("docs/build/html")
if _docs_build_dir.exists():
    app.mount("/docs", StaticFiles(directory=str(_docs_build_dir), html=True), name="docs")

# 存储活跃的 WebSocket 连接（按 session_id 隔离封装）
active_connections: Dict[str, List[WebSocket]] = {}

# 任务状态存储
tasks_db: Dict[str, Dict[str, Any]] = {}


@app.websocket("/ws/{session_id}")
async def websocket_endpoint(websocket: WebSocket, session_id: str):
    """WebSocket 多路隔离连接 - 支持 Web-Godot 协议 v1.0"""
    await handle_websocket(
        websocket=websocket,
        session_id=session_id,
        active_connections=active_connections,
        godot_controller=godot_controller,
        logger=logger,
    )


async def broadcast_all(message: Dict[str, Any]):
    """全局系统广播（针对通用任务）"""
    await broadcast_all_service(active_connections, message)


async def broadcast_session(session_id: str, message: Dict[str, Any]):
    """精准路由广播（针对 Godot 特殊频道）"""
    await broadcast_session_service(active_connections, session_id, message)


from web_panel.godot_controller import godot_controller

app.state.godot_controller = godot_controller


def godot_broadcast_adaptor(*args: Any):
    """将同步回调转换为异步广播"""
    try:
        session_id = None
        if len(args) == 1:
            message = args[0]
        elif len(args) == 2:
            session_id, message = args
        else:
            raise ValueError(f"Unsupported broadcast adaptor arguments: {len(args)}")

        loop = getattr(app.state, "server_loop", None)
        if loop is None:
            return
        if session_id:
            asyncio.run_coroutine_threadsafe(
                broadcast_session(session_id, message),
                loop,
            )
        else:
            asyncio.run_coroutine_threadsafe(broadcast_all(message), loop)
    except Exception as e:
        logger.info(f"广播适配器错误: {e}")


# --- Zenoh Monitor ---
distributed_monitor = DistributedMonitor()
app.state.nightly_status_provider = NightlyStatusProvider.from_env()
app.include_router(
    build_core_router(
        os.path.dirname(__file__),
        tasks_db,
        active_connections,
        distributed_monitor,
        lambda: get_godot_agent_status(app),
        lambda: get_nightly_regression_status(app),
        lambda limit=5: get_nightly_regression_dashboard(app, limit),
        lambda: build_release_control_plane_index_payload(
            project_root=Path(__file__).resolve().parent.parent
        ),
        lambda: build_release_control_plane_surface_payload(
            project_root=Path(__file__).resolve().parent.parent
        ),
        lambda: build_release_control_plane_next_payload(
            project_root=Path(__file__).resolve().parent.parent
        ),
        lambda: build_release_closeout_payload(
            project_root=Path(__file__).resolve().parent.parent
        ),
        lambda: build_release_closeout_next_payload(
            project_root=Path(__file__).resolve().parent.parent
        ),
        lambda: build_release_next_payload(
            project_root=Path(__file__).resolve().parent.parent
        ),
    )
)
app.include_router(build_tasks_router(tasks_db, broadcast_all))
app.include_router(build_services_router())
app.include_router(build_agent_router(app))
app.include_router(build_parts_sim2real_router())
app.include_router(build_auth_router())
app.include_router(build_workflows_router())


# ===========================================================================
# Godot 引擎远程控制桥接层 (Godot Bridge)
# ===========================================================================
_session_manager = GodotSessionManager()
app.state.godot_session_manager = _session_manager
app.include_router(build_godot_legacy_router(godot_controller))
app.include_router(build_godot_session_router(_session_manager, broadcast_session))


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Industrial persistence now managed via Alembic migrations

    app.state.server_loop = asyncio.get_running_loop()
    godot_controller.set_broadcast_callback(godot_broadcast_adaptor)
    app.state.telemetry_task = asyncio.create_task(
        telemetry_loop(_session_manager, broadcast_session)
    )
    distributed_monitor.initialize(godot_broadcast_adaptor)

    try:
        yield
    finally:
        telemetry_task = getattr(app.state, "telemetry_task", None)
        if telemetry_task is not None:
            telemetry_task.cancel()
            try:
                await telemetry_task
            except asyncio.CancelledError:
                pass

        distributed_monitor.close()

        if hasattr(app.state, "server_loop"):
            del app.state.server_loop


app.router.lifespan_context = lifespan


if __name__ == "__main__":
    logger.info("启动 AGI-Walker Web 控制面板")
    logger.info("访问: http://localhost:8000")
    # 确保在 Windows 上循环策略正确 (Python 3.8+)
    if os.name == "nt":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    uvicorn.run(app, host="0.0.0.0", port=8000)
