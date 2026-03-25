"""
AGI-Walker Web 控制面板
基于 FastAPI 的 Web 服务器

集成了 Web-Godot WebSocket 协议 (v1.0)，用于实时模拟控制和数据交互。
"""
import logging
logger = logging.getLogger(__name__)

import sys
from pathlib import Path
from fastapi import FastAPI, WebSocket
from fastapi.staticfiles import StaticFiles
from typing import List, Dict, Any
import asyncio
import uvicorn
import os
from contextlib import asynccontextmanager

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
)
from web_panel.websocket_api import (
    broadcast_all as broadcast_all_service,
    broadcast_session as broadcast_session_service,
    handle_websocket,
)
from web_panel.core_api import (
    build_router as build_core_router,
)

# Godot integration currently has two transport modes:
# 1. Legacy controller mode: command-oriented connect/load/start/stop/update flow.
# 2. Session bridge mode: launch/process management + TCP telemetry/control for RL-style loops.

app = FastAPI(title="AGI-Walker Control Panel", version="1.0.0")

# Mount Static Files
app.mount("/static", StaticFiles(directory="web_panel/static"), name="static")
os.makedirs("robots", exist_ok=True)
app.mount("/robots", StaticFiles(directory="robots"), name="robots")
app.mount("/docs", StaticFiles(directory="docs/build/html", html=True), name="docs")

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

def godot_broadcast_adaptor(message: Dict[str, Any]):
    """将同步回调转换为异步广播"""
    try:
        loop = getattr(app.state, "server_loop", None)
        if loop is None:
            return
        asyncio.run_coroutine_threadsafe(broadcast_all(message), loop)
    except Exception as e:
        logger.info(f"广播适配器错误: {e}")

# --- Zenoh Monitor ---
distributed_monitor = DistributedMonitor()
app.include_router(
    build_core_router(
        os.path.dirname(__file__),
        tasks_db,
        active_connections,
        distributed_monitor,
    )
)
app.include_router(build_tasks_router(tasks_db, broadcast_all))
app.include_router(build_services_router())
app.include_router(build_agent_router(app))
app.include_router(build_parts_sim2real_router())


# ===========================================================================
# Godot 引擎远程控制桥接层 (Godot Bridge)
# ===========================================================================
_session_manager = GodotSessionManager()
app.include_router(build_godot_legacy_router(godot_controller))
app.include_router(build_godot_session_router(_session_manager, broadcast_session))


@asynccontextmanager
async def lifespan(app: FastAPI):
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
    if os.name == 'nt':
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
    uvicorn.run(app, host="0.0.0.0", port=8000)
