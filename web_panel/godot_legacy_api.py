import inspect
from typing import Any, Dict, List, Optional

import pydantic
from fastapi import APIRouter
from fastapi import HTTPException
from fastapi import Query


class ConnectionRequest(pydantic.BaseModel):
    host: str = "127.0.0.1"
    port: int = 9999


class LoadRobotRequest(pydantic.BaseModel):
    parts: List[Dict[str, Any]]
    connections: List[Dict[str, Any]]


class StartSimRequest(pydantic.BaseModel):
    physics: Dict[str, Any] = {"gravity": 9.81, "timestep": 0.01}


class UpdateParamsRequest(pydantic.BaseModel):
    params: Dict[str, Any]


def _supports_session_argument(callback: Any) -> bool:
    try:
        signature = inspect.signature(callback)
    except (TypeError, ValueError):
        return False

    for parameter in signature.parameters.values():
        if parameter.kind == inspect.Parameter.VAR_KEYWORD:
            return True
    return "session_id" in signature.parameters


def _call_controller(callback: Any, *args: Any, session_id: Optional[str] = None) -> Any:
    if session_id is not None and _supports_session_argument(callback):
        return callback(*args, session_id=session_id)
    return callback(*args)


def connect_godot(
    godot_controller: Any,
    req: ConnectionRequest,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if _call_controller(godot_controller.connect, req.host, req.port, session_id=session_id):
        return {"status": "connected", "host": req.host, "port": req.port}
    raise HTTPException(status_code=500, detail="Connection refused or timeout")


def disconnect_godot(
    godot_controller: Any,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    _call_controller(godot_controller.disconnect, session_id=session_id)
    return {"status": "disconnected"}


def get_godot_status(godot_controller: Any, session_id: Optional[str] = None) -> Dict[str, Any]:
    client = _call_controller(godot_controller.get_client, session_id=session_id)
    return {
        "connected": _call_controller(godot_controller.is_connected, session_id=session_id),
        "client_running": client.running,
    }


def load_robot(
    godot_controller: Any,
    req: LoadRobotRequest,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if not _call_controller(godot_controller.is_connected, session_id=session_id):
        raise HTTPException(status_code=400, detail="Godot not connected")

    if _call_controller(
        godot_controller.load_robot,
        req.parts,
        req.connections,
        session_id=session_id,
    ):
        return {"status": "success", "message": "Robot config sent"}
    raise HTTPException(status_code=500, detail="Failed to send command")


def start_simulation(
    godot_controller: Any,
    req: StartSimRequest,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if not _call_controller(godot_controller.is_connected, session_id=session_id):
        raise HTTPException(status_code=400, detail="Godot not connected")

    if _call_controller(godot_controller.start_simulation, req.physics, session_id=session_id):
        return {"status": "started"}
    raise HTTPException(status_code=500, detail="Failed to start simulation")


def stop_simulation(
    godot_controller: Any,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if not _call_controller(godot_controller.is_connected, session_id=session_id):
        raise HTTPException(status_code=400, detail="Godot not connected")

    if _call_controller(godot_controller.stop_simulation, session_id=session_id):
        return {"status": "stopped"}
    raise HTTPException(status_code=500, detail="Failed to stop simulation")


def update_params(
    godot_controller: Any,
    req: UpdateParamsRequest,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if not _call_controller(godot_controller.is_connected, session_id=session_id):
        raise HTTPException(status_code=400, detail="Godot not connected")

    if _call_controller(godot_controller.update_params, req.params, session_id=session_id):
        return {"status": "updated", "params": req.params}
    raise HTTPException(status_code=500, detail="Failed to update parameters")


def build_router(godot_controller: Any) -> APIRouter:
    router = APIRouter()

    @router.post("/api/godot/connect")
    async def godot_connect(
        req: ConnectionRequest,
        session_id: Optional[str] = Query(default=None),
    ):
        """连接到 Godot"""
        return connect_godot(godot_controller, req, session_id=session_id)

    @router.post("/api/godot/disconnect")
    async def godot_disconnect(session_id: Optional[str] = Query(default=None)):
        """断开 Godot 连接"""
        return disconnect_godot(godot_controller, session_id=session_id)

    @router.get("/api/godot/status")
    async def godot_status(session_id: Optional[str] = Query(default=None)):
        """获取连接状态"""
        return get_godot_status(godot_controller, session_id=session_id)

    @router.post("/api/godot/load-robot")
    async def godot_load_robot(
        req: LoadRobotRequest,
        session_id: Optional[str] = Query(default=None),
    ):
        """加载机器人配置到 Godot"""
        return load_robot(godot_controller, req, session_id=session_id)

    @router.post("/api/godot/start")
    async def godot_start(
        req: StartSimRequest,
        session_id: Optional[str] = Query(default=None),
    ):
        """启动仿真"""
        return start_simulation(godot_controller, req, session_id=session_id)

    @router.post("/api/godot/stop")
    async def godot_stop(session_id: Optional[str] = Query(default=None)):
        """停止仿真"""
        return stop_simulation(godot_controller, session_id=session_id)

    @router.post("/api/godot/update-params")
    async def godot_update_params(
        req: UpdateParamsRequest,
        session_id: Optional[str] = Query(default=None),
    ):
        """实时更新参数"""
        return update_params(godot_controller, req, session_id=session_id)

    return router
