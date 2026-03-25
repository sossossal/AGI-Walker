from typing import Any, Dict, List

import pydantic
from fastapi import APIRouter
from fastapi import HTTPException


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


def connect_godot(godot_controller: Any, req: ConnectionRequest) -> Dict[str, Any]:
    if godot_controller.connect(req.host, req.port):
        return {"status": "connected", "host": req.host, "port": req.port}
    raise HTTPException(status_code=500, detail="Connection refused or timeout")


def disconnect_godot(godot_controller: Any) -> Dict[str, Any]:
    godot_controller.disconnect()
    return {"status": "disconnected"}


def get_godot_status(godot_controller: Any) -> Dict[str, Any]:
    return {
        "connected": godot_controller.is_connected(),
        "client_running": godot_controller.client.running,
    }


def load_robot(godot_controller: Any, req: LoadRobotRequest) -> Dict[str, Any]:
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")

    if godot_controller.load_robot(req.parts, req.connections):
        return {"status": "success", "message": "Robot config sent"}
    raise HTTPException(status_code=500, detail="Failed to send command")


def start_simulation(godot_controller: Any, req: StartSimRequest) -> Dict[str, Any]:
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")

    if godot_controller.start_simulation(req.physics):
        return {"status": "started"}
    raise HTTPException(status_code=500, detail="Failed to start simulation")


def stop_simulation(godot_controller: Any) -> Dict[str, Any]:
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")

    if godot_controller.stop_simulation():
        return {"status": "stopped"}
    raise HTTPException(status_code=500, detail="Failed to stop simulation")


def update_params(godot_controller: Any, req: UpdateParamsRequest) -> Dict[str, Any]:
    if not godot_controller.is_connected():
        raise HTTPException(status_code=400, detail="Godot not connected")

    if godot_controller.update_params(req.params):
        return {"status": "updated", "params": req.params}
    raise HTTPException(status_code=500, detail="Failed to update parameters")


def build_router(godot_controller: Any) -> APIRouter:
    router = APIRouter()

    @router.post("/api/godot/connect")
    async def godot_connect(req: ConnectionRequest):
        """连接到 Godot"""
        return connect_godot(godot_controller, req)

    @router.post("/api/godot/disconnect")
    async def godot_disconnect():
        """断开 Godot 连接"""
        return disconnect_godot(godot_controller)

    @router.get("/api/godot/status")
    async def godot_status():
        """获取连接状态"""
        return get_godot_status(godot_controller)

    @router.post("/api/godot/load-robot")
    async def godot_load_robot(req: LoadRobotRequest):
        """加载机器人配置到 Godot"""
        return load_robot(godot_controller, req)

    @router.post("/api/godot/start")
    async def godot_start(req: StartSimRequest):
        """启动仿真"""
        return start_simulation(godot_controller, req)

    @router.post("/api/godot/stop")
    async def godot_stop():
        """停止仿真"""
        return stop_simulation(godot_controller)

    @router.post("/api/godot/update-params")
    async def godot_update_params(req: UpdateParamsRequest):
        """实时更新参数"""
        return update_params(godot_controller, req)

    return router
