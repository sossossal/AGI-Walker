import inspect
from typing import Any, Dict, List, Optional

import pydantic
from fastapi import APIRouter
from fastapi import HTTPException
from fastapi import Query

from agi_walker.core.api.comm.instruction_control_contracts import (
    build_instruction_runtime_contract,
    normalize_simulated_circuit_config,
)


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


class SimulatedCircuitConfigRequest(pydantic.BaseModel):
    simulated_circuit: Dict[str, Any] = pydantic.Field(default_factory=dict)


class InstructionSetRequest(pydantic.BaseModel):
    instruction_set: Dict[str, Any]


def _supports_session_argument(callback: Any) -> bool:
    try:
        signature = inspect.signature(callback)
    except (TypeError, ValueError):
        return False

    for parameter in signature.parameters.values():
        if parameter.kind == inspect.Parameter.VAR_KEYWORD:
            return True
    return "session_id" in signature.parameters


def _call_controller(
    callback: Any, *args: Any, session_id: Optional[str] = None
) -> Any:
    if session_id is not None and _supports_session_argument(callback):
        return callback(*args, session_id=session_id)
    return callback(*args)


def connect_godot(
    godot_controller: Any,
    req: ConnectionRequest,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if _call_controller(
        godot_controller.connect, req.host, req.port, session_id=session_id
    ):
        return {"status": "connected", "host": req.host, "port": req.port}
    raise HTTPException(status_code=500, detail="Connection refused or timeout")


def disconnect_godot(
    godot_controller: Any,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    _call_controller(godot_controller.disconnect, session_id=session_id)
    return {"status": "disconnected"}


def get_godot_status(
    godot_controller: Any, session_id: Optional[str] = None
) -> Dict[str, Any]:
    client = _call_controller(godot_controller.get_client, session_id=session_id)
    return {
        "connected": _call_controller(
            godot_controller.is_connected, session_id=session_id
        ),
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

    if _call_controller(
        godot_controller.start_simulation, req.physics, session_id=session_id
    ):
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

    if _call_controller(
        godot_controller.update_params, req.params, session_id=session_id
    ):
        return {"status": "updated", "params": req.params}
    raise HTTPException(status_code=500, detail="Failed to update parameters")


def configure_simulated_circuit(
    godot_controller: Any,
    req: SimulatedCircuitConfigRequest,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if not _call_controller(godot_controller.is_connected, session_id=session_id):
        raise HTTPException(status_code=400, detail="Godot not connected")

    runtime_config = normalize_simulated_circuit_config(req.simulated_circuit)
    result = _call_controller(
        godot_controller.configure_simulated_circuit,
        runtime_config,
        session_id=session_id,
    )
    if result:
        return {
            "status": "configured",
            "simulated_circuit": runtime_config,
            "dispatch_result": result,
        }
    raise HTTPException(status_code=500, detail="Failed to configure simulated circuit")


def apply_instruction_set(
    godot_controller: Any,
    req: InstructionSetRequest,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    if not _call_controller(godot_controller.is_connected, session_id=session_id):
        raise HTTPException(status_code=400, detail="Godot not connected")

    runtime_contract = build_instruction_runtime_contract(req.instruction_set)
    instruction_payload = runtime_contract["instruction_set"]
    dispatch_result = _call_controller(
        godot_controller.send_instruction_set,
        instruction_payload,
        session_id=session_id,
    )
    if not dispatch_result:
        raise HTTPException(status_code=500, detail="Failed to dispatch instruction set")

    compatibility_params = runtime_contract["compatibility_params"]
    compatibility_dispatched = False
    if compatibility_params:
        compatibility_dispatched = bool(
            _call_controller(
                godot_controller.update_params,
                compatibility_params,
                session_id=session_id,
            )
        )

    return {
        "status": "applied",
        "instruction_set": instruction_payload,
        "simulated_circuit": runtime_contract["simulated_circuit"],
        "simulated_circuit_command_batch": runtime_contract[
            "simulated_circuit_command_batch"
        ],
        "compatibility_params": compatibility_params,
        "dispatch_result": dispatch_result,
        "compatibility_dispatched": compatibility_dispatched,
    }


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

    @router.post("/api/godot/simulated-circuit")
    async def godot_configure_simulated_circuit(
        req: SimulatedCircuitConfigRequest,
        session_id: Optional[str] = Query(default=None),
    ):
        """配置 canonical 模拟电路通信参数"""
        return configure_simulated_circuit(
            godot_controller, req, session_id=session_id
        )

    @router.post("/api/godot/instruction-set")
    async def godot_apply_instruction_set(
        req: InstructionSetRequest,
        session_id: Optional[str] = Query(default=None),
    ):
        """发送结构化指令集控制 payload"""
        return apply_instruction_set(godot_controller, req, session_id=session_id)

    return router
