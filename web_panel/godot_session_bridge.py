import asyncio
import json
import logging
import os
import socket
import struct
import subprocess
import threading
from typing import Any, Awaitable, Callable, Dict, List, Optional

import pydantic
from fastapi import APIRouter

logger = logging.getLogger(__name__)

GODOT_PROJECT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "godot_project",
)


class GodotBridge:
    """Manage a Godot process and its TCP control channel for one session."""

    def __init__(self, session_id: str, port: int):
        self.session_id = session_id
        self.process: Optional[subprocess.Popen] = None
        self.reader: Optional[asyncio.StreamReader] = None
        self.writer: Optional[asyncio.StreamWriter] = None
        self.last_sensor: Dict[str, Any] = {}
        self.schema: Dict[str, Any] = {}
        self.schema_fetched = False
        self.tcp_lock: Optional[asyncio.Lock] = None
        self._tcp_host = "127.0.0.1"
        self._tcp_port = port

    def launch(self, scene: str = "demo_generated_biped.tscn", godot_exe: str = "") -> Dict[str, Any]:
        if self.process and self.process.poll() is None:
            return {"status": "already_running", "pid": self.process.pid}

        exe = godot_exe or self._find_godot_exe()
        if not exe:
            return {"status": "error", "message": "未找到 Godot 可执行文件，请在请求中传入 godot_exe 路径"}

        scene_path = os.path.join(GODOT_PROJECT_DIR, scene)
        if not os.path.exists(scene_path):
            return {"status": "error", "message": f"场景文件不存在: {scene_path}"}

        cmd = [exe, "--path", GODOT_PROJECT_DIR, scene_path, f"--tcp-port={self._tcp_port}"]
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                creationflags=subprocess.CREATE_NEW_CONSOLE if os.name == "nt" else 0,
            )
            asyncio.create_task(self._delayed_tcp_connect())
            return {"status": "launched", "pid": self.process.pid, "scene": scene, "exe": exe}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _find_godot_exe(self) -> str:
        candidates = [
            "godot",
            r"C:\Program Files\Godot\Godot_v4.2.2-stable_win64.exe",
            r"C:\Godot\Godot_v4.2.2-stable_win64.exe",
            r"D:\Godot\Godot_v4.2.2-stable_win64.exe",
        ]
        for candidate in candidates:
            if os.path.isfile(candidate):
                return candidate
            try:
                subprocess.run([candidate, "--version"], capture_output=True, timeout=2)
                return candidate
            except Exception:
                continue
        return ""

    async def _delayed_tcp_connect(self, delay: float = 3.0) -> None:
        await asyncio.sleep(delay)
        await self._connect_tcp()

    async def _connect_tcp(self) -> bool:
        if self.tcp_lock is None:
            self.tcp_lock = asyncio.Lock()
        async with self.tcp_lock:
            try:
                self.reader, self.writer = await asyncio.wait_for(
                    asyncio.open_connection(self._tcp_host, self._tcp_port),
                    timeout=3.0,
                )
                return True
            except Exception:
                self.reader = None
                self.writer = None
                return False

    def stop(self) -> Dict[str, Any]:
        if self.writer:
            try:
                self.writer.close()
            except Exception:
                pass
        self.writer = None
        self.reader = None

        if self.process:
            if self.process.poll() is None:
                self.process.terminate()
                self.process = None
                return {"status": "stopped"}
            self.process = None
            return {"status": "was_not_running"}
        return {"status": "no_process"}

    def is_running(self) -> bool:
        return self.process is not None and self.process.poll() is None

    def is_connected(self) -> bool:
        return self.writer is not None

    async def _send_recv(self, payload: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not self.writer and not await self._connect_tcp():
            return None

        if self.tcp_lock is None:
            self.tcp_lock = asyncio.Lock()

        async with self.tcp_lock:
            try:
                data = json.dumps(payload).encode("utf-8")
                msg = struct.pack("<I", len(data)) + data
                self.writer.write(msg)
                await asyncio.wait_for(self.writer.drain(), timeout=2.0)

                raw_len = await asyncio.wait_for(self.reader.readexactly(4), timeout=3.0)
                resp_len = struct.unpack("<I", raw_len)[0]
                raw_body = await asyncio.wait_for(self.reader.readexactly(resp_len), timeout=3.0)
                return json.loads(raw_body.decode("utf-8"))
            except Exception:
                if self.writer:
                    try:
                        self.writer.close()
                    except Exception:
                        pass
                self.writer = None
                self.reader = None
                return None

    async def get_sensors(self) -> Dict[str, Any]:
        resp = await self._send_recv({"type": "reset"}) or {}
        if resp:
            self.last_sensor = resp
        return resp

    async def get_schema(self) -> Dict[str, Any]:
        resp = await self._send_recv({"type": "get_schema"}) or {}
        if resp:
            self.schema = resp
            self.schema_fetched = True
        return resp

    async def send_motor(self, action: List[float]) -> Dict[str, Any]:
        resp = await self._send_recv({"type": "step", "action": action}) or {}
        if resp:
            self.last_sensor = resp
        return resp


class GodotSessionManager:
    def __init__(self, start_port: int = 9000, max_ports: int = 100):
        self.sessions: Dict[str, GodotBridge] = {}
        self.start_port = start_port
        self.max_ports = max_ports
        self.lock = threading.Lock()

    def get_or_create(self, session_id: str) -> GodotBridge:
        with self.lock:
            if session_id not in self.sessions:
                port = self.start_port + len(self.sessions) % self.max_ports
                self.sessions[session_id] = GodotBridge(session_id, port)
            return self.sessions[session_id]

    def remove(self, session_id: str) -> None:
        with self.lock:
            if session_id in self.sessions:
                self.sessions[session_id].stop()
                del self.sessions[session_id]

    def get(self, session_id: str) -> Optional[GodotBridge]:
        return self.sessions.get(session_id)

    def iter_sessions(self) -> List[GodotBridge]:
        return list(self.sessions.values())


class GodotLaunchRequest(pydantic.BaseModel):
    scene: str = "demo_generated_biped.tscn"
    godot_exe: str = ""


class GodotMotorRequest(pydantic.BaseModel):
    action: List[float] = []


async def telemetry_loop(
    session_manager: GodotSessionManager,
    broadcast_session: Callable[[str, Dict[str, Any]], Awaitable[None]],
) -> None:
    while True:
        try:
            for bridge in session_manager.iter_sessions():
                if bridge.process and bridge.process.poll() is not None:
                    bridge.stop()
                    await broadcast_session(
                        bridge.session_id,
                        {
                            "type": "engine_crashed",
                            "message": "引擎意外崩溃退出，连接已被强行掐断！",
                        },
                    )
                    continue

                if bridge.is_connected():
                    if not bridge.schema_fetched:
                        schema = await bridge.get_schema()
                        if schema:
                            await broadcast_session(
                                bridge.session_id,
                                {"type": "schema_update", "data": schema},
                            )

                    sensors = await bridge.get_sensors()
                    if sensors:
                        await broadcast_session(
                            bridge.session_id,
                            {"type": "telemetry_update", "data": sensors},
                        )
        except Exception:
            pass
        await asyncio.sleep(0.05)


def build_router(
    session_manager: GodotSessionManager,
    broadcast_session: Callable[[str, Dict[str, Any]], Awaitable[None]],
) -> APIRouter:
    router = APIRouter()

    @router.post("/api/godot/{session_id}/launch")
    async def godot_launch(session_id: str, req: GodotLaunchRequest):
        """启动特定 Session 的 Godot 实例"""
        bridge = session_manager.get_or_create(session_id)
        result = bridge.launch(scene=req.scene, godot_exe=req.godot_exe)
        if result.get("status") == "launched":
            await broadcast_session(
                session_id,
                {"type": "engine_started", "data": result},
            )
        return result

    @router.post("/api/godot/{session_id}/stop")
    async def godot_stop_session(session_id: str):
        """停止特定 Session 的 Godot 实例"""
        bridge = session_manager.get(session_id)
        if not bridge:
            return {"status": "no_process"}

        result = bridge.stop()
        await broadcast_session(session_id, {"type": "engine_stopped", "data": result})
        session_manager.remove(session_id)
        return result

    @router.get("/api/godot/{session_id}/status")
    async def godot_status_session(session_id: str):
        """获取特定 Session 的 Godot 实例状态"""
        bridge = session_manager.get(session_id)
        if not bridge:
            return {"engine_running": False, "tcp_connected": False, "last_sensor": {}}

        return {
            "engine_running": bridge.is_running(),
            "tcp_connected": bridge.is_connected(),
            "last_sensor": bridge.last_sensor,
            "schema": bridge.schema,
            "pid": bridge.process.pid if bridge.process else None,
        }

    @router.post("/api/godot/{session_id}/control")
    async def godot_control(session_id: str, req: GodotMotorRequest):
        """向运行中的特定的 Godot 机器人发送电机速度指令"""
        bridge = session_manager.get(session_id)
        if not bridge:
            return {"status": "error", "message": "无法下发指令，对应的 Session 未启动"}

        if not bridge.is_connected() and not await bridge._connect_tcp():
            msg = "未连接到 Godot TCP 服务器，请先启动场景"
            await broadcast_session(session_id, {"type": "simulation_error", "message": msg})
            return {"status": "error", "message": msg}

        result = await bridge.send_motor(req.action)
        return {"status": "ok", "response": result}

    return router
