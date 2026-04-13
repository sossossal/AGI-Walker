import asyncio
import json
import logging
import os
import shutil
import struct
import subprocess
import threading
import time
import queue
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from fastapi import APIRouter

from agi_walker.core.utils.paths import RuntimePaths

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parent.parent
GODOT_PROJECT_DIR = str(REPO_ROOT / "godot_project")
GODOT_SESSION_LOG_DIR = RuntimePaths.SESSIONS / "godot_logs"
GODOT_SESSION_LOG_DIR.mkdir(parents=True, exist_ok=True)
GODOT_SESSION_STATUS_SCHEMA_VERSION = "1.0"
GODOT_SESSION_STATES = (
    "disconnected",
    "launching",
    "connected",
    "schema_ready",
    "running",
    "failed",
)


def disconnected_session_status(session_id: str) -> Dict[str, Any]:
    """Build the canonical status for a session that has not been created."""
    return {
        "schema_version": GODOT_SESSION_STATUS_SCHEMA_VERSION,
        "session_id": session_id,
        "session_state": "disconnected",
        "state_changed_at": datetime.now().isoformat(),
        "engine_running": False,
        "running": False,
        "tcp_connected": False,
        "connected": False,
        "schema_available": False,
        "schema": {},
        "last_sensor": {},
        "pid": None,
        "tcp_port": None,
        "log_file_path": None,
        "last_connect_error": None,
        "failure_stage": None,
        "failure_message": None,
    }


class TrajectoryRecorder:
    """AGI-Walker V3.0 High-Performance Data Engine (Isolated)"""

    def __init__(self, session_id: str):
        self.session_id = session_id
        self.output_dir = RuntimePaths.TRAJECTORIES
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.data_queue = queue.Queue()
        self.running = True
        self.worker_thread = threading.Thread(
            target=self._io_worker_loop, name=f"IO_{session_id}", daemon=True
        )
        self.worker_thread.start()

    def record(self, state: Dict[str, Any], action: List[float]):
        if not self.running:
            return
        try:
            self.data_queue.put_nowait(
                {"ts": time.time(), "state": state, "action": action}
            )
        except Exception:
            pass

    def _io_worker_loop(self):
        batch = []
        while self.running or not self.data_queue.empty():
            try:
                item = self.data_queue.get(timeout=0.5)
                batch.append(item)
                if len(batch) >= 50:
                    self._flush_batch(batch)
                    batch = []
                self.data_queue.task_done()
            except queue.Empty:
                if batch:
                    self._flush_batch(batch)
                    batch = []
                continue

    def _flush_batch(self, batch: List[Dict]):
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        file_path = self.output_dir / f"{self.session_id}_{ts_str}.json"
        try:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(batch, f)
        except Exception as e:
            logger.error(f"IO Error: {e}")

    def stop(self):
        self.running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=1.0)


class GodotBridge:
    """Manage a Godot process and its TCP control channel."""

    def __init__(self, session_id: str, port: int):
        self.session_id = session_id
        self._tcp_port = port
        self.process: Optional[subprocess.Popen[Any]] = None
        self.reader = None
        self.writer = None
        self.tcp_lock = asyncio.Lock()
        self.recorder = TrajectoryRecorder(session_id)
        self.last_sensor: Dict[str, Any] = {}
        self.on_telemetry = None
        self._schema: Optional[Dict[str, Any]] = None
        self._detached_pid: Optional[int] = None
        self._delayed_connect_task: Optional[asyncio.Task[Any]] = None
        self._log_file_path: Optional[str] = None
        self._last_connect_error: Optional[str] = None
        self.session_state = "disconnected"
        self.state_changed_at = datetime.now().isoformat()
        self.failure_stage: Optional[str] = None
        self.failure_message: Optional[str] = None

    def _set_state(
        self,
        state: str,
        *,
        failure_stage: Optional[str] = None,
        failure_message: Optional[str] = None,
    ) -> None:
        if state not in GODOT_SESSION_STATES:
            raise ValueError(f"Unknown Godot session state: {state}")
        if (
            self.session_state == state
            and self.failure_stage == failure_stage
            and self.failure_message == failure_message
        ):
            return
        self.session_state = state
        self.state_changed_at = datetime.now().isoformat()
        self.failure_stage = failure_stage if state == "failed" else None
        self.failure_message = failure_message if state == "failed" else None

    def _find_godot_exe(self) -> str:
        for env_name in (
            "GODOT_EXECUTABLE",
            "GODOT",
            "GODOT_EXE",
            "GODOT_PATH",
        ):
            configured = os.getenv(env_name, "").strip()
            if configured:
                return configured

        for candidate in ("godot", "godot4", "Godot_v4", "Godot"):
            resolved = shutil.which(candidate)
            if resolved:
                return resolved
        return ""

    def _build_log_file_path(self, scene: Optional[str]) -> str:
        scene_stem = Path(scene).stem if scene else "default"
        return str(GODOT_SESSION_LOG_DIR / f"{self.session_id}_{scene_stem}.log")

    def _build_launch_command(
        self, godot_exe: str, scene: Optional[str], headless: bool
    ) -> List[str]:
        cmd = [godot_exe]
        if headless:
            cmd.append("--headless")
        cmd.extend(["--path", GODOT_PROJECT_DIR])
        if self._log_file_path:
            cmd.extend(["--log-file", self._log_file_path])
        if scene:
            cmd.extend(["--scene", scene])
        cmd.extend(["--", f"--tcp-port={self._tcp_port}"])
        return cmd

    def _launch_windows_headless(self, cmd: List[str]) -> Dict[str, Any]:
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=creationflags,
        )
        self._detached_pid = self.process.pid
        return {
            "status": "launched",
            "pid": self.process.pid,
            "exe": cmd[0],
            "scene": cmd[cmd.index("--scene") + 1] if "--scene" in cmd else None,
        }

    async def _connect_tcp(self) -> bool:
        if self.is_connected():
            return True
        try:
            self.reader, self.writer = await asyncio.open_connection(
                "127.0.0.1", self._tcp_port
            )
            self._last_connect_error = None
            self._set_state("schema_ready" if self._schema else "connected")
            return True
        except Exception as exc:
            self._last_connect_error = str(exc)
            return False

    async def _delayed_connect(self, timeout_seconds: float = 15.0) -> None:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if await self._connect_tcp():
                return
            if self.process is not None and self.process.poll() is not None:
                self._set_state(
                    "failed",
                    failure_stage="process_exit",
                    failure_message="Godot process exited before TCP connection.",
                )
                return
            await asyncio.sleep(0.25)
        if not self.is_connected():
            self._set_state(
                "failed",
                failure_stage="tcp_connect",
                failure_message=self._last_connect_error or "TCP connect timeout.",
            )

    def launch(
        self,
        scene: Optional[str] = None,
        godot_exe: Optional[str] = None,
        headless: bool = True,
    ) -> Dict[str, Any]:
        if self.is_running():
            return {
                "status": "already_running",
                "pid": self.get_pid(),
                "scene": scene,
                "exe": godot_exe or self._find_godot_exe(),
                "session_state": self.session_state,
            }

        resolved_exe = (godot_exe or self._find_godot_exe()).strip()
        if not resolved_exe:
            message = "Godot executable not found. Configure GODOT_EXECUTABLE or related env vars."
            self._set_state("failed", failure_stage="launch", failure_message=message)
            return {
                "status": "error",
                "message": message,
                "session_state": self.session_state,
            }

        project_dir = Path(GODOT_PROJECT_DIR)
        if not project_dir.exists():
            message = f"Godot project directory does not exist: {project_dir}"
            self._set_state("failed", failure_stage="launch", failure_message=message)
            return {
                "status": "error",
                "message": message,
                "session_state": self.session_state,
            }

        self._log_file_path = self._build_log_file_path(scene)
        cmd = self._build_launch_command(resolved_exe, scene, headless)
        try:
            self._set_state("launching")
            result = self._launch_windows_headless(cmd)
        except Exception as exc:
            logger.error("Failed to launch Godot session bridge: %s", exc)
            self._set_state("failed", failure_stage="launch", failure_message=str(exc))
            return {
                "status": "error",
                "message": str(exc),
                "session_state": self.session_state,
            }

        try:
            self._delayed_connect_task = asyncio.create_task(self._delayed_connect())
        except RuntimeError:
            self._delayed_connect_task = None
        result["session_state"] = self.session_state
        return result

    async def start(self, headless: bool = True, scene: Optional[str] = None) -> bool:
        result = self.launch(scene=scene, headless=headless)
        return result.get("status") in {"launched", "already_running"}

    def is_connected(self) -> bool:
        return self.writer is not None and not self.writer.is_closing()

    def is_running(self) -> bool:
        if self.process is not None:
            return self.process.poll() is None
        return self._detached_pid is not None

    def get_pid(self) -> Optional[int]:
        if self.process is not None:
            return self.process.pid
        return self._detached_pid

    async def send_motor(self, action: List[float]) -> Dict[str, Any]:
        resp = await self._send_recv({"type": "step", "action": action}) or {}
        if resp:
            self.last_sensor = resp
            self._set_state("running")
            self.recorder.record(resp, action)
            if self.on_telemetry:
                await self.on_telemetry(resp)
        return resp

    async def get_sensors(self) -> Dict[str, Any]:
        return await self.send_motor([])

    async def send_load_robot(self, robot_config: Dict[str, Any]) -> Dict[str, Any]:
        response = (
            await self._send_recv({"type": "load_robot", "robot_config": robot_config})
            or {}
        )
        if response.get("status") == "error":
            self._set_state(
                "failed",
                failure_stage="load_robot",
                failure_message=str(response.get("message") or response),
            )
        elif response:
            self._set_state("schema_ready" if self._schema else "connected")
        return response

    async def wait_until_connected(self, timeout_seconds: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            if self.is_connected() or await self._connect_tcp():
                return True
            await asyncio.sleep(0.1)
        self._set_state(
            "failed",
            failure_stage="tcp_connect",
            failure_message=self._last_connect_error or "TCP connect timeout.",
        )
        return False

    async def wait_until_schema(
        self, timeout_seconds: float = 5.0
    ) -> Optional[Dict[str, Any]]:
        if self._schema:
            return self._schema

        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            schema = await self._send_recv({"type": "get_schema"})
            if isinstance(schema, dict) and schema:
                self._schema = schema
                self._set_state("schema_ready")
                return schema
            await asyncio.sleep(0.1)
        return None

    async def _send_recv(self, msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not self.writer:
            return None
        async with self.tcp_lock:
            try:
                data = json.dumps(msg).encode("utf-8")
                # Godot's TCP server reads StreamPeer lengths in little-endian
                # mode by default (`BigEndian=false`), so the bridge must use
                # the same framing to interoperate with real headless scenes.
                self.writer.write(struct.pack("<I", len(data)) + data)
                await self.writer.drain()
                header = await self.reader.readexactly(4)
                msg_len = struct.unpack("<I", header)[0]
                payload = await self.reader.readexactly(msg_len)
                return json.loads(payload.decode("utf-8"))
            except Exception as exc:
                self._last_connect_error = str(exc)
                self._set_state(
                    "failed",
                    failure_stage="tcp_io",
                    failure_message=str(exc),
                )
                return None

    def get_status_payload(self) -> Dict[str, Any]:
        return {
            "schema_version": GODOT_SESSION_STATUS_SCHEMA_VERSION,
            "session_id": self.session_id,
            "session_state": self.session_state,
            "state_changed_at": self.state_changed_at,
            "engine_running": self.is_running(),
            "running": self.is_running(),
            "tcp_connected": self.is_connected(),
            "connected": self.is_connected(),
            "tcp_port": self._tcp_port,
            "pid": self.get_pid(),
            "schema_available": bool(self._schema),
            "schema": self._schema or {},
            "last_sensor": self.last_sensor,
            "log_file_path": self._log_file_path,
            "last_connect_error": self._last_connect_error,
            "failure_stage": self.failure_stage,
            "failure_message": self.failure_message,
        }

    def get_process_diagnostics(self) -> Dict[str, Any]:
        diagnostics = self.get_status_payload()
        diagnostics["last_sensor_keys"] = sorted(self.last_sensor.keys())
        return diagnostics

    def stop(self):
        if self._delayed_connect_task is not None:
            self._delayed_connect_task.cancel()
            self._delayed_connect_task = None

        if self.writer is not None:
            self.writer.close()
            self.writer = None
            self.reader = None

        stopped_pid = self.get_pid()
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None
        self._detached_pid = None
        self.recorder.stop()
        self._set_state("disconnected")
        return {
            "status": "stopped",
            "pid": stopped_pid,
            "session_state": self.session_state,
        }


class GodotSessionManager:
    """Manages simulation sessions."""

    def __init__(self):
        self.sessions: Dict[str, GodotBridge] = {}
        self.base_port = 9000

    def create_session(self, session_id: str) -> GodotBridge:
        if session_id not in self.sessions:
            port = self.base_port + len(self.sessions)
            self.sessions[session_id] = GodotBridge(session_id, port)
        return self.sessions[session_id]

    def get_or_create(self, session_id: str) -> GodotBridge:
        return self.create_session(session_id)

    def get_session(self, session_id: str) -> Optional[GodotBridge]:
        return self.sessions.get(session_id)

    def close_all(self):
        for s in self.sessions.values():
            s.stop()
        self.sessions.clear()


async def telemetry_loop(manager: GodotSessionManager, broadcast_callback: Callable):
    """Background loop kept signature-compatible with server lifespan wiring."""
    while True:
        await asyncio.sleep(1.0)


def build_router(
    manager: GodotSessionManager, broadcast_callback: Callable
) -> APIRouter:
    """Provide the preferred session-bridge transport routes."""
    router = APIRouter()

    @router.get("/api/simulation/status")
    async def get_sim_status():
        return {"status": "ok", "active_sessions": len(manager.sessions)}

    @router.get("/api/godot/{session_id}/status")
    async def get_session_status(session_id: str):
        bridge = manager.get_session(session_id)
        if bridge is None:
            return disconnected_session_status(session_id)
        return bridge.get_status_payload()

    @router.post("/api/godot/{session_id}/launch")
    async def launch_session(session_id: str, payload: Dict[str, Any]):
        bridge = manager.get_or_create(session_id)
        result = bridge.launch(
            scene=payload.get("scene"),
            godot_exe=payload.get("godot_exe"),
            headless=payload.get("headless", True),
        )
        return {**result, "session": bridge.get_status_payload()}

    @router.post("/api/godot/{session_id}/stop")
    async def stop_session(session_id: str):
        bridge = manager.get_session(session_id)
        if bridge is None:
            return {"status": "error", "message": f"Session '{session_id}' not found."}
        return bridge.stop()

    @router.post("/api/godot/{session_id}/control")
    async def control_session(session_id: str, payload: Dict[str, Any]):
        bridge = manager.get_session(session_id)
        if bridge is None:
            return {
                "status": "error",
                "message": f"Session '{session_id}' not found.",
                "session": disconnected_session_status(session_id),
                "session_state": "disconnected",
            }
        if not bridge.is_connected():
            return {
                "status": "error",
                "message": f"Session '{session_id}' is not connected.",
                "session": bridge.get_status_payload(),
                "session_state": bridge.session_state,
            }
        action = payload.get("action") or []
        sensors = await bridge.send_motor(action)
        return {
            "status": "success",
            "sensors": sensors,
            "session": bridge.get_status_payload(),
            "session_state": bridge.session_state,
        }

    return router
