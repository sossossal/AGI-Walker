import asyncio
import json
import logging
import os
import shutil
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Awaitable, Callable, Dict, List, Optional

import pydantic
from fastapi import APIRouter

from web_panel.ws_protocol import MessageType, WsMessage

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
        self._detached_pid: Optional[int] = None
        self._detached_returncode: Optional[int] = None
        self._stdout_tail = ""
        self._stderr_tail = ""
        self._log_tail = ""
        self._log_file_path = ""
        self._delayed_connect_task: Optional[asyncio.Task[None]] = None
        self._launch_command: List[str] = []
        self._launch_scene = ""
        self._launch_headless = False
        self._launch_executable = ""

    def launch(self, scene: str = "demo_generated_biped.tscn", godot_exe: str = "", headless: bool = False) -> Dict[str, Any]:
        if self.is_running():
            return {"status": "already_running", "pid": self.get_pid()}

        exe = godot_exe or self._find_godot_exe()
        if not exe:
            return {"status": "error", "message": "未找到 Godot 可执行文件，请在环境变量中配置 GODOT_EXECUTABLE 路径或传入 godot_exe 参数"}

        if scene.startswith("res://"):
            scene_path = os.path.join(GODOT_PROJECT_DIR, scene.replace("res://", "", 1))
            scene_arg = scene
        else:
            scene_path = os.path.join(GODOT_PROJECT_DIR, scene)
            scene_arg = scene
        if not os.path.exists(scene_path):
            return {"status": "error", "message": f"场景文件不存在: {scene_path}"}

        self._stdout_tail = ""
        self._stderr_tail = ""
        self._log_tail = ""
        self._log_file_path = self._build_log_file_path(scene_arg) if headless else ""
        self._detached_returncode = None
        cmd = self._build_launch_command(exe, scene_arg, headless)
        self._launch_command = list(cmd)
        self._launch_scene = scene
        self._launch_headless = headless
        self._launch_executable = exe

        try:
            if os.name == "nt" and headless:
                launch_result = self._launch_windows_headless(cmd)
                if launch_result.get("status") == "launched":
                    self._delayed_connect_task = asyncio.create_task(self._delayed_tcp_connect())
                return launch_result
        except Exception as e:
            return {"status": "error", "message": str(e)}

        stdout_target: Any = subprocess.PIPE
        stderr_target: Any = subprocess.PIPE
        try:
            self.process = subprocess.Popen(
                cmd,
                stdout=stdout_target,
                stderr=stderr_target,
                creationflags=subprocess.CREATE_NEW_CONSOLE if os.name == "nt" and not headless else 0,
            )
            self._delayed_connect_task = asyncio.create_task(self._delayed_tcp_connect())
            return {"status": "launched", "pid": self.process.pid, "scene": scene, "exe": exe}
        except Exception as e:
            return {"status": "error", "message": str(e)}

    def _build_launch_command(self, exe: str, scene_arg: str, headless: bool) -> List[str]:
        cmd = [exe]
        if headless:
            cmd.append("--headless")
        cmd.extend(["--path", GODOT_PROJECT_DIR])
        if self._log_file_path:
            cmd.extend(["--log-file", self._log_file_path])
        cmd.extend(["--scene", scene_arg, "--", f"--tcp-port={self._tcp_port}"])
        return cmd

    def _build_log_file_path(self, scene_arg: str) -> str:
        scene_name = Path(scene_arg.replace("res://", "", 1)).stem or "scene"
        safe_session_id = "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in self.session_id)
        log_dir = Path(GODOT_PROJECT_DIR).parent / ".output" / "godot_session_bridge_logs"
        log_dir.mkdir(parents=True, exist_ok=True)
        return str(log_dir / f"{safe_session_id}_{scene_name}.log")

    def _powershell_quote(self, value: str) -> str:
        return "'" + value.replace("'", "''") + "'"

    def _launch_windows_headless(self, cmd: List[str]) -> Dict[str, Any]:
        exe, *args = cmd
        arg_list = ", ".join(self._powershell_quote(arg) for arg in args)
        script = (
            f"$proc = Start-Process -FilePath {self._powershell_quote(exe)} "
            f"-ArgumentList @({arg_list}) -PassThru; "
            "$proc.Id | Out-String"
        )
        result = subprocess.run(
            ["powershell.exe", "-NoProfile", "-NonInteractive", "-Command", script],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=10,
        )
        if result.returncode != 0:
            combined_output = "\n".join(
                part.strip() for part in (result.stdout, result.stderr) if part and part.strip()
            )
            return {"status": "error", "message": combined_output or f"PowerShell launcher failed: {result.returncode}"}

        pid_text = result.stdout.strip().splitlines()[0].strip() if result.stdout.strip() else ""
        if not pid_text.isdigit():
            combined_output = "\n".join(
                part.strip() for part in (result.stdout, result.stderr) if part and part.strip()
            )
            return {
                "status": "error",
                "message": combined_output or "PowerShell launcher did not return a valid PID",
            }

        self._detached_pid = int(pid_text)
        return {
            "status": "launched",
            "pid": self._detached_pid,
            "scene": self._launch_scene,
            "exe": self._launch_executable,
        }

    def _find_godot_exe(self) -> str:
        for env_key in ("GODOT_EXECUTABLE", "GODOT", "GODOT_EXE", "GODOT_PATH"):
            env_exe = os.environ.get(env_key)
            if env_exe and os.path.isfile(env_exe):
                return env_exe

        candidates = [
            "godot",
            r"C:\Program Files\Godot\Godot_v4.2.2-stable_win64.exe",
            r"C:\Godot\Godot_v4.2.2-stable_win64.exe",
            r"D:\Godot\Godot_v4.2.2-stable_win64.exe",
        ]
        for candidate in candidates:
            resolved_candidate = candidate
            if os.path.isfile(candidate):
                resolved_candidate = candidate
            else:
                resolved_candidate = shutil.which(candidate) or candidate
            try:
                result = subprocess.run(
                    [resolved_candidate, "--version"],
                    capture_output=True,
                    timeout=2,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                )
                version_output = f"{result.stdout}\n{result.stderr}".strip()
                if result.returncode == 0 and version_output:
                    return resolved_candidate
            except Exception:
                continue
        return ""

    async def _delayed_tcp_connect(self, delay: float = 3.0) -> None:
        await asyncio.sleep(delay)
        await self._connect_tcp()

    def _detached_process_status(self) -> tuple[bool, Optional[int]]:
        if os.name != "nt" or self._detached_pid is None:
            return False, self._detached_returncode

        import ctypes
        from ctypes import wintypes

        process_handle = ctypes.windll.kernel32.OpenProcess(0x1000, False, self._detached_pid)
        if not process_handle:
            return False, self._detached_returncode

        try:
            exit_code = wintypes.DWORD()
            if not ctypes.windll.kernel32.GetExitCodeProcess(process_handle, ctypes.byref(exit_code)):
                return False, self._detached_returncode
            if exit_code.value == 259:
                return True, None
            return False, int(exit_code.value)
        finally:
            ctypes.windll.kernel32.CloseHandle(process_handle)

    def _sync_detached_process_state(self) -> tuple[bool, Optional[int]]:
        running, exit_code = self._detached_process_status()
        if not running and exit_code is not None:
            self._detached_returncode = exit_code
            self._capture_process_output_tail()
        return running, exit_code

    def has_exited(self) -> bool:
        if self.process is not None and self.process.poll() is not None:
            self._capture_process_output_tail()
            return True
        if self._detached_pid is not None:
            running, _ = self._sync_detached_process_state()
            return not running
        return False

    def get_pid(self) -> Optional[int]:
        if self.process is not None:
            return self.process.pid
        return self._detached_pid

    def get_returncode(self) -> Optional[int]:
        if self.process is not None:
            return self.process.returncode
        if self._detached_pid is not None:
            self._sync_detached_process_state()
        return self._detached_returncode

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
        if self._delayed_connect_task and not self._delayed_connect_task.done():
            self._delayed_connect_task.cancel()
        self._delayed_connect_task = None

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
                try:
                    self.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2.0)
                self._capture_process_output_tail()
                self.process = None
                return {"status": "stopped"}
            self._capture_process_output_tail()
            self.process = None
            return {"status": "was_not_running"}

        if self._detached_pid is not None:
            pid = self._detached_pid
            running, exit_code = self._sync_detached_process_state()
            if running:
                subprocess.run(
                    [
                        "powershell.exe",
                        "-NoProfile",
                        "-NonInteractive",
                        "-Command",
                        f"Stop-Process -Id {pid} -Force -ErrorAction SilentlyContinue",
                    ],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=10,
                )
                deadline = time.time() + 3.0
                while time.time() < deadline:
                    running, exit_code = self._sync_detached_process_state()
                    if not running:
                        break
                    time.sleep(0.1)
                self._capture_process_output_tail()
                self._detached_pid = None
                self._detached_returncode = exit_code
                return {"status": "stopped"}

            self._capture_process_output_tail()
            self._detached_pid = None
            self._detached_returncode = exit_code
            return {"status": "was_not_running"}
        return {"status": "no_process"}

    def is_running(self) -> bool:
        if self.process is not None:
            return self.process.poll() is None
        if self._detached_pid is not None:
            running, _ = self._sync_detached_process_state()
            return running
        return False

    def is_connected(self) -> bool:
        return self.writer is not None

    async def _send_recv(self, payload: Dict[str, Any], max_retries: int = 2) -> Optional[Dict[str, Any]]:
        for attempt in range(max_retries):
            if not self.writer and not await self._connect_tcp():
                if attempt < max_retries - 1:
                    await asyncio.sleep(0.5)
                    continue
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
                except Exception as e:
                    logger.warning(f"[_send_recv] Attempt {attempt+1}/{max_retries} failed: {e}")
                    if self.writer:
                        try:
                            self.writer.close()
                        except Exception:
                            pass
                    self.writer = None
                    self.reader = None
                    if attempt < max_retries - 1:
                        await asyncio.sleep(0.5)
                        continue
                    return None
        return None

    async def get_sensors(self) -> Dict[str, Any]:
        # Poll telemetry via a no-op step instead of reset, otherwise the
        # background loop continually rewinds the simulation state.
        resp = await self._send_recv({"type": "step", "action": []}) or {}
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

    async def send_load_robot(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return await self._send_recv({"type": "load_robot", "robot_config": config}) or {}

    async def wait_until_connected(
        self,
        timeout_seconds: float = 10.0,
        poll_interval_seconds: float = 0.25,
    ) -> bool:
        deadline = asyncio.get_running_loop().time() + timeout_seconds
        while asyncio.get_running_loop().time() < deadline:
            if self.is_connected():
                return True
            if self.has_exited():
                return False
            if await self._connect_tcp():
                return True
            await asyncio.sleep(poll_interval_seconds)
        return self.is_connected()

    async def wait_until_schema(
        self,
        timeout_seconds: float = 10.0,
        poll_interval_seconds: float = 0.25,
    ) -> Dict[str, Any]:
        deadline = asyncio.get_running_loop().time() + timeout_seconds
        while asyncio.get_running_loop().time() < deadline:
            if self.has_exited():
                return {}
            schema = await self.get_schema()
            if schema.get("sensors") and schema.get("actuators"):
                return schema
            await asyncio.sleep(poll_interval_seconds)
        return self.schema if self.schema.get("sensors") and self.schema.get("actuators") else {}

    def _capture_process_output_tail(self, max_chars: int = 2000) -> None:
        if self._log_file_path and os.path.exists(self._log_file_path):
            try:
                self._log_tail = Path(self._log_file_path).read_text(
                    encoding="utf-8",
                    errors="replace",
                )[-max_chars:]
            except OSError:
                pass

        if not self.process:
            return

        if self.process.poll() is None:
            return

        try:
            stdout_data, stderr_data = self.process.communicate(timeout=0.2)
        except Exception:
            return

        if stdout_data:
            self._stdout_tail = stdout_data.decode("utf-8", errors="replace")[-max_chars:]
        if stderr_data:
            self._stderr_tail = stderr_data.decode("utf-8", errors="replace")[-max_chars:]

    def get_process_diagnostics(self) -> Dict[str, Any]:
        if self.has_exited():
            self._capture_process_output_tail()

        appdata_log_dir = None
        if os.name == "nt" and os.getenv("APPDATA"):
            appdata_log_dir = os.path.join(
                os.getenv("APPDATA", ""),
                "Godot",
                "app_userdata",
                "AGI-Walker Simulation",
                "logs",
            )

        return {
            "pid": self.get_pid(),
            "returncode": self.get_returncode(),
            "running": self.is_running(),
            "tcp_connected": self.is_connected(),
            "schema_fetched": self.schema_fetched,
            "tcp_host": self._tcp_host,
            "tcp_port": self._tcp_port,
            "launch_scene": self._launch_scene,
            "headless": self._launch_headless,
            "godot_executable": self._launch_executable,
            "launch_command": self._launch_command,
            "project_dir": GODOT_PROJECT_DIR,
            "delayed_connect_task_pending": bool(
                self._delayed_connect_task and not self._delayed_connect_task.done()
            ),
            "appdata_log_dir": appdata_log_dir,
            "log_file_path": self._log_file_path or None,
            "log_tail": self._log_tail,
            "stdout_tail": self._stdout_tail,
            "stderr_tail": self._stderr_tail,
        }


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
    headless: bool = False


class GodotMotorRequest(pydantic.BaseModel):
    action: List[float] = []


async def telemetry_loop(
    session_manager: GodotSessionManager,
    broadcast_session: Callable[[str, Dict[str, Any]], Awaitable[None]],
) -> None:
    while True:
        try:
            for bridge in session_manager.iter_sessions():
                if bridge.has_exited():
                    diagnostics = bridge.get_process_diagnostics()
                    returncode = diagnostics.get("returncode")
                    bridge.stop()
                    await broadcast_session(
                        bridge.session_id,
                        WsMessage(
                            type=MessageType.SIMULATION_ERROR.value,
                            payload={
                                "error": f"引擎意外崩溃退出 (退出码: {returncode})",
                                "error_type": "engine_crashed",
                                "details": diagnostics.get("stderr_tail")
                                or diagnostics.get("stdout_tail")
                                or diagnostics.get("log_tail", ""),
                            },
                            status="push",
                        ).to_dict(),
                    )
                    continue

                if bridge.is_connected():
                    if not bridge.schema_fetched:
                        schema = await bridge.get_schema()
                        if schema:
                            await broadcast_session(
                                bridge.session_id,
                                WsMessage(
                                    type="schema.update",
                                    payload={"data": schema},
                                    status="push",
                                ).to_dict(),
                            )

                    sensors = await bridge.get_sensors()
                    if sensors:
                        await broadcast_session(
                            bridge.session_id,
                            WsMessage(
                                type=MessageType.TELEMETRY_UPDATE.value,
                                payload={"data": sensors},
                                status="push",
                            ).to_dict(),
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
        result = bridge.launch(scene=req.scene, godot_exe=req.godot_exe, headless=req.headless)
        if result.get("status") == "launched":
            await broadcast_session(
                session_id,
                WsMessage(
                    type=MessageType.SIMULATION_STATUS.value,
                    payload={"status": "engine_started", "details": result},
                    status="push",
                ).to_dict(),
            )
        return result

    @router.post("/api/godot/{session_id}/stop")
    async def godot_stop_session(session_id: str):
        """停止特定 Session 的 Godot 实例"""
        bridge = session_manager.get(session_id)
        if not bridge:
            return {"status": "no_process"}

        result = bridge.stop()
        await broadcast_session(
            session_id,
            WsMessage(
                type=MessageType.SIMULATION_STATUS.value,
                payload={"status": "engine_stopped", "details": result},
                status="push",
            ).to_dict(),
        )
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
            "pid": bridge.get_pid(),
        }

    @router.post("/api/godot/{session_id}/control")
    async def godot_control(session_id: str, req: GodotMotorRequest):
        """向运行中的特定的 Godot 机器人发送电机速度指令"""
        bridge = session_manager.get(session_id)
        if not bridge:
            return {"status": "error", "message": "无法下发指令，对应的 Session 未启动"}

        if not bridge.is_connected() and not await bridge._connect_tcp():
            msg = "未连接到 Godot TCP 服务器，请先启动场景"
            await broadcast_session(
                session_id,
                WsMessage(
                    type=MessageType.SIMULATION_ERROR.value,
                    payload={"error": msg, "error_type": "connection"},
                    status="push",
                ).to_dict(),
            )
            return {"status": "error", "message": msg}

        result = await bridge.send_motor(req.action)
        return {"status": "ok", "response": result}

    return router
