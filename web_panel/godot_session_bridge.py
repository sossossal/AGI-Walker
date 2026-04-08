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
import queue
from pathlib import Path
from datetime import datetime
from typing import Any, Awaitable, Callable, Dict, List, Optional

import numpy as np
from fastapi import APIRouter
from web_panel.ws_protocol import MessageType, WsMessage
from agi_walker.core.utils.paths import RuntimePaths

logger = logging.getLogger(__name__)

class TrajectoryRecorder:
    """AGI-Walker V3.0 High-Performance Data Engine (Isolated)"""
    def __init__(self, session_id: str):
        self.session_id = session_id
        self.output_dir = RuntimePaths.TRAJECTORIES
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.data_queue = queue.Queue()
        self.running = True
        self.worker_thread = threading.Thread(target=self._io_worker_loop, name=f"IO_{session_id}", daemon=True)
        self.worker_thread.start()

    def record(self, state: Dict[str, Any], action: List[float]):
        if not self.running: return
        try: self.data_queue.put_nowait({"ts": time.time(), "state": state, "action": action})
        except Exception: pass

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
            with open(file_path, "w", encoding="utf-8") as f: json.dump(batch, f)
        except Exception as e: logger.error(f"IO Error: {e}")

    def stop(self):
        self.running = False
        if self.worker_thread.is_alive(): self.worker_thread.join(timeout=1.0)

class GodotBridge:
    """Manage a Godot process and its TCP control channel."""
    def __init__(self, session_id: str, port: int):
        self.session_id = session_id
        self._tcp_port = port
        self.process = None
        self.reader = None
        self.writer = None
        self.tcp_lock = asyncio.Lock()
        self.recorder = TrajectoryRecorder(session_id)
        self.last_sensor = {}
        self.on_telemetry = None

    async def start(self, headless: bool = True, scene: Optional[str] = None) -> bool:
        godot_exec = os.environ.get("GODOT_EXECUTABLE", "godot")
        project_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "godot_project")
        cmd = [godot_exec, "--path", project_dir]
        if headless: cmd.append("--headless")
        if scene: cmd.extend(["--scene", scene])
        cmd.extend(["--", "--tcp-port", str(self._tcp_port)])
        try:
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            return True
        except Exception as e:
            logger.error(f"Failed to launch Godot: {e}")
            return False

    async def send_motor(self, action: List[float]) -> Dict[str, Any]:
        resp = await self._send_recv({"type": "step", "action": action}) or {}
        if resp:
            self.last_sensor = resp
            self.recorder.record(resp, action)
            if self.on_telemetry:
                await self.on_telemetry(resp)
        return resp

    async def _send_recv(self, msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not self.writer: return None
        async with self.tcp_lock:
            try:
                data = json.dumps(msg).encode("utf-8")
                self.writer.write(struct.pack(">I", len(data)) + data)
                await self.writer.drain()
                header = await self.reader.readexactly(4)
                msg_len = struct.unpack(">I", header)[0]
                payload = await self.reader.readexactly(msg_len)
                return json.loads(payload.decode("utf-8"))
            except Exception: return None

    def stop(self):
        if self.process: self.process.terminate()
        self.recorder.stop()

class GodotSessionManager:
    """Manages simulation sessions."""
    def __init__(self):
        self.sessions = {}
        self.base_port = 9000

    def create_session(self, session_id: str) -> GodotBridge:
        if session_id not in self.sessions:
            port = self.base_port + len(self.sessions)
            self.sessions[session_id] = GodotBridge(session_id, port)
        return self.sessions[session_id]

    def get_session(self, session_id: str) -> Optional[GodotBridge]:
        return self.sessions.get(session_id)

    def close_all(self):
        for s in self.sessions.values(): s.stop()
        self.sessions.clear()

async def telemetry_loop(manager: GodotSessionManager, broadcast_callback: Callable):
    """Background loop kept signature-compatible with server lifespan wiring."""
    while True:
        await asyncio.sleep(1.0)

def build_router(manager: GodotSessionManager, broadcast_callback: Callable) -> APIRouter:
    """Align with server.py expectations (takes 2 arguments)."""
    router = APIRouter()
    @router.get("/api/simulation/status")
    async def get_sim_status():
        return {"status": "ok", "active_sessions": len(manager.sessions)}
    return router
