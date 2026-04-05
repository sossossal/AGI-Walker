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
from web_panel.ws_protocol import MessageType, WsMessage

logger = logging.getLogger(__name__)

class TrajectoryRecorder:
    """
    AGI-Walker V3.0 High-Performance Data Engine.
    Uses an asynchronous background thread to record trajectories without blocking the 1kHz loop.
    """
    def __init__(self, session_id: str):
        self.session_id = session_id
        self.output_dir = Path(".output/trajectories")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        self.data_queue = queue.Queue()
        self.running = True
        
        # 启动后台 IO 线程
        self.worker_thread = threading.Thread(target=self._io_worker_loop, name=f"IO_{session_id}")
        self.worker_thread.daemon = True
        self.worker_thread.start()

    def record(self, state: Dict[str, Any], action: List[float]):
        """Non-blocking record: pushes data into the queue and returns immediately."""
        if not self.running: return
        try:
            self.data_queue.put_nowait({
                "ts": time.time(),
                "state": state,
                "action": action
            })
        except Exception:
            pass

    def _io_worker_loop(self):
        """Background thread scanning the queue and writing batches to disk."""
        batch = []
        batch_size = 100
        
        while self.running or not self.data_queue.empty():
            try:
                # 获取数据，超时则检查 running 状态
                item = self.data_queue.get(timeout=0.5)
                batch.append(item)
                
                if len(batch) >= batch_size:
                    self._flush_batch(batch)
                    batch = []
                
                self.data_queue.task_done()
            except queue.Empty:
                if batch:
                    self._flush_batch(batch)
                    batch = []
                continue

    def _flush_batch(self, batch: List[Dict]):
        """Actual disk IO performed only by background thread."""
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        file_path = self.output_dir / f"{self.session_id}_{ts_str}.json"
        try:
            with open(file_path, "w", encoding="utf-8") as f:
                json.dump(batch, f)
        except Exception as e:
            logger.error(f"Failed to write trajectory batch: {e}")

    def stop(self):
        """Gracefully shutdown the IO worker and ensure all data is flushed."""
        self.running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        logger.info(f"Trajectory recorder stopped for {self.session_id}")

    def flush(self):
        pass

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
        self.last_shadow_pose: List[float] = [] # V2.1
        self.recorder = TrajectoryRecorder(session_id) # V2.1/V3.0 Async
        self.schema: Dict[str, Any] = {}
        self.schema_fetched = False
        self.tcp_lock: Optional[asyncio.Lock] = None
        self._tcp_host = "127.0.0.1"
        self._tcp_port = port
        self.telemetry_task: Optional[asyncio.Task] = None
        self.on_telemetry: Optional[Callable[[Dict[str, Any]], Awaitable[None]]] = None

    async def start(self, headless: bool = True, scene: Optional[str] = None) -> bool:
        godot_exec = os.environ.get("GODOT_EXECUTABLE", "godot")
        cmd = [godot_exec, "--path", GODOT_PROJECT_DIR]
        if headless: cmd.append("--headless")
        if scene:
            cmd.extend(["--scene", scene])
        
        cmd.extend(["--", "--tcp-port", str(self._tcp_port)])
        
        try:
            self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            self.tcp_lock = asyncio.Lock()
            logger.info(f"Godot process started (PID: {self.process.pid}) on port {self._tcp_port}")
            return True
        except Exception as e:
            logger.error(f"Failed to launch Godot: {e}")
            return False

    async def send_motor(self, action: List[float]) -> Dict[str, Any]:
        resp = await self._send_recv({"type": "step", "action": action}) or {}
        if resp:
            self.last_sensor = resp
            self.recorder.record(resp, action) # V3.0: Async Record
        return resp

    async def send_shadow_pose(self, pose: List[float]) -> Dict[str, Any]:
        """V2.1 Addition: Update the phantom shadow in simulation."""
        self.last_shadow_pose = pose
        return await self._send_recv({"type": "update_shadow", "pose": pose}) or {}

    async def send_load_robot(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return await self._send_recv({"type": "load_robot", "robot_config": config}) or {}

    async def wait_until_connected(self, timeout: float = 10.0) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                self.reader, self.writer = await asyncio.open_connection(self._tcp_host, self._tcp_port)
                logger.info("TCP Connection to Godot established.")
                return True
            except (ConnectionRefusedError, OSError):
                await asyncio.sleep(0.5)
        return False

    async def _send_recv(self, msg: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        if not self.writer or not self.reader: return None
        async with self.tcp_lock:
            try:
                data = json.dumps(msg).encode("utf-8")
                self.writer.write(struct.pack(">I", len(data)) + data)
                await self.writer.drain()
                
                header = await self.reader.readexactly(4)
                msg_len = struct.unpack(">I", header)[0]
                payload = await self.reader.readexactly(msg_len)
                return json.loads(payload.decode("utf-8"))
            except Exception as e:
                logger.error(f"TCP IO Error: {e}")
                return None

    def stop(self):
        if self.process: self.process.terminate()
        self.recorder.stop()
        if self.writer: self.writer.close()
