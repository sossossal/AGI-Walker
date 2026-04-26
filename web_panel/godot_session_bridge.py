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
from typing import Any, Callable, Dict, List, Literal, Optional
from sqlalchemy import delete, func, inspect, select, text
from sqlalchemy.sql import Select

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import JSONResponse, PlainTextResponse

from agi_walker.core.utils.paths import RuntimePaths
from web_panel.database import AsyncSessionLocal, engine
from web_panel.auth import decode_access_token
from web_panel.models import OperatorHistoryEntry, User

logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parent.parent
GODOT_PROJECT_DIR = str(REPO_ROOT / "godot_project")
GODOT_SESSION_LOG_DIR = RuntimePaths.SESSIONS / "godot_logs"
GODOT_SESSION_LOG_DIR.mkdir(parents=True, exist_ok=True)
GODOT_OPERATOR_HISTORY_MAX_ITEMS = max(
    1, int(os.getenv("AGI_WALKER_GODOT_OPERATOR_HISTORY_MAX_ITEMS", "200"))
)
DEFAULT_OPERATOR_HISTORY_PAGE_SIZE = max(
    1, int(os.getenv("AGI_WALKER_GODOT_OPERATOR_HISTORY_PAGE_SIZE", "10"))
)
_OPERATOR_HISTORY_TABLE_READY = False
GODOT_SESSION_STATUS_SCHEMA_VERSION = "1.0"
GODOT_SESSION_STATES = (
    "disconnected",
    "launching",
    "connected",
    "schema_ready",
    "running",
    "failed",
)


async def _ensure_operator_history_table() -> None:
    """Create the operator-history table on demand when migrations are absent."""
    global _OPERATOR_HISTORY_TABLE_READY
    if _OPERATOR_HISTORY_TABLE_READY:
        return
    async with engine.begin() as conn:
        def _ensure_schema(sync_conn: Any) -> None:
            OperatorHistoryEntry.__table__.create(sync_conn, checkfirst=True)
            existing_columns = {
                column["name"]
                for column in inspect(sync_conn).get_columns(
                    OperatorHistoryEntry.__tablename__
                )
            }
            missing_columns = {
                "operator": "ALTER TABLE operator_history_entries ADD COLUMN operator VARCHAR(96)",
                "tag": "ALTER TABLE operator_history_entries ADD COLUMN tag VARCHAR(96)",
                "note": "ALTER TABLE operator_history_entries ADD COLUMN note VARCHAR(512)",
                "audit_user_id": "ALTER TABLE operator_history_entries ADD COLUMN audit_user_id INTEGER",
                "audit_username": "ALTER TABLE operator_history_entries ADD COLUMN audit_username VARCHAR(96)",
                "audit_source": "ALTER TABLE operator_history_entries ADD COLUMN audit_source VARCHAR(32)",
            }
            for column_name, ddl in missing_columns.items():
                if column_name not in existing_columns:
                    sync_conn.execute(text(ddl))

        await conn.run_sync(_ensure_schema)
    _OPERATOR_HISTORY_TABLE_READY = True


def _parse_optional_iso_datetime(raw_value: Optional[str], field_name: str) -> Optional[datetime]:
    """Parse one optional ISO datetime query parameter."""
    if raw_value is None:
        return None
    normalized = raw_value.strip()
    if not normalized:
        return None
    try:
        return datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid {field_name} '{raw_value}'. Expected ISO datetime.",
        ) from exc


def _normalize_optional_text(raw_value: Optional[str]) -> Optional[str]:
    """Trim one optional text query parameter and collapse empty values to None."""
    if raw_value is None:
        return None
    normalized = raw_value.strip()
    return normalized or None


def _parse_history_sort_field(raw_value: Optional[str]) -> str:
    """Validate history sort field."""
    normalized = _normalize_optional_text(raw_value) or "created_at"
    if normalized not in {"created_at", "session_id", "kind", "route_mode"}:
        raise HTTPException(
            status_code=400,
            detail=(
                f"Invalid sort_by '{raw_value}'. "
                "Expected one of: created_at, session_id, kind, route_mode."
            ),
        )
    return normalized


def _parse_history_sort_order(raw_value: Optional[str]) -> Literal["asc", "desc"]:
    """Validate history sort order."""
    normalized = (_normalize_optional_text(raw_value) or "desc").lower()
    if normalized not in {"asc", "desc"}:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid sort_order '{raw_value}'. Expected 'asc' or 'desc'.",
        )
    return normalized  # type: ignore[return-value]


def _parse_note_exact(raw_value: Optional[bool]) -> bool:
    """Normalize one optional exact-match flag for note filtering."""
    return bool(raw_value)


async def _resolve_optional_audit_identity(
    authorization: Optional[str],
) -> Dict[str, Optional[Any]]:
    """Resolve one optional Bearer token into persisted audit identity fields."""
    raw_header = _normalize_optional_text(authorization)
    if not raw_header:
        return {
            "audit_user_id": None,
            "audit_username": None,
            "audit_source": None,
        }

    scheme, _, token = raw_header.partition(" ")
    if scheme.lower() != "bearer" or not token.strip():
        raise HTTPException(status_code=401, detail="Invalid Authorization header")

    payload = decode_access_token(token.strip())
    if payload is None or not payload.get("sub"):
        raise HTTPException(status_code=401, detail="Could not validate credentials")

    async with AsyncSessionLocal() as session:
        result = await session.execute(
            select(User).where(User.username == str(payload["sub"]))
        )
        user = result.scalar_one_or_none()

    if user is None:
        raise HTTPException(status_code=401, detail="Could not validate credentials")

    return {
        "audit_user_id": user.id,
        "audit_username": user.username,
        "audit_source": "bearer",
    }


def _apply_history_filters(
    statement: Any,
    *,
    session_id: Optional[str] = None,
    session_query: Optional[str] = None,
    operator: Optional[str] = None,
    tag: Optional[str] = None,
    note: Optional[str] = None,
    note_exact: bool = False,
    kind: Optional[str] = None,
    route_mode: Optional[str] = None,
    created_after: Optional[datetime] = None,
    created_before: Optional[datetime] = None,
) -> Any:
    """Apply shared operator-history filters to one SQLAlchemy statement."""
    if session_id:
        statement = statement.where(OperatorHistoryEntry.session_id == session_id)
    if session_query:
        statement = statement.where(
            OperatorHistoryEntry.session_id.ilike(f"%{session_query}%")
        )
    if operator:
        statement = statement.where(OperatorHistoryEntry.operator.ilike(f"%{operator}%"))
    if tag:
        statement = statement.where(OperatorHistoryEntry.tag.ilike(f"%{tag}%"))
    if note:
        if note_exact:
            statement = statement.where(OperatorHistoryEntry.note == note)
        else:
            statement = statement.where(OperatorHistoryEntry.note.ilike(f"%{note}%"))
    if kind:
        statement = statement.where(OperatorHistoryEntry.kind == kind)
    if route_mode:
        statement = statement.where(OperatorHistoryEntry.route_mode == route_mode)
    if created_after is not None:
        statement = statement.where(OperatorHistoryEntry.created_at >= created_after)
    if created_before is not None:
        statement = statement.where(OperatorHistoryEntry.created_at <= created_before)
    return statement


def _apply_history_sorting(
    statement: Select[Any],
    *,
    sort_by: str,
    sort_order: Literal["asc", "desc"],
) -> Select[Any]:
    """Apply one validated sort order to history statements."""
    sort_column = {
        "created_at": OperatorHistoryEntry.created_at,
        "session_id": OperatorHistoryEntry.session_id,
        "kind": OperatorHistoryEntry.kind,
        "route_mode": OperatorHistoryEntry.route_mode,
    }[sort_by]
    primary = sort_column.asc() if sort_order == "asc" else sort_column.desc()
    secondary = (
        OperatorHistoryEntry.id.asc()
        if sort_order == "asc"
        else OperatorHistoryEntry.id.desc()
    )
    return statement.order_by(primary, secondary)


async def _get_operator_history_listing(
    *,
    session_id: Optional[str] = None,
    session_query: Optional[str] = None,
    operator: Optional[str] = None,
    tag: Optional[str] = None,
    note: Optional[str] = None,
    note_exact: bool = False,
    limit: int = DEFAULT_OPERATOR_HISTORY_PAGE_SIZE,
    offset: int = 0,
    kind: Optional[str] = None,
    route_mode: Optional[str] = None,
    created_after: Optional[datetime] = None,
    created_before: Optional[datetime] = None,
    sort_by: str = "created_at",
    sort_order: Literal["asc", "desc"] = "desc",
) -> Dict[str, Any]:
    """Return one paginated operator-history listing across one or all sessions."""
    await _ensure_operator_history_table()
    safe_limit = max(1, limit)
    safe_offset = max(0, offset)

    item_statement = _apply_history_filters(
        select(OperatorHistoryEntry),
        session_id=session_id,
        session_query=session_query,
        operator=operator,
        tag=tag,
        note=note,
        note_exact=note_exact,
        kind=kind,
        route_mode=route_mode,
        created_after=created_after,
        created_before=created_before,
    )
    item_statement = _apply_history_sorting(
        item_statement,
        sort_by=sort_by,
        sort_order=sort_order,
    )
    count_statement = _apply_history_filters(
        select(func.count(OperatorHistoryEntry.id)),
        session_id=session_id,
        session_query=session_query,
        operator=operator,
        tag=tag,
        note=note,
        note_exact=note_exact,
        kind=kind,
        route_mode=route_mode,
        created_after=created_after,
        created_before=created_before,
    )

    async with AsyncSessionLocal() as session:
        item_result = await session.execute(
            item_statement.offset(safe_offset).limit(safe_limit)
        )
        count_result = await session.execute(count_statement)
        rows = item_result.scalars().all()
        total_count = int(count_result.scalar_one() or 0)

    return {
        "session_id": session_id,
        "history": [row.to_dict() for row in rows],
        "history_count": total_count,
        "offset": safe_offset,
        "limit": safe_limit,
        "has_more": safe_offset + safe_limit < total_count,
        "history_storage": "database",
        "filters": {
            "session_id": session_id,
            "session_query": session_query,
            "operator": operator,
            "tag": tag,
            "note": note,
            "note_exact": note_exact,
            "kind": kind,
            "route_mode": route_mode,
            "created_after": created_after.isoformat() if created_after else None,
            "created_before": created_before.isoformat() if created_before else None,
            "sort_by": sort_by,
            "sort_order": sort_order,
        },
    }


async def _get_operator_history_summary(
    *,
    session_id: Optional[str] = None,
    session_query: Optional[str] = None,
    operator: Optional[str] = None,
    tag: Optional[str] = None,
    note: Optional[str] = None,
    note_exact: bool = False,
    kind: Optional[str] = None,
    route_mode: Optional[str] = None,
    created_after: Optional[datetime] = None,
    created_before: Optional[datetime] = None,
) -> Dict[str, Any]:
    """Return aggregate summary data for operator history queries."""
    await _ensure_operator_history_table()
    statement = _apply_history_filters(
        select(OperatorHistoryEntry),
        session_id=session_id,
        session_query=session_query,
        operator=operator,
        tag=tag,
        note=note,
        note_exact=note_exact,
        kind=kind,
        route_mode=route_mode,
        created_after=created_after,
        created_before=created_before,
    )

    async with AsyncSessionLocal() as session:
        result = await session.execute(statement)
        rows = list(result.scalars().all())

    session_summary: Dict[str, Dict[str, Any]] = {}
    kind_counts: Dict[str, int] = {}
    route_mode_counts: Dict[str, int] = {}

    for row in rows:
        kind_counts[row.kind] = kind_counts.get(row.kind, 0) + 1
        route_mode_counts[row.route_mode] = route_mode_counts.get(row.route_mode, 0) + 1
        session_entry = session_summary.setdefault(
            row.session_id,
            {
                "session_id": row.session_id,
                "entry_count": 0,
                "last_created_at": None,
            },
        )
        session_entry["entry_count"] += 1
        created_at_iso = row.created_at.isoformat() if row.created_at else None
        if created_at_iso and (
            session_entry["last_created_at"] is None
            or created_at_iso > session_entry["last_created_at"]
        ):
            session_entry["last_created_at"] = created_at_iso

    sessions = sorted(
        session_summary.values(),
        key=lambda item: (
            item["last_created_at"] is not None,
            item["last_created_at"] or "",
        ),
        reverse=True,
    )

    return {
        "session_id": session_id,
        "history_storage": "database",
        "total_entries": len(rows),
        "session_count": len(session_summary),
        "sessions": sessions,
        "kind_counts": kind_counts,
        "route_mode_counts": route_mode_counts,
        "filters": {
            "session_id": session_id,
            "session_query": session_query,
            "operator": operator,
            "tag": tag,
            "note": note,
            "note_exact": note_exact,
            "kind": kind,
            "route_mode": route_mode,
            "created_after": created_after.isoformat() if created_after else None,
            "created_before": created_before.isoformat() if created_before else None,
        },
    }


async def _export_operator_history(
    *,
    session_id: Optional[str] = None,
    session_query: Optional[str] = None,
    operator: Optional[str] = None,
    tag: Optional[str] = None,
    note: Optional[str] = None,
    note_exact: bool = False,
    kind: Optional[str] = None,
    route_mode: Optional[str] = None,
    created_after: Optional[datetime] = None,
    created_before: Optional[datetime] = None,
    sort_by: str = "created_at",
    sort_order: Literal["asc", "desc"] = "desc",
    export_format: Literal["json", "csv"] = "json",
) -> Any:
    """Export one filtered history result set as JSON or CSV."""
    payload = await _get_operator_history_listing(
        session_id=session_id,
        session_query=session_query,
        operator=operator,
        tag=tag,
        note=note,
        note_exact=note_exact,
        limit=max(1, GODOT_OPERATOR_HISTORY_MAX_ITEMS),
        offset=0,
        kind=kind,
        route_mode=route_mode,
        created_after=created_after,
        created_before=created_before,
        sort_by=sort_by,
        sort_order=sort_order,
    )
    filename = f"operator_history_export.{export_format}"
    if export_format == "json":
        return JSONResponse(
            content={"status": "success", **payload},
            headers={"Content-Disposition": f'attachment; filename="{filename}"'},
        )

    csv_lines = [
        "entry_id,session_id,operator,tag,note,kind,route_mode,created_at,payload_json"
    ]
    for item in payload["history"]:
        payload_json = json.dumps(item.get("payload") or {}, ensure_ascii=False).replace('"', '""')
        note_value = str(item.get("note") or "").replace('"', '""')
        csv_lines.append(
            ",".join(
                [
                    str(item.get("entry_id") or ""),
                    str(item.get("session_id") or ""),
                    str(item.get("operator") or ""),
                    str(item.get("tag") or ""),
                    f'"{note_value}"',
                    str(item.get("kind") or ""),
                    str((item.get("payload") or {}).get("route_mode") or ""),
                    str(item.get("created_at") or ""),
                    f'"{payload_json}"',
                ]
            )
        )
    return PlainTextResponse(
        "\n".join(csv_lines),
        media_type="text/csv; charset=utf-8",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
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
        self.last_instruction_runtime: Dict[str, Any] = {}
        self.simulated_circuit_config: Dict[str, Any] = {}
        self.command_history: List[Dict[str, Any]] = []
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

    async def configure_simulated_circuit(
        self,
        simulated_circuit: Dict[str, Any],
        *,
        operator: Optional[str] = None,
        tag: Optional[str] = None,
        note: Optional[str] = None,
        audit_user_id: Optional[int] = None,
        audit_username: Optional[str] = None,
        audit_source: Optional[str] = None,
    ) -> Dict[str, Any]:
        response = (
            await self._send_recv(
                {
                    "type": "configure_simulated_circuit",
                    "simulated_circuit": simulated_circuit,
                }
            )
            or {}
        )
        if response.get("status") == "error":
            self._set_state(
                "failed",
                failure_stage="simulated_circuit",
                failure_message=str(response.get("message") or response),
            )
        elif response:
            self.simulated_circuit_config = response.get(
                "simulated_circuit", simulated_circuit
            )
            await self._record_command_history(
                "simulated_circuit",
                {
                    "simulated_circuit": simulated_circuit,
                    "route_mode": "session_bridge",
                    "operator": operator,
                    "tag": tag,
                    "note": note,
                    "audit_user_id": audit_user_id,
                    "audit_username": audit_username,
                    "audit_source": audit_source,
                },
            )
            self._set_state("schema_ready" if self._schema else "connected")
        return response

    async def apply_instruction_set(
        self,
        instruction_set: Dict[str, Any],
        *,
        compatibility_params: Optional[Dict[str, Any]] = None,
        simulated_circuit_command_batch: Optional[List[Dict[str, Any]]] = None,
        operator: Optional[str] = None,
        tag: Optional[str] = None,
        note: Optional[str] = None,
        audit_user_id: Optional[int] = None,
        audit_username: Optional[str] = None,
        audit_source: Optional[str] = None,
    ) -> Dict[str, Any]:
        response = (
            await self._send_recv(
                {
                    "type": "instruction_set",
                    "instruction_set": instruction_set,
                    "compatibility_params": compatibility_params or {},
                    "simulated_circuit_command_batch": (
                        simulated_circuit_command_batch or []
                    ),
                }
            )
            or {}
        )
        if response.get("status") == "error":
            self._set_state(
                "failed",
                failure_stage="instruction_set",
                failure_message=str(response.get("message") or response),
            )
        elif response:
            self.last_instruction_runtime = {
                "instruction_set": instruction_set,
                "compatibility_params": compatibility_params or {},
                "simulated_circuit_command_batch": (
                    simulated_circuit_command_batch or []
                ),
                "response": response,
            }
            await self._record_command_history(
                "instruction_set",
                {
                    "instruction_set": instruction_set,
                    "compatibility_params": compatibility_params or {},
                    "simulated_circuit_command_batch": (
                        simulated_circuit_command_batch or []
                    ),
                    "route_mode": "session_bridge",
                    "operator": operator,
                    "tag": tag,
                    "note": note,
                    "audit_user_id": audit_user_id,
                    "audit_username": audit_username,
                    "audit_source": audit_source,
                },
            )
            self._set_state("running")
        return response

    async def _load_command_history(
        self,
        *,
        limit: int = GODOT_OPERATOR_HISTORY_MAX_ITEMS,
        offset: int = 0,
        kind: Optional[str] = None,
        route_mode: Optional[str] = None,
        created_after: Optional[datetime] = None,
        created_before: Optional[datetime] = None,
    ) -> List[Dict[str, Any]]:
        payload = await _get_operator_history_listing(
            session_id=self.session_id,
            limit=limit,
            offset=offset,
            kind=kind,
            route_mode=route_mode,
            created_after=created_after,
            created_before=created_before,
        )
        return payload["history"]

    async def _count_command_history(
        self,
        *,
        kind: Optional[str] = None,
        route_mode: Optional[str] = None,
        created_after: Optional[datetime] = None,
        created_before: Optional[datetime] = None,
    ) -> int:
        payload = await _get_operator_history_listing(
            session_id=self.session_id,
            limit=1,
            offset=0,
            kind=kind,
            route_mode=route_mode,
            created_after=created_after,
            created_before=created_before,
        )
        return int(payload["history_count"])

    async def _record_command_history(self, kind: str, payload: Dict[str, Any]) -> None:
        await _ensure_operator_history_table()
        operator = _normalize_optional_text(str(payload.get("operator") or ""))
        tag = _normalize_optional_text(str(payload.get("tag") or ""))
        note = _normalize_optional_text(str(payload.get("note") or ""))
        audit_user_id = payload.get("audit_user_id")
        audit_username = _normalize_optional_text(
            str(payload.get("audit_username") or "")
        )
        audit_source = _normalize_optional_text(str(payload.get("audit_source") or ""))
        if operator is None and audit_username:
            operator = audit_username
        entry = {
            "entry_id": f"{kind}-{int(time.time() * 1000)}-{len(self.command_history)}",
            "schema_version": "1.0",
            "kind": kind,
            "operator": operator,
            "tag": tag,
            "note": note,
            "audit_user_id": audit_user_id,
            "audit_username": audit_username,
            "audit_source": audit_source,
            "created_at": datetime.now().isoformat(),
            "session_id": self.session_id,
            "payload": payload,
        }
        route_mode = str(payload.get("route_mode") or "session_bridge")
        async with AsyncSessionLocal() as session:
            session.add(
                OperatorHistoryEntry(
                    entry_id=entry["entry_id"],
                    session_id=self.session_id,
                    kind=kind,
                    route_mode=route_mode,
                    operator=operator,
                    tag=tag,
                    note=note,
                    audit_user_id=int(audit_user_id)
                    if isinstance(audit_user_id, int)
                    else None,
                    audit_username=audit_username,
                    audit_source=audit_source,
                    payload=payload,
                    created_at=datetime.fromisoformat(entry["created_at"]),
                )
            )
            await session.commit()

            count_result = await session.execute(
                select(func.count(OperatorHistoryEntry.id)).where(
                    OperatorHistoryEntry.session_id == self.session_id
                )
            )
            total_count = int(count_result.scalar_one() or 0)
            overflow = max(0, total_count - GODOT_OPERATOR_HISTORY_MAX_ITEMS)
            if overflow > 0:
                overflow_result = await session.execute(
                    select(OperatorHistoryEntry.id)
                    .where(OperatorHistoryEntry.session_id == self.session_id)
                    .order_by(
                        OperatorHistoryEntry.created_at.asc(),
                        OperatorHistoryEntry.id.asc(),
                    )
                    .limit(overflow)
                )
                overflow_ids = list(overflow_result.scalars().all())
                if overflow_ids:
                    await session.execute(
                        delete(OperatorHistoryEntry).where(
                            OperatorHistoryEntry.id.in_(overflow_ids)
                        )
                    )
                    await session.commit()
        self.command_history = await self._load_command_history()

    async def get_history_payload(
        self,
        limit: int = 10,
        offset: int = 0,
        *,
        session_query: Optional[str] = None,
        operator: Optional[str] = None,
        tag: Optional[str] = None,
        note: Optional[str] = None,
        note_exact: bool = False,
        kind: Optional[str] = None,
        route_mode: Optional[str] = None,
        created_after: Optional[datetime] = None,
        created_before: Optional[datetime] = None,
        sort_by: str = "created_at",
        sort_order: Literal["asc", "desc"] = "desc",
    ) -> Dict[str, Any]:
        safe_limit = max(1, limit)
        safe_offset = max(0, offset)
        payload = await _get_operator_history_listing(
            session_id=self.session_id,
            session_query=session_query,
            operator=operator,
            tag=tag,
            note=note,
            note_exact=note_exact,
            limit=safe_limit,
            offset=safe_offset,
            kind=kind,
            route_mode=route_mode,
            created_after=created_after,
            created_before=created_before,
            sort_by=sort_by,
            sort_order=sort_order,
        )
        self.command_history = await self._load_command_history()
        return payload

    async def clear_history(self) -> Dict[str, Any]:
        await _ensure_operator_history_table()
        async with AsyncSessionLocal() as session:
            await session.execute(
                delete(OperatorHistoryEntry).where(
                    OperatorHistoryEntry.session_id == self.session_id
                )
            )
            await session.commit()
        self.command_history = []
        return {
            "session_id": self.session_id,
            "history": [],
            "history_count": 0,
            "offset": 0,
            "limit": DEFAULT_OPERATOR_HISTORY_PAGE_SIZE,
            "has_more": False,
            "history_storage": "database",
        }

    async def replay_history_entry(
        self, entry_id: Optional[str] = None
    ) -> Dict[str, Any]:
        self.command_history = await self._load_command_history()
        entry: Optional[Dict[str, Any]]
        if entry_id is None:
            entry = self.command_history[0] if self.command_history else None
        else:
            entry = next(
                (
                    item
                    for item in self.command_history
                    if item.get("entry_id") == entry_id
                ),
                None,
            )
        if entry is None:
            return {"status": "error", "message": "History entry not found."}

        payload = entry.get("payload") or {}
        if entry.get("kind") == "instruction_set":
            result = await self.apply_instruction_set(
                payload.get("instruction_set") or {},
                compatibility_params=payload.get("compatibility_params") or {},
                simulated_circuit_command_batch=(
                    payload.get("simulated_circuit_command_batch") or []
                ),
            )
        elif entry.get("kind") == "simulated_circuit":
            result = await self.configure_simulated_circuit(
                payload.get("simulated_circuit") or {}
            )
        else:
            return {
                "status": "error",
                "message": f"Unsupported history entry kind: {entry.get('kind')}",
            }

        return {
            "status": "success" if result.get("status") != "error" else "error",
            "entry": entry,
            "dispatch_result": result,
        }

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
            "last_instruction_runtime": self.last_instruction_runtime,
            "simulated_circuit_config": self.simulated_circuit_config,
            "history_count": len(self.command_history),
            "history_storage": "database",
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

    @router.get("/api/godot/history")
    async def get_operator_history(
        session_id: Optional[str] = None,
        session_query: Optional[str] = None,
        operator: Optional[str] = None,
        tag: Optional[str] = None,
        note: Optional[str] = None,
        note_exact: Optional[bool] = None,
        limit: int = DEFAULT_OPERATOR_HISTORY_PAGE_SIZE,
        offset: int = 0,
        kind: Optional[str] = None,
        route_mode: Optional[str] = None,
        created_after: Optional[str] = None,
        created_before: Optional[str] = None,
        sort_by: Optional[str] = None,
        sort_order: Optional[str] = None,
    ):
        resolved_session_query = _normalize_optional_text(session_query)
        resolved_operator = _normalize_optional_text(operator)
        resolved_tag = _normalize_optional_text(tag)
        resolved_note = _normalize_optional_text(note)
        resolved_note_exact = _parse_note_exact(note_exact)
        resolved_created_after = _parse_optional_iso_datetime(
            created_after, "created_after"
        )
        resolved_created_before = _parse_optional_iso_datetime(
            created_before, "created_before"
        )
        resolved_sort_by = _parse_history_sort_field(sort_by)
        resolved_sort_order = _parse_history_sort_order(sort_order)
        return {
            "status": "success",
            **(
                await _get_operator_history_listing(
                    session_id=session_id,
                    session_query=resolved_session_query,
                    operator=resolved_operator,
                    tag=resolved_tag,
                    note=resolved_note,
                    note_exact=resolved_note_exact,
                    limit=limit,
                    offset=offset,
                    kind=kind,
                    route_mode=route_mode,
                    created_after=resolved_created_after,
                    created_before=resolved_created_before,
                    sort_by=resolved_sort_by,
                    sort_order=resolved_sort_order,
                )
            ),
        }

    @router.get("/api/godot/history/export")
    async def export_operator_history(
        session_id: Optional[str] = None,
        session_query: Optional[str] = None,
        operator: Optional[str] = None,
        tag: Optional[str] = None,
        note: Optional[str] = None,
        note_exact: Optional[bool] = None,
        kind: Optional[str] = None,
        route_mode: Optional[str] = None,
        created_after: Optional[str] = None,
        created_before: Optional[str] = None,
        sort_by: Optional[str] = None,
        sort_order: Optional[str] = None,
        format: str = "json",
    ):
        resolved_session_query = _normalize_optional_text(session_query)
        resolved_operator = _normalize_optional_text(operator)
        resolved_tag = _normalize_optional_text(tag)
        resolved_note = _normalize_optional_text(note)
        resolved_note_exact = _parse_note_exact(note_exact)
        resolved_created_after = _parse_optional_iso_datetime(
            created_after, "created_after"
        )
        resolved_created_before = _parse_optional_iso_datetime(
            created_before, "created_before"
        )
        resolved_sort_by = _parse_history_sort_field(sort_by)
        resolved_sort_order = _parse_history_sort_order(sort_order)
        normalized_format = (_normalize_optional_text(format) or "json").lower()
        if normalized_format not in {"json", "csv"}:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid format '{format}'. Expected 'json' or 'csv'.",
            )
        return await _export_operator_history(
            session_id=session_id,
            session_query=resolved_session_query,
            operator=resolved_operator,
            tag=resolved_tag,
            note=resolved_note,
            note_exact=resolved_note_exact,
            kind=kind,
            route_mode=route_mode,
            created_after=resolved_created_after,
            created_before=resolved_created_before,
            sort_by=resolved_sort_by,
            sort_order=resolved_sort_order,
            export_format=normalized_format,
        )

    @router.get("/api/godot/history/summary")
    async def get_operator_history_summary(
        session_id: Optional[str] = None,
        session_query: Optional[str] = None,
        operator: Optional[str] = None,
        tag: Optional[str] = None,
        note: Optional[str] = None,
        note_exact: Optional[bool] = None,
        kind: Optional[str] = None,
        route_mode: Optional[str] = None,
        created_after: Optional[str] = None,
        created_before: Optional[str] = None,
    ):
        resolved_session_query = _normalize_optional_text(session_query)
        resolved_operator = _normalize_optional_text(operator)
        resolved_tag = _normalize_optional_text(tag)
        resolved_note = _normalize_optional_text(note)
        resolved_note_exact = _parse_note_exact(note_exact)
        resolved_created_after = _parse_optional_iso_datetime(
            created_after, "created_after"
        )
        resolved_created_before = _parse_optional_iso_datetime(
            created_before, "created_before"
        )
        return {
            "status": "success",
            **(
                await _get_operator_history_summary(
                    session_id=session_id,
                    session_query=resolved_session_query,
                    operator=resolved_operator,
                    tag=resolved_tag,
                    note=resolved_note,
                    note_exact=resolved_note_exact,
                    kind=kind,
                    route_mode=route_mode,
                    created_after=resolved_created_after,
                    created_before=resolved_created_before,
                )
            ),
        }

    @router.get("/api/godot/{session_id}/status")
    async def get_session_status(session_id: str):
        bridge = manager.get_session(session_id)
        if bridge is None:
            return disconnected_session_status(session_id)
        return bridge.get_status_payload()

    @router.get("/api/godot/{session_id}/history")
    async def get_session_history(
        session_id: str,
        limit: int = DEFAULT_OPERATOR_HISTORY_PAGE_SIZE,
        offset: int = 0,
        session_query: Optional[str] = None,
        operator: Optional[str] = None,
        tag: Optional[str] = None,
        note: Optional[str] = None,
        note_exact: Optional[bool] = None,
        kind: Optional[str] = None,
        route_mode: Optional[str] = None,
        created_after: Optional[str] = None,
        created_before: Optional[str] = None,
        sort_by: Optional[str] = None,
        sort_order: Optional[str] = None,
    ):
        bridge = manager.get_or_create(session_id)
        resolved_session_query = _normalize_optional_text(session_query)
        resolved_operator = _normalize_optional_text(operator)
        resolved_tag = _normalize_optional_text(tag)
        resolved_note = _normalize_optional_text(note)
        resolved_note_exact = _parse_note_exact(note_exact)
        resolved_created_after = _parse_optional_iso_datetime(
            created_after, "created_after"
        )
        resolved_created_before = _parse_optional_iso_datetime(
            created_before, "created_before"
        )
        resolved_sort_by = _parse_history_sort_field(sort_by)
        resolved_sort_order = _parse_history_sort_order(sort_order)
        return {
            "status": "success",
            **(
                await bridge.get_history_payload(
                    limit=limit,
                    offset=offset,
                    session_query=resolved_session_query,
                    operator=resolved_operator,
                    tag=resolved_tag,
                    note=resolved_note,
                    note_exact=resolved_note_exact,
                    kind=kind,
                    route_mode=route_mode,
                    created_after=resolved_created_after,
                    created_before=resolved_created_before,
                    sort_by=resolved_sort_by,
                    sort_order=resolved_sort_order,
                )
            ),
        }

    @router.post("/api/godot/{session_id}/history/clear")
    async def clear_session_history(session_id: str):
        bridge = manager.get_or_create(session_id)
        return {
            "status": "success",
            **(await bridge.clear_history()),
        }

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

    @router.post("/api/godot/{session_id}/simulated-circuit")
    async def configure_session_simulated_circuit(
        session_id: str, payload: Dict[str, Any], request: Request
    ):
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
        audit_identity = await _resolve_optional_audit_identity(
            request.headers.get("Authorization")
        )
        simulated_circuit = payload.get("simulated_circuit") or payload
        result = await bridge.configure_simulated_circuit(
            simulated_circuit,
            operator=_normalize_optional_text(str(payload.get("operator") or "")),
            tag=_normalize_optional_text(str(payload.get("tag") or "")),
            note=_normalize_optional_text(str(payload.get("note") or "")),
            audit_user_id=audit_identity["audit_user_id"],
            audit_username=audit_identity["audit_username"],
            audit_source=audit_identity["audit_source"],
        )
        return {
            "status": "error" if result.get("status") == "error" else "success",
            "simulated_circuit": result.get(
                "simulated_circuit", bridge.simulated_circuit_config
            ),
            "operator": _normalize_optional_text(str(payload.get("operator") or "")),
            "tag": _normalize_optional_text(str(payload.get("tag") or "")),
            "note": _normalize_optional_text(str(payload.get("note") or "")),
            "audit_identity": audit_identity,
            "dispatch_result": result,
            "session": bridge.get_status_payload(),
            "session_state": bridge.session_state,
        }

    @router.post("/api/godot/{session_id}/instruction-set")
    async def apply_session_instruction_set(
        session_id: str, payload: Dict[str, Any], request: Request
    ):
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
        audit_identity = await _resolve_optional_audit_identity(
            request.headers.get("Authorization")
        )
        instruction_set = payload.get("instruction_set") or {}
        compatibility_params = payload.get("compatibility_params") or {}
        command_batch = payload.get("simulated_circuit_command_batch") or []
        result = await bridge.apply_instruction_set(
            instruction_set,
            compatibility_params=compatibility_params,
            simulated_circuit_command_batch=command_batch,
            operator=_normalize_optional_text(str(payload.get("operator") or "")),
            tag=_normalize_optional_text(str(payload.get("tag") or "")),
            note=_normalize_optional_text(str(payload.get("note") or "")),
            audit_user_id=audit_identity["audit_user_id"],
            audit_username=audit_identity["audit_username"],
            audit_source=audit_identity["audit_source"],
        )
        return {
            "status": "error" if result.get("status") == "error" else "success",
            "instruction_set": instruction_set,
            "compatibility_params": compatibility_params,
            "simulated_circuit_command_batch": command_batch,
            "operator": _normalize_optional_text(str(payload.get("operator") or "")),
            "tag": _normalize_optional_text(str(payload.get("tag") or "")),
            "note": _normalize_optional_text(str(payload.get("note") or "")),
            "audit_identity": audit_identity,
            "dispatch_result": result,
            "session": bridge.get_status_payload(),
            "session_state": bridge.session_state,
        }

    @router.post("/api/godot/{session_id}/history/replay")
    async def replay_session_history(
        session_id: str, payload: Optional[Dict[str, Any]] = None
    ):
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
        payload = payload or {}
        result = await bridge.replay_history_entry(payload.get("entry_id"))
        return {
            **result,
            "session": bridge.get_status_payload(),
            "session_state": bridge.session_state,
        }

    return router
