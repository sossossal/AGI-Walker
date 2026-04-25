"""
Workflows API for AGI-Walker Web Panel.

Exposes workflow discovery plus a background execution control plane with:
- explicit execution strategy and output-root routing
- in-process run registry for polling recent runs
- background worker subprocesses so cancel/timeout can terminate execution
- artifact and log download endpoints for completed runs
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import subprocess
import threading
import time
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional
from urllib.parse import quote
from uuid import uuid4

from fastapi import APIRouter, Body, Depends, HTTPException, Request, Response
from fastapi.responses import FileResponse, PlainTextResponse, StreamingResponse
from pydantic import BaseModel, Field
from sqlalchemy import select

from agi_walker.core.api.workflow_contracts import (
    WORKFLOW_CONTRACT_VERSION,
    validate_export_result,
    validate_robot_config,
    validate_workflow_step_artifact,
)
from agi_walker.workflow_orchestrator import get_workflow_orchestrator
from web_panel.auth_api import get_current_user
from web_panel.core_api import DEFAULT_GODOT_SESSION_ID
from web_panel.database import AsyncSessionLocal
from web_panel.models import User, WorkflowRun

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/workflows", tags=["workflows"])
WEB_PANEL_ENV_FILE_ENV_VAR = "AGI_WALKER_WEB_ENV_FILE"


def _strip_env_value(raw_value: str) -> str:
    """Normalize one env-file value, removing surrounding quotes when present."""
    value = raw_value.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        return value[1:-1]
    return value


def _resolve_web_panel_env_file() -> Optional[Path]:
    """Resolve the env file used for workflow Web runtime defaults."""
    explicit_path = os.getenv(WEB_PANEL_ENV_FILE_ENV_VAR)
    candidates: List[Path] = []

    if explicit_path:
        candidates.append(Path(explicit_path))
    else:
        candidates.extend(
            [
                Path("deployment") / "web_panel.env",
                Path("deployment") / "web_panel.env.example",
            ]
        )

    for candidate in candidates:
        if candidate.exists() and candidate.is_file():
            return candidate
    return None


def _load_web_panel_env_file(env_path: Path, *, override: bool = False) -> None:
    """Load simple KEY=VALUE assignments from one env file into os.environ."""
    try:
        lines = env_path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return

    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue

        key, value = stripped.split("=", 1)
        normalized_key = key.strip()
        if not normalized_key:
            continue
        if not override and normalized_key in os.environ:
            continue

        os.environ[normalized_key] = _strip_env_value(value)


def _read_int_env(name: str, default: int) -> int:
    """Read one integer env var, falling back to the provided default."""
    raw_value = os.getenv(name)
    if raw_value is None:
        return default

    try:
        return int(raw_value)
    except ValueError:
        return default


def _read_path_env(name: str, default: Path) -> Path:
    """Read one filesystem path env var, falling back to the provided default."""
    raw_value = os.getenv(name)
    if raw_value is None:
        return default

    normalized = raw_value.strip()
    if not normalized:
        return default
    return Path(normalized)


WEB_PANEL_ENV_FILE = _resolve_web_panel_env_file()
if WEB_PANEL_ENV_FILE is not None:
    _load_web_panel_env_file(WEB_PANEL_ENV_FILE, override=False)

DEFAULT_WEB_OUTPUT_ROOT = _read_path_env(
    "AGI_WALKER_WEB_OUTPUT_ROOT",
    Path("test_env") / "web_workflow_runs",
)
WORKFLOW_ARCHIVE_ROOT = _read_path_env(
    "AGI_WALKER_WEB_ARCHIVE_ROOT",
    Path(".output") / "web_workflow_archive",
)
MAX_HISTORY_ITEMS = 50
DEFAULT_RUNS_PAGE_SIZE = _read_int_env("AGI_WALKER_WEB_RUNS_PAGE_SIZE", 20)
MAX_RUNS_PAGE_SIZE = _read_int_env("AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE", 100)
ARCHIVE_RETENTION_MAX_RUNS = _read_int_env("AGI_WALKER_WEB_ARCHIVE_MAX_RUNS", 200)
ARCHIVE_RETENTION_MAX_AGE_DAYS = _read_int_env(
    "AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS", 30
)
VALID_EXECUTION_STRATEGIES = {"resume", "force"}
TERMINAL_RUN_STATUSES = {"completed", "failed", "cancelled", "timed_out"}
RUN_MONITOR_INTERVAL_SECONDS = 0.2
RUN_EVENTS_WAIT_TIMEOUT_SECONDS = 15.0
MAX_RUN_EVENTS = 200
MAX_LIVE_LOG_TAIL_LINES = 120
VALID_RUN_SCOPES = {"active", "archive", "all"}


def _run_async(coro: Any) -> Any:
    """Helper to run an async coroutine from a synchronous thread using the app's event loop."""
    from web_panel.server import app

    loop = getattr(app.state, "server_loop", None)
    if loop is None or loop.is_closed():
        # Fallback if loop is not yet available (e.g. during startup)
        return asyncio.run(coro)

    future = asyncio.run_coroutine_threadsafe(coro, loop)
    return future.result()


# In-memory execution tracking for the current web server process (Legacy V1)
execution_history: List[Dict[str, Any]] = []
execution_lock = threading.Lock()
run_events_condition = threading.Condition(execution_lock)
run_event_history: Dict[str, List[Dict[str, Any]]] = {}
run_event_counters: Dict[str, int] = {}
active_run_processes: Dict[str, Dict[str, Any]] = {}


class WorkflowRunRequest(BaseModel):
    """Request payload for running a workflow from the web panel."""

    use_real: bool = True
    execution_strategy: str = "force"
    output_root: Optional[str] = None
    timeout_seconds: Optional[float] = Field(default=None, ge=0.1)
    parameters: Dict[str, Any] = Field(default_factory=dict)


class WorkflowArtifactGodotLoadRequest(BaseModel):
    """Request payload for loading one workflow artifact into Godot."""

    transport_mode: str = "session_bridge"
    session_id: str = DEFAULT_GODOT_SESSION_ID
    auto_connect: bool = True
    host: str = "127.0.0.1"
    port: int = 9999
    launch_if_needed: bool = True
    scene: str = "demo_generated_biped.tscn"
    godot_exe: str = ""
    headless: bool = True
    wait_for_tcp_seconds: float = Field(default=10.0, ge=0.1, le=60.0)


class WorkflowRunGodotSyncRequest(WorkflowArtifactGodotLoadRequest):
    """Request payload for syncing the recommended workflow artifact into Godot."""

    artifact_index: Optional[int] = Field(default=None, ge=0)


def _now() -> datetime:
    """Return the current local datetime."""
    return datetime.now()


def _build_output_root(workflow_name: str, requested_root: Optional[str]) -> str:
    """Create or normalize the output root for a workflow run."""
    if requested_root:
        output_root = Path(requested_root)
    else:
        output_root = DEFAULT_WEB_OUTPUT_ROOT / workflow_name / uuid4().hex

    output_root.mkdir(parents=True, exist_ok=True)
    return str(output_root)


def _validate_execution_strategy(execution_strategy: str) -> str:
    """Validate the requested execution strategy."""
    normalized = execution_strategy.strip().lower()
    if normalized not in VALID_EXECUTION_STRATEGIES:
        valid = ", ".join(sorted(VALID_EXECUTION_STRATEGIES))
        raise HTTPException(
            status_code=400,
            detail=f"Invalid execution_strategy '{execution_strategy}'. Expected one of: {valid}",
        )
    return normalized


def _list_legacy_logs() -> List[str]:
    """List workflow logs from the legacy root .output directory."""
    log_dir = Path(".output")
    if not log_dir.exists():
        return []

    logs = [
        entry.name
        for entry in log_dir.iterdir()
        if entry.is_file()
        and entry.name.startswith("workflow_log_")
        and entry.suffix == ".json"
    ]
    return sorted(logs, reverse=True)


def _find_run_record_unlocked(run_id: str) -> Optional[Dict[str, Any]]:
    """Find a run record by ID without taking the global lock."""
    for run in execution_history:
        if run["run_id"] == run_id:
            return run
    return None


def _snapshot_json(value: Any) -> Any:
    """Create a JSON-safe deep snapshot for event payloads."""
    return json.loads(json.dumps(value))


def _copy_run_record(run: Dict[str, Any], run_source: str) -> Dict[str, Any]:
    """Copy one run record and annotate where it was loaded from."""
    copied = _snapshot_json(run)
    copied["run_source"] = run_source
    return copied


def _run_source_rank(run_source: str) -> int:
    """Return the precedence used when two records are otherwise equally fresh."""
    return {
        "active": 3,
        "archive": 2,
        "database": 1,
    }.get(run_source, 0)


def _publish_run_event_unlocked(
    run_id: str, event_type: str, record: Dict[str, Any]
) -> None:
    """Append one run event and notify all stream listeners."""
    event_id = run_event_counters.get(run_id, 0) + 1
    run_event_counters[run_id] = event_id

    event = {
        "event_id": event_id,
        "event_type": event_type,
        "timestamp": _now().isoformat(),
        "run": _snapshot_json(record),
    }
    history = run_event_history.setdefault(run_id, [])
    history.append(event)
    if len(history) > MAX_RUN_EVENTS:
        del history[:-MAX_RUN_EVENTS]
    run_events_condition.notify_all()


def _get_run_events_since_unlocked(
    run_id: str, after_event_id: int
) -> List[Dict[str, Any]]:
    """Return all cached run events strictly newer than the given event id."""
    return [
        _snapshot_json(event)
        for event in run_event_history.get(run_id, [])
        if event["event_id"] > after_event_id
    ]


def _wait_for_run_events(
    run_id: str, after_event_id: int, timeout_seconds: float
) -> List[Dict[str, Any]]:
    """Block until at least one newer event exists or the timeout expires."""
    deadline = time.monotonic() + timeout_seconds
    with run_events_condition:
        while True:
            events = _get_run_events_since_unlocked(run_id, after_event_id)
            if events:
                return events

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return []
            run_events_condition.wait(timeout=remaining)


def _format_sse_event(event: Dict[str, Any]) -> str:
    """Format one cached event as a Server-Sent Events frame."""
    payload = {
        "event_id": event["event_id"],
        "event_type": event["event_type"],
        "timestamp": event["timestamp"],
        "run": event["run"],
    }
    data = json.dumps(payload, ensure_ascii=False)
    return f"id: {event['event_id']}\nevent: workflow_run_event\ndata: {data}\n\n"


async def _get_run_record(run_id: str) -> Dict[str, Any]:
    """Find one run record by ID, reconciling live memory, archive, and database freshness."""
    candidates: List[Dict[str, Any]] = []

    with execution_lock:
        record = _find_run_record_unlocked(run_id)
        if record is not None:
            candidates.append(_copy_run_record(record, "active"))

    archived_record = _load_archived_run_record(run_id)
    if archived_record is not None:
        candidates.append(_copy_run_record(archived_record, "archive"))

    async with AsyncSessionLocal() as session:
        result = await session.execute(
            select(WorkflowRun).where(WorkflowRun.run_id == run_id)
        )
        db_run = result.scalar_one_or_none()
        if db_run:
            candidates.append(_copy_run_record(db_run.to_dict(), "database"))

    if candidates:
        selected = candidates[0]
        for candidate in candidates[1:]:
            selected = _prefer_run_record(selected, candidate)
        return selected

    raise HTTPException(status_code=404, detail=f"Workflow run '{run_id}' not found")


def _archive_run_path(run_id: str) -> Path:
    """Return the archive file path for one workflow run."""
    return WORKFLOW_ARCHIVE_ROOT / f"{run_id}.json"


def _get_archive_retention_policy() -> Dict[str, Any]:
    """Return the normalized archive retention policy exposed to callers."""
    return {
        "max_runs": (
            ARCHIVE_RETENTION_MAX_RUNS if ARCHIVE_RETENTION_MAX_RUNS > 0 else None
        ),
        "max_age_days": (
            ARCHIVE_RETENTION_MAX_AGE_DAYS
            if ARCHIVE_RETENTION_MAX_AGE_DAYS > 0
            else None
        ),
        "default_output_root": str(DEFAULT_WEB_OUTPUT_ROOT),
        "archive_root": str(WORKFLOW_ARCHIVE_ROOT),
        "env_file": str(WEB_PANEL_ENV_FILE) if WEB_PANEL_ENV_FILE is not None else None,
    }


def _get_protected_archive_run_ids() -> set[str]:
    """Return run IDs that should not be pruned from archive retention."""
    with execution_lock:
        protected_run_ids = {
            run["run_id"]
            for run in execution_history
            if run.get("status") not in TERMINAL_RUN_STATUSES
        }
        protected_run_ids.update(active_run_processes)
    return protected_run_ids


def _delete_archive_file(path: Path) -> bool:
    """Delete one archived run file if it still exists."""
    try:
        path.unlink()
    except FileNotFoundError:
        return False
    except OSError:
        return False
    return True


def _enforce_archive_retention_policy() -> Dict[str, Any]:
    """Prune archived run records according to max-age and max-count policy."""
    policy = _get_archive_retention_policy()
    if not WORKFLOW_ARCHIVE_ROOT.exists():
        return {
            "removed_run_ids": [],
            "remaining_files": 0,
            "protected_run_ids": [],
        }

    protected_run_ids = _get_protected_archive_run_ids()
    archive_entries: List[tuple[Path, float]] = []
    for path in WORKFLOW_ARCHIVE_ROOT.glob("*.json"):
        try:
            archive_entries.append((path, path.stat().st_mtime))
        except OSError:
            continue

    removed_run_ids: List[str] = []

    max_age_days = policy["max_age_days"]
    if max_age_days:
        cutoff_timestamp = (_now() - timedelta(days=max_age_days)).timestamp()
        for path, modified_at in list(archive_entries):
            if modified_at >= cutoff_timestamp or path.stem in protected_run_ids:
                continue
            if _delete_archive_file(path):
                removed_run_ids.append(path.stem)
        archive_entries = [
            (path, path.stat().st_mtime) for path, _ in archive_entries if path.exists()
        ]

    max_runs = policy["max_runs"]
    if max_runs and len(archive_entries) > max_runs:
        archive_entries.sort(key=lambda item: item[1], reverse=True)
        overflow = len(archive_entries) - max_runs
        removable_entries = [
            (path, modified_at)
            for path, modified_at in reversed(archive_entries)
            if path.stem not in protected_run_ids
        ]
        for path, _ in removable_entries[:overflow]:
            if _delete_archive_file(path):
                removed_run_ids.append(path.stem)
        archive_entries = [
            (path, path.stat().st_mtime) for path, _ in archive_entries if path.exists()
        ]

    return {
        "removed_run_ids": removed_run_ids,
        "remaining_files": len(archive_entries),
        "protected_run_ids": sorted(protected_run_ids),
    }


def _archive_run_record(record: Dict[str, Any]) -> None:
    """Persist one run record to the archive directory."""
    WORKFLOW_ARCHIVE_ROOT.mkdir(parents=True, exist_ok=True)
    archive_path = _archive_run_path(record["run_id"])
    archive_path.write_text(
        json.dumps(record, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    _enforce_archive_retention_policy()


def _load_archived_run_record(run_id: str) -> Optional[Dict[str, Any]]:
    """Load one archived run record from disk if it exists."""
    archive_path = _archive_run_path(run_id)
    if not archive_path.exists():
        return None

    try:
        record = json.loads(archive_path.read_text(encoding="utf-8"))
    except Exception:
        return None
    record["run_source"] = "archive"
    return record


def _load_all_archived_run_records() -> List[Dict[str, Any]]:
    """Load all archived run records from disk, skipping unreadable entries."""
    if not WORKFLOW_ARCHIVE_ROOT.exists():
        return []

    records: List[Dict[str, Any]] = []
    for archive_path in WORKFLOW_ARCHIVE_ROOT.glob("*.json"):
        try:
            payload = json.loads(archive_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(payload, dict) or "run_id" not in payload:
            continue
        payload["run_source"] = "archive"
        records.append(payload)
    return records


def _record_timestamp(record: Dict[str, Any]) -> datetime:
    """Return the best-effort timestamp used for sorting one run record."""
    for key in (
        "created_at",
        "started_at",
        "finished_at",
        "progress_updated_at",
        "live_log_updated_at",
    ):
        raw_value = record.get(key)
        if not raw_value:
            continue
        if isinstance(raw_value, datetime):
            return raw_value
        if isinstance(raw_value, str):
            try:
                return datetime.fromisoformat(raw_value)
            except ValueError:
                continue
    return datetime.min


def _record_filter_timestamp(record: Dict[str, Any]) -> datetime:
    """Return the best-effort timestamp used for date-range filtering."""
    for key in ("started_at", "finished_at", "created_at"):
        raw_value = record.get(key)
        if not raw_value:
            continue
        if isinstance(raw_value, datetime):
            return raw_value
        if isinstance(raw_value, str):
            try:
                return datetime.fromisoformat(raw_value)
            except ValueError:
                continue
    return _record_timestamp(record)


def _prefer_run_record(
    existing: Dict[str, Any], candidate: Dict[str, Any]
) -> Dict[str, Any]:
    """Pick the fresher run record across active memory, archive, and database sources."""
    existing_timestamp = _record_timestamp(existing)
    candidate_timestamp = _record_timestamp(candidate)

    if candidate_timestamp > existing_timestamp:
        return candidate
    if existing_timestamp > candidate_timestamp:
        return existing

    existing_terminal = existing.get("status") in TERMINAL_RUN_STATUSES
    candidate_terminal = candidate.get("status") in TERMINAL_RUN_STATUSES
    if candidate_terminal != existing_terminal:
        return candidate if candidate_terminal else existing

    return (
        candidate
        if _run_source_rank(candidate.get("run_source", ""))
        > _run_source_rank(existing.get("run_source", ""))
        else existing
    )


def _record_matches_filters(
    record: Dict[str, Any],
    *,
    workflow_name: Optional[str],
    status: Optional[str],
    mode: Optional[str],
    text: Optional[str],
    parsed_date_from: Optional[datetime],
    parsed_date_to: Optional[datetime],
    only_failures: bool,
) -> bool:
    """Return whether one run record matches the public filtering parameters."""
    if workflow_name and record.get("workflow_name") != workflow_name:
        return False
    if status and record.get("status") != status:
        return False
    if mode and record.get("mode") != mode:
        return False
    if only_failures and record.get("status") not in {
        "failed",
        "timed_out",
        "cancelled",
    }:
        return False

    filter_timestamp = _record_filter_timestamp(record)
    if parsed_date_from and filter_timestamp < parsed_date_from:
        return False
    if parsed_date_to and filter_timestamp > parsed_date_to:
        return False

    if text:
        haystack = " ".join(
            str(record.get(field) or "")
            for field in (
                "run_id",
                "workflow_name",
                "status_detail",
                "message",
                "failed_step_name",
                "failed_step_error",
                "diagnostic_summary",
                "current_step_name",
                "output_root",
            )
        ).lower()
        if text.lower() not in haystack:
            return False

    return True


async def _load_database_run_records() -> List[Dict[str, Any]]:
    """Load persisted run records from the database as a fallback listing source."""
    async with AsyncSessionLocal() as session:
        result = await session.execute(select(WorkflowRun))
        records = [row.to_dict() for row in result.scalars().all()]
    for record in records:
        record["run_source"] = "database"
    return records


async def _collect_run_records(scope: str) -> List[Dict[str, Any]]:
    """Collect run records for one listing scope, preferring live memory and archive data."""
    normalized_scope = _normalize_run_scope(scope)
    with execution_lock:
        active_records = [_copy_run_record(run, "active") for run in execution_history]

    archived_records = [
        _copy_run_record(run, "archive") for run in _load_all_archived_run_records()
    ]
    database_records = await _load_database_run_records()
    merged: Dict[str, Dict[str, Any]] = {}

    candidate_sets: List[List[Dict[str, Any]]] = []
    if normalized_scope in {"active", "all"}:
        candidate_sets.append(active_records)
    if normalized_scope in {"archive", "all"}:
        candidate_sets.append(archived_records)
    candidate_sets.append(database_records)

    for candidate_group in candidate_sets:
        for candidate in candidate_group:
            run_id = candidate["run_id"]
            existing = merged.get(run_id)
            merged[run_id] = (
                candidate
                if existing is None
                else _prefer_run_record(existing, candidate)
            )

    records = list(merged.values())
    if normalized_scope == "active":
        return [
            run for run in records if run.get("status") not in TERMINAL_RUN_STATUSES
        ]
    if normalized_scope == "archive":
        return [run for run in records if run.get("status") in TERMINAL_RUN_STATUSES]
    return records


async def _store_run_record(
    record: Dict[str, Any], user_id: Optional[int] = None
) -> None:
    """Persist a run record to database and keep in-memory history for SSE (V2.0)."""
    # 1. Persist to Database
    try:
        async with AsyncSessionLocal() as session:
            started_at = (
                datetime.fromisoformat(record["started_at"])
                if record.get("started_at")
                else None
            )
            finished_at = (
                datetime.fromisoformat(record["finished_at"])
                if record.get("finished_at")
                else None
            )
            db_run = WorkflowRun(
                run_id=record["run_id"],
                user_id=user_id,
                workflow_name=record["workflow_name"],
                status=record["status"],
                mode=record["mode"],
                execution_strategy=record["execution_strategy"],
                output_root=record["output_root"],
                timeout_seconds=record.get("timeout_seconds"),
                status_detail=record.get("status_detail"),
                cancel_requested=record.get("cancel_requested", False),
                current_step_name=record.get("current_step_name"),
                current_step_index=record.get("current_step_index"),
                last_event=record.get("last_event"),
                live_log_tail=record.get("live_log_tail", []),
                live_log_line_count=record.get("live_log_line_count", 0),
                live_log_updated_at=record.get("live_log_updated_at"),
                live_log_path=record.get("live_log_path"),
                live_log_download_url=record.get("live_log_download_url"),
                log_path=record.get("log_path"),
                log_download_url=record.get("log_download_url"),
                message=record.get("message"),
                worker_pid=record.get("worker_pid"),
                exit_reason=record.get("exit_reason"),
                preferred_godot_transport_mode=record.get(
                    "preferred_godot_transport_mode"
                ),
                parameters=record.get("parameters", {}),
                created_at=(
                    datetime.fromisoformat(record["created_at"])
                    if record.get("created_at")
                    else datetime.now()
                ),
                started_at=started_at,
                finished_at=finished_at,
                duration=record.get("duration") or 0.0,
                progress_updated_at=record.get("progress_updated_at"),
                steps_snapshot=record.get("steps_snapshot", []),
                step_errors=record.get("step_errors", []),
                artifacts=record.get("artifacts", []),
                godot_delivery=record.get("godot_delivery"),
                result=record.get("result"),
                failed_step_name=record.get("failed_step_name"),
                failed_step_error=record.get("failed_step_error"),
                worker_error_message=record.get("worker_error_message"),
                worker_traceback=record.get("worker_traceback"),
                diagnostic_summary=record.get("diagnostic_summary"),
            )
            session.add(db_run)
            await session.commit()
    except Exception as e:
        logger.error(f"Failed to persist run {record['run_id']} to database: {e}")

    # 2. Maintain in-memory for live polling/SSE (Bounded)
    with execution_lock:
        execution_history.append(record)
        if len(execution_history) > MAX_HISTORY_ITEMS:
            removed_runs = execution_history[:-MAX_HISTORY_ITEMS]
            del execution_history[:-MAX_HISTORY_ITEMS]
            for removed_run in removed_runs:
                removed_run_id = removed_run["run_id"]
                run_event_history.pop(removed_run_id, None)
                run_event_counters.pop(removed_run_id, None)
        _publish_run_event_unlocked(record["run_id"], "run_created", record)
        archived_record = _snapshot_json(record)
    _archive_run_record(archived_record)


async def _update_run_record(
    run_id: str, event_type: Optional[str] = None, **changes: Any
) -> Dict[str, Any]:
    """Apply in-place updates to database and memory, notifying listeners (V2.0)."""
    # 1. Update Database
    updated_record = None
    try:
        async with AsyncSessionLocal() as session:
            result = await session.execute(
                select(WorkflowRun).where(WorkflowRun.run_id == run_id)
            )
            db_run = result.scalar_one_or_none()
            if db_run:
                for key, value in changes.items():
                    if hasattr(db_run, key):
                        # Handle datetime conversions
                        if key in ("started_at", "finished_at") and isinstance(
                            value, str
                        ):
                            try:
                                value = datetime.fromisoformat(value)
                            except ValueError:
                                continue
                        setattr(db_run, key, value)

                # Recalculate duration if possible
                if db_run.started_at and db_run.finished_at:
                    db_run.duration = (
                        db_run.finished_at - db_run.started_at
                    ).total_seconds()

                await session.commit()
                updated_record = db_run.to_dict()
    except Exception as e:
        logger.error(f"Failed to update run {run_id} in database: {e}")

    # 2. Update Memory and trigger SSE events
    archived_record = None
    memory_record = None
    with execution_lock:
        record = _find_run_record_unlocked(run_id)
        if record is not None:
            record.update(changes)

            # Keep duration in sync
            if record.get("started_at") and record.get("finished_at"):
                try:
                    s = datetime.fromisoformat(record["started_at"])
                    f = datetime.fromisoformat(record["finished_at"])
                    record["duration"] = (f - s).total_seconds()
                except (ValueError, TypeError):
                    pass

            if event_type:
                _publish_run_event_unlocked(run_id, event_type, record)

            archived_record = _snapshot_json(record)
            memory_record = record

        # If not in memory but was in DB, use the DB version
        elif updated_record:
            return updated_record

    if memory_record is not None:
        if archived_record is not None:
            _archive_run_record(archived_record)
        return memory_record

    raise HTTPException(status_code=404, detail=f"Workflow run '{run_id}' not found")


def _trim_parameters(parameters: Dict[str, Any]) -> Dict[str, Any]:
    """Keep the public record payload concise and JSON-serializable."""
    return json.loads(json.dumps(parameters))


def _normalize_run_scope(scope: str) -> str:
    """Validate and normalize the requested run listing scope."""
    normalized = scope.strip().lower()
    if normalized not in VALID_RUN_SCOPES:
        valid = ", ".join(sorted(VALID_RUN_SCOPES))
        raise HTTPException(
            status_code=400,
            detail=f"Invalid scope '{scope}'. Expected one of: {valid}",
        )
    return normalized


def _normalize_page_number(page: int) -> int:
    """Clamp one requested page number to a valid positive integer."""
    return max(1, page)


def _normalize_page_size(page_size: Optional[int], limit: Optional[int]) -> int:
    """Normalize page-size inputs while keeping `limit` as a compatibility alias."""
    requested_size = page_size if page_size is not None else limit
    if requested_size is None:
        requested_size = DEFAULT_RUNS_PAGE_SIZE

    max_page_size = max(1, MAX_RUNS_PAGE_SIZE)
    return max(1, min(requested_size, max_page_size))


def _parse_filter_datetime(
    value: Optional[str], *, end_of_day: bool = False
) -> Optional[datetime]:
    """Parse an ISO datetime or date-only filter value."""
    if not value:
        return None

    normalized = value.strip()
    if not normalized:
        return None

    is_date_only = "T" not in normalized and " " not in normalized
    if is_date_only:
        normalized = (
            f"{normalized}T23:59:59.999999" if end_of_day else f"{normalized}T00:00:00"
        )

    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid datetime filter '{value}'. Use ISO date or datetime.",
        ) from None
    return parsed


def _build_steps_snapshot(steps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Build a stable, compact step snapshot for polling responses."""
    snapshots = []
    for step in steps:
        output = step.get("output") or {}
        snapshots.append(
            {
                "name": step.get("name"),
                "executor": step.get("executor"),
                "action": step.get("action"),
                "status": step.get("status"),
                "duration": step.get("duration"),
                "error": step.get("error"),
                "start_time": step.get("start_time"),
                "end_time": step.get("end_time"),
                "output_file": output.get("output_file"),
                "output_keys": sorted(output.keys()),
                "artifact_path": step.get("artifact_path"),
            }
        )
    return snapshots


def _extract_step_errors(steps: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Collect failed step diagnostics from serialized step snapshots."""
    errors: List[Dict[str, Any]] = []
    for step in steps:
        status = step.get("status")
        error = step.get("error")
        if status != "failed" and not error:
            continue
        errors.append(
            {
                "name": step.get("name"),
                "status": status,
                "error": error,
                "executor": step.get("executor"),
                "action": step.get("action"),
                "output_file": step.get("output_file"),
            }
        )
    return errors


def _build_failure_diagnostics(
    *,
    steps_snapshot: List[Dict[str, Any]],
    result_error_message: Optional[str] = None,
    worker_error_message: Optional[str] = None,
    worker_traceback: Optional[str] = None,
) -> Dict[str, Any]:
    """Build structured failure diagnostics for one run record."""
    step_errors = _extract_step_errors(steps_snapshot)
    failed_step_name = step_errors[-1]["name"] if step_errors else None
    failed_step_error = step_errors[-1]["error"] if step_errors else None

    return {
        "failed_step_name": failed_step_name,
        "failed_step_error": failed_step_error,
        "step_errors": step_errors,
        "worker_error_message": worker_error_message,
        "worker_traceback": worker_traceback,
        "diagnostic_summary": worker_error_message
        or failed_step_error
        or result_error_message,
    }


def _collect_artifacts_from_result_dict(
    result_dict: Dict[str, Any],
    run_id: str,
) -> List[Dict[str, Any]]:
    """Build a downloadable artifact manifest from serialized workflow step outputs."""
    artifacts: List[Dict[str, Any]] = []
    seen_paths: set[str] = set()

    for step in result_dict.get("steps", []):
        output = step.get("output") or {}
        output_file = output.get("output_file")
        if not isinstance(output_file, str):
            continue

        artifact_path = Path(output_file)
        normalized_path = str(artifact_path)
        if (
            normalized_path in seen_paths
            or not artifact_path.exists()
            or not artifact_path.is_file()
        ):
            continue

        seen_paths.add(normalized_path)
        artifact_index = len(artifacts)
        artifact_metadata = _build_artifact_metadata(
            artifact_path, run_id, artifact_index, step
        )
        artifacts.append(
            {
                "artifact_index": artifact_index,
                "name": artifact_path.name,
                "path": normalized_path,
                "step_name": step.get("name", "unknown"),
                "size_bytes": artifact_path.stat().st_size,
                "download_url": f"/api/workflows/runs/{run_id}/artifacts/{artifact_index}",
                **artifact_metadata,
            }
        )

    return artifacts


def _read_json_file(path: Path) -> Optional[Dict[str, Any]]:
    """Return one JSON document if it is a dict-shaped payload."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def _is_robot_config_payload(payload: Optional[Dict[str, Any]]) -> bool:
    """Return whether a JSON payload looks like a robot config understood by Godot."""
    if not isinstance(payload, dict):
        return False
    return isinstance(payload.get("parts"), list) and isinstance(
        payload.get("connections"), list
    )


def _build_step_artifact_contract(step: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Validate a persisted workflow-step artifact when one exists."""
    artifact_path = step.get("artifact_path")
    if not isinstance(artifact_path, str) or not artifact_path:
        return None

    payload = _read_json_file(Path(artifact_path))
    errors = (
        validate_workflow_step_artifact(payload)
        if payload is not None
        else [f"step artifact is not readable JSON: {artifact_path}"]
    )
    return {
        "path": artifact_path,
        "schema_version": (
            payload.get("schema_version") if isinstance(payload, dict) else None
        ),
        "valid": not errors,
        "errors": errors,
    }


def _build_artifact_contract(
    artifact_type: str,
    payload: Optional[Dict[str, Any]],
    step_output: Dict[str, Any],
) -> Dict[str, Any]:
    """Validate the public contract for one generated artifact."""
    if artifact_type == "robot_config":
        errors = validate_robot_config(payload)
    elif artifact_type in {"urdf", "sdf", "mjcf"}:
        errors = validate_export_result(step_output)
    else:
        errors = []

    return {
        "schema_version": WORKFLOW_CONTRACT_VERSION,
        "payload_type": artifact_type,
        "valid": not errors,
        "errors": errors,
    }


def _build_artifact_metadata(
    artifact_path: Path,
    run_id: str,
    artifact_index: int,
    step: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Attach preview/load capabilities to one workflow artifact."""
    artifact_type = "file"
    preview_mode = None
    preview_url = None
    godot_load_supported = False
    godot_load_url = None
    preferred_godot_transport_mode = None

    payload = None
    step_output = (step or {}).get("output") or {}
    suffix = artifact_path.suffix.lower()

    if suffix in {".urdf", ".sdf", ".mjcf"}:
        artifact_type = suffix[1:]
        if suffix == ".urdf":
            preview_mode = "web_urdf"
            preview_url = f"/api/workflows/runs/{run_id}/artifacts/{artifact_index}"
    elif suffix == ".json":
        payload = _read_json_file(artifact_path)
        if _is_robot_config_payload(payload):
            artifact_type = "robot_config"
        else:
            artifact_type = "json"

    contract = _build_artifact_contract(artifact_type, payload, step_output)
    if artifact_type == "robot_config" and contract["valid"]:
        godot_load_supported = True
        godot_load_url = (
            f"/api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load"
        )
        preferred_godot_transport_mode = "session_bridge"

    return {
        "schema_version": WORKFLOW_CONTRACT_VERSION,
        "artifact_type": artifact_type,
        "preview_mode": preview_mode,
        "preview_url": preview_url,
        "godot_load_supported": godot_load_supported,
        "godot_load_url": godot_load_url,
        "preferred_godot_transport_mode": preferred_godot_transport_mode,
        "contract": contract,
        "contract_valid": contract["valid"],
        "contract_errors": contract["errors"],
        "step_artifact_contract": _build_step_artifact_contract(step or {}),
    }


def _control_dir(output_root: str) -> Path:
    """Return the control directory for one workflow run."""
    path = Path(output_root) / ".output"
    path.mkdir(parents=True, exist_ok=True)
    return path


def _build_live_log_path(output_root: str, run_id: str) -> Path:
    """Return the text log path used for incremental workflow logs."""
    return _control_dir(output_root) / f"web_workflow_live_{run_id}.log"


def _build_initial_run_record(
    run_id: str,
    workflow_name: str,
    request: WorkflowRunRequest,
    output_root: str,
) -> Dict[str, Any]:
    """Create the public run record before the worker starts."""
    created_at = _now().isoformat()
    mode = "real" if request.use_real else "mock"
    parameters = _trim_parameters(request.parameters)
    live_log_path = _build_live_log_path(output_root, run_id)

    return {
        "run_id": run_id,
        "run_source": "active",
        "workflow_name": workflow_name,
        "status": "pending",
        "mode": mode,
        "execution_strategy": request.execution_strategy,
        "output_root": output_root,
        "timeout_seconds": request.timeout_seconds,
        "parameters": parameters,
        "created_at": created_at,
        "started_at": None,
        "finished_at": None,
        "duration": 0.0,
        "success_rate": 0.0,
        "total_steps": None,
        "completed_steps": 0,
        "skipped_steps": 0,
        "failed_steps": 0,
        "current_step_name": None,
        "current_step_index": None,
        "last_event": None,
        "progress_updated_at": None,
        "steps_snapshot": [],
        "cancel_requested": False,
        "status_detail": "Run accepted and waiting for worker process.",
        "message": None,
        "worker_pid": None,
        "exit_reason": None,
        "log_path": None,
        "log_download_url": None,
        "live_log_path": str(live_log_path),
        "live_log_download_url": f"/api/workflows/runs/{run_id}/live-log",
        "live_log_tail": [],
        "live_log_line_count": 0,
        "live_log_updated_at": None,
        "failed_step_name": None,
        "failed_step_error": None,
        "step_errors": [],
        "worker_error_message": None,
        "worker_traceback": None,
        "diagnostic_summary": None,
        "artifacts": [],
        "workflow_contract_version": WORKFLOW_CONTRACT_VERSION,
        "workflow_result_schema_version": None,
        "workflow_result_artifact_type": None,
        "godot_delivery": None,
        "preferred_godot_transport_mode": "session_bridge",
        "recommended_godot_sync_url": f"/api/workflows/runs/{run_id}/godot-sync",
        "poll_url": f"/api/workflows/runs/{run_id}",
        "status_url": f"/api/workflows/runs/{run_id}/status",
        "cancel_url": f"/api/workflows/runs/{run_id}/cancel",
        "result": None,
    }


async def _finalize_run_from_result(
    run_id: str,
    result_dict: Dict[str, Any],
) -> Dict[str, Any]:
    """Apply a serialized workflow result to the public run record (V2.0 Persistence)."""
    status = str(result_dict.get("status", "failed"))
    log_path = result_dict.get("log_path")
    artifacts = _collect_artifacts_from_result_dict(result_dict, run_id)
    error_message = result_dict.get("error_message")
    total_steps = result_dict.get("total_steps")
    steps_snapshot = _build_steps_snapshot(result_dict.get("steps", []))
    failure_diagnostics = _build_failure_diagnostics(
        steps_snapshot=steps_snapshot,
        result_error_message=error_message,
    )

    record = await _update_run_record(
        run_id,
        event_type="run_terminal",
        status=status,
        started_at=result_dict.get("start_time"),
        finished_at=result_dict.get("end_time"),
        duration=result_dict.get("duration") or 0.0,
        success_rate=result_dict.get("success_rate") or 0.0,
        total_steps=total_steps,
        completed_steps=result_dict.get("completed_steps") or 0,
        skipped_steps=result_dict.get("skipped_steps") or 0,
        failed_steps=result_dict.get("failed_steps") or 0,
        current_step_name=None,
        current_step_index=total_steps,
        last_event="workflow_finished",
        progress_updated_at=result_dict.get("end_time") or _now().isoformat(),
        steps_snapshot=steps_snapshot,
        status_detail=(
            "Workflow finished successfully."
            if status == "completed"
            else (error_message or "Workflow finished with errors.")
        ),
        message=error_message,
        exit_reason="workflow_result",
        log_path=log_path,
        log_download_url=f"/api/workflows/runs/{run_id}/log" if log_path else None,
        artifacts=artifacts,
        workflow_contract_version=result_dict.get("schema_version")
        or WORKFLOW_CONTRACT_VERSION,
        workflow_result_schema_version=result_dict.get("schema_version"),
        workflow_result_artifact_type=result_dict.get("artifact_type"),
        failed_step_name=failure_diagnostics["failed_step_name"],
        failed_step_error=failure_diagnostics["failed_step_error"],
        step_errors=failure_diagnostics["step_errors"],
        worker_error_message=failure_diagnostics["worker_error_message"],
        worker_traceback=failure_diagnostics["worker_traceback"],
        diagnostic_summary=failure_diagnostics["diagnostic_summary"],
        result=result_dict,
    )
    return record


async def _mark_run_terminal(
    run_id: str,
    status: str,
    *,
    status_detail: str,
    message: Optional[str] = None,
    exit_reason: Optional[str] = None,
    failed_step_name: Optional[str] = None,
    failed_step_error: Optional[str] = None,
    step_errors: Optional[List[Dict[str, Any]]] = None,
    worker_error_message: Optional[str] = None,
    worker_traceback: Optional[str] = None,
    diagnostic_summary: Optional[str] = None,
) -> Dict[str, Any]:
    """Mark a run as terminal without a structured workflow result (V2.0 Persistence)."""
    finished_at = _now()
    record = await _get_run_record(run_id)

    if record["started_at"]:
        started_at = datetime.fromisoformat(record["started_at"])
        duration = max(0.0, (finished_at - started_at).total_seconds())
    else:
        duration = 0.0

    return await _update_run_record(
        run_id,
        event_type="run_terminal",
        status=status,
        finished_at=finished_at.isoformat(),
        duration=duration,
        current_step_name=None,
        current_step_index=None,
        last_event=status,
        progress_updated_at=finished_at.isoformat(),
        status_detail=status_detail,
        message=message,
        exit_reason=exit_reason,
        log_path=None,
        log_download_url=None,
        artifacts=[],
        failed_step_name=failed_step_name,
        failed_step_error=failed_step_error,
        step_errors=step_errors or [],
        worker_error_message=worker_error_message,
        worker_traceback=worker_traceback,
        diagnostic_summary=diagnostic_summary
        or worker_error_message
        or failed_step_error
        or message,
        result=None,
    )


def _get_run_artifact(run: Dict[str, Any], artifact_index: int) -> Dict[str, Any]:
    """Return one artifact manifest entry or raise 404."""
    try:
        return run["artifacts"][artifact_index]
    except IndexError as exc:
        raise HTTPException(
            status_code=404, detail="Workflow artifact not found"
        ) from exc


def _get_recommended_godot_artifact(run: Dict[str, Any]) -> Dict[str, Any]:
    """Return the first artifact that can be forwarded into Godot."""
    for artifact in run.get("artifacts", []):
        if artifact.get("godot_load_supported"):
            return artifact
    raise HTTPException(
        status_code=404,
        detail="No Godot-loadable robot config artifact found for this run",
    )


def _load_robot_config_from_artifact(artifact: Dict[str, Any]) -> Dict[str, Any]:
    """Read and validate a robot config artifact payload."""
    if not artifact.get("godot_load_supported"):
        raise HTTPException(
            status_code=400,
            detail="Selected artifact is not a Godot-loadable robot config",
        )

    artifact_path = Path(artifact["path"])
    payload = _read_json_file(artifact_path)
    if not _is_robot_config_payload(payload):
        raise HTTPException(
            status_code=400,
            detail="Artifact JSON does not contain parts/connections robot config",
        )
    errors = validate_robot_config(payload)
    if errors:
        raise HTTPException(
            status_code=400,
            detail={
                "message": "Artifact JSON does not match the robot config contract",
                "schema_version": WORKFLOW_CONTRACT_VERSION,
                "errors": errors,
            },
        )
    return payload


def _normalize_transport_mode(transport_mode: str) -> str:
    """Validate workflow-to-Godot transport mode selection."""
    normalized = transport_mode.strip().lower()
    valid_modes = {"legacy_controller", "session_bridge"}
    if normalized not in valid_modes:
        raise HTTPException(
            status_code=400,
            detail=f"Invalid transport_mode '{transport_mode}'. Expected one of: {', '.join(sorted(valid_modes))}",
        )
    return normalized


def _build_godot_delivery_record(
    run_id: str,
    artifact: Dict[str, Any],
    transport_result: Dict[str, Any],
    payload: WorkflowArtifactGodotLoadRequest,
    *,
    auto_selected: bool,
) -> Dict[str, Any]:
    """Serialize one successful workflow-to-Godot delivery result."""
    transport_mode = transport_result.get("transport_mode", payload.transport_mode)
    session_id = transport_result.get("session_id", payload.session_id)
    return {
        "status": "success",
        "timestamp": _now().isoformat(),
        "artifact_index": artifact["artifact_index"],
        "artifact_name": artifact["name"],
        "artifact_type": artifact.get("artifact_type"),
        "artifact_path": artifact["path"],
        "artifact_schema_version": artifact.get("schema_version"),
        "artifact_contract": artifact.get("contract"),
        "auto_selected": auto_selected,
        "transport_mode": transport_mode,
        "session_id": session_id,
        "session_state": transport_result.get("session_state"),
        "session_status": transport_result.get("session_status"),
        "engine_running": transport_result.get("engine_running"),
        "connected": transport_result.get("connected"),
        "schema_available": transport_result.get("schema_available"),
        "schema_keys": transport_result.get("schema_keys") or [],
        "launch_result": transport_result.get("launch_result"),
        "load_result": transport_result.get("load_result"),
        "host": transport_result.get("host", payload.host),
        "port": transport_result.get("port", payload.port),
        "scene": payload.scene,
        "headless": payload.headless,
        "auto_connect": payload.auto_connect,
        "launch_if_needed": payload.launch_if_needed,
        "transport_status_url": _build_godot_transport_status_url(
            transport_mode, session_id
        ),
        "artifact_retry_url": f"/api/workflows/runs/{run_id}/artifacts/{artifact['artifact_index']}/godot-load",
        "recommended_sync_url": f"/api/workflows/runs/{run_id}/godot-sync",
        "delivery_target": _build_godot_delivery_target_summary(
            transport_mode, session_id
        ),
        "message": (f"Artifact '{artifact['name']}' delivered via {transport_mode}."),
    }


def _build_godot_delivery_failure_record(
    run_id: str,
    artifact: Dict[str, Any],
    payload: WorkflowArtifactGodotLoadRequest,
    detail: str,
    *,
    auto_selected: bool,
) -> Dict[str, Any]:
    """Serialize one failed workflow-to-Godot delivery attempt."""
    classification = _classify_godot_delivery_failure(detail, payload.transport_mode)
    return {
        "status": "error",
        "timestamp": _now().isoformat(),
        "artifact_index": artifact["artifact_index"],
        "artifact_name": artifact["name"],
        "artifact_type": artifact.get("artifact_type"),
        "artifact_path": artifact["path"],
        "artifact_schema_version": artifact.get("schema_version"),
        "artifact_contract": artifact.get("contract"),
        "auto_selected": auto_selected,
        "transport_mode": payload.transport_mode,
        "session_id": payload.session_id,
        "scene": payload.scene,
        "headless": payload.headless,
        "auto_connect": payload.auto_connect,
        "launch_if_needed": payload.launch_if_needed,
        "transport_status_url": _build_godot_transport_status_url(
            payload.transport_mode, payload.session_id
        ),
        "artifact_retry_url": f"/api/workflows/runs/{run_id}/artifacts/{artifact['artifact_index']}/godot-load",
        "recommended_sync_url": f"/api/workflows/runs/{run_id}/godot-sync",
        "delivery_target": _build_godot_delivery_target_summary(
            payload.transport_mode, payload.session_id
        ),
        "failure_stage": classification["failure_stage"],
        "retry_hint": classification["retry_hint"],
        "message": detail,
    }


def _build_godot_transport_status_url(transport_mode: str, session_id: str) -> str:
    """Return the appropriate status endpoint for one Godot transport/session pair."""
    normalized_mode = _normalize_transport_mode(transport_mode)
    normalized_session_id = session_id or DEFAULT_GODOT_SESSION_ID
    if normalized_mode == "legacy_controller":
        return f"/api/godot/status?session_id={quote(normalized_session_id, safe='')}"
    return f"/api/godot/{quote(normalized_session_id, safe='')}/status"


def _build_godot_delivery_target_summary(transport_mode: str, session_id: str) -> str:
    """Build a short human-readable target summary for UI rendering."""
    normalized_mode = _normalize_transport_mode(transport_mode)
    normalized_session_id = session_id or DEFAULT_GODOT_SESSION_ID
    if normalized_mode == "legacy_controller":
        return f"legacy session {normalized_session_id}"
    return f"session bridge {normalized_session_id}"


def _classify_godot_delivery_failure(
    detail: str, transport_mode: str
) -> Dict[str, str]:
    """Classify one delivery error into a smaller set of UI-facing failure stages."""
    lowered = detail.lower()
    normalized_mode = _normalize_transport_mode(transport_mode)

    if normalized_mode == "session_bridge":
        if "failed to launch godot session bridge" in lowered:
            return {
                "failure_stage": "launch",
                "retry_hint": "检查 Godot 可执行文件、场景路径和 headless 参数后重试。",
            }
        if "did not become ready in time" in lowered or "ready in time" in lowered:
            return {
                "failure_stage": "tcp_connect",
                "retry_hint": "确认 Godot 场景已启动 TCP 服务器，并检查端口或启动时序。",
            }
        if "rejected robot config" in lowered:
            return {
                "failure_stage": "load_robot",
                "retry_hint": "检查机器人配置产物内容和 Godot 侧 load_robot 处理逻辑。",
            }
        if "schema" in lowered:
            return {
                "failure_stage": "schema",
                "retry_hint": "确认 Godot 侧 schema 已正确返回，并检查 bridge 协议兼容性。",
            }
        return {
            "failure_stage": "transport",
            "retry_hint": "检查 session bridge 进程状态、TCP 连接与最新运行日志。",
        }

    if "not connected" in lowered:
        return {
            "failure_stage": "connect",
            "retry_hint": "先连接 legacy Godot controller，或启用 auto_connect 后重试。",
        }
    if "robot config" in lowered or "send command" in lowered:
        return {
            "failure_stage": "load_robot",
            "retry_hint": "检查 legacy controller 会话和机器人配置内容后重试。",
        }
    return {
        "failure_stage": "transport",
        "retry_hint": "检查 legacy controller 连接状态和相关会话。",
    }


def _load_robot_via_legacy_controller(
    request: Request,
    robot_config: Dict[str, Any],
    payload: WorkflowArtifactGodotLoadRequest,
) -> Dict[str, Any]:
    """Send one robot config to the legacy Godot controller transport."""
    godot_controller = getattr(request.app.state, "godot_controller", None)
    if godot_controller is None:
        raise HTTPException(
            status_code=500,
            detail="Legacy Godot controller is not available on app state",
        )

    connected = godot_controller.is_connected(payload.session_id)
    if not connected and payload.auto_connect:
        connected = godot_controller.connect(
            payload.host, payload.port, session_id=payload.session_id
        )

    if not connected:
        raise HTTPException(
            status_code=400,
            detail="Godot legacy controller is not connected. Start or connect a Godot instance first.",
        )

    success = godot_controller.load_robot(
        robot_config.get("parts", []),
        robot_config.get("connections", []),
        session_id=payload.session_id,
    )
    if not success:
        raise HTTPException(
            status_code=502,
            detail="Failed to send robot config to legacy Godot controller",
        )

    return {
        "transport_mode": "legacy_controller",
        "session_id": payload.session_id,
        "connected": True,
        "auto_connected": payload.auto_connect,
        "host": payload.host,
        "port": payload.port,
    }


async def _load_robot_via_session_bridge(
    request: Request,
    robot_config: Dict[str, Any],
    payload: WorkflowArtifactGodotLoadRequest,
) -> Dict[str, Any]:
    """Send one robot config to the session-bridge Godot transport."""
    session_manager = getattr(request.app.state, "godot_session_manager", None)
    if session_manager is None:
        raise HTTPException(
            status_code=500,
            detail="Godot session manager is not available on app state",
        )

    bridge = session_manager.get_or_create(payload.session_id)
    launch_result = None
    if payload.launch_if_needed and not bridge.is_running():
        launch_result = bridge.launch(
            scene=payload.scene,
            godot_exe=payload.godot_exe,
            headless=payload.headless,
        )
        if launch_result.get("status") not in {"launched", "already_running"}:
            raise HTTPException(
                status_code=502,
                detail=f"Failed to launch Godot session bridge: {launch_result.get('message', 'unknown error')}",
            )

    connected = bridge.is_connected() or await bridge.wait_until_connected(
        timeout_seconds=payload.wait_for_tcp_seconds,
    )
    if not connected:
        diagnostics = bridge.get_process_diagnostics()
        raise HTTPException(
            status_code=504,
            detail=f"Godot session bridge did not become ready in time: {json.dumps(diagnostics, ensure_ascii=False)}",
        )

    load_result = await bridge.send_load_robot(robot_config)
    if load_result.get("status") not in {None, "success"}:
        raise HTTPException(
            status_code=502,
            detail=f"Godot session bridge rejected robot config: {json.dumps(load_result, ensure_ascii=False)}",
        )

    schema = await bridge.wait_until_schema(
        timeout_seconds=min(payload.wait_for_tcp_seconds, 5.0)
    )
    session_status = (
        bridge.get_status_payload()
        if hasattr(bridge, "get_status_payload")
        else {
            "session_state": None,
            "engine_running": bridge.is_running(),
            "tcp_connected": bridge.is_connected(),
        }
    )
    return {
        "transport_mode": "session_bridge",
        "session_id": payload.session_id,
        "connected": bridge.is_connected(),
        "engine_running": bridge.is_running(),
        "session_state": session_status.get("session_state"),
        "session_status": session_status,
        "launch_result": launch_result,
        "load_result": load_result,
        "schema_available": bool(schema),
        "schema_keys": sorted(schema.keys()) if schema else [],
    }


async def _execute_godot_delivery(
    request: Request,
    artifact: Dict[str, Any],
    payload: WorkflowArtifactGodotLoadRequest,
) -> Dict[str, Any]:
    """Load one artifact into Godot and return the transport payload."""
    robot_config = _load_robot_config_from_artifact(artifact)
    transport_mode = _normalize_transport_mode(payload.transport_mode)

    if transport_mode == "legacy_controller":
        return _load_robot_via_legacy_controller(request, robot_config, payload)
    return await _load_robot_via_session_bridge(request, robot_config, payload)


def _terminate_process(process: subprocess.Popen[Any]) -> None:
    """Terminate a worker process, escalating to kill if needed."""
    if process.poll() is not None:
        return

    process.terminate()
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)


def _cleanup_active_run(run_id: str) -> None:
    """Drop the process handle for a completed run."""
    with execution_lock:
        active_run_processes.pop(run_id, None)


def _read_worker_result(result_path: Path) -> Optional[Dict[str, Any]]:
    """Read the worker's structured JSON payload if it exists."""
    if not result_path.exists():
        return None

    try:
        with result_path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except json.JSONDecodeError:
        return None


def _read_worker_progress(progress_path: Path) -> Optional[Dict[str, Any]]:
    """Read the worker's progress snapshot if it exists."""
    if not progress_path.exists():
        return None

    try:
        with progress_path.open("r", encoding="utf-8") as handle:
            return json.load(handle)
    except json.JSONDecodeError:
        return None


def _read_live_log_delta(live_log_path: Path, offset: int) -> tuple[int, List[str]]:
    """Read newly appended lines from the incremental live log file."""
    if not live_log_path.exists():
        return offset, []

    with live_log_path.open("r", encoding="utf-8") as handle:
        handle.seek(offset)
        delta = handle.read()
        new_offset = handle.tell()

    if not delta:
        return new_offset, []

    lines = [line for line in delta.splitlines() if line.strip()]
    return new_offset, lines


def _merge_live_log_lines(existing_lines: List[str], new_lines: List[str]) -> List[str]:
    """Append new lines and keep only the configured tail window."""
    merged = list(existing_lines) + list(new_lines)
    if len(merged) > MAX_LIVE_LOG_TAIL_LINES:
        return merged[-MAX_LIVE_LOG_TAIL_LINES:]
    return merged


async def _consume_live_log_delta(run_id: str, live_log_path: Path, offset: int) -> int:
    """Merge any newly appended live log lines into the public run record."""
    new_offset, new_lines = _read_live_log_delta(live_log_path, offset)
    if not new_lines:
        return new_offset

    record = await _get_run_record(run_id)
    merged_tail = _merge_live_log_lines(record.get("live_log_tail") or [], new_lines)
    await _update_run_record(
        run_id,
        event_type="run_log",
        live_log_tail=merged_tail,
        live_log_line_count=(record.get("live_log_line_count") or 0) + len(new_lines),
        live_log_updated_at=_now().isoformat(),
    )
    return new_offset


async def _apply_progress_snapshot(
    run_id: str, progress_payload: Dict[str, Any]
) -> Dict[str, Any]:
    """Merge a worker progress payload into the public run record."""
    current_step = progress_payload.get("current_step") or {}
    total_steps = progress_payload.get("total_steps")
    step_index = progress_payload.get("step_index")
    event = progress_payload.get("event")
    workflow_status = progress_payload.get("workflow_status")
    current_step_name = current_step.get("name")
    steps_snapshot = _build_steps_snapshot(progress_payload.get("steps", []))
    failure_diagnostics = _build_failure_diagnostics(
        steps_snapshot=steps_snapshot,
        result_error_message=progress_payload.get("error_message"),
    )

    if event == "step_started" and current_step_name and step_index and total_steps:
        status_detail = f"Running step {step_index}/{total_steps}: {current_step_name}"
    elif event == "step_finished" and current_step_name and step_index and total_steps:
        status_detail = f"Finished step {step_index}/{total_steps}: {current_step_name} ({current_step.get('status')})"
    elif event == "workflow_started":
        status_detail = "Workflow worker started executing steps."
    elif event == "workflow_finished":
        status_detail = f"Workflow finished with status '{workflow_status}'."
    else:
        status_detail = f"Progress update received: {event or 'unknown'}"

    started_at = progress_payload.get("started_at")
    updated_status = "running"
    record = await _get_run_record(run_id)
    if record["status"] in TERMINAL_RUN_STATUSES:
        updated_status = record["status"]

    return await _update_run_record(
        run_id,
        event_type="run_progress",
        status=updated_status,
        started_at=record["started_at"] or started_at,
        duration=record["duration"],
        total_steps=total_steps,
        completed_steps=progress_payload.get("completed_steps") or 0,
        skipped_steps=progress_payload.get("skipped_steps") or 0,
        failed_steps=progress_payload.get("failed_steps") or 0,
        success_rate=progress_payload.get("success_rate") or 0.0,
        current_step_name=current_step_name,
        current_step_index=step_index,
        last_event=event,
        progress_updated_at=progress_payload.get("timestamp") or _now().isoformat(),
        steps_snapshot=steps_snapshot,
        status_detail=status_detail,
        failed_step_name=failure_diagnostics["failed_step_name"],
        failed_step_error=failure_diagnostics["failed_step_error"],
        step_errors=failure_diagnostics["step_errors"],
        diagnostic_summary=failure_diagnostics["diagnostic_summary"],
    )


async def _start_background_run(
    run_id: str,
    workflow_name: str,
    request: WorkflowRunRequest,
    output_root: str,
) -> Dict[str, Any]:
    """Dispatch a production Celery task for workflow execution (V2.0)."""
    from web_panel.workflow_worker_task import run_workflow_task

    live_log_path = _build_live_log_path(output_root, run_id)

    # Payload for the remote worker
    task_payload = {
        "run_id": run_id,
        "workflow_name": workflow_name,
        "use_real": request.use_real,
        "parameters": request.parameters,
        "output_root": output_root,
        "live_log_path": str(live_log_path),
    }

    # Dispatch to Redis queue
    try:
        task = run_workflow_task.delay(task_payload)
        logger.info(f"Dispatched Celery task {task.id} for run {run_id}")
    except Exception as e:
        logger.error(f"Failed to dispatch Celery task: {e}")
        raise HTTPException(
            status_code=500, detail="Task queue unavailable (Redis down?)"
        )

    # Initial DB update
    await _update_run_record(
        run_id,
        event_type="run_started",
        status="running",
        status_detail="Workflow queued in Celery.",
        worker_pid=None,  # Not applicable for remote workers
    )

    return await _get_run_record(run_id)


def _monitor_background_run(run_id: str) -> None:
    """Compatibility monitor for legacy in-process worker tests and local fallback flows."""
    with execution_lock:
        state = active_run_processes.get(run_id)
    if state is None:
        return

    process = state.get("process")
    result_path = Path(state["result_path"])
    progress_path = Path(state["progress_path"])
    live_log_path = Path(state["live_log_path"])
    live_log_offset = int(state.get("live_log_offset", 0))
    timeout_seconds = state.get("timeout_seconds")
    started_monotonic = state.get("started_monotonic")

    progress_payload = _read_worker_progress(progress_path)
    if progress_payload:
        _run_async(_apply_progress_snapshot(run_id, progress_payload))

    new_offset = _run_async(
        _consume_live_log_delta(run_id, live_log_path, live_log_offset)
    )
    with execution_lock:
        if run_id in active_run_processes:
            active_run_processes[run_id]["live_log_offset"] = new_offset

    record = _run_async(_get_run_record(run_id))
    if record.get("cancel_requested"):
        if process is not None:
            _terminate_process(process)
        _run_async(
            _mark_run_terminal(
                run_id,
                "cancelled",
                status_detail="Workflow cancelled after user request.",
                message="Cancellation requested from Web workflow control plane.",
                exit_reason="cancelled_by_user",
            )
        )
        _cleanup_active_run(run_id)
        return

    if (
        timeout_seconds is not None
        and started_monotonic is not None
        and (time.monotonic() - started_monotonic) >= timeout_seconds
    ):
        if process is not None:
            _terminate_process(process)
        _run_async(
            _mark_run_terminal(
                run_id,
                "timed_out",
                status_detail=f"Workflow timed out after {timeout_seconds} seconds.",
                message=f"Workflow timed out after {timeout_seconds} seconds.",
                exit_reason="timeout",
            )
        )
        _cleanup_active_run(run_id)
        return

    exit_code = process.poll() if process is not None else 0
    if exit_code is None:
        return

    worker_result = _read_worker_result(result_path)
    if worker_result:
        if worker_result.get("status") in {"ok", "success"} and isinstance(
            worker_result.get("result"), dict
        ):
            _run_async(_finalize_run_from_result(run_id, worker_result["result"]))
        else:
            worker_error_message = (
                worker_result.get("error_message")
                or worker_result.get("message")
                or f"Workflow worker exited with code {exit_code}."
            )
            worker_traceback = worker_result.get("traceback")
            _run_async(
                _mark_run_terminal(
                    run_id,
                    "failed",
                    status_detail="Workflow worker exited with an error.",
                    message=worker_error_message,
                    exit_reason="worker_error",
                    worker_error_message=worker_error_message,
                    worker_traceback=worker_traceback,
                    diagnostic_summary=worker_error_message,
                )
            )
        _cleanup_active_run(run_id)
        return

    _run_async(
        _mark_run_terminal(
            run_id,
            "completed" if exit_code == 0 else "failed",
            status_detail=(
                "Workflow finished successfully."
                if exit_code == 0
                else "Workflow worker exited with a non-zero status."
            ),
            message=(
                None
                if exit_code == 0
                else f"Workflow worker exited with code {exit_code}."
            ),
            exit_reason="worker_exit" if exit_code == 0 else "worker_exit_nonzero",
            diagnostic_summary=(
                None
                if exit_code == 0
                else f"Workflow worker exited with code {exit_code}."
            ),
        )
    )
    _cleanup_active_run(run_id)


@router.get("/")
async def list_workflows() -> List[Dict[str, Any]]:
    """List all available workflows."""
    orchestrator = get_workflow_orchestrator()
    workflows = []
    for name in orchestrator.list_workflows():
        workflow = orchestrator.get_workflow(name) or {}
        steps = workflow.get("steps", [])
        is_valid, validation_message = orchestrator.validate_workflow(name)
        workflows.append(
            {
                "schema_version": WORKFLOW_CONTRACT_VERSION,
                "name": name,
                "description": workflow.get("description", ""),
                "steps_count": len(steps),
                "step_names": [step.get("name", "unnamed") for step in steps],
                "validation": {
                    "valid": is_valid,
                    "message": validation_message,
                },
            }
        )
    return workflows


@router.get("/runs")
async def list_runs(
    scope: str = "all",
    workflow_name: Optional[str] = None,
    status: Optional[str] = None,
    mode: Optional[str] = None,
    text: Optional[str] = None,
    date_from: Optional[str] = None,
    date_to: Optional[str] = None,
    only_failures: bool = False,
    page: int = 1,
    page_size: Optional[int] = None,
    limit: Optional[int] = None,
) -> Dict[str, Any]:
    """Return paginated workflow runs while preserving active/archive/all API semantics."""
    parsed_date_from = _parse_filter_datetime(date_from, end_of_day=False)
    parsed_date_to = _parse_filter_datetime(date_to, end_of_day=True)
    normalized_scope = _normalize_run_scope(scope)
    normalized_page = _normalize_page_number(page)
    normalized_page_size = _normalize_page_size(page_size, limit)
    records = await _collect_run_records(normalized_scope)
    filtered_runs = [
        run
        for run in records
        if _record_matches_filters(
            run,
            workflow_name=workflow_name,
            status=status,
            mode=mode,
            text=text,
            parsed_date_from=parsed_date_from,
            parsed_date_to=parsed_date_to,
            only_failures=only_failures,
        )
    ]
    filtered_runs.sort(key=_record_timestamp, reverse=True)

    total_count = len(filtered_runs)
    offset = (normalized_page - 1) * normalized_page_size
    runs = filtered_runs[offset : offset + normalized_page_size]
    total_pages = max(
        1, (total_count + normalized_page_size - 1) // normalized_page_size
    )

    return {
        "status": "success",
        "scope": normalized_scope,
        "count": len(runs),
        "total_count": total_count,
        "page": normalized_page,
        "page_size": normalized_page_size,
        "total_pages": total_pages,
        "offset": offset,
        "has_previous_page": normalized_page > 1,
        "has_next_page": offset + len(runs) < total_count,
        "runs": runs,
        "filters": {
            "workflow_name": workflow_name,
            "status": status,
            "mode": mode,
            "text": text,
            "date_from": date_from,
            "date_to": date_to,
            "only_failures": only_failures,
        },
        "archive_retention_policy": _get_archive_retention_policy(),
    }


@router.get("/runs/{run_id}")
async def get_run_detail(run_id: str) -> Dict[str, Any]:
    """Get the full execution record for a workflow run."""
    return {"status": "success", "run": await _get_run_record(run_id)}


@router.get("/runs/{run_id}/status")
async def get_run_status(run_id: str) -> Dict[str, Any]:
    """Get a light-weight status payload for polling."""
    run = await _get_run_record(run_id)
    return {
        "status": "success",
        "run": {
            "run_id": run["run_id"],
            "run_source": run.get("run_source", "active"),
            "workflow_name": run["workflow_name"],
            "status": run["status"],
            "status_detail": run["status_detail"],
            "cancel_requested": run["cancel_requested"],
            "started_at": run["started_at"],
            "finished_at": run["finished_at"],
            "duration": run["duration"],
            "success_rate": run["success_rate"],
            "total_steps": run["total_steps"],
            "completed_steps": run["completed_steps"],
            "skipped_steps": run["skipped_steps"],
            "failed_steps": run["failed_steps"],
            "current_step_name": run["current_step_name"],
            "current_step_index": run["current_step_index"],
            "last_event": run["last_event"],
            "progress_updated_at": run["progress_updated_at"],
            "steps_snapshot": run["steps_snapshot"],
            "live_log_tail": run["live_log_tail"],
            "live_log_line_count": run["live_log_line_count"],
            "live_log_updated_at": run["live_log_updated_at"],
            "failed_step_name": run["failed_step_name"],
            "failed_step_error": run["failed_step_error"],
            "step_errors": run["step_errors"],
            "worker_error_message": run["worker_error_message"],
            "diagnostic_summary": run["diagnostic_summary"],
            "worker_pid": run["worker_pid"],
            "workflow_contract_version": run.get("workflow_contract_version"),
            "workflow_result_schema_version": run.get("workflow_result_schema_version"),
            "workflow_result_artifact_type": run.get("workflow_result_artifact_type"),
            "godot_delivery": run["godot_delivery"],
            "preferred_godot_transport_mode": run["preferred_godot_transport_mode"],
            "recommended_godot_sync_url": run["recommended_godot_sync_url"],
        },
    }


@router.get("/runs/{run_id}/events")
async def stream_run_events(run_id: str, request: Request):
    """Stream run updates as Server-Sent Events for one workflow run."""
    await _get_run_record(run_id)
    last_event_id_raw = request.headers.get("last-event-id")
    try:
        last_event_id = int(last_event_id_raw) if last_event_id_raw else 0
    except ValueError as exc:
        raise HTTPException(
            status_code=400, detail="Invalid Last-Event-ID header"
        ) from exc

    def event_stream():
        next_after_event_id = last_event_id
        while True:
            events = _wait_for_run_events(
                run_id,
                after_event_id=next_after_event_id,
                timeout_seconds=RUN_EVENTS_WAIT_TIMEOUT_SECONDS,
            )
            if not events:
                yield ": keep-alive\n\n"
                continue

            for event in events:
                yield _format_sse_event(event)
                next_after_event_id = event["event_id"]
                if event["event_type"] == "run_terminal":
                    return

    return StreamingResponse(
        event_stream(),
        media_type="text/event-stream",
        headers={
            "Cache-Control": "no-cache",
            "Connection": "keep-alive",
            "X-Accel-Buffering": "no",
        },
    )


@router.post("/runs/{run_id}/cancel")
async def cancel_run(run_id: str) -> Dict[str, Any]:
    """Request cancellation of a running workflow."""
    run = await _get_run_record(run_id)
    if run["status"] in TERMINAL_RUN_STATUSES:
        return {
            "status": "success",
            "message": f"Run already finished with status '{run['status']}'.",
            "run": run,
        }

    updated = await _update_run_record(
        run_id,
        event_type="cancel_requested",
        cancel_requested=True,
        status_detail="Cancellation requested. Waiting for worker to stop.",
        message="Cancellation requested from Web workflow control plane.",
    )
    with execution_lock:
        local_process_state = active_run_processes.get(run_id)
    if local_process_state is not None:
        _monitor_background_run(run_id)
    return {
        "status": "success",
        "message": "Cancellation requested.",
        "run": updated,
    }


@router.get("/runs/{run_id}/artifacts/{artifact_index}")
async def download_run_artifact(run_id: str, artifact_index: int):
    """Download one generated artifact from a recorded workflow run."""
    run = await _get_run_record(run_id)
    artifact = _get_run_artifact(run, artifact_index)

    artifact_path = Path(artifact["path"])
    if not artifact_path.exists():
        raise HTTPException(
            status_code=404, detail="Artifact file no longer exists on disk"
        )

    return FileResponse(artifact_path, filename=artifact["name"])


@router.post("/runs/{run_id}/artifacts/{artifact_index}/godot-load")
async def load_run_artifact_into_godot(
    run_id: str,
    artifact_index: int,
    request: Request,
    payload: Optional[WorkflowArtifactGodotLoadRequest] = Body(default=None),
) -> Dict[str, Any]:
    """Load one workflow-produced robot config artifact into Godot."""
    run = await _get_run_record(run_id)
    artifact = _get_run_artifact(run, artifact_index)
    robot_config = _load_robot_config_from_artifact(artifact)
    normalized_payload = payload or WorkflowArtifactGodotLoadRequest()
    transport_mode = _normalize_transport_mode(normalized_payload.transport_mode)

    try:
        transport_result = await _execute_godot_delivery(
            request, artifact, normalized_payload
        )
    except HTTPException as exc:
        await _update_run_record(
            run_id,
            event_type="godot_delivery_updated",
            godot_delivery=_build_godot_delivery_failure_record(
                run_id,
                artifact,
                normalized_payload.model_copy(
                    update={"transport_mode": transport_mode}
                ),
                str(exc.detail),
                auto_selected=False,
            ),
        )
        raise

    await _update_run_record(
        run_id,
        event_type="godot_delivery_updated",
        godot_delivery=_build_godot_delivery_record(
            run_id,
            artifact,
            transport_result,
            normalized_payload.model_copy(update={"transport_mode": transport_mode}),
            auto_selected=False,
        ),
    )

    return {
        "status": "success",
        "message": f"Artifact '{artifact['name']}' sent to Godot via {transport_mode}.",
        "run_id": run_id,
        "workflow_name": run["workflow_name"],
        "artifact": artifact,
        "transport": transport_result,
        "godot_delivery": (await _get_run_record(run_id))["godot_delivery"],
        "robot_config_summary": {
            "parts_count": len(robot_config.get("parts", [])),
            "connections_count": len(robot_config.get("connections", [])),
        },
    }


@router.post("/runs/{run_id}/godot-sync")
async def sync_run_into_godot(
    run_id: str,
    request: Request,
    payload: Optional[WorkflowRunGodotSyncRequest] = Body(default=None),
) -> Dict[str, Any]:
    """Auto-select one Godot-loadable artifact from a run and load it through the official path."""
    run = await _get_run_record(run_id)
    normalized_payload = payload or WorkflowRunGodotSyncRequest()
    transport_mode = _normalize_transport_mode(normalized_payload.transport_mode)
    artifact = (
        _get_run_artifact(run, normalized_payload.artifact_index)
        if normalized_payload.artifact_index is not None
        else _get_recommended_godot_artifact(run)
    )
    normalized_payload = normalized_payload.model_copy(
        update={"transport_mode": transport_mode}
    )

    try:
        transport_result = await _execute_godot_delivery(
            request, artifact, normalized_payload
        )
    except HTTPException as exc:
        await _update_run_record(
            run_id,
            event_type="godot_delivery_updated",
            godot_delivery=_build_godot_delivery_failure_record(
                run_id,
                artifact,
                normalized_payload,
                str(exc.detail),
                auto_selected=normalized_payload.artifact_index is None,
            ),
        )
        raise

    delivery_record = _build_godot_delivery_record(
        run_id,
        artifact,
        transport_result,
        normalized_payload,
        auto_selected=normalized_payload.artifact_index is None,
    )
    await _update_run_record(
        run_id,
        event_type="godot_delivery_updated",
        godot_delivery=delivery_record,
    )

    return {
        "status": "success",
        "message": delivery_record["message"],
        "run_id": run_id,
        "workflow_name": run["workflow_name"],
        "artifact": artifact,
        "transport": transport_result,
        "godot_delivery": delivery_record,
    }


@router.get("/runs/{run_id}/live-log")
async def get_run_live_log(run_id: str):
    """Return the full incremental text log for a workflow run."""
    run = await _get_run_record(run_id)
    live_log_path = run.get("live_log_path")
    if not live_log_path:
        raise HTTPException(status_code=404, detail="Live log not available")

    resolved_log = Path(live_log_path)
    if not resolved_log.exists():
        raise HTTPException(
            status_code=404, detail="Live log file no longer exists on disk"
        )

    return PlainTextResponse(resolved_log.read_text(encoding="utf-8"))


@router.get("/runs/{run_id}/log")
async def download_run_log(run_id: str):
    """Download the saved workflow log for a recorded run."""
    run = await _get_run_record(run_id)
    log_path = run.get("log_path")
    if not log_path:
        raise HTTPException(status_code=404, detail="Workflow log not available")

    resolved_log = Path(log_path)
    if not resolved_log.exists():
        raise HTTPException(
            status_code=404, detail="Workflow log file no longer exists on disk"
        )

    return FileResponse(resolved_log, filename=resolved_log.name)


@router.get("/logs")
async def list_logs() -> List[str]:
    """List workflow logs from the legacy root .output directory."""
    return _list_legacy_logs()


@router.get("/logs/{filename}")
async def get_log_detail(filename: str):
    """Get detailed JSON content of a specific legacy log file."""
    if os.path.basename(filename) != filename:
        raise HTTPException(status_code=400, detail="Invalid log filename")

    log_path = Path(".output") / filename
    if not log_path.exists():
        raise HTTPException(status_code=404, detail="Log file not found")

    try:
        with open(log_path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except Exception as exc:
        raise HTTPException(
            status_code=500, detail=f"Error reading log: {exc}"
        ) from exc


@router.get("/{name}")
async def get_workflow_detail(name: str) -> Dict[str, Any]:
    """Return the detailed definition of a named workflow."""
    orchestrator = get_workflow_orchestrator()
    workflow = orchestrator.get_workflow(name)
    if not workflow:
        raise HTTPException(status_code=404, detail=f"Workflow '{name}' not found")

    is_valid, validation_message = orchestrator.validate_workflow(name)
    return {
        "status": "success",
        "workflow": {
            "schema_version": WORKFLOW_CONTRACT_VERSION,
            "name": workflow.get("name", name),
            "description": workflow.get("description", ""),
            "steps": workflow.get("steps", []),
            "validation": {
                "valid": is_valid,
                "message": validation_message,
            },
        },
    }


@router.post("/{name}/run", status_code=202)
async def run_workflow(
    name: str,
    response: Response,
    request: Optional[WorkflowRunRequest] = Body(default=None),
    use_real: Optional[bool] = None,
    current_user: User = Depends(get_current_user),
) -> Dict[str, Any]:
    """Start a workflow in the background and return the polling record (Protected)."""
    orchestrator = get_workflow_orchestrator()
    if name not in orchestrator.list_workflows():
        raise HTTPException(status_code=404, detail=f"Workflow '{name}' not found")

    payload = request or WorkflowRunRequest()
    if use_real is not None:
        payload = payload.model_copy(update={"use_real": use_real})

    execution_strategy = _validate_execution_strategy(payload.execution_strategy)
    output_root = _build_output_root(name, payload.output_root)

    parameters = dict(payload.parameters)
    parameters["execution_strategy"] = execution_strategy
    parameters["output_root"] = output_root

    normalized_request = payload.model_copy(
        update={
            "execution_strategy": execution_strategy,
            "output_root": output_root,
            "parameters": parameters,
        }
    )

    run_id = uuid4().hex
    record = _build_initial_run_record(run_id, name, normalized_request, output_root)
    await _store_run_record(record, user_id=current_user.id)

    try:
        started_record = await _start_background_run(
            run_id, name, normalized_request, output_root
        )
    except Exception as exc:
        response.status_code = 500
        failed_record = await _mark_run_terminal(
            run_id,
            "failed",
            status_detail="Failed to start workflow worker process.",
            message=str(exc),
            exit_reason="worker_start_error",
        )
        return {
            "status": "error",
            "workflow": name,
            "run": failed_record,
            "message": str(exc),
        }

    return {
        "status": "accepted",
        "workflow": name,
        "run": started_record,
        "message": "Workflow started in background. Poll the run status endpoint for completion.",
    }


def build_router() -> APIRouter:
    """Return the configured workflows router."""
    return router
