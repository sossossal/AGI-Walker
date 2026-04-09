from __future__ import annotations
import asyncio
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from web_panel.celery_app import celery_app
from agi_walker.workflow_orchestrator import get_workflow_orchestrator
import web_panel.workflows_api as workflow_tracking


def _run_async(coro):
    """Helper to run async code inside sync Celery task."""
    try:
        loop = asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)

    if loop.is_running():
        fallback_loop = asyncio.new_event_loop()
        try:
            return fallback_loop.run_until_complete(coro)
        finally:
            fallback_loop.close()
    return loop.run_until_complete(coro)


@celery_app.task(bind=True, name="web_panel.run_workflow_task")
def run_workflow_task(self, payload: Dict[str, Any]):
    """
    Industrial Celery Task for Workflow Execution.
    No longer relies on file system polling from the Web Server.
    """
    run_id = payload["run_id"]
    workflow_name = payload["workflow_name"]
    use_real = payload.get("use_real", True)
    parameters = payload.get("parameters", {})
    live_log_path = Path(payload.get("live_log_path", f".logs/live_{run_id}.log"))

    live_log_path.parent.mkdir(parents=True, exist_ok=True)
    orchestrator = get_workflow_orchestrator()
    live_log_offset = 0

    def _flush_live_log():
        nonlocal live_log_offset
        live_log_offset = _run_async(
            workflow_tracking._consume_live_log_delta(
                run_id, live_log_path, live_log_offset
            )
        )

    def _log(msg: str):
        ts = datetime.now().isoformat(timespec="seconds")
        with live_log_path.open("a", encoding="utf-8") as f:
            f.write(f"[{ts}] {msg}\n")
        _flush_live_log()

    _log(f"Celery Worker started task {self.request.id} for run {run_id}")

    def progress_callback(progress_payload: Dict[str, Any]):
        _run_async(workflow_tracking._apply_progress_snapshot(run_id, progress_payload))

        event = progress_payload.get("event")
        if event == "step_started":
            step = progress_payload.get("current_step", {})
            _log(
                f"Step {progress_payload.get('step_index')}: {step.get('name')} started."
            )
        elif event == "step_finished":
            step = progress_payload.get("current_step", {})
            _log(
                f"Step {progress_payload.get('step_index')}: {step.get('name')} finished ({step.get('status')})."
            )

    try:
        _run_async(
            workflow_tracking._update_run_record(
                run_id,
                event_type="run_started",
                status="running",
                started_at=datetime.now().isoformat(),
                status_detail="Workflow worker started executing steps.",
                worker_pid=None,
            )
        )

        result = orchestrator.execute_workflow(
            workflow_name,
            parameters=parameters,
            use_real=use_real,
            progress_callback=progress_callback,
        )

        _log(f"Workflow {workflow_name} finished: {result.status}")
        _run_async(
            workflow_tracking._finalize_run_from_result(run_id, result.to_dict())
        )
        return {"status": "success", "result": result.to_dict()}

    except Exception as e:
        err_msg = f"Task failed: {str(e)}"
        _log(err_msg)
        _log(traceback.format_exc())

        _run_async(
            workflow_tracking._mark_run_terminal(
                run_id,
                "failed",
                status_detail="Workflow worker exited with an error.",
                message=str(e),
                exit_reason="worker_error",
                worker_error_message=str(e),
                worker_traceback=traceback.format_exc(),
                diagnostic_summary=str(e),
            )
        )

        return {"status": "error", "message": str(e)}
