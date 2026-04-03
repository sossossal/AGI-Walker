"""
Background worker entrypoint for Web workflow runs.

The parent web process writes a JSON request file, spawns this module in a
separate Python process, and then watches the result file for completion.
"""

from __future__ import annotations

import json
import sys
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any, Dict

from agi_walker.workflow_orchestrator import get_workflow_orchestrator


def _load_payload(request_path: Path) -> Dict[str, Any]:
    with request_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _write_result(result_path: Path, payload: Dict[str, Any]) -> None:
    result_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = result_path.with_suffix(".tmp")
    temp_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    temp_path.replace(result_path)


def _write_progress(progress_path: Path, payload: Dict[str, Any]) -> None:
    progress_path.parent.mkdir(parents=True, exist_ok=True)
    temp_path = progress_path.with_suffix(".tmp")
    temp_path.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    temp_path.replace(progress_path)


def _append_live_log_line(live_log_path: Path, message: str) -> None:
    live_log_path.parent.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().isoformat(timespec="seconds")
    with live_log_path.open("a", encoding="utf-8") as handle:
        handle.write(f"[{timestamp}] {message}\n")


def execute_request(request_path: Path, result_path: Path) -> int:
    payload = _load_payload(request_path)
    orchestrator = get_workflow_orchestrator()
    progress_path_raw = payload.get("progress_path")
    progress_path = Path(progress_path_raw) if progress_path_raw else None
    live_log_path_raw = payload.get("live_log_path")
    live_log_path = Path(live_log_path_raw) if live_log_path_raw else None

    if live_log_path is not None:
        _append_live_log_line(
            live_log_path,
            f"Worker started for workflow '{payload['workflow_name']}' (mode={'real' if payload.get('use_real') else 'mock'}).",
        )

    def progress_callback(progress_payload: Dict[str, Any]) -> None:
        if progress_path is not None:
            _write_progress(progress_path, progress_payload)
        if live_log_path is None:
            return

        event = progress_payload.get("event")
        step = progress_payload.get("current_step") or {}
        step_name = step.get("name")
        step_status = step.get("status")
        step_index = progress_payload.get("step_index")
        total_steps = progress_payload.get("total_steps")

        if event == "workflow_started":
            _append_live_log_line(
                live_log_path,
                f"Workflow execution started with {total_steps or 0} steps.",
            )
        elif event == "step_started" and step_name:
            _append_live_log_line(
                live_log_path,
                f"Step {step_index}/{total_steps}: {step_name} started.",
            )
        elif event == "step_finished" and step_name:
            _append_live_log_line(
                live_log_path,
                f"Step {step_index}/{total_steps}: {step_name} finished with status={step_status}.",
            )
        elif event == "workflow_finished":
            _append_live_log_line(
                live_log_path,
                f"Workflow finished with status={progress_payload.get('workflow_status')}.",
            )

    result = orchestrator.execute_workflow(
        payload["workflow_name"],
        parameters=payload.get("parameters") or {},
        use_real=payload.get("use_real"),
        progress_callback=progress_callback,
    )
    if live_log_path is not None:
        _append_live_log_line(
            live_log_path,
            f"Final success_rate={result.success_rate:.1f}% completed={result.completed_steps} skipped={result.skipped_steps} failed={result.failed_steps}.",
        )
    _write_result(
        result_path,
        {
            "status": "ok",
            "result": result.to_dict(),
        },
    )
    return 0


def main(argv: list[str] | None = None) -> int:
    args = argv if argv is not None else sys.argv[1:]
    if len(args) != 2:
        print(
            "Usage: python -m web_panel.workflow_worker <request_path> <result_path>",
            file=sys.stderr,
        )
        return 2

    request_path = Path(args[0])
    result_path = Path(args[1])

    try:
        return execute_request(request_path, result_path)
    except Exception as exc:
        payload = {}
        try:
            payload = _load_payload(request_path)
        except Exception:
            payload = {}
        live_log_path_raw = payload.get("live_log_path") if payload else None
        if live_log_path_raw:
            _append_live_log_line(Path(live_log_path_raw), f"Worker failed: {exc}")
        _write_result(
            result_path,
            {
                "status": "error",
                "error_message": str(exc),
                "traceback": traceback.format_exc(),
            },
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
