"""Deterministic orchestration for the worktree release-blocker flow."""

from __future__ import annotations

import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from agi_walker.core.api.release_ops_contracts import (
    WorktreeReleaseBlockerRequest,
    WorktreeReleaseBlockerResult,
)


def _find_repo_root() -> Path:
    current = Path(__file__).resolve()
    for candidate in current.parents:
        if (candidate / "pyproject.toml").exists() and (candidate / "agi_walker").exists():
            return candidate
    return current.parent


PROJECT_ROOT = _find_repo_root()


def _resolve_project_path(path: str | Path) -> Path:
    candidate = Path(path)
    if candidate.is_absolute():
        return candidate
    return PROJECT_ROOT / candidate


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _default_cleanup_report_path(output_root: Path) -> Path:
    return output_root / "worktree_cleanup_report.json"


def _default_tracked_review_report_path(output_root: Path) -> Path:
    return output_root / "tracked_artifact_review_report.json"


def _default_blocker_report_path(output_root: Path) -> Path:
    return output_root / "worktree_release_blocker_report.json"


def _run_checked_command(
    command: list[str],
    *,
    cwd: Path,
    failure_message: str,
) -> str:
    result = subprocess.run(
        command,
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip() or "unknown failure"
        raise RuntimeError(f"{failure_message}: {detail}")
    return result.stdout


def _load_json_payload(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _validate_cleanup_report(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["cleanup report must be an object"]
    errors: list[str] = []
    if payload.get("artifact_type") != "worktree_cleanup_report":
        errors.append("cleanup report artifact_type must be 'worktree_cleanup_report'")
    for field in [
        "clean_worktree",
        "total_paths",
        "tracked_paths",
        "untracked_paths",
        "staged_paths",
        "unstaged_tracked_paths",
        "staged_and_unstaged_paths",
        "tracked_review_candidate_count",
        "tracked_review_report_path",
        "next_step_plan",
    ]:
        if field not in payload:
            errors.append(f"cleanup report missing required field: {field}")
    if "clean_worktree" in payload and not isinstance(payload.get("clean_worktree"), bool):
        errors.append("cleanup report clean_worktree must be a boolean")
    for field in [
        "total_paths",
        "tracked_paths",
        "untracked_paths",
        "staged_paths",
        "unstaged_tracked_paths",
        "staged_and_unstaged_paths",
        "tracked_review_candidate_count",
    ]:
        value = payload.get(field)
        if field in payload and (not isinstance(value, int) or value < 0):
            errors.append(f"cleanup report {field} must be a non-negative integer")
    if "next_step_plan" in payload and not isinstance(payload.get("next_step_plan"), list):
        errors.append("cleanup report next_step_plan must be a list")
    return errors


def _validate_tracked_review_report(payload: Any) -> list[str]:
    if not isinstance(payload, dict):
        return ["tracked review report must be an object"]
    errors: list[str] = []
    if payload.get("artifact_type") != "tracked_artifact_review_report":
        errors.append(
            "tracked review report artifact_type must be 'tracked_artifact_review_report'"
        )
    for field in ["tracked_candidate_count", "entries", "next_step_plan"]:
        if field not in payload:
            errors.append(f"tracked review report missing required field: {field}")
    candidate_count = payload.get("tracked_candidate_count")
    if candidate_count is not None and (
        not isinstance(candidate_count, int) or candidate_count < 0
    ):
        errors.append("tracked review report tracked_candidate_count must be a non-negative integer")
    if "entries" in payload and not isinstance(payload.get("entries"), list):
        errors.append("tracked review report entries must be a list")
    if "next_step_plan" in payload and not isinstance(payload.get("next_step_plan"), list):
        errors.append("tracked review report next_step_plan must be a list")
    return errors


def _coerce_string_list(value: Any) -> list[str]:
    if not isinstance(value, list):
        return []
    return [str(item).strip() for item in value if isinstance(item, str) and item.strip()]


def _build_summary(
    *,
    status: str,
    clean_worktree: bool,
    total_paths: int,
    staged_paths: int,
    unstaged_tracked_paths: int,
    untracked_paths: int,
    tracked_review_candidate_count: int,
    tracked_review_status: str,
) -> str:
    if clean_worktree:
        return (
            "worktree release blocker ready: "
            "clean_worktree=true, total_paths=0, tracked_review=not_required."
        )
    return (
        "worktree release blocker blocked: "
        f"clean_worktree=false, total_paths={total_paths}, "
        f"staged_paths={staged_paths}, "
        f"unstaged_tracked_paths={unstaged_tracked_paths}, "
        f"untracked_paths={untracked_paths}, "
        f"tracked_review_candidates={tracked_review_candidate_count}, "
        f"tracked_review={tracked_review_status}, status={status}."
    )


def _write_report(report_path: Path, payload: dict[str, Any]) -> Path:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return report_path


def write_worktree_release_blocker_report(
    payload: dict[str, Any],
    report_path: str | Path,
) -> Path:
    return _write_report(Path(report_path), payload)


def execute_worktree_release_blocker(
    request: WorktreeReleaseBlockerRequest,
) -> WorktreeReleaseBlockerResult:
    source_root = _resolve_project_path(request.source_root).resolve()
    output_root = _resolve_project_path(request.output_root).resolve()
    cleanup_report_path = (
        _resolve_project_path(request.cleanup_report).resolve()
        if request.cleanup_report
        else _default_cleanup_report_path(output_root)
    )
    review_report_path = (
        _resolve_project_path(request.review_report).resolve()
        if request.review_report
        else _default_tracked_review_report_path(output_root)
    )
    report_path = (
        _resolve_project_path(request.report_file).resolve()
        if request.report_file
        else _default_blocker_report_path(output_root)
    )
    output_root.mkdir(parents=True, exist_ok=True)

    cleanup_report_command = [
        sys.executable,
        "tools/build_worktree_cleanup_report.py",
        "--source-root",
        str(source_root),
        "--output-root",
        str(output_root),
        "--report-file",
        str(cleanup_report_path),
    ]
    tracked_review_command = [
        sys.executable,
        "tools/build_tracked_artifact_review_report.py",
        "--source-root",
        str(source_root),
        "--cleanup-report",
        str(cleanup_report_path),
        "--report-file",
        str(review_report_path),
    ]

    cleanup_payload: dict[str, Any] = {}
    tracked_review_payload: dict[str, Any] = {}
    failed_steps: list[str] = []
    tracked_review_status = "not_required"

    try:
        _run_checked_command(
            cleanup_report_command,
            cwd=PROJECT_ROOT,
            failure_message="worktree cleanup report failed",
        )
        cleanup_payload = _load_json_payload(cleanup_report_path)
        cleanup_errors = _validate_cleanup_report(cleanup_payload)
        if cleanup_errors:
            failed_steps.append("build_worktree_cleanup_report")
            raise RuntimeError("; ".join(cleanup_errors))

        tracked_review_candidate_count = int(
            cleanup_payload.get("tracked_review_candidate_count", 0)
        )
        if tracked_review_candidate_count > 0:
            _run_checked_command(
                tracked_review_command,
                cwd=PROJECT_ROOT,
                failure_message="tracked artifact review report failed",
            )
            tracked_review_payload = _load_json_payload(review_report_path)
            review_errors = _validate_tracked_review_report(tracked_review_payload)
            if review_errors:
                failed_steps.append("build_tracked_artifact_review_report")
                raise RuntimeError("; ".join(review_errors))
            tracked_review_status = "ready"
    except Exception as exc:
        if not failed_steps:
            failed_steps.append(
                "build_worktree_cleanup_report"
                if not cleanup_payload
                else "build_tracked_artifact_review_report"
            )
        payload = {
            "schema_version": "1.0",
            "artifact_type": "worktree_release_blocker_report",
            "generated_at": _now_iso(),
            "source_root": str(source_root),
            "output_root": str(output_root),
            "report_path": str(report_path),
            "status": "blocked",
            "summary": f"worktree release blocker runner failed: {exc}",
            "clean_worktree": False,
            "total_paths": cleanup_payload.get("total_paths", 0),
            "tracked_paths": cleanup_payload.get("tracked_paths", 0),
            "untracked_paths": cleanup_payload.get("untracked_paths", 0),
            "staged_paths": cleanup_payload.get("staged_paths", 0),
            "unstaged_tracked_paths": cleanup_payload.get(
                "unstaged_tracked_paths", 0
            ),
            "staged_and_unstaged_paths": cleanup_payload.get(
                "staged_and_unstaged_paths", 0
            ),
            "tracked_review_candidate_count": cleanup_payload.get(
                "tracked_review_candidate_count", 0
            ),
            "tracked_review_required": bool(
                cleanup_payload.get("tracked_review_candidate_count", 0)
            ),
            "tracked_review_status": tracked_review_status,
            "cleanup_report_path": str(cleanup_report_path),
            "tracked_review_report_path": str(review_report_path),
            "tracked_review_entries": len(tracked_review_payload.get("entries", [])),
            "cleanup_report_command": " ".join(cleanup_report_command),
            "tracked_review_command": " ".join(tracked_review_command),
            "failed_steps": failed_steps,
            "category_summary": cleanup_payload.get("category_summary", []),
            "status_summary": cleanup_payload.get("status_summary", []),
            "next_step_plan": _coerce_string_list(cleanup_payload.get("next_step_plan")),
        }
        written_report = _write_report(report_path, payload)
        return WorktreeReleaseBlockerResult(
            payload=payload,
            report_path=written_report,
            cleanup_payload=cleanup_payload,
            cleanup_report_path=cleanup_report_path,
            tracked_review_payload=tracked_review_payload,
            tracked_review_report_path=review_report_path,
        )

    clean_worktree = bool(cleanup_payload.get("clean_worktree"))
    total_paths = int(cleanup_payload.get("total_paths", 0))
    tracked_paths = int(cleanup_payload.get("tracked_paths", 0))
    untracked_paths = int(cleanup_payload.get("untracked_paths", 0))
    staged_paths = int(cleanup_payload.get("staged_paths", 0))
    unstaged_tracked_paths = int(cleanup_payload.get("unstaged_tracked_paths", 0))
    staged_and_unstaged_paths = int(
        cleanup_payload.get("staged_and_unstaged_paths", 0)
    )
    tracked_review_candidate_count = int(
        cleanup_payload.get("tracked_review_candidate_count", 0)
    )
    tracked_review_required = tracked_review_candidate_count > 0
    status = "ready" if clean_worktree else "blocked"
    next_step_plan = _coerce_string_list(cleanup_payload.get("next_step_plan"))
    if tracked_review_required:
        next_step_plan.extend(
            item
            for item in _coerce_string_list(tracked_review_payload.get("next_step_plan"))
            if item not in next_step_plan
        )
    payload = {
        "schema_version": "1.0",
        "artifact_type": "worktree_release_blocker_report",
        "generated_at": _now_iso(),
        "source_root": str(source_root),
        "output_root": str(output_root),
        "report_path": str(report_path),
        "status": status,
        "summary": _build_summary(
            status=status,
            clean_worktree=clean_worktree,
            total_paths=total_paths,
            staged_paths=staged_paths,
            unstaged_tracked_paths=unstaged_tracked_paths,
            untracked_paths=untracked_paths,
            tracked_review_candidate_count=tracked_review_candidate_count,
            tracked_review_status=tracked_review_status,
        ),
        "clean_worktree": clean_worktree,
        "total_paths": total_paths,
        "tracked_paths": tracked_paths,
        "untracked_paths": untracked_paths,
        "staged_paths": staged_paths,
        "unstaged_tracked_paths": unstaged_tracked_paths,
        "staged_and_unstaged_paths": staged_and_unstaged_paths,
        "tracked_review_candidate_count": tracked_review_candidate_count,
        "tracked_review_required": tracked_review_required,
        "tracked_review_status": tracked_review_status,
        "tracked_review_entries": len(tracked_review_payload.get("entries", [])),
        "cleanup_report_path": str(cleanup_report_path),
        "tracked_review_report_path": str(review_report_path),
        "cleanup_report_command": " ".join(cleanup_report_command),
        "tracked_review_command": (
            " ".join(tracked_review_command) if tracked_review_required else None
        ),
        "failed_steps": [],
        "category_summary": cleanup_payload.get("category_summary", []),
        "status_summary": cleanup_payload.get("status_summary", []),
        "next_step_plan": next_step_plan,
    }
    written_report = _write_report(report_path, payload)
    return WorktreeReleaseBlockerResult(
        payload=payload,
        report_path=written_report,
        cleanup_payload=cleanup_payload,
        cleanup_report_path=cleanup_report_path,
        tracked_review_payload=tracked_review_payload,
        tracked_review_report_path=review_report_path,
    )


__all__ = [
    "execute_worktree_release_blocker",
    "write_worktree_release_blocker_report",
]
