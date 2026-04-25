"""Deterministic orchestration for industrial delivery rehearsal reports."""

from __future__ import annotations

import json
from pathlib import Path

from agi_walker.core.api.release_contracts import (
    build_industrial_delivery_rehearsal_report_artifact,
    write_industrial_delivery_rehearsal_report_artifact,
)
from agi_walker.core.api.release_ops_contracts import (
    IndustrialDeliveryRehearsalReportRequest,
    IndustrialDeliveryRehearsalReportResult,
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


def execute_industrial_delivery_rehearsal_report(
    request: IndustrialDeliveryRehearsalReportRequest,
) -> IndustrialDeliveryRehearsalReportResult:
    rehearsal_report_path = _resolve_project_path(request.rehearsal_report).resolve()
    if not rehearsal_report_path.is_file():
        raise ValueError(f"--rehearsal-report does not exist: {rehearsal_report_path}")

    try:
        release_rehearsal_report = json.loads(
            rehearsal_report_path.read_text(encoding="utf-8")
        )
    except Exception as exc:
        raise ValueError(f"--rehearsal-report is unreadable: {exc}") from exc

    output_path = (
        _resolve_project_path(request.output).resolve()
        if request.output
        else rehearsal_report_path.parent / "industrial_delivery_rehearsal_report.json"
    )
    payload = build_industrial_delivery_rehearsal_report_artifact(
        release_rehearsal_report=release_rehearsal_report,
        release_rehearsal_report_path=rehearsal_report_path,
    )
    written_report = write_industrial_delivery_rehearsal_report_artifact(
        payload,
        output_path,
    )
    return IndustrialDeliveryRehearsalReportResult(
        payload=payload,
        output_path=Path(written_report),
        rehearsal_report_path=rehearsal_report_path,
    )


__all__ = ["execute_industrial_delivery_rehearsal_report"]
