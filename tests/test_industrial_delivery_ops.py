from __future__ import annotations

import json
from pathlib import Path

import pytest

from agi_walker.core.api.release_ops_contracts import (
    IndustrialDeliveryRehearsalReportRequest,
)
from agi_walker.ops.industrial_delivery import (
    execute_industrial_delivery_rehearsal_report,
)
from tests.test_industrial_delivery_rehearsal_report import (
    _ready_release_rehearsal_report,
)


def test_execute_industrial_delivery_rehearsal_report_writes_ready_report(
    tmp_path: Path,
) -> None:
    rehearsal_report_path = tmp_path / "release_rehearsal_report.json"
    rehearsal_report_path.write_text(
        json.dumps(_ready_release_rehearsal_report(), ensure_ascii=False, indent=2)
        + "\n",
        encoding="utf-8",
    )
    output_path = tmp_path / "industrial_delivery_rehearsal_report.json"

    result = execute_industrial_delivery_rehearsal_report(
        IndustrialDeliveryRehearsalReportRequest(
            rehearsal_report=str(rehearsal_report_path),
            output=str(output_path),
        )
    )

    assert result.output_path == output_path
    assert result.output_path.exists()
    assert result.rehearsal_report_path == rehearsal_report_path
    assert result.payload["artifact_type"] == "industrial_delivery_rehearsal_report"
    assert result.payload["status"] == "ready"
    assert result.payload["stage_summary"] == {"total": 6, "passed": 6, "failed": 0}
    assert result.payload["external_mainline_input_checklist"]["status"] == "blocked"


def test_execute_industrial_delivery_rehearsal_report_rejects_missing_report(
    tmp_path: Path,
) -> None:
    with pytest.raises(ValueError, match="--rehearsal-report does not exist"):
        execute_industrial_delivery_rehearsal_report(
            IndustrialDeliveryRehearsalReportRequest(
                rehearsal_report=str(tmp_path / "missing.json"),
            )
        )
