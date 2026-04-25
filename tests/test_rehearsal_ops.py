from __future__ import annotations

from pathlib import Path

from agi_walker.core.api.release_ops_contracts import ReleaseRehearsalRequest
from agi_walker.ops.rehearsal import execute_release_rehearsal


def test_execute_release_rehearsal_writes_ready_reports(tmp_path: Path) -> None:
    output_root = tmp_path / "release_rehearsal"

    result = execute_release_rehearsal(
        ReleaseRehearsalRequest(
            version="2026.04.12-rehearsal",
            build_id="release-rehearsal-ops-test",
            output_root=str(output_root),
        )
    )

    assert result.payload["status"] == "passed"
    assert result.report_path == output_root / "release_rehearsal_report.json"
    assert result.report_path.exists()
    assert result.payload["release_gate_status"] == "ready"
    assert result.payload["customer_delivery_status"] == "ready"
    assert result.payload["industrial_delivery_status"] == "ready"
    assert result.gate_status == "ready"
    assert result.tag == "2026.04.12-rehearsal"
    assert result.industrial_delivery_rehearsal_payload["status"] == "ready"
    assert result.industrial_delivery_rehearsal_report_path.exists()
    assert (
        result.payload["control_plane_session"]["engagement_id"]
        == "release-rehearsal-ops-test"
    )
    assert (
        result.payload["control_plane_event_stream"]["event_count"]
        == 3
    )
    assert (
        result.payload["external_mainline_execution_plan"]["control_plane_session"][
            "engagement_id"
        ]
        == "release-rehearsal-ops-test"
    )
    assert (
        result.payload["external_mainline_execution_plan"]["control_plane_event_stream"][
            "event_count"
        ]
        == 3
    )
    assert (
        result.payload["industrial_customer_acceptance_bundle"][
            "external_mainline_input_checklist"
        ]["control_plane_session"]["engagement_id"]
        == "release-rehearsal-ops-test"
    )
    assert (
        result.payload["industrial_customer_acceptance_bundle"][
            "external_mainline_input_checklist"
        ]["control_plane_event_stream"]["event_count"]
        == 3
    )
