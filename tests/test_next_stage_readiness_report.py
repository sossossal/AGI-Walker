from datetime import datetime
from pathlib import Path

from tools.build_next_stage_readiness_report import (
    DEFAULT_ARTIFACTS,
    build_next_stage_readiness_report,
    main,
)


ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"


def test_next_stage_readiness_report_tracks_all_route_closeouts() -> None:
    expected = {
        "hardware_live_closeout",
        "ros2_typed_idl_cutover",
        "operator_delivery_checklist",
        "industrial_live_evidence_archive",
        "vendor_fault_sample_closeout",
        "vendor_fault_data_review",
        "vendor_data_promotion",
        "web_browser_evidence_pack",
    }

    assert set(DEFAULT_ARTIFACTS) == expected
    report = build_next_stage_readiness_report()
    assert report["schema_version"] == "1.0"
    generated_at = datetime.fromisoformat(report["generated_at"])
    assert generated_at.tzinfo is not None
    assert {item["id"] for item in report["artifacts"]} == expected
    assert report["summary"]["artifact_count"] == len(expected)


def test_next_stage_readiness_report_fails_closed_with_current_missing_evidence(
    tmp_path: Path,
) -> None:
    output = tmp_path / "next_stage_readiness_report.json"

    exit_code = main(["--output", str(output)])

    assert exit_code == 1
    content = output.read_text(encoding="utf-8")
    assert '"status": "blocked"' in content
    assert '"generated_at":' in content
    assert "hardware_live_closeout" in content
    assert "web_browser_evidence_pack" in content


def test_next_stage_readiness_report_includes_actionable_blocker_details() -> None:
    report = build_next_stage_readiness_report()

    assert report["summary"]["blocker_detail_count"] == len(report["blockers"])
    details = {item["id"]: item for item in report["blocker_details"]}
    assert set(details) == set(report["blockers"])
    assert details["vendor_fault_data_review"]["blockers"]
    assert details["vendor_data_promotion"]["blockers"] or details[
        "vendor_data_promotion"
    ]["blocked_steps"]
    assert details["industrial_live_evidence_archive"]["blockers"] or details[
        "industrial_live_evidence_archive"
    ]["warnings"]
    assert details["hardware_live_closeout"]["next_actions"]


def test_next_stage_readiness_report_builds_ordered_action_plan() -> None:
    report = build_next_stage_readiness_report()

    assert [item["artifact_id"] for item in report["action_plan"]] == report["blockers"]
    vendor_review = next(
        item
        for item in report["action_plan"]
        if item["artifact_id"] == "vendor_fault_data_review"
    )
    assert vendor_review["issues"]
    assert vendor_review["primary_next_action"]
    promotion = next(
        item
        for item in report["action_plan"]
        if item["artifact_id"] == "vendor_data_promotion"
    )
    assert promotion["issues"]
    assert report["next_actions"][1].startswith("hardware_live_closeout:")


def test_next_stage_readiness_report_marks_current_remaining_actions_external() -> None:
    report = build_next_stage_readiness_report()

    assert report["summary"]["code_or_config_action_count"] == 0
    assert report["summary"]["external_input_action_count"] == len(report["action_plan"])
    assert all(item["requires_real_input"] is True for item in report["action_plan"])
    assert {item["execution_scope"] for item in report["action_plan"]} == {
        "external_input"
    }


def test_next_stage_readiness_report_is_documented() -> None:
    tool = "tools/build_next_stage_readiness_report.py"
    report = "test_env/next_stage/next_stage_readiness_report.json"

    assert tool in README.read_text(encoding="utf-8")
    assert tool in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert report in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
