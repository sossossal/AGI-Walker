from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNBOOK = ROOT / "docs/guides/OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
WEB_PANEL_GUIDE = ROOT / "docs/guides/WEB_PANEL_GUIDE.md"
HARDWARE_GUIDE = ROOT / "docs/hardware/HARDWARE_INTEGRATION_GUIDE.md"
README = ROOT / "README.md"


def test_operator_hardware_recovery_runbook_covers_required_flow() -> None:
    content = RUNBOOK.read_text(encoding="utf-8")

    assert "hardware_recovery_operator" in content
    assert "delivery_lead" in content
    assert "rollback_owner" in content
    assert "故障树" in content
    assert "恢复流程" in content
    assert "升级条件" in content
    assert "tools/run_hardware_transport_diagnostics.py" in content
    assert "tools/build_vendor_fault_data_review.py" in content
    assert "tools/build_vendor_data_promotion_checklist.py" in content
    assert "operator history" in content
    assert "evidence" in content


def test_operator_hardware_recovery_runbook_is_linked_from_active_docs() -> None:
    runbook_path = "docs/guides/OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md"

    assert runbook_path in README.read_text(encoding="utf-8")
    assert runbook_path in WEB_PANEL_GUIDE.read_text(encoding="utf-8")
    assert runbook_path in HARDWARE_GUIDE.read_text(encoding="utf-8")
    assert runbook_path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
