from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BASELINE = ROOT / "docs/guides/MONITORING_ALERTING_BASELINE_20260427.md"
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
SUPPORT_MATRIX = ROOT / "docs/guides/SUPPORT_MATRIX.md"
OPERATOR_RUNBOOK = ROOT / "docs/guides/OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md"


def test_monitoring_alerting_baseline_covers_required_surfaces():
    content = BASELINE.read_text(encoding="utf-8")

    assert "/api/system/status" in content
    assert "/api/workflows/runs/{run_id}/status" in content
    assert "/api/godot/history" in content
    assert "hardware_fault_summary" in content
    assert "run_hardware_transport_diagnostics.py" in content
    assert "tools/build_vendor_fault_data_review.py" in content
    assert "tools/build_web_browser_validation_closeout.py" in content
    assert "P1" in content and "P2" in content and "P3" in content
    assert "30 分钟内" in content
    assert "4 小时内" in content
    assert "hardware_recovery_operator" in content


def test_monitoring_alerting_baseline_is_linked_from_active_docs():
    path = "docs/guides/MONITORING_ALERTING_BASELINE_20260427.md"

    assert path in README.read_text(encoding="utf-8")
    assert path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert path in SUPPORT_MATRIX.read_text(encoding="utf-8")
    assert path in OPERATOR_RUNBOOK.read_text(encoding="utf-8")
