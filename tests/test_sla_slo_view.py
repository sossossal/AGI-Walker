from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SLO_VIEW = ROOT / "docs/guides/SLA_SLO_VIEW_20260427.md"
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
MONITORING_BASELINE = ROOT / "docs/guides/MONITORING_ALERTING_BASELINE_20260427.md"
SUPPORT_MATRIX = ROOT / "docs/guides/SUPPORT_MATRIX.md"


def test_sla_slo_view_defines_current_observable_service_targets():
    content = SLO_VIEW.read_text(encoding="utf-8")

    assert "GET /api/system/status" in content
    assert "/api/workflows/runs/{run_id}/status" in content
    assert "/api/godot/history" in content
    assert "HardwareRecovery" in content
    assert "vendor review / promotion checklist" in content
    assert "browser validation closeout" in content
    assert "P1" in content and "P2" in content and "P3" in content
    assert "不承诺 24x7 托管 SLA" in content
    assert "真实设备 smoke" in content
    assert "industrial live evidence" in content


def test_sla_slo_view_is_linked_from_active_docs():
    path = "docs/guides/SLA_SLO_VIEW_20260427.md"

    assert path in README.read_text(encoding="utf-8")
    assert path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert "SLA_SLO_VIEW_20260427.md" in MONITORING_BASELINE.read_text(
        encoding="utf-8"
    )
    assert path in SUPPORT_MATRIX.read_text(encoding="utf-8")
