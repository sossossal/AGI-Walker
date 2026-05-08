from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKLIST = ROOT / "docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md"
INSTRUCTION_CONSOLE = ROOT / "web_panel/static/instruction-control.html"
OPERATOR_HISTORY = ROOT / "web_panel/static/operator-history.html"
OPERATOR_TIMELINE = ROOT / "web_panel/static/operator-history-timeline.html"


def test_browser_validation_checklist_covers_current_static_surfaces() -> None:
    checklist = CHECKLIST.read_text(encoding="utf-8")
    instruction_console = INSTRUCTION_CONSOLE.read_text(encoding="utf-8")
    operator_history = OPERATOR_HISTORY.read_text(encoding="utf-8")
    operator_timeline = OPERATOR_TIMELINE.read_text(encoding="utf-8")

    assert 'id="build-recovery-plan-button"' in instruction_console
    assert 'id="recover-faults-button"' in instruction_console
    assert 'id="clear-faults-button"' in instruction_console
    assert '"simulated_circuit_command_batch"' in instruction_console
    assert '"target_angle": 0.3' in instruction_console
    assert "recovery plan" in checklist
    assert "clear faults" in checklist

    assert 'id="history-note-exact-filter"' in operator_history
    assert "note_exact" in checklist
    assert "replay" in checklist

    assert 'id="timeline-note-exact-filter"' in operator_timeline
    assert 'id="timeline-clear-compare-button"' in operator_timeline
    assert "timeline compare" in checklist.lower()
    assert "JSON / CSV" in checklist


def test_browser_validation_checklist_keeps_live_risk_explicit() -> None:
    checklist = CHECKLIST.read_text(encoding="utf-8")

    assert "不接真实硬件" in checklist
    assert "不执行现场恢复动作" in checklist
    assert "不伪造浏览器通过态" in checklist
