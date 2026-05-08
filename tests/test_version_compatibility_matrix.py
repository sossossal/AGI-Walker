from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MATRIX = ROOT / "docs/guides/VERSION_COMPATIBILITY_MATRIX_20260427.md"
README = ROOT / "README.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"
SUPPORT_MATRIX = ROOT / "docs/guides/SUPPORT_MATRIX.md"
OPERATOR_RUNBOOK = ROOT / "docs/guides/OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md"


def test_version_compatibility_matrix_covers_current_surfaces() -> None:
    content = MATRIX.read_text(encoding="utf-8")

    assert "Python 3.14 alpha" in content
    assert "FastAPI/Pydantic/SQLAlchemy" in content
    assert "Web Panel" in content
    assert "Godot" in content
    assert "ROS2 Humble" in content
    assert "serial_bridge" in content
    assert "socketcan" in content
    assert "pcan" in content
    assert "hardware_recovery_operator" in content
    assert "tools/build_vendor_fault_data_review.py" in content
    assert "tools/build_web_browser_validation_closeout.py" in content


def test_version_compatibility_matrix_is_linked_from_active_docs() -> None:
    matrix_path = "docs/guides/VERSION_COMPATIBILITY_MATRIX_20260427.md"

    assert matrix_path in README.read_text(encoding="utf-8")
    assert matrix_path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
    assert matrix_path in SUPPORT_MATRIX.read_text(encoding="utf-8")
    assert matrix_path in OPERATOR_RUNBOOK.read_text(encoding="utf-8")
