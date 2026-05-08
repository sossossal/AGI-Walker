import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / "deployment/web_hardware_role_policy.json"
SESSION_BRIDGE = ROOT / "web_panel/godot_session_bridge.py"
INSTRUCTION_CONSOLE = ROOT / "web_panel/static/instruction-control.html"
README = ROOT / "README.md"
WEB_PANEL_GUIDE = ROOT / "docs/guides/WEB_PANEL_GUIDE.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"


def test_web_hardware_role_policy_defines_recovery_operator() -> None:
    payload = json.loads(POLICY.read_text(encoding="utf-8"))

    assert payload["schema_version"] == "1.0"
    role = payload["roles"]["hardware_recovery_operator"]
    assert "recover_by_fault_class" in role["allowed_operations"]
    assert "clear_faults" in role["allowed_operations"]
    assert "hardware_recovery_operator" in payload["admin_roles"]
    assert "hardware_recovery_operator" in payload["users"]["field-operator"]


def test_web_hardware_role_policy_is_wired_to_api_and_ui() -> None:
    source = SESSION_BRIDGE.read_text(encoding="utf-8")
    html = INSTRUCTION_CONSOLE.read_text(encoding="utf-8")

    assert "deployment\" / \"web_hardware_role_policy.json" in source
    assert "_roles_from_policy_for_user(user.username" in source
    assert "@router.get(\"/api/godot/hardware/role-policy\")" in source
    assert "hardware-role-policy-table" in html
    assert "renderHardwareRolePolicy" in html
    assert "loadHardwareRolePolicy()" in html


def test_web_hardware_role_policy_is_documented() -> None:
    path = "deployment/web_hardware_role_policy.json"

    assert path in README.read_text(encoding="utf-8")
    assert path in WEB_PANEL_GUIDE.read_text(encoding="utf-8")
    assert path in NEXT_STAGE_PLAN.read_text(encoding="utf-8")
