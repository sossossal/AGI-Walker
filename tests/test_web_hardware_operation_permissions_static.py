from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SESSION_BRIDGE = ROOT / "web_panel/godot_session_bridge.py"
INSTRUCTION_CONSOLE = ROOT / "web_panel/static/instruction-control.html"
WEB_PANEL_GUIDE = ROOT / "docs/guides/WEB_PANEL_GUIDE.md"
NEXT_STAGE_PLAN = ROOT / "docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md"


def test_hardware_recovery_routes_have_permission_enforcement() -> None:
    source = SESSION_BRIDGE.read_text(encoding="utf-8")

    assert "WEB_HARDWARE_ROLE_POLICY_PATH" in source
    assert "_roles_from_policy_for_user" in source
    assert "_web_hardware_role_policy_payload" in source
    assert '"/api/godot/hardware/role-policy"' in source
    assert "HARDWARE_OPERATION_ROLE_MATRIX" in source
    assert '"recover_by_fault_class"' in source
    assert '"clear_faults"' in source
    assert '"required_role": "hardware_recovery_operator"' in source
    assert '"required_role_claim": "hardware_recovery_operator"' in source
    assert "_audit_roles_from_token_payload" in source
    assert "hardware_roles" in source
    assert "roles" in source
    assert 'roles.add("hardware_recovery_operator")' in source
    assert "requires hardware_recovery_operator privileges" in source
    assert "_enforce_hardware_operation_permission(" in source
    assert '"permission": permission' in source


def test_instruction_console_requires_token_before_high_risk_actions() -> None:
    html = INSTRUCTION_CONSOLE.read_text(encoding="utf-8")

    assert "recover / clear 必填" in html
    assert "recover / clear-faults 需要 hardware_recovery_operator Bearer token" in html
    assert "hardware-role-policy-table" in html
    assert "/api/godot/hardware/role-policy" in html
    assert "deployment/web_hardware_role_policy.json" in html
    assert "renderHardwareRolePolicy" in html
    assert "requireAuditTokenForHardwareOperation" in html
    assert "需要 hardware_recovery_operator Audit Bearer Token" in html
    assert "requireAuditTokenForHardwareOperation('fault recovery')" in html
    assert "requireAuditTokenForHardwareOperation('clear faults')" in html


def test_permission_model_is_documented_in_active_guides() -> None:
    guide = WEB_PANEL_GUIDE.read_text(encoding="utf-8")
    plan = NEXT_STAGE_PLAN.read_text(encoding="utf-8")

    assert "recover_by_fault_class / clear_faults" in guide
    assert "hardware_recovery_operator" in guide
    assert "permission.required_role=hardware_recovery_operator" in guide
    assert "deployment/web_hardware_role_policy.json" in guide
    assert "/api/godot/hardware/role-policy" in guide
    assert "hardware_recovery_operator` Bearer token 强制权限判定" in plan
    assert "受管角色策略" in plan
