"""
Regression Unit Tests for GodotAgent Adapter Boundary.
"""

import os
import pytest
from unittest.mock import patch, MagicMock

from agi_walker.integrations.godot_agent.adapter import LegacyGodotAgentAdapter

def test_adapter_missing_dir_graceful_degradation():
    """
    用不存在的路径引发内部 ImportError 并确保 API 全面返回失败但不崩溃。
    主要模拟 Github Action 或客户机器上未连带 GodotStudio 目录时。
    """
    fake_dir = "/tmp/fake_dir_12345_no_exist"
    adapter = LegacyGodotAgentAdapter(fake_dir)
    
    assert adapter.router is None, "不具有合法系统路径不应被挂载"
    
    # 动作 1：命令抛投
    res_cmd = adapter.execute_command("Hello")
    assert res_cmd.get("status") == "error"
    assert "missing" in res_cmd.get("message").lower() or "not imported" in res_cmd.get("message").lower()
    
    # 动作 2：多管线抛投
    res_pipe = adapter.execute_pipeline(["Do This"])
    assert isinstance(res_pipe, list)
    assert len(res_pipe) > 0
    assert res_pipe[0].get("status") == "error"
    
    # 动作 3：获取角色支持列表
    res_roles = adapter.get_roles_info()
    assert isinstance(res_roles, list)
    assert len(res_roles) == 0

def test_adapter_config_path_injection(monkeypatch):
    """
    验证从 Adapter 向下传递了拼接出的正确 config.yaml 绝对地址。
    防止底层的 `config_path="config.yaml"` 乱撞当前工作目录引发参数空白。
    """
    mock_router_instance = MagicMock()
    MockRouterClass = MagicMock(return_value=mock_router_instance)
    
    # 无需真的载入庞大耦合组件，仅 Mock 拦截
    import sys
    import types
    fake_module = types.ModuleType("agent_system.router")
    fake_module.GodotStudioRouter = MockRouterClass
    monkeypatch.setitem(sys.modules, "agent_system.router", fake_module)
    
    dir_path = "/mock_godot_project/agent"
    adapter = LegacyGodotAgentAdapter(dir_path)
    
    expected_config_path = os.path.join(dir_path, "config.yaml")
    MockRouterClass.assert_called_once_with(config_path=expected_config_path)
    assert adapter.router is mock_router_instance

def test_adapter_pipeline_context_passing(monkeypatch):
    """
    确保 Web 发送给 Pipeline 的参数能够被无损传达进 GodotStudio_Router.execute_pipeline(..., context=xxx)。
    """
    mock_router_instance = MagicMock()
    mock_router_instance.execute_pipeline.return_value = [{"success": True, "step": 1}]
    
    MockRouterClass = MagicMock(return_value=mock_router_instance)
    
    import sys
    import types
    fake_module = types.ModuleType("agent_system.router")
    fake_module.GodotStudioRouter = MockRouterClass
    monkeypatch.setitem(sys.modules, "agent_system.router", fake_module)
    
    adapter = LegacyGodotAgentAdapter("/any/dir")
    
    mock_context = {"scene_mode": "sandbox", "headless": True}
    res = adapter.execute_pipeline(["Create floor"], context=mock_context)
    
    assert len(res) == 1
    assert res[0]["success"] is True
    
    # 最核心的断言：必须带有透传字典
    mock_router_instance.execute_pipeline.assert_called_once_with(
        ["Create floor"], 
        context=mock_context
    )
