# MCP 集成

AGI-Walker 现在提供一个可直接被 MCP 客户端接入的 `stdio` server。

## 安装

```bash
pip install -e .
```

安装后会得到一个命令行入口：

```bash
agi-walker-mcp
```

也可以直接使用模块入口：

```bash
python -m agi_walker.mcp.server
```

## 已暴露工具

- `mission_execute`
- `robot_telemetry`
- `rag_query`
- `workflows_list`
- `workflow_get`
- `workflow_execute`
- `skills_list`
- `skill_get`
- `godot_agent_status`
- `godot_agent_templates`
- `godot_agent_plan`
- `godot_agent_doctor`
- `godot_agent_history`
- `capability_matrix_get`

## 工具能力范围

- `mission_execute`：把自然语言任务交给 `SimplePlanner` 和 `WorkflowOrchestrator`
- `workflow_*`：读取和执行内置 workflow
- `skills_*`：读取 `SKILL.md` 元数据和正文
- `godot_agent_*`：读取 Godot Agent backend 状态、模板、计划、自检和历史
- `capability_matrix_get`：读取发布面 capability matrix、契约版本和已知限制

## 验证

建议至少运行：

```bash
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
ruff check agi_walker/core/api/mcp_tools.py agi_walker/mcp/server.py tests/test_mcp_tools.py tests/test_mcp_server.py
```
