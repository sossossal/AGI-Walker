# API Reference

更新日期：`2026-04-08`

本页提供 AGI-Walker 当前一线可用接口的总览。它不是自动生成的全量 API 文档，而是面向实际使用的稳定入口索引。

## 约定

- CLI 入口优先使用 `python -m agi_walker.cli`
- MCP 入口优先使用 `agi-walker-mcp`
- Web API 基础地址默认是 `http://localhost:8000`
- 更细的 MCP 工具说明见 [mcp.md](mcp.md)

## Python 入口

### Skills

```python
from agi_walker.skills_loader import get_skills_loader
```

签名：

```python
get_skills_loader(skills_dir: str = "agi_walker/skills") -> SkillsLoader
```

用途：

- 扫描并加载 skill metadata
- 读取 `SKILL.md`
- 按名称、分类和关键词查找 skills

常用能力：

- `get_skills_list()`
- `get_skill(name)`
- `get_categories()`
- `search_skills(query)`
- `get_skill_doc(name)`

### Workflows

```python
from agi_walker.workflow_orchestrator import get_workflow_orchestrator
```

签名：

```python
get_workflow_orchestrator()
```

常见用途：

- `list_workflows()`
- `get_workflow(name)`
- `execute_workflow(name, parameters=None, use_real=True)`
- `validate_workflow(name)`

### MCP Provider

```python
from agi_walker.core.api.mcp_tools import MCPToolProvider
```

签名：

```python
MCPToolProvider(
    orchestrator=None,
    planner=None,
    monitor=None,
    knowledge_base=None,
    skills_loader=None,
    godot_backend_factory=None,
)
```

用途：

- 作为 MCP server 的能力桥接层
- 对 workflow、skills、RAG、telemetry 和 Godot backend 做统一封装

主要方法：

- `execute_mission(instruction)`
- `get_telemetry()`
- `query_rag(orient, top_k=1)`
- `list_workflows()`
- `get_workflow(name)`
- `execute_workflow(name, parameters=None, use_real=None)`
- `list_skills()`
- `get_skill(name, include_doc=False)`
- `get_godot_agent_status()`
- `list_godot_templates()`
- `plan_godot_command(command, context=None, project_path=None)`
- `doctor_godot_agent(project_path=None)`
- `get_godot_history(limit=20)`

### MCP Server

```python
from agi_walker.mcp.server import create_server, build_initialization_options
```

签名：

```python
create_server(provider: MCPToolProvider | None = None) -> Server
build_initialization_options(server: Server) -> InitializationOptions
```

用途：

- 创建一个标准 MCP `stdio` server
- 构造与当前 `mcp>=1.27.0` 兼容的初始化参数

### Environment Doctor

```python
from agi_walker.utils.doctor import run_diagnostics
```

签名：

```python
run_diagnostics() -> Dict[str, Any]
```

用途：

- 检查核心依赖
- 检查可选训练依赖
- 检查默认端口占用情况
- 检查关键目录读写权限

## MCP Tools

当前通过 MCP 暴露的工具：

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

启动方式：

```bash
agi-walker-mcp
```

或：

```bash
python -m agi_walker.mcp.server
```

详细说明见 [mcp.md](mcp.md)。

## Web API

### 系统级接口

- `GET /api/system/status`
- `GET /api/capabilities/matrix`
- `GET /api/godot/capabilities`
- `GET /api/distributed/status`
- `GET /api/nightly/regressions`
- `GET /api/skills/catalog`
- `GET /metrics`
- `GET /openapi.json`

### Auth

- `POST /api/auth/register`
- `POST /api/auth/login`
- `GET /api/auth/me`

### Tasks / Services

- `GET /api/tasks`
- `POST /api/tasks`
- `GET /api/tasks/{task_id}`
- `PUT /api/tasks/{task_id}`
- `DELETE /api/tasks/{task_id}`
- `POST /api/generate_robot`
- `GET /api/skills/list`
- `POST /api/skills/model`
- `POST /api/skills/optimize`
- `POST /api/skills/export-urdf`
- `POST /api/skills/pipeline`
- `POST /api/sim2real/analyze`
- `GET /api/parts/market`
- `POST /api/parts/import`

### Workflows

主要 workflow 路由：

- `GET /api/workflows/`
- `GET /api/workflows/{name}`
- `POST /api/workflows/{name}/run`
- `GET /api/workflows/runs`
- `GET /api/workflows/runs/{run_id}`
- `GET /api/workflows/runs/{run_id}/status`
- `GET /api/workflows/runs/{run_id}/events`
- `POST /api/workflows/runs/{run_id}/cancel`
- `GET /api/workflows/runs/{run_id}/live-log`
- `GET /api/workflows/runs/{run_id}/log`
- `GET /api/workflows/runs/{run_id}/artifacts/{artifact_index}`
- `POST /api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load`
- `POST /api/workflows/runs/{run_id}/godot-sync`
- `GET /api/workflows/logs`
- `GET /api/workflows/logs/{filename}`

### Godot Agent

- `GET /api/godot-agent/status`
- `GET /api/godot-agent/templates`
- `GET /api/godot-agent/templates/{template_id}`
- `POST /api/godot-agent/plan`
- `GET /api/godot-agent/doctor`
- `GET /api/godot-agent/history`
- `POST /api/godot-agent/launch`

兼容别名：

- `GET /api/godot_skills/list`
- `POST /api/godot_skills/apply`

### Legacy Godot / Session Bridge

legacy controller：

- `POST /api/godot/connect`
- `POST /api/godot/disconnect`
- `GET /api/godot/status`
- `POST /api/godot/load-robot`
- `POST /api/godot/start`
- `POST /api/godot/stop`
- `POST /api/godot/update-params`

session bridge：

- `GET /api/simulation/status`
- `GET /api/godot/{session_id}/status`
- `POST /api/godot/{session_id}/launch`
- `POST /api/godot/{session_id}/stop`
- `POST /api/godot/{session_id}/control`
- `WS /ws/{session_id}`

## 静态页面

当前常用页面：

- `/`
- `/static/index.html`
- `/static/workflows.html`
- `/static/nightly.html`
- `/static/distributed.html`
- `/static/design.html`
- `/static/godot-control.html`

## 验证命令

```bash
python -m pytest tests/test_docs_utf8.py tests/test_mcp_tools.py tests/test_mcp_server.py -q
python tests/run_smoke_tests.py
```

## 相关文档

- [README.md](../README.md)
- [CURRENT_STATUS.md](CURRENT_STATUS.md)
- [CLI 指南](guides/CLI_GUIDE.md)
- [Web Panel 指南](guides/WEB_PANEL_GUIDE.md)
- [MCP 集成说明](mcp.md)
