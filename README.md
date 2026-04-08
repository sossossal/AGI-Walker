# AGI-Walker

AGI-Walker 是一个面向机器人设计、仿真、工作流编排和 Web/Godot 集成的 Python 平台。仓库当前包含 Skills 系统、Workflow Orchestrator、Web Panel、Godot Agent backend，以及一个可直接接入 MCP 客户端的 `stdio` server。

## 主要能力

- `Skills + Workflows`：用 `SKILL.md` 和 workflow 定义组织机器人建模、参数优化、任务执行等流程。
- `Web Panel`：基于 FastAPI 的控制面板，提供 workflow、服务状态、Godot 会话和 nightly 状态接口。
- `Godot 集成`：同时支持 legacy backend 和 modern `godot-agent` backend。
- `Distributed / Smoke`：仓库内置 CLI、workflow、Web、distributed 和可选的 Godot headless smoke 测试。
- `MCP Server`：通过 `agi_walker.mcp.server` 把任务执行、workflow、skills 和 Godot 能力暴露给 MCP 客户端。

## 安装

要求：

- Python `>=3.10`

基础安装：

```bash
pip install -e .
```

开发依赖：

```bash
pip install -e ".[dev]"
```

## 快速开始

列出 skills：

```bash
python -m agi_walker.cli skills list
```

运行环境自检：

```bash
python -m agi_walker.cli doctor
```

查看 workflow 帮助：

```bash
python -m agi_walker.cli workflows run --help
```

启动 Web Panel：

```bash
python -m web_panel.server
```

默认地址：

```text
http://localhost:8000
```

## MCP 集成

仓库已经提供 MCP `stdio` server。安装后可以直接启动：

```bash
agi-walker-mcp
```

也可以使用模块入口：

```bash
python -m agi_walker.mcp.server
```

当前暴露的工具包括：

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

这些工具覆盖的能力范围：

- 自然语言任务规划与执行
- workflow 查询与执行
- skills 元数据和文档读取
- Godot Agent backend 状态、模板、计划、自检和历史记录

更完整的说明见 [docs/mcp.md](docs/mcp.md)。

## 验证

MCP 相关测试：

```bash
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

仓库 smoke 测试：

```bash
python tests/run_smoke_tests.py
```

可选的 Godot headless smoke：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

## 目录概览

- `agi_walker/`：核心 Python 包，包含 CLI、workflow、skills、integrations 和 MCP server。
- `web_panel/`：FastAPI Web 控制面板和相关 API。
- `godot_project/`：Godot 工程与场景资源。
- `godot_studio_agent/`：Godot Agent 侧角色、路由和工具。
- `tests/`：单元测试、smoke 测试和集成测试。
- `docs/`：架构、迁移、CLI/Web/MCP 指南及当前状态说明。

## 文档入口

- [当前状态](docs/CURRENT_STATUS.md)
- [MCP 集成说明](docs/mcp.md)
- [CLI 指南](docs/guides/CLI_GUIDE.md)
- [Web Panel 指南](docs/guides/WEB_PANEL_GUIDE.md)
- [迁移指南](docs/MIGRATION_GUIDE.md)
- [生产部署 Runbook](PRODUCTION_DEPLOYMENT_RUNBOOK.md)

## License

[MIT](LICENSE)
