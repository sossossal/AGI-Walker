# AGI-Walker Current Status

更新日期：`2026-04-08`

## 摘要

AGI-Walker 当前可用的主入口已经恢复到可读、可执行、可验证的状态。此次修复的重点是：

- 仓库首页 `README.md` 已恢复为正常 UTF-8 文档。
- MCP `stdio` server 已可通过 `agi-walker-mcp` 和 `python -m agi_walker.mcp.server` 启动。
- `mcp>=1.27.0` 下的初始化兼容问题已修复。
- CLI、Workflow、Web Panel 和 Godot Agent 的入口文档已重新对齐到真实代码接口。

## 当前可用入口

- CLI：`python -m agi_walker.cli`
- MCP：`agi-walker-mcp`
- MCP 模块入口：`python -m agi_walker.mcp.server`
- Web Panel：`python -m web_panel.server`
- Smoke：`python tests/run_smoke_tests.py`

## 已验证项

以下命令在 `2026-04-08` 的当前工作区已执行并通过：

- `python -m agi_walker.cli skills list`
- `python -m agi_walker.cli workflows list`
- `python -m agi_walker.cli workflows run robot_creation_pipeline --mock --output-root test_env/doc_check_workflow --resume`
- `python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q`

已验证结果：

- Skills 列表可正常读取，当前检出包含 `robot-modeling`、`parameter-optimizer`、`urdf-generator`、`model-distiller`。
- Workflows 列表可正常读取，当前至少包含 `robot_creation_pipeline` 和 `simulation_ready_robot`。
- `robot_creation_pipeline` 的 mock 执行可完成，输出可写入 `test_env/doc_check_workflow`。
- MCP 相关测试当前为 `7 passed`。

## Web / Godot 状态

Web Panel 的 FastAPI 入口位于 [web_panel/server.py](../web_panel/server.py)，当前公开的主要能力包括：

- 根页面 `/`
- 主控制台 `/static/index.html`
- Workflow 控制台 `/static/workflows.html`
- Nightly 面板 `/static/nightly.html`
- Distributed 面板 `/static/distributed.html`
- Godot 设计页 `/static/design.html`
- Godot 控制页 `/static/godot-control.html`

Godot 集成当前同时支持两种 backend：

- `legacy`
- `godot-agent`

切换环境变量：

```text
AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent
```

## 推荐的配置入口

Web workflow 相关参数优先从以下 env 文件读取：

- `deployment/web_panel.env`
- `deployment/web_panel.env.example`
- 或通过 `AGI_WALKER_WEB_ENV_FILE` 显式指定

当前需要重点关注的环境变量：

- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`
- `AGI_WALKER_ZENOH_ENDPOINT`
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`
- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`
- `AGI_WALKER_GITHUB_WORKFLOW_FILE`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`

Godot headless smoke 相关环境变量：

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE`
- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

## 当前风险与边界

本次修复聚焦于“主入口文档 + MCP 入口 + 核心用户路径”。以下内容仍然可能需要后续清理：

- `docs/archive_and_reports/` 下的历史归档文档
- 部分未纳入本轮修复的旧文档页面
- 依赖真实 Godot 可执行文件的 headless smoke 路径

这意味着：

- 入口文档已经可以作为当前操作手册使用。
- 历史归档文档不应继续作为一线使用说明。

## 推荐阅读顺序

1. [README.md](../README.md)
2. [MCP 集成说明](mcp.md)
3. [CLI 指南](guides/CLI_GUIDE.md)
4. [Web Panel 指南](guides/WEB_PANEL_GUIDE.md)
5. [迁移指南](MIGRATION_GUIDE.md)
