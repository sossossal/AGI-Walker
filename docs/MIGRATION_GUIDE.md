# Migration Guide

更新日期：`2026-04-08`

本页用于把旧 README / 旧指南里的历史写法迁移到当前仓库真实可用的入口。

## 入口迁移

| 旧写法 | 当前建议 |
| --- | --- |
| `agi_walker skills ...` | `python -m agi_walker.cli skills ...` |
| `agi_walker workflows ...` | `python -m agi_walker.cli workflows ...` |
| `agi_walker skills workflows ...` | 仍可用，但建议优先使用 `python -m agi_walker.cli workflows ...` |
| `python web_panel/server.py` | `python -m web_panel.server` |
| 自定义脚本启动 MCP | `agi-walker-mcp` 或 `python -m agi_walker.mcp.server` |

说明：

- 仓库当前在 `pyproject.toml` 中只显式注册了 `agi-walker-mcp`。
- CLI 的最稳妥入口是模块方式 `python -m agi_walker.cli`。
- Windows 下如需快捷方式，可使用 [scripts/agi_walker.bat](../scripts/agi_walker.bat)。

## CLI 迁移

推荐把旧命令统一替换成下面这些形式：

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli skills info robot-modeling -d
python -m agi_walker.cli skills validate -v
python -m agi_walker.cli workflows list
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume
python -m agi_walker.cli doctor
```

工作流别名关系：

- `python -m agi_walker.cli workflows ...`
- `python -m agi_walker.cli skills workflows ...`

两者当前都会走到同一套 workflow CLI 逻辑。

## MCP 迁移

当前标准 MCP 入口：

```bash
agi-walker-mcp
```

或：

```bash
python -m agi_walker.mcp.server
```

当前暴露的工具以 [docs/mcp.md](mcp.md) 为准，核心分组包括：

- mission / telemetry / rag
- workflows
- skills
- godot-agent

`2026-04-08` 的修复点：

- 修正了 `mcp>=1.27.0` 下的初始化兼容问题。
- 修正了 `python -m agi_walker.mcp.server` 的导入警告路径。

## Web Panel 迁移

当前建议启动方式：

```bash
python -m web_panel.server
```

不要再优先使用旧文档里的零散脚本入口作为主启动方式。

Web 相关配置建议收敛到：

- `deployment/web_panel.env`
- `deployment/web_panel.env.example`
- `AGI_WALKER_WEB_ENV_FILE`

Workflow Web 运行时参数：

- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`

Nightly 相关参数：

- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`
- `AGI_WALKER_GITHUB_WORKFLOW_FILE`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`

Distributed 相关参数：

- `AGI_WALKER_ZENOH_ENDPOINT`
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`

## Godot Backend 迁移

当前 Godot backend 通过环境变量切换：

```text
AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent
```

常用配套变量：

- `AGI_WALKER_GODOT_AGENT_DIR`
- `AGI_WALKER_GODOT_PROJECT_PATH`
- `AGI_WALKER_GODOT_AGENT_HISTORY_FILE`

推荐理解为：

- `legacy`：兼容旧控制流
- `godot-agent`：现代模板 / 计划 / doctor / history 接口

## Smoke / Nightly 迁移

主 smoke 入口：

```bash
python tests/run_smoke_tests.py
```

Godot headless smoke 现在是显式 opt-in：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
```

附加变量：

- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

## 迁移完成的判断标准

如果你现在使用的是以下入口，就说明已经迁移到当前文档口径：

- `python -m agi_walker.cli ...`
- `python -m web_panel.server`
- `agi-walker-mcp`
- `python tests/run_smoke_tests.py`

后续请优先参考：

- [README.md](../README.md)
- [CLI 指南](guides/CLI_GUIDE.md)
- [Web Panel 指南](guides/WEB_PANEL_GUIDE.md)
- [MCP 集成说明](mcp.md)
