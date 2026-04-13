# Hands-On Guide

更新日期：`2026-04-08`

本页是一个可直接照做的实操清单。目标不是理解所有概念，而是把 AGI-Walker 的主路径真正跑一遍，并留下可检查的产物。

## 场景

你希望完成下面这组动作：

1. 用 CLI 看见当前 skills 和 workflows。
2. 用 mock 模式执行一个 workflow。
3. 用 Web 查看系统状态。
4. 用 MCP 暴露统一工具面。

## 0. 安装

```bash
pip install -e .
pip install -e ".[dev]"
```

## 1. 查看 skills

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli skills list -v
```

检查点：

- 命令能正常返回
- 至少能看到 `robot-modeling`、`parameter-optimizer`、`urdf-generator`

## 2. 查看 workflows

```bash
python -m agi_walker.cli workflows list
```

检查点：

- 至少能看到 `robot_creation_pipeline`

## 3. 执行 workflow

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/hands_on_run
```

检查点：

- 命令返回 `completed`
- 输出目录存在：`test_env/hands_on_run`
- 终端里能看到步骤列表和最终输出摘要

## 4. 启动 Web Panel

```bash
python -m web_panel.server
```

检查点：

- 能打开 `http://localhost:8000/static/index.html`
- `http://localhost:8000/api/system/status` 能返回 JSON
- `http://localhost:8000/api/workflows/` 能列出 workflows

## 5. 启动 MCP Server

新开一个终端：

```bash
agi-walker-mcp
```

或：

```bash
python -m agi_walker.mcp.server
```

检查点：

- 进程能启动
- 不再出现当前 `mcp` 版本下的初始化报错

## 6. 跑文档和 MCP 最小回归

```bash
python -m pytest tests/test_docs_utf8.py tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

检查点：

- docs UTF-8 检查通过
- MCP tool/server 测试通过

## 7. 可选：启用真实 Godot headless smoke

如果你的机器已经具备 Godot 可执行文件，并且希望再往前走一步：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
```

常用附加变量：

- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

## 结果判定

如果下面四项都成立，就说明你已经完成一次有效的 hands-on：

1. CLI 可以列出 skills 和 workflows。
2. mock workflow 可以完成。
3. Web Panel 可以启动并返回系统状态。
4. MCP server 可以启动，且相关测试通过。

## 完成后建议

- 把本次 workflow 输出保留在 `test_env/hands_on_run`
- 继续阅读 [CLI_GUIDE.md](../guides/CLI_GUIDE.md)
- 继续阅读 [WEB_PANEL_GUIDE.md](../guides/WEB_PANEL_GUIDE.md)
- 如果要接 MCP client，继续阅读 [mcp.md](../mcp.md)
