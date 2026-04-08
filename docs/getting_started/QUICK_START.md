# Quick Start

更新日期：`2026-04-08`

目标：用 5 分钟确认 AGI-Walker 的主路径在你的机器上能工作。

## 1. 安装

```bash
pip install -e .
```

如果你还需要测试工具：

```bash
pip install -e ".[dev]"
```

## 2. 列出 skills

```bash
python -m agi_walker.cli skills list
```

预期你会看到类似这些条目：

- `robot-modeling`
- `parameter-optimizer`
- `urdf-generator`
- `model-distiller`

## 3. 列出 workflows

```bash
python -m agi_walker.cli workflows list
```

当前常见条目至少包括：

- `robot_creation_pipeline`
- `simulation_ready_robot`

## 4. 执行一个最小 workflow

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/quick_start_run
```

如果成功，你应该看到：

- 状态 `completed`
- 多个步骤输出
- `test_env/quick_start_run` 下生成产物

## 5. 启动 Web Panel

```bash
python -m web_panel.server
```

打开：

```text
http://localhost:8000/static/index.html
```

## 6. 启动 MCP Server

```bash
agi-walker-mcp
```

或：

```bash
python -m agi_walker.mcp.server
```

## 7. 跑最小回归

```bash
python -m pytest tests/test_docs_utf8.py tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

## 如果你只记住三条命令

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/quick_start_run
python -m web_panel.server
```

## 下一步

- 想继续看命令行：读 [CLI_GUIDE.md](../guides/CLI_GUIDE.md)
- 想继续看 Web：读 [WEB_PANEL_GUIDE.md](../guides/WEB_PANEL_GUIDE.md)
- 想继续看 MCP：读 [mcp.md](../mcp.md)
- 想按步骤做第一次完整操作：读 [BEGINNER_TUTORIAL.md](BEGINNER_TUTORIAL.md)
