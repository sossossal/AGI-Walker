# Getting Started

更新日期：`2026-04-08`

本页是 AGI-Walker 的总入口。如果你第一次接触这个仓库，建议先按这里的顺序走，而不是直接翻历史归档文档。

## 你会在这里得到什么

通过这一组入门页面，你会完成四件事：

1. 安装并确认仓库能运行。
2. 通过 CLI 看见 skills 和 workflows。
3. 跑通一次最小 workflow。
4. 启动 Web Panel 和 MCP server。

## 前置条件

- Python `>=3.10`
- 当前仓库已经检出到本地

基础安装：

```bash
pip install -e .
```

开发依赖：

```bash
pip install -e ".[dev]"
```

## 第一步：确认主入口正常

建议依次执行：

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows list
python -m agi_walker.cli doctor
```

如果这些命令都能跑通，说明你已经具备继续探索仓库的最小环境。

## 第二步：执行一个最小 workflow

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/quick_check
```

这条命令适合做第一次验证，因为它：

- 不依赖真实硬件
- 不依赖真实 Godot 进程
- 能验证 workflow 编排和输出目录是否正常

## 第三步：启动 Web Panel

```bash
python -m web_panel.server
```

打开：

```text
http://localhost:8000/static/index.html
```

再查看：

- `/static/workflows.html`
- `/static/nightly.html`
- `/api/system/status`

## 第四步：启动 MCP Server

```bash
agi-walker-mcp
```

或：

```bash
python -m agi_walker.mcp.server
```

当前 MCP 工具面已经覆盖：

- mission execution
- telemetry / RAG
- workflows
- skills
- Godot Agent status / templates / planning / doctor / history

## 建议阅读顺序

1. [README.md](../../README.md)
2. [QUICK_START.md](QUICK_START.md)
3. [BEGINNER_TUTORIAL.md](BEGINNER_TUTORIAL.md)
4. [HANDS_ON_GUIDE.md](HANDS_ON_GUIDE.md)
5. [CLI 指南](../guides/CLI_GUIDE.md)
6. [Web Panel 指南](../guides/WEB_PANEL_GUIDE.md)
7. [MCP 集成说明](../mcp.md)

## 什么时候再看旧文档

只有当你已经跑通当前入口之后，再去看以下内容：

- 架构专题页
- 历史设计方案
- `docs/archive_and_reports/`

这些页面更适合补背景，不适合作为第一次上手的操作手册。
