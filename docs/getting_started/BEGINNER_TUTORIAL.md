# Beginner Tutorial

更新日期：`2026-04-08`

这个教程带你第一次完整走通 AGI-Walker 的主路径：skills -> workflows -> Web -> MCP。

## Step 1：确认 CLI 可用

```bash
python -m agi_walker.cli --help
```

然后执行：

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows list
```

你现在应该知道两件事：

- 仓库里有哪些 skills
- 仓库里有哪些 workflows

## Step 2：查看一个 skill

```bash
python -m agi_walker.cli skills info robot-modeling
```

如果想看完整文档正文：

```bash
python -m agi_walker.cli skills info robot-modeling -d
```

这一步的目的不是背内容，而是知道 skill metadata 从哪里来，以及 CLI 如何读取 `SKILL.md`。

## Step 3：跑一次 workflow

执行：

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/beginner_tutorial_run
```

重点观察：

- workflow 名称
- 执行器模式
- 输出根目录
- 每一步的状态
- 最终输出摘要

如果命令完成，你就已经跑通了当前仓库最重要的一条轻量编排路径。

## Step 4：启动 Web Panel

```bash
python -m web_panel.server
```

打开：

```text
http://localhost:8000/static/index.html
```

接着访问：

- `http://localhost:8000/static/workflows.html`
- `http://localhost:8000/api/system/status`
- `http://localhost:8000/api/workflows/`

这一步的目标是把“命令行里的 workflow”与“Web 上的 workflow 视图”对应起来。

## Step 5：启动 MCP Server

新开一个终端：

```bash
agi-walker-mcp
```

或：

```bash
python -m agi_walker.mcp.server
```

你不需要马上写 MCP 客户端代码，先理解这件事就够了：

- CLI 是给人直接操作
- Web 是给浏览器和 HTTP client
- MCP 是给 agent / model client

它们背后共享的是同一批 skills、workflows 和 backend 能力。

## Step 6：跑最小回归

```bash
python -m pytest tests/test_docs_utf8.py tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

如果这一步也通过，说明你当前环境的“文档入口 + MCP 入口”至少是健康的。

## 你已经完成了什么

到这里你已经完成：

1. 验证 CLI 可用。
2. 读取 skill metadata。
3. 跑通最小 workflow。
4. 启动 Web Panel。
5. 启动 MCP server。
6. 跑通最小入口回归。

## 下一步建议

- 想深入命令行：读 [CLI_GUIDE.md](../guides/CLI_GUIDE.md)
- 想深入 Web：读 [WEB_PANEL_GUIDE.md](../guides/WEB_PANEL_GUIDE.md)
- 想理解架构：读 [AGI_WALKER_V3_WHITE_PAPER.md](../AGI_WALKER_V3_WHITE_PAPER.md)
- 想继续做一次带检查点的实操：读 [HANDS_ON_GUIDE.md](HANDS_ON_GUIDE.md)
