# AGI-Walker V3 White Paper

更新日期：`2026-04-08`

## 摘要

AGI-Walker 是一个面向机器人设计、工作流编排、Web 控制面板、Godot 集成和 MCP 接入的工程化平台。它不是单一的“机器人算法库”，而是一套把 skills、workflow、Web API、Godot backend、nightly / smoke 校验串起来的操作系统式工程骨架。

当前仓库中的核心目标是：

- 用结构化 skill 和 workflow 组织机器人相关任务
- 用 Web Panel 暴露状态、运行和诊断能力
- 用 Godot backend 连接可视化场景与控制流程
- 用 MCP `stdio` server 把能力开放给外部 agent / client
- 用 smoke 和专项回归守住主入口

## 设计目标

### 1. 统一入口

项目希望把多个传统上分散的界面统一起来：

- CLI
- Web API
- 静态控制台页面
- Godot backend
- MCP tools

统一入口之后，用户可以围绕同一套 workflow、同一套 skill metadata 和同一套诊断输出工作，而不是在多个孤立脚本之间来回切换。

### 2. 工程化优先

AGI-Walker 当前更强调工程路径而不是论文式功能堆叠。仓库里的关键能力优先围绕这些问题展开：

- 如何列出当前 skills 和 workflows
- 如何稳定地执行一条 workflow
- 如何在 Web 上看见 runs、artifacts、日志和 Godot 同步入口
- 如何把这些能力暴露给 MCP 客户端
- 如何通过 smoke / regression 快速知道主路径是否损坏

### 3. 渐进式集成

系统支持从轻量模式到重型模式逐步扩展：

- 只跑 CLI
- 跑 workflow mock 执行
- 启动 Web Panel
- 启动 Godot backend
- 接入 nightly / distributed / headless smoke

这让新用户不需要一次性具备完整仿真或硬件环境，也能先验证主路径。

## 核心组成

### Skills Layer

skills 通过 `SKILL.md` 描述自身元数据、能力边界、输入输出和依赖。当前仓库中可被 `SkillsLoader` 扫描到的 skills 包括：

- `robot-modeling`
- `parameter-optimizer`
- `urdf-generator`
- `model-distiller`

skills 的作用不是直接替代业务逻辑，而是作为：

- 人类可读的能力目录
- CLI / Web / MCP 共享的元数据源
- workflow 编排时的稳定能力单元

### Workflow Layer

workflow orchestrator 负责把多步任务串成一条可执行流水线。当前一线使用方式包括：

- CLI 执行：`python -m agi_walker.cli workflows ...`
- Web 执行：`/api/workflows/*`
- MCP 封装：`workflow_*`

workflow 是 AGI-Walker 的主编排面。技能、产物、日志和 Godot 同步都围绕它组织。

### MCP Layer

MCP server 通过 `agi_walker.mcp.server` 暴露统一工具面。当前工具覆盖：

- mission planning
- telemetry / RAG
- workflows
- skills
- Godot Agent status / templates / planning / doctor / history

它的意义在于把仓库内部能力变成外部 agent 可调用的稳定接口，而不是要求每个客户端自己了解 CLI 或 Web 细节。

### Web Layer

Web Panel 基于 FastAPI，既提供 API，也提供静态控制台页面。当前关键页面包括：

- `/static/index.html`
- `/static/workflows.html`
- `/static/nightly.html`
- `/static/distributed.html`
- `/static/design.html`
- `/static/godot-control.html`

Web 层的职责不是“做所有事情”，而是：

- 展示系统状态
- 提供 workflow runs 和 artifacts 的可视化入口
- 暴露 Godot 能力、nightly 状态和 distributed 状态

### Godot Layer

Godot 集成当前支持两种 backend：

- `legacy`
- `godot-agent`

统一切换方式：

```text
AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent
```

系统同时保留：

- legacy controller 路由
- session bridge 路由
- workflow 到 Godot 的桥接接口

当前推荐路径是 `session_bridge`。

## 操作模型

### 模式 1: CLI-first

适合验证仓库能否工作：

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows list
python -m agi_walker.cli doctor
```

### 模式 2: Workflow-first

适合验证编排与输出目录：

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/quick_check
```

### 模式 3: Web-first

适合查看 runs、nightly、Godot 状态：

```bash
python -m web_panel.server
```

### 模式 4: Agent / MCP-first

适合给外部 agent 或 MCP client 接入：

```bash
agi-walker-mcp
```

或：

```bash
python -m agi_walker.mcp.server
```

## 质量策略

AGI-Walker 当前的质量策略不是“所有页面和模块都已经完美一致”，而是先守住高频主路径：

- README
- 顶层 docs 入口页
- CLI 入口
- MCP 入口
- Web Panel 主页面和关键 API
- smoke / docs / MCP 回归测试

当前已落地的回归重点：

- `tests/test_docs_utf8.py`
- `tests/test_mcp_tools.py`
- `tests/test_mcp_server.py`
- `tests/run_smoke_tests.py`

## 当前边界

当前仓库仍然存在两类现实边界：

### 1. 历史文档债务

虽然本轮已经修复了一批一线入口页，但仍有不少专题文档和历史归档页还没有全部清理。

### 2. 真实环境依赖

某些路径依赖真实运行环境，例如：

- Godot headless smoke
- distributed smoke
- 外部 `godot-agent` 目录
- GitHub Actions nightly 数据源

因此项目的默认策略是：

- 先提供可本地验证的轻量入口
- 再逐步打开更重的集成模式

## 结论

AGI-Walker V3 的实际价值不在于某个单点算法，而在于它把机器人相关的多种操作界面统一到了同一个工程骨架里：

- skill metadata
- workflow 编排
- Web 控制台
- Godot backend
- MCP tools
- smoke / regression

对当前仓库来说，最重要的不是继续堆叠名词，而是继续收紧这条主路径，让入口、文档、代码和回归测试保持一致。
