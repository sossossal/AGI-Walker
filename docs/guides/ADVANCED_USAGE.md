# Advanced Usage

更新日期：`2026-04-08`

本页面向已经熟悉 AGI-Walker 基础入口的用户，说明当前仓库里比较高阶但仍有实际落地点的使用方式。

## 1. Workflow 执行策略

CLI 的推荐入口仍然是：

```bash
python -m agi_walker.cli workflows list
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/advanced_run
```

常用高级选项：

- `--resume`：如果产物已存在，则跳过对应步骤
- `--force`：强制重跑所有步骤
- `--output-root`：把产物隔离到指定目录
- `--mock`：不调用真实 skill 执行器，适合先验路径检查

推荐做法：

- 本地验证优先用 `--mock`
- 需要复现实验结果时显式指定 `--output-root`
- 调试 idempotency 时优先对比 `--resume` 和 `--force`

## 2. Workflow 产物与恢复

当前 `WorkflowOrchestrator` 支持：

- 步骤变量引用
- step artifact JSON 写盘
- state 持久化
- real / mock executor 切换

因此 workflow 更像“可恢复的编排任务”，而不是一次性脚本。对需要反复调试的任务，建议始终隔离输出目录。

## 3. Web Workflow 控制面

Web 端 workflow 路由位于 `/api/workflows/*`。它支持：

- 后台启动任务
- 查询 run 列表
- SSE 事件流
- live log
- artifact 下载
- Godot 同步

关键点：

- `/api/workflows/{name}/run` 是后台执行入口
- `/api/workflows/runs/{run_id}/status` 适合轮询
- `/api/workflows/runs/{run_id}/events` 适合 SSE
- `/api/workflows/runs/{run_id}/godot-sync` 可把推荐 artifact 同步到 Godot

这些接口更适合控制台页面或集成层使用，而不是手工裸调。

## 4. Web 运行时环境文件

workflow Web 运行时支持通过环境文件注入默认参数。当前相关变量包括：

- `AGI_WALKER_WEB_ENV_FILE`
- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`

默认候选文件位于：

- `deployment/web_panel.env`
- `deployment/web_panel.env.example`

## 5. Godot Agent 后端切换

Godot Agent 集成支持两种后端：

- `legacy`
- `godot-agent` / `modern`

切换变量：

```text
AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent
AGI_WALKER_GODOT_AGENT_DIR=...
AGI_WALKER_GODOT_PROJECT_PATH=...
```

默认规则：

- `legacy` 使用仓库内的 `godot_studio_agent`
- `godot-agent` 默认寻找仓库同级目录下的 `godot-agent`

如果你没有外部 `godot-agent` 仓库，不要把 modern backend 写成默认依赖。

## 6. Godot Session Bridge

Web 层同时保留 legacy controller 和 session bridge。当前更值得优先使用的是 session bridge 路径：

- `GET /api/simulation/status`
- `POST /api/godot/{session_id}/launch`
- `POST /api/godot/{session_id}/stop`
- `POST /api/godot/{session_id}/control`
- `WS /ws/{session_id}`

这条路径更适合与 workflow artifacts、telemetry 和多会话调试配合。

## 7. MCP 作为统一能力面

如果你的上层系统是 agent、桌面编排器或其他外部客户端，优先走 MCP，而不是直接耦合 CLI 细节：

```bash
agi-walker-mcp
```

当前 MCP 工具已经覆盖：

- workflow list / get / execute
- skills list / get
- telemetry / RAG
- Godot agent status / templates / plan / doctor / history

## 8. 真实 Godot Headless Smoke

真实 Godot headless smoke 是显式 opt-in，不会默认跑。最小前提：

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1`
- 可用的 Godot 可执行文件
- 真实存在的场景文件

常见变量：

- `GODOT_EXECUTABLE`
- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`

## 9. Distributed 模式

distributed 模式依赖 Zenoh、Docker Compose 和额外服务，不属于默认本地开发路径。优先使用：

```bash
python tests/run_distributed_smoke.py --build --stop-after
```

而不是手工记忆整条 compose 命令。

## 10. 推荐顺序

对大多数高级使用场景，推荐顺序如下：

1. 先用 CLI 跑通 workflow
2. 再接入 Web workflow runs
3. 再接入 Godot session bridge
4. 最后再接入 distributed 和真实 headless smoke

## 结论

AGI-Walker 当前最有价值的高级能力，不是单个实验模块，而是 workflow、Web、Godot 和 MCP 之间的衔接能力。高级用法也应围绕这条主线展开。
