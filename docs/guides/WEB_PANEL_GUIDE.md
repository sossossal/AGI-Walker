# AGI-Walker Web Panel Guide

更新日期：`2026-04-13`

本页记录当前 `web_panel` 目录中真实存在的启动方式、静态页面、核心 API 和关键环境变量。

## 启动

推荐启动方式：

```bash
python -m web_panel.server
```

默认监听：

```text
http://localhost:8000
```

如果文档已构建，`/docs` 会挂载到本地构建结果：

```text
http://localhost:8000/docs
```

## 静态页面

当前主要页面：

- `/`
- `/static/index.html`
- `/static/workflows.html`
- `/static/nightly.html`
- `/static/distributed.html`
- `/static/design.html`
- `/static/godot-control.html`
- `/static/instruction-control.html`
- `/static/operator-history.html`

静态资源目录位于 [web_panel/static](../../web_panel/static)。

## 核心 API

### 系统与概览

- `GET /api/system/status`
- `GET /api/capabilities/matrix`
- `GET /api/godot/capabilities`
- `GET /api/distributed/status`
- `GET /api/nightly/regressions`
- `GET /api/skills/catalog`

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

### Workflows

workflow 路由前缀：

```text
/api/workflows
```

主要接口：

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

对应的可视化页面是：

- `/static/workflows.html`

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
- `POST /api/godot/{session_id}/instruction-set`
- `POST /api/godot/{session_id}/simulated-circuit`
- `WS /ws/{session_id}`

当前推荐路径是 `session_bridge`。
现在 session bridge 已经原生接通结构化 `instruction_set` 与
`simulated_circuit` 配置，不再只限于 `step/load_robot`。

如需最小控制与调试面，直接使用：

- `/static/instruction-control.html`

如需独立查看 operator history / 跨 session 总览 / replay，使用：

- `/static/operator-history.html`
- `/static/operator-history-timeline.html`

这两个页面会直接调用：

- `POST /api/godot/instruction-set`
- `POST /api/godot/simulated-circuit`
- `POST /api/godot/{session_id}/instruction-set`
- `POST /api/godot/{session_id}/simulated-circuit`
- `GET /api/godot/{session_id}/status`
- `GET /api/godot/{session_id}/history`
- `GET /api/godot/history`
- `GET /api/godot/history/export`
- `GET /api/godot/history/summary`
- `POST /api/godot/{session_id}/history/replay`
- `POST /api/godot/{session_id}/history/clear`

其中 `GET /api/godot/{session_id}/history`、`GET /api/godot/history` 与
`GET /api/godot/history/export` 还支持：

- `limit`
- `offset`
- `session_id`（仅跨 session 总览接口）
- `session_query`
- `operator`
- `tag`
- `note`
- `kind`
- `route_mode`
- `created_after`
- `created_before`
- `sort_by`
- `sort_order`

`GET /api/godot/history/export` 额外支持：

- `format=json`
- `format=csv`

`/static/instruction-control.html` 当前最小版页面会额外展示：

- `last_instruction_runtime`
- `simulated_circuit_config`
- `simulated_circuit_feedback` 摘要

并支持：

- 连接 `/ws/{session_id}`
- 监听 `telemetry.update`
- 把最新 telemetry 投影到 runtime / circuit 展示区
- 发送 `operator / tag / note` metadata

`/static/operator-history.html` 当前支持：

- 使用服务端持久化 operator history
- 分页、按当前/全部 session 切换、session 搜索、按 `operator / tag / note` 搜索、筛选、排序
- 回填、重放、清空
- JSON / CSV 导出
- 查看聚合摘要：总条目数、session 数、kind 分布、route mode 分布

`/static/operator-history-timeline.html` 当前支持：

- 按 timeline 查看跨 session history
- 使用 `session_query / operator / tag / note / kind / route_mode / created_after / created_before`
- 查看聚合摘要
- JSON / CSV 导出

## 关键环境变量

### Workflow Web 运行时

可通过 `AGI_WALKER_WEB_ENV_FILE` 指向 env 文件；未指定时，会尝试读取：

- `deployment/web_panel.env`
- `deployment/web_panel.env.example`

分页与归档参数：

- `AGI_WALKER_DATABASE_URL`
- `AGI_WALKER_WEB_OUTPUT_ROOT`
- `AGI_WALKER_WEB_ARCHIVE_ROOT`
- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`
- `AGI_WALKER_GODOT_OPERATOR_HISTORY_MAX_ITEMS`
- `AGI_WALKER_GODOT_OPERATOR_HISTORY_PAGE_SIZE`

默认行为：

- 默认数据库 URL：`sqlite+aiosqlite:///./agi_walker.db`
- 默认 workflow 输出根目录：`test_env/web_workflow_runs/`
- 默认 runs 页大小：`20`
- 最大页大小：`100`
- 默认归档最大保留：`200`
- 默认归档最大天数：`30`

源码运行时默认归档目录：

```text
.output/web_workflow_archive/
```

Compose 客户部署默认会通过 `deployment/web_panel.env` 覆盖为：

```text
/var/lib/agi_walker/workflow_runs
/var/lib/agi_walker/workflow_archive
```

### Distributed Monitor

- `AGI_WALKER_ZENOH_ENDPOINT`
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`

默认值：

- Zenoh endpoint：`tcp/127.0.0.1:7447`
- Actor TTL：`30` 秒

### Nightly / GitHub Actions

- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`
- `AGI_WALKER_GITHUB_WORKFLOW_FILE`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`

默认 workflow 文件：

```text
.github/workflows/ci.yml
```

默认缓存 TTL：

```text
300
```

Nightly 跟踪的 job：

- `quality`
- `smoke`
- `distributed-smoke`
- `godot-headless-smoke`
- `ros2-bridge-smoke`

### Godot Agent Backend

- `AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent`
- `AGI_WALKER_GODOT_AGENT_DIR`
- `AGI_WALKER_GODOT_PROJECT_PATH`
- `AGI_WALKER_GODOT_AGENT_HISTORY_FILE`

理解方式：

- `legacy`：旧式控制流兼容模式
- `godot-agent`：模板 / 计划 / history / doctor 的现代 backend

## 常用操作

### 1. 启动 Web Panel

```bash
python -m web_panel.server
```

### 2. 打开主页面和 workflow 页面

```text
http://localhost:8000/static/index.html
http://localhost:8000/static/workflows.html
```

### 3. 查看系统状态

```bash
curl http://localhost:8000/api/system/status
```

### 4. 查看 Godot 能力与 backend 状态

```bash
curl http://localhost:8000/api/godot/capabilities
curl http://localhost:8000/api/godot-agent/status
```

### 5. 查看 workflow 目录和运行记录

```bash
curl http://localhost:8000/api/workflows/
curl "http://localhost:8000/api/workflows/runs?scope=all&limit=20"
```

## 验证

最小 smoke：

```bash
python tests/run_smoke_tests.py
```

可选的 Godot headless smoke：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
```

可选的 ROS2 bridge smoke：

```powershell
$env:AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE='1'
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv
```

常用 headless 调整项：

- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

headless smoke 报告默认输出：

```text
test_env/godot_headless_smoke/headless_smoke_report.json
```

## 排错

### `/api/system/status` 看不到 nightly 数据

检查：

- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`
- `AGI_WALKER_GITHUB_WORKFLOW_FILE`

### `/api/godot-agent/*` 无法返回模板

检查：

- `AGI_WALKER_GODOT_AGENT_BACKEND`
- `AGI_WALKER_GODOT_AGENT_DIR`
- `AGI_WALKER_GODOT_PROJECT_PATH`

### workflow 页面 runs 太多或分页异常

检查：

- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`
