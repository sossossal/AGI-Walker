# Web 控制面板指南

AGI-Walker 提供了一个基�?Web 的控制面板，用于管理任务、查看页面资源，并调�?Web-Godot 集成相关功能�?
## 启动面板

在项目根目录下运行：
```bash
python web_panel/server.py
```
也可以使用模块方式：

```bash
python -m web_panel.server
```

启动后访�?[http://localhost:8000](http://localhost:8000)�?
说明�?
- `web_panel/` 内已经包�?FastAPI 服务、静态页面和 WebSocket 协议处理代码�?- 直接启动服务不等于完�?Godot 联动已验证；Godot 侧仍需要额外环境与联调�?- 在显式配置可�?Godot 环境后，`session_bridge` 官方链路已经通过真实 headless smoke 验证�?- 如果�?Windows 终端中遇到编码问题，优先使用 UTF-8 终端�?- Web 服务的系统状态接口现在会额外报告 `distributed_monitor` 能力状态，用于说明当前运行环境是否具备 Zenoh 分布式监控能力�?- 如果配置�?GitHub Actions nightly 状态来源，首页系统状态卡还会显示 nightly 专项回归摘要�?
## 当前页面入口

- `http://localhost:8000/static/index.html`
  主控制台，包含任务、Godot 相关入口�?workflow 控制台跳转�?- `http://localhost:8000/static/workflows.html`
  workflow 控制台，面向 workflow 执行、状态查看和历史检索�?- `http://localhost:8000/static/nightly.html`
  nightly 运维页，面向专项回归、artifact 与本地复现入口�?
## 当前可确认的内容

- Web 服务入口存在：`web_panel/server.py`
- 协议处理模块存在：`web_panel/ws_protocol.py`
- 静态页面资源存在：`web_panel/static/`
- 部分页面和接口面�?Web-Godot 联调
- workflow 控制台支持后�?workflow run、SSE 状态流、实时日志、故障诊断和历史 runs 检�?- workflow 控制台现在支持把机器人配置产物通过正式后端路由送往 Godot
- `session_bridge` 现在�?workflow -> Godot 官方推荐路径，legacy controller 主要保留兼容用�?- Godot Agent 集成层现在支�?backend 工厂切换，可�?legacy `godot_studio_agent` 与外�?`godot-agent` 后端之间切换

## Workflow 控制�?
`/static/workflows.html` 当前已经具备一条相对完整的 Web workflow 闭环�?
- 列出可用 workflows
- �?`real/mock` 模式触发后台 workflow
- 显式选择执行策略：`force` / `resume`
- 使用 `output_root` 隔离产物目录
- 查看步级进度、实时日志、故障诊断和最�?JSON 结果
- 下载 artifacts、workflow log 和完整实时日�?- �?`robot_config` �?artifacts 送往 `legacy_controller` �?`session_bridge`
- 使用 `/api/workflows/runs/{run_id}/godot-sync` 自动选择推荐机器人配置产物并送往 Godot
- 记录最近一�?Godot 交付结果，并�?run 详情里展�?transport/session/schema 状�?- run 详情现在还会显示 Godot session 实时状态、失败阶段、重试提示和重试入口
- �?workflow 完成后可由前端自动触�?“送往 Godot�?- 通过 `scope/status/mode/date/text/only_failures` 检索当�?runs 与归�?runs
- �?runs 列表分页浏览

说明�?
- 当前 workflow run 仍由 Web 服务进程管理，适合调试、演示和最小操作闭环�?- 在显式配�?Godot 环境后，workflow -> `session_bridge` -> Godot 这条官方主线已经达到 V1 验收预期�?- 这仍不等于“跨平台、零配置、任意外�?Godot 环境”都已经达到统一稳定产品状态�?
### Runs 列表分页

`GET /api/workflows/runs` 现在支持�?
- `page`
- `page_size`
- 兼容旧参�?`limit`，但它现在等价于 `page_size`

返回结果会包含：

- `total_count`
- `page`
- `page_size`
- `total_pages`
- `has_previous_page`
- `has_next_page`

### 归档保留策略

Web workflow run 记录默认归档到：

```text
.output/web_workflow_archive/
```

默认保留策略�?
- 最多保�?`200` 条归�?runs
- 最多保�?`30` �?
可通过环境变量调整�?
- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`：默认每页条数，默认 `20`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`：允许的最大每页条数，默认 `100`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`：归档最多保留多少条 run，默�?`200`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`：归档最多保留多少天，默�?`30`

如果你需要准备部署环境变量文件，可直接参考：

- [`deployment/web_panel.env.example`](../../deployment/web_panel.env.example)

运行时查找顺序：

1. `AGI_WALKER_WEB_ENV_FILE`
2. `deployment/web_panel.env`
3. `deployment/web_panel.env.example`

这意味着�?
- 直接运行 `python -m web_panel.server` 时，workflow API 会自动尝试读取这些文�?- `quick_start.sh` / `quick_start.bat` 现在也会在启动前显式设置 `AGI_WALKER_WEB_ENV_FILE`

### Docker 运行变体

部署目录现在提供两条 Web 面板容器路径�?
- 默认 `web-panel`
  使用核心依赖集，优先保证 workflow 控制台和基础页面稳定启动�?- 可�?`web-panel-distributed`
  通过 compose `distributed` profile 启动，额外安�?`eclipse-zenoh`，用于需要容器内 Zenoh 分布式监控的场景�?
常用命令�?
```bash
docker compose -f deployment/docker-compose.yml up -d web-panel
docker compose -f deployment/docker-compose.yml --profile distributed up -d web-panel-distributed
python tests/run_distributed_smoke.py --build
```

默认访问地址�?
- `web-panel`: `http://localhost:8080/static/index.html`
- `web-panel-distributed`: `http://localhost:8081/static/index.html`

说明�?
- `python tests/run_distributed_smoke.py --build` 会启�?`distributed` �?`smoke` profiles，并附带拉起测试专用�?`mock-godot`�?- 该脚本用于验�?`sidecar-1 -> zenoh-router -> learner -> web-panel-distributed` 是否真正打通�?- 验证通过后，可在 `http://localhost:8081/api/distributed/status` 看到活跃 actor�?- 脚本入口位于 [`tests/run_distributed_smoke.py`](../../tests/run_distributed_smoke.py)�?
相关接口�?
- `GET /api/system/status`
  返回 `distributed_monitor` 字段，说�?`zenoh_available`、`monitor_active`、`endpoint` 和最近错误�?- `GET /api/distributed/status`
  返回 `actors` 以及 `monitor` 状态摘要�?
`/api/system/status` 现在还会额外返回 `nightly_regressions`，用于给首页系统状态卡显示�?
- `Nightly 回归`
- `最近专项运行`
- `Smoke / Distributed / Godot`

启用所需环境变量�?
- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`，可�?- `AGI_WALKER_GITHUB_WORKFLOW_FILE`，默�?`.github/workflows/ci.yml`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`，默�?`300`

如果你需要查看最近几次专�?run，而不仅是首页摘要，可使用�?
- `GET /api/nightly/regressions?limit=6`

这个接口会返回：

- 最近几�?`schedule / workflow_dispatch` 专项回归
- 每个 run �?`smoke / distributed-smoke / godot-headless-smoke` job 状�?- 对应 artifact 名称
- 推荐本地复现命令

分布式监控还会对长期未更新的 actor �?TTL 清理。默�?TTL �?`30` 秒，可通过以下环境变量调整�?
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`

## 当前接口结构

当前仓库里的 Godot 集成分为两种模式，这一点需要明确：

### 1. Legacy Controller 模式

这条链路面向“连�?Godot、加载机器人、启�?停止仿真、更新参数”�?
对应接口�?- `POST /api/godot/connect`
- `POST /api/godot/disconnect`
- `GET /api/godot/status`
- `POST /api/godot/load-robot`
- `POST /api/godot/start`
- `POST /api/godot/stop`
- `POST /api/godot/update-params`

WebSocket 协议中的这些命令也属于这一类：
- `simulation.start`
- `simulation.stop`
- `config.load_robot`
- `params.update`
- `ping`

### 2. Session Bridge 模式

这条链路面向“按会话启动 Godot 进程、通过 TCP 读取遥测/发送动作”，更接�?RL 或调试桥，同时也是当�?workflow -> Godot 的推荐官方路径�?
对应接口�?- `POST /api/godot/{session_id}/launch`
- `POST /api/godot/{session_id}/stop`
- `GET /api/godot/{session_id}/status`
- `POST /api/godot/{session_id}/control`
- `WS /ws/{session_id}`

当前 `godot_project/scripts/tcp_server.gd` 可确认支持的 TCP 命令是：
- `reset`
- `step`
- `get_schema`
- `load_robot`

这意味着 Session Bridge 已经能够承担 workflow 产物下发这条主线。Legacy Controller 仍然保留，但更适合作为兼容接口，而不是后续默认扩展方向�?
补充说明�?
- `launch / status / schema / load_robot / step / stop` 这条 lifecycle 已纳入真�?headless smoke 验收范围�?- `2026-04-02` �?Windows 本地环境下，默认场景 `demo_generated_biped.tscn` �?runner 场景 `run_rl_server.tscn` 都已通过这条 lifecycle 验收�?
## 调试建议

1. 先确认基础服务可以启动并监�?`8000` 端口�?2. 再访问首页和静态页面，确认静态资源加载正常�?3. 最后再接入 Godot 仿真端，逐步调试 WebSocket 与控制接口�?
## API 参�?
### Godot 接口
- `POST /api/godot/connect`: 连接仿真�?- `POST /api/godot/start`: 启动仿真
- `POST /api/godot/stop`: 停止仿真
- `POST /api/godot/update-params`: 更新物理参数
- `GET /api/godot/capabilities`: 查看当前支持�?Godot 接入模式

### Godot Agent 接口
- `POST /execute`: 执行 Godot Agent 单条命令
- `POST /pipeline`: 执行 Godot Agent 命令流水�?- `GET /roles`: 查看当前 backend 的角色矩�?- `GET /api/godot-agent/templates`: 列出当前 backend 暴露的模板资�?- `GET /api/godot-agent/templates/{template_id}`: 获取单个模板详情
- `GET /api/godot_skills/list`: 旧兼容别名；modern backend 下会投影 templates
- `POST /api/godot_skills/apply`: 旧兼容别名；modern backend 下会投影单个 template
- `POST /api/godot-agent/plan`: 生成可展示的 Godot Agent 任务计划
- `GET /api/godot-agent/history`: 获取最近任务历�?- `GET /api/godot-agent/doctor`: 运行 Godot Agent 环境自检
- `POST /api/godot-agent/launch`: 请求启动 Godot 编辑�?
说明�?
- `godot-agent` backend 的正式资源语义现在是 `templates`
- legacy `godot_studio_agent` backend 仍保�?`godot_skills`
- 为了不打断旧前端和旧脚本，`/api/godot_skills/*` 继续保留，但返回里会�?`compatibility_alias` �?`source_kind`
- `/api/system/status` 现在会额外返�?`godot_agent` 摘要，首页状态卡会直接显示当�?backend、资源模式以�?roles/templates 计数
- `godot-agent` modern backend 现在会显式绑定默�?`project_path` �?`history_file`，避免在未配置项目路径时回退�?Web 进程当前工作目录

可通过环境变量切换 backend�?
- `AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent`
- `AGI_WALKER_GODOT_AGENT_DIR=<agent_dir>`
- `AGI_WALKER_GODOT_PROJECT_PATH=<project_dir>`
- `AGI_WALKER_GODOT_AGENT_HISTORY_FILE=<history_file>`

默认行为�?
- 如果未显式配�?`AGI_WALKER_GODOT_PROJECT_PATH`，modern backend 会默认使用仓库内�?`godot_project/`
- 如果未显式配�?`AGI_WALKER_GODOT_AGENT_HISTORY_FILE`，任务历史会默认写到 `.output/godot_agent_backend/task_history.json`
- 对应回滚备份目录默认位于 `.output/godot_agent_backend/backups/`

`/api/system/status` 里的 `godot_agent` 摘要现在还会返回�?
- `project_path`
- `history_file`

### 任务接口
- `GET /api/tasks`: 获取任务列表
- `POST /api/tasks`: 创建任务
- `DELETE /api/tasks/{id}`: 删除任务

### Workflow 接口
- `GET /api/workflows/`: 列出 workflows
- `GET /api/workflows/{name}`: 查看 workflow 定义
- `POST /api/workflows/{name}/run`: 启动后台 workflow run
- `GET /api/workflows/runs`: 查询 runs 列表，支持筛选、分页和归档范围
- `GET /api/workflows/runs/{run_id}`: 获取完整 run 详情
- `GET /api/workflows/runs/{run_id}/status`: 获取轻量状态快�?- `POST /api/workflows/runs/{run_id}/cancel`: 发送取消请�?- `GET /api/workflows/runs/{run_id}/events`: 订阅 SSE 事件�?- `GET /api/workflows/runs/{run_id}/live-log`: 获取完整实时日志文本
- `GET /api/workflows/runs/{run_id}/log`: 下载 workflow JSON log
- `GET /api/workflows/runs/{run_id}/artifacts/{artifact_index}`: 下载指定产物
- `POST /api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load`: �?`robot_config` 产物送往 Godot，可�?`legacy_controller` �?`session_bridge`
- `POST /api/workflows/runs/{run_id}/godot-sync`: 自动选择推荐 `robot_config` 产物，并按官方路径送往 Godot

### 真实 Godot Headless Smoke

仓库现在保留了一条显�?opt-in 的真�?Godot headless smoke�?
```bash
AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m integration
```

默认 `python tests/run_smoke_tests.py` 不会强制跑这条检查；只有在显式设置：

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1`

后，smoke runner 才会追加真实 Godot headless 验收�?
如果需要把这条链路当作独立 integration 标准件运行，建议显式补上诊断产物目录�?
```powershell
$env:GODOT_EXECUTABLE='D:\迅雷下载\Godot\godot.EXE'
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
$env:AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR='test_env/godot_headless_smoke'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

如果你要验证 runner 场景，也可以显式指定�?
```powershell
$env:GODOT_EXECUTABLE='D:\迅雷下载\Godot\godot.EXE'
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
$env:AGI_WALKER_GODOT_HEADLESS_SCENE='run_rl_server.tscn'
$env:AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR='test_env/godot_headless_smoke_runner'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

可调参数包括�?
- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

每次运行都会写出结构化诊断文件：

- `test_env/godot_headless_smoke/headless_smoke_report.json`

其中会记录：

- preflight 环境结果
- failure stage：`environment / launch / tcp_connect / schema / load_robot / step_loop / teardown`
- Godot 进程诊断�?stdout/stderr tail
- schema 摘要�?step 样本

`2026-04-02` 的本地验收快照：

- 显式设置 `GODOT_EXECUTABLE` 并启�?`AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1` 后，`python tests/run_smoke_tests.py --output-root test_env/smoke_runs/v1_plan_check_after_fix` 已通过
- `python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv` 已通过
- 设置 `AGI_WALKER_GODOT_HEADLESS_SCENE=run_rl_server.tscn` 后复跑同一�?lifecycle smoke，也已通过

CI 中另外保留了一�?`godot-headless-smoke` job，它现在支持�?
- `workflow_dispatch`
- nightly schedule

同样，`distributed-smoke` 也已进入 nightly。nightly 不会运行完整 unit/integration 矩阵，而是专门保留高价值的专项链路回归�?
