# AGI-Walker v1.0.0

AGI-Walker 是一个面向机器人建模、参数优化、自动化任务编排与 Godot 仿真的集成开发平台。项目已完成 V1 阶段的全面闭环，提供从参数化设计到物理仿真验证的完整链路。

## 核心能力 (V1)

- **模块化建模**: 快速创建双足/四足/轮式机器人模型，支持参数化定制。
- **参数自动优化**: 使用遗传算法或梯度下降优化重心分布、关节限位与 PID 参数。
- **Workflow 编排**: 标准化的机器人设计流水线，支持 Mock/Real 双模执行与状态持久化。
- **Web 控制台**: 直观的任务监控界面，集成实时日志、产物管理与 Godot 一键同步。
- **仿真集成**: 与 Godot 深度集成，提供高可靠的 TCP 控制链路与 Headless 自动化测试能力。
- **运维与监控**: 完整的 Nightly 专项回归系统，实时追踪 Smoke / Distributed / Godot 链路健康度。

## 仓库结构

- `agi_walker/`: Python 主包，包含 Skills 引擎、Workflow 编排器与 CLI 实现。
- `web_panel/`: FastAPI Web 控制台，集成 SSE 实时通信与 Nightly 运维看板。
- `godot_project/`: Godot 仿真工程，提供物理模拟与机器人加载环境。
- `tests/`: 包含全量 Smoke Test、集成测试与 Nightly 回归脚本。
- `docs/`: 包含 V1 状态宣言、CLI 指南、Web 面板指南与部署手册。

## 代码质量标准 (v1.0.0)

AGI-Walker v2.0 完成了全面的代码规范化升级：

| 指标 | 现状 | 说明 |
|------|------|------|
| **Logger 框架** | 93% 覆盖 | 统一的日志系统，支持日志级别控制和生产部署 |
| **Print 清理** | 1600+清理 | 核心系统 (Phase 3-6) 完全消除 print 语句 |
| **Type Hints** | 941+ 函数 | 关键函数添加返回类型，IDE 智能补全 |
| **异常处理** | 100% 覆盖 | 所有异常正确捕获和记录 |
| 编译验证 | 100% 通过 | 所有文件无语法错误 |
| **Pytest 测试** | 567 发现 | 测试框架完整性保证 |

### 快速开始 (新用户)

```python
# ✅ 标准的 Logger 用法
import logging
logger = logging.getLogger(__name__)

logger.info("操作启动")
logger.warning("潜在问题") 
logger.error("错误发生")

# ✅ 函数返回类型提示
def process_observation(obs: np.ndarray) -> Dict[str, float]:
    """处理机器人观测数据."""
    return results

# ✅ 异常处理规范
try:
    import torch
    TORCH_AVAILABLE = True
except ImportError:
    logger.warning("torch 不可用")
    TORCH_AVAILABLE = False
```

### 文档资源

- 📖 [MIGRATION_GUIDE.md](docs/MIGRATION_GUIDE.md) - 升级指南和最佳实践
- 📊 [CODE_QUALITY.md](docs/CODE_QUALITY.md) - 详细的代码质量报告
- 📋 [CHANGELOG.md](CHANGELOG.md) - 完整的变更清单
- 📢 [RELEASE_NOTES.md](RELEASE_NOTES.md) - v2.0 发版说明
- 🧭 [docs/CURRENT_STATUS.md](docs/CURRENT_STATUS.md) - 当前状态与历史文档边界

### 向后兼容性

✅ **100% 向后兼容** - 现有代码无需修改即可继续工作。这些改进仅添加功能，不删除或破坏现有 API。
## 项目结构

- `agi_walker/`
  Python 主包，包含 Skills 加载器、CLI、工作流编排与相关实现。
- `examples/`
  示例脚本和标准任务目录，例如楼梯攀爬任务。
- `web_panel/`
  Web 控制面板、WebSocket 协议处理和静态页面资源。
- `godot_project/`
  Godot 仿真工程。
- `python_api/`、`python_controller/`
  Python 侧接口与控制逻辑。
- `parts_library/`
  零件库与相关文档资源。
- `training/`、`sysid/`
  训练和系统辨识相关代码。
- `tests/`
  单元测试、集成验证脚本和基准测试。

## 仓库里现在有什么

### 1. Skills 系统

仓库内目前包含 3 个核心 Skills 目录：

- `agi_walker/skills/robot-modeling/`
- `agi_walker/skills/parameter-optimizer/`
- `agi_walker/skills/urdf-generator/`

相关文档：

- [Skills 系统指南](.agent/AGENTS.md)
- [CLI 使用指南](docs/guides/CLI_GUIDE.md)
- [GUI 使用指南](docs/guides/GUI_GUIDE.md)
- [开发新 Skill](docs/guides/SKILLS_DEVELOPMENT.md)

说明：

- 这些 Skills 的实现和文档存在于仓库中。
- 当前首页不再把具体导入示例写成“开箱即用”，因为部分入口仍需要进一步整理和验证。

### 2. 任务示例

任务示例位于 `examples/tasks/`，目前文档中可见的重点示例包括：

- `examples/tasks/stair_climbing/`
- `examples/tasks/README.md`

可参考的入口：

```bash
python examples/tasks/stair_climbing/env.py
```

注意：

- 该示例会在终端输出包含 emoji 的文本；在部分 Windows GBK 终端中，直接运行可能触发编码错误。
- 更适合作为代码阅读和本地调试入口，而不是首页承诺的“零配置演示”。

### 3. Web 控制面板

Web 面板代码位于 `web_panel/`，文档位于：

- [Web 面板指南](docs/guides/WEB_PANEL_GUIDE.md)

当前判断：

- 面板代码、协议处理和前端页面资源已存在。
- `/static/workflows.html` 现在已经提供一条更完整的 workflow 控制台链路：
  - 后台发起 workflow run
  - 轮询 + SSE 接收状态更新
  - 查看步级进度、实时日志、故障诊断
  - 下载产物和 workflow log
  - 使用推荐 `session_bridge` 路径，将 workflow 产物中的机器人配置通过正式后端路由送往 Godot
  - 使用 `/api/workflows/runs/{run_id}/godot-sync` 自动选择可用机器人配置产物并送往 Godot
  - 在 run 详情里保留最近一次 Godot 交付结果，并显示会话状态接口、失败阶段和重试入口
  - 按 `scope/status/mode/date/text` 检索 runs
  - 对 runs 列表进行分页
  - 首页现在提供 `Nightly 运维页` 入口：`/static/nightly.html`
- 该部分依赖 FastAPI、静态资源路径和 Godot 联动环境。
- 在显式配置可用 Godot 环境后，Web workflow 控制台与 `session_bridge` 官方链路已具备稳定闭环，并已通过真实 headless smoke 验证。
- 当前仍不应承诺“跨平台、零配置、任意外部 Godot 环境下都稳定可用”。
- 容器化部署现在区分默认 `web-panel` 核心镜像和可选 `web-panel-distributed` 变体，避免把分布式监控依赖强行耦合进默认启动路径。
- Docker 下默认 `web-panel` 暴露在 `http://localhost:8080`，可选 `web-panel-distributed` 暴露在 `http://localhost:8081`。

Web workflow 历史记录默认会归档到：

```text
.output/web_workflow_archive/
```

默认保留策略：

- 最多保留 `200` 条归档 runs
- 最多保留 `30` 天
- runs 列表默认每页 `20` 条，最大每页 `100` 条

可通过环境变量覆盖：

- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`
- `AGI_WALKER_ZENOH_ENDPOINT`

运行时加载顺序：

- 如果设置了 `AGI_WALKER_WEB_ENV_FILE`，优先读取该文件
- 否则尝试读取 `deployment/web_panel.env`
- 再否则回退到 `deployment/web_panel.env.example`

`/api/system/status` 现在会额外返回 `distributed_monitor` 状态，用于说明当前 Web 面板运行环境是否具备 Zenoh 分布式监控能力。

分布式监控当前还支持 actor TTL 清理，默认会在 `30` 秒后清走长期未更新的 actor，可通过以下环境变量覆盖：

- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`

如果配置了 GitHub Actions nightly 状态来源，首页系统状态卡现在还会显示专项回归摘要：

- `Nightly 回归`
- `最近专项运行`
- `Smoke / Distributed / Godot`

对应后端数据来自 `/api/system/status` 的 `nightly_regressions` 字段。启用所需环境变量：

- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`（可选，私有仓库或提高 GitHub API 额度时使用）
- `AGI_WALKER_GITHUB_WORKFLOW_FILE`，默认 `.github/workflows/ci.yml`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`，默认 `300`

### 工作流产物追踪

- `.output/workflow_artifacts/{workflow_name}/` 下的 JSON 文件会记录每个 step 的 inputs/outputs/状态，workflow run 时 real executor 模式会自动产生；
- 每个 artifact 满足固定 schema（workflow/step/executor/action/status/inputs/output/mode/started_at/ended_at），写入前会进行字段检验，避免遗留不完整数据；
- workflow 日志（`.output/workflow_log_*`）包含 `artifact_path` 和 `artifacts` 数组，方便自动化消费或运维人员定位对应 step；
- 这些产物也可以作为回归或 Godot session_bridge 的可信数据源，直接引用在 `.output` 目录下。

如果需要查看最近几次专项运行、每个 job 的 artifact 名称以及本地复现命令，可直接打开：
- `/static/nightly.html`

对应运维 API：

- `GET /api/nightly/regressions?limit=6`

### 4. Godot 集成

Godot 相关内容位于：

- `godot_project/`
- `godot_studio_agent/`

这部分已经形成 V1 官方链路：

- `session_bridge` 是默认推荐路径
- workflow 产物可通过 `/api/workflows/runs/{run_id}/godot-sync` 正式送往 Godot
- Windows 本地环境下的真实 headless smoke 已于 `2026-04-02` 实测通过
- `legacy controller` 继续保留，但只作为兼容路径

Godot Agent 集成层现在支持通过环境变量切换 backend：

- `AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent`
- `AGI_WALKER_GODOT_AGENT_DIR=<agent_dir>`
- `AGI_WALKER_GODOT_PROJECT_PATH=<project_dir>`
- `AGI_WALKER_GODOT_AGENT_HISTORY_FILE=<history_file>`

如果没有显式指定 `AGI_WALKER_GODOT_PROJECT_PATH`，modern `godot-agent` backend 现在会默认绑定到仓库内的：

- `godot_project/`

同时它的任务历史与回滚备份会默认隔离到：

- `.output/godot_agent_backend/task_history.json`
- `.output/godot_agent_backend/backups/`

`/api/system/status` 会返回当前 Godot Agent backend 摘要，主控制台首页也会直接显示：

- 当前 backend 模式
- 资源模式（`skills` 或 `templates`）
- roles / templates 计数
- 当前默认 `project_path`
- 当前 `history_file`

当切换到外部 `godot-agent` 后端后，Web 面板会额外提供：

- `GET /api/godot-agent/templates`
- `GET /api/godot-agent/templates/{template_id}`
- `POST /api/godot-agent/plan`
- `GET /api/godot-agent/history`
- `GET /api/godot-agent/doctor`
- `POST /api/godot-agent/launch`

旧的：

- `GET /api/godot_skills/list`
- `POST /api/godot_skills/apply`

仍然保留为兼容别名；在 modern `godot-agent` backend 下，它们会投影到 templates，并在返回里标出 `compatibility_alias=true`。

## 安装

### 基础依赖

```bash
pip install -r requirements.txt
```

### 辅助脚本

- Windows: `scripts/install.bat`
- Linux/macOS: `scripts/install.sh`

说明：

- 仓库内提供了安装脚本和 Dockerfile。
- 不同模块的依赖并不完全相同，涉及 Godot、硬件、训练环境时通常需要额外配置。

## 最小 Smoke 验收

如果你想先确认仓库最核心的 CLI / workflow / Web 面板导入链路是否正常，建议先运行：

```bash
python tests/run_smoke_tests.py
```

该脚本会执行一组最小但高信号的检查：

- `skills list`
- `skills validate`
- `robot_creation_pipeline` 的 mock workflow
- `robot_creation_pipeline` 的 real workflow
- `web_panel.server` 导入与 `WsMessage(type="ping")` 向后兼容检查
- Godot Agent fake backend 的最小 Web/API smoke
- 如果检测到外部 `godot-agent` 目录，还会追加 modern backend 的 `templates / plan / doctor / skills alias` smoke
- 如果显式设置 `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1`，还会追加真实 Godot headless integration smoke

workflow 相关产物默认会被隔离到：

```text
test_env/smoke_runs/<timestamp>/
```

如果你需要自定义隔离目录：

```bash
python tests/run_smoke_tests.py --output-root test_env/smoke_runs/manual
```

如果你想显式指定 external `godot-agent` 目录给 smoke 使用：

```bash
AGI_WALKER_SMOKE_GODOT_AGENT_DIR=/path/to/godot-agent python tests/run_smoke_tests.py
```

Windows PowerShell:

```powershell
$env:AGI_WALKER_SMOKE_GODOT_AGENT_DIR='D:\新建文件夹\godot-agent'
python tests/run_smoke_tests.py
```

如果你要显式启用真实 Godot headless smoke：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python tests/run_smoke_tests.py
```

如果你要单独运行这条 integration smoke，并保留标准化诊断产物：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
$env:AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR='test_env/godot_headless_smoke'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

如果你的工作环境暂时没有安装 Godot，可使用 `tests/stubs/` 目录下提供的 `godot_stub.bat` + `godot_stub.py` 来模拟 headless 进程。把 `GODOT_EXECUTABLE` 指向这份 `.bat`，其内部会调用 `python godot_stub.py` 并在 `--tcp-port` 指定的端口上开启模拟服务，从而在不依赖 Godot 二进制的情况下也能跑通 headless smoke。例如：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
$env:AGI_WALKER_GODOT_HEADLESS_SCENE='run_rl_server.tscn'
$env:GODOT_EXECUTABLE='D:\新建文件夹\AGI-Walker\tests\stubs\godot_stub.bat'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

模拟脚本会输出健康的 `sensors/schema` 结果，同时保持在 `test_env/godot_headless_smoke/headless_smoke_report.json` 中写入诊断报告，便于进一步排查。

`2026-04-02` 的本地验收快照：

- 显式设置 `GODOT_EXECUTABLE` 并启用 `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1` 后，`python tests/run_smoke_tests.py --output-root test_env/smoke_runs/v1_plan_check_after_fix` 已通过
- `python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv` 已通过
- 在设置 `AGI_WALKER_GODOT_HEADLESS_SCENE=run_rl_server.tscn` 后，同一条 headless lifecycle smoke 也已通过

常用可调参数：

- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

运行后会在以下位置留下结构化诊断文件：

- `test_env/godot_headless_smoke/headless_smoke_report.json`

这比直接复用仓库根目录下已有 `.output/` 和 `exports/` 更适合作为回归验证入口。

仓库的 CI 现在还提供两个专项回归入口，并会在 nightly 中自动运行：

- `distributed-smoke`
- `godot-headless-smoke`

触发方式：

- `workflow_dispatch`
- nightly schedule（UTC `02:00`）

其中 `godot-headless-smoke` 会在 Windows runner 上执行：

```bash
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

nightly 不会触发完整 unit/integration 矩阵；它只保留：

- `quality`
- `smoke`
- `distributed-smoke`
- `godot-headless-smoke`

这样可以把高价值的专项链路持续跑起来，又避免把定时任务成本膨胀到整套回归。

如果你需要查看 nightly 专项回归的 artifact 名称、本地复现命令和故障排查顺序，请直接参考：

- `PRODUCTION_DEPLOYMENT_RUNBOOK.md`

仓库也已经提供对应的 GitHub issue 模板：

- `.github/ISSUE_TEMPLATE/nightly_regression.md`

如果你要验证 Docker 下的分布式训练链路，可运行：

```bash
python tests/run_distributed_smoke.py --build
```

该脚本会使用 `deployment/docker-compose.yml` 启动一条最小分布式 smoke 链路：

- `zenoh-router`
- `learner`
- `web-panel-distributed`
- `mock-godot`（`smoke` profile，测试专用）
- `sidecar-1`

验证目标是让 `sidecar-1 -> zenoh-router -> learner -> web-panel-distributed` 真正产生 actor 数据，并能在：

- `http://localhost:8081/static/distributed.html`
- `http://localhost:8081/api/distributed/status`

看到活跃 actor 与基础遥测信息。

## 建议阅读顺序

如果你是第一次进入这个仓库，建议按下面顺序理解项目：

1. [任务目录说明](examples/tasks/README.md)
2. [CLI 使用指南](docs/guides/CLI_GUIDE.md)（`agi_walker workflows ...` 现在等同于 `agi_walker skills workflows ...`）
3. [GUI 使用指南](docs/guides/GUI_GUIDE.md)
4. [Web 面板指南](docs/guides/WEB_PANEL_GUIDE.md)
5. [开发者指南](docs/guides/developer_guide.md)
6. [硬件部署文档](docs/hardware/HARDWARE_DEPLOYMENT.md)

## 当前 README 的边界

这个 README 只做三件事：

- 说明仓库目标和模块分布
- 给出真实存在的文档入口
- 区分 V1 官方入口与实验/扩展方向

它不再在首页直接承诺以下内容：

- 未验证的导入示例一定可直接运行
- 跨平台、零配置、任意外部 Godot 环境下都稳定可用
- 所有路线图条目已经进入可演示状态
- 缺少落地证据的性能结论

另外，`docs/archive_and_reports/` 与 `docs/architecture/` 中部分 2026-01 的文档属于历史阶段快照。
这些文档可用于理解演进过程，但不应直接作为当前“完成度”或“生产就绪度”的权威说明。
当前状态请优先参考 [docs/CURRENT_STATUS.md](docs/CURRENT_STATUS.md)。

## 相关文档

- [贡献指南](CONTRIBUTING.md)
- [模块化零件系统文档](docs/hardware/MODULAR_ROBOT_BUILDER.md)
- [硬件部署文档](docs/hardware/HARDWARE_DEPLOYMENT.md)
- [性能基准脚本](tests/benchmark_performance.py)

## 许可证

本项目采用 [MIT 许可证](LICENSE)。
