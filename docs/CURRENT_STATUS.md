# AGI-Walker Current Status

更新日期：`2026-04-12`

## 摘要

AGI-Walker 当前可用的主入口已经恢复到可读、可执行、可验证的状态。此次修复的重点是：

- 仓库首页 `README.md` 已恢复为正常 UTF-8 文档。
- MCP `stdio` server 已可通过 `agi-walker-mcp` 和 `python -m agi_walker.mcp.server` 启动。
- `mcp>=1.27.0` 下的初始化兼容问题已修复。
- CLI、Workflow、Web Panel 和 Godot Agent 的入口文档已重新对齐到真实代码接口。
- 阶段一主链稳定化已完成：Workflow v1 契约、真实产物 JSON schema、工作流定义校验和回归测试已经落地。
- 阶段二 Web/Godot 操作闭环已完成：Web workflow artifact manifest、Godot session 状态机、delivery record 和前端展示已复用 v1 契约。
- 阶段三 Distributed Runtime 已完成并完成本机实机验收：Docker smoke report v1、distributed monitor v1、actor discovery 状态、compose log 诊断和 live smoke 证据都已落地。
- 阶段四训练、仿真与硬件边界已完成：`training_run` artifact v1 已落地，mock RL、offline dataset training、sim training 路径会写出可校验训练 manifest，HITL 分类已进入回归测试；IMC-22、真实串口驱动和 ROS2 bridge 的 replay/mock 已接入默认 pytest，Godot headless live smoke 与 ROS2 bridge 的真实 Humble live smoke 都已留存结构化通过报告。
- Nightly / manual 专项回归当前已扩展到 `ros2-bridge-smoke`，并为该 live smoke 增加了 CI job、nightly 跟踪和 artifact 上传入口。
- 阶段五产品化与发布门禁已完成：`capability_matrix` contract v1、`release_manifest` contract v1、发布指南和 release artifact 生成脚本已落地，并通过 smoke 与默认非 live 门禁验收。
- `release_policy`、`release_approval` 与 `release_source` 已成为 release manifest 的机器可读字段，`dev / rc / stable` 的 gate 语义、stable 签核要求，以及“签核 SHA 必须绑定当前 Git HEAD、stable 版本必须有匹配 Git tag”的规则已经收口到同一套 contract，而不是继续依赖人工解释。
- stable gate 现在还会显式检查 Git worktree 是否 clean，避免在脏工作区上误做 stable promotion。
- 显式 stable 发布演练脚本 `tools/run_release_rehearsal.py` 已落地，并已接入默认 smoke，用于验证“临时 Git repo + 匹配 tag + stable signoff -> release_gate_status=ready”的正向路径。

## 当前可用入口

- CLI：`python -m agi_walker.cli`
- MCP：`agi-walker-mcp`
- MCP 模块入口：`python -m agi_walker.mcp.server`
- Web Panel：`python -m web_panel.server`
- Smoke：`python tests/run_smoke_tests.py`
- Release Artifact：`python tools/build_release_artifact.py --version ... --channel ... --build-id ... --release-summary ...`
- Release Readiness：`python tools/check_release_readiness.py`
- Worktree Cleanup Report：`python tools/build_worktree_cleanup_report.py`
- Tracked Artifact Review Report：`python tools/build_tracked_artifact_review_report.py`
- Stable Promotion Checklist：`python tools/build_stable_promotion_checklist.py`

## 已验证项

以下命令在当前工作区已执行并通过：

- `python -m agi_walker.cli skills list`
- `python -m agi_walker.cli workflows list`
- `python -m agi_walker.cli workflows run robot_creation_pipeline --mock --output-root test_env/doc_check_workflow --resume`
- `python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q`
- `python tests/run_smoke_tests.py --output-root test_env/smoke_phase5_acceptance`
- `python tools/build_release_artifact.py --version 2026.04.12-rc6 --channel rc --build-id build-20260412-006 --release-summary "Release gate now resolves to ready when all live evidence is attached." --output test_env/release/release_manifest.json`
- `python tools/check_release_readiness.py`
- `python tools/build_worktree_cleanup_report.py`
- `python tools/build_tracked_artifact_review_report.py`
- `python tools/build_stable_promotion_checklist.py`
- `python tools/run_release_rehearsal.py --version 2026.04.12-rehearsal --build-id release-rehearsal`
- `python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json`
- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv`
- `docker run --rm -v "<repo>:/workspace" -w /workspace ros:humble-ros-base bash -lc 'source /opt/ros/humble/setup.bash && python3 -m pip install --no-cache-dir --upgrade "pytest>=7.4,<8" pytest-asyncio pytest-cov pytest-mock anyio numpy pyyaml pydantic && export PYTHONPATH=/workspace:$PYTHONPATH && export AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 && python3 -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv'`
- `python -m pytest tests\test_workflow_contracts.py tests\test_workflow_orchestrator.py -q`
- `python -m pytest tests\test_web_godot_session_bridge.py tests\test_web_panel_integration_routes.py -q`
- `python -m pytest tests\test_distributed_smoke_runner.py tests\test_web_panel_aux_apis.py::test_core_panel_routes_smoke tests\test_web_panel_aux_apis.py::test_distributed_monitor_prunes_stale_actors tests\test_active_path_references.py::test_distributed_runtime_uses_current_package_entrypoints -q`
- `python -m pytest -m "not live" -q`
- `python -m pytest tests\ -m "not integration and not hardware" -v --tb=short --cov=agi_walker --cov-report=xml --cov-report=term-missing`

已验证结果：

- Skills 列表可正常读取，当前检出包含 `robot-modeling`、`parameter-optimizer`、`urdf-generator`、`model-distiller`。
- Workflows 列表可正常读取，当前至少包含 `robot_creation_pipeline` 和 `simulation_ready_robot`。
- `robot_creation_pipeline` 的 mock 执行可完成，输出可写入 `test_env/doc_check_workflow`。
- MCP 相关测试当前为 `7 passed`。
- smoke runner 当前为 `PASS`，并已覆盖 capability matrix 与 release artifact 生成链路。
- smoke runner 当前为 `PASS`，并已覆盖 capability matrix、release artifact 生成链路、release readiness 诊断链路、worktree cleanup report、tracked artifact review report、stable promotion checklist 诊断链路，以及 stable release rehearsal 正向路径。
- release artifact 生成脚本会写出 `test_env/release/release_manifest.json`，当前 `release_gate_status=ready`，其中 distributed、Godot headless 和 ROS2 bridge live evidence 都已为 `passed`，`blocked_evidence=0`、`opt_in_evidence=0`，`distributed_runtime` domain 已提升为 `ready`。
- release artifact 现在会写出 `release_policy`，其中 `dev` 允许未闭合的 `opt_in` evidence / `diagnostic_ready` domain，`rc` 和 `stable` 则要求这些项先收口后才能达到 `ready`。
- `stable` 通道还会显式要求 `release_approval.status=approved`，并校验 `approved_by`、`approved_at`、`commit_sha`；如果当前仓库 Git HEAD 可解析，`release_source` 会被写入 manifest，stable gate 还会校验签核 SHA 与当前 HEAD 一致，并要求当前 HEAD 存在与 `version` 或 `v{version}` 匹配的 Git tag。
- Workflow 契约和 orchestrator 回归测试当前为 `16 passed`。
- Web/Godot session 与 workflow 路由回归测试当前为 `36 passed`。
- Distributed smoke runner、distributed status 和容器入口路径回归测试当前为 `7 passed`。
- 默认非 live 门禁当前为 `792 passed, 3 skipped, 3 deselected`。
- 当前真实仓库的 stable readiness 仍为 `blocked`，下一步动作已增至 `4` 项：stable 签核、HEAD 绑定、clean worktree、匹配版本 tag。
- 当前 `clean_worktree` 阻塞项现在会直接指向 `worktree_cleanup_report.json`，用于把运行时产物、生成物候选和真实源码改动拆开，而不是只给出一条 `git status --short`。
- 当前最新 cleanup report 显示工作区共有 `189` 个待处理路径，其中运行时产物 `6`、生成物候选 `1`、源码/文档人工审查项 `182`。
- 新增的 `.gitignore` 规则已经先压掉大部分 untracked runtime / generated 噪音，因此 cleanup report 现在主要暴露 tracked 的真实阻塞项。
- 新增的 tracked artifact review report 会继续聚焦这 `7` 个 tracked 候选，避免直接回退用户改动。
- 默认非 integration/hardware 覆盖率门禁当前为 `704 passed, 28 deselected`，并生成 `coverage.xml`。

## 阶段一契约状态

阶段一新增的稳定契约入口位于：

- [agi_walker/core/api/workflow_contracts.py](../agi_walker/core/api/workflow_contracts.py)

当前已固定并测试的契约包括：

- `workflow_step` artifact
- `workflow_result`
- workflow definition
- RobotConfig
- PartSpec
- mass optimization result
- URDF/SDF/MJCF export result

真实 workflow step artifact 当前包含 `schema_version=1.0`、`artifact_type=workflow_step`、executor/action/status/mode、resolved inputs、executor output、attempt count、duration 和创建时间。后续 Web、MCP、distributed runtime 应复用该契约，不应重新定义 ad hoc 字段。

完整补全路线见：

- [功能补全计划](FEATURE_COMPLETION_PLAN.md)
- [发布指南](guides/RELEASE_GUIDE.md)

## Web / Godot 状态

Web Panel 的 FastAPI 入口位于 [web_panel/server.py](../web_panel/server.py)，当前公开的主要能力包括：

- 根页面 `/`
- 主控制台 `/static/index.html`
- Workflow 控制台 `/static/workflows.html`
- Nightly 面板 `/static/nightly.html`
- Distributed 面板 `/static/distributed.html`
- Godot 设计页 `/static/design.html`
- Godot 控制页 `/static/godot-control.html`
- 发布面矩阵接口 `/api/capabilities/matrix`

Godot 集成当前同时支持两种 backend：

- `legacy`
- `godot-agent`

切换环境变量：

```text
AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent
```

阶段二后，Web workflow 与 Godot delivery 使用以下稳定元数据：

- workflow/run 响应包含 `workflow_contract_version`、`workflow_result_schema_version` 和 `workflow_result_artifact_type`。
- artifact manifest 包含 `schema_version`、`contract.valid`、`contract.errors`、`contract.payload_type` 和 `step_artifact_contract`。
- Godot session status payload 使用 `schema_version=1.0`，状态值限定为 `disconnected`、`launching`、`connected`、`schema_ready`、`running`、`failed`。
- Godot delivery record 会保存 artifact contract、`session_state`、`session_status`、`schema_available` 和 `transport_status_url`。

## 推荐的配置入口

Web workflow 相关参数优先从以下 env 文件读取：

- `deployment/web_panel.env`
- `deployment/web_panel.env.example`
- 或通过 `AGI_WALKER_WEB_ENV_FILE` 显式指定

当前需要重点关注的环境变量：

- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`
- `AGI_WALKER_ZENOH_ENDPOINT`
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`
- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`
- `AGI_WALKER_GITHUB_WORKFLOW_FILE`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`

Godot headless smoke 相关环境变量：

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE`
- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

ROS2 bridge live smoke 相关环境变量：

- `AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE`
- `AGI_WALKER_ROS2_BRIDGE_SMOKE_ARTIFACT_DIR`

Nightly 当前跟踪的专项 job：

- `smoke`
- `distributed-smoke`
- `godot-headless-smoke`
- `ros2-bridge-smoke`

发布面 capability matrix 当前覆盖：

- `cli`
- `web_panel`
- `mcp`
- `distributed_runtime`
- `godot_integration`

发布门禁当前通过以下产物固定：

- `release_manifest` artifact
- `capability_matrix` artifact
- `docs/guides/RELEASE_GUIDE.md`
- `tools/build_release_artifact.py`

## Distributed Runtime 状态

当前 Docker compose 分布式入口使用模块路径：

- learner：`python -u -m agi_walker.core.distributed.run_learner`
- sidecar：`python -u -m agi_walker.core.distributed.sidecar`

分布式监控和烟测当前使用以下稳定元数据：

- `/api/distributed/status` 返回 `schema_version=1.0`、`actors`、`actor_ids`、`actors_count` 和 `monitor`。
- `distributed_monitor` capability 返回 `schema_version=1.0`、Zenoh endpoint、订阅 topic `ag/*/obs`、actor TTL、active actor 数量和最近裁剪状态。
- `tests/run_distributed_smoke.py --report-file ...` 会输出 `schema_version=1.0` 的 smoke report，包含 compose build/up、Web status、monitor、sidecar start、actor discovery 和 learner action loop 的逐项结果。
- smoke runner 默认发布 Zenoh 宿主端口 `17447/18000`，避免与本地 Web/API 服务常用 `8000` 端口冲突。
- smoke runner 默认向 sidecar 注入 `AGI_WALKER_FORCE_OFFLOAD=1`，用于 Docker smoke 的启动闭环；否则 sidecar 初始 `cloud_available=False` 会导致 learner 收不到第一帧 observation。
- CI `distributed-smoke` job 只在 `workflow_dispatch` 或 `schedule` 路径运行，并上传 `distributed-smoke-artifacts`。
- smoke 失败诊断会识别 Docker buildx 权限、Docker Desktop Linux engine 不可用、旧 `/app/distributed/*.py` 入口、Python 模块导入失败、Zenoh 连接和 Godot TCP 连接问题。

本机实机 smoke 已在 `2026-04-12` 重新执行并通过。最新通过报告位于 `test_env/distributed_smoke/distributed_smoke_report.json`，状态为 `passed`，7/7 checks 通过，已覆盖 `compose_build`、`compose_up`、`web_panel_status`、`distributed_monitor`、`sidecar_start`、`actor_discovery` 和 `learner_action_loop`。当前 release artifact 也已成功吸收该证据，并将 `distributed_runtime` 从 `diagnostic_ready` 提升为 `ready`。复跑命令保持不变：

```powershell
python tests\run_distributed_smoke.py --build --stop-after --report-file test_env\distributed_smoke\distributed_smoke_report.json
```

Godot headless live smoke 也已在 `2026-04-12` 本机通过。当前通过报告位于 `test_env/godot_headless_smoke/headless_smoke_report.json`，状态为 `passed`，已覆盖 Godot executable 发现、headless 场景拉起、TCP 建连、schema 获取、`load_robot` 和 step loop。复跑命令：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests\test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
```

ROS2 bridge live smoke 也已在 `2026-04-12` 的 `ros:humble-ros-base` 容器化 ROS2 Humble runtime 中通过。当前通过报告位于 `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`，状态为 `passed`，已覆盖 ROS2 runtime 预检、`start_sim` / `stop_sim` service、JointState 映射、`/cmd_vel -> update_params` 和 TCP bridge 命令闭环。复跑时应在具备 ROS2 Humble Python runtime 的主机或容器中执行：

```powershell
$env:AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE='1'
python -m pytest tests\test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv
```

## Training / Hardware 边界状态

阶段四新增的稳定训练契约入口位于：

- [agi_walker/core/api/training_contracts.py](../agi_walker/core/api/training_contracts.py)

当前 `training_run` artifact 使用 `schema_version=1.0`，字段包括：

- `run_id`
- `run_type`
- `stage`
- `status`
- `algorithm`
- `environment`
- `inputs`
- `metrics`
- `artifacts`
- `hardware_required`
- `hardware_enabled`
- `started_at`
- `finished_at`
- `duration_seconds`

训练类型当前限定为：

- `mock_training`
- `offline_dataset_training`
- `sim_training`
- `hardware_in_the_loop`

`EvolutionManager.stage_rl_training()` 当前会在 mock RL 阶段写出 `training_run_manifest.json`。`OfflineRLTrainer.train_offline()` 当前会在 `save_dir` 写出 `training_run_manifest.json`，并记录数据摘要、训练步数、模型目录和硬件边界。`RLOptimizer.train()` 当前也会写出同名 manifest，并按以下规则分类：

- `DummyEnv` -> `mock_training`
- 普通仿真环境 -> `sim_training`
- 显式 `hardware_required=True` -> `hardware_in_the_loop`

这一步先解决训练产物“无统一元数据”的问题，并把 sim/mock/hardware 的训练标签从约定提升为受测试约束的行为。

硬件控制器边界本轮也完成了一次收口：

- `IMC22Controller` 支持 `bus=` 注入，默认单元测试不需要真实 `python-can` 总线。
- `IMC22Controller.from_replay()` 可从回放文件构造 controller。
- `ReplayCANBus` 和 `tests/fixtures/imc22_status_replay.json` 可回放 IMC-22 状态帧。
- `HardwareEnvironment(controller=...)` 可直接复用 replay controller 跑 `reset()` / `step()`。
- `RealRobotDriver` 支持 `transport=` 注入和 `RealRobotDriver.from_replay()`。
- `tests/fixtures/real_robot_driver_replay.json` 可回放串口状态帧。
- `tests/test_real_robot_driver.py` 已覆盖 mock driver、replay driver 和 mock SysID 数据采集。
- ROS2 bridge 支持 replay payload 校验 helper。
- `tests/fixtures/ros2_bridge_replay.json` 可回放 bridge 输入数据。
- `tests/test_ros2_bridge_runtime.py` 已在 fake ROS2 runtime 下覆盖 JointState/RobotState 映射、`/cmd_vel` 参数转换和 start/stop service 基线。
- `tests/test_ros2_bridge_smoke.py` 已补成真实 ROS2 Python runtime 下的 opt-in live smoke，并会输出 `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json`。
- CI 已新增 `ros2-bridge-smoke` manual/schedule job，并上传 `ros2-bridge-smoke-artifacts`。
- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json` 当前已为 `passed`，通过证据来自容器化 `ros:humble-ros-base` 运行时，而不是当前宿主 Python 环境。

这意味着默认 pytest 已经能验证 IMC-22 协议编码、节点发现、基础环境闭环，串口驱动的数据包打包和状态回放，以及 ROS2 bridge 的核心消息映射，而不会访问真实 CAN、真实串口或真实 ROS 2 环境；同时目标 ROS2 Humble 环境现在也有单独的 opt-in smoke 入口。

当前已验证：

- `python -m pytest tests\test_training_contracts.py tests\test_verify_mocked.py tests\test_docs_utf8.py tests\test_offline_rl.py tests\test_rl_optimizer_training_contract.py -q`：`12 passed`
- `python -m pytest tests\test_hardware_controller.py -q`：`16 passed, 1 skipped`
- `python -m pytest tests\test_real_robot_driver.py -q`：`5 passed, 1 skipped`
- `python -m pytest tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py -q`：`5 passed`
- `python -m pytest tests\test_ros2_bridge_smoke.py tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py -q`：`5 passed, 1 skipped`
- `docker run --rm -v "<repo>:/workspace" -w /workspace ros:humble-ros-base bash -lc 'source /opt/ros/humble/setup.bash && ... && AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python3 -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv'`：`1 passed`
- `python -m pytest tests\test_training_contracts.py tests\test_verify_mocked.py tests\test_offline_rl.py tests\test_rl_optimizer_training_contract.py tests\test_hardware_controller.py tests\test_real_robot_driver.py tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py tests\test_docs_utf8.py -q`：`38 passed, 2 skipped`
- `python -m pytest -m "not live" -q`：`792 passed, 3 skipped, 3 deselected`

## 当前风险与边界

本次修复聚焦于“主入口文档 + MCP 入口 + 核心用户路径”。以下内容仍然可能需要后续清理：

- `docs/archive_and_reports/` 下的历史归档文档
- 部分未纳入本轮修复的旧文档页面
- 依赖真实 Godot 可执行文件的 headless smoke 路径
- 依赖 Docker Desktop/Linux engine 的 distributed smoke 路径

当前最新 release gate 状态为 `ready`。当前仍保留的信息性边界包括：

- MCP 当前只声明 `stdio` 作为产品化传输面
- live Godot / ROS2 验证刻意不纳入默认 `not live` 门禁
- Godot headless / ROS2 bridge live smoke 仍要求显式环境准备；当前 ROS2 通过证据来自容器化 Humble runtime，而非宿主 Python

这意味着：

- 入口文档已经可以作为当前操作手册使用。
- 历史归档文档不应继续作为一线使用说明。

## 推荐阅读顺序

1. [README.md](../README.md)
2. [MCP 集成说明](mcp.md)
3. [CLI 指南](guides/CLI_GUIDE.md)
4. [Web Panel 指南](guides/WEB_PANEL_GUIDE.md)
5. [发布指南](guides/RELEASE_GUIDE.md)
6. [迁移指南](MIGRATION_GUIDE.md)
