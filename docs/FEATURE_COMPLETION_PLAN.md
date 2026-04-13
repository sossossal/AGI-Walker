# AGI-Walker Feature Completion Plan

更新日期：`2026-04-12`

## 阶段一：主链稳定化

状态：`完成`

阶段一目标是把项目从“能演示”收敛到“可验证、可回归、可继续扩展”。本阶段不追求重型真实硬件或完整分布式性能，而是锁住核心用户路径和产物契约。

已完成项：

- CLI/Skills/Workflow 主链以 `robot_creation_pipeline` 和 `simulation_ready_robot` 为基准，支持 mock 与 real executor。
- 工作流定义校验从“仅检查存在”升级为结构化校验，可拦截未注册 executor、未知 action、重复/缺失 step、非法 `{step.key}` 引用和未来 step 引用。
- 新增阶段一契约模块 `agi_walker.core.api.workflow_contracts`，固定以下 payload 的 v1 字段和校验器：
  - workflow step artifact
  - workflow definition
  - RobotConfig
  - PartSpec
  - mass optimization result
  - URDF/SDF/MJCF export result
- real workflow step artifact 写入统一 JSON 结构，包含 `schema_version`、`artifact_type`、`workflow`、`step`、`executor`、`action`、`status`、`mode`、`inputs`、`output`、`attempts`、`duration_seconds` 和 `created_at`。
- `WorkflowResult.to_dict()` 输出 `schema_version`、`artifact_type` 和 `artifacts` 摘要，便于 Web/API/MCP 后续复用同一契约。
- real mass optimizer 输出已规范为 JSON-safe 结构，避免 numpy 数组或标量在产物中变成不稳定字符串。
- 回归测试已覆盖契约成功/失败路径、真实工作流产物字段、RobotConfig、优化结果和导出结果。

阶段一验收命令：

```powershell
python -m pytest tests\test_workflow_contracts.py tests\test_workflow_orchestrator.py -q
python tests\run_smoke_tests.py --output-root test_env\smoke_phase1
python -m pytest -m "not live" -q
```

当前已验证：

```text
tests/test_workflow_contracts.py + tests/test_workflow_orchestrator.py: 16 passed
```

## 阶段二：Web/Godot 操作闭环

状态：`完成`

目标：让 Web Panel、Godot session bridge、workflow artifact delivery 成为稳定的一线操作路径。

已完成项：

- workflow artifact manifest 已接入 `schema_version=1.0`、artifact contract 校验结果、step artifact contract 校验结果和 Godot 可加载性判断。
- RobotConfig 送往 Godot 前会复用 `validate_robot_config`，无效配置不再被标记为可加载产物。
- Web workflow list/detail/run/status 响应暴露 workflow/result 契约版本与校验状态。
- Godot session bridge 已固定状态机：`disconnected`、`launching`、`connected`、`schema_ready`、`running`、`failed`。
- `/api/godot/{session_id}/status` 返回版本化状态 payload，并保留 `engine_running`、`tcp_connected`、`last_sensor` 等旧字段兼容。
- workflow-to-Godot delivery record 已记录 artifact contract、session state、session status 和 transport status URL。
- Web workflow 页面已展示 workflow contract、artifact contract、Godot session state 和状态 schema。
- headless Godot smoke 保持 `integration + live` opt-in，不进入默认非 live 门禁。

阶段二验收命令：

```powershell
python -m pytest tests\test_web_godot_session_bridge.py tests\test_web_panel_integration_routes.py -q
python -m pytest tests\test_web_godot_integration.py tests\test_web_panel_aux_apis.py -q
python -m pytest -m "not live" -q
```

当前已验证：

```text
tests/test_web_godot_session_bridge.py + tests/test_web_panel_integration_routes.py: 36 passed
python -m pytest tests\ -m "not integration and not hardware" ...: 704 passed, 28 deselected
```

## 阶段三：Distributed Runtime

状态：`完成`

目标：把 Docker distributed smoke 从“可启动”推进到“可诊断、可重复”。

已完成项：

- 固定 learner、sidecar、mock-godot、web-panel-distributed 的容器入口和健康检查。
- learner 与 sidecar 使用 `python -m agi_walker.core.distributed.*` 模块入口，避免重新引入已移除的 `/app/distributed/*.py` 路径。
- `tests/run_distributed_smoke.py` 支持 `--report-file`，输出 `schema_version=1.0` 的机器可读 smoke report。
- smoke report 已记录 compose file、Web Panel URL、actor id、Zenoh endpoint、服务清单、逐项 check、服务状态和日志诊断。
- smoke runner 默认使用宿主机 Zenoh 端口 `17447/18000`，避免与本地 Web API 的 `8000` 端口冲突。
- smoke runner 默认设置 `AGI_WALKER_FORCE_OFFLOAD=1`，避免 sidecar 初始 `cloud_available=False` 导致 learner 收不到首帧 observation 的启动死锁。
- CI `distributed-smoke` job 已保持在 workflow_dispatch/schedule 专项路径，并上传 `test_env/distributed_smoke` 报告产物，不阻塞普通 unit/smoke。
- smoke checks 已拆分为 `compose_build`、`compose_up`、`web_panel_status`、`distributed_monitor`、`sidecar_start`、`actor_discovery`、`learner_action_loop`。
- 失败诊断已覆盖 Docker buildx 权限、Docker Desktop Linux engine 不可用、旧容器入口路径、Python 模块入口导入失败、Zenoh 连接和 Godot TCP 连接问题。
- Web distributed monitor 使用 `schema_version=1.0`，公开订阅 topic `ag/*/obs`、actor TTL、actor ids 和 monitor 状态。
- `/api/distributed/status` 返回 `schema_version`、`actors`、`actor_ids`、`actors_count` 和 `monitor`，前端分布式页面展示 monitor schema、endpoint、subscription 和 actor 计数。
- sidecar 与 learner 已兼容 Zenoh payload `to_bytes()`，sidecar 日志会记录已发布 observation 和已执行 cloud action。

验收标准：

- `python tests/run_distributed_smoke.py --build --stop-after` 在 Docker 可用环境通过。
- smoke report 留存服务名、actor id、Zenoh endpoint、逐项 check 和结构化诊断。

当前已验证：

```text
python -m pytest tests\test_distributed_smoke_runner.py tests\test_web_panel_aux_apis.py tests\test_active_path_references.py -q
24 passed

python -m pytest -m "not live" -q
792 passed, 3 skipped, 3 deselected

python tests\run_distributed_smoke.py --build --stop-after --report-file test_env\distributed_smoke\distributed_smoke_report.json
PASS

distributed smoke report
status=passed
checks=7/7 passed
actor_id=actor_docker_1
```

## 阶段四：训练、仿真与硬件边界

状态：`完成`

目标：把训练和硬件路径从示例状态推进到可声明能力边界。

已完成项：

- 新增 `agi_walker.core.api.training_contracts`，固定 `training_run` artifact v1 字段、状态、训练类型和校验器。
- `training_run` 支持区分 `mock_training`、`offline_dataset_training`、`sim_training`、`hardware_in_the_loop`。
- `training_run` 统一记录 run id、stage、status、algorithm、environment、inputs、metrics、artifacts、hardware_required、hardware_enabled、started/finished time 和 duration。
- `EvolutionManager.stage_rl_training()` 已写出 `models/<iteration>/rl/training_run_manifest.json`，mock RL 阶段开始复用该契约。
- `OfflineRLTrainer.train_offline()` 已写出 `save_dir/training_run_manifest.json`，offline dataset training 开始复用同一契约。
- `RLOptimizer.train()` 已写出 `save_dir/training_run_manifest.json`，并按 `DummyEnv -> mock_training`、普通仿真环境 -> `sim_training`、显式 `hardware_required=True` -> `hardware_in_the_loop` 进行分类。
- `IMC22Controller` 已支持 `bus=` 注入、`IMC22Controller.from_replay(...)` 和 `ReplayCANBus`，`HardwareEnvironment` 已支持注入 controller，默认 pytest 可通过 replay fixture 覆盖协议编码、节点发现、reset/step。
- `RealRobotDriver` 已支持协议 helper、`transport=` 注入、`RealRobotDriver.from_replay(...)` 和 replay fixture，默认 pytest 可覆盖串口协议打包、状态回放和 mock SysID 数据采集。
- ROS2 bridge 已支持 replay payload 校验和 fake-runtime collected tests，默认 pytest 可验证 JointState/RobotState 映射、`/cmd_vel` 参数转换和 start/stop service 基线，而不依赖 `rclpy` 或 ROS 2 Humble 运行时。
- ROS2 bridge 已新增 `tests/test_ros2_bridge_smoke.py`，在显式设置 `AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1` 时，可用真实 `rclpy` 运行时加仓库内 mock Godot TCP server 验证 topic、service 和 TCP bridge。
- 重型本地 smoke 已统一打上 `live` marker，`python -m pytest -m "not live" -q` 不再应触发真实 Godot、生产 compose、ROS2 bridge live smoke。
- CI 已新增 `ros2-bridge-smoke` manual/schedule job，并将 nightly 跟踪集扩展到 `smoke`、`distributed-smoke`、`godot-headless-smoke`、`ros2-bridge-smoke`。
- `test_env/ros2_bridge_smoke/ros2_bridge_smoke_report.json` 已在 `ros:humble-ros-base` 容器化 ROS2 Humble runtime 下通过并归档，阶段四最后一条 live 证据已补齐。

验收标准：

- 非 hardware 测试不依赖真实设备。
- hardware 测试可在明确配置后运行，并输出可诊断报告。

当前已验证：

```text
python -m pytest tests\test_ros2_bridge_smoke.py tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py -q
5 passed, 1 skipped

docker run --rm -v "<repo>:/workspace" -w /workspace ros:humble-ros-base bash -lc 'source /opt/ros/humble/setup.bash && ... && AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python3 -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv'
1 passed

python -m pytest tests\test_training_contracts.py tests\test_verify_mocked.py tests\test_offline_rl.py tests\test_rl_optimizer_training_contract.py tests\test_hardware_controller.py tests\test_real_robot_driver.py tests\test_ros2_bridge_runtime.py tests\test_ros2_workspace.py tests\test_docs_utf8.py -q
38 passed, 2 skipped

python -m pytest -m "not live" -q
792 passed, 3 skipped, 3 deselected

通过后应看到 mock RL、offline dataset training、sim/HITL 分类 manifest，以及 IMC-22、串口驱动和 ROS2 bridge 的 replay/mock 路径都被校验；live smoke 应保持 opt-in 并从默认非 live 门禁中剥离。
```

## 阶段五：产品化与发布门禁

状态：`完成`

目标：把项目发布质量从“代码可跑”提升到“版本可追踪、文档可执行”。

关键任务：

- 为 CLI、Web、MCP、distributed、Godot integration 建立版本化 capability matrix。
- 发布产物包含 changelog、contract version、test evidence、known limitations。
- 文档分层：快速开始、开发者指南、部署指南、故障排查、历史归档。
- 清理历史文档中已失效入口，避免旧路径重新污染一线文档。

已完成项：

- 新增 `agi_walker.core.api.capability_matrix`，固定 `capability_matrix` artifact v1 字段、发布面范围和校验器。
- capability matrix 当前固定覆盖 `cli`、`web_panel`、`mcp`、`distributed_runtime`、`godot_integration` 五个发布面。
- Web 已公开 `/api/capabilities/matrix`，`/api/system/status` 也会返回矩阵摘要，首页展示矩阵版本与 ready/diagnostic 计数。
- MCP 已新增 `capability_matrix_get` 工具，可直接导出同一份发布面矩阵。
- smoke runner 已新增 capability matrix probe，避免该接口或摘要在后续改动中静默失效。
- 新增 `agi_walker.core.api.release_contracts`，固定 `release_manifest` artifact v1 字段、release gate 统计、test evidence 和 known limitations 校验。
- `release_manifest` 现已附带 `release_policy`，并将 `dev / rc / stable` 的 gate 差异落到机器可读 contract 中。
- `stable` 发布现在还要求显式 `release_approval`，并把 `release_source` 一起写入 manifest；签核字段、当前 Git HEAD 绑定、版本 tag 对齐和 gate 校验已经进入 contract 与 builder 回归测试。
- stable gate 还会检查当前 Git worktree 是否 clean，dirty worktree 会在 readiness 和 promotion checklist 中被明确列为阻塞项。
- 已新增 `tools/check_release_readiness.py`，用于机器可读地预览当前 `rc/stable` 门禁状态，并给出下一步动作。
- 已新增 `tools/build_worktree_cleanup_report.py`，用于把 dirty worktree 非破坏性分类成运行时产物、生成物候选和源码/文档人工审查项。
- 已新增 `tools/build_tracked_artifact_review_report.py`，用于继续聚焦 tracked 的 runtime/generated 候选，给出 diff 摘要和建议动作。
- 已新增 `tools/build_stable_promotion_checklist.py`，用于把当前 HEAD 的 stable 阻塞项转成结构化 promotion checklist，并接入 smoke / targeted 回归。
- 已新增 `tools/run_release_rehearsal.py`，用于在临时 Git repo 中显式演练匹配版本 tag 的 stable 正向路径，并接入 smoke / targeted 回归。
- 当前真实仓库的 cleanup report 已显示 `189` 个待处理路径，其中运行时产物 `6`、生成物候选 `1`、源码/文档人工审查项 `182`；新增 `.gitignore` 规则后，untracked runtime / generated 噪音已被明显压缩，这一步已经把“先清理什么”从口头建议变成结构化报告。
- 当前 tracked artifact review report 已进一步锁定 `7` 个 tracked 候选，需要人工决定回退、保留还是改成不跟踪。
- 新增 `tools/build_release_artifact.py`，可生成 `release_manifest.json` 并输出当前 release gate 状态。
- 新增 `docs/guides/RELEASE_GUIDE.md` 作为当前有效发布手册，archive 中的 release checklist / git release 说明已回指该指南。
- smoke runner 已新增 release artifact probe，默认 smoke 会验证 release manifest 生成链路。

剩余任务：

- 继续清理一线文档中残留的旧入口或过期命令，作为后续持续维护项而不是阶段五阻塞项。

验收标准：

- README 和 docs 当前路径全部可执行。
- release checklist 能关联测试证据和契约版本。
