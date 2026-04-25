# AGI-Walker Feature Completion Plan

更新日期：`2026-04-16`

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
854 passed, 4 skipped, 3 deselected

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
854 passed, 4 skipped, 3 deselected

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
- 已新增 `tools/run_worktree_release_blocker.py`，用于把 `build_worktree_cleanup_report.py` 与 `build_tracked_artifact_review_report.py` 收成统一 runner，并输出结构化 `worktree_release_blocker_report.json`。
- 已新增 `tools/build_worktree_cleanup_report.py`，用于把 dirty worktree 非破坏性分类成运行时产物、生成物候选和源码/文档人工审查项；当前还会额外挂出 `tracked_review_candidate_count` 与 `tracked_review_command`，用于判断是否需要继续生成 tracked artifact review report。
- 已新增 `tools/build_tracked_artifact_review_report.py`，用于继续聚焦 tracked 的 runtime/generated 候选，给出 diff 摘要和建议动作。
- 已新增 `tools/build_stable_promotion_checklist.py`，用于把当前 HEAD 的 stable 阻塞项转成结构化 promotion checklist，并接入 smoke / targeted 回归。
- `tools/check_release_readiness.py` 和 `tools/build_stable_promotion_checklist.py` 现在都支持 `--approval-manifest`，可直接复用已生成 stable manifest 的签核元数据，避免重复手填 approval 参数。
- 已新增 `tools/run_release_rehearsal.py`，用于在临时 Git repo 中显式演练匹配版本 tag 的 stable 正向路径；当前 rehearsal 还会 seed customer delivery / security / remediation 产物，并验证 `customer_delivery_surface` 与 `industrial_delivery_gate` 一并达到 `ready`。
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

## 阶段六：严格工业化交付

状态：`进行中`

目标：把当前“工程上可发布”的 stable 版本，推进到“客户可安装、可验收、可运维、可升级、可审计”的严格工业化交付状态。

当前进展：

- 阶段 A / P0 已启动。
- required release evidence 现已优先从 `test_env/release_evidence/*.json` 的结构化报告读取，并带上生成时间、命令、报告路径和 commit SHA。
- 已新增 `tools/write_pytest_evidence_report.py` 与 `tools/collect_release_evidence.py`，用于统一生成和汇总 release evidence。
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md` 已收口到当前真实支持的 Docker Compose 路径，不再把 Helm / Kubernetes / `docker-compose.prod.yml` 当成一线入口。
- 阶段六的阶段 C 客户部署包装已完成：新增 `deployment/compose.env.example`、`docs/guides/DEPLOYMENT_MATRIX.md`、`docs/guides/CUSTOMER_INSTALLATION_GUIDE.md`，Compose 端口、卷、数据库、workflow runs/archive 和备份目录约定已进入代码、文档和回归测试。
- `tests/test_workflow_orchestrator.py` 中此前会回写 `.agi_data/workflows/artifacts/*_smoke_real/` 的两条 real smoke 测试，现已改为显式使用临时 `output_root`，重复执行后不再改动那 6 个 tracked artifact。
- `tools/run_clean_checkout_smoke.py` 已在目标 clean checkout 上完成整条默认 smoke 的双跑无副作用验证，并生成 `test_env/clean_checkout_smoke_real/clean_checkout_smoke_report.json`，当前 `status=passed`、`runs=2`。
- `clean_checkout_smoke` 现已取代开发态 `smoke_runner`，成为 canonical required release evidence；`tools/collect_release_evidence.py` 和 release builder 会优先读取 `test_env/release_evidence/clean_checkout_smoke_report.json`。
- `python tools/collect_release_evidence.py --output-root test_env/release_evidence` 当前已实跑通过，canonical evidence 三件套均为 `passed`。
- `python tools/build_release_artifact.py --version 2026.04.15-rc-evidence --channel rc --build-id build-20260415-security-preflight --output test_env/release/release_manifest_rc_evidence.json` 当前已证明 builder 会优先消费最新 `test_env/release_evidence/*.json`，并生成 `release_gate_status=ready` 的 rc manifest。
- `python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_rc_evidence.json --output test_env/release/customer_acceptance_bundle_rc_evidence.json` 当前已实跑通过，并成为阶段六客户验收链路的结构化出口，用于汇总 manifest、canonical evidence 和一线交付文档。
- 阶段 D 当前已落地第一批机器可读安全产物：`sbom_artifact`、`vulnerability_scan_report`、`security_posture_report` contract 与 builder 已进入仓库。
- 阶段 D 当前已新增 `backup_restore_rehearsal_report` contract 与 `tools/run_backup_restore_rehearsal.py`，默认 smoke 和 canonical release evidence 都会生成结构化恢复演练报告。
- 阶段 D 当前已补齐 4 份一线基线文档：`SECURITY_BASELINE.md`、`AUDIT_TRAIL_POLICY.md`、`BACKUP_RESTORE_RUNBOOK.md`、`INCIDENT_RESPONSE_MATRIX.md`。
- 默认 smoke 当前已新增 SBOM、Python/container 漏洞扫描报告占位写入，以及 `security_posture_report` 的 ready 路径检查。
- `tools/collect_release_evidence.py` 当前已把 `test_env/release_evidence/security/` 作为 canonical security evidence 目录，并会收集 SBOM、外部漏洞报告、恢复演练报告和 `security_posture_report`。
- `tools/write_vulnerability_scan_report.py` 当前已支持把 `pip-audit JSON` 和 `trivy JSON` 规范化为 `vulnerability_scan_report` contract，`tools/collect_release_evidence.py` 也可直接消费这两类原始 scanner 输出。
- `tools/run_python_vulnerability_scan.py` 与 `tools/run_container_vulnerability_scan.py` 当前已落地；collector 现支持 `--run-python-vuln-scan` / `--run-container-vuln-scan`，优先级保持为 `structured report > raw JSON import > actual scanner execution`。
- `tools/run_security_release_preflight.py` 当前已落地，成为阶段 D 的正式安全预检入口；它会执行 canonical security evidence 收集、读取 `security_posture_report.json` 并写出 `security_release_preflight_report.json`。
- `.github/workflows/ci.yml` 当前已新增 `security-preflight` job，在 manual/schedule 路径上执行真实 `pip-audit` / `trivy` 链路并上传 `test_env/release_evidence_ci`。
- `tools/check_release_readiness.py` 与 `tools/build_stable_promotion_checklist.py` 当前已直接感知 `security_release_preflight_report.json`；在最新 canonical evidence 上，security preflight 已不再是 stable promotion 的 blocking step。
- canonical `test_env/release_evidence/security_release_preflight_report.json` 当前已实跑通过并为 `passed`，因此 readiness/checklist 现在反映的主要剩余工程阻塞已回到 `clean_worktree`，不再是“真实安全预检未通过”。
- `customer_acceptance_bundle` 当前已汇总 14 份 required documents，并显式暴露 `10/10` acceptance reports；最新 canonical bundle 为 `ready`，`customer_acceptance_bundle_security_posture=ready`。
- Phase E 的第二批客户文档当前已进入 bundle 主链：`SUPPORT_MATRIX.md`、`CAPACITY_AND_SCALE.md`、`CUSTOMER_ACCEPTANCE_CHECKLIST.md` 与 `KNOWN_LIMITATIONS.md` 已成为默认 required docs。
- `tools/check_release_readiness.py` 与 `tools/build_stable_promotion_checklist.py` 当前已开始输出 `customer_delivery_surface`，并把 Phase E 文档状态接入 readiness / checklist 的机器可读产物。
- `release_manifest` contract 当前已新增 `customer_delivery_surface` 字段，Phase E 文档状态现在可随 manifest 一起落盘，而不是只停留在 readiness / checklist 二次计算结果。
- `release_manifest` contract 当前已新增 `industrial_delivery_gate` 字段，并正式携带 `deployment_package_status`、`evidence_attested`、`sbom_attached`、`vuln_scan_status`、`backup_restore_verified` 与 support/capacity/checklist/limitations 状态。
- 正式 `industrial` release channel 当前已落地；它继承 stable gate，并额外要求 `customer_delivery_surface.status=ready` 与 `industrial_delivery_gate.status=ready`。
- `tools/check_release_readiness.py` 与 `tools/build_stable_promotion_checklist.py` 当前也会输出 `industrial_delivery_gate`；readiness 会把缺失 industrial requirements 写入 `next_actions`，stable checklist 则新增“确认 industrial delivery gate 已闭合”步骤。
- `tools/check_industrial_release_readiness.py` 与 `tools/build_industrial_promotion_checklist.py` 当前已落地，`industrial` 通道已有独立 readiness / promotion flow；industrial checklist 会把 customer delivery / industrial delivery 都视为正式 blocking step。
- `customer_acceptance_bundle` 当前会按 manifest channel 自动切换 readiness / promotion 报告；`industrial` 默认读取 `industrial_release_readiness_report.json` 与 `industrial_promotion_checklist.json`，smoke 也已接入这条路径。
- `customer_delivery_surface.extension_support_surface` 当前已落地，distributed / ROS2 / Godot / Helm-Kubernetes 的支持边界和非支持范围现在会随 manifest、industrial gate 和 customer acceptance bundle 一起落盘。
- `extension_support_surface.profiles[*]` 当前已进一步携带 `deployment_commands`、`acceptance_checks` 与 `rollback_prerequisites`，把扩展实施动作和回滚前提一起进入机器可读交付面。
- `customer_acceptance_bundle`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始附带 `extension_execution_plan`，把 actionable profile 与 deployment/acceptance 命令汇总成执行面摘要。
- `extension_execution_plan.profiles[*].runbook_entrypoints` 当前已把 `PRODUCTION_DEPLOYMENT_RUNBOOK.md`、客户安装指南和 distributed / ROS2 / Godot 专项指南接进机器可读执行面。
- `extension_execution_plan.profiles[*].execution_template` 当前已把角色分工、升级窗口步骤、值班动作、on-call 交接记录、残余风险交接、exception 到期复核、异常升级闭环证据和回滚证据归档责任接进机器可读执行面，runbook / 安装指南 / promotion checklist / rehearsal report 现在都能引用这组模板。
- `release_manifest`、`customer_acceptance_bundle`、`industrial_release_readiness_report`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始附带 `extension_execution_evidence`，把 `extension_on_call_rehearsal`、`extension_exception_review_schedule` 与 `extension_escalation_closure` 三类留痕报告汇总成执行留痕面。
- `release_manifest`、`customer_acceptance_bundle`、`industrial_release_readiness_report`、`industrial_promotion_checklist` 与 `release_rehearsal_report` 当前已开始附带 `extension_execution_instance`，把客户实例化的交付窗口、exception 复核到期时间和 closure archive 目标汇总成执行实例面。
- 同一批产物当前已继续附带 `extension_execution_schedule`，把 `window_trigger_at`、`signoff_due_at`、`closure_archive_due_at`、`window_trigger_record_path`、`residual_risk_review_record_path` 与 `closure_manifest_path` 汇总成客户窗口排程面。
- 同一批产物当前已开始附带 `extension_execution_actuals`，把 `window_trigger_recorded_at`、`signoff_recorded_at`、`residual_risk_reviewed_at`、`closure_archived_at`、`*_recorded_by`、`approval_identity_source_path`、`approval_identity_source_type`、`approval_identity_reference`、`archive_target_binding_type`、`archive_target_binding_reference_base`、`due_trigger_binding_type`、`due_trigger_binding_reference_base`、`due_trigger_checked_at`、`exception_review_due_at`、`closure_archive_due_at` 以及 profile 级 `approval_identity_source.json`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json`、`closure_manifest.json` 汇总成客户窗口实际执行留痕面。
- 最新 canonical rc manifest 上，`industrial_delivery_gate` 当前已进入真实产物并达到 `ready`；容器扫描残余 findings 现已通过 `vulnerability_remediation_report.status=ready` / `security_posture_report.status=ready` 收口，不再单纯被原始 `container_vuln_scan_report.status=blocked` 卡住。
- `tools/run_release_rehearsal.py` 当前已完成真实 industrial rehearsal 闭环；无论 `--output-root` 使用相对路径还是绝对路径，rehearsal manifest 都要求并实测达到 `customer_delivery_surface.status=ready` 与 `industrial_delivery_gate.status=ready`。
- `python tests/run_smoke_tests.py --output-root test_env/smoke_phase_d_rehearsal_chain` 当前已通过，说明 Phase D 的安全产物链在真实仓库上可以生成 `backup_restore_rehearsal_report`、`security_posture_status=ready` 与 `security_release_preflight_status=passed` 的闭环。
- `tools/run_python_vulnerability_scan.py` 与 `tools/run_container_vulnerability_scan.py` 当前已在 canonical release evidence 路径下完成真实执行；canonical Python 漏洞报告当前已 `passed` 且 `finding_count=0`，Zenoh router 交付镜像已稳定收口到自管 `deployment-zenoh-router` 扫描。
- `tools/build_vulnerability_exception_report.py` 与 `vulnerability_exception_report` contract 当前已落地。tracked canonical input 现已收口到 `deployment/security/vulnerability_exceptions.input.json`，`tools/collect_release_evidence.py` 与 `tools/run_security_release_preflight.py` 默认会从该路径生成结构化 exception report；当前还会追加独立的 `vulnerability_exception_review_report`，把 review-due / expired exception 列表与 follow-up 信号固化成 release evidence。
- `check_release_readiness.py`、`build_stable_promotion_checklist.py` 与 `build_industrial_promotion_checklist.py` 当前也已切到这条独立 review artifact：在 residual-risk 仍需复核时，它们会先推荐 `build_vulnerability_exception_review_report.py`，再提示更新 `deployment/security/vulnerability_exceptions.input.json` 并重跑 security release preflight。
- `tools/build_vulnerability_remediation_report.py` 当前已可消费 `vulnerability_exception_report`，并把 accepted findings 与 unresolved findings 分开输出；`tools/build_security_posture_report.py` 也已直接感知这条 residual risk contract。dockerized Trivy fallback 的 `/scan/image.tar` 镜像标识问题也已修复，因此镜像级 exception 现在能稳定匹配 canonical findings。
- 当前阶段 D 已完成一次 checkpoint 收口：Zenoh router 交付镜像 `deployment-zenoh-router` 当前已复绿，`deployment-web-panel-distributed` 的 `104 findings / 31 affected components` 当前已通过 `31` 条 active no-fix exceptions 进入 canonical remediation；`accepted_finding_count=104`、`unresolved_finding_count=0`、`security_posture_status=ready`、`security_release_preflight_status=passed`，并且最新 canonical review surface 已显式写出 `vulnerability_exception_review_report_status=passed` / `vulnerability_exception_review_candidate_count=31`。

阶段六不再继续扩功能面，而是收口以下五类差距：

- release evidence 自动采集与摘要一致性
- 默认 smoke / readiness / rehearsal 的无副作用执行
- 一线部署路径、安装矩阵、升级回滚文档
- SBOM、漏洞扫描、密钥管理、审计、备份恢复等安全与运维基线
- 客户支持矩阵、容量声明、已知限制、交付验收 checklist

当前焦点：

- 阶段六当前已完成从 Phase D 的 preflight 复绿到 Phase E 文档接线、再到 Phase F rehearsal bridge 和 industrial channel gate 的推进。当前保存的下一步顺序为：
  1. 把 `extension_execution_actuals` 从当前默认 `customer_ticket_registry` / `customer_archive_destination` / `customer_due_trigger_schedule` binding 参考值，继续推进到客户真实审批系统、真实 archive target 和真实调度触发适配，而不是只停留在 repo 内默认 reference string
  2. 持续复跑 canonical security evidence，并在 `2026-05-15` 前用真实修复替换现有 no-fix exceptions
  3. 在独立 industrial readiness / promotion flow 稳定后，推进阶段 F 的完整工业交付演练
  4. 持续把 Phase E 文档和支持边界声明压进后续 industrial release surface

详细执行面、分阶段 deliverable 和退出门禁见：

- [严格工业化交付执行计划](guides/INDUSTRIAL_DELIVERY_PLAN.md)
