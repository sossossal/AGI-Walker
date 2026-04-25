# AGI-Walker 生产部署 Runbook

更新日期：`2026-04-16`

## 适用范围

本 runbook 只描述仓库当前真实支持的自托管部署路径，不再保留旧的 Kubernetes / Helm / 灰度发布脚本模板。

当前一线部署路径：

- Docker Compose: [deployment/docker-compose.yml](D:/新建文件夹/AGI-Walker/deployment/docker-compose.yml)
- Compose 级变量模板: [deployment/compose.env.example](D:/新建文件夹/AGI-Walker/deployment/compose.env.example)
- Web 配置模板: [deployment/web_panel.env.example](D:/新建文件夹/AGI-Walker/deployment/web_panel.env.example)

当前不应作为一线入口的路径：

- `helm/agi-walker`
- `docker-compose.prod.yml`
- Kubernetes / Istio / PrometheusRule / Grafana service mesh 脚本模板

## 当前支持边界

当前可交付的部署形态是“单宿主机、自托管、Docker Compose”。

当前明确支持：

- `zenoh-router`
- `web-panel`
- 可选的 `web-panel-distributed`
- 可选的 `learner`
- 可选的 `sidecar-1`

当前明确不作为默认交付承诺：

- Helm / Kubernetes 集群部署
- 高可用数据库与对象存储编排
- 内建 Grafana / Prometheus 生产栈
- 多租户隔离
- 自动灰度发布

如果客户需要这些能力，必须在交付项目中单独补齐，而不是假定仓库已经产品化提供。

## 扩展执行计划入口

当前客户交付链会把扩展动作汇总到 `extension_execution_plan`。正式运行手册应与该对象保持一致，不要再单独维护另一套命令口径。
当前 release manifest / bundle / industrial promotion / rehearsal 还会附带 `extension_execution_evidence`。正式运行手册除了解释动作，也要对齐四类留痕报告：`extension_on_call_rehearsal_report.json`、`extension_exception_review_schedule_report.json`、`extension_escalation_closure_report.json`、`customer_external_bindings_confirmation_report.json`。
同一批产物现在还会附带 `extension_execution_instance`。正式运行手册还要对齐本次客户实例化的 `engagement_id`、`window_id`、`exception_review_due_at`、`delivery_root`、`closure_archive_root` 和 profile 级实例化路径。
同一批产物现在还会附带 `extension_execution_schedule`。正式运行手册还要对齐 `window_trigger_at`、`signoff_due_at`、`closure_archive_due_at`、`window_trigger_record_path`、`residual_risk_review_record_path` 和 `closure_manifest_path` 这些客户窗口排程字段。
同一批产物现在还会附带 `extension_execution_actuals`。正式运行手册还要对齐 `window_trigger_recorded_at`、`signoff_recorded_at`、`residual_risk_reviewed_at`、`closure_archived_at`、`window_trigger_recorded_by`、`signoff_recorded_by`、`residual_risk_reviewed_by`、`closure_archived_by`、`approval_identity_source_path`、`approval_identity_source_type`、`approval_identity_reference`、`archive_target_binding_type`、`archive_target_binding_reference_base`、`due_trigger_binding_type`、`due_trigger_binding_reference_base`、`due_trigger_checked_at`、`exception_review_due_at`、`closure_archive_due_at`，以及 profile 级 `approval_identity_source.json`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json` 与 `closure_manifest.json` 的实际落盘路径。
每个 profile 现在还会附带 `execution_template`，固定 `operator_roles`、`upgrade_window_steps`、`handoff_owner_role`、`handoff_checkpoints`、`watch_owner_role`、`watch_actions`、`on_call_handoff_owner_role`、`on_call_handoff_records`、`residual_risk_owner_role`、`residual_risk_handoff_steps`、`exception_review_owner_role`、`exception_review_steps`、`incident_escalation_owner_role`、`incident_escalation_steps`、`escalation_closure_owner_role`、`escalation_closure_steps`、`signoff_checkpoints`、`rollback_owner_role`、`rollback_steps`、`rollback_evidence_owner_role` 和 `rollback_evidence_archive_steps`，用于把现场角色分工、升级窗口顺序、值班动作、on-call 交接、风险复核、升级闭环和回滚证据归档责任一起压成机器字段。

- `distributed_profile.runbook_entrypoints`
  - `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
  - `docs/guides/CUSTOMER_INSTALLATION_GUIDE.md`
  - `docs/guides/DISTRIBUTED_GUIDE.md`
- `ros2_bridge_extension.runbook_entrypoints`
  - `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
  - `docs/guides/CUSTOMER_INSTALLATION_GUIDE.md`
  - `docs/ros2/ROS2_QUICK_START.md`
- `godot_extension.runbook_entrypoints`
  - `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
  - `docs/guides/CUSTOMER_INSTALLATION_GUIDE.md`
  - `docs/guides/GODOT_TESTING_GUIDE.md`
- `kubernetes_delivery.runbook_entrypoints`
  - `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
  - `docs/guides/SUPPORT_MATRIX.md`
  - `docs/guides/KNOWN_LIMITATIONS.md`

如果客户 bundle / industrial checklist / rehearsal report 中的 `extension_execution_plan` 与本 runbook 冲突，应先修正 runbook 或 contract，不要现场口头覆盖。
如果现场执行顺序、值班角色、on-call 交接、exception 复核、交付窗口、closure archive 目标、实际触发留痕或回滚责任发生变化，也必须同步更新 `execution_template`、`extension_execution_evidence`、`extension_execution_instance`、`extension_execution_schedule` 与 `extension_execution_actuals`，不要只改文档正文。

### 现场执行模板字段

- `operator_roles`
  - 固定声明交付负责人、客户现场操作人和回滚负责人，避免现场临时口头分派。
- `upgrade_window_steps`
  - 固定升级窗口的执行顺序，要求按顺序完成冻结窗口、部署、验收和签收/转回滚。
- `handoff_owner_role` / `handoff_checkpoints`
  - 固定谁负责把当前运行态、artifact 路径和联系人移交给客户现场团队，避免窗口结束后责任悬空。
- `watch_owner_role` / `watch_actions`
  - 固定谁负责首个值班窗口内的观测动作，避免切换完成后无人盯守运行态。
- `on_call_handoff_owner_role` / `on_call_handoff_records`
  - 固定谁负责把首个值班窗口的结果、联系人和留痕 artifact 交给持续值班人，避免窗口一结束就断档。
- `residual_risk_owner_role` / `residual_risk_handoff_steps`
  - 固定谁负责把当前已知限制、accepted residual risk 和对应 artifact 交给现场团队，避免风险只停留在交付方脑中。
- `exception_review_owner_role` / `exception_review_steps`
  - 固定谁负责复核 exception 输入、到期日和当前覆盖报告，避免 accepted exception 无人续期或过期后继续沿用。
- `incident_escalation_owner_role` / `incident_escalation_steps`
  - 固定异常升级路径，明确何时按 incident matrix 升级、何时转交 rollback owner。
- `escalation_closure_owner_role` / `escalation_closure_steps`
  - 固定谁负责把升级后的 closure 结论和恢复证据补回 promotion/bundle，避免事件升级后没有闭环留痕。
- `signoff_checkpoints`
  - 固定窗口关闭前必须完成的签收点和所需 artifact，避免“口头验收”。
- `rollback_owner_role`
  - 固定本 profile 的回滚责任角色，现场不应再临时推诿。
- `rollback_steps`
  - 固定回滚执行顺序，确保停止当前变更、恢复上一个稳定组合并补录恢复证据。
- `rollback_evidence_owner_role` / `rollback_evidence_archive_steps`
  - 固定谁负责把回滚证据归档到 smoke / bundle / 已知限制等目标，避免回滚后只有口头确认没有留痕。

## 预部署检查

1. 确认 Docker Engine 与 `docker compose` 可用。
2. 确认宿主机端口未冲突：
   - `8080` 对应 `web-panel`
   - `8081` 对应 `web-panel-distributed`
   - `7447` 对应 Zenoh TCP
   - `8000` 对应 Zenoh REST
3. 确认当前发布证据已收集：

```powershell
python tools/collect_release_evidence.py
python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json
python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json
python tools/build_release_artifact.py --version 2026.04.12 --channel stable --build-id build-20260413-stable
```

4. 如需 distributed runtime，确认 Godot 侧 TCP 目标已准备：
   - `AGI_WALKER_GODOT_HOST`
   - `AGI_WALKER_GODOT_PORT`
   - `AGI_WALKER_SIDECAR_ACTOR_ID`

## 配置准备

当前 compose 文件直接引用：

- [deployment/compose.env.example](D:/新建文件夹/AGI-Walker/deployment/compose.env.example)
- [deployment/web_panel.env.example](D:/新建文件夹/AGI-Walker/deployment/web_panel.env.example)

当前建议做法：

1. 复制两份模板：

```powershell
Copy-Item deployment/compose.env.example deployment/compose.env
Copy-Item deployment/web_panel.env.example deployment/web_panel.env
```

2. 在部署时通过 `--env-file deployment/compose.env` 让 compose 使用部署专用 host 变量。
3. 至少审查以下变量：
   - `AGI_WALKER_COMPOSE_WEB_ENV_FILE`
   - `AGI_WALKER_RUNTIME_ROOT`
   - `AGI_WALKER_WEB_PORT`
   - `AGI_WALKER_WEB_DISTRIBUTED_PORT`
   - `AGI_WALKER_DATABASE_URL`
   - `AGI_WALKER_WEB_OUTPUT_ROOT`
   - `AGI_WALKER_WEB_ARCHIVE_ROOT`
   - `AGI_WALKER_SECRET_KEY`
   - `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
   - `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
   - `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
   - `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`
   - `AGI_WALKER_ZENOH_ENDPOINT`
4. 默认持久化目录约定：
   - `<runtime-root>/db`
   - `<runtime-root>/workflow_runs`
   - `<runtime-root>/workflow_archive`
   - `<runtime-root>/backups`
5. 默认日志约定：
   - 运行态日志通过 `docker compose logs` 读取
   - 如需宿主机留档，建议导出到 `<runtime-root>/logs/compose/`

## 部署模式

### 模式 A：控制面最小部署

适用于只需要 Web Panel 和基础系统状态接口的单机部署。

```powershell
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

启动后检查：

```powershell
Invoke-RestMethod http://127.0.0.1:8080/api/system/status
Invoke-RestMethod http://127.0.0.1:8080/api/capabilities/matrix
```

### 模式 B：distributed runtime 扩展部署

适用于需要 learner / sidecar / distributed monitor 的部署。

该模式对应 `extension_execution_plan.profiles[distributed_profile]`，现场执行顺序以 `execution_template.upgrade_window_steps` 为准。

注意：`sidecar-1` 默认会连接 `host.docker.internal:9000`。如果目标环境不是本机 Godot，请先显式设置目标地址。

```powershell
$env:AGI_WALKER_GODOT_HOST='host.docker.internal'
$env:AGI_WALKER_GODOT_PORT='9000'
$env:AGI_WALKER_SIDECAR_ACTOR_ID='actor_customer_1'
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed
```

启动后检查：

```powershell
Invoke-RestMethod http://127.0.0.1:8081/api/system/status
Invoke-RestMethod http://127.0.0.1:8081/api/distributed/status
```

## 健康检查

最小检查：

```powershell
Invoke-RestMethod http://127.0.0.1:8080/api/system/status
```

distributed 检查：

```powershell
Invoke-RestMethod http://127.0.0.1:8081/api/distributed/status
```

日志检查：

```powershell
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs --tail=200 web-panel zenoh-router
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs --tail=200 web-panel-distributed learner sidecar-1
```

## 发布前验收

部署前至少完成以下检查：

```powershell
python tools/collect_release_evidence.py
python tools/check_release_readiness.py --approval-manifest test_env/release/release_manifest_stable.json
python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json
```

如当前交付声明包含 live Godot / ROS2 证据，再补：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv

$env:AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE='1'
python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv
```

ROS2 / Godot 扩展当前不作为基础部署步骤，而是通过 `extension_execution_plan` 进入专项实施动作：

- `ros2_bridge_extension`
  - runbook 入口：`docs/ros2/ROS2_QUICK_START.md`
  - 关键命令：`ros2 launch agi_walker_ros2 agi_walker.launch.py`
  - 专项验收：`AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv`
- `godot_extension`
  - runbook 入口：`docs/guides/GODOT_TESTING_GUIDE.md`
  - 关键前提：`GODOT_EXECUTABLE`、`AGI_WALKER_GODOT_HEADLESS_SCENE`
  - 专项验收：`AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv`

## 升级

当前仓库的一线升级方式是“切换到目标版本检出后重建镜像并重新拉起 compose 服务”，不是 Kubernetes rollout。
如 bundle 声明了扩展 profile，应先对齐对应 `execution_template.operator_roles` / `handoff_owner_role` / `watch_owner_role` / `on_call_handoff_owner_role`，再按 `execution_template.upgrade_window_steps`、`watch_actions`、`on_call_handoff_records`、`residual_risk_handoff_steps`、`exception_review_steps` 和 `signoff_checkpoints` 顺序执行，不要直接跳过窗口冻结、值班、on-call 交接、风险复核或签收步骤。

```powershell
git checkout <target-tag-or-commit>
python tools/collect_release_evidence.py
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

如果使用 distributed profile：

```powershell
git checkout <target-tag-or-commit>
python tools/collect_release_evidence.py
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed
```

## 回滚

当前回滚方式同样是“切换到上一个已知稳定版本，再重新构建并拉起 compose 服务”。
现场回滚责任以 `extension_execution_plan.profiles[*].execution_template.rollback_owner_role` 为准，执行顺序以 `rollback_steps` 为准；异常升级则以 `incident_escalation_owner_role` / `incident_escalation_steps` 为准；升级闭环则以 `escalation_closure_owner_role` / `escalation_closure_steps` 为准；回滚证据归档则以 `rollback_evidence_owner_role` 和 `rollback_evidence_archive_steps` 为准。

```powershell
git checkout <previous-stable-tag>
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml down
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

如果要回滚 distributed profile：

```powershell
git checkout <previous-stable-tag>
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed down
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed
```

如本次交付包含 ROS2 / Godot 扩展，还应按 `extension_execution_plan.profiles[*].rollback_prerequisites` 额外确认：

- ROS2：保留 ROS2 Humble 环境、launch/params 配置和当前 topic/service 连通性记录。
- Godot：保留上一个已验收的 scene、可执行文件路径和 backend 选择。

## 故障定位

### `web-panel` 无法启动

优先检查：

- `docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs web-panel`
- `deployment/web_panel.env.example` 中的配置是否被部署值覆盖
- `8080` 端口是否冲突

### `web-panel-distributed` 无 actor

优先检查：

- `docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs web-panel-distributed`
- `docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs learner`
- `docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs sidecar-1`
- `AGI_WALKER_GODOT_HOST` / `AGI_WALKER_GODOT_PORT` 是否正确
- `AGI_WALKER_ZENOH_TCP_PORT` / `AGI_WALKER_ZENOH_REST_PORT` 是否与宿主机冲突

### distributed smoke 失败

直接看结构化报告：

- [test_env/distributed_smoke/distributed_smoke_report.json](D:/新建文件夹/AGI-Walker/test_env/distributed_smoke/distributed_smoke_report.json)

复现命令：

```powershell
python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json
```

## 当前限制

1. 当前生产 runbook 只覆盖仓库真实存在的 Docker Compose 路径。
2. 当前 compose 没有内建 Helm、Kubernetes、Grafana、Prometheus、PostgreSQL 生产拓扑。
3. 当前持久化策略仍需要交付项目按客户环境补齐，不能把容器本地状态当成最终生产方案。
4. 当前 `sidecar-1` 依赖外部 Godot TCP 目标，不适合作为零配置默认生产面。

## 判定标准

只有以下条件都满足，才能把当前部署判定为“可交付客户使用”的 compose 交付：

1. release evidence 已收集并与目标版本绑定。
2. `release_readiness` 对目标 stable 检出为 `ready`。
3. 控制面健康检查通过。
4. 如声明 distributed 能力，则 distributed status 与 smoke report 均通过。
5. 升级和回滚至少演练一次并留存日志。
