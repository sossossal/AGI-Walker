# AGI-Walker Customer Installation Guide

更新日期：`2026-04-16`

本页面向客户交付和实施团队，只覆盖当前真实支持的 Docker Compose 部署路径。

## 1. 安装前检查

确认以下前提：

- Docker Engine 可用
- `docker compose version` 可正常返回
- 宿主机可写部署目录
- 以下端口未冲突：
  - `8080`
  - `8081`
  - `7447`
  - `8000`

建议预检查命令：

```powershell
docker version
docker compose version
```

## 2. 准备部署配置

复制两份模板：

```powershell
Copy-Item deployment/compose.env.example deployment/compose.env
Copy-Item deployment/web_panel.env.example deployment/web_panel.env
```

至少修改：

- `deployment/compose.env`
  - `AGI_WALKER_RUNTIME_ROOT`
  - `AGI_WALKER_WEB_PORT`
  - `AGI_WALKER_WEB_DISTRIBUTED_PORT`
  - `AGI_WALKER_DATABASE_URL`
  - `AGI_WALKER_WEB_OUTPUT_ROOT`
  - `AGI_WALKER_WEB_ARCHIVE_ROOT`
- `deployment/web_panel.env`
  - `AGI_WALKER_SECRET_KEY`

宿主机目录约定：

- `<runtime-root>/db`
- `<runtime-root>/workflow_runs`
- `<runtime-root>/workflow_archive`
- `<runtime-root>/backups`

如果本次交付包包含 `extension_execution_plan`，应在首次启动前先核对其中的 `runbook_entrypoints` 和 `execution_template`。如果交付包还包含 `extension_execution_evidence`，还应同步确认 `extension_on_call_rehearsal_report.json`、`extension_exception_review_schedule_report.json`、`extension_escalation_closure_report.json` 和 `customer_external_bindings_confirmation_report.json` 四类留痕报告已经就位。如果交付包还包含 `extension_execution_instance`，还应同步确认本次 `engagement_id`、`window_id`、`exception_review_due_at`、`delivery_root` 和 `closure_archive_root` 已按客户现场实例化。如果交付包还包含 `extension_execution_schedule`，还应同步确认 `window_trigger_at`、`signoff_due_at`、`closure_archive_due_at`、`window_trigger_record_path`、`exception_review_record_path`、`residual_risk_review_record_path`、`closure_index_path` 和 `closure_manifest_path` 已指向本次客户窗口。如果交付包还包含 `extension_execution_actuals`，还应同步确认 `window_trigger_recorded_at`、`signoff_recorded_at`、`residual_risk_reviewed_at`、`closure_archived_at`、`window_trigger_recorded_by`、`signoff_recorded_by`、`residual_risk_reviewed_by`、`closure_archived_by`、`approval_identity_source_path`、`approval_identity_reference`、`archive_target_binding_type`、`archive_target_binding_reference_base`、`due_trigger_binding_type`、`due_trigger_binding_reference_base`、`due_trigger_checked_at`、`exception_review_due_at`、`closure_archive_due_at` 已回填，并且 `extension_execution_actuals.profiles[*]` 指向的 `approval_identity_source.json`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json` 与 `closure_manifest.json` 已随交付窗口实际落盘。安装指南只覆盖基础 Compose 路径，distributed / ROS2 / Godot 的专项动作必须以该计划列出的文档入口、角色分工、值班动作、on-call 交接记录、残余风险交接、exception 复核、异常升级闭环和窗口步骤为准。

## 3. 首次启动

最小控制面部署：

```powershell
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

如需 distributed 扩展：

```powershell
$env:AGI_WALKER_GODOT_HOST='host.docker.internal'
$env:AGI_WALKER_GODOT_PORT='9000'
$env:AGI_WALKER_SIDECAR_ACTOR_ID='actor_customer_1'
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed
```

该步骤对应 `extension_execution_plan.profiles[distributed_profile]`。如 bundle 中未声明该 profile，不应在客户现场默认启用 distributed 服务。现场应按该 profile 的 `execution_template.upgrade_window_steps`、`handoff_checkpoints`、`watch_actions`、`on_call_handoff_records`、`residual_risk_handoff_steps`、`exception_review_steps` 和 `signoff_checkpoints` 顺序执行，并核对 `extension_execution_evidence` 中的 `extension_on_call_rehearsal_report.json` 与 `extension_exception_review_schedule_report.json` 已指向本次窗口使用的留痕路径，同时核对 `extension_execution_instance.profiles[distributed_profile]` 已落到本次现场要使用的 handoff / watch / closure 路径，`extension_execution_schedule.profiles[distributed_profile]` 已指向本次窗口的 `window_trigger_record_path`、`exception_review_record_path`、`residual_risk_review_record_path`、`closure_index_path` 与 `closure_manifest_path`，且 `extension_execution_actuals.profiles[distributed_profile]` 已回填本次实际 `approval_identity_source_path`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json` 与 `closure_manifest.json`。

## 4. 健康检查

控制面检查：

```powershell
Invoke-RestMethod http://127.0.0.1:8080/api/system/status
Invoke-RestMethod http://127.0.0.1:8080/api/capabilities/matrix
```

distributed 检查：

```powershell
Invoke-RestMethod http://127.0.0.1:8081/api/system/status
Invoke-RestMethod http://127.0.0.1:8081/api/distributed/status
```

日志检查：

```powershell
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs --tail=200 web-panel zenoh-router
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml logs --tail=200 web-panel-distributed learner sidecar-1
```

如需宿主机留档，建议把日志导出到 `<runtime-root>/logs/compose/`。

如本次交付包含 ROS2 / Godot 扩展，应继续查看 `extension_execution_plan.profiles[*].runbook_entrypoints`、`execution_template.operator_roles`、`watch_actions`、`on_call_handoff_records`、`residual_risk_handoff_steps`、`exception_review_steps` 和 `signoff_checkpoints`，并同步核对 `extension_execution_evidence` 四类报告、`extension_execution_instance.profiles[*]` 的实例化路径、`extension_execution_schedule.profiles[*]` 的窗口排程路径，以及 `extension_execution_actuals.profiles[*]` 的实际执行留痕路径：

- ROS2：`docs/ros2/ROS2_QUICK_START.md`
- Godot：`docs/guides/GODOT_TESTING_GUIDE.md`

## 5. 升级

1. 切换到目标版本或 tag。
2. 重新收集 release evidence。
3. 重建并拉起 Compose 服务。
4. 如交付包声明了扩展 profile，按对应 `execution_template.upgrade_window_steps`、`handoff_checkpoints`、`watch_actions`、`on_call_handoff_records`、`exception_review_steps`、`incident_escalation_steps`、`escalation_closure_steps` 和 `signoff_checkpoints` 顺序执行升级窗口，不要跳过值班、on-call 交接、exception 复核、升级闭环或签收步骤。

```powershell
git checkout <target-tag-or-commit>
python tools/collect_release_evidence.py
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

distributed 版本：

```powershell
git checkout <target-tag-or-commit>
python tools/collect_release_evidence.py
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed
```

## 6. 回滚

回滚前，先备份：

- `<runtime-root>/db`
- `<runtime-root>/workflow_runs`
- `<runtime-root>/workflow_archive`

建议把备份文件写到 `<runtime-root>/backups/<timestamp>/`。

回滚命令：

```powershell
git checkout <previous-stable-tag>
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml down
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

distributed 回滚：

```powershell
git checkout <previous-stable-tag>
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed down
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml --profile distributed up -d --build zenoh-router learner sidecar-1 web-panel-distributed
```

如果 `extension_execution_plan` 中还声明了 ROS2 / Godot 扩展，应在执行基础回滚后继续满足对应 profile 的 `rollback_prerequisites`，并由 `execution_template.rollback_owner_role` 对应的责任人执行 `rollback_steps`；值班窗口中的异常升级应按 `incident_escalation_owner_role` / `incident_escalation_steps` 处理；升级后的 closure 证据应按 `escalation_closure_owner_role` / `escalation_closure_steps` 补回，并同步更新 `extension_escalation_closure_report.json`；回滚后再由 `rollback_evidence_owner_role` 按 `rollback_evidence_archive_steps` 完成证据归档，不要只回滚 Compose 容器而遗漏外部运行时参数。

## 7. 卸载

停止并删除容器：

```powershell
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml down --remove-orphans
```

如需完全清理，再手动删除 `<runtime-root>`。这一步是破坏性操作，执行前先确认备份已经完成。

## 8. 当前边界

- 当前不提供 Helm / Kubernetes 客户交付路径。
- 当前默认数据库是 SQLite，适合单机客户部署，不等同于高可用生产数据库方案。
- 当前日志以 `docker compose logs` 为主，不内建完整日志采集栈。
- 当前 distributed 扩展仍依赖外部 Godot TCP 目标准备完毕。
