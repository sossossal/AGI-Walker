# AGI-Walker Customer Acceptance Checklist

更新日期：`2026-04-16`

本页用于客户交付、实施和发布签核时的最小验收清单。默认假设当前交付面是 Docker Compose 主部署，扩展面按需追加。

## 1. 交付物确认

- [ ] `release_manifest` 已生成，且 `release_gate_status=ready`
- [ ] `customer_acceptance_bundle` 已生成，且 `bundle_status=ready`
- [ ] `clean_checkout_smoke_report.json` 已存在并为 `passed`
- [ ] `non_live_gate_report.json` 已存在并为 `passed`
- [ ] `security_posture_report.json` 已存在并为 `ready`
- [ ] `security_release_preflight_report.json` 已存在并为 `passed`

## 2. 部署验收

- [ ] `deployment/compose.env` 与 `deployment/web_panel.env` 已按客户环境落地
- [ ] `docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel` 可完成启动
- [ ] `GET /api/system/status` 可返回成功
- [ ] `GET /api/capabilities/matrix` 可返回成功
- [ ] `docker compose logs` 可读取控制面与 router 日志

## 3. 功能验收

- [ ] Web 主页面可访问
- [ ] workflow 页面可访问并列出当前 runs
- [ ] capability matrix 返回当前发布面状态
- [ ] 如本次交付包含 distributed profile，`GET /api/distributed/status` 可返回成功
- [ ] 如本次交付包含 ROS2 / Godot 扩展，已单独附带对应 live evidence 或专项说明
- [ ] `customer_delivery_surface.extension_support_surface` 已与本次交付拓扑对齐，未把条件支持项误写成默认支持
- [ ] `customer_acceptance_bundle.extension_execution_plan` 已生成，且其中的 deployment / acceptance / rollback 动作与现场实施步骤一致
- [ ] `customer_acceptance_bundle.extension_execution_evidence` 已生成，且 `status=ready`
- [ ] `customer_acceptance_bundle.extension_execution_instance` 已生成，且 `status=ready`
- [ ] `customer_acceptance_bundle.extension_execution_schedule` 已生成，且 `status=ready`
- [ ] `customer_acceptance_bundle.extension_execution_actuals` 已生成，且 `status=ready`
- [ ] `extension_on_call_rehearsal_report.json`、`extension_exception_review_schedule_report.json`、`extension_escalation_closure_report.json` 与 `customer_external_bindings_confirmation_report.json` 已进入本次交付包，不存在只声明模板但没有留痕报告的状态
- [ ] `extension_execution_instance.json` 已进入本次交付包，且 `engagement_id`、`window_id`、`exception_review_due_at`、`delivery_root` 与 `closure_archive_root` 已按本次客户现场实例化
- [ ] `extension_execution_schedule.json` 已进入本次交付包，且 `window_trigger_at`、`signoff_due_at`、`closure_archive_due_at`、`window_trigger_record_path`、`residual_risk_review_record_path` 与 `closure_manifest_path` 已按本次客户窗口实例化
- [ ] `extension_execution_actuals.json` 已进入本次交付包，且 `window_trigger_recorded_at`、`signoff_recorded_at`、`residual_risk_reviewed_at`、`closure_archived_at`、`window_trigger_recorded_by`、`signoff_recorded_by`、`residual_risk_reviewed_by`、`closure_archived_by`、`approval_identity_source_path`、`approval_identity_reference`、`archive_target_binding_type`、`archive_target_binding_reference_base`、`due_trigger_binding_type`、`due_trigger_binding_reference_base`、`due_trigger_checked_at`、`exception_review_due_at`、`closure_archive_due_at` 已按本次客户窗口回填
- [ ] `approval_identity_source.json`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json` 与 `closure_manifest.json` 已在 `extension_execution_actuals.profiles[*]` 指定路径落盘，并可被客户现场抽样核对
- [ ] `extension_execution_plan.profiles[*].runbook_entrypoints` 已指向当前交付实际使用的 runbook / 安装指南 / 专项指南，而不是历史占位文档
- [ ] `extension_execution_plan.profiles[*].execution_template.operator_roles` 已与本次现场角色分工一致，未把交付负责人、客户操作人和回滚负责人留成口头约定
- [ ] `extension_execution_plan.profiles[*].execution_template.upgrade_window_steps` 已覆盖冻结窗口、部署、验收和签收/转回滚顺序
- [ ] `extension_execution_plan.profiles[*].execution_template.handoff_owner_role` 和 `handoff_checkpoints` 已覆盖现场交接责任，不存在“交付团队退场后无人接手”的空档
- [ ] `extension_execution_plan.profiles[*].execution_template.watch_owner_role` 和 `watch_actions` 已覆盖首个值班窗口，不存在切换完成后无人盯守运行态的空档
- [ ] `extension_execution_plan.profiles[*].execution_template.on_call_handoff_owner_role` 和 `on_call_handoff_records` 已覆盖值班窗口结束后的交班记录，不存在首个 watch 窗口结束后无人接续的空档
- [ ] `extension_execution_plan.profiles[*].execution_template.residual_risk_owner_role` 和 `residual_risk_handoff_steps` 已覆盖已知限制 / accepted residual risk 的交接，不存在只在交付侧口头说明的风险
- [ ] `extension_execution_plan.profiles[*].execution_template.exception_review_owner_role` 和 `exception_review_steps` 已覆盖 exception 到期复核动作，不存在沿用过期 exception 仍继续交付的状态
- [ ] `extension_execution_plan.profiles[*].execution_template.incident_escalation_owner_role` 和 `incident_escalation_steps` 已覆盖异常升级路径，不存在现场故障时无人知道该升到哪里
- [ ] `extension_execution_plan.profiles[*].execution_template.escalation_closure_owner_role` 和 `escalation_closure_steps` 已覆盖升级后的闭环证据，不存在事件升级过但 promotion / bundle 无 closure 留痕的状态
- [ ] `extension_execution_plan.profiles[*].execution_template.signoff_checkpoints` 已明确每个签收点所需 artifact，不存在口头通过但无证据留档的状态
- [ ] `extension_execution_plan.profiles[*].execution_template.rollback_owner_role` 与 `rollback_steps` 已对齐，回滚责任和执行顺序可直接照单执行
- [ ] `extension_execution_plan.profiles[*].execution_template.rollback_evidence_owner_role` 与 `rollback_evidence_archive_steps` 已对齐，回滚证据可直接归档到约定目标
- [ ] 如本次交付包含 distributed / ROS2 / Godot 扩展，对应 profile 的 `deployment_commands`、`acceptance_checks` 与 `rollback_prerequisites` 已一并交付并与现场操作册一致

## 4. 安全验收

- [ ] `sbom.json` 已附带到 canonical release evidence
- [ ] Python/container 漏洞扫描报告均已附带
- [ ] `backup_restore_rehearsal_report.json` 已附带并为 `passed`
- [ ] `AGI_WALKER_SECRET_KEY` 已替换默认占位值
- [ ] 当前 active vulnerability exceptions 已明确审批人、到期时间和适用范围

## 5. 性能与运维验收

- [ ] runtime 根目录、数据库、workflow runs、archive、backups 目录已确认
- [ ] incident response、backup/restore、audit trail 文档已纳入交付包
- [ ] support matrix 已与客户环境对齐
- [ ] `CAPACITY_AND_SCALE.md` 已与客户目标拓扑、共享方式和并发预期对齐
- [ ] known limitations 已由交付团队与客户共同确认

## 6. 回滚验收

- [ ] 上一个可回滚版本或 tag 已记录
- [ ] `<runtime-root>/db`、`workflow_runs`、`workflow_archive` 已备份
- [ ] `docker compose ... down` / `up -d --build` 回滚命令已准备
- [ ] 如本次变更涉及 distributed profile，distributed 回滚命令也已准备
- [ ] 如本次交付包含 ROS2 / Godot 扩展，扩展 profile 的 `rollback_prerequisites` 已完成确认，不存在未留档的外部运行时参数

## 推荐命令

```powershell
python tools/check_release_readiness.py
python tools/check_industrial_release_readiness.py
python tools/build_industrial_promotion_checklist.py
python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json --vulnerability-exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json --instance-artifact test_env/release_evidence/operations/extension_execution_instance.json
python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json
python tests/run_distributed_smoke.py --build --stop-after --report-file test_env/distributed_smoke/distributed_smoke_report.json
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'; python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
AGI_WALKER_ENABLE_ROS2_BRIDGE_SMOKE=1 python -m pytest tests/test_ros2_bridge_smoke.py -q -m "integration and live" --tb=short -vv
python tools/collect_release_evidence.py --output-root test_env/release_evidence
python tools/run_security_release_preflight.py --output-root test_env/release_evidence --run-python-vuln-scan --run-container-vuln-scan --container-image-ref deployment-zenoh-router --container-image-ref deployment-web-panel-distributed --vulnerability-exception-input-source deployment/security/vulnerability_exceptions.input.json
python tools/build_customer_acceptance_bundle.py --manifest test_env/release/release_manifest_rc_evidence.json --output test_env/release/customer_acceptance_bundle_rc_evidence.json
```
