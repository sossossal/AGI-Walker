# Placeholder Ticket-Ready TODO 2026-04-24

更新日期：`2026-04-24`

本页用于把当前阻塞 industrial closeout 的占位符字段整理成可直接抄送到工单或交付任务里的待办列表。

## 1. 客户侧提供

### 文件：`deployment/customer_delivery.external_bindings.customer.overrides.json`

- 字段：`approval_identity.source_path`
  - 当前值：`<export-root>/changes/CHG-2026-0001/approval.json`
  - 需要：真实审批导出文件路径
  - 责任方：客户侧

- 字段：`approval_identity.reference`
  - 当前值：`change://<customer-tenant>/CHG-2026-0001`
  - 需要：真实审批记录引用
  - 责任方：客户侧

- 字段：`approval_identity.system_name`
  - 当前值：`<Customer Change Registry>`
  - 需要：真实审批系统名称
  - 责任方：客户侧

- 字段：`approval_identity.portal_url`
  - 当前值：`https://<customer-change-portal>/records/CHG-2026-0001`
  - 需要：真实审批系统入口
  - 责任方：客户侧

- 字段：`archive_target.binding_reference_base`
  - 当前值：`sharepoint://<customer-tenant>/releases/2026-04/window-001`
  - 需要：真实归档根引用
  - 责任方：客户侧

- 字段：`archive_target.system_name`
  - 当前值：`<Customer Release Archive>`
  - 需要：真实归档系统名称
  - 责任方：客户侧

- 字段：`archive_target.portal_url`
  - 当前值：`https://<customer-sharepoint>/sites/releases/archive/2026-04/window-001`
  - 需要：真实归档系统入口
  - 责任方：客户侧

- 字段：`due_trigger.binding_reference_base`
  - 当前值：`servicenow://<customer-tenant>/change-window/CHG-2026-0001`
  - 需要：真实调度/变更窗口引用
  - 责任方：客户侧

- 字段：`due_trigger.system_name`
  - 当前值：`<Customer Change Schedule>`
  - 需要：真实调度系统名称
  - 责任方：客户侧

- 字段：`due_trigger.portal_url`
  - 当前值：`https://<customer-servicenow>/change/CHG-2026-0001`
  - 需要：真实调度系统入口
  - 责任方：客户侧

## 2. 交付侧补录

### 文件：`deployment/external_mainline.inputs.json`

- 字段：`customer_external_bindings.confirmed_by`
  - 当前值：`<real-customer-confirmed-by>`
  - 需要：真实确认人
  - 责任方：交付侧

- 字段：`customer_external_bindings.confirmed_at`
  - 当前值：`<real-confirmed-at-iso8601>`
  - 需要：真实确认时间
  - 责任方：交付侧

- 字段：`customer_external_bindings.confirmation_ticket`
  - 当前值：`<real-customer-confirmation-ticket>`
  - 需要：真实确认单号
  - 责任方：交付侧

- 字段：`customer_external_bindings.confirmation_evidence`
  - 当前值：`<customer-confirmation-evidence-path>`
  - 需要：真实确认证据路径
  - 责任方：交付侧

## 3. 客户侧与交付侧共同确认

### 文件：`deployment/external_mainline.inputs.json`

- 字段：`industrial_live_evidence.target_environment`
  - 当前值：`<real-customer-production-environment-id>`
  - 需要：真实生产环境标识
  - 责任方：客户侧提供，交付侧录入

- 字段：`industrial_live_evidence.access_method`
  - 当前值：`<real-access-method>`
  - 需要：真实访问方式
  - 责任方：客户侧提供，交付侧录入

- 字段：`industrial_live_evidence.install_entrypoint`
  - 当前值：`<real-install-command-or-runbook>`
  - 需要：真实安装入口
  - 责任方：客户侧提供，交付侧录入

- 字段：`industrial_live_evidence.upgrade_entrypoint`
  - 当前值：`<real-upgrade-command-or-runbook>`
  - 需要：真实升级入口
  - 责任方：客户侧提供，交付侧录入

- 字段：`industrial_live_evidence.rollback_entrypoint`
  - 当前值：`<real-rollback-command-or-runbook>`
  - 需要：真实回滚入口
  - 责任方：客户侧提供，交付侧录入

- 字段：`industrial_live_evidence.backup_restore_entrypoint`
  - 当前值：`<real-backup-restore-command-or-runbook>`
  - 需要：真实备份恢复入口
  - 责任方：客户侧提供，交付侧录入

- 字段：`industrial_live_evidence.closure_archive_root`
  - 当前值：`<real-customer-archive-root>`
  - 需要：真实归档根目录或 URI
  - 责任方：客户侧提供，交付侧录入

- 字段：`industrial_live_evidence.evidence_output_root`
  - 当前值：`test_env/industrial_live_evidence/<real-customer-production-environment-id>`
  - 需要：与真实环境标识一致的输出目录
  - 责任方：交付侧

## 4. 替换后执行

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket> --skip-collect-release-evidence
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
python tools/run_security_release_preflight.py
python tools/run_worktree_release_blocker.py
python tools/check_release_readiness.py
```
