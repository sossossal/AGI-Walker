# Placeholder Replacement Checklist 2026-04-24

更新日期：`2026-04-24`

本页列出当前会阻塞 industrial closeout 的占位符字段。这些值必须替换成真实客户或真实环境信息，否则：

- `customer external bindings closure` 不能安全闭合
- `industrial live evidence` 不能进入可执行状态
- `security preflight` 可能因非法路径或占位符输入继续失败

## 1. `deployment/customer_delivery.external_bindings.customer.overrides.json`

### `approval_identity`

- [ ] `source_path`
  - 当前值：`<export-root>/changes/CHG-2026-0001/approval.json`
- [ ] `reference`
  - 当前值：`change://<customer-tenant>/CHG-2026-0001`
- [ ] `system_name`
  - 当前值：`<Customer Change Registry>`
- [ ] `portal_url`
  - 当前值：`https://<customer-change-portal>/records/CHG-2026-0001`

### `archive_target`

- [ ] `binding_reference_base`
  - 当前值：`sharepoint://<customer-tenant>/releases/2026-04/window-001`
- [ ] `system_name`
  - 当前值：`<Customer Release Archive>`
- [ ] `portal_url`
  - 当前值：`https://<customer-sharepoint>/sites/releases/archive/2026-04/window-001`

### `due_trigger`

- [ ] `binding_reference_base`
  - 当前值：`servicenow://<customer-tenant>/change-window/CHG-2026-0001`
- [ ] `system_name`
  - 当前值：`<Customer Change Schedule>`
- [ ] `portal_url`
  - 当前值：`https://<customer-servicenow>/change/CHG-2026-0001`

## 2. `deployment/external_mainline.inputs.json`

### `customer_external_bindings`

- [ ] `confirmed_by`
  - 当前值：`<real-customer-confirmed-by>`
- [ ] `confirmed_at`
  - 当前值：`<real-confirmed-at-iso8601>`
- [ ] `confirmation_ticket`
  - 当前值：`<real-customer-confirmation-ticket>`
- [ ] `confirmation_evidence`
  - 当前值：`<customer-confirmation-evidence-path>`

### `industrial_live_evidence`

- [ ] `target_environment`
  - 当前值：`<real-customer-production-environment-id>`
- [ ] `access_method`
  - 当前值：`<real-access-method>`
- [ ] `install_entrypoint`
  - 当前值：`<real-install-command-or-runbook>`
- [ ] `upgrade_entrypoint`
  - 当前值：`<real-upgrade-command-or-runbook>`
- [ ] `rollback_entrypoint`
  - 当前值：`<real-rollback-command-or-runbook>`
- [ ] `backup_restore_entrypoint`
  - 当前值：`<real-backup-restore-command-or-runbook>`
- [ ] `closure_archive_root`
  - 当前值：`<real-customer-archive-root>`
- [ ] `evidence_output_root`
  - 当前值：`test_env/industrial_live_evidence/<real-customer-production-environment-id>`

## 3. 替换后推荐执行顺序

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket> --skip-collect-release-evidence
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
python tools/run_security_release_preflight.py
python tools/run_worktree_release_blocker.py
python tools/check_release_readiness.py
```

## 4. 通过标准

- `test_env/release_evidence/operations/customer_external_bindings_closure_report.json` 已生成
- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json` 不再包含 customer / industrial placeholder 缺口
- `test_env/release_evidence/security_release_preflight_report.json` 不再因 placeholder 路径失败
- `test_env/release_readiness/release_readiness_report.json` 中阻塞项进一步收敛
