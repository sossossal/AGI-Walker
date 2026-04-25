# External Mainline Inputs Field Map 2026-04-24

更新日期：`2026-04-24`

本页用于把当前需要人工补齐的值，直接映射到 `deployment/external_mainline.inputs.json` 的具体字段，避免在多个模板之间来回查找。

## 目标文件

- `deployment/external_mainline.inputs.json`
- `deployment/external_mainline.inputs.customer_draft.json`
- `deployment/external_mainline.inputs.completed_example.json`

## 1. Customer External Bindings

来源模板：

- `deployment/customer_delivery.external_bindings.customer.overrides.json`

需要同步确认的目标字段：

- `customer_external_bindings.config`
  - 固定为：`deployment/customer_delivery.external_bindings.customer.json`
- `customer_external_bindings.overrides_file`
  - 固定为：`deployment/customer_delivery.external_bindings.customer.overrides.json`
- `customer_external_bindings.sections`
  - 默认保持：`approval_identity`、`archive_target`、`due_trigger`
- `customer_external_bindings.confirmed_by`
  - 填真实确认人
- `customer_external_bindings.confirmation_ticket`
  - 填真实确认单号
- `customer_external_bindings.confirmed_at`
  - 可选，建议填真实确认时间
- `customer_external_bindings.confirmation_notes`
  - 可选，建议写本次窗口说明
- `customer_external_bindings.confirmation_evidence`
  - 可选，建议写真实证据路径

注意：

- 三段 section 的真实元数据本身不直接写进 `deployment/external_mainline.inputs.json`
- 它们应写入 `deployment/customer_delivery.external_bindings.customer.json` 或 `deployment/customer_delivery.external_bindings.customer.overrides.json`

## 2. Industrial Live Evidence

来源模板：

- `deployment/industrial_live_evidence.customer.template.json`

目标字段映射：

- `industrial_live_evidence.target_environment`
  - 来源：`industrial_live_evidence.target_environment`
- `industrial_live_evidence.access_method`
  - 来源：`industrial_live_evidence.access_method`
- `industrial_live_evidence.install_entrypoint`
  - 来源：`industrial_live_evidence.install_entrypoint`
- `industrial_live_evidence.upgrade_entrypoint`
  - 来源：`industrial_live_evidence.upgrade_entrypoint`
- `industrial_live_evidence.rollback_entrypoint`
  - 来源：`industrial_live_evidence.rollback_entrypoint`
- `industrial_live_evidence.backup_restore_entrypoint`
  - 来源：`industrial_live_evidence.backup_restore_entrypoint`
- `industrial_live_evidence.closure_archive_root`
  - 来源：`industrial_live_evidence.closure_archive_root`
- `industrial_live_evidence.evidence_output_root`
  - 来源：`industrial_live_evidence.evidence_output_root`
- `industrial_live_evidence.notes`
  - 来源：`industrial_live_evidence.notes`

## 3. Vulnerability Review

通常无需额外手填，只需保持：

- `vulnerability_exception_review.enabled=true`
- `vulnerability_exception_review.report_output=test_env/release_evidence/security/vulnerability_exception_review_report.json`

后续通过命令刷新：

```bash
python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json
```

## 4. 最短填写顺序

1. 先填 `deployment/customer_delivery.external_bindings.customer.overrides.json`
2. 再填 `deployment/industrial_live_evidence.customer.template.json`
3. 如需集中在一个文件里填写，直接编辑 `deployment/external_mainline.inputs.customer_draft.json`
4. 如需参考完成态结构，可对照 `deployment/external_mainline.inputs.completed_example.json`
5. 把 draft 里的真实值同步到 `deployment/external_mainline.inputs.json`
6. 把 `customer_external_bindings.confirmed_by` 与 `customer_external_bindings.confirmation_ticket` 同步到 `deployment/external_mainline.inputs.json`

## 5. 执行命令

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket>
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

## 6. 完成判定

- `external_mainline_execution_plan_industrial_live_evidence_inputs_ready=true`
- `external_mainline_input_checklist_report.json` 的 customer 缺口消失
- `external_mainline_input_checklist_report.json` 的 industrial 缺口消失
