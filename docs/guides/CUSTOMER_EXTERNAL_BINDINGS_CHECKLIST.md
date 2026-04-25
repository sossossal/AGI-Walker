# Customer External Bindings Checklist

更新日期：`2026-04-24`

本页用于收口真实客户窗口里的 external bindings。目标是把 `deployment/customer_delivery.external_bindings.customer.json` 从 `binding_state=draft` 推进到可被 `extension_execution_actuals` 识别为 `external_bindings_status=ready`。

## 当前文件

- customer config: `deployment/customer_delivery.external_bindings.customer.json`
- overrides 模板: `deployment/customer_delivery.external_bindings.overrides.example.json`
- customer overrides 草稿: `deployment/customer_delivery.external_bindings.customer.overrides.json`
- managed inputs: `deployment/external_mainline.inputs.json`
- closure report: `test_env/release_evidence/operations/customer_external_bindings_closure_report.json`
- input checklist: `test_env/release_evidence/operations/external_mainline_input_checklist_report.json`
- 示例值说明: `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md`

## 最小待填字段

### `approval_identity`

- `source_path`: 真实客户审批系统导出的记录路径，或可审计导出文件路径
- `source_type`: 真实审批系统类型，例如 `customer_change_registry`
- `reference`: 真实审批记录引用，例如变更单号、审批单号、工单 URI
- `system_name`: 客户实际系统名

### `archive_target`

- `binding_type`: 真实归档目标类型，例如 `sharepoint_archive_library`
- `binding_reference_base`: 真实归档根引用
- `system_name`: 客户实际归档系统名

### `due_trigger`

- `binding_type`: 真实到期触发系统类型，例如 `service_now_schedule`
- `binding_reference_base`: 真实调度或变更窗口引用
- `checked_at`: 最近一次人工核对时间
- `system_name`: 客户实际调度系统名

## 必要确认字段

执行 confirm 或 closure runner 时，三段 section 除了真实系统字段，还必须附带：

- `confirmed_by`
- `confirmed_at`
- `confirmation_ticket`

如果缺这三个字段，`binding_state=confirmed` 不会被视为真实闭合。

## 推荐顺序

1. 先编辑 `deployment/customer_delivery.external_bindings.customer.json`，或基于 `deployment/customer_delivery.external_bindings.overrides.example.json` 另存为客户专用 overrides 文件。
2. 用真实 `confirmed_by` 与 `confirmation_ticket` 执行 confirm。
3. 重建 `extension_execution_actuals` 与 confirmation report。
4. 重跑 external-mainline runner，确认 `customer_external_bindings_closure` 不再停留在 `waiting_external_input`。

## 推荐命令

```bash
python tools/confirm_customer_external_bindings.py --config deployment/customer_delivery.external_bindings.customer.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket>
python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json --external-bindings-config deployment/customer_delivery.external_bindings.customer.json
python tools/build_customer_external_bindings_confirmation_report.py --output test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json --actuals-artifact test_env/release_evidence/operations/extension_execution_actuals.json
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

如果更适合用受管 JSON 覆盖而不是直接改 config，可先复制 example：

```bash
Copy-Item deployment/customer_delivery.external_bindings.overrides.example.json deployment/customer_delivery.external_bindings.customer.overrides.json
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket>
```

## 完成判定

完成后至少应满足：

- `deployment/customer_delivery.external_bindings.customer.json` 中三段 section 都不再是默认生成值
- `customer_external_bindings_confirmation_report.json` 为 `status=ready` 或等价通过态
- `external_mainline_input_checklist_report.json` 不再把 `confirmed_by`、`confirmation_ticket` 和“补齐真实客户 approval/archive/due-trigger 元数据”列为缺口
