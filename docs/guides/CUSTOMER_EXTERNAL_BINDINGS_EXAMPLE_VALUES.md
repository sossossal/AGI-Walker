# Customer External Bindings Example Values

更新日期：`2026-04-24`

本页给出 `deployment/customer_delivery.external_bindings.customer.overrides.json` 的填写示例。这里的值是模板，不代表真实客户数据。

## `approval_identity`

- `source_path`
  - 示例：`D:/customer_exports/changes/CHG-2026-0001/approval.json`
- `source_type`
  - 推荐：`customer_change_registry`
  - 其他常见值：`service_now_change_record`、`jira_change_registry`
- `reference`
  - 示例：`change://acme-corp/CHG-2026-0001`
- `system_name`
  - 示例：`Acme Change Registry`
- `portal_url`
  - 示例：`https://changes.acme.example/records/CHG-2026-0001`

## `archive_target`

- `binding_type`
  - 推荐：`sharepoint_archive_library`
  - 其他常见值：`s3_release_archive`、`confluence_release_space`
- `binding_reference_base`
  - 示例：`sharepoint://acme-corp/releases/2026-04/window-001`
- `system_name`
  - 示例：`Acme Release Archive`
- `portal_url`
  - 示例：`https://acme.sharepoint.example/sites/releases/archive/2026-04/window-001`

## `due_trigger`

- `binding_type`
  - 推荐：`service_now_schedule`
  - 其他常见值：`jira_change_calendar`、`opsgenie_maintenance_window`
- `binding_reference_base`
  - 示例：`servicenow://acme-corp/change-window/CHG-2026-0001`
- `checked_at`
  - 必须使用 ISO 8601 时间
  - 示例：`2026-04-24T15:30:00+00:00`
- `system_name`
  - 示例：`Acme Change Schedule`
- `portal_url`
  - 示例：`https://acme.service-now.example/change/CHG-2026-0001`

## 执行顺序

1. 先替换 `deployment/customer_delivery.external_bindings.customer.overrides.json` 里的 `<...>` 占位符。
2. 再运行：

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket>
```

3. 最后检查：
   - `test_env/release_evidence/operations/customer_external_bindings_closure_report.json`
   - `test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json`
   - `test_env/release_evidence/operations/external_mainline_input_checklist_report.json`

