# Industrial Live Evidence Checklist

更新日期：`2026-04-24`

本页用于收口 `deployment/external_mainline.inputs.json` 里的 `industrial_live_evidence` 缺口。只有这些字段补齐后，`industrial_delivery_live_evidence` 才会从 `waiting_external_input` 变成 `ready_to_run`。

## 当前待填字段

- `target_environment`
- `access_method`
- `install_entrypoint`
- `upgrade_entrypoint`
- `rollback_entrypoint`
- `backup_restore_entrypoint`
- `closure_archive_root`

## 推荐模板

- `deployment/industrial_live_evidence.customer.template.json`

## 字段说明

- `target_environment`
  - 真实客户生产环境标识
  - 示例：`customer-prod-eu-west-1`
- `access_method`
  - 真实访问方式
  - 示例：`VPN + Bastion host + customer SRE approval window`
- `install_entrypoint`
  - 真实安装命令或 runbook 路径
  - 示例：`docs/runbooks/customer-prod/install.md`
- `upgrade_entrypoint`
  - 真实升级命令或 runbook 路径
- `rollback_entrypoint`
  - 真实回滚命令或 runbook 路径
- `backup_restore_entrypoint`
  - 真实备份恢复命令或 runbook 路径
- `closure_archive_root`
  - 客户现场归档根目录或 URI
  - 示例：`archive://customer-prod-eu-west-1/release-window-2026-04-24`

## 推荐顺序

1. 先把 `deployment/industrial_live_evidence.customer.template.json` 里的占位符替换成真实值。
2. 再把这些值同步进 `deployment/external_mainline.inputs.json` 的 `industrial_live_evidence` 段。
3. 然后执行 external-mainline runner。

## 建议命令

```bash
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

## 完成判定

- `external_mainline_execution_plan_industrial_live_evidence_inputs_ready=true`
- `test_env/release_evidence/operations/external_mainline_execution_plan.json` 中 `industrial_delivery_live_evidence.status=ready_to_run` 或后续完成态
- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json` 不再列出 industrial 缺口

