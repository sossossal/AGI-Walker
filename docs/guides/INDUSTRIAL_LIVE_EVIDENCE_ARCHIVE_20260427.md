# Industrial Live Evidence Archive 2026-04-27

本说明用于把 `industrial_live_evidence` 的真实现场字段和已生成 evidence 汇总成归档报告。该报告只判断输入与证据是否齐备；不会替代客户现场真实设备 smoke。

## 输入字段

字段来自 `deployment/external_mainline.inputs.json` 的 `industrial_live_evidence` 段：

- `target_environment`
- `access_method`
- `install_entrypoint`
- `upgrade_entrypoint`
- `rollback_entrypoint`
- `backup_restore_entrypoint`
- `closure_archive_root`
- `evidence_output_root`

这些字段不得保留 `<...>` 占位符。若仍有占位符，archive report 必须保持 `blocked`。

## 归档报告命令

```bash
python tools/build_industrial_live_evidence_archive_report.py
```

默认输出：

```bash
test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json
```

可显式绑定专项 evidence：

```bash
python tools/build_industrial_live_evidence_archive_report.py \
  --hardware-diagnostics test_env/hardware_live/live_diagnostics_checklist.json \
  --vendor-promotion test_env/hardware_live/vendor_data_promotion_checklist.json \
  --browser-closeout test_env/web_browser_manual_validation/web_browser_validation_closeout.json \
  --customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json
```

工业签收或客户现场真实设备交付时，必须把 customer-site smoke 升级为 strict evidence：

```bash
python tools/build_industrial_live_evidence_archive_report.py \
  --customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json \
  --require-customer-site-smoke
```

## 必需 evidence

| evidence | 默认路径 | 要求 |
| --- | --- | --- |
| operator delivery checklist | `test_env/operator_delivery/operator_delivery_checklist.json` | `status=ready` |
| external-mainline plan | `test_env/release_evidence/operations/external_mainline_execution_plan.json` | industrial live evidence step 不再是 `waiting_external_input` |
| customer-site smoke | `test_env/customer_site_live_smoke/customer_site_live_smoke_report.json` | strict 模式下必须 `status=passed` |

## 可选但建议归档

- hardware diagnostics checklist
- vendor data promotion checklist
- browser validation closeout
- ROS2 smoke / bag replay 结果
- operator history export
- `customer_site_smoke`：customer-site real device smoke report
- hardware live closeout report：`test_env/hardware_live/hardware_live_closeout_report.json`

## 状态规则

| 状态 | 含义 |
| --- | --- |
| `ready` | 真实字段齐备，必需 evidence 齐备，industrial step 不再等待外部输入；若启用 strict customer-site smoke，则 smoke 已 `passed` |
| `blocked` | 字段仍有占位符、必需 evidence 缺失、strict customer-site smoke 缺失/未通过，或 external-mainline 仍等待 industrial 输入 |

## 与客户现场 smoke 的关系

本报告默认只能说明“现场输入和证据归档面已闭合”。若目标是工业级交付签收，还必须继续执行客户现场真实设备 smoke，并使用 `--require-customer-site-smoke` 把 `test_env/customer_site_live_smoke/customer_site_live_smoke_report.json` 作为阻塞性 evidence 归档。执行说明见 `docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md`。
