# Operator Delivery Checklist Automation 2026-04-27

本说明用于把现场交付 checklist 从手工勾选推进到机器可读产物。它只汇总已有 evidence，不替代真实浏览器点击、真实硬件 smoke 或 industrial live evidence。

## 输入

默认模板：

```bash
deployment/operator_delivery_checklist.template.json
```

默认输出：

```bash
test_env/operator_delivery/operator_delivery_checklist.json
```

模板当前覆盖：

- Web Panel system status
- workflow terminal status
- operator history export
- hardware live diagnostics checklist
- vendor data promotion checklist
- browser validation closeout
- optional ROS2 smoke / launch / bag replay result

## 生成命令

```bash
python tools/build_operator_delivery_checklist.py
```

如某个 evidence 路径不使用默认位置，可显式覆盖：

```bash
python tools/build_operator_delivery_checklist.py \
  --set-evidence system_status=test_env/customer_site/system_status.json \
  --set-evidence browser_validation_closeout=test_env/customer_site/web_browser_validation_closeout.json
```

模板和 `--set-evidence` 中的 evidence path 必须是仓库根相对路径，禁止绝对路径和 `..`。生成器只会读取这个边界内的 JSON evidence；路径越界会让对应 checklist item `blocked` 或 `warning`，不能用现场机器私有路径绕过归档。

## 状态规则

| 状态 | 含义 |
| --- | --- |
| `ready` | 所有 required item 都存在，且状态落在模板允许值内 |
| `blocked` | 至少一个 required item 缺失或状态不符合预期 |
| `warning` | optional item 缺失或状态不符合预期，不阻塞总状态 |

## 归档要求

交付窗口结束前，应把以下产物一起归档：

- `test_env/operator_delivery/operator_delivery_checklist.json`
- `docs/guides/MONITORING_ALERTING_BASELINE_20260427.md`
- `docs/guides/SLA_SLO_VIEW_20260427.md`
- `docs/guides/OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md`
- 如涉及真实硬件：vendor review / promotion checklist 与 live diagnostics evidence

若本次交付进入 industrial live evidence 收口，应继续运行：

```bash
python tools/build_customer_site_live_smoke_report.py --output test_env/customer_site_live_smoke/customer_site_live_smoke_report.json
python tools/build_industrial_live_evidence_archive_report.py \
  --customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json \
  --require-customer-site-smoke
```

工业签收不能只依赖 archive report 的默认兼容模式；必须用 `--require-customer-site-smoke` 把真实客户现场 smoke 作为阻塞性 evidence。

## 不覆盖的事项

- 不自动判定真实客户签收
- 不自动生成 industrial live evidence
- 不把 replay / simulated 证据升级为 live 证据
- 不绕过 `hardware_recovery_operator` 权限要求
