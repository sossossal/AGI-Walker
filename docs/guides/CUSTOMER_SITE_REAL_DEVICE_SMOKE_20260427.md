# Customer Site Real Device Smoke 2026-04-27

本说明用于客户现场真实设备 smoke 的最小证据闭环。它不执行真实硬件动作，只把现场 operator 已完成的安全检查、连接、telemetry、限幅命令、fault mapping、recovery plan 和归档证据整理成机器可读报告。

## 输入模板

```bash
deployment/customer_site_live_smoke.template.json
```

现场执行前必须替换所有 `<...>` 占位符，并确认：

- `site.*` 指向真实客户现场和执行窗口
- `device.*` 指向真实 IMC-22/CAN/串口设备
- `safety_precheck.*` 全部为 `true`
- `checks[*].status` 均为 `passed` 或 `ready`
- `archive.closure_archive_root` 指向真实客户归档位置

## 生成报告

```bash
python tools/build_customer_site_live_smoke_report.py
```

默认输出：

```bash
test_env/customer_site_live_smoke/customer_site_live_smoke_report.json
```

如果现场 evidence 文件必须逐项存在，使用：

```bash
python tools/build_customer_site_live_smoke_report.py --require-evidence-files
```

## smoke 检查项

| check | 目的 |
| --- | --- |
| `transport_connect` | 验证真实 transport 可连接 |
| `telemetry_read` | 验证可读取真实 telemetry 和 raw fault code |
| `fault_table_mapping` | 验证 raw fault code 可映射到 vendor fault table |
| `bounded_command` | 验证限幅低风险命令可执行 |
| `recovery_plan` | 验证 recovery plan 可生成并可人工确认 |
| `clear_faults_or_recover` | 在安全前提下执行批准的 clear/recover 动作 |
| `operator_history_archived` | 归档高风险操作审计记录 |

## 安全规则

- 未确认 emergency stop、供电、机械空间和 `hardware_recovery_operator` 角色时，报告必须 `blocked`
- 任一必需 check 非 `passed / ready` 时，报告必须 `blocked`
- 该 smoke 不能用 replay/simulated 结果替代
- P1/P2 事件按 `docs/guides/MONITORING_ALERTING_BASELINE_20260427.md` 升级

## 与 industrial live evidence 的关系

`customer_site_live_smoke_report.json` 是 industrial live evidence 的最终现场 smoke 证据之一。生成后应与以下产物一起归档：

- `test_env/hardware_live/hardware_live_closeout_report.json`
- `test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json`
- `test_env/operator_delivery/operator_delivery_checklist.json`
- vendor review / promotion checklist
- hardware diagnostics / telemetry report
- operator history export

用于工业签收时，重新生成 archive report 时必须启用 strict 绑定：

```bash
python tools/build_industrial_live_evidence_archive_report.py \
  --customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json \
  --require-customer-site-smoke
```
