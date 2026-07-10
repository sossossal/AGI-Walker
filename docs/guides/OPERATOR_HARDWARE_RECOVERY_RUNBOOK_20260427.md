# Operator Hardware Recovery Runbook 2026-04-27

本手册面向现场 operator / support engineer，用于在 Web / ROS2 / hardware diagnostics 已具备的前提下处理硬件恢复问题。它不替代真实设备 SOP；真实 IMC-22 / CAN / 串口环境必须以客户现场安全规程为准。

版本与组合边界以 `docs/guides/VERSION_COMPATIBILITY_MATRIX_20260427.md` 为准；不在兼容矩阵中的运行时组合不得直接声明为现场支持。

监控与告警等级以 `docs/guides/MONITORING_ALERTING_BASELINE_20260427.md` 为准；P1/P2 事件必须按该基线升级并归档。

## 适用范围

- Web instruction console 中的 `recovery plan / recover_by_fault_class / clear_faults`
- ROS2 hardware recovery service
- `tools/run_hardware_transport_diagnostics.py`
- `tools/build_hardware_live_diagnostics_checklist.py`
- vendor fault table / recovery policy / telemetry field map

## 角色

| 角色 | 允许动作 |
| --- | --- |
| `viewer` | 查看 status、telemetry、recovery plan、operator history |
| `operator` | 发送 instruction set / simulated circuit，记录 operator/tag/note |
| `hardware_recovery_operator` | 执行 `recover_by_fault_class` 和 `clear_faults` |
| `delivery_lead` | 批准 live recovery、升级 incident、归档 evidence |
| `rollback_owner` | 执行 rollback 与 rollback evidence 归档 |

Web 当前要求 `recover_by_fault_class / clear_faults` 的 Bearer token 具备 `hardware_recovery_operator`；管理员 token 会自动兼容映射到该角色。

## 故障树

### A. 页面可打开但无法恢复

1. 检查是否只是在读取 recovery plan：
   - `GET /api/godot/{session_id}/hardware/recovery-plan`
   - 不需要 `hardware_recovery_operator`
2. 如果执行 recover / clear 被拒绝：
   - 检查 token 是否存在
   - 检查 token claim 是否包含 `hardware_roles=["hardware_recovery_operator"]` 或 `roles` 中包含该角色
   - 管理员 token 应自动带 `hardware_recovery_operator`
3. 如果权限通过但恢复失败：
   - 查看响应中的 `hardware_recovery_result_summary`
   - 查看 Web recovery timeline
   - 查看 operator history 中的 `permission / audit_identity / operator / tag / note`

### B. recovery plan 为空或状态异常

1. 确认 session 已连接并有最新 runtime：
   - Web: `/static/instruction-control.html`
   - API: `/api/godot/{session_id}/status`
2. 检查 `hardware_fault_summary` 是否为空：
   - 为空：先运行 diagnostics 或 replay 生成 fault telemetry
   - 非空：检查 fault class 是否存在 recovery policy
3. 检查 vendor 数据：
   - `deployment/hardware/imc22_reflex_fault_table.json`
   - `deployment/hardware/imc22_reflex_recovery_policy.json`
   - `deployment/hardware/imc22_fault_telemetry_fields.json`

### C. 真实设备无响应

1. 不直接反复 recover；先停止高风险动作。
2. 执行 live diagnostics checklist：

```bash
python tools/build_hardware_live_diagnostics_checklist.py --transport serial_bridge --profile-file deployment/hardware/imc22_live_transport.template.json --output test_env/hardware_live/live_diagnostics_checklist.json
```

`--profile-file` 必须是相对路径，不能使用绝对路径或 `..`。生成的 checklist 会在 `profile_file_status` 中记录解析结果，非法路径会保持 `status=blocked`。

3. 确认现场参数：
   - serial port / CAN channel
   - bitrate / baudrate
   - node id 范围
   - 供电与急停状态
4. 再运行 transport diagnostics：

```bash
python tools/run_hardware_transport_diagnostics.py --transport replay --replay-source tests/fixtures/imc22_status_replay.json --fault-table-file deployment/hardware/imc22_reflex_fault_table.json --recovery-policy-file deployment/hardware/imc22_reflex_recovery_policy.json --telemetry-output test_env/hardware_live/hardware_fault_telemetry_report.json
```

真实设备参数未确认前，不要把 replay 结果当作 live evidence。

### D. fault code 不认识或恢复策略缺失

1. 导出 `hardware_fault_telemetry_report.json`。
2. 复制并填写样本归档：

```bash
cp deployment/hardware/imc22_vendor_fault_samples.template.json test_env/hardware_live/imc22_vendor_fault_samples.json
```

3. 运行 vendor review：

```bash
python tools/build_vendor_fault_data_review.py --telemetry-report test_env/hardware_live/hardware_fault_telemetry_report.json --sample-archive-file test_env/hardware_live/imc22_vendor_fault_samples.json --output test_env/hardware_live/vendor_fault_data_review.json
```

4. 只有 review 通过后，才进入 promotion checklist：

```bash
python tools/build_vendor_data_promotion_checklist.py --sample-archive-file test_env/hardware_live/imc22_vendor_fault_samples.json --vendor-review-file test_env/hardware_live/vendor_fault_data_review.json --output test_env/hardware_live/vendor_data_promotion_checklist.json
```

## 恢复流程

1. 建立状态快照：
   - Web 截图
   - latest runtime JSON
   - recovery plan JSON
   - operator / tag / note
2. 判断环境：
   - `replay/simulated`：可执行非 live 验证
   - `live candidate`：必须由 `delivery_lead` 确认
   - `live`：必须遵守现场安全 SOP
3. 执行 `recovery plan`。
4. 由 `hardware_recovery_operator` 执行 `recover_by_fault_class`。
5. 检查 `hardware_recovery_result_summary`。
6. 如 fault 已解除，由 `hardware_recovery_operator` 执行 `clear_faults`。
7. 归档：
   - Web screenshots
   - operator history export
   - diagnostics report
   - telemetry report
   - vendor review / promotion checklist（如涉及新 fault code）

## 升级条件

立即升级到 `delivery_lead`：

- live 设备失联
- fault class 为 `unknown_fault`
- recovery 后 fault count 未下降
- 需要新增 vendor fault table / recovery policy
- 操作员缺少 `hardware_recovery_operator` 权限
- Web / ROS2 / diagnostics 三者结果不一致

升级后必须记录：

- incident id
- session id
- node id / raw error
- attempted action
- operator identity
- rollback owner
- evidence root

## 完成判定

- recovery plan 已生成
- recover / clear 操作权限已记录
- fault summary 已更新
- operator history 可导出
- evidence 已归档到本次交付或现场 support 目录
