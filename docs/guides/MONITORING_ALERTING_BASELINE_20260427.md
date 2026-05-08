# Monitoring and Alerting Baseline 2026-04-27

本基线面向 operator、support engineer 和交付负责人，用于定义 AGI-Walker 当前产品面的最小观测项、告警触发条件和升级路径。它不承诺托管式 24x7 SLA；容量与支持边界仍以 `CAPACITY_AND_SCALE.md`、`SUPPORT_MATRIX.md`、`VERSION_COMPATIBILITY_MATRIX_20260427.md` 和 `SLA_SLO_VIEW_20260427.md` 为准。

## 观测面

| 面 | 观测入口 | 必看字段 | 告警条件 | 首要动作 |
| --- | --- | --- | --- | --- |
| Web Panel | `GET /api/system/status` | `status`、release summaries、distributed monitor | `status != running` 或 route 返回失败 | 重启 Web Panel，检查 env 和日志 |
| Workflow runs | `/api/workflows/runs/{run_id}/status` | `status`、`status_detail`、`cancel_requested`、`failed_step_error` | `failed / timed_out / cancelled` | 查看 run detail 和 live log |
| Operator history | `/api/godot/history`、history export | `kind`、`operator`、`tag`、`audit_identity` | recover / clear 无审计身份或缺 tag/note | 暂停高风险操作，补齐记录 |
| Web hardware recovery | instruction console / recovery API | `hardware_fault_summary`、`hardware_recovery_result_summary`、`permission` | fault count 不降、permission 拒绝、unknown fault | 按硬件恢复 runbook 升级 |
| Hardware diagnostics | `run_hardware_transport_diagnostics.py` | transport status、node coverage、fault telemetry | live diagnostics 非 ready、节点缺失、raw error 未映射 | 停止 live recover，确认现场参数 |
| Vendor data | vendor review / promotion checklist | `status`、`blockers`、`sample_archive_present` | review blocked 或 promotion 非 ready | 不晋升 vendor 数据 |
| ROS2 bridge | ROS2 smoke / launch logs | typed service/topic、runtime publisher、node coverage | typed IDL smoke 未通过或仍只能走 JSON | 保持 JSON 兼容面，延后 cutover |
| Browser validation | browser validation closeout | manual report、Playwright smoke、closeout status | `manual_report_missing` 或 closeout blocked | 按浏览器清单补真实点击证据 |

## 告警等级

| 等级 | 条件 | 响应目标 | 升级对象 |
| --- | --- | --- | --- |
| P1 | live 设备失联、错误 recovery 导致设备不可控、生产 Web Panel 不可访问 | 30 分钟内响应 | `delivery_lead` + `rollback_owner` |
| P2 | hardware fault count 不下降、ROS2/Godot 控制链中断、browser closeout 阻塞演示 | 4 小时内响应 | `delivery_lead` |
| P3 | vendor review blocked、Playwright missing、文档/证据缺口 | 下一个工作日响应 | support engineer |

## 基线检查命令

### Web Panel

```bash
curl http://127.0.0.1:8000/api/system/status
```

### Hardware diagnostics

```bash
python tools/build_hardware_live_diagnostics_checklist.py --transport serial_bridge --profile-file deployment/hardware/imc22_live_transport.template.json --output test_env/hardware_live/live_diagnostics_checklist.json
```

### Vendor data

```bash
python tools/build_vendor_fault_data_review.py --telemetry-report test_env/hardware_live/hardware_fault_telemetry_report.json --sample-archive-file test_env/hardware_live/imc22_vendor_fault_samples.json --output test_env/hardware_live/vendor_fault_data_review.json
python tools/build_vendor_data_promotion_checklist.py --sample-archive-file test_env/hardware_live/imc22_vendor_fault_samples.json --vendor-review-file test_env/hardware_live/vendor_fault_data_review.json --output test_env/hardware_live/vendor_data_promotion_checklist.json
```

### Browser validation

```bash
python tools/run_web_browser_playwright_smoke.py --output test_env/web_browser_manual_validation/playwright_smoke_report.json
python tools/build_web_browser_validation_closeout.py --output test_env/web_browser_manual_validation/web_browser_validation_closeout.json
```

## 升级与恢复

1. 先判断是否 live 场景：
   - live：停止重复 recover，按 `OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md` 升级
   - replay/simulated：可继续非 live 排查
2. 收集证据：
   - system status JSON
   - workflow run detail
   - operator history export
   - diagnostics report
   - fault telemetry report
   - browser validation closeout
3. 归档到当前交付或 support 目录。
4. 对 P1/P2 事件，必须补 incident id、operator、session id、node id、raw error、恢复动作和 rollback owner。

## 非承诺项

- 不承诺 24x7 托管式 SLA
- 不承诺未验证 ROS2 发行版
- 不承诺 Python 3.14 alpha 交付运行时
- 不承诺 replay 结果等价于 live evidence
- 不承诺未带 `hardware_recovery_operator` 的恢复操作

## 完成判定

监控与告警基线闭合需要：

- 本文进入 README、Support Matrix 和下一阶段计划
- 每个关键产品面都有观测入口和告警条件
- P1/P2/P3 有响应目标和升级对象
- live hardware、vendor data、browser validation 的 blocked 状态不会被误判为 ready
