# SLA / SLO View 2026-04-27

本视图把当前已产品化的 Web、workflow、hardware、vendor、ROS2 和 browser validation 面转成 operator 可读的服务目标。它不是托管式 SLA 合同；正式 SLA 仍需要客户现场拓扑、运维值班、容量压测和真实设备 smoke 证据后单独签署。

## 服务边界

| 服务面 | 当前支持状态 | SLO 指标 | 目标 | 数据来源 |
| --- | --- | --- | --- | --- |
| Web Panel | 支持本地/客户自管部署 | system status 可用性 | 演示窗口内 `GET /api/system/status` 返回 `running` | `docs/guides/MONITORING_ALERTING_BASELINE_20260427.md` |
| Workflow runs | 支持单套控制面 | run 终态可追踪率 | 每个演示 run 均可查到终态或失败原因 | `/api/workflows/runs/{run_id}/status` |
| Operator history | 支持审计记录与导出 | 高风险操作审计完整率 | recover / clear 操作必须有 operator、role、tag/note 或 incident id | `/api/godot/history` |
| Hardware recovery | 支持 replay / serial bridge / CAN 预检 | 故障恢复可解释率 | 每次恢复输出 fault class、动作、结果和失败原因 | Web recovery API、ROS2 `HardwareRecovery` |
| Vendor data | 支持外置 fault table / policy | vendor 晋升阻塞率 | review 或 promotion blocked 时不得晋升 | vendor review / promotion checklist |
| ROS2 bridge | 支持 JSON 兼容与 typed IDL 迁移 | launch/smoke 通过率 | 目标 profile 的 smoke 通过后才可声明 ready | ROS2 launch logs、typed IDL tests |
| Browser validation | 支持手工证据与可选 Playwright smoke | 手工验收闭合率 | closeout 不得缺 manual report | browser validation closeout |

## 告警到 SLO 的映射

| 告警等级 | 影响的 SLO | 判定 |
| --- | --- | --- |
| P1 | Web Panel 可用性、hardware recovery 安全性、live device controllability | 当前演示/交付窗口不可继续，必须升级 |
| P2 | workflow 终态、ROS2/Godot 控制链、browser closeout | 当前功能面不可声明 ready，需当日处理 |
| P3 | vendor review、文档证据、Playwright optional smoke | 不阻塞非 live 开发，但阻塞客户交付签收 |

## 当前非承诺项

- 不承诺 24x7 托管 SLA
- 不承诺高可用、多租户或固定吞吐
- 不承诺 Python 3.14 alpha 作为交付运行时
- 不承诺 replay/simulated 证据等同 live evidence
- 不承诺未通过 vendor review 的真实设备恢复策略

## 交付判定

每次交付或演示前，应至少归档：

1. `GET /api/system/status` 响应
2. 关键 workflow run status
3. operator history export
4. hardware diagnostics 或 recovery result
5. vendor review / promotion checklist（真实硬件场景必需）
6. browser validation closeout
7. 如涉及 ROS2：对应 launch profile、smoke 或 bag replay 结果

可使用 `tools/build_operator_delivery_checklist.py` 基于 `deployment/operator_delivery_checklist.template.json` 汇总上述 evidence，操作说明见 `docs/guides/OPERATOR_DELIVERY_CHECKLIST_AUTOMATION_20260427.md`。
涉及客户现场真实设备时，必须先使用 `tools/build_customer_site_live_smoke_report.py` 生成 `test_env/customer_site_live_smoke/customer_site_live_smoke_report.json`，操作说明见 `docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md`。
涉及 industrial live evidence 或工业签收时，再使用 `tools/build_industrial_live_evidence_archive_report.py --customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json --require-customer-site-smoke` 生成 `test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json`，操作说明见 `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_ARCHIVE_20260427.md`。

## 后续升级条件

只有在以下证据齐备后，才允许把本视图升级为客户 SLA：

- 客户现场容量目标已确认
- 真实设备 smoke 与 live diagnostics 通过
- industrial live evidence 已归档
- 值班、升级、rollback owner 已指定
- 连续交付窗口内没有 P1/P2 未闭合事件
