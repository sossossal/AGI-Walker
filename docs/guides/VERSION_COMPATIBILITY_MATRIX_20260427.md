# Version Compatibility Matrix 2026-04-27

本矩阵面向 operator、support engineer 和交付负责人，用于判断当前 AGI-Walker 产品面在 Web、Godot、ROS2、hardware transport 与 vendor 数据之间的兼容边界。它补充 `SUPPORT_MATRIX.md`，不替代真实客户环境验收。

## 总览

| 面 | 当前基线 | 状态 | 兼容说明 |
| --- | --- | --- | --- |
| Python runtime | `>=3.10`，CI 主线为 `3.10 / 3.11` | 支持 | 本地 Python `3.14 alpha` 可触发 FastAPI/Pydantic/SQLAlchemy 原生扩展崩溃，不作为交付运行时 |
| Web Panel | FastAPI + static console | 支持 | 基础 workflow、Godot session bridge、operator history、hardware recovery 操作面可用 |
| Godot legacy/session bridge | 外部 TCP 目标或 headless/stub | 条件支持 | 需要显式准备 Godot TCP 或使用仓库 stub；不属于零配置基础部署 |
| Godot Agent backend | `godot-agent` modern backend | 条件支持 | 作为扩展 backend；客户环境需单独验证 agent 目录、模板和 self-check |
| ROS2 | ROS2 Humble | 条件支持 | typed IDL 已入仓；target Humble 环境仍需执行 cutover smoke |
| Hardware transport | `replay / serial_bridge / socketcan / pcan` | 条件支持 | `replay` 可非 live 验证；真实 serial/CAN 需要现场参数 |
| IMC-22 vendor data | `schema_version=1.0`，`data_version=2026.04.26-r1` | 参考基线 | 真实 fault code 晋升必须通过 vendor review 和 promotion checklist |
| Web browser validation | Chromium / Playwright 可选 | 待真实证据 | 工具链已具备；当前缺 `web_browser_manual_validation_report.json` |

## 兼容组合

| 场景 | 推荐组合 | 支持状态 | 必要证据 |
| --- | --- | --- | --- |
| 本地演示 | Python 3.10/3.11 + Web Panel + replay transport | 支持 | smoke / Web static / operator history evidence |
| Web 硬件恢复演示 | Web Panel + replay transport + vendor reference table | 支持 | recovery plan / recover / clear-faults history |
| Godot instruction demo | Web Panel + Godot session bridge + stub/headless | 条件支持 | Godot instruction-set smoke report |
| ROS2 typed bridge demo | ROS2 Humble + `hardware/ros2_ws` + mock Godot TCP | 条件支持 | typed IDL build + ROS2 bridge smoke |
| 真实 IMC-22 联调 | Web Panel + serial_bridge/CAN + live diagnostics | 待现场验证 | live diagnostics、fault telemetry、operator recovery evidence |
| 客户现场交付 | Docker Compose Web Panel + extension evidence + live evidence | 待客户输入 | external-mainline、industrial live evidence、browser validation closeout |

## 不支持或禁止组合

| 组合 | 判定 | 原因 |
| --- | --- | --- |
| Python 3.14 alpha 作为交付运行时 | 不支持 | 当前本地已观察到 FastAPI/Pydantic/SQLAlchemy 导入链崩溃 |
| 未确认 serial/CAN 参数直接执行 live recover | 禁止 | 可能误操作真实设备 |
| replay diagnostics 作为 live evidence | 禁止 | replay 只能证明工具链，不证明现场设备 |
| 未带 `hardware_recovery_operator` 的 token 执行 recover/clear | 禁止 | Web 后端会拒绝高风险恢复动作 |
| 未绑定 sample archive 晋升 vendor fault table | 禁止 | 缺少 raw error 来源与复核证据 |
| 非 Humble ROS2 发行版声明正式支持 | 不支持 | 尚未进入当前兼容矩阵 |

## 契约版本

| 契约 | 当前版本 | 位置 | 兼容规则 |
| --- | --- | --- | --- |
| Workflow contract | `1.0` | workflow artifacts / release evidence | 非 `1.0` 需专项迁移 |
| Godot session status | `1.0` | Web session bridge status | 前端和 history 依赖该 shape |
| ROS2 typed IDL | repo-local message/service set | `hardware/ros2_ws/src/agi_walker_msgs` | JSON 面保留为兼容层，typed IDL 是新标准面 |
| Vendor fault table | `schema_version=1.0` | `deployment/hardware/imc22_reflex_fault_table.json` | 新 raw code 必须绑定 sample archive |
| Recovery policy | `schema_version=1.0` | `deployment/hardware/imc22_reflex_recovery_policy.json` | 新 fault class 必须有恢复策略 |
| Telemetry field map | `schema_version=1.0` | `deployment/hardware/imc22_fault_telemetry_fields.json` | 必填字段缺失会阻塞 vendor review |
| Browser validation | `schema_version=1.0` | `deployment/web_browser_manual_validation.template.json` | 手工报告必须 passed 才能 closeout |

## 升级规则

1. 改 Web hardware recovery API 时，同步更新：
   - `WEB_PANEL_GUIDE.md`
   - `OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md`
   - browser manual validation checklist
2. 改 ROS2 IDL 时，同步更新：
   - `ROS2_TYPED_IDL_MIGRATION.md`
   - `hardware/ros2_ws/README.md`
   - ROS2 workspace tests
3. 改 vendor data 时，同步运行：
   - `tools/build_vendor_fault_data_review.py`
   - `tools/build_vendor_data_promotion_checklist.py`
4. 改 browser 操作面时，同步运行：
   - `tools/run_web_browser_playwright_smoke.py`（可选）
   - `tools/build_web_browser_manual_validation_report.py`
   - `tools/build_web_browser_validation_closeout.py`

## 完成判定

一个交付组合只有同时满足以下条件，才可对外声明为“当前兼容”：

- 组合出现在本矩阵或 `SUPPORT_MATRIX.md`
- 对应 smoke / checklist / evidence 为 passed 或 ready
- live 场景有真实现场证据，不使用 replay 冒充
- 高风险硬件恢复动作有 `hardware_recovery_operator` 权限留痕
- 相关 contract / schema / data version 未发生未记录变更
