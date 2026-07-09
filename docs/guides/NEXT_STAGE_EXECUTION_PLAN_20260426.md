# 下一阶段开发执行计划

更新日期：`2026-04-28`

本页用于承接当前项目从“主线已闭环”进入“产品面深化”的下一阶段工作。

总 readiness 汇总命令：

```bash
python tools/build_next_stage_readiness_report.py --output test_env/next_stage/next_stage_readiness_report.json
python tools/build_next_stage_external_evidence_checklist.py --readiness-report test_env/next_stage/next_stage_readiness_report.json --output test_env/next_stage/next_stage_external_evidence_checklist.json
python tools/build_next_stage_external_evidence_status_report.py --checklist test_env/next_stage/next_stage_external_evidence_checklist.json --output test_env/next_stage/next_stage_external_evidence_status_report.json --expected-status blocked
```

该报告聚合路线 A-F 的关键 closeout / evidence pack。仓内工具可以生成结构化 `blocked / ready` 判定；真实硬件、客户现场浏览器复核和目标 ROS2 Humble 环境仍需要现场执行。

如果 CI 或本地流程只是为了归档当前 blocked evidence，而不是宣告下一阶段 ready，可以显式运行：

```bash
python tools/build_next_stage_readiness_report.py --output test_env/next_stage/next_stage_readiness_report.json --expected-status blocked
```

该模式只在报告自校验通过且状态确实为 `blocked` 时返回 0，便于保留 artifact；最终验收仍必须使用默认命令或 `--expected-status ready`，并要求 `next_stage_readiness_report.status=ready`。

GitHub Actions 的 `next-stage-readiness` job 会先运行默认命令；如果当前仍缺真实外部 evidence，再用 `--expected-status blocked` 生成并上传 `next-stage-readiness-artifacts`。该 artifact 同时保留 `next_stage_readiness_report.json`、`next_stage_external_evidence_checklist.json` 和 `next_stage_external_evidence_status_report.json`，前者是最终 gate，后两者是现场执行清单与回填状态快照。这只是 evidence retention，不是 release readiness 通过条件；但 status report 自身的 `validation_errors` 仍必须为空，否则 CI job 应失败而不是静默上传 malformed artifact。

如果需要把当前 blocked readiness 转成现场执行清单，运行 `tools/build_next_stage_external_evidence_checklist.py`。该工具输出 `next_stage_external_evidence_checklist.v1`，只把 `blocker_details` / `action_plan` 整理成外部证据项；它不替代总 readiness，也不会把缺真实输入的路线标成 ready。每个 `items[]` 会保留该路线的 `evidence_commands`、`input_templates` 和 `guide_paths`，分别用于重建 route artifact、填写真实输入和查阅执行前检查。若输出 `handoff_validation_errors` 或顶层 `validation_errors` 非空，说明清单自身命令、模板/指南路径或 artifact 结构已漂移，应先修复仓内契约再交给现场执行。

如果现场团队已经按 checklist 回填了部分 artifact，运行 `tools/build_next_stage_external_evidence_status_report.py`。该工具输出 `next_stage_external_evidence_status_report.v1`，逐项检查 checklist 中的 `artifact_path` 是否存在、`actual_status` 是否达到 `target_status`，并输出 `ready_item_count`、`blocked_item_count`、`missing_item_count`、`blocked_items` 和顶层 `validation_errors`。该 status report 是 intake 快照，用于判断回填证据是否足以重跑总 readiness；最终仍必须以 `next_stage_readiness_report.status=ready` 为通过条件。

清单还会输出并自校验 `execution_prerequisites`。现场执行 `items[].evidence_commands` 前必须使用项目支持的 Python 解释器：最低 `>=3.10`，推荐 `3.12`；Windows 上优先使用已注册的 `py -3.12`，否则使用激活后的 3.12 virtualenv 或完整 Python 3.12 路径。不要因为某台机器的裸 `python` 指向未验证版本而改写 evidence 判定，也不要用占位值替代真实外部 evidence。若 stdout 或 JSON 中的 `execution_prerequisite_validation_errors` 非空，应先修复清单契约，再交给现场执行。

当前已经具备：

- 本地最小部署
- distributed 本地链路
- Godot headless smoke
- ROS2 bridge smoke
- Godot / ROS2 / simulated circuit instruction-control 主干
- Web instruction console
- operator history / timeline / compare
- IMC-22 transport profiles
- vendor fault table / recovery policy 外置配置
- hardware diagnostics / fault telemetry export
- Web / ROS2 hardware recovery operations
- 演示与非 live 验收入口
- `tools/build_next_stage_readiness_report.py`

因此下一阶段的重点不再是补主线，而是把已经成型的模拟闭环推进到真实设备、正式操作面、ROS2 标准接口和交付级 live evidence。

## 1. 总证据矩阵

后续执行不再以“是否新增功能”为主判定，而以以下 evidence 是否闭合为准：

| 路线 | 关键 evidence | 目标状态 | 输入来源 |
| --- | --- | --- | --- |
| A 真实硬件联调 | `test_env/hardware_live/hardware_live_closeout_report.json` | `status=ready` | 真实 diagnostics、telemetry、customer-site smoke |
| B Web 硬件操作面 | `deployment/web_hardware_role_policy.json` 与 operator history | 只读受管策略可审计 | 组织级 IAM / RBAC 或受管 token claim |
| C ROS2 标准化 | `test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json` | `status=ready` | 目标 ROS2 Humble typed cutover |
| D 交付产品化 | `test_env/operator_delivery/operator_delivery_checklist.json` | `status=ready` | checklist、runbook、现场证据 |
| D live evidence | `test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json` | `status=ready` | `industrial_live_evidence` 与 external-mainline |
| E vendor 数据 | `test_env/hardware_live/vendor_fault_sample_closeout.json` | `status=ready` | 真实 raw error code 样本 |
| E vendor 晋升 | `test_env/hardware_live/vendor_data_promotion_checklist.json` | `status=ready` | review、sample archive、versioned change log |
| F 浏览器验收 | `test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json` | `status=ready` | manual report、closeout、截图、导出和 console summary |

只有上述 evidence 被 `tools/build_next_stage_readiness_report.py` 聚合为无 blocker，下一阶段计划才算真正闭合。

## 2. 执行命令索引

以下命令是下一阶段的最小可执行闭环。真实硬件、客户现场浏览器复核和 ROS2 Humble 相关输入未落地时，命令应保持 fail-closed 或 blocked，而不是伪造通过。

```bash
# A：真实硬件 live closeout
python tools/build_hardware_live_closeout_report.py --output test_env/hardware_live/hardware_live_closeout_report.json

# E：vendor raw fault sample closeout / review / promotion
python tools/build_vendor_fault_sample_closeout.py --output test_env/hardware_live/vendor_fault_sample_closeout.json
python tools/build_vendor_fault_data_review.py --telemetry-report test_env/hardware_live/hardware_fault_telemetry_report.json --output test_env/hardware_live/vendor_fault_data_review.json
python tools/build_vendor_data_promotion_checklist.py --sample-archive-file deployment/hardware/imc22_vendor_fault_samples.template.json --vendor-review-file test_env/hardware_live/vendor_fault_data_review.json --output test_env/hardware_live/vendor_data_promotion_checklist.json

# C：ROS2 typed IDL cutover
python tools/build_ros2_typed_inventory.py --output test_env/ros2_typed_idl_cutover/typed_inventory.json
python tools/build_ros2_typed_idl_cutover_report.py --output test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json

# D：operator delivery 与 industrial live archive
python tools/build_operator_delivery_checklist.py --output test_env/operator_delivery/operator_delivery_checklist.json
python tools/build_customer_site_live_smoke_report.py --output test_env/customer_site_live_smoke/customer_site_live_smoke_report.json
python tools/build_industrial_live_evidence_archive_report.py --customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json --require-customer-site-smoke --output test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json

# F：浏览器手工验证 closeout
python tools/build_web_browser_manual_validation_report.py --output test_env/web_browser_manual_validation/web_browser_manual_validation_report.json
python tools/build_web_browser_validation_closeout.py --output test_env/web_browser_manual_validation/web_browser_validation_closeout.json
python tools/build_web_browser_validation_evidence_pack.py --output test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json

# 总 readiness
python tools/build_next_stage_readiness_report.py --output test_env/next_stage/next_stage_readiness_report.json

# 外部 evidence 执行清单
python tools/build_next_stage_external_evidence_checklist.py --readiness-report test_env/next_stage/next_stage_readiness_report.json --output test_env/next_stage/next_stage_external_evidence_checklist.json

# 外部 evidence 回填状态快照
python tools/build_next_stage_external_evidence_status_report.py --checklist test_env/next_stage/next_stage_external_evidence_checklist.json --output test_env/next_stage/next_stage_external_evidence_status_report.json --expected-status blocked
```

执行原则：

- 先跑总 readiness，优先按 `action_plan` 执行；需要展开原因时再看 `blocker_details[].blockers / blocked_steps / next_actions`。
- 需要交给现场 operator 或客户团队执行时，再生成 external evidence checklist；该 checklist 的 `items[].acceptance_evidence` 只描述目标 evidence，不是通过证明。
- checklist 的 `items[].evidence_commands` 是该路线重建证据的最小命令链；`items[].input_templates` 是需要复制或填写真实输入的模板；`items[].guide_paths` 是执行前必须阅读的 runbook / checklist。
- checklist 的 `execution_prerequisites` 固定说明 Python 版本和 evidence policy，并通过 `execution_prerequisite_validation_errors` 自校验；若本机 `python` 指向未验证解释器，应换用 `py -3.12`、激活的 3.12 virtualenv 或完整 Python 3.12 路径后再执行命令。
- checklist 的 `handoff_validation_errors` 必须为空；该字段只校验清单本身是否可执行，不代表真实外部 evidence 已完成。
- checklist 的顶层 `validation_errors` 必须为空；该字段校验 schema、时间戳、集合、summary 计数和 status 一致性，不替代总 readiness。
- external evidence status report 的 `validation_errors` 必须为空；`blocked_items` 只说明 checklist item 的当前 artifact 尚未达到 `target_status`，不应被手工改写为 ready。
- external evidence status report 支持 `--expected-status blocked`，只用于归档当前回填状态；最终验收必须省略该参数或显式使用 `--expected-status ready`。
- 总 readiness CLI 会在 stdout 打印 `next_stage_readiness_status`、`next_stage_readiness_expected_status`、`next_stage_readiness_exit_code`、`next_stage_readiness_validation_errors`、`next_stage_readiness_actions` 和 `next_stage_readiness_git`，CI 日志可先看这些摘要，再打开 JSON artifact。
- 每次补完外部 evidence 后，只重跑对应路线命令和最后的总 readiness。
- 不用占位值把 `blocked` 改成 `ready`；缺真实输入时保留 fail-closed 状态。
- Route B 的 Web 权限策略已经仓内闭合；若客户要求在线授权管理，单独走组织级 IAM / RBAC 集成，不在现场控制台写策略。

## 3. 输入文件清单

执行前先确认以下输入是否存在；缺失时不要修改 runner 判定，只补真实 evidence 或受管模板。

| 路线 | 必要输入 | 用途 | 缺失处理 |
| --- | --- | --- | --- |
| A 真实硬件联调 | `deployment/hardware/imc22_live_transport.template.json` | 真实 transport 参数基准 | 复制为现场受管输入后填写端口、bus、baud、节点 |
| A 真实硬件联调 | `test_env/hardware_live/hardware_fault_telemetry_report.json` | live telemetry 与 raw fault 对照 | 由真实 diagnostics / telemetry 导出生成 |
| C ROS2 标准化 | `deployment/ros2_typed_idl_cutover.template.json` | typed cutover 证据模板 | 复制为 cutover 输入并填 live smoke、inventory、rollback owner |
| D 交付产品化 | `deployment/operator_delivery_checklist.template.json` | operator checklist 输入模板 | 用真实 evidence path 覆盖 checklist item |
| D live evidence | `deployment/external_mainline.inputs.json` | industrial live evidence 来源 | 补真实 `industrial_live_evidence.*`，再重建 archive |
| D customer smoke | `deployment/customer_site_live_smoke.template.json` | 客户现场真实设备 smoke 模板 | 现场执行后填真实检查结果与 evidence path |
| E vendor 数据 | `deployment/hardware/imc22_vendor_fault_samples.template.json` | raw fault 样本归档模板 | 复制并填真实 raw error、fault class、设备和时间戳 |
| E vendor 晋升 | `deployment/hardware/imc22_reflex_fault_table.json` 与 `deployment/hardware/imc22_reflex_recovery_policy.json` | fault table / recovery policy 基线 | 只通过版本化变更更新，不直接写临时现场值 |
| F 浏览器验收 | `deployment/web_browser_manual_validation.template.json` | 手工浏览器验证输入模板 | 手工点击后填截图、导出文件、console summary |
| B Web 权限 | `deployment/web_hardware_role_policy.json` | 受管硬件恢复角色策略 | 组织级 IAM / RBAC 未接入前只读使用 |

## 4. blocker 解释表

常见 blocker 的处理方式如下；没有真实证据时保留 blocker 是正确状态。

| blocker | 含义 | 修复动作 | 目标产物 |
| --- | --- | --- | --- |
| `hardware_live_closeout_blocked` | 真实硬件 closeout 尚未 ready | 执行 live diagnostics、telemetry、customer-site smoke 后重建 closeout | `test_env/hardware_live/hardware_live_closeout_report.json` |
| `vendor_fault_sample_closeout_blocked` | 缺真实 raw fault 样本或版本绑定 | 填写真实 sample archive 并重跑 sample closeout | `test_env/hardware_live/vendor_fault_sample_closeout.json` |
| `vendor_fault_data_review_blocked` | telemetry、fault table、policy 对照未通过 | 修正真实样本、fault class 或 recovery policy 后重跑 review | `test_env/hardware_live/vendor_fault_data_review.json` |
| `vendor_data_promotion_blocked` | vendor 数据未满足晋升条件 | 补齐 review、sample archive、`data_version`、`change_log` | `test_env/hardware_live/vendor_data_promotion_checklist.json` |
| `ros2_typed_idl_cutover_blocked` | 目标 Humble typed cutover 证据不足 | 补 typed inventory、live smoke、rollback owner 并重建报告 | `test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json` |
| `operator_delivery_checklist_blocked` | operator checklist evidence 未闭合 | 补 checklist item evidence path 并重建 checklist | `test_env/operator_delivery/operator_delivery_checklist.json` |
| `industrial_live_evidence_archive_blocked` | industrial live evidence、external-mainline 或 strict customer-site smoke 缺失 | 补 `industrial_live_evidence.*`、external-mainline plan 与 `customer_site_live_smoke_report.status=passed` 后重建 archive | `test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json` |
| `manual_report_missing` | 浏览器手工验证报告不存在 | 按清单完成浏览器点击并生成 manual report | `test_env/web_browser_manual_validation/web_browser_manual_validation_report.json` |
| `screenshots_missing` | 浏览器验证缺截图证据 | 归档 instruction console、history、timeline、recovery 操作截图 | `test_env/web_browser_manual_validation/` |
| `exports_missing` | 浏览器验证缺导出文件 | 归档 history / timeline export 文件 | `test_env/web_browser_manual_validation/` |
| `console_error_summary_missing` | 浏览器 console 错误汇总缺失 | 保存 DevTools console summary 或 Playwright console report | `test_env/web_browser_manual_validation/` |
| `validation_closeout_blocked` | manual report / closeout / evidence pack 未全通过 | 先修复上游 blocker，再重建 closeout 和 evidence pack | `test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json` |

当前本地加强后的已知状态：

- `web_browser_evidence_pack` 当前按 canonical artifact 仍为 `blocked`：Playwright 支持证据已通过，但 manual report、screenshots、exports、console summary 与 validation closeout 仍需真实浏览器验收输入后重建。
- `operator_delivery_checklist` 已补齐 `system_status`、`workflow_status`、`operator_history_export`、`browser_validation_closeout` 与 `hardware_live_diagnostics`。
- `operator_delivery_checklist` 当前已无 warning，仅剩必需 blocker：`vendor_data_promotion`。
- `vendor_data_promotion` 不应以模板或占位样本强制置 `ready`，必须等待真实 vendor fault sample、review 与 promotion 条件闭合。
- `vendor_fault_data_review` 当前缺 `test_env/hardware_live/hardware_fault_telemetry_report.json`；没有真实 telemetry entries 时不应生成通过态 review。
- `vendor_data_promotion_checklist` 当前应保持 `blocked`，明确卡在 `change_request` 占位值和 `vendor_review` 未通过。
- `industrial_live_evidence_archive_report` 当前已支持 strict customer-site smoke 绑定；工业签收时 `--require-customer-site-smoke` 会把缺失或未通过的 `customer_site_smoke` 升级为 blocker。当前 canonical strict report 仍应保持 `blocked`，因为真实 customer-site smoke 与 operator/vendor 现场证据尚未闭合。
- `next_stage_readiness_report` 当前会输出 `action_plan`、`blocker_details`、`generated_at`、`git` 和 `validation_errors`：前者给执行顺序与首要动作，后者保留每个阻塞 artifact 的内部 `blockers`、`blocked_steps`、`warnings` 和 `next_actions`；`next_stage_readiness_report.generated_at` 是带时区的报告生成时间，用于区分旧 artifact 与本轮 evidence；`next_stage_readiness_report.git` 记录 commit、branch 和 dirty 状态，用于避免把不同分支或脏工作区 evidence 混为同一份 readiness 结果；在 GitHub Actions detached checkout 中，branch/ref 和 commit 会从 `GITHUB_HEAD_REF`、`GITHUB_REF_NAME` 与 `GITHUB_SHA` 回填；`validation_errors` 固定校验 summary 计数、blocker/detail/action 顺序和基础元数据形状。
- ROS2 typed surfaces 的本地代码 inventory 已可由 `tools/build_ros2_typed_inventory.py` 重建；当前 typed surface blocker 已收敛，仅保留真实 cutover 输入 `target_environment`、`operator`、`rollback_owner` 与 `json_writers_disabled`。
- `next_stage_readiness_report.action_plan` 当前会标注 `execution_scope` 和 `requires_real_input`；代码层 blocker 清零后，剩余 action 应全部归类为 `external_input`。
- `next_stage_external_evidence_checklist.json` 当前可从 readiness report 生成，输出每个 unresolved artifact 的 `target_status`、`issues`、`primary_next_action`、`acceptance_evidence`、`evidence_commands`、`input_templates` 与 `guide_paths`，用于现场执行分派；同时输出并自校验 `execution_prerequisites` 固定 Python 版本和 evidence policy，输出 `handoff_validation_errors` 和顶层 `validation_errors` 防止 stale handoff 或 malformed checklist artifact 进入现场流程；`next_stage_external_evidence_status_report.json` 可进一步检查 checklist item artifact 是否已经回填到目标状态；最终仍以 `next_stage_readiness_report.status=ready` 为准。

## 5. 真实环境执行前检查

进入现场或真实设备节点前，先确认以下前置条件。任一项缺失时，只能运行 dry-run / non-live 检查，不能把 live evidence 标成 ready。

| 检查项 | 必须确认 | 失败时处理 |
| --- | --- | --- |
| 硬件连接 | IMC-22 / CAN / 串口设备、bus、baud、node id 与 `imc22_live_transport` 输入一致 | 停止 live diagnostics，先修正 transport 输入 |
| 操作权限 | 当前 operator 具备 `hardware_recovery_operator` 或管理员 token claim | 禁止执行 recover / clear-faults，只允许只读诊断 |
| 安全边界 | 限幅、watchdog、急停、回滚入口已由现场负责人确认 | 不执行任何真实恢复动作 |
| ROS2 环境 | 目标节点为 ROS2 Humble，typed IDL package 可构建且 source 后可见 | 只运行 replay/local profile，不进入 typed cutover |
| 浏览器环境 | Chromium 或等价浏览器可打开 Web console，DevTools console 可导出 | 客户现场复核缺失时保留现场验收缺口，不覆盖本地已通过 evidence pack |
| Playwright 可选项 | 若要求自动浏览器证据，需安装 Playwright browser runtime | 无自动化运行时时改走手工浏览器验收，并记录缺口 |
| 客户现场证据 | evidence 目录可写，截图、导出、console summary 有归档路径 | 不执行最终 closeout，仅记录缺口 |
| industrial live 输入 | `industrial_live_evidence.*` 字段已由现场负责人确认，且 `customer_site_live_smoke_report.status=passed` | `industrial_live_evidence_archive_report` 保持 blocked |
| vendor 数据 | raw error 样本包含设备、时间戳、raw code、fault class、恢复结果 | 不允许晋升 fault table / recovery policy |

## 6. 总体优先级

建议按以下顺序推进：

1. 真实硬件联调
2. Web 硬件操作面深化
3. ROS2 标准化与生态接入
4. vendor 数据沉淀
5. 交付级 live evidence
6. 浏览器手工验证

原因：

- 当前后端 contract 与 smoke 主线已经稳定
- Web 控制台和恢复操作面已经可用，下一步应优先验证真实设备
- 最大真实性缺口在真实硬件联调，而不是 replay / smoke
- ROS2 和交付面需要依赖真实 telemetry、fault code 和恢复策略继续收敛
- 最近 Web UI 改动有静态和后端测试，但还缺浏览器点击级验证

## 7. 路线 A：真实硬件联调

### 目标

把当前 `serial_bridge`、fault table、recovery policy、diagnostics、Web/ROS2 恢复操作面接到真实 IMC-22 / CAN / 串口设备上做 live 验证。

### 当前已具备

- `socketcan / pcan / replay / serial_bridge` transport profile
- `tools/run_hardware_transport_diagnostics.py`
- `tools/build_hardware_live_diagnostics_checklist.py`
- `tools/build_hardware_live_closeout_report.py`
- `deployment/hardware/imc22_live_transport.template.json`
- 外置 fault table：`deployment/hardware/imc22_reflex_fault_table.json`
- 外置 recovery policy：`deployment/hardware/imc22_reflex_recovery_policy.json`
- Web / ROS2 recovery operations
- fault telemetry export
- hardware live closeout report

### 剩余外部执行

- 仓内 closeout 汇总已具备。
- 仍需在真实设备上执行 diagnostics、telemetry、customer-site smoke。
- 现场结果必须归档到 `test_env/hardware_live/hardware_live_closeout_report.json`。
- Route A 的最终判定以 `hardware_live_closeout_report.status=ready` 为准。

### 建议落点

- `agi_walker/core/api/godot_robot_env/hardware_controller.py`
- `tools/build_hardware_live_diagnostics_checklist.py`
- `tools/run_hardware_transport_diagnostics.py`
- `tools/build_hardware_live_closeout_report.py`
- `deployment/hardware/`
- `docs/hardware/HARDWARE_INTEGRATION_GUIDE.md`

### 完成标准

- `hardware_live_closeout_report.status=ready`
- 真实设备 diagnostics、telemetry、customer-site smoke 都已归档且无 fail-closed blocker
- 至少一次真实恢复流程能生成 plan、执行 recovery、清除 fault 状态
- `next_stage_readiness_report` 不再阻塞 `hardware_live_closeout`

## 8. 路线 B：Web 硬件操作面深化

### 目标

把当前 Web console 从“能发命令和看摘要”推进成可用于现场操作的硬件恢复控制台。

### 当前已具备

- instruction-set / simulated-circuit 发送
- telemetry WebSocket
- operator history / replay / timeline / compare
- hardware fault summary
- recovery plan / recover / clear-faults 按钮
- recovery summary 展示
- 节点级 fault / recovery 状态表
- recovery 操作时间线
- failure drill-down
- 高风险恢复操作确认弹窗
- recover / clear-faults 操作历史与审计身份绑定
- replay / simulated 与 live candidate 模式提示
- recover / clear-faults `hardware_recovery_operator` Bearer token 强制权限判定；管理员 token 自动兼容映射
- `deployment/web_hardware_role_policy.json` 受管角色策略
- `/api/godot/hardware/role-policy` 只读角色策略 API
- instruction console 展示硬件角色、允许操作和已分配用户

### 剩余外部执行

- 仓内已提供只读受管 role policy 展示。
- 若需要在线增删角色，后续应接入正式组织级 IAM / RBAC。
- 不建议在现场控制台直接提供写权限，避免权限变更绕过审计。

### 建议落点

- `web_panel/static/instruction-control.html`
- `web_panel/static/operator-history.html`
- `web_panel/static/operator-history-timeline.html`
- `web_panel/godot_session_bridge.py`
- `docs/guides/WEB_PANEL_GUIDE.md`

### 完成标准

- 当前仓内 Web 操作面已满足节点级查看、确认、历史、权限留痕和恢复结果可视化
- 若客户要求在线授权管理，以组织级 IAM / RBAC 接入完成为准
- 现场控制台不直接写角色策略，角色变更必须走受管策略与审计链

## 9. 路线 C：ROS2 标准化与生态接入

### 目标

把当前 ROS2 bridge 从“桥接可用”推进到“标准接口层可复用”。

### 当前已具备

- instruction-set topic
- simulated circuit topic
- runtime telemetry publisher
- replay / apply-default service
- hardware recovery services
- hardware fault / recovery summary
- typed custom IDL for instruction-set, simulated-circuit and hardware-recovery contracts
- launch profile selection via `config_file` with local / replay / live profiles
- typed compatibility topics/services alongside the legacy JSON surface
- bag-style replay fixture and ordered fake-runtime replay helper
- multi-node replay smoke report with expected node coverage and state coverage checks
- behavior / navigation / perception typed topic hooks
- typed IDL migration runbook and JSON deprecation policy
- `deployment/ros2_typed_idl_cutover.template.json`
- `tools/build_ros2_typed_idl_cutover_report.py`

### 剩余外部执行

- 仓内 typed IDL cutover closeout 已具备。
- 仍需在目标 ROS2 Humble 环境执行下游 typed cutover。
- 必须禁用对应 JSON writer，并归档 live smoke、typed inventory 与 rollback owner。
- Route C 的最终判定以 `test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json.status=ready` 为准。

### 建议落点

- `hardware/ros2_ws/src/agi_walker_ros2/`
- `hardware/ros2_ws/launch/`
- `docs/ros2/`

### 完成标准

- `ros2_typed_idl_cutover_report.status=ready`
- 下游 typed inventory、live smoke、rollback owner 已归档
- 对应 cutover 面的 JSON writer 已禁用或有明确 rollback 豁免
- `next_stage_readiness_report` 不再阻塞 `ros2_typed_idl_cutover`

## 10. 路线 D：交付与运维产品化

### 目标

把现有 smoke / runbook / signoff 继续推进成 operator-facing 的交付体系。

### 当前已具备

- release/signoff/readiness 主链
- instruction-control validation runner
- 演示 runbook
- non-live smoke 与本地专项验证
- Web/ROS2/hardware replay 证据链
- `docs/guides/OPERATOR_HARDWARE_RECOVERY_RUNBOOK_20260427.md`
- `docs/guides/VERSION_COMPATIBILITY_MATRIX_20260427.md`
- `docs/guides/MONITORING_ALERTING_BASELINE_20260427.md`
- `docs/guides/SLA_SLO_VIEW_20260427.md`
- `docs/guides/OPERATOR_DELIVERY_CHECKLIST_AUTOMATION_20260427.md`
- `deployment/operator_delivery_checklist.template.json`
- `tools/build_operator_delivery_checklist.py`
- `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_ARCHIVE_20260427.md`
- `tools/build_industrial_live_evidence_archive_report.py`
- `docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md`
- `deployment/customer_site_live_smoke.template.json`
- `tools/build_customer_site_live_smoke_report.py`

### 本轮进展

- 已新增 operator-facing 硬件恢复 runbook，覆盖角色、故障树、恢复流程、升级条件和证据归档。
- 已把 Web recovery、ROS2 service、hardware diagnostics、vendor review / promotion checklist 串成现场排障路径。
- 已新增版本兼容矩阵，明确 Python/Web/Godot/ROS2/hardware/vendor/browser 的支持组合、禁止组合和升级规则。
- 已新增监控与告警基线，定义 Web、workflow、hardware、vendor、ROS2、browser validation 的观测入口、P1/P2/P3 等级和升级动作。
- 已新增 SLA / SLO 视图，把 Web、workflow、operator history、hardware recovery、vendor data、ROS2 和 browser validation 转成可度量服务目标，同时保留非托管式 SLA 边界。
- 已新增 operator delivery checklist 自动化模板和 builder，把 system status、workflow、operator history、hardware diagnostics、vendor promotion、browser validation 和 optional ROS2 evidence 汇总成机器可读 checklist。
- 已新增 industrial live evidence archive report，把真实 `industrial_live_evidence` 字段、operator delivery checklist 与 external-mainline plan 汇总成 fail-closed 归档报告。
- 已新增客户现场真实设备 smoke 模板和报告 builder，把现场安全检查、真实 transport、telemetry、fault mapping、限幅命令、recovery plan 和 operator history evidence 收口成 fail-closed 报告。

### 剩余外部执行

- 仓内实现已闭合；真实客户现场仍需按 `docs/guides/CUSTOMER_SITE_REAL_DEVICE_SMOKE_20260427.md` 执行并归档真实设备 evidence。

### 建议落点

- `docs/guides/`
- `tools/`
- `agi_walker/ops/`

### 完成标准

- `operator_delivery_checklist.status=ready`
- `industrial_live_evidence_archive_report.status=ready`
- 真实硬件在交付范围内时，`customer_site_live_smoke_report.status=passed`
- 非研发角色能按文档完成演示、验收和基础排障

## 11. 路线 E：vendor 数据沉淀

### 目标

把现场真实 error code、恢复策略、阈值和 telemetry 字段沉淀回外置 vendor 数据，而不是继续停留在参考表。

### 当前已具备

- `deployment/hardware/imc22_reflex_fault_table.json`
- `deployment/hardware/imc22_reflex_recovery_policy.json`
- `deployment/hardware/imc22_fault_telemetry_fields.json`
- `deployment/hardware/imc22_vendor_fault_samples.template.json`
- `fault_telemetry_report.entries[].raw_error_value`
- `fault_telemetry_report.entries[].fault_class`
- `tools/build_vendor_fault_data_review.py`
- `tools/build_vendor_data_promotion_checklist.py`
- `tools/build_vendor_fault_sample_closeout.py`

### 本轮进展

- 已新增 vendor 数据审查 runner，用现场 telemetry 对照 fault table 与 recovery policy。
- 已新增 telemetry 字段对照表，并在审查报告里输出必填字段覆盖度。
- 已新增 vendor fault 样本归档模板，并支持 `--sample-archive-file` 纳入审查。
- 已新增 vendor data promotion checklist，强制晋升前绑定 review、样本归档、`data_version` 和 `change_log`。
- 已新增 vendor fault sample closeout，强制真实 raw error 样本、fault table 版本和 recovery policy 版本在 review/promotion 前先闭合。
- 已覆盖通过、fault class 不一致、必填字段缺失、policy 缺失四类非 live 回归。
- 已把审查命令接入硬件集成文档和 README。

### 剩余外部执行

- 仓内 sample closeout / review / promotion 已具备。
- 仍需真实现场 raw error code 样本。
- 仍需把真实 fault table 变更和真实 recovery policy 适用条件写入版本化数据。
- Route E 的最终判定依次以 `vendor_fault_sample_closeout.status=ready`、`vendor_fault_data_review.status=passed`、`vendor_data_promotion_checklist.status=ready` 为准。

### 建议落点

- `deployment/hardware/`
- `tests/fixtures/`
- `tools/run_hardware_transport_diagnostics.py`
- `tools/build_vendor_fault_data_review.py`
- `docs/hardware/HARDWARE_INTEGRATION_GUIDE.md`

### 完成标准

- `vendor_fault_sample_closeout.status=ready`
- `vendor_fault_data_review.status=passed`
- `vendor_data_promotion_checklist.status=ready`
- 每个新增 fault code 都有来源、分类、恢复策略、回归样本和可审计 raw-code 对照

## 12. 路线 F：浏览器手工验证

### 目标

补齐最近 Web UI 改动的真实浏览器验证，覆盖控制台、历史页、时间线和硬件恢复操作。

### 当前已具备

- 静态页面断言
- 后端路由测试
- non-live 回归
- `docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md`
- `deployment/web_browser_manual_validation.template.json`
- `tools/run_web_browser_playwright_smoke.py`
- `tools/build_web_browser_manual_validation_report.py`
- `tools/build_web_browser_validation_closeout.py`
- `tools/build_web_browser_validation_evidence_pack.py`

### 当前状态

- 仓内静态、模板、报告生成器、Playwright smoke、closeout 判定和 evidence pack 汇总已具备。
- 当前 canonical `web_browser_validation_evidence_pack.json` 仍为 `status=blocked`：`playwright_status=passed`，但 manual report、screenshots、exports、console summary 与 validation closeout 尚未归档。
- 客户现场或本地桌面浏览器必须按同一清单复核；现场复核结果应作为增量 evidence 归档，不应用占位值覆盖 blocker。

### 剩余外部执行

- 客户现场如要求复核，按浏览器清单重新执行桌面 Chromium 手工或 Playwright 验证
- 归档客户现场截图、导出文件和 console summary
- 重新生成客户现场范围内的 `web_browser_manual_validation_report.json`
- 重新生成客户现场范围内的 `web_browser_validation_closeout.json`
- 重新生成客户现场范围内的 `web_browser_validation_evidence_pack.json`
- instruction console 点击发送/恢复/清故障
- operator history 筛选、导出、replay
- timeline compare
- 响应式布局基本检查
- Route F 的最终判定以 `test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json.status=ready` 为准。

### 建议落点

- `tests/test_web_panel_aux_apis.py`
- 可选新增 Playwright smoke
- `docs/guides/WEB_PANEL_GUIDE.md`
- `docs/guides/WEB_BROWSER_MANUAL_VALIDATION_CHECKLIST_20260426.md`
- `tools/run_web_browser_playwright_smoke.py`
- `tools/build_web_browser_manual_validation_report.py`
- `tools/build_web_browser_validation_closeout.py`
- `tools/build_web_browser_validation_evidence_pack.py`

### 完成标准

- 浏览器中完成一次 instruction-set 发送、recovery plan、recover、clear-faults
- history / timeline 页面无阻塞性布局或交互问题
- `web_browser_manual_validation_report.json` 为 `status=passed`
- `web_browser_validation_closeout.json` 为 `status=passed`
- `web_browser_validation_evidence_pack.json` 为 `status=ready`

## 13. 推荐的下一步迭代节奏

从现在开始，不再按“新增仓内功能”排序，而按总 readiness blockers 排序。

先运行：

```bash
python tools/build_next_stage_readiness_report.py --output test_env/next_stage/next_stage_readiness_report.json
```

然后按 `blockers` 顺序推进：

### Iteration 1：真实硬件与 vendor 数据

- 执行真实硬件 diagnostics / telemetry / customer-site smoke
- 生成 `hardware_live_closeout_report.json`
- 填充真实 vendor raw error 样本
- 生成 `vendor_fault_sample_closeout.json`
- 运行 vendor review / promotion

### Iteration 2：浏览器真实验收

- 完成 Chromium 手工验证或 Playwright 验证
- 生成 manual report、closeout、evidence pack
- 目标：`web_browser_validation_evidence_pack.status=ready`

### Iteration 3：ROS2 Humble typed cutover

- 在目标 Humble 环境执行下游 typed cutover
- 禁用对应 JSON writer
- 归档 typed inventory、live smoke 和 rollback owner
- 目标：`ros2_typed_idl_cutover_report.status=ready`

### Iteration 4：交付归档总收口

- 重建 operator delivery checklist
- 重建 industrial live evidence archive
- 重建 next-stage readiness report
- 目标：`next_stage_readiness_report.status=ready`

## 14. 当前最需要注意的风险

- 如果不做真实硬件 live 验证，项目会长期停留在“高质量模拟/桥接”阶段
- 如果硬件权限策略不接组织级 IAM / RBAC，客户现场授权仍需靠受管 JSON 与 token claim 审计
- 如果下游不执行 typed IDL cutover，JSON 兼容面会继续拖慢生态复用
- 如果 vendor fault table 不沉淀真实样本，恢复策略会缺乏现场可信度
- 如果客户现场复核缺失，浏览器证据只能证明本地 Web 操作面闭环，不能替代现场验收签收

## 15. 建议结论

当前最优先的方向是：

- 先运行 `tools/build_next_stage_readiness_report.py`
- 再按报告中的 `blockers` 顺序补真实 evidence
- 当前最高优先级通常是 `hardware_live_closeout` 和 `vendor_fault_sample_closeout`

ROS2 typed cutover、浏览器验收和 industrial live evidence 可以并行推进，但最终都必须回到 `next_stage_readiness_report.status=ready`。
