from pathlib import Path


PLAN = Path("docs/guides/NEXT_STAGE_EXECUTION_PLAN_20260426.md")
CURRENT_STATUS = Path("docs/CURRENT_STATUS.md")


def test_next_stage_plan_uses_evidence_driven_iteration_order() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert "## 1. 总证据矩阵" in content
    assert "不再按“新增仓内功能”排序，而按总 readiness blockers 排序" in content
    assert "python tools/build_next_stage_readiness_report.py" in content
    assert "hardware_live_closeout_report.status=ready" in content
    assert "web_browser_validation_evidence_pack.status=ready" in content
    assert "ros2_typed_idl_cutover_report.status=ready" in content
    assert "operator_delivery_checklist.status=ready" in content
    assert "industrial_live_evidence_archive_report.status=ready" in content
    assert "vendor_fault_sample_closeout.status=ready" in content
    assert "vendor_fault_data_review.status=passed" in content
    assert "vendor_data_promotion_checklist.status=ready" in content
    assert "next_stage_readiness_report.status=ready" in content


def test_next_stage_plan_has_evidence_matrix_for_all_routes() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert "| A 真实硬件联调 | `test_env/hardware_live/hardware_live_closeout_report.json` | `status=ready` |" in content
    assert "| B Web 硬件操作面 | `deployment/web_hardware_role_policy.json` 与 operator history | 只读受管策略可审计 |" in content
    assert "| C ROS2 标准化 | `test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json` | `status=ready` |" in content
    assert "| D 交付产品化 | `test_env/operator_delivery/operator_delivery_checklist.json` | `status=ready` |" in content
    assert "| D live evidence | `test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json` | `status=ready` |" in content
    assert "| E vendor 数据 | `test_env/hardware_live/vendor_fault_sample_closeout.json` | `status=ready` |" in content
    assert "| E vendor 晋升 | `test_env/hardware_live/vendor_data_promotion_checklist.json` | `status=ready` |" in content
    assert "| F 浏览器验收 | `test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json` | `status=ready` |" in content


def test_next_stage_plan_has_command_index_for_closeout_tools() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert "## 2. 执行命令索引" in content
    assert "优先按 `action_plan` 执行" in content
    assert "python tools/build_hardware_live_closeout_report.py --output test_env/hardware_live/hardware_live_closeout_report.json" in content
    assert "python tools/build_vendor_fault_sample_closeout.py --output test_env/hardware_live/vendor_fault_sample_closeout.json" in content
    assert "python tools/build_vendor_fault_data_review.py --telemetry-report test_env/hardware_live/hardware_fault_telemetry_report.json --output test_env/hardware_live/vendor_fault_data_review.json" in content
    assert "python tools/build_vendor_data_promotion_checklist.py --sample-archive-file deployment/hardware/imc22_vendor_fault_samples.template.json --vendor-review-file test_env/hardware_live/vendor_fault_data_review.json --output test_env/hardware_live/vendor_data_promotion_checklist.json" in content
    assert "python tools/build_ros2_typed_idl_cutover_report.py --output test_env/ros2_typed_idl_cutover/ros2_typed_idl_cutover_report.json" in content
    assert "python tools/build_ros2_typed_inventory.py --output test_env/ros2_typed_idl_cutover/typed_inventory.json" in content
    assert "python tools/build_operator_delivery_checklist.py --output test_env/operator_delivery/operator_delivery_checklist.json" in content
    assert "python tools/build_customer_site_live_smoke_report.py --output test_env/customer_site_live_smoke/customer_site_live_smoke_report.json" in content
    assert (
        "python tools/build_industrial_live_evidence_archive_report.py "
        "--customer-site-smoke test_env/customer_site_live_smoke/customer_site_live_smoke_report.json "
        "--require-customer-site-smoke "
        "--output test_env/industrial_live_evidence/industrial_live_evidence_archive_report.json"
        in content
    )
    assert "python tools/build_web_browser_manual_validation_report.py --output test_env/web_browser_manual_validation/web_browser_manual_validation_report.json" in content
    assert "python tools/build_web_browser_validation_closeout.py --output test_env/web_browser_manual_validation/web_browser_validation_closeout.json" in content
    assert "python tools/build_web_browser_validation_evidence_pack.py --output test_env/web_browser_manual_validation/web_browser_validation_evidence_pack.json" in content
    assert "不用占位值把 `blocked` 改成 `ready`" in content


def test_next_stage_plan_has_input_file_checklist() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert "## 3. 输入文件清单" in content
    assert "| A 真实硬件联调 | `deployment/hardware/imc22_live_transport.template.json` | 真实 transport 参数基准 |" in content
    assert "| C ROS2 标准化 | `deployment/ros2_typed_idl_cutover.template.json` | typed cutover 证据模板 |" in content
    assert "| D 交付产品化 | `deployment/operator_delivery_checklist.template.json` | operator checklist 输入模板 |" in content
    assert "| D live evidence | `deployment/external_mainline.inputs.json` | industrial live evidence 来源 |" in content
    assert "| E vendor 数据 | `deployment/hardware/imc22_vendor_fault_samples.template.json` | raw fault 样本归档模板 |" in content
    assert "| F 浏览器验收 | `deployment/web_browser_manual_validation.template.json` | 手工浏览器验证输入模板 |" in content
    assert "| B Web 权限 | `deployment/web_hardware_role_policy.json` | 受管硬件恢复角色策略 |" in content


def test_next_stage_plan_has_blocker_resolution_table() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert "## 4. blocker 解释表" in content
    assert "| `hardware_live_closeout_blocked` | 真实硬件 closeout 尚未 ready |" in content
    assert "| `vendor_fault_sample_closeout_blocked` | 缺真实 raw fault 样本或版本绑定 |" in content
    assert "| `ros2_typed_idl_cutover_blocked` | 目标 Humble typed cutover 证据不足 |" in content
    assert "| `industrial_live_evidence_archive_blocked` | industrial live evidence、external-mainline 或 strict customer-site smoke 缺失 |" in content
    assert "| `manual_report_missing` | 浏览器手工验证报告不存在 |" in content
    assert "| `screenshots_missing` | 浏览器验证缺截图证据 |" in content
    assert "| `validation_closeout_blocked` | manual report / closeout / evidence pack 未全通过 |" in content
    assert "没有真实证据时保留 blocker 是正确状态" in content
    assert "`web_browser_evidence_pack` 当前按 canonical artifact 仍为 `blocked`" in content
    assert "manual report、screenshots、exports、console summary 与 validation closeout 仍需真实浏览器验收输入后重建" in content
    assert "`operator_delivery_checklist` 当前已无 warning，仅剩必需 blocker：`vendor_data_promotion`" in content
    assert "`vendor_data_promotion` 不应以模板或占位样本强制置 `ready`" in content
    assert "`vendor_fault_data_review` 当前缺 `test_env/hardware_live/hardware_fault_telemetry_report.json`" in content
    assert "`vendor_data_promotion_checklist` 当前应保持 `blocked`，明确卡在 `change_request` 占位值和 `vendor_review` 未通过" in content
    assert "`industrial_live_evidence_archive_report` 当前已支持 strict customer-site smoke 绑定" in content
    assert "`--require-customer-site-smoke` 会把缺失或未通过的 `customer_site_smoke` 升级为 blocker" in content
    assert "`next_stage_readiness_report` 当前会输出 `action_plan`、`blocker_details`、`generated_at`、`git` 和 `validation_errors`" in content
    assert "`next_stage_readiness_report.generated_at`" in content
    assert "`next_stage_readiness_report.git`" in content
    assert "`validation_errors` 固定校验 summary 计数" in content
    assert "当前 typed surface blocker 已收敛，仅保留真实 cutover 输入" in content
    assert "剩余 action 应全部归类为 `external_input`" in content


def test_next_stage_plan_has_live_environment_preflight() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert "## 5. 真实环境执行前检查" in content
    assert "| 硬件连接 | IMC-22 / CAN / 串口设备、bus、baud、node id 与 `imc22_live_transport` 输入一致 |" in content
    assert "| 操作权限 | 当前 operator 具备 `hardware_recovery_operator` 或管理员 token claim |" in content
    assert "| 安全边界 | 限幅、watchdog、急停、回滚入口已由现场负责人确认 |" in content
    assert "| ROS2 环境 | 目标节点为 ROS2 Humble，typed IDL package 可构建且 source 后可见 |" in content
    assert "| 浏览器环境 | Chromium 或等价浏览器可打开 Web console，DevTools console 可导出 |" in content
    assert "| industrial live 输入 | `industrial_live_evidence.*` 字段已由现场负责人确认，且 `customer_site_live_smoke_report.status=passed` |" in content
    assert "| vendor 数据 | raw error 样本包含设备、时间戳、raw code、fault class、恢复结果 |" in content
    assert "不能把 live evidence 标成 ready" in content


def test_next_stage_plan_marks_remaining_work_as_external_execution() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert content.count("### 剩余外部执行") >= 6
    assert "### 还需实现" not in content
    assert "仓内 closeout 汇总已具备" in content
    assert "仓内 typed IDL cutover closeout 已具备" in content
    assert "仓内 sample closeout / review / promotion 已具备" in content
    assert "若需要在线增删角色，后续应接入正式组织级 IAM / RBAC" in content
    assert "当前 canonical `web_browser_validation_evidence_pack.json` 仍为 `status=blocked`" in content
    assert "`playwright_status=passed`" in content


def test_next_stage_plan_no_longer_claims_feature_iteration_as_next_step() -> None:
    content = PLAN.read_text(encoding="utf-8")

    assert "### Iteration 2\n\n- Web 硬件恢复动作时间线" not in content
    assert "### Iteration 3\n\n- ROS2 标准 message / service 定义" not in content
    assert "先做 `真实硬件联调`" not in content
    assert "如果 Web 恢复操作没有确认和权限约束" not in content
    assert "如果 ROS2 长期保持 JSON string bridge 形态" not in content


def test_current_status_separates_release_ready_from_next_stage_blocked() -> None:
    content = CURRENT_STATUS.read_text(encoding="utf-8")

    assert "这不等同于下一阶段真实客户现场 evidence 已完成" in content
    assert "next-stage readiness 当前仍为 `blocked`" in content
    assert "`external_input_action_count=8`、`code_or_config_action_count=0`" in content
