# Minimal Closeout Review Checklist 2026-04-24

用于把 `MINIMAL_CLOSEOUT_SUBMISSION_SET_20260424.md` 进一步收成一页**可执行的人工审查顺序清单**。

目标：

- 降低 `stable_worktree_release_blocker` 的人工处理复杂度
- 先锁定当前 release / closeout 主线
- 暂缓不影响当前闭环的扩展面

## 使用原则

- 先看“直接影响 gate / evidence”的文件
- 再看“跟实现绑定的测试”
- 再看“必要输入和 runbook”
- 不要一开始就把 `web_panel / ci / mcp 扩展 / distributed 扩展` 混进第一轮

## 第 1 步：先锁定 `release_and_ops` 核心主线

优先审查：

- `agi_walker/core/api/release_contracts.py`
- `agi_walker/core/api/release_control_plane.py`
- `agi_walker/core/api/release_ops_contracts.py`
- `agi_walker/core/api/security_posture_contracts.py`
- `agi_walker/ops/`
- `tools/build_extension_execution_actuals.py`
- `tools/build_external_mainline_execution_plan.py`
- `tools/build_external_mainline_input_checklist.py`
- `tools/check_release_readiness.py`
- `tools/collect_release_evidence.py`
- `tools/run_clean_checkout_smoke.py`
- `tools/run_security_release_preflight.py`
- `tools/run_worktree_release_blocker.py`

本步判断问题：

- 这些文件是不是同一条 release / closeout 主线
- 有没有明显与当前收口目标无关的实验性内容
- 有没有你明确不想在第一批提交里的改动

完成标准：

- 这批文件你已经能回答“保留 / 暂缓 / 剥离”

## 第 2 步：再锁定 `customer / industrial / extension execution`

优先审查：

- `tools/build_customer_acceptance_bundle.py`
- `tools/build_customer_external_bindings_config.py`
- `tools/build_customer_external_bindings_confirmation_report.py`
- `tools/build_extension_execution_evidence.py`
- `tools/build_extension_execution_instance.py`
- `tools/build_extension_execution_schedule.py`
- `tools/build_industrial_delivery_rehearsal_report.py`
- `tools/build_industrial_promotion_checklist.py`
- `tools/confirm_customer_external_bindings.py`
- `tools/run_customer_external_bindings_closure.py`
- `tools/run_external_mainline_execution_plan.py`

本步判断问题：

- 这些是否都服务于当前 customer / industrial closeout 主线
- 是否需要和第 1 步同批落地

完成标准：

- `external_mainline + extension_execution + customer_acceptance + industrial_rehearsal`
 这一条线的实现面已经锁定

## 第 3 步：只跟进 `must_keep tests`

优先审查：

- `tests/test_clean_checkout_smoke.py`
- `tests/test_customer_acceptance_bundle.py`
- `tests/test_customer_external_bindings_config_builder.py`
- `tests/test_external_mainline_execution_plan.py`
- `tests/test_industrial_delivery_rehearsal_report.py`
- `tests/test_industrial_promotion_checklist.py`
- `tests/test_industrial_release_readiness.py`
- `tests/test_readiness_ops.py`
- `tests/test_release_ops.py`
- `tests/test_release_readiness.py`
- `tests/test_release_rehearsal.py`
- `tests/test_run_smoke_tests.py`
- `tests/test_security_release_preflight.py`
- `tests/test_worktree_cleanup_report.py`
- `tests/test_worktree_ops.py`
- `tests/test_worktree_release_blocker_report.py`

本步判断问题：

- 这些测试是否直接跟你在第 1 / 第 2 步保留的实现绑定
- 有没有测试已经脱离实现、可以暂缓

完成标准：

- 第一批提交至少带上“实现必须跟着走”的测试

## 第 4 步：确认必要输入 / 文档

优先审查：

- `README.md`
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
- `deployment/customer_delivery.external_bindings.*`
- `deployment/external_mainline.inputs*.json`
- `deployment/industrial_live_evidence.customer.template.json`
- `deployment/security/vulnerability_exceptions.input.json`
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md`
- `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md`
- `docs/guides/EXTERNAL_MAINLINE_INPUTS_FIELD_MAP_20260424.md`
- `docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md`
- `docs/guides/PLACEHOLDER_REPLACEMENT_CHECKLIST_20260424.md`

本步判断问题：

- 这些文件是不是当前 closeout 主线的必要输入/说明
- 有没有只是补充说明、可以放到第二批的文档

完成标准：

- 第一批提交至少带上“实现已依赖”的输入模板和操作说明

## 第 5 步：显式暂缓扩展面

建议默认暂缓到第二批再确认：

- `web_panel/*`
- `.github/workflows/ci.yml`
- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`
- distributed / MCP / Portal / workflow 扩展测试

这样做的目的：

- 不让第一批提交被额外范围拖大
- 先把当前 closeout 主线锁住

## 审查完成后的判定

如果第 1～4 步已经完成，而且第 5 步的扩展面被显式隔离，那么你就已经得到了一批：

- 范围相对最小
- 与当前 closeout 主线强绑定
- 更适合先解除 `stable_worktree_release_blocker` 人工不确定性

的第一批提交候选。

