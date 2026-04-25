# Minimal Closeout Submission Set 2026-04-24

用于从当前脏 worktree 中提炼出一批**最小但可闭环**的提交面，帮助优先解除：

- `stable_worktree_release_blocker`
- 并保持当前已经推进到的 closeout 主线结果

这份清单基于三份上游分诊文档收敛而来：

- `WORKTREE_REVIEW_GROUPS_20260424.md`
- `RELEASE_AND_OPS_TRIAGE_20260424.md`
- `TESTS_TRIAGE_20260424.md`

## 目标

先形成一批“最小可闭环提交面”，覆盖：

- `external_mainline`
- `extension_execution`
- `clean_checkout_smoke`
- `security_preflight`
- `readiness`
- `customer_acceptance / industrial_rehearsal / promotion checklist`

而不是一次性把全部 Portal / MCP / distributed / CI 改动都混进来。

## A. `release_and_ops` 最小保留集

### 核心 contracts / orchestration

- `agi_walker/core/api/release_contracts.py`
- `agi_walker/core/api/release_control_plane.py`
- `agi_walker/core/api/release_ops_contracts.py`
- `agi_walker/core/api/security_posture_contracts.py`
- `agi_walker/ops/acceptance.py`
- `agi_walker/ops/external_mainline.py`
- `agi_walker/ops/industrial_delivery.py`
- `agi_walker/ops/promotion.py`
- `agi_walker/ops/readiness.py`
- `agi_walker/ops/rehearsal.py`
- `agi_walker/ops/release_ops.py`
- `agi_walker/ops/worktree.py`

### 核心 builders / runners

- `tools/build_customer_acceptance_bundle.py`
- `tools/build_customer_external_bindings_config.py`
- `tools/build_customer_external_bindings_confirmation_report.py`
- `tools/build_extension_execution_actuals.py`
- `tools/build_extension_execution_evidence.py`
- `tools/build_extension_execution_instance.py`
- `tools/build_extension_execution_schedule.py`
- `tools/build_external_mainline_execution_plan.py`
- `tools/build_external_mainline_input_checklist.py`
- `tools/build_external_mainline_inputs.py`
- `tools/build_industrial_delivery_rehearsal_report.py`
- `tools/build_industrial_promotion_checklist.py`
- `tools/build_sbom_artifact.py`
- `tools/build_security_posture_report.py`
- `tools/build_vulnerability_exception_report.py`
- `tools/build_vulnerability_exception_review_report.py`
- `tools/build_vulnerability_remediation_report.py`
- `tools/check_industrial_release_readiness.py`
- `tools/check_release_readiness.py`
- `tools/collect_release_evidence.py`
- `tools/confirm_customer_external_bindings.py`
- `tools/run_backup_restore_rehearsal.py`
- `tools/run_clean_checkout_smoke.py`
- `tools/run_customer_external_bindings_closure.py`
- `tools/run_external_mainline_execution_plan.py`
- `tools/run_release_ops.py`
- `tools/run_security_release_preflight.py`
- `tools/run_worktree_release_blocker.py`
- `tools/write_pytest_evidence_report.py`
- `tools/write_vulnerability_scan_report.py`

## B. `tests` 最小保留集

- `tests/test_active_path_references.py`
- `tests/test_clean_checkout_smoke.py`
- `tests/test_customer_acceptance_bundle.py`
- `tests/test_customer_external_bindings_config_builder.py`
- `tests/test_external_mainline_execution_plan.py`
- `tests/test_industrial_delivery_rehearsal_report.py`
- `tests/test_industrial_promotion_checklist.py`
- `tests/test_industrial_release_readiness.py`
- `tests/test_readiness_ops.py`
- `tests/test_release_evidence_reports.py`
- `tests/test_release_ops.py`
- `tests/test_release_readiness.py`
- `tests/test_release_rehearsal.py`
- `tests/test_run_smoke_tests.py`
- `tests/test_security_posture_reports.py`
- `tests/test_security_release_preflight.py`
- `tests/test_stable_promotion_checklist.py`
- `tests/test_vulnerability_exception_review_report.py`
- `tests/test_worktree_cleanup_report.py`
- `tests/test_worktree_ops.py`
- `tests/test_worktree_release_blocker_report.py`

## C. 必要 `delivery_and_docs` 最小保留集

### 必要受管输入 / 模板

- `deployment/customer_delivery.external_bindings.customer.json`
- `deployment/customer_delivery.external_bindings.customer.overrides.json`
- `deployment/customer_delivery.external_bindings.example.json`
- `deployment/customer_delivery.external_bindings.json`
- `deployment/customer_delivery.external_bindings.overrides.example.json`
- `deployment/customer_delivery.external_bindings.rehearsal.json`
- `deployment/external_mainline.inputs.completed_example.json`
- `deployment/external_mainline.inputs.customer_draft.json`
- `deployment/external_mainline.inputs.example.json`
- `deployment/external_mainline.inputs.json`
- `deployment/industrial_live_evidence.customer.template.json`
- `deployment/security/vulnerability_exceptions.input.json`

### 必要 runbook / 指南

- `README.md`
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
- `docs/guides/RELEASE_GUIDE.md`
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md`
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md`
- `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md`
- `docs/guides/EXTERNAL_MAINLINE_INPUTS_FIELD_MAP_20260424.md`
- `docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md`
- `docs/guides/PLACEHOLDER_REPLACEMENT_CHECKLIST_20260424.md`
- `docs/guides/PLACEHOLDER_TICKET_READY_TODO_20260424.md`
- `docs/guides/WORKTREE_REVIEW_GROUPS_20260424.md`
- `docs/guides/RELEASE_AND_OPS_TRIAGE_20260424.md`
- `docs/guides/TESTS_TRIAGE_20260424.md`

## D. 当前不必优先并入这批的内容

### 可后置确认

- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/build_worktree_cleanup_report.py`
- `tools/run_release_rehearsal.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`

### 更适合独立判断

- `web_panel/*`
- `.github/workflows/ci.yml`
- distributed / MCP / Portal 扩展测试
- 与当前 closeout 主线弱耦合的说明性文档

## 推荐使用方式

建议你先用这份最小集做第一轮人工审查：

1. 先确认 `A + B`
2. 再确认 `C`
3. 暂时不要让 `web_panel / ci / mcp 扩展 / distributed 扩展` 混进第一批

## 最重要的结论

如果目标是尽快把当前仓库推进到“**worktree 可人工收口**”而不是“**所有同时期改动一次性定稿**”，这份清单就是最小优先批次。

