# First Batch Candidate Files 2026-04-24

用于把 `MINIMAL_CLOSEOUT_REVIEW_CHECKLIST_20260424.md` 再进一步压缩成**第一批候选文件清单**。

这份清单的目标不是替你决定最终提交内容，而是给出一条更容易执行的第一轮人工审查顺序：

- 先看哪些文件
- 哪些文件建议一起判断
- 哪些文件默认放到第二批

## 第一组：先看 `release / readiness / worktree / preflight`

建议一起判断：

- `agi_walker/core/api/release_contracts.py`
- `agi_walker/core/api/release_control_plane.py`
- `agi_walker/core/api/release_ops_contracts.py`
- `agi_walker/core/api/security_posture_contracts.py`
- `agi_walker/ops/readiness.py`
- `agi_walker/ops/release_ops.py`
- `agi_walker/ops/worktree.py`
- `tools/check_release_readiness.py`
- `tools/run_worktree_release_blocker.py`
- `tools/build_worktree_cleanup_report.py`
- `tools/run_security_release_preflight.py`
- `tools/collect_release_evidence.py`
- `tools/run_clean_checkout_smoke.py`

原因：

- 这一组直接决定当前 gate / blocker / preflight / clean checkout 的主状态

## 第二组：再看 `external_mainline / extension_execution`

建议一起判断：

- `agi_walker/ops/external_mainline.py`
- `tools/build_external_mainline_execution_plan.py`
- `tools/build_external_mainline_input_checklist.py`
- `tools/build_external_mainline_inputs.py`
- `tools/run_external_mainline_execution_plan.py`
- `tools/build_extension_execution_actuals.py`
- `tools/build_extension_execution_evidence.py`
- `tools/build_extension_execution_instance.py`
- `tools/build_extension_execution_schedule.py`
- `tools/confirm_customer_external_bindings.py`
- `tools/build_customer_external_bindings_config.py`
- `tools/build_customer_external_bindings_confirmation_report.py`
- `tools/run_customer_external_bindings_closure.py`

原因：

- 这一组直接决定 `external_mainline`、`customer external bindings`、`extension execution` 三条主线

## 第三组：再看 `customer_acceptance / industrial_delivery / rehearsal`

建议一起判断：

- `agi_walker/ops/acceptance.py`
- `agi_walker/ops/industrial_delivery.py`
- `agi_walker/ops/promotion.py`
- `agi_walker/ops/rehearsal.py`
- `tools/build_customer_acceptance_bundle.py`
- `tools/build_industrial_delivery_rehearsal_report.py`
- `tools/build_industrial_promotion_checklist.py`
- `tools/check_industrial_release_readiness.py`
- `tools/build_sbom_artifact.py`
- `tools/build_security_posture_report.py`
- `tools/build_vulnerability_exception_report.py`
- `tools/build_vulnerability_exception_review_report.py`
- `tools/build_vulnerability_remediation_report.py`
- `tools/run_backup_restore_rehearsal.py`
- `tools/write_pytest_evidence_report.py`
- `tools/write_vulnerability_scan_report.py`

原因：

- 这一组承接 customer acceptance、industrial rehearsal、promotion checklist 和 security posture 汇总面

## 第四组：跟实现一起看的测试

建议第一批只跟这些测试：

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
- `tests/test_vulnerability_exception_review_report.py`
- `tests/test_worktree_cleanup_report.py`
- `tests/test_worktree_ops.py`
- `tests/test_worktree_release_blocker_report.py`
- `tests/test_active_path_references.py`

原因：

- 这些测试和前面三组实现强绑定

## 第五组：第一批里建议保留的输入 / 文档

建议一起判断：

- `README.md`
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
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
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md`
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md`
- `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md`
- `docs/guides/EXTERNAL_MAINLINE_INPUTS_FIELD_MAP_20260424.md`
- `docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md`
- `docs/guides/PLACEHOLDER_REPLACEMENT_CHECKLIST_20260424.md`

原因：

- 这些文件直接支撑当前 closeout 主线的输入面和执行说明

## 默认放第二批

建议先不要混入第一批：

- `web_panel/*`
- `.github/workflows/ci.yml`
- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/run_release_rehearsal.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`
- distributed / MCP / Portal / workflow 扩展测试

## 一句话使用方法

如果你现在就要开始人工审查，最简单的方式是：

1. 先过第一组
2. 再过第二组
3. 再过第三组
4. 只带上第四组测试
5. 最后补第五组必要输入和文档

