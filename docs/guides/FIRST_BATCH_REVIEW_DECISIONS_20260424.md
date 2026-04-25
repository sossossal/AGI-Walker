# First Batch Review Decisions 2026-04-24

用于把 `docs/guides/FIRST_BATCH_CANDIDATE_CHECKLIST_20260424.md` 从“候选表”推进到“推荐决定表”。

这份文档只给出**第一批默认建议**，不直接替你执行 `git add` / `git restore` / 删除文件。这样可以避免把真实在做的工作误清掉。

---

## 总体结论

- 第一批建议**保留**：release / readiness / worktree / preflight 主线、external-mainline / extension-execution 主线、customer / industrial closeout 主线、与这些实现直接绑定的测试、以及必要输入与 closeout 文档。
- 第一批建议**暂缓单独判断**：`deployment/security/vulnerability_exceptions.input.json`，因为它还需要真实 upstream fix / scanner 结果，不适合今天在没有外部输入时硬定稿。
- 第一批建议**放第二批**：`web_panel/*`、`.github/workflows/ci.yml`、MCP / Portal 扩展、与第二批扩展功能强绑定的工具和测试。

---

## 推荐立即保留

### `release / readiness / worktree / preflight`

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

### `external_mainline / extension_execution`

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

### `customer / industrial / rehearsal`

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

### `must_keep tests`

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

### `必要输入 / 文档`

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
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md`
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md`
- `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md`
- `docs/guides/EXTERNAL_MAINLINE_INPUTS_FIELD_MAP_20260424.md`
- `docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md`
- `docs/guides/PLACEHOLDER_REPLACEMENT_CHECKLIST_20260424.md`

---

## 推荐暂缓单独判断

- `deployment/security/vulnerability_exceptions.input.json`

原因：

- 它属于第一批 closeout 主线的一部分；
- 但它当前仍依赖真实的 upstream fix 版本或重算后的 scanner 结果；
- 所以建议先保留文件位点，再等安全输入补齐后单独确认内容。

---

## 推荐放第二批

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

---

## 人工勾选建议

按下面顺序人工处理会最稳：

1. 先把“推荐立即保留”全部视为第一批；
2. 把 `deployment/security/vulnerability_exceptions.input.json` 标成“暂缓单独判断”；
3. 把“推荐放第二批”统一归入第二批；
4. 处理完后再重跑 `docs/guides/FIRST_BATCH_VERIFICATION_SEQUENCE_20260424.md` 里的命令链。
