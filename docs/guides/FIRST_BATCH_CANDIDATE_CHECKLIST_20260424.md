# First Batch Candidate Checklist 2026-04-24

用于把 `FIRST_BATCH_CANDIDATE_FILES_20260424.md` 进一步压成**可勾选表**。

建议你在人工审查时，对每个文件只做一个决定：

- `保留`
- `暂缓`
- `第二批`

---

## 1. `release / readiness / worktree / preflight`

| 文件 | 推荐批次 | 建议动作 |
|---|---|---|
| `agi_walker/core/api/release_contracts.py` | 第一批 | 保留 |
| `agi_walker/core/api/release_control_plane.py` | 第一批 | 保留 |
| `agi_walker/core/api/release_ops_contracts.py` | 第一批 | 保留 |
| `agi_walker/core/api/security_posture_contracts.py` | 第一批 | 保留 |
| `agi_walker/ops/readiness.py` | 第一批 | 保留 |
| `agi_walker/ops/release_ops.py` | 第一批 | 保留 |
| `agi_walker/ops/worktree.py` | 第一批 | 保留 |
| `tools/check_release_readiness.py` | 第一批 | 保留 |
| `tools/run_worktree_release_blocker.py` | 第一批 | 保留 |
| `tools/build_worktree_cleanup_report.py` | 第一批 | 保留 |
| `tools/run_security_release_preflight.py` | 第一批 | 保留 |
| `tools/collect_release_evidence.py` | 第一批 | 保留 |
| `tools/run_clean_checkout_smoke.py` | 第一批 | 保留 |

## 2. `external_mainline / extension_execution`

| 文件 | 推荐批次 | 建议动作 |
|---|---|---|
| `agi_walker/ops/external_mainline.py` | 第一批 | 保留 |
| `tools/build_external_mainline_execution_plan.py` | 第一批 | 保留 |
| `tools/build_external_mainline_input_checklist.py` | 第一批 | 保留 |
| `tools/build_external_mainline_inputs.py` | 第一批 | 保留 |
| `tools/run_external_mainline_execution_plan.py` | 第一批 | 保留 |
| `tools/build_extension_execution_actuals.py` | 第一批 | 保留 |
| `tools/build_extension_execution_evidence.py` | 第一批 | 保留 |
| `tools/build_extension_execution_instance.py` | 第一批 | 保留 |
| `tools/build_extension_execution_schedule.py` | 第一批 | 保留 |
| `tools/confirm_customer_external_bindings.py` | 第一批 | 保留 |
| `tools/build_customer_external_bindings_config.py` | 第一批 | 保留 |
| `tools/build_customer_external_bindings_confirmation_report.py` | 第一批 | 保留 |
| `tools/run_customer_external_bindings_closure.py` | 第一批 | 保留 |

## 3. `customer_acceptance / industrial_delivery / rehearsal`

| 文件 | 推荐批次 | 建议动作 |
|---|---|---|
| `agi_walker/ops/acceptance.py` | 第一批 | 保留 |
| `agi_walker/ops/industrial_delivery.py` | 第一批 | 保留 |
| `agi_walker/ops/promotion.py` | 第一批 | 保留 |
| `agi_walker/ops/rehearsal.py` | 第一批 | 保留 |
| `tools/build_customer_acceptance_bundle.py` | 第一批 | 保留 |
| `tools/build_industrial_delivery_rehearsal_report.py` | 第一批 | 保留 |
| `tools/build_industrial_promotion_checklist.py` | 第一批 | 保留 |
| `tools/check_industrial_release_readiness.py` | 第一批 | 保留 |
| `tools/build_sbom_artifact.py` | 第一批 | 保留 |
| `tools/build_security_posture_report.py` | 第一批 | 保留 |
| `tools/build_vulnerability_exception_report.py` | 第一批 | 保留 |
| `tools/build_vulnerability_exception_review_report.py` | 第一批 | 保留 |
| `tools/build_vulnerability_remediation_report.py` | 第一批 | 保留 |
| `tools/run_backup_restore_rehearsal.py` | 第一批 | 保留 |
| `tools/write_pytest_evidence_report.py` | 第一批 | 保留 |
| `tools/write_vulnerability_scan_report.py` | 第一批 | 保留 |

## 4. `must_keep tests`

| 文件 | 推荐批次 | 建议动作 |
|---|---|---|
| `tests/test_clean_checkout_smoke.py` | 第一批 | 保留 |
| `tests/test_customer_acceptance_bundle.py` | 第一批 | 保留 |
| `tests/test_customer_external_bindings_config_builder.py` | 第一批 | 保留 |
| `tests/test_external_mainline_execution_plan.py` | 第一批 | 保留 |
| `tests/test_industrial_delivery_rehearsal_report.py` | 第一批 | 保留 |
| `tests/test_industrial_promotion_checklist.py` | 第一批 | 保留 |
| `tests/test_industrial_release_readiness.py` | 第一批 | 保留 |
| `tests/test_readiness_ops.py` | 第一批 | 保留 |
| `tests/test_release_ops.py` | 第一批 | 保留 |
| `tests/test_release_readiness.py` | 第一批 | 保留 |
| `tests/test_release_rehearsal.py` | 第一批 | 保留 |
| `tests/test_run_smoke_tests.py` | 第一批 | 保留 |
| `tests/test_security_release_preflight.py` | 第一批 | 保留 |
| `tests/test_vulnerability_exception_review_report.py` | 第一批 | 保留 |
| `tests/test_worktree_cleanup_report.py` | 第一批 | 保留 |
| `tests/test_worktree_ops.py` | 第一批 | 保留 |
| `tests/test_worktree_release_blocker_report.py` | 第一批 | 保留 |
| `tests/test_active_path_references.py` | 第一批 | 保留 |

## 5. `必要输入 / 文档`

| 文件 | 推荐批次 | 建议动作 |
|---|---|---|
| `README.md` | 第一批 | 保留 |
| `PRODUCTION_DEPLOYMENT_RUNBOOK.md` | 第一批 | 保留 |
| `deployment/customer_delivery.external_bindings.customer.json` | 第一批 | 保留 |
| `deployment/customer_delivery.external_bindings.customer.overrides.json` | 第一批 | 保留 |
| `deployment/customer_delivery.external_bindings.example.json` | 第一批 | 保留 |
| `deployment/customer_delivery.external_bindings.json` | 第一批 | 保留 |
| `deployment/customer_delivery.external_bindings.overrides.example.json` | 第一批 | 保留 |
| `deployment/customer_delivery.external_bindings.rehearsal.json` | 第一批 | 保留 |
| `deployment/external_mainline.inputs.completed_example.json` | 第一批 | 保留 |
| `deployment/external_mainline.inputs.customer_draft.json` | 第一批 | 保留 |
| `deployment/external_mainline.inputs.example.json` | 第一批 | 保留 |
| `deployment/external_mainline.inputs.json` | 第一批 | 保留 |
| `deployment/industrial_live_evidence.customer.template.json` | 第一批 | 保留 |
| `deployment/security/vulnerability_exceptions.input.json` | 第一批 | 保留 |
| `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md` | 第一批 | 保留 |
| `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md` | 第一批 | 保留 |
| `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md` | 第一批 | 保留 |
| `docs/guides/EXTERNAL_MAINLINE_INPUTS_FIELD_MAP_20260424.md` | 第一批 | 保留 |
| `docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md` | 第一批 | 保留 |
| `docs/guides/PLACEHOLDER_REPLACEMENT_CHECKLIST_20260424.md` | 第一批 | 保留 |

## 默认第二批

| 文件/范围 | 推荐批次 | 建议动作 |
|---|---|---|
| `web_panel/*` | 第二批 | 第二批 |
| `.github/workflows/ci.yml` | 第二批 | 第二批 |
| `agi_walker/core/api/mcp_tools.py` | 第二批 | 第二批 |
| `agi_walker/mcp/server.py` | 第二批 | 第二批 |
| `tools/build_release_artifact.py` | 第二批 | 第二批 |
| `tools/build_stable_promotion_checklist.py` | 第二批 | 第二批 |
| `tools/run_release_rehearsal.py` | 第二批 | 第二批 |
| `tools/run_container_vulnerability_scan.py` | 第二批 | 第二批 |
| `tools/run_python_vulnerability_scan.py` | 第二批 | 第二批 |
| distributed / MCP / Portal / workflow 扩展测试 | 第二批 | 第二批 |

