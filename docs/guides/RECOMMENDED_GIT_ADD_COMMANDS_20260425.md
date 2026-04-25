# Recommended Git Add Commands · 2026-04-25

## 使用方式

这份文档把 `docs/guides/RECOMMENDED_COMMIT_ORDER_20260425.md` 里的批次，进一步压成可直接执行的 `git add` 草稿。

建议先做两件事：

1. 先确认这 3 个混合路径的最终内容：
   - `README.md`
   - `tests/run_smoke_tests.py`
   - `tests/test_run_smoke_tests.py`
2. 确认后，先执行一次：

```powershell
git add README.md tests/run_smoke_tests.py tests/test_run_smoke_tests.py
```

## Batch 1 · Release Ops 主线

```powershell
git add `
  agi_walker/core/api/release_contracts.py `
  agi_walker/core/api/release_control_plane.py `
  agi_walker/core/api/release_ops_contracts.py `
  agi_walker/core/api/security_posture_contracts.py `
  agi_walker/ops/__init__.py `
  agi_walker/ops/acceptance.py `
  agi_walker/ops/external_mainline.py `
  agi_walker/ops/industrial_delivery.py `
  agi_walker/ops/promotion.py `
  agi_walker/ops/readiness.py `
  agi_walker/ops/rehearsal.py `
  agi_walker/ops/release_ops.py `
  agi_walker/ops/worktree.py `
  tools/build_customer_acceptance_bundle.py `
  tools/build_extension_execution_actuals.py `
  tools/build_external_mainline_execution_plan.py `
  tools/build_external_mainline_input_checklist.py `
  tools/build_industrial_promotion_checklist.py `
  tools/build_release_artifact.py `
  tools/build_stable_promotion_checklist.py `
  tools/build_worktree_cleanup_report.py `
  tools/check_industrial_release_readiness.py `
  tools/check_release_readiness.py `
  tools/collect_release_evidence.py `
  tools/run_release_ops.py `
  tools/run_release_rehearsal.py `
  tools/run_security_release_preflight.py `
  tools/run_worktree_release_blocker.py `
  tests/test_acceptance_ops.py `
  tests/test_industrial_delivery_ops.py `
  tests/test_promotion_ops.py `
  tests/test_readiness_ops.py `
  tests/test_rehearsal_ops.py `
  tests/test_release_contracts.py `
  tests/test_release_ops.py `
  tests/test_release_readiness.py `
  tests/test_release_rehearsal.py `
  tests/test_worktree_cleanup_report.py `
  tests/test_worktree_ops.py `
  tests/test_worktree_release_blocker_report.py
```

## Batch 2 · External Mainline / Customer Bindings

```powershell
git add `
  deployment/customer_delivery.external_bindings.customer.json `
  deployment/customer_delivery.external_bindings.customer.overrides.json `
  deployment/customer_delivery.external_bindings.example.json `
  deployment/customer_delivery.external_bindings.json `
  deployment/customer_delivery.external_bindings.overrides.example.json `
  deployment/customer_delivery.external_bindings.rehearsal.json `
  deployment/external_mainline.inputs.completed_example.json `
  deployment/external_mainline.inputs.customer_draft.json `
  deployment/external_mainline.inputs.example.json `
  deployment/external_mainline.inputs.json `
  deployment/industrial_live_evidence.customer.template.json `
  deployment/security/vulnerability_exceptions.input.json `
  tools/build_customer_external_bindings_config.py `
  tools/build_customer_external_bindings_confirmation_report.py `
  tools/build_external_mainline_inputs.py `
  tools/confirm_customer_external_bindings.py `
  tools/run_customer_external_bindings_closure.py `
  tools/run_external_mainline_execution_plan.py `
  tests/test_customer_acceptance_bundle.py `
  tests/test_customer_external_bindings_config_builder.py `
  tests/test_external_mainline_execution_plan.py `
  tests/test_external_mainline_ops.py `
  tests/test_industrial_delivery_rehearsal_report.py `
  tests/test_industrial_promotion_checklist.py `
  tests/test_industrial_release_readiness.py
```

## Batch 3 · Security / Evidence / Smoke

```powershell
git add `
  tools/build_sbom_artifact.py `
  tools/build_security_posture_report.py `
  tools/build_vulnerability_exception_report.py `
  tools/build_vulnerability_exception_review_report.py `
  tools/build_vulnerability_remediation_report.py `
  tools/run_backup_restore_rehearsal.py `
  tools/run_clean_checkout_smoke.py `
  tools/run_container_vulnerability_scan.py `
  tools/run_python_vulnerability_scan.py `
  tools/write_pytest_evidence_report.py `
  tools/write_vulnerability_scan_report.py `
  tests/fixtures/pip_audit_clean_report.json `
  tests/fixtures/trivy_clean_report.json `
  tests/test_backup_restore_rehearsal_report.py `
  tests/test_clean_checkout_smoke.py `
  tests/test_pytest_evidence_report.py `
  tests/test_security_posture_reports.py `
  tests/test_security_release_preflight.py `
  tests/test_vulnerability_exception_report.py `
  tests/test_vulnerability_exception_review_report.py `
  tests/test_vulnerability_remediation_report.py `
  tests/test_vulnerability_scan_runners.py
```

## Batch 4 · MCP

```powershell
git add `
  agi_walker/core/api/mcp_tools.py `
  agi_walker/mcp/server.py `
  docs/mcp.md `
  tests/test_mcp_server.py `
  tests/test_mcp_tools.py
```

## Batch 5 · Web Panel

```powershell
git add `
  web_panel/core_api.py `
  web_panel/distributed_monitor.py `
  web_panel/server.py `
  web_panel/static/index.html `
  web_panel/static/release-closeout-plan.html `
  web_panel/static/release-closeout.html `
  web_panel/static/release-control-plane.html `
  web_panel/static/release-next.html `
  web_panel/static/workflows.html `
  web_panel/workflows_api.py `
  docs/guides/WEB_PANEL_GUIDE.md `
  tests/test_web_panel_aux_apis.py `
  tests/test_web_panel_integration_routes.py
```

## Batch 6 · Deployment Runtime

```powershell
git add `
  deployment/Dockerfile `
  deployment/Dockerfile.distributed_runtime `
  deployment/Dockerfile.web_panel `
  deployment/Dockerfile.zenoh_router `
  deployment/compose.env.example `
  deployment/docker-compose.yml `
  deployment/web_panel.env.example `
  docs/guides/DISTRIBUTED_GUIDE.md `
  docs/guides/GODOT_TESTING_GUIDE.md `
  docs/ros2/ROS2_QUICK_START.md `
  tests/run_distributed_smoke.py `
  tests/test_distributed_smoke_runner.py `
  tests/test_performance_stability.py `
  tests/test_prod_compose_smoke.py
```

## Batch 7 · Artifact / Rehearsal / Final Validation

```powershell
git add `
  agentization.md `
  docs/CURRENT_STATUS.md `
  docs/FEATURE_COMPLETION_PLAN.md `
  tools/run_clean_checkout_final_validation.py `
  tests/test_clean_checkout_final_validation.py `
  tests/test_release_artifact_builder.py `
  tests/test_stable_promotion_checklist.py `
  tests/test_workflow_orchestrator.py
```

## Batch 8 · CI

```powershell
git add .github/workflows/ci.yml
```

## Docs-only 收口批次

```powershell
git add `
  PRODUCTION_DEPLOYMENT_RUNBOOK.md `
  docs/guides/AUDIT_TRAIL_POLICY.md `
  docs/guides/BACKUP_RESTORE_RUNBOOK.md `
  docs/guides/CAPACITY_AND_SCALE.md `
  docs/guides/CLEAN_CHECKOUT_FINAL_VALIDATION_20260425.md `
  docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md `
  docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md `
  docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md `
  docs/guides/CUSTOMER_INSTALLATION_GUIDE.md `
  docs/guides/DEPLOYMENT_MATRIX.md `
  docs/guides/EXTERNAL_MAINLINE_INPUTS_FIELD_MAP_20260424.md `
  docs/guides/EXTERNAL_MAINLINE_REMAINING_INPUTS_20260424.md `
  docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md `
  docs/guides/FINAL_RELEASE_SIGNOFF_SHORT_20260425.md `
  docs/guides/FINAL_RELEASE_SIGNOFF_SUMMARY_20260425.md `
  docs/guides/FIRST_BATCH_CANDIDATE_CHECKLIST_20260424.md `
  docs/guides/FIRST_BATCH_CANDIDATE_FILES_20260424.md `
  docs/guides/FIRST_BATCH_REVIEW_DECISIONS_20260424.md `
  docs/guides/FIRST_BATCH_VERIFICATION_SEQUENCE_20260424.md `
  docs/guides/INCIDENT_RESPONSE_MATRIX.md `
  docs/guides/INDUSTRIAL_DELIVERY_OWNER_VIEW_20260424.md `
  docs/guides/INDUSTRIAL_DELIVERY_PLAN.md `
  docs/guides/INDUSTRIAL_DELIVERY_PRIORITY_MATRIX_20260424.md `
  docs/guides/INDUSTRIAL_DELIVERY_REMAINING_WORK_20260424.md `
  docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md `
  docs/guides/KNOWN_LIMITATIONS.md `
  docs/guides/MINIMAL_CLOSEOUT_REVIEW_CHECKLIST_20260424.md `
  docs/guides/MINIMAL_CLOSEOUT_SUBMISSION_SET_20260424.md `
  docs/guides/PLACEHOLDER_REPLACEMENT_CHECKLIST_20260424.md `
  docs/guides/PLACEHOLDER_TICKET_READY_TODO_20260424.md `
  docs/guides/RECOMMENDED_COMMIT_ORDER_20260425.md `
  docs/guides/RECOMMENDED_GIT_ADD_COMMANDS_20260425.md `
  docs/guides/RELEASE_AND_OPS_TRIAGE_20260424.md `
  docs/guides/RELEASE_CLOSEOUT_MISSING_INPUTS_CHECKLIST_20260424.md `
  docs/guides/RELEASE_CLOSEOUT_TASKS_20260424.md `
  docs/guides/RELEASE_GUIDE.md `
  docs/guides/SECOND_BATCH_EXCLUSION_CHECKLIST_20260424.md `
  docs/guides/SECOND_BATCH_MODULE_MATRIX_20260424.md `
  docs/guides/SECOND_BATCH_REVIEW_DECISIONS_20260424.md `
  docs/guides/SECURITY_BASELINE.md `
  docs/guides/STAGED_AND_UNSTAGED_PRIORITY_CHECKLIST_20260424.md `
  docs/guides/SUPPORT_MATRIX.md `
  docs/guides/TESTS_TRIAGE_20260424.md `
  docs/guides/VULNERABILITY_EXCEPTION_UPDATE_PLAN_20260424.md `
  docs/guides/WORKTREE_BLOCKER_CLOSEOUT_PLAN_20260424.md `
  docs/guides/WORKTREE_REVIEW_GROUPS_20260424.md
```

## 最后提醒

如果你希望完全按批次提交，建议每批执行完后都先跑对应验证，再 `git commit`。  
这样回滚和 review 都会轻松很多。
