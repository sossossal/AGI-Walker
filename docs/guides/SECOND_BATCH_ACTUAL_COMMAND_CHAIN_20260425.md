# Second Batch Actual Command Chain · 2026-04-25

## 适用范围

这份命令串对应第二批扩展面提交，目标是按独立批次继续收口：

- `Batch 4` MCP
- `Batch 5` Web Panel
- `Batch 6` Deployment Runtime
- `Batch 7` Artifact / Final Validation
- `Batch 8` CI

建议前提：

- 第一批已经提交完成
- 当前工作区只剩第二批与 docs-only 收口内容

## Batch 4 · MCP

### 暂存

```powershell
git add `
  agi_walker/core/api/mcp_tools.py `
  agi_walker/mcp/server.py `
  docs/mcp.md `
  tests/test_mcp_server.py `
  tests/test_mcp_tools.py
```

### 验证

```powershell
python -m pytest tests/test_mcp_server.py tests/test_mcp_tools.py -q
```

### 提交

```powershell
git commit -m "feat(mcp): expand release and closeout MCP surfaces"
```

## Batch 5 · Web Panel

### 暂存

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

### 验证

```powershell
python -m pytest tests/test_web_panel_aux_apis.py tests/test_web_panel_integration_routes.py -q
```

### 提交

```powershell
git commit -m "feat(web-panel): add release closeout and control-plane web surfaces"
```

## Batch 6 · Deployment Runtime

### 暂存

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

### 验证

```powershell
python -m pytest `
  tests/test_distributed_smoke_runner.py `
  tests/test_prod_compose_smoke.py `
  tests/test_performance_stability.py `
  -q
```

### 提交

```powershell
git commit -m "feat(deployment): add distributed runtime and deployment validation surfaces"
```

## Batch 7 · Artifact / Final Validation

### 暂存

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

### 验证

```powershell
python -m pytest `
  tests/test_release_artifact_builder.py `
  tests/test_stable_promotion_checklist.py `
  tests/test_workflow_orchestrator.py `
  tests/test_clean_checkout_final_validation.py `
  -q
```

### 提交

```powershell
git commit -m "feat(signoff): add clean-checkout final validation and signoff helpers"
```

## Batch 8 · CI

### 暂存

```powershell
git add .github/workflows/ci.yml
```

### 建议检查

```powershell
git diff --cached -- .github/workflows/ci.yml
```

### 提交

```powershell
git commit -m "ci: align workflows with release closeout validation"
```

## Docs-only 收口批次

如果你希望把分诊 / runbook / 签核 / 提交指南统一作为最后一个 docs-only commit，可以使用：

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
  docs/guides/FIRST_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md `
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
  docs/guides/RECOMMENDED_COMMIT_MESSAGES_20260425.md `
  docs/guides/RECOMMENDED_COMMIT_ORDER_20260425.md `
  docs/guides/RECOMMENDED_GIT_ADD_COMMANDS_20260425.md `
  docs/guides/RELEASE_AND_OPS_TRIAGE_20260424.md `
  docs/guides/RELEASE_CLOSEOUT_MISSING_INPUTS_CHECKLIST_20260424.md `
  docs/guides/RELEASE_CLOSEOUT_TASKS_20260424.md `
  docs/guides/RELEASE_GUIDE.md `
  docs/guides/SECOND_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md `
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

### docs-only 提交建议

```powershell
git commit -m "docs(release): add closeout, signoff, and submission guides"
```

## 推荐顺序

建议第二批实际执行顺序：

1. `Batch 4` MCP
2. `Batch 5` Web Panel
3. `Batch 6` Deployment Runtime
4. `Batch 7` Artifact / Final Validation
5. `Batch 8` CI
6. docs-only 收口
