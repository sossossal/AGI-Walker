# First Batch Actual Command Chain · 2026-04-25

## 适用范围

这份命令串对应第一批主线提交，目标是先锁住：

- release ops 主线
- readiness / promotion / rehearsal / worktree blocker
- external-mainline / customer bindings 主线
- security / evidence / smoke 基础设施

在执行前，建议先确认这 3 个混合路径的最终内容已经稳定：

- `README.md`
- `tests/run_smoke_tests.py`
- `tests/test_run_smoke_tests.py`

## Step 0 · 重新暂存 3 个混合文件

```powershell
git add README.md tests/run_smoke_tests.py tests/test_run_smoke_tests.py
```

## Step 1 · 暂存第一批主线实现

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
  tools/build_customer_external_bindings_config.py `
  tools/build_customer_external_bindings_confirmation_report.py `
  tools/build_extension_execution_actuals.py `
  tools/build_external_mainline_execution_plan.py `
  tools/build_external_mainline_input_checklist.py `
  tools/build_external_mainline_inputs.py `
  tools/build_industrial_promotion_checklist.py `
  tools/build_release_artifact.py `
  tools/build_sbom_artifact.py `
  tools/build_security_posture_report.py `
  tools/build_stable_promotion_checklist.py `
  tools/build_vulnerability_exception_report.py `
  tools/build_vulnerability_exception_review_report.py `
  tools/build_vulnerability_remediation_report.py `
  tools/build_worktree_cleanup_report.py `
  tools/check_industrial_release_readiness.py `
  tools/check_release_readiness.py `
  tools/collect_release_evidence.py `
  tools/confirm_customer_external_bindings.py `
  tools/run_backup_restore_rehearsal.py `
  tools/run_clean_checkout_final_validation.py `
  tools/run_clean_checkout_smoke.py `
  tools/run_customer_external_bindings_closure.py `
  tools/run_external_mainline_execution_plan.py `
  tools/run_release_ops.py `
  tools/run_release_rehearsal.py `
  tools/run_security_release_preflight.py `
  tools/run_worktree_release_blocker.py `
  tools/write_pytest_evidence_report.py `
  tools/write_vulnerability_scan_report.py
```

## Step 2 · 暂存第一批输入与测试

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
  tests/fixtures/pip_audit_clean_report.json `
  tests/fixtures/trivy_clean_report.json `
  tests/test_acceptance_ops.py `
  tests/test_backup_restore_rehearsal_report.py `
  tests/test_clean_checkout_final_validation.py `
  tests/test_clean_checkout_smoke.py `
  tests/test_customer_acceptance_bundle.py `
  tests/test_customer_external_bindings_config_builder.py `
  tests/test_external_mainline_execution_plan.py `
  tests/test_external_mainline_ops.py `
  tests/test_industrial_delivery_ops.py `
  tests/test_industrial_delivery_rehearsal_report.py `
  tests/test_industrial_promotion_checklist.py `
  tests/test_industrial_release_readiness.py `
  tests/test_promotion_ops.py `
  tests/test_pytest_evidence_report.py `
  tests/test_readiness_ops.py `
  tests/test_rehearsal_ops.py `
  tests/test_release_artifact_builder.py `
  tests/test_release_contracts.py `
  tests/test_release_evidence_reports.py `
  tests/test_release_ops.py `
  tests/test_release_readiness.py `
  tests/test_release_rehearsal.py `
  tests/test_security_posture_reports.py `
  tests/test_security_release_preflight.py `
  tests/test_stable_promotion_checklist.py `
  tests/test_vulnerability_exception_report.py `
  tests/test_vulnerability_exception_review_report.py `
  tests/test_vulnerability_remediation_report.py `
  tests/test_vulnerability_scan_runners.py `
  tests/test_worktree_cleanup_report.py `
  tests/test_worktree_ops.py `
  tests/test_worktree_release_blocker_report.py
```

## Step 3 · 跑第一批建议验证

```powershell
python -m pytest `
  tests/test_customer_external_bindings_config_builder.py `
  tests/test_external_mainline_execution_plan.py `
  tests/test_external_mainline_ops.py `
  tests/test_release_contracts.py `
  tests/test_release_ops.py `
  tests/test_readiness_ops.py `
  tests/test_promotion_ops.py `
  tests/test_rehearsal_ops.py `
  tests/test_worktree_cleanup_report.py `
  tests/test_worktree_ops.py `
  tests/test_worktree_release_blocker_report.py `
  tests/test_clean_checkout_smoke.py `
  tests/test_clean_checkout_final_validation.py `
  tests/test_security_posture_reports.py `
  tests/test_security_release_preflight.py `
  tests/test_vulnerability_exception_report.py `
  tests/test_vulnerability_exception_review_report.py `
  tests/test_vulnerability_remediation_report.py `
  tests/test_vulnerability_scan_runners.py `
  -q
```

## Step 4 · 生成第一批 commit

### 推荐 commit title

```powershell
git commit -m "feat(release): add deterministic release closeout backbone"
```

### 可选更完整版本

```powershell
git commit -m "feat(release): add deterministic release closeout backbone" `
  -m "- add release ops, readiness, promotion, rehearsal, and worktree gates" `
  -m "- add external-mainline, customer bindings, security evidence, and smoke foundations" `
  -m "- add regression coverage for release contracts and release ops surfaces"
```

## Step 5 · 提交后建议立即确认

```powershell
git status --short
```

你理想中会看到：

- 第一批主线已经不再出现在 staged 里
- 第二批模块（MCP / web_panel / deployment / CI / docs-only）仍然留在后续批次中

## 推荐节奏

如果第一批提交成功，下一步建议按这个顺序继续：

1. `Batch 4` MCP  
2. `Batch 5` Web Panel  
3. `Batch 6` Deployment Runtime  
4. `Batch 7` Artifact / Final Validation  
5. `Batch 8` CI  
6. docs-only 收口
