# Tests Triage 2026-04-24

用于细化 `WORKTREE_REVIEW_GROUPS_20260424.md` 里的 `tests` 组，把当前测试改动拆成：

- `must_keep`
- `follow_implementation`
- `separate_later`

目标是帮助你优先锁定“和 release / closeout 主线强绑定的测试”，避免在 `stable_worktree_release_blocker` 人工拆分时被大批测试文件淹没。

## 1. `must_keep`

这些测试直接覆盖我们已经推进和验证过的主线：

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

判断依据：

- 直接覆盖：
  - `clean_checkout_smoke`
  - `worktree_release_blocker`
  - `external_mainline`
  - `security_preflight`
  - `readiness`
  - `promotion checklist`
  - `customer acceptance`
  - `industrial rehearsal`
- 或者是我们这几轮实际反复跑过的测试

## 2. `follow_implementation`

这些测试大概率应该跟随 `release_and_ops` 主线一起保留，但可以在核心测试定稿后再确认：

- `tests/test_release_contracts.py`
- `tests/test_release_artifact_builder.py`
- `tests/test_backup_restore_rehearsal_report.py`
- `tests/test_external_mainline_ops.py`
- `tests/test_industrial_delivery_ops.py`
- `tests/test_promotion_ops.py`
- `tests/test_pytest_evidence_report.py`
- `tests/test_rehearsal_ops.py`
- `tests/test_vulnerability_exception_report.py`
- `tests/test_vulnerability_remediation_report.py`
- `tests/test_vulnerability_scan_runners.py`
- `tests/fixtures/pip_audit_clean_report.json`
- `tests/fixtures/trivy_clean_report.json`

判断依据：

- 更偏向契约层、builder 层、report builder 层和 scan runner 层
- 很重要，但不一定需要在你先判断“核心主线是否保留”之前立刻决定

建议：

- 如果保留对应实现文件，就一起保留这些测试
- 如果你想先压缩最小提交面，可以先把它们放进第二批

## 3. `separate_later`

这些测试更像“同一时期一起动过，但未必必须和当前 closeout 主线一起提交”的内容：

- `tests/run_distributed_smoke.py`
- `tests/test_distributed_smoke_runner.py`
- `tests/test_mcp_server.py`
- `tests/test_mcp_tools.py`
- `tests/test_performance_stability.py`
- `tests/test_prod_compose_smoke.py`
- `tests/test_web_panel_aux_apis.py`
- `tests/test_web_panel_integration_routes.py`
- `tests/test_workflow_orchestrator.py`
- `tests/test_acceptance_ops.py`

判断依据：

- 偏 distributed / MCP / Portal / workflow / compose smoke 扩展面
- 对仓库整体完整性有价值，但和当前“先解除 closeout 主线阻塞”的优先级相比略低

注意：

- 这里不是说这些测试不重要
- 而是说它们更适合作为第二或第三批确认

## 推荐处理顺序

### 第一批：跟主线一起锁定

- `must_keep`

### 第二批：跟对应实现一起过

- `follow_implementation`

### 第三批：最后确认是否纳入当前提交

- `separate_later`

## 最重要的结论

如果你的目标是尽快让当前 worktree 审查变得可控，`tests` 这一组不要一次性全看。更高效的方式是：

1. 先锁定 `clean_checkout_smoke / worktree / external_mainline / security_preflight / readiness`
2. 再锁定 `customer_acceptance / industrial_rehearsal / promotion_checklist`
3. 最后再判断 distributed / MCP / Portal / workflow 扩展测试

