# Recommended Commit Order · 2026-04-25

## 目标

把当前已经完成的 closeout / release / smoke / signoff 结果，整理成一组 **可分批提交** 的建议顺序，尽量做到：

- 每批边界清楚
- 每批都有对应验证面
- 第一批先锁住 release 主线
- 第二批再接扩展面
- 最后再放文档收口与签核摘要

## 提交前先处理的 3 个混合文件

当前还有少数路径处于 `MM` / `AM` 状态，建议先确认 staged 与 working tree 是否一致，再开始正式提交：

- `README.md`
- `tests/run_smoke_tests.py`
- `tests/test_run_smoke_tests.py`

建议动作：

1. 先确认这些文件的 working tree 内容就是你要的最终版本  
2. 如需要，把最新版本重新 `git add` 一次  
3. 再按下面批次顺序提交

## 建议提交顺序

### Batch 1 · Release Ops 主线

优先提交 release / readiness / promotion / rehearsal / worktree blocker 主线实现与 contract：

- `agi_walker/core/api/release_contracts.py`
- `agi_walker/core/api/release_control_plane.py`
- `agi_walker/core/api/release_ops_contracts.py`
- `agi_walker/core/api/security_posture_contracts.py`
- `agi_walker/ops/__init__.py`
- `agi_walker/ops/acceptance.py`
- `agi_walker/ops/external_mainline.py`
- `agi_walker/ops/industrial_delivery.py`
- `agi_walker/ops/promotion.py`
- `agi_walker/ops/readiness.py`
- `agi_walker/ops/rehearsal.py`
- `agi_walker/ops/release_ops.py`
- `agi_walker/ops/worktree.py`
- `tools/build_customer_acceptance_bundle.py`
- `tools/build_extension_execution_actuals.py`
- `tools/build_external_mainline_execution_plan.py`
- `tools/build_external_mainline_input_checklist.py`
- `tools/build_industrial_promotion_checklist.py`
- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/build_worktree_cleanup_report.py`
- `tools/check_industrial_release_readiness.py`
- `tools/check_release_readiness.py`
- `tools/collect_release_evidence.py`
- `tools/run_release_ops.py`
- `tools/run_release_rehearsal.py`
- `tools/run_security_release_preflight.py`
- `tools/run_worktree_release_blocker.py`

推荐验证：

- `python -m pytest tests/test_release_contracts.py tests/test_release_ops.py tests/test_readiness_ops.py tests/test_promotion_ops.py tests/test_rehearsal_ops.py tests/test_worktree_cleanup_report.py tests/test_worktree_ops.py tests/test_worktree_release_blocker_report.py -q`

### Batch 2 · External Mainline 与 Customer Bindings

提交 external-mainline 输入闭环与客户 external bindings 链：

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
- `tools/build_customer_external_bindings_config.py`
- `tools/build_customer_external_bindings_confirmation_report.py`
- `tools/build_external_mainline_inputs.py`
- `tools/confirm_customer_external_bindings.py`
- `tools/run_customer_external_bindings_closure.py`
- `tools/run_external_mainline_execution_plan.py`
- `tests/test_customer_external_bindings_config_builder.py`
- `tests/test_external_mainline_execution_plan.py`
- `tests/test_external_mainline_ops.py`

推荐验证：

- `python -m pytest tests/test_customer_external_bindings_config_builder.py tests/test_external_mainline_execution_plan.py tests/test_external_mainline_ops.py -q`

### Batch 3 · Security / Evidence / Smoke 基础设施

提交 security posture、scan runner、pytest evidence、clean checkout smoke 主线：

- `tools/build_sbom_artifact.py`
- `tools/build_security_posture_report.py`
- `tools/build_vulnerability_exception_report.py`
- `tools/build_vulnerability_exception_review_report.py`
- `tools/build_vulnerability_remediation_report.py`
- `tools/run_backup_restore_rehearsal.py`
- `tools/run_clean_checkout_smoke.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`
- `tools/write_pytest_evidence_report.py`
- `tools/write_vulnerability_scan_report.py`
- `tests/fixtures/pip_audit_clean_report.json`
- `tests/fixtures/trivy_clean_report.json`
- `tests/test_backup_restore_rehearsal_report.py`
- `tests/test_clean_checkout_smoke.py`
- `tests/test_pytest_evidence_report.py`
- `tests/test_security_posture_reports.py`
- `tests/test_security_release_preflight.py`
- `tests/test_vulnerability_exception_report.py`
- `tests/test_vulnerability_exception_review_report.py`
- `tests/test_vulnerability_remediation_report.py`
- `tests/test_vulnerability_scan_runners.py`

推荐验证：

- `python -m pytest tests/test_backup_restore_rehearsal_report.py tests/test_clean_checkout_smoke.py tests/test_pytest_evidence_report.py tests/test_security_posture_reports.py tests/test_security_release_preflight.py tests/test_vulnerability_exception_report.py tests/test_vulnerability_exception_review_report.py tests/test_vulnerability_remediation_report.py tests/test_vulnerability_scan_runners.py -q`

### Batch 4 · MCP

将 MCP 作为独立批次提交：

- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `docs/mcp.md`
- `tests/test_mcp_server.py`
- `tests/test_mcp_tools.py`

推荐验证：

- `python -m pytest tests/test_mcp_server.py tests/test_mcp_tools.py -q`

### Batch 5 · Web Panel

将 web panel 独立提交：

- `web_panel/core_api.py`
- `web_panel/distributed_monitor.py`
- `web_panel/server.py`
- `web_panel/static/index.html`
- `web_panel/static/release-closeout-plan.html`
- `web_panel/static/release-closeout.html`
- `web_panel/static/release-control-plane.html`
- `web_panel/static/release-next.html`
- `web_panel/static/workflows.html`
- `web_panel/workflows_api.py`
- `docs/guides/WEB_PANEL_GUIDE.md`
- `tests/test_web_panel_aux_apis.py`
- `tests/test_web_panel_integration_routes.py`

推荐验证：

- `python -m pytest tests/test_web_panel_aux_apis.py tests/test_web_panel_integration_routes.py -q`

### Batch 6 · Deployment Runtime

提交 distributed / deployment runtime 面：

- `deployment/Dockerfile`
- `deployment/Dockerfile.distributed_runtime`
- `deployment/Dockerfile.web_panel`
- `deployment/Dockerfile.zenoh_router`
- `deployment/compose.env.example`
- `deployment/docker-compose.yml`
- `deployment/web_panel.env.example`
- `docs/guides/DISTRIBUTED_GUIDE.md`
- `docs/guides/GODOT_TESTING_GUIDE.md`
- `docs/ros2/ROS2_QUICK_START.md`
- `tests/run_distributed_smoke.py`
- `tests/test_distributed_smoke_runner.py`
- `tests/test_performance_stability.py`
- `tests/test_prod_compose_smoke.py`

推荐验证：

- `python -m pytest tests/test_distributed_smoke_runner.py tests/test_prod_compose_smoke.py tests/test_performance_stability.py -q`

### Batch 7 · Artifact / Rehearsal / Workflow Docs

提交 release artifact、workflow orchestrator 与状态文档：

- `agentization.md`
- `docs/CURRENT_STATUS.md`
- `docs/FEATURE_COMPLETION_PLAN.md`
- `tests/test_release_artifact_builder.py`
- `tests/test_stable_promotion_checklist.py`
- `tests/test_workflow_orchestrator.py`
- `tools/run_clean_checkout_final_validation.py`
- `tests/test_clean_checkout_final_validation.py`

推荐验证：

- `python -m pytest tests/test_release_artifact_builder.py tests/test_stable_promotion_checklist.py tests/test_workflow_orchestrator.py tests/test_clean_checkout_final_validation.py -q`

### Batch 8 · CI

最后再提交 CI：

- `.github/workflows/ci.yml`

建议最后提交的原因：

- 它会把新的验证面正式接到自动化
- 更适合在其它批次稳定后再落地

## 文档收口批次

如果你更喜欢把“执行指南 / 分诊 / 签核摘要”单独成批，建议把以下文档放到最后一个 docs-only 提交：

- `docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md`
- `docs/guides/CLEAN_CHECKOUT_FINAL_VALIDATION_20260425.md`
- `docs/guides/FINAL_RELEASE_SIGNOFF_SUMMARY_20260425.md`
- `docs/guides/FINAL_RELEASE_SIGNOFF_SHORT_20260425.md`
- 以及本轮新增的分诊、矩阵、清单类文档

## 最实用的执行建议

如果你想最稳地落地，推荐节奏是：

1. 先提交 `Batch 1 + Batch 2 + Batch 3`
2. 再提交 `Batch 4 + Batch 5 + Batch 6`
3. 再提交 `Batch 7`
4. 最后单独提交 `Batch 8`

这样做的好处是：

- 先锁 release 主线与证据链
- 再锁扩展面
- 最后再把 CI 接上
