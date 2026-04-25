# Release And Ops Triage 2026-04-24

用于细化 `WORKTREE_REVIEW_GROUPS_20260424.md` 里的 `release_and_ops` 组，把当前 release / closeout 主线改动拆成：

- `must_keep`
- `likely_keep`
- `separate_later`

目标不是自动替你删改，而是帮助你优先锁定最关键的一批文件，降低 `stable_worktree_release_blocker` 的人工拆分成本。

## 1. `must_keep`

这些文件直接支撑当前已经推进的主线结果：

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

判断依据：

- 直接参与 `security_preflight`
- 直接参与 `clean_checkout_smoke`
- 直接参与 `external_mainline`
- 直接参与 `readiness / promotion / customer acceptance / industrial rehearsal`
- 或者已经在我们刚跑过的修复链路里被验证到

## 2. `likely_keep`

这些文件大概率也属于同一条 release/control-plane 主线，但可以在 `must_keep` 之后再确认：

- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/build_worktree_cleanup_report.py`
- `tools/run_release_rehearsal.py`
- `agi_walker/ops/__init__.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`

判断依据：

- 它们支撑 Portal / MCP / stable promotion / vulnerability scan runner 的完整面
- 对当前 closeout 主线是重要增强，但未必每个都必须和第一批一起落

建议：

- 如果你希望“发布面 + 控制面 + CLI/MCP”一起闭环，就保留
- 如果你想先最小化收口，可以在 `must_keep` 定稿后再判断

## 3. `separate_later`

这类文件更像“同一时期发生，但不一定必须和 closeout 主线一起提交”的内容：

- 与 scan runner 相关、但当前环境里暂未直接验证的补充 runner
- 只为 Portal/MCP 展示层补语义，而不影响当前 evidence 产出闭环的薄包装改动
- 纯结构整理、导出包装、兼容性别名层

当前 `release_and_ops` 里相对更适合后置确认的候选：

- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`

注意：

- 这里不是说这些文件“不该提交”
- 而是说它们比 `readiness / external-mainline / clean-checkout / security-preflight` 主链优先级更低

## 推荐落地顺序

### 第一批：先定稿

- `release_contracts`
- `release_control_plane / release_ops_contracts / security_posture_contracts`
- `agi_walker/ops/*`
- `build_* / run_*` 中直接影响：
  - `external_mainline`
  - `extension_execution_*`
  - `security_preflight`
  - `clean_checkout_smoke`
  - `worktree_release_blocker`
  - `readiness`
  - `customer_acceptance_bundle`
  - `industrial_*`

### 第二批：再跟随补齐

- `build_release_artifact.py`
- `build_stable_promotion_checklist.py`
- `mcp_tools.py`
- `mcp/server.py`

### 第三批：最后确认

- `run_container_vulnerability_scan.py`
- `run_python_vulnerability_scan.py`

## 最重要的结论

如果你的目标是尽快解除 `stable_worktree_release_blocker` 的人工不确定性，最应该优先看的不是所有 50 个 `release_and_ops` 文件，而是：

1. `agi_walker/ops/`
2. `agi_walker/core/api/release_*`
3. `tools/run_*` / `tools/build_*` 中与：
   - `external_mainline`
   - `extension_execution`
   - `clean_checkout_smoke`
   - `security_preflight`
   - `readiness`
   直接相关的那一批

