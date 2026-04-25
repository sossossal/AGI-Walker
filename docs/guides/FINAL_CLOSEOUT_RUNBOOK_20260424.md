# Final Closeout Runbook 2026-04-24

更新日期：`2026-04-24`

本页用于执行 AGI-Walker 当前剩余的最终收口动作。适用前提是：

- `deployment/customer_delivery.external_bindings.customer.overrides.json` 已填入真实客户字段
- `deployment/external_mainline.inputs.json` 已填入真实 `confirmed_by` / `confirmation_ticket`
- `deployment/external_mainline.inputs.json` 已填入真实 `industrial_live_evidence` 字段

## 执行顺序

### 1. 先闭合 customer external bindings

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket> --skip-collect-release-evidence
```

成功判定：

- `test_env/release_evidence/operations/customer_external_bindings_closure_report.json` 已生成
- `test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json` 已生成
- `customer_external_bindings_closure_status=passed`

### 2. 刷新 external-mainline 计划

```bash
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

成功判定：

- `test_env/release_evidence/operations/external_mainline_execution_plan.json` 已更新
- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json` 已更新
- `customer_external_bindings_closure` 不再是 `waiting_external_input`
- `industrial_delivery_live_evidence` 不再是 `waiting_external_input`

### 3. 刷新 vulnerability review

```bash
python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json
```

成功判定：

- `test_env/release_evidence/security/vulnerability_exception_review_report.json` 已更新
- review report 进入当前可接受状态

### 4. 重跑 security preflight

```bash
python tools/run_security_release_preflight.py
```

成功判定：

- `test_env/release_evidence/security_release_preflight_report.json` 已更新
- `stable_security_preflight` 不再是 `blocked`

说明：

- 如果该命令依赖真实 scanner / image build，请在具备对应环境的节点执行

### 5. 处理 worktree 阻塞

```bash
python tools/run_worktree_release_blocker.py
```

成功判定：

- `test_env/worktree_cleanup/worktree_release_blocker_report.json` 已更新
- `stable_worktree_release_blocker` 不再阻塞最终收口

### 6. 在 clean checkout 上重跑 readiness

```bash
python tools/check_release_readiness.py
```

成功判定：

- `test_env/release_readiness/release_readiness_report.json` 已更新
- `stable_security_preflight` 不再 blocked
- `stable_worktree_release_blocker` 不再 blocked
- `stable_release_gate` 进入可闭环状态

## 建议检查顺序

每一步执行后按这个顺序检查：

1. 先看命令 stdout
2. 再看对应 JSON 报告是否生成
3. 再看下一步是否仍有 `waiting_external_input` 或 `blocked`

## 推荐核对文件

- `deployment/customer_delivery.external_bindings.customer.overrides.json`
- `deployment/external_mainline.inputs.json`
- `test_env/release_evidence/operations/customer_external_bindings_closure_report.json`
- `test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json`
- `test_env/release_evidence/operations/external_mainline_execution_plan.json`
- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json`
- `test_env/release_evidence/security/vulnerability_exception_review_report.json`
- `test_env/release_evidence/security_release_preflight_report.json`
- `test_env/worktree_cleanup/worktree_release_blocker_report.json`
- `test_env/release_readiness/release_readiness_report.json`
