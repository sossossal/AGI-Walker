# Release Closeout Missing Inputs Checklist 2026-04-24

更新日期：`2026-04-24`

本页把当前 `external_mainline_input_checklist_report.json` 里的 `15` 个缺口拆成逐项可勾选清单，便于按项关闭。

## 当前总览

- customer missing: `3`
- vulnerability missing: `3`
- industrial missing: `7`
- waiting steps: `3`
- ready steps: `0`
- completed steps: `0`

证据来源：

- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json`
- `test_env/release_evidence/operations/external_mainline_execution_plan.json`
- `test_env/release_readiness/release_readiness_report.json`

## 1. Customer External Bindings

- [ ] 填写真实 `approval_identity` 元数据
- [ ] 填写真实 `archive_target` 元数据
- [ ] 填写真实 `due_trigger` 元数据
- [ ] 提供真实 `confirmed_by`
- [ ] 提供真实 `confirmation_ticket`

涉及文件：

- `deployment/customer_delivery.external_bindings.customer.json`
- `deployment/customer_delivery.external_bindings.customer.overrides.json`

执行命令：

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket>
```

完成判定：

- `test_env/release_evidence/operations/customer_external_bindings_closure_report.json` 已生成
- `test_env/release_evidence/operations/customer_external_bindings_confirmation_report.json` 为通过态
- `external_mainline_input_checklist_report.json` 不再列出 customer 缺口

## 2. Vulnerability Exception Replacement

- [ ] 生成最新 review report
- [ ] 更新 `deployment/security/vulnerability_exceptions.input.json`
- [ ] 补齐最新 upstream fix 版本或重算后的 scanner 结果
- [ ] 在 `2026-05-15T00:00:00+01:00` 前完成 review / replacement

涉及文件：

- `deployment/security/vulnerability_exceptions.input.json`
- `test_env/release_evidence/security/vulnerability_exception_review_report.json`
- `test_env/release_evidence/security_release_preflight_report.json`

执行命令：

```bash
python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/run_security_release_preflight.py
```

完成判定：

- review report 已更新
- security preflight 恢复到通过态
- `external_mainline_input_checklist_report.json` 不再列出 vulnerability 缺口

## 3. Industrial Delivery Live Evidence

- [ ] 填写真实客户环境标识
- [ ] 填写真实客户环境访问方式
- [ ] 填写 `install` 实际命令或入口
- [ ] 填写 `upgrade` 实际命令或入口
- [ ] 填写 `rollback` 实际命令或入口
- [ ] 填写 `backup-restore` 实际命令或入口
- [ ] 填写 `closure archive` 与现场留痕目录

涉及文件：

- `deployment/external_mainline.inputs.json`
- `deployment/industrial_live_evidence.customer.template.json`
- `test_env/release_rehearsal_industrial/industrial_delivery_rehearsal_report.json`

参考：

- `docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md`

建议命令：

```bash
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
python tools/run_release_rehearsal.py --version <version> --build-id <build-id> --output-root test_env/release_rehearsal_industrial
```

完成判定：

- `industrial_delivery_live_evidence` 不再是 `waiting_external_input`
- `external_mainline_input_checklist_report.json` 不再列出 industrial 缺口

## 4. Release Gate 额外阻塞项

- [ ] 重跑 `python tools/run_security_release_preflight.py`
- [ ] 处理 `stable_worktree_release_blocker=blocked`
- [ ] 在 clean checkout 上重跑 `python tools/check_release_readiness.py`

执行命令：

```bash
python tools/run_worktree_release_blocker.py
python tools/check_release_readiness.py
```

完成判定：

- `stable_security_preflight` 不再是 `blocked`
- `stable_worktree_release_blocker` 不再是 `blocked`
- `stable_release_gate` 进入可闭环状态

## 推荐完成顺序

1. 先完成 customer external bindings
2. 再完成 vulnerability review / replacement
3. 再补 industrial live evidence 的真实环境字段
4. 最后在 clean checkout 上重跑 readiness
