# Release Closeout Tasks 2026-04-24

更新日期：`2026-04-24`

本页根据当前仓库里的最新 `release_readiness`、`external_mainline_execution_plan` 与 `external_mainline_input_checklist` 生成，目的是给出“距离完成还差什么”的最短执行视图。

## 当前状态

- `external_mainline_execution_plan.status=ready`
- `external_mainline_execution_plan.summary=completed=0, ready_to_run=1, waiting_external_input=2, blocked=0`
- `external_mainline_input_checklist.status=blocked`
- `external_mainline_input_checklist.metrics.missing_input_count=15`
- `stable_release_gate=blocked`
- `stable_security_preflight=blocked`
- `stable_customer_delivery=ready`
- `stable_industrial_delivery=blocked`
- `stable_worktree_release_blocker=blocked`

## 距离完成还剩哪些任务

### 1. 客户 External Bindings 收口

还缺：

- `confirmed_by`
- `confirmation_ticket`
- 真实客户 `approval_identity / archive_target / due_trigger` 元数据

建议顺序：

1. 填写 `deployment/customer_delivery.external_bindings.customer.overrides.json`
2. 运行 closure runner
3. 复查 closure report 与 confirmation report

命令：

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket>
```

参考：

- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md`
- `docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md`

### 2. Vulnerability Exception 替换或复核

还缺：

- 在 `2026-05-15T00:00:00+01:00` 前完成 replacement / review
- 更新 `deployment/security/vulnerability_exceptions.input.json`
- 最新 upstream fix 版本或重算后的 scanner 结果

当前可直接执行的自动步骤：

```bash
python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json
```

后续建议：

1. 先重建 review report
2. 再更新 exception 输入
3. 再重跑 security preflight

### 3. Industrial 现场真实留痕

还缺：

- 真实客户环境标识
- 真实客户环境访问方式
- `install` 实际命令或入口
- `upgrade` 实际命令或入口
- `rollback` 实际命令或入口
- `backup-restore` 实际命令或入口
- `closure archive` 与现场留痕目录

当前 rehearsal baseline 已有，但还只是演练路径，不是客户现场留痕。

参考命令：

```bash
python tools/run_release_rehearsal.py --version <version> --build-id <build-id> --output-root test_env/release_rehearsal_industrial
```

### 4. Security Preflight 重算

当前 `check_release_readiness.py` 最新结果显示：

- `stable_security_preflight=blocked`

说明当前 canonical security 链需要重算或补齐。建议在 vulnerability exception 输入更新后执行：

```bash
python tools/run_security_release_preflight.py
```

如果这条命令耗时过长或依赖真实 scanner / image build，请在具备对应环境的节点执行。

### 5. Clean Worktree

当前 `stable_worktree_release_blocker=blocked`，说明即使业务证据闭合，stable 仍会被当前开发工作区阻塞。

建议命令：

```bash
python tools/run_worktree_release_blocker.py
```

完成后应在 clean checkout 上重跑：

```bash
python tools/check_release_readiness.py
```

## 最短完成路径

如果只看“离闭环最近的一条路”，建议按这个顺序：

1. 完成 customer external bindings
2. 重建 vulnerability exception review 并更新 exception 输入
3. 重跑 `python tools/run_security_release_preflight.py`
4. 补齐 industrial live evidence 的真实环境字段
5. 在 clean checkout 上重跑 `python tools/check_release_readiness.py`

## 机器证据入口

- `test_env/release_evidence/operations/external_mainline_execution_plan.json`
- `test_env/release_evidence/operations/external_mainline_input_checklist_report.json`
- `test_env/release_readiness/release_readiness_report.json`
