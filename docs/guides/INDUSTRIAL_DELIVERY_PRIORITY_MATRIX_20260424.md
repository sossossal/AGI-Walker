# Industrial Delivery Priority Matrix 2026-04-24

更新日期：`2026-04-24`

本页把当前工业级交付剩余工作按执行优先级拆成三类：

- `Blocker`：不完成就无法进入最终闭环
- `Parallel`：可以并行推进，但最终仍需回收进主链
- `Final Gate`：必须放在最后做的验证动作

## Blocker

### 1. Customer External Bindings

原因：

- `customer_external_bindings_closure` 虽已提升到 `ready_to_run`，但还没真正闭合
- 缺失真实 `confirmed_by` / `confirmation_ticket` 会直接阻塞 customer closure report 落盘

必须完成：

- 填真实 `approval_identity`
- 填真实 `archive_target`
- 填真实 `due_trigger`
- 填真实 `confirmed_by`
- 填真实 `confirmation_ticket`

输入文件：

- `deployment/customer_delivery.external_bindings.customer.overrides.json`
- `deployment/external_mainline.inputs.json`

执行命令：

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket> --skip-collect-release-evidence
```

### 2. Industrial Live Evidence

原因：

- `industrial_delivery_live_evidence` 当前仍是 `waiting_external_input`
- 不补现场真实入口，industrial 主线无法进入可执行状态

必须完成：

- 真实客户环境标识
- 真实访问方式
- `install / upgrade / rollback / backup-restore` 实际入口
- `closure archive` 现场目录

输入文件：

- `deployment/industrial_live_evidence.customer.template.json`
- `deployment/external_mainline.inputs.json`

执行命令：

```bash
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

### 3. Security Preflight 输入闭环

原因：

- 当前 `stable_security_preflight=blocked`
- 安全链不恢复，最终 release gate 不可能闭合

必须完成：

- 更新 `deployment/security/vulnerability_exceptions.input.json`
- 补最新 upstream fix / scanner 结果

执行命令：

```bash
python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json
python tools/run_security_release_preflight.py
```

## Parallel

### 1. 文档与模板完善

当前状态：

- 已基本完成
- 不再是 release blocker

保留价值：

- 继续帮助客户窗口准备
- 降低人工误填成本

### 2. Industrial Rehearsal Refresh

当前状态：

- 可在真实现场入口齐备后并行推进

命令：

```bash
python tools/run_release_rehearsal.py --version <version> --build-id <build-id> --output-root test_env/release_rehearsal_industrial
```

说明：

- 这一步有价值，但不是第一阻塞项
- 应在真实现场入口信息完整后执行

### 3. Residual Risk 持续跟踪

当前状态：

- 需要在 `2026-05-15T00:00:00+01:00` 前完成 exception review / replacement

说明：

- 这是并行风险收敛项
- 但在最终 gate 前必须回收到 security preflight 主链

## Final Gate

### 1. Worktree Blocker

必须最后执行，因为：

- 最终判定不能基于当前开发态 worktree
- 需要在接近最终提交状态时检查

命令：

```bash
python tools/run_worktree_release_blocker.py
```

### 2. Clean Checkout Readiness

必须最后执行，因为：

- 最终 release gate 判定以 clean checkout 为准
- 这是工业级交付是否真正闭环的最终验证

命令：

```bash
python tools/check_release_readiness.py
```

成功标准：

- `stable_security_preflight` 不再 `blocked`
- `stable_worktree_release_blocker` 不再 `blocked`
- `stable_release_gate` 进入可闭环状态

## 推荐顺序

1. 先做全部 `Blocker`
2. 再按条件推进 `Parallel`
3. 最后只在 clean checkout 上做 `Final Gate`

## 当前一句话结论

当前距离工业级交付的真正阻塞，不在实现层，而在：

- 真实客户输入
- 真实安全输入
- clean checkout 最终验证
