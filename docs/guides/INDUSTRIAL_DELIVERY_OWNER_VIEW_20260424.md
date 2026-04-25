# Industrial Delivery Owner View 2026-04-24

更新日期：`2026-04-24`

本页把当前工业级交付剩余工作按责任方拆成三组：

- 客户侧需要提供什么
- 交付侧需要执行什么
- 发布侧最后验证什么

## 1. 客户侧

客户侧当前需要提供的，主要是真实业务输入，而不是代码改动。

### Customer External Bindings

需要提供：

- 真实 `approval_identity` 信息
- 真实 `archive_target` 信息
- 真实 `due_trigger` 信息
- 真实确认人 `confirmed_by`
- 真实确认单号 `confirmation_ticket`

对应文件：

- `deployment/customer_delivery.external_bindings.customer.overrides.json`
- `deployment/external_mainline.inputs.json`

### Industrial Live Evidence

需要提供：

- 真实客户环境标识
- 真实客户环境访问方式
- 真实安装入口
- 真实升级入口
- 真实回滚入口
- 真实备份恢复入口
- 真实归档目录或 URI

对应文件：

- `deployment/industrial_live_evidence.customer.template.json`
- `deployment/external_mainline.inputs.json`

## 2. 交付侧

交付侧当前负责把客户输入收口成可执行 evidence。

### 需要执行

1. 执行 customer external bindings closure

```bash
python tools/run_customer_external_bindings_closure.py --config deployment/customer_delivery.external_bindings.customer.json --overrides-file deployment/customer_delivery.external_bindings.customer.overrides.json --section approval_identity --section archive_target --section due_trigger --confirmed-by <real-user> --confirmation-ticket <real-ticket> --skip-collect-release-evidence
```

2. 刷新 external-mainline

```bash
python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json
```

3. 刷新 vulnerability review

```bash
python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json
```

4. 在具备 scanner / image build 的环境里重跑 security preflight

```bash
python tools/run_security_release_preflight.py
```

### 交付侧成功判定

- `customer_external_bindings_closure_report.json` 已生成
- `customer_external_bindings_confirmation_report.json` 已生成
- `external_mainline_execution_plan.json` 已更新
- `external_mainline_input_checklist_report.json` 中 customer / industrial 缺口收敛
- `security_release_preflight_report.json` 恢复通过态

## 3. 发布侧

发布侧负责做最终 gate 验证，而不是提前在脏工作区上给结论。

### 需要执行

1. 处理 worktree blocker

```bash
python tools/run_worktree_release_blocker.py
```

2. 在 clean checkout 上重跑 readiness

```bash
python tools/check_release_readiness.py
```

### 发布侧成功判定

- `stable_worktree_release_blocker` 不再阻塞
- `stable_security_preflight` 不再阻塞
- `stable_release_gate` 进入可闭环状态

## 一句话分工

- 客户侧：提供真实字段
- 交付侧：把真实字段转成 evidence
- 发布侧：在 clean checkout 上完成最终 gate 验证
