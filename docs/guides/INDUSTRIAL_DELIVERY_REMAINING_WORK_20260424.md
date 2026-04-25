# Industrial Delivery Remaining Work 2026-04-24

更新日期：`2026-04-24`

本页用于汇总 AGI-Walker 距离“工业级交付闭环”还剩哪些工作，并按当前状态拆成四类：

- 已完成
- 待客户输入
- 待环境执行
- 待最终验证

## 已完成

- 工业化交付相关 contract、runner、报告链和 Portal/MCP/Web 只读面已落地
- `customer external bindings`、`industrial live evidence`、`external_mainline` 的模板和对照文档已补齐
- `external_mainline_execution_plan` 已能稳定生成
- `vulnerability_exception_review_report` 已能重建并输出结构化结果
- `worktree_release_blocker` 与 `release_readiness` 报告链已可用
- 最终执行手册已整理完成：
  - `docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md`

## 待客户输入

这类工作当前不是缺代码，而是缺真实业务信息。

### Customer External Bindings

- 真实 `approval_identity` 元数据
- 真实 `archive_target` 元数据
- 真实 `due_trigger` 元数据
- 真实 `confirmed_by`
- 真实 `confirmation_ticket`

相关文件：

- `deployment/customer_delivery.external_bindings.customer.overrides.json`
- `deployment/customer_delivery.external_bindings.customer.json`
- `deployment/external_mainline.inputs.json`

### Industrial Live Evidence

- 真实客户环境标识
- 真实客户环境访问方式
- `install` 实际命令或入口
- `upgrade` 实际命令或入口
- `rollback` 实际命令或入口
- `backup-restore` 实际命令或入口
- `closure archive` 与现场留痕目录

相关文件：

- `deployment/industrial_live_evidence.customer.template.json`
- `deployment/external_mainline.inputs.json`

## 待环境执行

这类工作需要在具备真实环境、scanner、镜像构建或 clean checkout 条件的节点执行。

### Security / Vulnerability 链

- 根据最新 upstream fix 或 scanner 结果更新：
  - `deployment/security/vulnerability_exceptions.input.json`
- 重跑：
  - `python tools/build_vulnerability_exception_review_report.py --output test_env/release_evidence/security/vulnerability_exception_review_report.json --exception-report test_env/release_evidence/security/vulnerability_exception_report.json`
  - `python tools/run_security_release_preflight.py`

### External Mainline

- 在 customer / industrial 字段补齐后重跑：
  - `python tools/run_customer_external_bindings_closure.py ...`
  - `python tools/run_external_mainline_execution_plan.py --output test_env/release_evidence/operations/external_mainline_execution_plan.json`

### Industrial 现场链

- 如需真实工业交付现场留痕，补跑：
  - `python tools/run_release_rehearsal.py --version <version> --build-id <build-id> --output-root test_env/release_rehearsal_industrial`

## 待最终验证

这类工作是最终 gate 判定，不应在当前开发态 worktree 上作为最终结论。

### Clean Worktree

- 运行：
  - `python tools/run_worktree_release_blocker.py`
- 目标：
  - `stable_worktree_release_blocker` 不再阻塞

### Clean Checkout Readiness

- 在 clean checkout 上运行：
  - `python tools/check_release_readiness.py`
- 目标：
  - `stable_security_preflight` 不再 `blocked`
  - `stable_worktree_release_blocker` 不再 `blocked`
  - `stable_release_gate` 进入可闭环状态

## 当前最短剩余路径

1. 先闭合 `customer external bindings`
2. 再补齐 `industrial live evidence`
3. 更新 vulnerability exception 输入并重跑 security preflight
4. 在 clean checkout 上重跑 worktree blocker 和 readiness

## 当前结论

截至 `2026-04-24`，AGI-Walker 距离工业级交付不再缺基础实现，剩余工作已经主要转为：

- 真实客户输入
- 真实环境执行
- 最终 clean checkout 验证

这意味着当前阶段的主要风险不再是“仓库功能面未完成”，而是“外部输入是否及时到位，以及最终验证是否在正确环境执行”。
