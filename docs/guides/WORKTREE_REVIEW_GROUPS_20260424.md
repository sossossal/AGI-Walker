# Worktree Review Groups 2026-04-24

用于把当前 `git status` 的 160 个改动拆成可执行的人工审查批次，服务于 `stable_worktree_release_blocker` 收口。

## 当前结论

- 当前 `worktree_release_blocker` 仍然是 `blocked`
- 当前 `worktree_cleanup_report` 已把全部改动归类为 `manual_review`
- 当前没有 `tracked runtime/generated artifact` 误报
- 当前真正阻塞的是：工作区里仍有大量真实源码 / 文档 / 测试改动尚未人工拆分

## 推荐处理顺序

### 1. `release_and_ops`

优先处理，因为这一组最直接影响：

- `release_readiness`
- `security_release_preflight`
- `external_mainline`
- `promotion checklist`
- `customer / industrial delivery`

代表路径：

- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/core/api/release_contracts.py`
- `agi_walker/core/api/release_control_plane.py`
- `agi_walker/core/api/release_ops_contracts.py`
- `agi_walker/core/api/security_posture_contracts.py`
- `agi_walker/ops/`
- `tools/build_release_artifact.py`
- `tools/check_release_readiness.py`
- `tools/run_clean_checkout_smoke.py`
- `tools/run_security_release_preflight.py`
- `tools/run_worktree_release_blocker.py`

建议动作：

- 先确认这些文件是不是同一条 release / closeout 主线
- 如果是，作为第一批保留
- 如果不是，拆出和当前发布闭环无关的实验性改动

### 2. `delivery_and_docs`

这一组主要是交付输入面、模板、runbook 和对外文档。

代表路径：

- `README.md`
- `PRODUCTION_DEPLOYMENT_RUNBOOK.md`
- `deployment/`
- `docs/`
- `docs/guides/`
- `agentization.md`

建议动作：

- 保留直接支撑 release closeout / industrial delivery 的文档
- 暂缓与当前闭环无关的说明性文档
- 单独确认新增 deployment JSON / template 是否都属于本次交付

### 3. `web_panel`

这一组主要是 Portal / closeout / control-plane UI。

代表路径：

- `web_panel/core_api.py`
- `web_panel/distributed_monitor.py`
- `web_panel/server.py`
- `web_panel/workflows_api.py`
- `web_panel/static/release-closeout.html`
- `web_panel/static/release-closeout-plan.html`
- `web_panel/static/release-control-plane.html`
- `web_panel/static/release-next.html`

建议动作：

- 如果本次要交付 Portal 侧 closeout/control-plane，可作为独立一批
- 如果当前目标只是不让 release gate 阻塞，可以先不和 release 主线混在同一批里

### 4. `tests`

这一组覆盖面很大，但基本都是跟随主线变化的验证层。

代表路径：

- `tests/run_smoke_tests.py`
- `tests/test_release_contracts.py`
- `tests/test_release_readiness.py`
- `tests/test_release_rehearsal.py`
- `tests/test_clean_checkout_smoke.py`
- `tests/test_customer_external_bindings_config_builder.py`
- `tests/test_security_release_preflight.py`
- `tests/test_worktree_cleanup_report.py`

建议动作：

- 跟随对应实现批次一起保留
- 不要单独先处理 tests，再回头找实现
- 如果某些 tests 对应的是你不准备提交的功能，就一起剥离

### 5. `ci`

代表路径：

- `.github/workflows/ci.yml`

建议动作：

- 单独判断是否必须随本次改动进入
- 如果只是为了临时调试 CI，建议不要混入本次 release closeout 主线

## 最实用的人工拆分方式

建议按下面顺序逐组过：

1. `release_and_ops`
2. `tests`
3. `delivery_and_docs`
4. `web_panel`
5. `ci`

原因：

- `release_and_ops + tests` 最接近当前 gate / evidence 主线
- `delivery_and_docs` 次之
- `web_panel` 更适合视为独立面
- `ci` 最适合最后单独判断

## 当前 blocker 的真实含义

`stable_worktree_release_blocker=blocked` 现在表示：

- 不是 runtime artifact 误报
- 不是 generated artifact 误报
- 而是当前 checkout 里还有大量尚未人工定稿的真实改动

因此下一步不是继续修脚本，而是：

- 决定哪些改动属于本次提交
- 决定哪些改动应暂缓 / 撤回 / 另起批次
- 最终在 clean checkout 上重跑 readiness

