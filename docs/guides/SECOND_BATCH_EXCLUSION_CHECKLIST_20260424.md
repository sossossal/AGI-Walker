# Second Batch Exclusion Checklist 2026-04-24

用于把当前工作区里**明确不该进入第一批收口**的改动，单独列成一张排除清单。

这份清单的目标不是回滚文件，而是帮助你把第一批 closeout 主线和第二批扩展面彻底分开。

---

## 当前结论

根据 `docs/guides/FIRST_BATCH_REVIEW_DECISIONS_20260424.md` 的分层规则，下面这些内容应优先视为**第二批**：

- `web_panel/*`
- `.github/workflows/ci.yml`
- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/run_release_rehearsal.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`
- distributed / MCP / Portal / workflow 扩展测试

---

## 建议先排除的第二批路径

### `web_panel / Portal`

- `web_panel/core_api.py`
- `web_panel/distributed_monitor.py`
- `web_panel/server.py`
- `web_panel/static/index.html`
- `web_panel/static/workflows.html`
- `web_panel/workflows_api.py`
- `web_panel/static/release-closeout-plan.html`
- `web_panel/static/release-closeout.html`
- `web_panel/static/release-control-plane.html`
- `web_panel/static/release-next.html`

### `MCP / 集成扩展`

- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `tests/test_mcp_server.py`
- `tests/test_mcp_tools.py`

### `CI / 第二批 release 扩展`

- `.github/workflows/ci.yml`
- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/run_release_rehearsal.py`
- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`

### `distributed / Portal 扩展测试`

- `tests/run_distributed_smoke.py`
- `tests/test_distributed_smoke_runner.py`
- `tests/test_web_panel_aux_apis.py`
- `tests/test_web_panel_integration_routes.py`

---

## 建议动作

对这批路径，只做下面三类动作之一：

- `第二批`：明确不放进当前第一批 closeout
- `暂缓`：还没决定，但本轮先不混入第一批
- `保留到工作区`：先留着，不执行删除或回滚

不建议在还没确认第二批意图前直接大范围 `git restore`。

---

## 推荐处理顺序

1. 先标记 `web_panel / Portal`
2. 再标记 `MCP / 集成扩展`
3. 再标记 `CI / 第二批 release 扩展`
4. 最后标记 `distributed / Portal 扩展测试`

这样做的好处是：

- 第一批收口主线会最快显形；
- `worktree blocker` 的人工不确定性会先缩到主线附近；
- 第二批扩展面不会再干扰 closeout 判定。

---

## 完成后建议重跑

```powershell
python tools/run_worktree_release_blocker.py
python tools/check_release_readiness.py
```

如果你同时调整了第一批/第二批边界，也建议再看：

- `test_env/worktree_cleanup/worktree_release_blocker_report.json`
- `test_env/release_readiness/release_readiness_report.json`
