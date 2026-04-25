# Second Batch Module Matrix 2026-04-24

用于把当前 `worktree blocker` 剩余的第二批内容，按模块拆成更容易人工判断的四组：

- `CI / MCP`
- `distributed / deployment`
- `web_panel`
- `scan runners`

目标不是立刻把它们并入第一批，而是把“剩下这 34 + 12 到底是什么”压缩成一页可执行清单。

---

## 总体结论

- 当前剩余内容已经高度集中在第二批扩展面，不再是第一批 closeout 主线边界不清。
- 这些路径大多与：
  - CI 执行链
  - MCP 暴露面
  - distributed / deployment 扩展
  - web panel UI / API
  - 真实 scanner runner
  强绑定。
- 因此推荐策略是：
  1. 不再继续把它们混入第一批
  2. 先按模块人工判断是否要保留为第二批
  3. 每个模块单独验证，再决定是否 staged

---

## 1. `CI / MCP`

### 路径

- `.github/workflows/ci.yml`
- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `docs/mcp.md`
- `tests/test_mcp_server.py`
- `tests/test_mcp_tools.py`

### 建议动作

- `建议批次`：第二批
- `建议处理`：单独判断、单独验证、不要混入第一批 closeout

### 原因

- 这组变更会扩大：
  - CI 执行面
  - MCP 暴露面
  - 对外工具接口面
- 它们和当前第一批目标（release/readiness/external-mainline/security/worktree closeout）并非完全同一条最小闭环。

### 验证建议

```bash
python -m pytest tests/test_mcp_server.py tests/test_mcp_tools.py -q
```

如果要验证 CI 配置改动，还应额外人工审查：

- job 是否引入新依赖
- 是否改变默认触发条件
- 是否把真实 scanner / build 链强绑定到常规 PR 路径

---

## 2. `distributed / deployment`

### 路径

- `deployment/docker-compose.yml`
- `deployment/Dockerfile`
- `deployment/Dockerfile.distributed_runtime`
- `deployment/Dockerfile.web_panel`
- `deployment/web_panel.env.example`
- `deployment/compose.env.example`
- `deployment/Dockerfile.zenoh_router`
- `docs/guides/DISTRIBUTED_GUIDE.md`
- `docs/guides/GODOT_TESTING_GUIDE.md`
- `docs/ros2/ROS2_QUICK_START.md`
- `docs/CURRENT_STATUS.md`
- `docs/FEATURE_COMPLETION_PLAN.md`
- `tests/run_distributed_smoke.py`
- `tests/test_distributed_smoke_runner.py`
- `tests/test_prod_compose_smoke.py`
- `tests/test_performance_stability.py`
- `tests/test_release_artifact_builder.py`
- `tests/test_workflow_orchestrator.py`
- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/run_release_rehearsal.py`
- `agentization.md`

### 建议动作

- `建议批次`：第二批
- `建议处理`：拆成两个子面单独判断
  - `deployment/runtime`
  - `release artifact / rehearsal / status docs`

### 原因

- 这一组虽然和交付有关，但已经超出第一批“最小 closeout 提交面”：
  - 牵涉 distributed runtime
  - 牵涉 compose / Docker 镜像
  - 牵涉 rehearsal / artifact 扩展
  - 牵涉更广的状态与计划文档更新

### 推荐子顺序

1. 先看 `deployment/*`
2. 再看 `tools/build_release_artifact.py`、`tools/build_stable_promotion_checklist.py`、`tools/run_release_rehearsal.py`
3. 最后看 `docs/CURRENT_STATUS.md`、`docs/FEATURE_COMPLETION_PLAN.md` 与专项指南

### 验证建议

```bash
python -m pytest tests/test_release_artifact_builder.py tests/test_prod_compose_smoke.py tests/test_distributed_smoke_runner.py -q
python tests/run_distributed_smoke.py --help
```

如果准备纳入第二批，还建议补看：

- Dockerfile 是否引入新基础镜像风险
- compose env 示例是否与当前文档一致
- rehearsal / artifact builder 是否改变 canonical output surface

---

## 3. `web_panel`

### 路径

- `web_panel/core_api.py`
- `web_panel/distributed_monitor.py`
- `web_panel/server.py`
- `web_panel/workflows_api.py`
- `web_panel/static/index.html`
- `web_panel/static/workflows.html`
- `web_panel/static/release-closeout-plan.html`
- `web_panel/static/release-closeout.html`
- `web_panel/static/release-control-plane.html`
- `web_panel/static/release-next.html`
- `docs/guides/WEB_PANEL_GUIDE.md`
- `tests/test_web_panel_aux_apis.py`
- `tests/test_web_panel_integration_routes.py`

### 建议动作

- `建议批次`：第二批
- `建议处理`：整体作为一个独立功能面判断

### 原因

- 这组是典型的 UI + API 联动改动：
  - 一旦纳入，就不再只是 closeout 主线
  - 而是把 release control plane / web panel 能力一起带进提交
- 适合独立评估，而不是夹带在第一批里。

### 验证建议

```bash
python -m pytest tests/test_web_panel_aux_apis.py tests/test_web_panel_integration_routes.py -q
```

如果要继续推进，还应人工确认：

- 静态页面入口是否与现有导航一致
- `server.py` / `core_api.py` 是否新增对外路由
- UI 文案是否和当前 closeout 实际状态一致

---

## 4. `scan runners`

### 路径

- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`
- `tests/test_vulnerability_scan_runners.py`
- `tests/fixtures/pip_audit_clean_report.json`
- `tests/fixtures/trivy_clean_report.json`

### 建议动作

- `建议批次`：第二批
- `建议处理`：作为安全自动化增强单独提交

### 原因

- 第一批已经通过现有 canonical evidence 和 preflight 收口；
- 这组属于“把真实 scanner 执行链继续产品化”的增强，不是解除当前 blocker 的最低必要条件。

### 验证建议

```bash
python -m pytest tests/test_vulnerability_scan_runners.py -q
```

如果要实际启用，还需人工确认：

- 本机/CI 是否具备 `pip-audit`
- 本机/CI 是否具备 `trivy`
- 运行失败时是否会误伤常规 closeout 链路

---

## 推荐处理顺序

如果目标是继续把 `worktree blocker` 收到“只剩非常明确的第二批”，建议按下面顺序处理：

1. `CI / MCP`
2. `web_panel`
3. `scan runners`
4. `distributed / deployment`

原因：

- `CI / MCP` 和 `web_panel` 最容易明确为“不要混进第一批”
- `scan runners` 自成体系，也容易单独打包
- `distributed / deployment` 面最广、耦合最多，适合最后处理

---

## 现在最重要的结论

当前剩余路径已经足够清楚地表明：

- 第一批 closeout 主线基本已经收口
- `worktree blocker` 之所以还在，主要是因为第二批扩展面仍保留在工作区

因此下一步最合理的动作不是继续扩大第一批，而是：

1. 明确这些模块是否进入第二批
2. 按模块单独验证
3. 再决定是否 staged / 暂缓 / 独立提交
