# Second Batch Review Decisions 2026-04-24

用于在 `SECOND_BATCH_MODULE_MATRIX_20260424.md` 的基础上，把当前第二批剩余内容进一步收敛成：

- `建议保留为第二批独立批次`
- `建议拆成子批次`
- `建议暂缓`

目标是让这四个模块都能进入“可做决定”的状态，而不是继续停留在笼统的“以后再看”。

---

## 总体结论

当前剩余的第二批内容，建议不要再作为一个大包处理，而是拆成 **4 个模块、6 个子批次**：

1. `CI`
2. `MCP`
3. `web_panel`
4. `scan_runners`
5. `deployment_runtime`
6. `artifact_rehearsal_docs`

这样做的好处是：

- 风险边界更清楚
- 每批验证命令更短
- 不会把 UI、CI、scanner、deployment runtime 全部捆在一起

---

## 1. `CI / MCP`

### 建议结论

- `CI`：建议**暂缓单独判断**
- `MCP`：建议**保留为第二批独立批次**

### 推荐拆分

#### `CI` 子批次

- `.github/workflows/ci.yml`

**建议动作**

- 暂不并入当前第二批实现批次
- 等 `MCP`、`scan_runners`、`deployment_runtime` 至少有一组稳定后，再决定是否把它们接入 CI

**原因**

- CI 改动会改变默认执行路径
- 一旦和未完全稳定的模块同时落地，定位失败会更困难

#### `MCP` 子批次

- `agi_walker/core/api/mcp_tools.py`
- `agi_walker/mcp/server.py`
- `docs/mcp.md`
- `tests/test_mcp_server.py`
- `tests/test_mcp_tools.py`

**建议动作**

- 作为单独第二批功能面处理
- 不与 `CI` 同批提交

**原因**

- 这是一组高度内聚的接口 / server / docs / tests 改动
- 改动体量大，但边界清楚
- 独立验证成本可控

**推荐验证**

```bash
python -m pytest tests/test_mcp_server.py tests/test_mcp_tools.py -q
```

---

## 2. `web_panel`

### 建议结论

- 建议**保留为第二批独立批次**

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

- 作为第二批独立功能面整体推进
- 不与 `distributed / deployment` 混成一个更大的提交

### 原因

- 这组已经形成完整的：
  - server
  - api
  - static pages
  - integration tests
- 模块边界非常清楚，适合单独看、单独验

### 推荐验证

```bash
python -m pytest tests/test_web_panel_aux_apis.py tests/test_web_panel_integration_routes.py -q
```

---

## 3. `scan runners`

### 建议结论

- 建议**保留为第二批独立批次**

### 路径

- `tools/run_container_vulnerability_scan.py`
- `tools/run_python_vulnerability_scan.py`
- `tests/test_vulnerability_scan_runners.py`
- `tests/fixtures/pip_audit_clean_report.json`
- `tests/fixtures/trivy_clean_report.json`

### 建议动作

- 单独作为“安全自动化 runner”批次
- 不和 `CI` 同时提交

### 原因

- 这组本身很自洽：
  - runner
  - fixture
  - test
- 但它一旦和 `CI` 一起落地，失败来源会变复杂
- 因此建议先把 runner 本身做成一个稳定批次，再考虑把它们接进 workflow

### 推荐验证

```bash
python -m pytest tests/test_vulnerability_scan_runners.py -q
```

---

## 4. `distributed / deployment`

### 建议结论

- 建议**拆成两个子批次**
  - `deployment_runtime`
  - `artifact_rehearsal_docs`

### 4.1 `deployment_runtime`

#### 路径

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
- `tests/run_distributed_smoke.py`
- `tests/test_distributed_smoke_runner.py`
- `tests/test_prod_compose_smoke.py`
- `tests/test_performance_stability.py`

#### 建议动作

- 作为第二批独立 deployment/runtime 批次
- 不和 artifact/rehearsal builder 混在一起

#### 原因

- 这一组主要回答的是：
  - 镜像怎么构建
  - compose 怎么部署
  - distributed runtime 怎么验证
- 是同一类运行时问题

#### 推荐验证

```bash
python -m pytest tests/test_distributed_smoke_runner.py tests/test_prod_compose_smoke.py tests/test_performance_stability.py -q
```

### 4.2 `artifact_rehearsal_docs`

#### 路径

- `tools/build_release_artifact.py`
- `tools/build_stable_promotion_checklist.py`
- `tools/run_release_rehearsal.py`
- `tests/test_release_artifact_builder.py`
- `tests/test_workflow_orchestrator.py`
- `docs/CURRENT_STATUS.md`
- `docs/FEATURE_COMPLETION_PLAN.md`
- `agentization.md`

#### 建议动作

- 作为第二批单独的 orchestration / artifact / rehearsal 批次
- 优先级放在 `deployment_runtime` 之后

#### 原因

- 这组会影响：
  - manifest surface
  - stable checklist
  - rehearsal chain
  - 状态文档口径
- 比纯 deployment runtime 更靠近“发布面规则”，变更影响更广

#### 推荐验证

```bash
python -m pytest tests/test_release_artifact_builder.py tests/test_stable_promotion_checklist.py tests/test_workflow_orchestrator.py -q
```

---

## 建议优先级

如果现在要继续往前推进第二批，推荐顺序是：

1. `scan_runners`
2. `MCP`
3. `web_panel`
4. `deployment_runtime`
5. `artifact_rehearsal_docs`
6. `CI`

---

## 为什么是这个顺序

- `scan_runners` 最自洽，最容易单独验证
- `MCP` 与 `web_panel` 都是边界清楚的功能面
- `deployment_runtime` 和 `artifact_rehearsal_docs` 影响面更广，适合后置
- `CI` 最好永远最后接，因为它是“把前面几批接到自动化上”的动作

---

## 现在最重要的结论

第二批已经不需要再按“34 + 12 个零散路径”来思考了。

更合适的方式是：

1. 先把它们视为 **6 个明确子批次**
2. 每次只推进其中 1 个
3. 每推进 1 个，就跑对应最短验证链

这样我们就能继续稳定地往前走，而不是再次把工作区变成一个难以判断的大拼盘。
