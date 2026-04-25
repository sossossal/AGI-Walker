# Recommended Commit Messages · 2026-04-25

## 使用方式

这份文档给出与当前批次拆分对应的建议 commit message 草稿。  
你可以直接使用，也可以在保留结构的前提下按团队风格微调。

建议风格：

- 标题一句话说清本批次主题
- 如需正文，只补 2–4 行关键变化
- 保持和验证面一致，方便 review

## Batch 1 · Release Ops 主线

### 建议标题

```text
feat(release): add deterministic release ops, readiness, promotion, and worktree gates
```

### 可选正文

```text
- add release ops contracts and orchestration services
- wire readiness, promotion, rehearsal, and worktree blocker flows
- add regression coverage for release contracts and ops surfaces
```

## Batch 2 · External Mainline / Customer Bindings

### 建议标题

```text
feat(external-mainline): add customer bindings closure and managed input surfaces
```

### 可选正文

```text
- add customer external bindings configs and confirmation flow
- add external-mainline execution and input checklist tooling
- cover managed closure and external-mainline behavior with tests
```

## Batch 3 · Security / Evidence / Smoke

### 建议标题

```text
feat(release-evidence): add security posture, scan runners, and clean-checkout smoke tooling
```

### 可选正文

```text
- add sbom, vulnerability, remediation, and posture report builders
- add scan runners and pytest evidence writers
- add clean-checkout smoke and related regression coverage
```

## Batch 4 · MCP

### 建议标题

```text
feat(mcp): expand MCP release and closeout surfaces
```

### 可选正文

```text
- expose release, closeout, and external-mainline surfaces through MCP
- update MCP server wiring and docs
- add MCP server and tool regression coverage
```

## Batch 5 · Web Panel

### 建议标题

```text
feat(web-panel): add release closeout and control-plane web surfaces
```

### 可选正文

```text
- extend web panel release APIs and workflows surfaces
- add release closeout static pages
- cover panel routes with integration and auxiliary API tests
```

## Batch 6 · Deployment Runtime

### 建议标题

```text
feat(deployment): add distributed runtime and deployment validation surfaces
```

### 可选正文

```text
- update compose and Docker runtime assets
- add distributed deployment and runtime docs
- cover distributed smoke and deployment validation flows
```

## Batch 7 · Artifact / Rehearsal / Final Validation

### 建议标题

```text
feat(signoff): add final validation, artifact closeout, and signoff helpers
```

### 可选正文

```text
- add clean checkout final validation runner and tests
- stabilize release artifact and signoff verification flows
- update status and planning docs for final closeout
```

## Batch 8 · CI

### 建议标题

```text
ci: update workflow coverage for release closeout and validation
```

### 可选正文

```text
- align CI workflow with new release validation surfaces
- keep automated coverage in sync with staged release gates
```

## Docs-only 收口批次

### 建议标题

```text
docs(release): add closeout, signoff, and commit workflow guides
```

### 可选正文

```text
- add closeout, placeholder, worktree, and signoff guidance
- document clean-checkout validation and release gate results
- add commit order, git add, and commit message references
```

## 最简短版

如果你想保持非常短的提交标题，也可以直接用这组：

- `feat(release): add release ops closeout backbone`
- `feat(external-mainline): add managed closure inputs`
- `feat(release-evidence): add smoke and security evidence tools`
- `feat(mcp): extend release MCP surfaces`
- `feat(web-panel): add release closeout pages`
- `feat(deployment): add distributed runtime assets`
- `feat(signoff): add clean-checkout final validation`
- `ci: align workflows with release gates`
- `docs(release): add closeout and signoff guides`
