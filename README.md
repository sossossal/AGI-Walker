# AGI-Walker

AGI-Walker 是一个面向机器人设计、仿真、工作流编排和 Web/Godot 集成的 Python 平台。仓库当前包含 Skills 系统、Workflow Orchestrator、Web Panel、Godot Agent backend，以及一个可直接接入 MCP 客户端的 `stdio` server。

## 主要能力

- `Skills + Workflows`：用 `SKILL.md` 和 workflow 定义组织机器人建模、参数优化、任务执行等流程。
- `Web Panel`：基于 FastAPI 的控制面板，提供 workflow、服务状态、Godot 会话和 nightly 状态接口。
- `Capability Matrix`：提供版本化发布面矩阵，统一描述 CLI、Web、MCP、distributed 和 Godot integration 的发布状态。
- `Godot 集成`：同时支持 legacy backend 和 modern `godot-agent` backend。
- `Godot 扩展控制面`：新增结构化指令集控制与模拟电路通信契约，可复用到 Godot / ROS2 / IMC-22 近似层。
- `ROS2 指令集模拟`：支持通过 JSON topic / service 重放结构化 `instruction_set` 与模拟电路配置，并发布运行态快照。
- `Distributed / Smoke`：仓库内置 CLI、workflow、Web、distributed 和可选的 Godot headless smoke 测试。
- `MCP Server`：通过 `agi_walker.mcp.server` 把任务执行、workflow、skills 和 Godot 能力暴露给 MCP 客户端。

## 安装

要求：

- Python `>=3.10`

基础安装：

```bash
pip install -e .
```

开发依赖：

```bash
pip install -e ".[dev]"
```

## 快速开始

列出 skills：

```bash
python -m agi_walker.cli skills list
```

运行环境自检：

```bash
python -m agi_walker.cli doctor
```

查看 workflow 帮助：

```bash
python -m agi_walker.cli workflows run --help
```

启动 Web Panel：

```bash
python -m web_panel.server
```

默认地址：

```text
http://localhost:8000
```

Portal 侧现在还提供只读 release/control-plane 总览入口：

```text
GET /api/release/control-plane
```

Portal 侧现在还提供更细粒度的只读入口：

```text
GET /api/release/control-plane/surface
GET /api/release/control-plane/catalog
GET /api/release/control-plane/action
GET /api/release/control-plane/next
GET /api/release/control-plane/request-templates
GET /api/release/control-plane/request-file
GET /api/release/next
GET /api/release/next/primary
GET /api/release/next/follow-up
GET /api/release/next/request-file
GET /api/release/closeout
GET /api/release/closeout/next
GET /api/release/closeout/plan
GET /api/release/closeout/plan/stage
GET /api/release/closeout/plan/next
GET /api/release/closeout/component
```

其中总览入口会返回 canonical `control_plane_surface`、`release_ops` catalog、`request_templates` 和 `release_closeout`；`surface` 可单独读取 canonical control-plane surface，catalog / request-templates 两条细粒度路由适合 Portal/HTTP 客户端分别拉取动作目录或按 action 生成 `request-file` 草稿，而新增的 `action` 路由会把单个 action 的 catalog entry 与 request template 聚合成一份只读 payload。新增的 `next` 路由则会把“推荐下一步 action”的 `action_definition`、`request_template` 与 `request-file scaffold` 收成单一 payload，适合 Portal/MCP/HTTP 客户端直接消费推荐入口。新增的 `request-file` 路由则会直接返回单 action 的 request-file scaffold、推荐文件名和 pretty JSON，适合 Portal/MCP/HTTP 客户端直接导出草稿。`release/next` 则会把 control-plane next 与 closeout next 再聚合成统一的“当前 release 下一步”入口。`closeout` 则专门汇总 external-mainline、安全 residual-risk 与 worktree blocker 三类剩余收口项，并直接附带 `action_items` / `command`；新增的 `closeout/next` 会把推荐下一步 component 的 `action_item` 与组件 payload 收成单一只读入口，`closeout/component` 会返回单组件的只读聚合详情，而 `closeout/plan` 会把剩余 external closeout 输入拆成分阶段 runbook，直接给出输入文件、建议命令与完成标准。`closeout/plan/stage` 和 `closeout/plan/next` 则把这份 runbook 再收成单阶段与推荐下一阶段入口，适合 Portal/MCP/HTTP 客户端直接消费当前聚焦 stage，而不必继续在前端本地过滤整份 plan。

`/api/release/control-plane` 总览入口本身现在也会直接带 `next_action`、`next_action_route`、`next_action_request_route`、`next_action_request_file_route` 和 `next_action_request_file_name`，因此客户端即使不做本地聚合，也能直接拿到“下一步 action”的详情深链与 request-file 草稿入口。

同一条 `/api/release/control-plane/request-file` 现在还支持 `download=1`，会直接返回带 `Content-Disposition` 文件名的 JSON 下载响应；对应的 canonical payload 也会附带 `request_file_download_route`，因此 Portal 不需要再本地拼装下载逻辑。

上述 `action` / `request-templates` / `closeout` / `closeout/component` payload 现在还会直接携带 canonical Portal/API route 字段，例如 `portal_route`、`action_route`、`request_template_route`、`component_route` 和 `component_api_route`，因此 Portal/MCP/HTTP 客户端不需要再本地拼接深链或 JSON 路径。

首页 dashboard 现在也会直接消费 `/api/system/status` 里的 `release_control_plane` 与 `release_closeout` 摘要，显示 control-plane 状态、动作/模板计数、policy profiles / route，以及 closeout 的 `blocked / waiting / ready / missing` 计数；同一份 summary 现在还会附带 `top_action_items`，所以首页还能直接看到“下一步组件”和建议命令，而不必再打开 `/api/release/closeout` 详情。Portal 还新增了独立静态页 `/static/release-closeout.html`，会直接消费 `/api/release/closeout` 并展开 action items、阻塞输入和建议命令；同时新增 `/static/release-closeout-plan.html`，会直接消费 `/api/release/closeout/plan`，把 external closeout 剩余输入拆成阶段化执行计划。

首页 dashboard 里的 `release_closeout` 摘要现在还会直接带 `/static/release-closeout.html?component=...` 深链，因此不打开 closeout 详情页也能直接跳到当前优先收口组件；同时 summary 也会附带 canonical `/api/release/closeout/next`，首页因此可以直接暴露“下一步 JSON”快捷入口。closeout 详情页本身现在会直接使用 `/api/release/closeout/component?component=...`，稳定渲染单组件视图。

同一页在未选择 component 时，现在会优先消费 `/api/release/closeout/next`，直接读取推荐下一步 component 的 `action_item` 与组件 payload，而不再继续从 `/api/release/closeout` 总览 payload 本地挑第一项。

首页 dashboard 现在还会直接消费 `/api/system/status.release_next`，统一显示当前推荐的 release 下一步，并链接到 canonical `/api/release/next`，这样 Portal 不需要再同时从 control-plane 和 closeout 两边本地推断“现在应该先做什么”。同一块摘要现在还直接暴露 `primary_portal_route`、`primary_api_route`、canonical `primary_payload_route`、`primary_follow_up_route`、`primary_follow_up_payload_route` 和 `primary_follow_up_download_route`，因此不打开 `release-next.html` 也能直接跳到“主推荐详情 / 主推荐 JSON / 下一步执行 / Follow-up JSON / 下一步草稿下载”，并且“主推荐 JSON”不再复用具体 action/component 的 API 路径。当当前主推荐项是 control-plane request-file 时，首页现在还会优先消费 unified `request_file_route / request_file_payload_route / request_file_download_route`，而不再继续跳回底层 `primary_follow_up_*` 路由；同时首页和 `release-next.html` 也都会显式提供 `request-file JSON` 入口，不再把它只隐含在 follow-up 区块里。

Portal 现在还新增了独立静态页 `/static/release-next.html`，它会直接消费单一 `/api/release/next` canonical payload；这个总览 payload 现在内嵌 `primary_payload`、`follow_up_payload` 与 `request_file_payload`，因此页面不再额外拉 `/api/release/next/primary` 与 `/api/release/next/follow-up` 来本地拼装，并且当当前主推荐项是 control-plane request-file 时，也会优先走统一 `/api/release/next/request-file` 下载草稿。与此同时，独立 `release_next_primary` / `release_next_follow_up` 入口仍然保留给 MCP/HTTP 客户端做细粒度只读读取。

Portal 现在还新增了独立静态页 `/static/release-control-plane.html`，会直接消费 `/api/release/control-plane`，展开 canonical `control_plane_surface`、`release_ops` catalog 和 `request_templates`；页面现在还会直接使用 `/api/release/control-plane/action?action=...` 做按 action 的模板钻取，并把选中的 action 同步到 URL 查询参数，因此已经可以在 Portal 内直接浏览和分享单个 action 的 request schema 深链。在未选择 action 时，页面会优先消费 `/api/release/control-plane/next`，直接绑定推荐下一步 action 的 schema 与 request-file scaffold，而不是继续从总览 payload 本地拆默认值。

同一页现在还会直接使用 `/api/release/control-plane/request-file?action=...`，并提供“查看 request-file / 下载草稿 / 复制 JSON”三个入口，因此 Portal 里已经可以直接导出某个 action 的 request-file scaffold，而不需要再手工拼 JSON。

首页 dashboard 里的 `release_control_plane` 摘要现在还会直接带 `next_action` 和 `/static/release-control-plane.html?action=...` 深链，因此不打开详情页也能直接跳到推荐 action。

首页 dashboard 现在还会直接带 `next_action_request_file_route` 和推荐的 `request_file_name`，因此不打开详情页也能直接跳到下一步 action 的 request-file 草稿 JSON。

## 客户部署

当前唯一的一线客户部署路径是 Docker Compose。

最小控制面：

```powershell
Copy-Item deployment/compose.env.example deployment/compose.env
Copy-Item deployment/web_panel.env.example deployment/web_panel.env
docker compose --env-file deployment/compose.env -f deployment/docker-compose.yml up -d --build zenoh-router web-panel
```

客户部署文档入口：

- [部署矩阵](docs/guides/DEPLOYMENT_MATRIX.md)
- [客户安装指南](docs/guides/CUSTOMER_INSTALLATION_GUIDE.md)
- [支持矩阵](docs/guides/SUPPORT_MATRIX.md)
- [容量与规模声明](docs/guides/CAPACITY_AND_SCALE.md)
- [客户验收清单](docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md)
- [已知限制](docs/guides/KNOWN_LIMITATIONS.md)
- [生产部署 Runbook](PRODUCTION_DEPLOYMENT_RUNBOOK.md)

## MCP 集成

仓库已经提供 MCP `stdio` server。安装后可以直接启动：

```bash
agi-walker-mcp
```

也可以使用模块入口：

```bash
python -m agi_walker.mcp.server
```

当前暴露的工具包括：

- `mission_execute`
- `robot_telemetry`
- `rag_query`
- `workflows_list`
- `workflow_get`
- `workflow_execute`
- `skills_list`
- `skill_get`
- `godot_agent_status`
- `godot_agent_templates`
- `godot_agent_plan`
- `godot_agent_doctor`
- `godot_agent_history`
- `capability_matrix_get`
- `release_control_plane_surface_get`
- `release_closeout_get`
- `release_closeout_component_get`
- `release_closeout_next_get`
- `release_closeout_plan_get`
- `release_closeout_plan_stage_get`
- `release_closeout_plan_next_get`
- `release_ops_catalog_get`
- `release_ops_request_templates_get`
- `release_control_plane_action_get`
- `release_control_plane_next_get`
- `release_control_plane_request_file_get`
- `release_control_plane_index_get`
- `release_next_get`
- `release_next_primary_get`
- `release_next_follow_up_get`
- `release_next_request_file_get`

这些工具覆盖的能力范围：

- 自然语言任务规划与执行
- workflow 查询与执行
- skills 元数据和文档读取
- Godot Agent backend 状态、模板、计划、自检和历史记录
- 发布面能力矩阵和契约版本摘要
- canonical release/control-plane surface 摘要，优先读取 `release_manifest.control_plane_surface`
- release closeout 的统一剩余问题聚合面，以及单组件只读详情
- release closeout 推荐下一步 component 的只读聚合详情，可直接读取建议命令与组件 payload
- release closeout 的分阶段执行计划，可直接读取输入文件、建议命令和完成标准
- release_ops control plane 的只读 action catalog 与 policy profiles
- release_ops action 的只读 request template defaults，可直接用于生成 `request-file` 草稿
- release/control-plane 单 action 的只读聚合详情，可直接读取 catalog entry 与 request template
- release/control-plane 推荐下一步 action 的只读聚合详情，可直接读取推荐 schema 与 request-file scaffold
- release/control-plane 单 action 的只读 request-file scaffold，可直接导出 `request-file` 草稿
- control-plane surface、release_ops catalog 与 request template defaults 的单入口只读聚合视图
- 统一 release 下一步聚合视图，会把 control-plane next 与 closeout next 收成单入口只读 payload
- 统一 release 下一步里的主推荐入口，可直接读取 primary kind/name/status 与对应的 Portal/API 深链
- 统一 release 下一步里的 canonical follow-up contract，可直接读取 request-file / 建议命令 / 下一步 JSON 的归一化入口
- 统一 release 下一步里的 follow-up 只读入口，可直接读取当前主推荐项对应的执行/导出动作

更完整的说明见 [docs/mcp.md](docs/mcp.md)。

## 验证

MCP 相关测试：

```bash
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

仓库 smoke 测试：

```bash
python tests/run_smoke_tests.py
```

默认 smoke 现在还会覆盖：

- Godot instruction-set smoke
- ROS2 instruction-set smoke
- simulated circuit replay smoke

统一 release-ops control plane：

```bash
python tools/run_release_ops.py --list-actions
python tools/run_release_ops.py --list-policy-profiles
python tools/run_release_ops.py stable_promotion_checklist --request-file path/to/request.json
python tools/run_release_ops.py stable_promotion_checklist --engagement-id eng-20260421 --window-id window-rc --change-ticket CHG-4242 --channel stable --event-stream-file test_env/release_ops/stable_promotion.jsonl --request-file path/to/request.json
python tools/run_release_ops.py stable_promotion_checklist --request-file path/to/request.json --evidence-report-file test_env/release_ops/stable_promotion_evidence.json
python tools/run_release_ops.py external_mainline_execution --policy-profile requires_attestation --request-file path/to/request.json
```

`run_release_ops.py` 默认使用 `local_safe_refresh` policy profile；像 `external_mainline_execution` 这类 `requires_attestation` action 现在必须显式传入 `--policy-profile requires_attestation`，否则 control plane 会 fail closed。
同一入口现在也支持轻量 delivery session 上下文与 JSONL 事件流：可通过 `--engagement-id`、`--window-id`、`--change-ticket`、`--channel` 传入现场上下文，并通过 `--event-stream-file` 落盘 `action_started`、`action_completed`、`artifact_written`、`policy_denied` 等控制面事件。
当 `release_ops` 以 `external_mainline_execution` 运行时，这组 control-plane session / event-stream 摘要还会回写到 `external_mainline_execution_plan.json` 与 `external_mainline_input_checklist_report.json`，并继续进入 customer acceptance / rehearsal / industrial delivery summary surface。`worktree_release_blocker` action 也会把同一组摘要回写到 `worktree_release_blocker_report.json`，供 readiness / promotion / smoke surface 继续透传。现在 `release_readiness`、`industrial_release_readiness`、`stable_promotion_checklist` 与 `industrial_promotion_checklist` 这四类顶层 artifact 也会在 `release_ops` 运行后直接带上 `control_plane_session` / `control_plane_event_stream`，因此 control-plane 输出不再只存在于 preview/component 层；同一组聚合字段现在也会继续提升到 `customer_acceptance_bundle.json`、`release_rehearsal_report.json` 与 `industrial_delivery_rehearsal_report.json` 顶层，供 acceptance / rehearsal / industrial delivery 摘要直接消费。若显式传入 `--evidence-report-file`，control plane 还会额外写出一份标准 `release_evidence_report` 作为 action-level canonical wrapper；最新 contract 还会把这份 `release_ops_execution` evidence 继续挂进 `customer_acceptance_bundle`、`release_rehearsal_report` 与 `industrial_delivery_rehearsal_report`，并进一步提升到 `release_manifest.customer_delivery_surface`、`release_manifest.industrial_delivery_gate`、`release_manifest` 顶层以及统一 `control_plane_surface`，因此 action/policy/request_type/output_path/event_count 与 delivery-session 摘要不再只停留在 CLI 临时结果里。

采集 release evidence：

```bash
python tools/collect_release_evidence.py
python tools/build_extension_execution_evidence.py --output-root test_env/release_evidence/operations
python tools/build_extension_execution_instance.py --output test_env/release_evidence/operations/extension_execution_instance.json
python tools/build_extension_execution_schedule.py --output test_env/release_evidence/operations/extension_execution_schedule.json
python tools/build_extension_execution_actuals.py --output test_env/release_evidence/operations/extension_execution_actuals.json --schedule-artifact test_env/release_evidence/operations/extension_execution_schedule.json
```

生成发布门禁产物：

```bash
python tools/build_release_artifact.py --version 2026.04.12-rc1 --channel rc --build-id build-20260412-001 --release-summary "阶段五发布门禁闭环。"
```

严格工业交付通道会在 stable 基础上额外要求 `customer_delivery_surface.status=ready` 与 `industrial_delivery_gate.status=ready`：

```bash
python tools/build_release_artifact.py --version 2026.04.12 --channel industrial --build-id build-20260412-industrial --release-summary "工业交付门禁闭环。"
```

如需先机器化预览当前 HEAD 离 industrial 发布还差什么，可运行：

```bash
python tools/check_industrial_release_readiness.py
python tools/build_industrial_promotion_checklist.py
```

当前 `release_manifest`、`industrial_delivery_gate` 和 `customer_acceptance_bundle` 还会附带 `extension_support_surface`，把 distributed / ROS2 / Godot / Helm-Kubernetes 的支持边界，以及每个扩展面的 `deployment_commands`、`acceptance_checks`、`rollback_prerequisites` 一起压成机器可读交付字段。
`customer_acceptance_bundle`、`industrial_promotion_checklist` 和 `release_rehearsal_report` 现在还会额外挂出 `extension_execution_plan`，用于把这些动作以执行面摘要带给交付、验收和 rehearsal 链路。
`release_manifest`、`customer_acceptance_bundle`、`industrial_release_readiness_report`、`industrial_promotion_checklist` 和 `release_rehearsal_report` 现在还会附带 `extension_execution_evidence`，把 `extension_on_call_rehearsal`、`extension_exception_review_schedule`、`extension_escalation_closure` 和 `customer_external_bindings_confirmation` 四类留痕报告压成机器可读交付字段。
同一批产物现在还会附带 `extension_execution_instance`，把客户实例化的 `engagement_id`、交付窗口、exception 复核到期时间、delivery root 和 closure archive root 收口成机器字段。
同一批产物现在也会附带 `extension_execution_schedule`，把 `window_trigger_at`、`signoff_due_at`、`closure_archive_due_at`、`residual_risk_review_record_path` 和 `closure_manifest_path` 一起压成客户窗口排程面。
同一批产物现在也会附带 `extension_execution_actuals`，把 `window_trigger_recorded_at`、`signoff_recorded_at`、`residual_risk_reviewed_at`、`closure_archived_at`、`*_recorded_by`、`approval_identity_source_path`、`approval_identity_reference`、`archive_target_binding_type`、`archive_target_binding_reference_base`、`due_trigger_binding_type`、`due_trigger_binding_reference_base`、`due_trigger_checked_at`、`exception_review_due_at`、`closure_archive_due_at` 以及 profile 级的 `approval_identity_source.json`、`window_trigger.json`、`signoff.json`、`exception_review.json`、`residual_risk_review.json`、`due_trigger_check.json`、`archive_target.json`、`closure_archive/index.json`、`closure_manifest.json` 收口成客户窗口实际执行与后续跟进留痕面。
每个 actionable profile 现在还会附带 `runbook_entrypoints`，把正式执行文档入口固定到 `PRODUCTION_DEPLOYMENT_RUNBOOK.md`、客户安装指南以及 distributed / ROS2 / Godot 专项指南。
每个 profile 现在还会附带 `execution_template`，把现场角色分工、值班动作、on-call 交接记录、残余风险交接、exception 到期复核、异常升级闭环证据和回滚证据归档责任压成可交付字段。

可选的 Godot headless smoke：

```powershell
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
python -m pytest tests/test_godot_headless_smoke.py -q -m "integration and live" --tb=short -vv
```

ROS2 bridge smoke 本地环境补齐清单：

- `docs/guides/ROS2_LOCAL_ENV_CHECKLIST_20260425.md`
- `docs/guides/ROS2_WINDOWS_BOOTSTRAP_20260425.md`

本地测试状态摘要：

- `docs/guides/LOCAL_TEST_STATUS_SUMMARY_20260425.md`

当前仓库 + 本地验证完成度摘要：

- `docs/guides/CURRENT_REPO_AND_LOCAL_VALIDATION_STATUS_20260425.md`

## 目录概览

- `agi_walker/`：核心 Python 包，包含 CLI、workflow、skills、integrations 和 MCP server。
- `agi_walker/ops/`：确定性 release / delivery orchestration service，当前已开始承接 external-mainline、readiness、promotion checklist、rehearsal、worktree blocker、customer acceptance bundle、industrial delivery rehearsal report 与统一 `release_ops` action dispatch / policy enforcement，供 CLI runner 和后续控制面复用。
- `web_panel/`：FastAPI Web 控制面板和相关 API。
- `godot_project/`：Godot 工程与场景资源。
- `godot_studio_agent/`：Godot Agent 侧角色、路由和工具。
- `tests/`：单元测试、smoke 测试和集成测试。
- `docs/`：架构、迁移、CLI/Web/MCP 指南及当前状态说明。

## 文档入口

- [当前状态](docs/CURRENT_STATUS.md)
- [严格工业化交付执行计划](docs/guides/INDUSTRIAL_DELIVERY_PLAN.md)
- [部署矩阵](docs/guides/DEPLOYMENT_MATRIX.md)
- [客户安装指南](docs/guides/CUSTOMER_INSTALLATION_GUIDE.md)
- [支持矩阵](docs/guides/SUPPORT_MATRIX.md)
- [容量与规模声明](docs/guides/CAPACITY_AND_SCALE.md)
- [客户验收清单](docs/guides/CUSTOMER_ACCEPTANCE_CHECKLIST.md)
- [客户 External Bindings 清单](docs/guides/CUSTOMER_EXTERNAL_BINDINGS_CHECKLIST.md)
- [客户 External Bindings 示例值](docs/guides/CUSTOMER_EXTERNAL_BINDINGS_EXAMPLE_VALUES.md)
- [Release 收口任务清单 2026-04-24](docs/guides/RELEASE_CLOSEOUT_TASKS_20260424.md)
- [Release 缺口勾选清单 2026-04-24](docs/guides/RELEASE_CLOSEOUT_MISSING_INPUTS_CHECKLIST_20260424.md)
- [Industrial Live Evidence 清单](docs/guides/INDUSTRIAL_LIVE_EVIDENCE_CHECKLIST.md)
- [External Mainline Inputs 对照表 2026-04-24](docs/guides/EXTERNAL_MAINLINE_INPUTS_FIELD_MAP_20260424.md)
- [最终收口 Runbook 2026-04-24](docs/guides/FINAL_CLOSEOUT_RUNBOOK_20260424.md)
- [Clean Checkout 最终验证 2026-04-25](docs/guides/CLEAN_CHECKOUT_FINAL_VALIDATION_20260425.md)
- [最终发布签核摘要 2026-04-25](docs/guides/FINAL_RELEASE_SIGNOFF_SUMMARY_20260425.md)
- [最终发布签核短版 2026-04-25](docs/guides/FINAL_RELEASE_SIGNOFF_SHORT_20260425.md)
- [建议提交顺序 2026-04-25](docs/guides/RECOMMENDED_COMMIT_ORDER_20260425.md)
- [建议 Git Add 命令 2026-04-25](docs/guides/RECOMMENDED_GIT_ADD_COMMANDS_20260425.md)
- [建议 Commit Message 2026-04-25](docs/guides/RECOMMENDED_COMMIT_MESSAGES_20260425.md)
- [第一批实际命令串 2026-04-25](docs/guides/FIRST_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md)
- [第二批实际命令串 2026-04-25](docs/guides/SECOND_BATCH_ACTUAL_COMMAND_CHAIN_20260425.md)
- [极简提交顺序 2026-04-25](docs/guides/COMPACT_SUBMISSION_SEQUENCE_20260425.md)
- [最短下一步操作 2026-04-25](docs/guides/SHORTEST_NEXT_ACTIONS_20260425.md)
- [工业级交付剩余工作 2026-04-24](docs/guides/INDUSTRIAL_DELIVERY_REMAINING_WORK_20260424.md)
- [工业级交付优先级矩阵 2026-04-24](docs/guides/INDUSTRIAL_DELIVERY_PRIORITY_MATRIX_20260424.md)
- [工业级交付负责人视图 2026-04-24](docs/guides/INDUSTRIAL_DELIVERY_OWNER_VIEW_20260424.md)
- [占位符替换清单 2026-04-24](docs/guides/PLACEHOLDER_REPLACEMENT_CHECKLIST_20260424.md)
- [工单版占位符待办 2026-04-24](docs/guides/PLACEHOLDER_TICKET_READY_TODO_20260424.md)
- [Worktree 审查分组 2026-04-24](docs/guides/WORKTREE_REVIEW_GROUPS_20260424.md)
- [Release & Ops 分诊 2026-04-24](docs/guides/RELEASE_AND_OPS_TRIAGE_20260424.md)
- [Tests 分诊 2026-04-24](docs/guides/TESTS_TRIAGE_20260424.md)
- [最小收口提交面 2026-04-24](docs/guides/MINIMAL_CLOSEOUT_SUBMISSION_SET_20260424.md)
- [最小收口审查清单 2026-04-24](docs/guides/MINIMAL_CLOSEOUT_REVIEW_CHECKLIST_20260424.md)
- [第一批候选文件清单 2026-04-24](docs/guides/FIRST_BATCH_CANDIDATE_FILES_20260424.md)
- [第一批候选勾选表 2026-04-24](docs/guides/FIRST_BATCH_CANDIDATE_CHECKLIST_20260424.md)
- [第一批勾选决定 2026-04-24](docs/guides/FIRST_BATCH_REVIEW_DECISIONS_20260424.md)
- [第一批验证顺序 2026-04-24](docs/guides/FIRST_BATCH_VERIFICATION_SEQUENCE_20260424.md)
- [External Mainline 剩余输入 2026-04-24](docs/guides/EXTERNAL_MAINLINE_REMAINING_INPUTS_20260424.md)
- [Worktree Blocker 收口计划 2026-04-24](docs/guides/WORKTREE_BLOCKER_CLOSEOUT_PLAN_20260424.md)
- [第二批剥离清单 2026-04-24](docs/guides/SECOND_BATCH_EXCLUSION_CHECKLIST_20260424.md)
- [第二批模块矩阵 2026-04-24](docs/guides/SECOND_BATCH_MODULE_MATRIX_20260424.md)
- [第二批分栏决定 2026-04-24](docs/guides/SECOND_BATCH_REVIEW_DECISIONS_20260424.md)
- [Vulnerability Exception 更新计划 2026-04-24](docs/guides/VULNERABILITY_EXCEPTION_UPDATE_PLAN_20260424.md)
- [混合路径优先处理清单 2026-04-24](docs/guides/STAGED_AND_UNSTAGED_PRIORITY_CHECKLIST_20260424.md)
- [已知限制](docs/guides/KNOWN_LIMITATIONS.md)
- [MCP 集成说明](docs/mcp.md)
- [CLI 指南](docs/guides/CLI_GUIDE.md)
- [Web Panel 指南](docs/guides/WEB_PANEL_GUIDE.md)
- [发布指南](docs/guides/RELEASE_GUIDE.md)
- [迁移指南](docs/MIGRATION_GUIDE.md)
- [生产部署 Runbook](PRODUCTION_DEPLOYMENT_RUNBOOK.md)

## License

[MIT](LICENSE)
