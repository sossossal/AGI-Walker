# MCP 集成

AGI-Walker 现在提供一个可直接被 MCP 客户端接入的 `stdio` server。

## 安装

```bash
pip install -e .
```

安装后会得到一个命令行入口：

```bash
agi-walker-mcp
```

也可以直接使用模块入口：

```bash
python -m agi_walker.mcp.server
```

## 已暴露工具

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

## 工具能力范围

- `mission_execute`：把自然语言任务交给 `SimplePlanner` 和 `WorkflowOrchestrator`
- `workflow_*`：读取和执行内置 workflow
- `skills_*`：读取 `SKILL.md` 元数据和正文
- `godot_agent_*`：读取 Godot Agent backend 状态、模板、计划、自检和历史
- `capability_matrix_get`：读取发布面 capability matrix、契约版本和已知限制
- `release_control_plane_surface_get`：读取 canonical release/control-plane surface，优先来自 `release_manifest.control_plane_surface`，缺失时回退到 canonical `release_ops_execution_report.json`
- `release_closeout_get`：读取剩余 release closeout 问题的统一只读聚合面，汇总 `external_mainline_execution_plan`、`vulnerability_exception_review` 与 `worktree_release_blocker`，并直接带 `action_items` / `command`
- `release_closeout_component_get`：读取单个 release closeout component 的只读聚合详情，返回 component payload、对应 action item 与 Portal 深链
- `release_closeout_next_get`：读取推荐下一步 release closeout component 的只读聚合详情，直接返回 `action_item`、组件 payload 与建议命令
- `release_closeout_plan_get`：读取 external closeout 的分阶段执行计划，直接返回阶段、输入文件、建议命令和完成标准
- `release_closeout_plan_stage_get`：读取单个 external closeout plan stage 的只读聚合详情，适合客户端按 `stage` 深链直接消费
- `release_closeout_plan_next_get`：读取推荐下一步 external closeout plan stage 的只读聚合详情，适合客户端直接消费当前焦点 stage
- `release_ops_catalog_get`：读取 release_ops control plane 的只读 action catalog 与 policy profiles，用于客户端发现当前可用动作及其 policy level
- `release_ops_request_templates_get`：读取 release_ops action 的只读 request template defaults，可选按 action 过滤，适合客户端直接生成 `request-file` 草稿而不触发执行
- `release_control_plane_next_get`：读取推荐下一步 release/control-plane action 的只读聚合详情，直接返回 `action_definition`、`request_template` 与 `request-file scaffold`
- `release_control_plane_request_file_get`：读取单个 release/control-plane action 的只读 request-file scaffold，返回推荐文件名、Portal/API route 和 pretty JSON，适合客户端直接导出 request-file 草稿
- `release_control_plane_action_get`：读取单个 release/control-plane action 的只读聚合详情，返回 catalog entry、request template 与对应的 filtered route
- `release_control_plane_index_get`：一次返回 canonical control-plane surface、`release_closeout`、release_ops catalog 与 request template defaults，适合 MCP 客户端先做总览再决定读取细项
- `release_next_get`：一次读取统一 release 下一步入口，聚合 control-plane next 与 closeout next
- `release_next_primary_get`：读取统一 release 下一步里的主推荐入口，直接返回 primary kind/name/status、对应 Portal/API 深链，以及 canonical follow-up contract（request-file / 建议命令 / 下一步 JSON）
- `release_next_follow_up_get`：读取统一 release 下一步里的规范化 follow-up 入口，直接返回当前主推荐项对应的执行/导出动作
- `release_next_request_file_get`：读取统一 release 下一步里的 request-file 导出入口；仅当当前主推荐项为 control-plane request-file 时返回草稿内容

## 验证

建议至少运行：

```bash
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
ruff check agi_walker/core/api/mcp_tools.py agi_walker/mcp/server.py tests/test_mcp_tools.py tests/test_mcp_server.py
```
