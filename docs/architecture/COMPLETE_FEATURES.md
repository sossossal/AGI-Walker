# Complete Features

更新日期：`2026-04-08`

本页汇总 AGI-Walker 当前仓库里可见、可运行、可追踪的功能面。它不是路线图；未在本页列出的能力不应默认视为稳定支持。

## 核心入口

当前一线入口：

- `python -m agi_walker.cli`
- `python -m web_panel.server`
- `agi-walker-mcp`

辅助入口：

- `python -m agi_walker.mcp.server`
- `python tests/run_smoke_tests.py`

## CLI 能力

### Skills 管理

- 列出 skills：`skills list`
- 查看详情：`skills info <name>`
- 搜索 skills：`skills search <query>`
- 列出分类：`skills categories`
- 验证 skills：`skills validate`

### Workflow 管理

- 列出 workflow：`workflows list`
- 校验 workflow：`workflows validate <name>`
- 执行 workflow：`workflows run <name>`
- 支持 `--mock`
- 支持 `--resume` / `--force`
- 支持 `--output-root`

### 环境诊断

- `doctor` 会检查依赖、端口和关键路径可用性

## Skills 系统

当前仓库有一条可用的 skill metadata 体系：

- 通过 `SKILL.md` 定义描述、分类和依赖
- 通过 `SkillsLoader` 扫描、检索和读取文档
- 通过 skill executor 执行动作

当前可见的重点 skills：

- `robot-modeling`
- `parameter-optimizer`
- `urdf-generator`
- `model-distiller`

## Workflow 编排

### 内置 workflow

- `robot_creation_pipeline`
- `simulation_ready_robot`

### 已实现的编排能力

- 步骤顺序执行
- 参数注入
- 结果变量引用
- `resume` / `force` 执行策略
- `output_root` 输出重定向
- `WorkflowStateStore` 持久化
- real / mock executor 切换
- artifact JSON 写盘
- progress callback
- `TaskGraph` DAG 执行入口

## MCP 能力

当前 MCP server 已暴露以下工具：

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

MCP 当前主要服务于两类场景：

- 让外部 agent 用结构化接口调用 workflow / skills / Godot 能力
- 用统一工具面替代直接操作 CLI 或 HTTP API

## Web Panel

### 系统与状态接口

- `/api/system/status`
- `/api/godot/capabilities`
- `/api/distributed/status`
- `/api/nightly/regressions`
- `/metrics`
- `/openapi.json`

### 任务与技能接口

- `/api/tasks`
- `/api/generate_robot`
- `/api/skills/list`
- `/api/skills/model`
- `/api/skills/optimize`
- `/api/skills/export-urdf`
- `/api/skills/pipeline`

### Workflow 接口

- `/api/workflows/`
- `/api/workflows/{name}`
- `/api/workflows/{name}/run`
- `/api/workflows/runs/*`
- `/api/workflows/logs/*`

### Godot 与仿真接口

- `/api/godot/*` legacy controller
- `/api/godot-agent/*` Godot agent
- `/api/godot/{session_id}/*` session bridge
- `/api/simulation/status`
- `WS /ws/{session_id}`

### 静态控制台页面

- `/static/index.html`
- `/static/workflows.html`
- `/static/nightly.html`
- `/static/distributed.html`
- `/static/design.html`
- `/static/godot-control.html`

## Godot 集成

当前同时保留两条集成路径：

- legacy controller API
- session bridge / telemetry bridge

与 workflow 的结合点包括：

- workflow artifacts 装载
- run 到 Godot 的同步接口
- WebSocket 广播与会话隔离

## AI 与控制栈

### 已有推理后端

- Ollama，本地模型名可配置，默认常见示例是 `phi3:mini`
- llama.cpp，本地 `model_path` 驱动
- ONNX Runtime，通过 `ONNXInferenceEngine`

### 已有控制与安全模块

- `AIController`
- `SafetyChecker`
- `PredictiveSafetyChecker`
- `SystemMonitor`
- `ModelOrchestrator`

### 已有知识与扩展模块

- `PhysicsKnowledgeBase`
- `VLAAdapter`
- `RLOptimizer`
- `MediumModel`

其中需要额外说明：

- `ai_model.py`、`onnx_inference.py` 比较适合作为当前真实依赖面来写
- `vla_adapter.py`、`rl_optimizer.py` 更适合写成“扩展模块”或“实验性能力”

## 模型与产物

当前仓库里的模型相关资产分为三类：

- 工作流输出 JSON，如 `created_robot.json`、`optimized_robot.json`
- Python 控制网络脚本，如 `imc22_control_net.py`
- 外部运行时模型，由 Ollama、llama.cpp 或 ONNX 文件提供

仓库当前没有维护一个完整的、可下载的内置模型动物园。

## 质量与回归

当前可见的质量守护主要包括：

- `tests/test_docs_utf8.py`
- `tests/test_mcp_tools.py`
- `tests/test_mcp_server.py`
- `tests/test_workflow_orchestrator.py`
- `tests/test_integration_workflows.py`
- `tests/run_smoke_tests.py`

## 当前不应夸大的部分

下面这些方向虽然代码里有痕迹，但不应在对外文档里写成“默认即开即用”：

- 完整硬件控制闭环
- 预训练模型下载中心
- 零配置的 VLA / 多模态推理
- 无依赖的 RL 训练环境
- 不需要外部 Godot 或本地端口的完整仿真链路

## 结论

AGI-Walker 当前的“完整功能”应理解为一套工程主线：

- skill metadata
- workflow orchestration
- Web Panel
- Godot bridge
- MCP integration
- docs and tests

这条主线已经足够支撑本地验证、代理接入和后续文档收敛。
