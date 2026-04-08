# Project Summary

更新日期：`2026-04-08`

本页描述 AGI-Walker 当前仓库的真实结构。它关注的是已经存在于代码和测试中的能力，而不是历史路线图或营销式功能列表。

## 项目定位

AGI-Walker 现在更像一套机器人工程工作台，而不是单一算法库。仓库把几类能力收拢到同一套入口下：

- skill metadata 与 skill 执行器
- workflow 编排与产物管理
- FastAPI Web Panel 与静态控制台页面
- Godot 连接层与会话桥接
- MCP `stdio` server
- 文档、smoke 和专项回归测试

主路径强调可组合与可诊断，而不是一次性依赖完整硬件环境。

## 一线入口

当前推荐优先记住三个入口：

- CLI：`python -m agi_walker.cli`
- Web：`python -m web_panel.server`
- MCP：`agi-walker-mcp`

对应的一线使用方式：

```bash
python -m agi_walker.cli skills list
python -m agi_walker.cli workflows list
python -m agi_walker.cli doctor
python -m web_panel.server
agi-walker-mcp
```

## 代码结构

### `agi_walker/cli`

命令行入口。当前重点能力是：

- `skills list/info/search/validate`
- `workflows list/run/validate`
- `doctor`

CLI 的 `workflows` 实际是 `skills workflows` 的别名，便于用户直接进入工作流面。

### `agi_walker/skills` 与 `skills_loader`

这里管理 `SKILL.md` 驱动的能力目录。当前仓库能稳定扫描出的 skills 包括：

- `robot-modeling`
- `parameter-optimizer`
- `urdf-generator`
- `model-distiller`

Skills 在当前项目里承担三层角色：

- 作为人类可读的能力索引
- 作为 workflow 的稳定执行单元
- 作为 CLI / Web / MCP 共享的元数据来源

### `agi_walker/workflow_orchestrator.py`

工作流编排器是主线模块之一。当前内置 workflow 有两个：

- `robot_creation_pipeline`
- `simulation_ready_robot`

它支持：

- `resume` / `force` 两种执行策略
- `output_root` 重定向
- 步骤变量引用，如 `{create_model.output_file}`
- state 持久化
- 真实执行与 mock 执行切换
- step artifact 写盘
- 基于 `TaskGraph` 的 DAG 执行入口

### `agi_walker/mcp`

MCP server 把仓库能力统一暴露给外部 agent / client。当前覆盖的重点是：

- workflow 列表、查询、执行
- skills 列表、查询
- telemetry 与 RAG 查询
- Godot agent 状态、模板、计划、诊断、历史

`agi_walker.mcp.server` 现在已经对齐当前 `mcp` 版本的初始化方式，并可通过 `stdio` 启动。

### `web_panel`

Web Panel 基于 FastAPI。它既是 API 服务，也是控制台页面宿主。当前主要承担：

- 系统状态展示
- workflow run 管理
- artifacts 下载与 Godot 同步
- Godot legacy controller
- Godot session bridge
- distributed / nightly 状态聚合
- WebSocket 会话通道

### `agi_walker/core/controllers`

这里包含控制与推理相关模块。它们的重要性并不完全相同：

- 一线可直接落地的能力：`ai_model.py`、`onnx_inference.py`、`predictive_safety.py`
- 主线路上可组合但依赖环境的能力：`ai_controller.py`、`model_orchestrator.py`
- 更偏实验/扩展方向的模块：`vla_adapter.py`、`rl_optimizer.py`、`rag_knowledge_base.py`

写文档时应把这些模块区分开，避免默认假设所有模块都已进入稳定生产路径。

## 端到端主路径

### 路径 1：机器人建模工作流

```text
robot-modeling -> parameter-optimizer -> urdf-generator
```

对应 workflow：`robot_creation_pipeline`

典型用途：

- 从模板生成机器人配置
- 优化质量分布
- 导出 URDF

### 路径 2：仿真就绪工作流

```text
load_config -> validate_physics -> export sdf
```

对应 workflow：`simulation_ready_robot`

典型用途：

- 读取现有配置
- 做物理校验
- 导出用于仿真的 SDF

### 路径 3：Web / Godot 联动

```text
workflow run -> artifact store -> web_panel -> godot sync / session bridge
```

这条路径适合人工操作、结果浏览和后续场景验证。

### 路径 4：Agent / MCP 接入

```text
external client -> MCP tools -> workflow / skills / godot backends
```

这条路径适合把 AGI-Walker 当作上层 agent 的能力提供方。

## 当前资产状态

仓库中 `weights/` 目录目前更接近工作流输出与示例资产目录，而不是一个完整模型仓库。现有内容主要是：

- `created_robot.json`
- `optimized_robot.json`
- `sim_robot.json`
- `validated_robot.json`
- `imc22_control_net.py`

因此，任何“模型动物园”类文档都应该按当前现实改写，而不是延续旧版的下载清单。

## 当前边界

当前仍应明确以下边界：

- 很多控制器模块依赖额外 Python 包或本地服务，默认环境未必全部可运行
- Web / Godot 的完整链路仍依赖本地端口、外部可执行程序和场景资源
- 并非所有历史文档都已完成清理
- 仓库存在研究型模块，但其中一部分仍是原型接口，不应写成“默认生产能力”

## 推荐阅读顺序

建议按下面顺序理解项目：

1. [README.md](../../README.md)
2. [CURRENT_STATUS.md](../CURRENT_STATUS.md)
3. [CLI_GUIDE.md](../guides/CLI_GUIDE.md)
4. [WEB_PANEL_GUIDE.md](../guides/WEB_PANEL_GUIDE.md)
5. [API_REFERENCE.md](../API_REFERENCE.md)
6. 本目录中的 `AI_SETUP_GUIDE.md`、`ADVANCED_FEATURES.md`、`MODEL_ZOO.md`

## 结论

AGI-Walker 当前最有价值的部分，不是某个单独的 AI 控制器，而是把 skills、workflow、Web、Godot 和 MCP 串成了一条可操作、可检查、可回归的工程主线。后续继续修文档时，也应围绕这条主线组织内容。
