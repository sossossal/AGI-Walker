# Advanced Features

更新日期：`2026-04-08`

本页聚焦 AGI-Walker 中相对高级的能力。这里的“高级”指的是跨模块编排、依赖额外运行时，或者适合在扩展场景中使用的功能；不等同于默认已经全部稳定量产。

## Artifact-Aware Workflow 执行

`WorkflowOrchestrator` 不只是按顺序跑步骤，它还带了较完整的执行上下文能力：

- 输出文件路径规范化
- `output_root` 隔离输出
- `resume` 模式下遇到已有产物自动跳过
- `force` 模式下强制重跑
- 步骤变量引用
- 步骤 artifact JSON 写盘
- 工作流状态持久化

这让 workflow 可以被反复运行、恢复和检查，而不只是一次性脚本。

## TaskGraph / DAG 执行入口

除了线性 workflow，编排器还支持 `execute_task_graph(...)`：

- 按 `TaskGraph` 解析可运行节点
- 允许并发执行可独立节点
- 将节点状态映射到统一的 workflow result
- 为 Web / agent 场景保留 DAG 级扩展空间

这部分能力已经在代码里落地，但仍更适合作为高级集成入口，而不是普通用户的第一使用路径。

## 多后端推理栈

### Ollama

`ai_model.py` 和 `medium_model.py` 都支持 Ollama。本地服务可提供：

- 小模型实时控制
- 中模型环境调整
- 大模型离线优化入口

### llama.cpp

`create_ai_model(engine="llamacpp", model_path=...)` 支持本地 GGUF 模型与 `llama-cpp-python`。

### ONNX Runtime

`ONNXInferenceEngine` 支持：

- CPU / 可选 GPU provider
- 低延迟同步推理
- benchmark

这条路径更接近边缘推理或训练后部署的现实使用方式。

## 分层模型编排

`model_orchestrator.py` 内置了 small / medium / large 三层模型调度概念：

- `SMALL` 处理实时控制
- `MEDIUM` 处理环境调整
- `LARGE` 处理离线优化

它还跟踪：

- tier usage
- fallback 次数
- 延迟阈值
- 升级与回调

这更像控制系统的策略路由骨架，而不是一个单独模型类。

## 安全与降级控制

AGI-Walker 当前有两层安全思路：

### 规则式安全

- `SafetyChecker` 负责关节限位和速度限制
- `SystemMonitor` 结合 `AIController` 进入降级模式

### 预测式安全

- `PredictiveSafetyChecker` 使用简化物理模型做短时轨迹预测
- 风险分级为 `SAFE` 到 `CRITICAL`
- 检测高风险后自动缩放动作输出

这类模块对“先保命，再谈智能”非常重要，因此适合在架构文档中单独说明。

## RAG 物理知识库

`rag_knowledge_base.py` 当前实现了一条轻量 RAG 路线：

- JSON 索引持久化
- 内置物理知识条目
- 历史轨迹扫描
- 基于姿态的近邻经验检索
- prompt augment

它的现实作用不是“通用知识库平台”，而是给控制或规划提示补一层轻量物理上下文。

## VLA TaskGraph 注入

`vla_adapter.py` 提供 Vision-Language-Action 适配器原型：

- 输入图像和文本指令
- 返回建议节点
- 动态注入 `TaskGraph`

当前更适合视为原型接口。它说明仓库为多模态规划预留了插槽，但不应写成默认稳定依赖。

## RL 到 ONNX 的部署链路

`rl_optimizer.py` 提供一条比较清晰的研究/部署转换路径：

- 用 Stable-Baselines3 训练 PPO / SAC / TD3 / A2C
- 评估策略
- 导出策略为 ONNX
- 再交给 `ONNXInferenceEngine` 做推理

这条路径的价值在于把训练态和部署态连接起来，但它依赖额外训练环境与包。

## Web + Godot Session Bridge

Web 层的高级能力不只是 REST API。当前还包括：

- WebSocket 会话通道
- `GodotSessionManager`
- telemetry loop
- session 级广播
- workflow artifacts 到 Godot 的同步桥接

这部分是 Web、workflow 和 Godot 三者之间最重要的连接层。

## MCP 统一能力面

MCP 层的高级价值不是“多一个启动命令”，而是把分散在仓库内部的能力统一成工具集合：

- workflow execution
- skills catalog
- telemetry / RAG
- Godot agent planning
- doctor / history

这让 AGI-Walker 可以被上层 agent 编排，而不要求调用方理解内部 Python 模块。

## 使用建议

如果目标是稳定落地，优先顺序建议如下：

1. 先使用 CLI 和 workflow
2. 再接入 Web Panel
3. 再接入 MCP
4. 最后再尝试 VLA、RL、复杂 Godot 场景或多层模型编排

## 结论

AGI-Walker 的高级能力主要体现在“跨模块衔接”而不是某个单点特性：

- workflow 与 artifact
- 模型后端与安全控制
- RAG 与任务规划
- Web 与 Godot 会话桥接
- MCP 统一工具面

这些能力共同构成了项目的扩展上限，但文档应始终注明它们各自的稳定度和环境依赖。
