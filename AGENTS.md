# AGI-Walker Codex 工作规范

本文件是本项目 Codex 编程任务的默认工作规范。除非用户在当前任务中明确覆盖，后续所有编程、重构、测试、CI、发布和文档变更都必须遵循：

- `$codex-project-plan`
- `$closure-first-engineer`
- `$efficient-engineer`
- `$enterprise-code-acceptance`

## 任务启动

每次开始任务前，Codex 必须先读取并理解：

- `PROJECT_PLAN.md`
- 相关模块子计划：`plans/modules/<module>.md`
- 与任务直接相关的源码、测试、配置、文档和现有 artifact 契约

如果 `PROJECT_PLAN.md` 或相关模块子计划尚不存在，必须先说明缺失状态。大项目或跨模块任务应先创建或更新这些计划文件，再实施代码变更。

## 计划管理

大项目计划必须写入 `PROJECT_PLAN.md`，并作为跨模块目标、范围、接口、依赖、里程碑、风险、验证和验收标准的单一事实来源。

模块任务必须写入 `plans/modules/<module>.md`。模块子计划只记录模块内的任务、文件责任、局部决策、测试命令、阻塞项、进展、临时发现和局部 TODO。

模块内进展、临时发现和局部 TODO 不得污染 `PROJECT_PLAN.md`。只有跨模块目标、范围、契约、依赖、验收标准、重大风险或交付策略变化才应进入 `PROJECT_PLAN.md`。

跨模块契约、范围、验收标准变化必须记录到 `PROJECT_PLAN.md` 的 `Change Control`。影响架构或交付策略的决定还应记录到 `Decision Log`。

## 实现前要求

实现前必须明确：

- 变更目标和非目标
- 受影响模块及所有权边界
- 公共 API、schema、CLI flag、配置、文件格式、路由、事件或数据契约
- 向后兼容、迁移、回滚或弃用策略
- 验证方式，包括单元测试、契约测试、集成测试、CI gate、静态检查、live/manual smoke 或文档检查

不得在契约不清、模块边界不清或验证方式不清的情况下直接扩大实现范围。

## 实现原则

代码应保持最小、清晰、局部化：

- 优先复用现有模式、工具和契约。
- 避免无关重构、格式 churn、临时文件和无用依赖。
- 输入在信任边界处验证，错误处理覆盖预期失败模式。
- 共享字段名应在后端模型、API payload、CLI 输出、报告、Portal 状态和测试中保持一致。
- 对跨层概念先定义 canonical contract，再接入调用方、报告、文档和测试。

任何重复逻辑、冗余变量、深层嵌套、隐藏全局状态、未解释复杂度或无法验证的实现，都应视为需要修正的问题。

## 实现后要求

实现后必须补齐与风险相称的测试，并更新必要文档。公共契约、数据结构、CLI、CI、报告、用户可见行为或操作流程变化必须有对应文档或计划更新。

验证应尽量按两层执行：

1. 目标测试：覆盖本次变更的最小相关测试。
2. 回归验证：运行项目中最强、可行且相关的非 live 验证命令。

如果 live、浏览器、硬件、外部服务或完整 CI 无法运行，必须在最终输出中说明未运行原因和残余风险。

## 企业级验收

Codex 必须按 `$enterprise-code-acceptance` 判断变更是否可接受。至少检查：

- Requirements and scope
- Architecture and contracts
- Code quality
- Security and compliance
- Data, migration, and compatibility
- Testing and verification
- Operations and release readiness
- Documentation and handoff

存在以下情况时不得给出无条件接受结论：

- 必要测试或构建失败。
- 行为变更没有验证证据。
- 公共契约未文档化或未测试。
- 安全、权限、隐私、数据丢失或迁移风险未解决。
- 范围扩张未进入计划或未获用户确认。
- 实现依赖隐藏假设、手工状态或未提交生成物。

## 最终输出

每次编程任务最终输出必须包含：

- 变更摘要
- 关键文件
- 验证证据，包括命令和结果摘要
- 未运行的检查及原因
- 残余风险
- 企业级验收结论：`Accepted`、`Accepted with documented risk` 或 `Blocked`

如果任务只是规划或文档变更，也必须说明验证方式和残余风险。
