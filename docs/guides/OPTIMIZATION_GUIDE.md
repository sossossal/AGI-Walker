# Optimization Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库里真正值得优先做的优化。重点是减少排错成本、缩短反馈回路和选择更合适的运行时，而不是追求抽象意义上的“全局最优”。

## 1. 先优化反馈回路

最先该优化的是开发和验证路径：

- 给 workflow 指定独立 `--output-root`
- 用 `--resume` 重用已有产物
- 用 `--mock` 先验证编排路径
- 先跑 smoke，再跑重型集成

示例：

```bash
python -m agi_walker.cli workflows run robot_creation_pipeline --mock --resume --output-root test_env/opt_run
```

## 2. 推理后端选择

当前三类主要后端：

- Ollama
- llama.cpp
- ONNX Runtime

推荐选择顺序：

- 原型验证优先 Ollama
- 部署和延迟优先 ONNX
- 完全离线文件模型再考虑 llama.cpp

## 3. Workflow 优化

当前 workflow 最重要的优化点不是并发，而是可恢复性和可追踪性：

- 使用 `resume`
- 使用显式 `output_root`
- 保留 artifacts
- 通过 Web run 记录和 live log 观察执行过程

## 4. Web 控制面优化

如果 Web 上 run 数量很多，可调整：

- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`

如果你已经有环境文件，建议统一放进：

- `deployment/web_panel.env`

或通过：

- `AGI_WALKER_WEB_ENV_FILE`

指定。

## 5. 安全与稳定性优化

控制链路当前已有两类安全模块：

- `SafetyChecker`
- `PredictiveSafetyChecker`

如果你在调控制器，不要只盯吞吐量；优先确认：

- 动作是否被安全层裁剪
- 负载过高时是否进入降级模式
- 失败时是否有 fallback

## 6. Distributed 优化不是第一步

distributed 模式适合做专项优化，但不该成为日常第一选择。只有在下面场景才值得优先进入：

- 需要验证 learner / sidecar / Zenoh 路径
- 需要夜间集成或 Docker smoke

## 7. 文档与入口优化

当前仓库一个非常现实的优化方向是“降低误导”：

- 文档只写真实入口
- 不再引用旧 `python_api/`
- 不把实验模块写成默认能力

这类优化对团队效率的价值，往往高于继续堆功能描述。

## 8. 推荐顺序

1. 优化 CLI / workflow 反馈回路
2. 优化推理后端选型
3. 优化 Web 运行记录与归档
4. 最后再优化分布式和真实 Godot 链路

## 结论

AGI-Walker 当前最有效的优化，是让主线更快、更稳、更容易排错。优先优化 workflow、Web 和推理后端选择，比追求复杂部署模式更划算。
