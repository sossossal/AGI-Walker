# Compile And Optimize

更新日期：`2026-04-08`

这个页面名称沿用了历史叫法，但对当前仓库来说，“compile” 不应理解为必须先做一轮大型原生编译。AGI-Walker 现在是 Python-first 项目，绝大多数场景优先做的是环境准备、运行时选择和部署优化。

## 1. 先不要编译

默认建议顺序：

1. 创建干净 Python 环境
2. 安装依赖
3. 运行 `doctor`
4. 运行 smoke / workflow
5. 只有在需要时再接入 ONNX、llama.cpp、Docker 或分布式运行时

最小入口：

```bash
pip install -r requirements.txt
python -m agi_walker.cli doctor
python tests/run_smoke_tests.py
```

## 2. 推理优化优先级

当前仓库最现实的性能优化顺序是：

- 先保证依赖完整
- 再选择更合适的推理后端
- 最后才考虑底层编译或镜像构建

### Ollama

优点：

- 上手快
- 适合本地调试
- 适合控制链路原型验证

### ONNX Runtime

优点：

- 更接近部署态
- `ONNXInferenceEngine` 更适合低延迟推理
- 可以衔接 RL 导出流程

### llama.cpp

优点：

- 本地文件模型管理清晰
- 完全离线

代价：

- 对本机编译环境和模型格式更敏感

## 3. Docker 构建路径

仓库当前提供了多种 Docker 入口：

- `deployment/Dockerfile`
- `deployment/Dockerfile.web_panel`
- `deployment/Dockerfile.distributed_runtime`
- `deployment/docker-compose.yml`

常见场景：

- 单独构建 Web Panel
- 构建 distributed runtime
- 跑 distributed smoke

如果只是本地开发文档、CLI 或 workflow，不要先上 Docker。

## 4. Web Panel 优化

Web workflow 控制面可通过环境变量调整分页和归档保留：

- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`

环境文件可通过：

- `AGI_WALKER_WEB_ENV_FILE`

指向 `deployment/web_panel.env` 或自定义文件。

## 5. Distributed 优化

分布式模式的实际优化点主要在：

- Zenoh endpoint 选择
- actor TTL
- sidecar 与 learner 的网络稳定性

当前相关变量：

- `AGI_WALKER_ZENOH_ENDPOINT`
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`
- `ZENOH_ROUTER`
- `AGI_WALKER_SIDECAR_ACTOR_ID`

## 6. 不建议继续沿用的旧思路

以下做法不再适合作为主线：

- 把所有功能都写成“先编译再运行”
- 假设仓库自带完整模型权重
- 把 GUI 工具视为主部署面
- 把 Godot 或分布式路径写成零配置默认可用

## 7. 一条务实的优化路线

推荐路线：

1. `python -m agi_walker.cli doctor`
2. `python tests/run_smoke_tests.py`
3. workflow 使用独立 `--output-root`
4. 推理侧优先尝试 ONNX
5. 需要隔离部署时再用 Docker
6. 只有确实需要时再进入 distributed 和 headless smoke

## 结论

当前仓库的“优化”重点是运行时与工程路径优化，而不是统一原生编译。优先把 CLI、workflow、Web 和推理后端选型跑顺，比追求一个不存在的总编译目标更重要。
