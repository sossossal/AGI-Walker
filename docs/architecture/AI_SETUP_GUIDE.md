# AI Setup Guide

更新日期：`2026-04-08`

本页说明 AGI-Walker 当前仓库里的 AI / 推理相关模块应如何准备环境。这里重点覆盖已经在代码里明确存在的三类后端：

- Ollama
- llama.cpp
- ONNX Runtime

默认建议从最轻量的 Ollama 或 ONNX 开始。

## 1. 基础准备

先创建并激活 Python 环境，然后安装项目依赖：

```bash
pip install -r requirements.txt
```

如果只想验证仓库主路径，先跑：

```bash
python -m agi_walker.cli doctor
```

注意：控制与推理模块大多不是主 CLI 的默认命令面，它们更适合通过 Python 导入、workflow、Web 或 MCP 集成来使用。

## 2. Ollama 后端

### 适用模块

- `agi_walker/core/controllers/ai_model.py`
- `agi_walker/core/controllers/medium_model.py`
- `agi_walker/core/controllers/model_orchestrator.py`

### 安装步骤

1. 安装 Ollama 本体并启动本地服务
2. 安装 Python SDK

```bash
pip install ollama
```

3. 拉取至少一个本地模型

```bash
ollama pull phi3:mini
```

如果你要尝试分层模型编排，还可以额外准备：

```bash
ollama pull mistral:7b
ollama pull llama2:70b
```

### 最小验证

```python
from agi_walker.core.controllers.ai_model import create_ai_model

model = create_ai_model(engine="ollama", model_name="phi3:mini")
print(model.get_stats())
```

### 与 `AIController` 的关系

`AIController` 的初始化逻辑是：

- 当 `backend == "onnx"` 且提供了 `model_path` 时，使用 ONNX
- 否则回退到 `create_ai_model(engine="ollama")`

这意味着如果你没有准备 ONNX 模型文件，最现实的第一选择就是先把 Ollama 跑通。

## 3. llama.cpp 后端

### 适用模块

- `agi_walker/core/controllers/ai_model.py`

### 安装步骤

```bash
pip install llama-cpp-python
```

然后准备一个本地 GGUF 模型文件，例如：

```text
models/my-controller.gguf
```

### 最小验证

```python
from agi_walker.core.controllers.ai_model import create_ai_model

model = create_ai_model(
    engine="llamacpp",
    model_path="models/my-controller.gguf",
)
print(model.get_stats())
```

如果需要 JSON 语法约束，还可以传 `grammar_path`。

### 何时选择 llama.cpp

优先在下面场景考虑：

- 需要完全本地离线推理
- 你已经有现成 GGUF 文件
- 不希望把模型管理交给 Ollama

## 4. ONNX Runtime 后端

### 适用模块

- `agi_walker/core/controllers/onnx_inference.py`
- `agi_walker/core/controllers/ai_controller.py`
- `agi_walker/core/controllers/rl_optimizer.py` 导出后的部署链路

### 安装步骤

CPU 版：

```bash
pip install onnxruntime
```

如果你明确需要 GPU provider，请按本机环境安装对应版本。

### 最小验证

```python
from agi_walker.core.controllers.onnx_inference import ONNXInferenceEngine

engine = ONNXInferenceEngine("models/policy.onnx")
print(engine.benchmark(iterations=10))
```

### 接入 `AIController`

```python
from agi_walker.core.controllers.ai_controller import AIController

controller = AIController(
    model_path="models/policy.onnx",
    backend="onnx",
)
```

如果 ONNX 文件不存在或加载失败，控制链路不会自动帮你生成模型文件，所以这一步通常放在训练或导出之后。

## 5. 分层模型编排

`model_orchestrator.py` 用于 small / medium / large 三层模型的路由。它比较适合：

- 想区分实时控制与离线优化
- 想把日志过滤和环境调整交给独立模型
- 想测量 tier usage、fallback 和延迟

最低准备通常是：

- `phi3:mini`
- `mistral:7b`

大模型层不是强依赖；加载失败时它会被禁用。

## 6. RAG 与知识增强

`PhysicsKnowledgeBase` 不需要大型外部服务，但会读写运行时索引目录。它适合：

- 给 prompt 增加物理背景
- 从历史轨迹中找相似经验

这部分更像辅助模块，而不是控制主链路的必选项。

## 7. RL 训练与 ONNX 导出

如果你要走训练路线，需要额外安装：

```bash
pip install stable-baselines3 gymnasium torch
```

然后可以通过 `RLOptimizer`：

- 训练 PPO / SAC / TD3 / A2C
- 评估策略
- 导出 ONNX

导出的 ONNX 文件可以再交给 `ONNXInferenceEngine` 使用。

## 8. 常见问题

### 无法连接 Ollama

常见原因：

- Ollama 服务未启动
- 模型未拉取
- Python 环境里未安装 `ollama`

### llama.cpp 无法加载模型

常见原因：

- `model_path` 不存在
- `llama-cpp-python` 未安装
- 模型格式与当前构建不兼容

### ONNX 模型无法加载

常见原因：

- 路径错误
- 模型输入输出不符合控制器预期
- 缺少对应 runtime

### 某些控制器模块看起来存在但跑不通

这是正常的。仓库里同时包含：

- 主线路模块
- 依赖外部服务的模块
- 研究性质原型

文档应以“当前稳定入口”优先，不应要求所有模块在裸环境下都可运行。

## 9. 推荐起步顺序

建议按下面顺序准备：

1. `python -m agi_walker.cli doctor`
2. Ollama + `phi3:mini`
3. ONNX Runtime
4. llama.cpp
5. RL 训练与导出

## 结论

对当前仓库而言，最务实的 AI 环境策略是：

- 默认先跑通 Ollama
- 有现成部署模型时接入 ONNX
- 需要完全本地文件模型时再用 llama.cpp
- 把 RL、VLA、多层编排留给后续扩展
