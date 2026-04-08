# Model Zoo

更新日期：`2026-04-08`

本仓库当前不维护一个传统意义上的“预训练模型动物园”。旧文档中如果出现大批可下载模型、版本排行榜或预打包权重清单，应视为历史内容，不代表当前仓库状态。

本页改为说明 AGI-Walker 现在实际存在的模型相关资产与接入方式。

## 当前仓库内的模型相关文件

`weights/` 目录当前可以看到的内容主要是：

- `created_robot.json`
- `optimized_robot.json`
- `sim_robot.json`
- `validated_robot.json`
- `imc22_control_net.py`

这些文件更接近：

- 工作流输出物
- 控制脚本示例
- 仿真相关资产

它们不是统一标准下的可复用模型注册表。

## 当前支持的模型来源

### Ollama

仓库通过 `create_ai_model(engine="ollama", ...)` 使用本地 Ollama 模型。模型本体由你的本地 Ollama 服务管理，不随仓库分发。

典型模型名示例：

- `phi3:mini`
- `mistral:7b`
- `llama2:70b`

这些名称只是当前代码里的常见默认值或示例值，不代表仓库已经内置了对应权重。

### llama.cpp / GGUF

仓库支持通过本地 `model_path` 加载 GGUF 模型文件。模型文件需要你自行准备，例如：

```text
models/controller.gguf
```

### ONNX

仓库支持使用 `ONNXInferenceEngine` 加载 `.onnx` 文件。这类模型通常来自：

- 外部训练管线
- RL 导出流程
- 手工准备的部署模型

## 训练与导出路径

当前最明确的模型生成路径来自 `RLOptimizer`：

- 用 Stable-Baselines3 训练策略
- 评估策略
- 导出为 ONNX

因此，如果你想在 AGI-Walker 里形成一条相对闭环的模型链路，比较现实的做法是：

```text
train policy -> export ONNX -> load with ONNXInferenceEngine
```

## 工作流输出与模型的区别

请不要把 workflow 产物和模型权重混为一谈。

当前 workflow 生成的主要是：

- 机器人配置 JSON
- 导出的 URDF / SDF
- 步骤 artifact JSON

这些产物描述的是结构、参数和导出结果，而不是神经网络权重。

## 如果要新增模型资产

建议遵守以下原则：

- 大文件不要直接无约束提交到仓库
- 在文档里写清楚模型来源、格式和消费者模块
- 为 ONNX / GGUF / Ollama 分开写加载说明
- 给出最小可运行示例
- 说明是否需要额外依赖或 GPU

建议至少记录这些字段：

- 模型名称
- 来源或训练方式
- 文件格式
- 推荐路径
- 负责加载的 Python 模块
- 输入输出约定

## 当前不再承诺的内容

截至 `2026-04-08`，仓库没有正式维护以下内容：

- 统一下载中心
- 多版本 benchmark 排行
- 自动同步的 release 权重库
- 全量预训练策略清单
- 细分任务的官方最佳模型推荐

## 推荐用法

### 想快速试用

优先用 Ollama：

```bash
ollama pull phi3:mini
```

### 想做本地部署

优先用 ONNX：

- 训练或准备 `.onnx`
- 用 `ONNXInferenceEngine` 加载

### 想做完全离线文件分发

优先用 llama.cpp / GGUF：

- 自备 `.gguf`
- 用 `create_ai_model(engine="llamacpp", model_path=...)`

## 结论

AGI-Walker 当前没有“模型动物园”，只有“模型接入能力”。最准确的理解方式是：

- 模型可以来自 Ollama
- 模型可以来自 GGUF
- 模型可以来自 ONNX
- 仓库里的 `weights/` 主要是工作流和控制相关资产，不是正式模型目录
