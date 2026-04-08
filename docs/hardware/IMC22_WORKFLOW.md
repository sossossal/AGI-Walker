# IMC22 Workflow

更新日期：`2026-04-08`

本页整理仓库中与 IMC-22 相关的当前工作流。需要先说明：这条链已经有多个真实文件，但仍然是“参考工具链”，不是官方一键生产流水线。

## 1. 当前相关文件

### 模型定义

- `weights/imc22_control_net.py`

### 数据准备

- `tools/imc22_data_preparer.py`

### 量化与导出

- `tools/imc22_quantizer.py`

### 示例脚本

- `examples/train_imc22_model.py`
- `examples/deploy_to_hardware.py`

### 参考硬件资料

- `hardware/hive-reflex/`
- `hardware/imc22-rtl/`

## 2. 推荐理解方式

当前 IMC-22 工作流可拆成四段：

1. 准备简化数据集
2. 训练轻量控制网络
3. 做 INT8 量化与 C 导出
4. 接到参考 firmware / SDK / RTL

## 3. 数据准备

`tools/imc22_data_preparer.py` 当前会：

- 读取 `episode_*.pkl`
- 简化状态为 3 维
- 简化动作为 3 维
- 做归一化
- 生成 train / val / test

当前简化状态约为：

- position
- velocity
- stable

当前简化动作约为：

- motor_power
- stiffness
- damping

## 4. 控制网络

`weights/imc22_control_net.py` 当前定义了一个很小的 MLP：

- 输入 3 维
- 两层隐藏层
- 输出 3 维

这个模型的定位是：

- 小参数量
- INT8 友好
- 面向 NPU / 边缘控制实验

## 5. 量化与 C 导出

`tools/imc22_quantizer.py` 当前支持：

- 动态量化
- 静态量化
- 提取 INT8 权重
- 导出 C header
- 导出 C source

这条路径是当前 IMC-22 工作流里最清晰的一段。

## 6. 当前需要特别注意的地方

截至当前仓库状态，训练和量化脚本仍有一部分历史路径漂移：

- `examples/train_imc22_model.py` 和 `tools/imc22_quantizer.py` 使用了 `models.imc22_control_net`
- 真实模型文件当前在 `weights/imc22_control_net.py`

这意味着：

- 这条链是“可参考”
- 但在实际运行前，你需要先修正导入路径或本地工程布局

## 7. Hive-Reflex 与 RTL 的位置

### Hive-Reflex

目录：

- `hardware/hive-reflex/`

内容：

- C 控制代码
- 仿真器
- 训练模板
- IMC22 SDK 头文件

### IMC22 RTL

目录：

- `hardware/imc22-rtl/`

内容：

- NPU Verilog
- reflex 相关 Verilog
- system 顶层

这两部分都更偏参考实现和实验工程。

## 8. 一个务实的工作顺序

1. 先准备简化数据
2. 确认模型脚本路径
3. 训练小模型
4. 跑量化导出
5. 再决定接 Hive-Reflex 还是板级工程

## 9. 不建议的做法

- 直接把示例脚本当成无改动可用的生产流水线
- 不核对导入路径就开始训练
- 不做 mock / 导出验证就直接上硬件

## 结论

AGI-Walker 当前已经有一条看得见的 IMC-22 参考工作流，但它仍处于工程骨架阶段。最稳妥的方式是先把数据准备、模型定义和量化导出三段跑顺，再进入 Hive-Reflex、RTL 或真实板级集成。
