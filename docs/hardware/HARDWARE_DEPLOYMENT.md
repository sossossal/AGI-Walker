# Hardware Deployment

更新日期：`2026-04-12`

本页说明 AGI-Walker 当前仓库下“从策略到硬件”的现实部署路径。当前部署更接近一条参考流程，而不是完全收敛的一键流水线。

## 1. 当前部署层次

可以把当前部署理解为四层：

1. Python / mock driver 验证
2. 仿真训练或控制验证
3. 模型导出与量化
4. 真实硬件接口接入

## 2. 最务实的起点

在进入真实硬件前，先确认这几步：

```bash
python -m agi_walker.cli doctor
python tests/run_smoke_tests.py
python -m pytest tests/test_mcp_tools.py tests/test_mcp_server.py -q
```

这样能先排除文档、主入口和依赖层的问题。

## 3. 参考部署脚本

文件：

- `examples/deploy_to_hardware.py`

它试图覆盖：

- 在仿真中训练策略
- 导出 ONNX
- INT8 量化
- 调用 `HardwareEnvironment` 做硬件测试

但当前应把它视为参考脚本，因为：

- 它依赖较重
- 其中一部分模型和路径假设仍偏历史
- 并不保证在裸环境下一步到位

## 4. 一个更稳的部署顺序

### 第一步：Mock 验证

先用：

- `RealRobotDriver(mock=True)`
- `RealRobotDriver.from_replay(...)`
- `collect_sysid_data.py --mock`

确认：

- 命令结构正确
- 数据日志能写出
- 上层流程不依赖真实硬件
- 默认 pytest 不访问真实串口

### 第二步：仿真侧策略或控制验证

如果你在做策略部署，先在仿真里证明：

- 策略可推理
- 输出维度正确
- 观测结构与部署侧一致

### 第三步：模型导出与量化

当前相关文件：

- `weights/imc22_control_net.py`
- `tools/imc22_quantizer.py`

导出链覆盖：

- INT8 权重
- C 头文件
- C 推理代码

### 第四步：真实接口接入

再根据硬件类型进入：

- 串口 `RealRobotDriver`
- CAN `IMC22Controller`
- Hive-Reflex / IMC22 参考层

## 5. 部署时最常见的分叉

### Python 驱动部署

适合：

- PC 直连
- 串口或 CAN 控制
- 小规模验证

### IMC-22 参考链部署

适合：

- NPU / 边缘控制实验
- 参考 C 代码和量化模型落地

### ESP32 / Zenoh 原型部署

适合：

- 神经元节点原型
- 分布式通信实验

## 6. 当前风险点

- 训练脚本与模型文件路径仍有历史漂移
- 量化脚本和导出脚本不一定能无改动直接接到你的板级工程
- 真实硬件测试依赖串口、CAN、供电和节点发现

## 7. 建议的验证闭环

当前最推荐的闭环是：

```text
mock/replay driver -> sysid collection -> simulation validation -> export/quantize -> real interface
```

而不是：

```text
直接上板 -> 现场排错
```

## 8. 与 Hive-Reflex 的关系

`hardware/hive-reflex/` 目录更像独立参考项目：

- 带自己的 README
- 带自己的 SDK / Makefile / 仿真器

它可以作为后续部署参考，但不应被视为当前 Python 主线的自动下一步。

## 结论

AGI-Walker 当前的硬件部署是一条分层参考流程。先守住 mock、日志、导出和接口骨架，再进入真实板级或 IMC-22 实验，会比追求一步到位更可靠。
