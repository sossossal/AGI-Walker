# Hardware Spec

更新日期：`2026-04-08`

本页描述 AGI-Walker 当前仓库里与硬件相关的真实资产。它不是一份“官方量产规格书”，而是一份帮助你理解仓库目前支持到哪一层的索引。

## 1. 当前硬件相关目录

主要目录包括：

- `hardware/firmware/`
- `hardware/hive-reflex/`
- `hardware/imc22-rtl/`
- `hardware/ros2_ws/`
- `parts_library/`
- `agi_walker/core/drivers/`
- `agi_walker/core/api/godot_robot_env/hardware_controller.py`

## 2. 当前主线硬件资产

### Parts Library

当前最稳定、最结构化的硬件数据源是：

- `parts_library/complete_parts_database.json`

它包含：

- 零件分类
- 零件参数
- 完整套件
- 低成本套件

当前分类包括：

- `motors`
- `motors_diy`
- `sensors`
- `controllers`
- `joints`
- `structure`
- `power`
- `communication`
- `accessories`

### 参考 BOM

仓库内已有一份 BOM 文本：

- `docs/hardware/BOM_biped.txt`

它来自 parts library 的生成结果，更适合作为参考材料，而不是正式采购清单。

## 3. 当前控制接口

### 串口/Mock 驱动

文件：

- `agi_walker/core/drivers/real_robot_driver.py`

作用：

- 提供串口硬件驱动骨架
- 支持 `mock=True`
- 支持电机命令发送
- 支持基础状态读取

### CAN / IMC-22 控制器

文件：

- `agi_walker/core/api/godot_robot_env/hardware_controller.py`

作用：

- 通过 `python-can` 建立 CAN 总线
- 提供 IMC-22 风格节点发现和命令发送
- 提供 `HardwareEnvironment` 包装

## 4. IMC-22 相关资产

当前仓库里与 IMC-22 相关的内容分散在几处：

- `weights/imc22_control_net.py`
- `tools/imc22_data_preparer.py`
- `tools/imc22_quantizer.py`
- `examples/train_imc22_model.py`
- `examples/deploy_to_hardware.py`
- `hardware/hive-reflex/`
- `hardware/imc22-rtl/`

这说明仓库覆盖了：

- 轻量网络定义
- 数据准备
- INT8 量化
- C 头文件 / 推理代码导出
- firmware / SDK / RTL 参考

但这仍然更接近参考工具链，而不是一键量产流程。

## 5. 参考硬件方向

### Hive-Reflex

目录：

- `hardware/hive-reflex/`

包含：

- C 控制代码
- 仿真器
- 训练脚本
- SDK 头文件
- Windows / WSL 指南

### IMC22 RTL

目录：

- `hardware/imc22-rtl/`

包含：

- NPU RTL
- reflex 相关 RTL
- system 级 Verilog

### ESP32 Neuron Firmware

目录：

- `hardware/firmware/esp32_neuron/`

定位：

- Zenoh-Pico + Servo 的参考固件

## 6. ROS 2 Workspace

仓库还带了一个 ROS 2 workspace：

- `hardware/ros2_ws/`

它更适合作为独立桥接与生态集成方向，而不是当前硬件主线的默认依赖。

## 7. 当前不应默认承诺的内容

截至当前仓库状态，不应默认承诺：

- 官方认证硬件清单
- 已验证的量产电气规格
- 一键可跑的 IMC-22 完整烧录链
- 零配置真实机器人部署

## 8. 建议理解方式

当前硬件层应该分三层理解：

1. 数据与部件层：`parts_library`
2. Python 控制与驱动骨架：`real_robot_driver`、`hardware_controller`
3. 实验与参考层：`hive-reflex`、`imc22-rtl`、`firmware`、`ros2_ws`

## 结论

AGI-Walker 当前的硬件资产已经覆盖了部件数据库、驱动骨架和 IMC-22 相关参考链，但仓库还没有收敛成一份统一、稳定、量产级的硬件规格体系。写文档时应始终把“当前可用骨架”和“参考研究资产”区分开。
