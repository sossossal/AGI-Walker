# OpenNeuro MVP 说明文档

**模块化部件架构最小可行性验证 (MVP)**

## 📂 目录结构

```
mvp/
├── simulation/
│   └── virtual_ganglion.py    # 虚拟Zone Controller (PC端运行)
├── firmware/
│   └── stm32/
│       └── main.c             # STM32固件参考代码
└── README.md
```

## 🚀 快速开始 (纯软件仿真)

在还没收到硬件之前，您可以直接在PC上运行虚拟仿真，验证通信架构。

### 1. 启动 Zenoh Router

确保您已经安装了zenohd。

```bash
zenohd
```

### 2. 启动虚拟 Zone Controller

这个脚本模拟了 "Zone 1 (左臂)" 的控制器行为：
- 连接到Zenoh
- 发布 `/zone/1/state` (100Hz, 模拟电机数据)
- 发布 `/zone/1/imc22/output` (模拟AI推理结果)
- 订阅 `/zone/1/cmd` (接收控制指令)

```bash
python openneuro/mvp/simulation/virtual_ganglion.py
```

### 3. 验证数据流

打开一个新的终端，使用Zenoh客户端工具监听数据：

```bash
# 监听状态
python -m zenoh_cli.sub "/zone/1/state"

# 发送控制指令
python -m zenoh_cli.pub "/zone/1/cmd" '{"targets": [1.57, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

## 🔧 硬件开发指南

`firmware/stm32/main.c` 提供了基于STM32H7 + FreeRTOS + LwIP + Zenoh-Pico 的参考实现。

**关键特性**:
- **3任务架构**:
  - `MotorControlTask`: 1kHz 实时电机控制
  - `CommTask`: 100Hz 通信循环 (Zenoh)
  - `AIInferenceTask`: IMC-22 推理接口
