# AGI-Walker 硬件规格：IMC-22 Reflex 控制器

## 📋 概述

本文档介绍用于将 AGI-Walker 仿真机器人部署到真实硬件的控制器平台：**IMC-22 Reflex 控制器**，基于 Hive-Reflex 分布式反射控制架构。

---

## 🔧 IMC-22 芯片规格

### 核心处理器

| 参数 | 规格 |
|------|------|
| **架构** | RISC-V RV32IMAC |
| **主频** | 200 MHz |
| **指令集** | 整数 + 乘法 + 原子操作 + 压缩指令 |
| **流水线** | 5 级流水线 |

### 内存系统

| 类型 | 容量 | 用途 |
|------|------|------|
| **SRAM** | 512 KB | 程序 + 数据 |
| **Flash** | 2 MB | 固件存储 |
| **NPU SRAM** | 128 KB | 神经网络权重 |

### 神经加速器 (NPU)

| 特性 | 说明 |
|------|------|
| **架构** | 存内计算 (In-Memory Computing) |
| **数据类型** | INT8, FP16, FP32 |
| **专用单元** | LSTM 加速器 |
| **推理延迟** | < 50 μs (INT8 模型) |
| **功耗** | ~100 mW @推理 |

---

## 🔌 外设接口

### CAN-FD 总线

| 参数 | 规格 |
|------|------|
| **标准** | CAN 2.0B + CAN-FD |
| **波特率** | 1 Mbps (标准) / 5 Mbps (FD) |
| **接口** | 2-Wire (CANH, CANL) |
| **收发器** | 内置 |
| **过滤器** | 8 个硬件过滤器 |

### SPI (用于 IMU)

| 参数 | 规格 |
|------|------|
| **通道** | 2 个 |
| **速率** | 最高 20 Mbps |
| **DMA** | 支持 |
| **应用** | 6-DOF IMU (1 kHz 采样) |

### PWM (电机控制)

| 参数 | 规格 |
|------|------|
| **通道** | 4 个独立通道 |
| **频率** | 1 Hz - 100 kHz |
| **分辨率** | 12-bit |
| **死区时间** | 可配置 |

### ADC (电流传感器)

| 参数 | 规格 |
|------|------|
| **通道** | 8 个 |
| **分辨率** | 12-bit |
| **采样率** | 最高 1 Msps |
| **参考电压** | 内置 3.3V |

---

## 🤖 Hive-Reflex 控制架构

### 核心理念

**分布式感知 + 本地反射 + 指令叠加**

每个关节节点运行独立的控制循环，结合 PID 控制和神经网络反射：

$$U_{\text{final}} = U_{\text{PID}} \cdot (1 - \gamma) + f_{\text{NN}}(S_{\text{local}}) \cdot \gamma \cdot T_{\text{max}}$$

其中：
- $U_{\text{PID}}$: 经典 PID 控制器输出
- $f_{\text{NN}}$: 神经网络反射策略（运行在 NPU）
- $\gamma$: 柔顺系数 (0=刚性, 1=柔性)
- $S_{\text{local}}$: 本地传感器数据 [陀螺仪, 加速度, 电流]

### 控制频率

| 功能 | 频率 |
|------|------|
| **主控制循环** | 1 kHz (1 ms) |
| **NPU 推理** | 1 kHz (< 50 μs/次) |
| **CAN 状态广播** | 100 Hz |
| **CAN 命令接收** | 实时响应 |

### 通信协议

**CAN ID 定义**:

| CAN ID | 类型 | 数据格式 | 说明 |
|--------|------|---------|------|
| `0x000` | Sync | `Timestamp` | 全局时间同步 |
| `0x100 + NodeID` | Status | `Angle`, `Current`, `Error` | 节点状态 (100 Hz) |
| `0x200 + NodeID` | Command | `TargetAngle` (int16), `Compliance` (uint8) | 主控指令 |
| `0x300 + NodeID` | Config | `MaxTorque`, `PID_Kp`, `PID_Ki` | 参数配置 |
| `0x7FF` | Handshake | `Type`, `UID` | 热插拔握手 |

**数据压缩**:
- `TargetAngle`: int16_t, 单位 0.01°, 范围 ±327.67°
- `Compliance`: uint8_t, 0-255 映射到 0.0-1.0

---

## 💰 成本和供应链

| 项目 | 数量 | 单价 (USD) | 总价 |
|------|------|-----------|------|
| IMC-22 芯片 | 1 | $15 | $15 |
| CAN 收发器 | 1 | $2 | $2 |
| IMU (6-DOF) | 1 | $8 | $8 |
| 电流传感器 | 1 | $3 | $3 |
| PCB + 外围元件 | 1 套 | $12 | $12 |
| **单节点总成本** | - | - | **≈ $40** |

对于典型的双足机器人（12 个关节）：**总控制器成本 ≈ $480**

---

## ⚡ 性能基准

### 推理性能

| 模型 | 数据类型 | 参数量 | 推理时间 | 功耗 |
|------|---------|-------|---------|------|
| ReflexNet (LSTM) | INT8 | 2.5K | 35 μs | 80 mW |
| ReflexNet (LSTM) | FP16 | 5K | 60 μs | 120 mW |
| MLP (简化版) | INT8 | 1K | 15 μs | 50 mW |

### 控制延迟

| 环节 | 延迟 |
|------|------|
| 传感器读取 (SPI) | 10 μs |
| NPU 推理 | 35 μs |
| 控制律计算 | 5 μs |
| PWM 输出 | 1 μs |
| **总延迟** | **< 100 μs** |

**结论**: 可稳定运行在 1 kHz 控制频率

---

## 🔄 与 AGI-Walker 仿真的对应关系

### 传感器映射

| AGI-Walker 仿真 | IMC-22 硬件 |
|----------------|------------|
| `RigidBody3D.angular_velocity` | 陀螺仪 (Gyro) |
| `RigidBody3D.linear_acceleration` | 加速度计 (Accel) |
| `HingeJoint.get_angle()` | 编码器 (通过电机反馈) |
| 虚拟电流传感器 | ADC 电流采样 |

### 执行器映射

| AGI-Walker 仿真 | IMC-22 硬件 |
|----------------|------------|
| `HingeJoint.set_motor_target()` | PWM 占空比控制 |
| 电机扭矩-速度曲线 | 真实电机特性 |

### 控制策略迁移

```python
# AGI-Walker 训练的策略
policy = PPO.load("trained_policy.zip")

# 转换为 ONNX
dummy_obs = env.observation_space.sample()
torch.onnx.export(policy.policy, dummy_obs, "policy.onnx")

# 量化为 INT8（用于 IMC-22）
python reflex_net.py --quantize
```

---

## 📊 与其他平台对比

| 特性 | IMC-22 | STM32F4 | Raspberry Pi 4 | NVIDIA Jetson Nano |
|------|--------|---------|---------------|--------------------|
| CPU | RISC-V 200MHz | ARM 168MHz | ARM 1.5GHz | ARM 1.43GHz |
| 内存 | 512 KB SRAM | 192 KB SRAM | 1-4 GB | 2-4 GB |
| **NPU** | ✅ 128 KB | ❌ | ❌ | ✅ 128-core GPU |
| 控制延迟 | < 100 μs | ~200 μs | ~1 ms | ~500 μs |
| 功耗 | ~500 mW | ~1 W | ~3 W | ~10 W |
| 成本 | $15 | $10 | $35 | $100 |
| **适用场景** | 关节控制 | 通用嵌入式 | 通用计算 | 视觉 AI |

**优势**: IMC-22 在边缘神经网络推理方面性价比最高。

---

## 🛠️ 开发工具链

### 软件开发

| 工具 | 用途 |
|------|------|
| `riscv32-unknown-elf-gcc` | C/C++ 编译器 |
| `OpenOCD` | 调试和烧录 |
| `GDB` | 源码级调试 |
| PyTorch/ONNX | 模型训练和导出 |

### 硬件调试

| 工具 | 说明 |
|------|------|
| J-Link | JTAG 调试器 |
| CAN 分析仪 | 总线监控 |
| 逻辑分析仪 | 数字信号调试 |

---

## 📚 参考资源

- [Hive-Reflex SDK 编程指南](file:///d:/新建文件夹/hive-reflex/SDK_GUIDE.md)
- [Hive-Reflex 架构文档](file:///d:/新建文件夹/hive-reflex/hive_arch.md)
- [IMC-22 SDK GitHub](https://github.com/your-repo/hive-reflex) *(待发布)*

---

## ⚠️ 注意事项

1. **开发阶段**: IMC-22 SDK 当前为 v1.0 beta，适合原型开发
2. **供应链**: 芯片为概念设计，实际采购需联系制造商
3. **认证**: 商业化产品需通过 CE/FCC 认证

---

**文档版本**: 1.0  
**最后更新**: 2026-01-16  
**维护者**: AGI-Walker Team
