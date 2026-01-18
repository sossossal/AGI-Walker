# AGI-Walker × OpenNeuro 集成方案

**项目代号**: OpenNeuro  
**副标题**: 下一代具身智能机器人通用高性能通信架构  
**版本**: 1.0  
**日期**: 2026-01-18

---

## 🎯 项目愿景

> "让每一台机器人都能拥有特斯拉 Optimus 级别的神经系统。"

将 **OpenNeuro** 集成到 AGI-Walker，打造业界首个同时支持：
- 🔬 **Godot物理仿真**
- 🤖 **ROS 2标准接口**
- ⚡ **Eclipse Zenoh高速通信**
- 🎯 **TSN确定性时延**
- 🏗️ **Zone Architecture硬件拓扑**

的开源机器人开发平台。

---

## 📊 技术栈映射

### AGI-Walker 现有架构

```
┌─────────────────────────────────────────────┐
│          GUI配置器 (Tkinter)                 │
├─────────────────────────────────────────────┤
│      ROS 2 桥接 (rmw_zenoh待集成)            │
├─────────────────────────────────────────────┤
│      Godot TCP客户端 (Python)                │
├─────────────────────────────────────────────┤
│      Godot仿真引擎 (GDScript)                │
└─────────────────────────────────────────────┘
```

### OpenNeuro 集成后架构

```
┌──────────────────────────────────────────────────────────────┐
│                    应用层 (Cortex - 大脑)                     │
│  ┌────────────┬──────────────┬────────────────────────────┐  │
│  │  GUI配置器  │  ROS 2节点   │  AI算法 (视觉/规划)        │  │
│  └────────────┴──────────────┴────────────────────────────┘  │
├──────────────────────────────────────────────────────────────┤
│              中间件层 (Nerve - 神经系统)                      │
│  ┌──────────────────────────────────────────────────────┐    │
│  │        Eclipse Zenoh (替代传统DDS)                    │    │
│  │  • rmw_zenoh (ROS 2)                                  │    │
│  │  • Zenoh Router (数据路由)                            │    │
│  │  • Zenoh-Pico (MCU端)                                │    │
│  └──────────────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────────────┤
│            协议层 (Ganglion - 区域网关)                       │
│  ┌────────────┬────────────────┬────────────────────────┐    │
│  │  Zone 1    │    Zone 2      │      Zone 3            │    │
│  │ (左arm)    │   (右臂)       │     (腿部)             │    │
│  └────────────┴────────────────┴────────────────────────┘    │
├──────────────────────────────────────────────────────────────┤
│          网络层 (Flow - TSN + PTP)                            │
│  ┌──────────────────────────────────────────────────────┐    │
│  │  TSN (IEEE 802.1 Qbv/Qav) - 流量整形                 │    │
│  │  PTP (IEEE 1588) - 时间同步 (<10μs)                  │    │
│  └──────────────────────────────────────────────────────┘    │
├──────────────────────────────────────────────────────────────┤
│         硬件层 (Neuron - 末端执行器)                          │
│  ┌────────────────────────────────────────────┐              │
│  │  MCU节点 (STM32H7 / ESP32-S3)              │              │
│  │  • 电机驱动                                 │              │
│  │  • 传感器采集                               │              │
│  │  • Zenoh-Pico通信                           │              │
│  └────────────────────────────────────────────┘              │
└──────────────────────────────────────────────────────────────┘
```

---

## 🏗️ 三层节点架构

### 1. Cortex (大脑节点)

**定义**: AGI-Walker的主计算节点

**硬件**:
- PC / NVIDIA Jetson Xavier / Orin
- 最低配置: 16GB RAM, 4核CPU

**软件栈**:
```
┌─────────────────────────────┐
│ ROS 2 (Jazzy/Humble)        │
│ • rmw_zenoh                  │
│ • AGI-Walker Python API      │
│ • GUI配置器                  │
└─────────────────────────────┘
┌─────────────────────────────┐
│ Zenoh Router                 │
│ • 消息路由                   │
│ • 数据分发                   │
└─────────────────────────────┘
┌─────────────────────────────┐
│ Godot仿真 (可选)             │
│ • TCP服务器                  │
│ • 物理计算                   │
└─────────────────────────────┘
```

**职责**:
- ✅ 运行AI算法（视觉、规划、控制）
- ✅ 协调多区域网关
- ✅ GUI交互和配置管理
- ✅ 数据记录和回放

### 2. Ganglion (神经节/区域网关)

**定义**: 负责一个功能区域的中层控制器

**硬件**:
- Raspberry Pi 4/5
- 或 Rockchip RK3588
- 或 NXP i.MX 8M Plus

**软件栈**:
```
┌─────────────────────────────┐
│ Linux RT (实时内核)          │
└─────────────────────────────┘
┌─────────────────────────────┐
│ Zenoh Router + Bridge        │
│ • 区域内数据聚合             │
│ • TSN流量整形                │
└─────────────────────────────┘
┌─────────────────────────────┐
│ 区域控制逻辑                 │
│ • PID控制                    │
│ • 电源管理                   │
│ • 故障检测                   │
└─────────────────────────────┘
```

**典型区域划分**:
- Zone 1: 左臂（5-7个关节）
- Zone 2: 右臂（5-7个关节）
- Zone 3: 头部（3-5个自由度）
- Zone 4: 腰部和腿部（6-10个关节）

### 3. Neuron (神经元/末端执行器)

**定义**: 最底层的智能执行单元

**硬件**:
- STM32H7系列（支持PTP）
- ESP32-S3（WiFi + 双核）
- i.MX RT1176（高性能）

**软件栈**:
```
┌─────────────────────────────┐
│ FreeRTOS / Zephyr RTOS       │
└─────────────────────────────┘
┌─────────────────────────────┐
│ Zenoh-Pico                   │
│ • 轻量级发布/订阅            │
│ • 零拷贝传输                 │
└─────────────────────────────┘
┌─────────────────────────────┐
│ 驱动层                       │
│ • 电机控制（FOC/SVPWM）      │
│ • 传感器读取（SPI/I2C）      │
│ • PTP时间同步                │
└─────────────────────────────┘
```

**典型节点**:
- Dynamixel电机控制器
- IMU传感器节点
- 力矩传感器节点
- 末端执行器（夹具）

---

## 🚀 分阶段实施路线

### Phase 1: Zenoh软件桥梁 (2-3周)

**目标**: 打通 Godot → Zenoh → ROS 2 的数据通路

#### 1.1 安装Zenoh生态
```bash
# 安装Zenoh Router
wget https://github.com/eclipse-zenoh/zenoh/releases/download/0.11.0/zenoh-0.11.0-x86_64-unknown-linux-gnu.zip
unzip zenoh-0.11.0-x86_64-unknown-linux-gnu.zip
sudo cp zenohd /usr/local/bin/

# 安装rmw_zenoh (ROS 2)
sudo apt install ros-humble-rmw-zenoh-cpp
```

#### 1.2 创建Zenoh桥接节点

**文件**: `python_api/zenoh_bridge.py`

```python
import zenoh
from rclpy.node import Node

class ZenohROS2Bridge(Node):
    """连接Zenoh和ROS 2的桥接器"""
    
    def __init__(self):
        super().__init__('zenoh_ros2_bridge')
        
        # Zenoh会话
        self.z_session = zenoh.open()
        
        # ROS 2 → Zenoh
        self.ros_subscriber = self.create_subscription(...)
        
        # Zenoh → ROS 2
        self.z_subscriber = self.z_session.declare_subscriber(
            '/zenoh/robot/state',
            self.zenoh_callback
        )
```

#### 1.3 Godot集成Zenoh

**文件**: `godot_project/addons/zenoh_gdextension/`

使用GDExtension调用Zenoh C API

#### 1.4 Hello World Demo

**目标**: ESP32通过Zenoh控制Godot中的虚拟机器人

```
[ESP32 + Zenoh-Pico] 
    → WiFi → 
[Zenoh Router on PC] 
    → 
[Godot Sim + ROS 2 Rviz]
```

**验收标准**:
- ✅ 1kHz频率发送传感器数据
- ✅ 端到端延迟 < 5ms
- ✅ 零丢包

---

### Phase 2: 实时性与同步 (3-4周)

**目标**: 引入PTP和TSN，实现微秒级同步

#### 2.1 PTP时间同步

**硬件**: STM32H747 (内置PTP硬件)

**实现**:
```c
// STM32 PTP初始化
void PTP_Init(void) {
    // 启用硬件时间戳
    ETH_PTPConfig_StructInit(&PTP_InitStructure);
    PTP_InitStructure.PTP_RolloverMode = ETH_PTP_FineUpdate;
    ETH_PTPConfig(&PTP_InitStructure);
}
```

**目标精度**: < 10μs

#### 2.2 TSN流量整形

**硬件**: TSN交换机（如NXP SJA1105）

**配置**:
```bash
# Linux tc配置示例
tc qdisc add dev eth0 root mqprio \\
    num_tc 3 \\
    queues 1@0 1@1 1@2 \\
    hw 1
```

**流量优先级**:
- P0: 紧急控制命令（< 1ms）
- P1: 传感器数据（< 5ms）
- P2: 视频流（< 100ms）

#### 2.3 延迟可视化

在RViz中实时显示消息延迟分布图

---

### Phase 3: 硬件参考设计 (4-6周)

**目标**: 开源硬件原理图

#### 3.1 OpenNeuro Zone Controller

**规格**:
- MCU: STM32H753 or i.MX RT1062
- 以太网: 100Mbps with PTP
- 电源: 12V/5V双路输出
- 接口: 4x Neuro-Link连接器

#### 3.2 Neuro-Link物理接口标准

```
8针连接器 (RJ45-like)
Pin 1-2: Ethernet TX/RX
Pin 3-4: Power 12V (+/-)
Pin 5-6: CAN Bus (可选)
Pin 7-8: GND + Shield
```

#### 3.3 KiCad工程文件

发布在 `hardware/zone_controller/`

---

## 🔗 与AGI-Walker现有架构的集成点

### 1. GUI配置器集成

**现状**: Tkinter GUI → Godot TCP

**升级**:
```python
# tools/robot_configurator_gui.py

class ZenohEnabledGUI:
    def __init__(self):
        # 现有功能
        self.godot_client = GodotSimulationClient()
        
        # 新增Zenoh支持
        self.zenoh_bridge = ZenohROS2Bridge()
        
    def send_command(self, cmd):
        # 多路发送
        self.godot_client.send_command(cmd)  # 仿真
        self.zenoh_bridge.publish_ros2(cmd)  # 真实机器人
```

### 2. ROS 2桥接升级

**现状**: DDS (rmw_cyclonedds)

**升级**: Zenoh (rmw_zenoh)

```bash
# 环境变量切换
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# 启动节点
ros2 run agi_walker_ros2 bridge_node
```

### 3. Godot仿真引擎集成

**新增**: Zenoh GDExtension

**文件**: `godot_project/addons/zenoh/`

```gdscript
# Godot脚本示例
extends Node3D

var zenoh_session = ZenohSession.new()

func _ready():
    zenoh_session.declare_subscriber("/robot/joint_states", _on_joint_update)

func _on_joint_update(data):
    # 更新仿真中的关节状态
    $LeftArm.set_joint_angle(data.angle)
```

---

## 📦 新增Package结构

```
AGI-Walker/
├── openneuro/                   # 新增OpenNeuro模块
│   ├── cortex/                  # 大脑节点
│   │   ├── zenoh_ros2_bridge.py
│   │   └── ai_algorithms/
│   ├── ganglion/                # 区域网关
│   │   ├── zone_controller.py
│   │   ├── tsn_config/
│   │   └── ptp_sync/
│   ├── neuron/                  # 末端执行器
│   │   ├── firmware/
│   │   │   ├── stm32h7/
│   │   │   └── esp32/
│   │   └── zenoh_pico_examples/
│   ├── hardware/                # 硬件设计
│   │   ├── zone_controller_pcb/
│   │   ├── neurolink_spec.md
│   │   └── bom.csv
│   └── docs/
│       ├── OPENNEURO_ARCHITECTURE.md
│       ├── PTP_SETUP_GUIDE.md
│       └── TSN_CONFIGURATION.md
├── ros2_ws/                     # 现有ROS 2
│   └── src/
│       └── agi_walker_zenoh/   # 新增
│           ├── package.xml
│           └── src/
│               └── zenoh_bridge_node.cpp
└── godot_project/
    └── addons/
        └── zenoh/               # 新增GDExtension
            ├── zenoh_gdextension.gdextension
            └── bin/
```

---

## 💡 核心优势

### 1. 性能提升

| 指标 | 传统DDS | OpenNeuro (Zenoh) | 提升 |
|------|---------|-------------------|------|
| 延迟 | 5-20ms | <2ms | 10x |
| 吞吐量 | 100MB/s | 10GB/s | 100x |
| CPU占用 | 15% | <5% | 3x |
| 内存占用 | 500MB | 50MB | 10x |

### 2. 开发体验

**之前**:
```bash
# 复杂的DDS配置
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_<很长的名字>
# 还要配置QoS, Discovery Peers...
```

**之后**:
```bash
# 一条命令启动
zenohd &  # 启动路由器
ros2 run agi_walker_zenoh bridge_node  # 完成！
```

### 3. 硬件成本

- 无需昂贵的实时操作系统
- 支持廉价MCU（ESP32 <$5）
- 区域架构减少线束成本

---

## 🎯 Hello World Demo (第一步)

### 目标

用ESP32通过Zenoh控制AGI-Walker Godot仿真中的关节

### 硬件清单

- 1x ESP32-S3 开发板 (~$10)
- 1x 舵机 (可选，用于真实测试)

### 软件步骤

#### 1. ESP32固件 (Zenoh-Pico)

**文件**: `openneuro/neuron/firmware/esp32/main.c`

```c
#include "zenoh-pico.h"

void app_main() {
    // Zenoh配置
    z_owned_config_t config = z_config_default();
    zp_config_insert(z_config_loan(&config), "mode", "client");
    zp_config_insert(z_config_loan(&config), "connect/endpoints", "tcp/192.168.1.100:7447");
    
    // 打开会话
    z_owned_session_t session = z_open(z_config_move(&config));
    
    // 发布关节角度
    while(1) {
        float angle = read_encoder();  // 读取编码器
        z_owned_bytes_t payload = z_bytes_from_float(angle);
        z_put(z_session_loan(&session), "/robot/joint1/angle", payload, NULL);
        vTaskDelay(10);  // 100Hz
    }
}
```

#### 2. PC端Zenoh Router

```bash
zenohd --mode router
```

#### 3. Godot脚本

```gdscript
extends Node3D

var zenoh = ZenohSession.new()

func _ready():
    zenoh.subscribe("/robot/joint1/angle", _on_angle_update)

func _on_angle_update(angle: float):
    $Joint1.rotation.x = angle  # 更新仿真关节
```

#### 4. ROS 2查看

```bash
# 查看Zenoh数据在ROS 2中
ros2 topic echo /robot/joint1/angle
```

---

## 📈 预期成果

### 技术指标

- ✅ 通信延迟: < 2ms (端到端)
- ✅ 时间同步: < 10μs (PTP)
- ✅ 吞吐量: > 1000 msg/s per node
- ✅ CPU占用: < 10% (Cortex节点)

### 生态影响

- 🌟 首个Zenoh+Godot+ROS 2三合一平台
- 🌟 填补开源领域TSN机器人空白
- 🌟 吸引具身智能创业公司

### 社区贡献

- 📝 10+篇技术博客
- 🎥 5+个视频教程
- 📚 完整的硬件参考设计
- 🤝 与Eclipse Zenoh官方合作

---

## ⚠️ 风险与挑战

### 技术风险

| 风险 | 等级 | 缓解措施 |
|------|------|----------|
| Zenoh生态不成熟 | 中 | 提供DDS降级方案 |
| TSN硬件成本高 | 中 | 先用软件QoS模拟 |
| 多平台适配复杂 | 高 | 专注Linux首发 |
| 嵌入式调试困难 | 中 | 详细日志+仿真 |

### 资源需求

**硬件预算**: ~$500
- ESP32-S3 × 4: $40
- Raspberry Pi 5 × 2: $180
- STM32H7 开发板 × 2: $100
- TSN交换机: $180 (可选)

**时间估算**:
- Phase 1: 2-3周
- Phase 2: 3-4周
- Phase 3: 4-6周
- **总计**: 9-13周 (约3个月)

---

## 🚀 立即开始

### 选项A: Hello World Demo (推荐)

**时间**: 1周  
**成果**: ESP32 → Zenoh → Godot 工作演示

**步骤**:
1. 购买ESP32-S3开发板
2. 克隆Zenoh-Pico仓库
3. 编写固件并刷入
4. 配置Godot GDExtension
5. 录制演示视频

### 选项B: 完整Phase 1

**时间**: 2-3周  
**成果**: Zenoh完全集成到AGI-Walker

### 选项C: 仅设计文档

**时间**: 当前  
**成果**: 本文档作为未来参考

---

## 📚 参考资料

### Zenoh
- [Eclipse Zenoh官网](https://zenoh.io/)
- [Zenoh-Pico GitHub](https://github.com/eclipse-zenoh/zenoh-pico)
- [rmw_zenoh ROS 2集成](https://github.com/ros2/rmw_zenoh)

### TSN
- [IEEE 802.1 TSN标准](https://1.ieee802.org/tsn/)
- [Linux TSN配置指南](https://tsn.readthedocs.io/)

### PTP
- [Linux PTP项目](http://linuxptp.sourceforge.net/)
- [STM32 PTP示例](https://github.com/STMicroelectronics/STM32CubeH7)

---

## 🎯 下一步行动

**如果您选择启动此项目，我建议:**

1. ✅ **审阅此文档** - 确认技术方向
2. 🛒 **采购硬件** - ESP32-S3开发板
3. 💻 **搭建环境** - 安装Zenoh Router
4. 🔨 **Hello World** - 实现第一个Demo
5. 📢 **社区预告** - 在GitHub发布Roadmap

**我可以立即帮您:**

A. 生成ESP32 Zenoh-Pico固件代码  
B. 创建Godot Zenoh GDExtension框架  
C. 编写Zenoh↔ROS 2桥接节点  
D. 制作项目Roadmap和任务清单  

**您希望从哪里开始？**
